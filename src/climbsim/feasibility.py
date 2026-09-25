"""Runs a full route attempt move-by-move and decides, for each move, whether
it's physically feasible -- the core of the project.

Per move, in order:
  1. Kinematic reachability: is the target hold within the moving limb's
     geometric reach at all (hard bound), and does numerical IK (holding
     torso + the other three limbs at their current, already-settled
     configuration) actually find a joint solution that reaches it?
  2. Forward-dynamics simulation of the transition: the target limb detaches
     (its grip constraint switches off) and is driven toward the IK
     solution by its PD position actuators while the other three limbs stay
     passively anchored; gravity acts throughout. Every step, each still-
     anchored limb's real constraint force (see forces.py) is checked
     against its hold's friction cone and max-grip rating.
  3. Settle: once the moving limb's new grip constraint activates, run a
     short settle window and re-check all four grips -- this is the static
     (steady-state) equilibrium check, done via genuine forward dynamics
     rather than a separate analytic solve.

Any friction-cone or max-grip violation, at any point, fails the move and
(for the route as a whole) triggers an unconstrained fall for however long
the caller wants to keep rendering.
"""

from dataclasses import dataclass, field
from typing import List, Optional

import mujoco
import numpy as np

from climbsim import anthropometry as A
from climbsim.forces import check_grip
from climbsim.kinematics import solve_ik_best, solve_pose_ik
from climbsim.model import ClimberModel, LIMB_JOINTS, arm_reach, leg_reach
from climbsim.route import LIMBS, Move, Route

DT = 0.002
INITIAL_SETTLE_S = 3.0  # let the climber fall from the (somewhat arbitrary) starting
# pose guess into its true, physically-consistent resting stance before any move is
# attempted. This transient is genuinely large -- during development, the torso dropped
# ~1 m and briefly exceeded 10 rad/s of joint velocity before settling -- because whole-
# body IK (solve_pose_ik) only guarantees the four limb sites reach their holds, not that
# the resulting pose is close to the low-energy natural hang. Starting a move mid-transient
# (the original 0.4s here) fed a moving limb's already-swinging anchor state into the
# per-step grip check and produced spurious multi-kN "slip" failures on a route that is
# in fact easily climbable.
TRANSITION_S = 3.0
SETTLE_S = 3.0
REACH_TOL = 0.02


def _max_reach(limb: str) -> float:
    return arm_reach() if "hand" in limb else leg_reach()


@dataclass
class MoveOutcome:
    move: Move
    passed: bool
    phase: str  # "reachability" | "dynamic" | "settle"
    reason: str
    detail: dict = field(default_factory=dict)


@dataclass
class AttemptResult:
    route: Route
    outcomes: List[MoveOutcome] = field(default_factory=list)
    frames: List[np.ndarray] = field(default_factory=list)
    fell: bool = False

    @property
    def success(self) -> bool:
        return len(self.outcomes) > 0 and all(o.passed for o in self.outcomes) and not self.fell

    @property
    def first_failure(self) -> Optional[MoveOutcome]:
        for o in self.outcomes:
            if not o.passed:
                return o
        return None


def _check_all_anchored(cm: ClimberModel, current_holds: dict, exclude=()) -> Optional[MoveOutcome]:
    for limb, hold_name in current_holds.items():
        if limb in exclude or hold_name is None:
            continue
        hold = cm.route.hold(hold_name)
        gc = check_grip(limb, hold, cm.model, cm.data, cm.limb_eq_id[limb])
        if not gc.ok:
            reason = ("max grip force exceeded" if not gc.within_max_grip else "hold slipped (friction cone exceeded)")
            return MoveOutcome(
                move=None, passed=False, phase="dynamic",
                reason=f"{reason} on {limb} at hold '{hold_name}'",
                detail=dict(limb=limb, hold=hold_name, force_n=float(np.linalg.norm(gc.force)),
                            normal_n=gc.normal_n, tangential_n=gc.tangential_n),
            )
    return None


def _set_ctrl_from_qpos(cm: ClimberModel, limb: str) -> None:
    for j in LIMB_JOINTS[limb]:
        qadr = cm.model.joint(j).qposadr[0]
        aid = cm.model.actuator(f"act_{j}").id
        cm.data.ctrl[aid] = cm.data.qpos[qadr]


def _capture(cm: ClimberModel, renderer_factory) -> Optional[np.ndarray]:
    if renderer_factory is None:
        return None
    r = renderer_factory.bind(cm.model)
    r.update_scene(cm.data, camera="climb_cam")
    return r.render()


def attempt_route(route: Route, renderer=None, frame_every: int = 15) -> AttemptResult:
    """`renderer`, if given, is a `render.make_renderer()` factory (not a bound
    mujoco.Renderer -- this function builds its own ClimberModel internally,
    so the renderer is bound to that model lazily, on first frame capture)."""
    cm = ClimberModel.build(route)
    cm.init_qpos_standing()

    current_holds = dict(route.start)
    start_targets = {limb: route.hold(hn).pos for limb, hn in current_holds.items()}
    solve_pose_ik(cm, start_targets)
    for limb, hn in current_holds.items():
        cm.set_limb_anchor(limb, route.hold(hn).pos)
        cm.set_grip_active(limb, True)
        cm.set_limb_gain(limb, A.JOINT_KP, A.JOINT_KV)
    mujoco.mj_forward(cm.model, cm.data)
    for limb in LIMBS:
        _set_ctrl_from_qpos(cm, limb)

    result = AttemptResult(route=route)

    def run_steps(n, capture=True):
        for i in range(n):
            mujoco.mj_step(cm.model, cm.data)
            if capture and renderer is not None and i % frame_every == 0:
                result.frames.append(_capture(cm, renderer))

    run_steps(int(INITIAL_SETTLE_S / DT))  # let the fall-into-stance transient die out
    bad = _check_all_anchored(cm, current_holds)
    if bad is not None:
        bad.reason = "starting stance is not stable: " + bad.reason
        result.outcomes.append(bad)
        result.fell = True
        run_steps(int(1.0 / DT))
        return result

    for move in route.moves:
        limb = move.limb
        target_hold = route.hold(move.hold)
        target_pos = np.array(target_hold.pos)

        dist_from_anchor = float(np.linalg.norm(target_pos - cm.limb_site_pos(limb)))
        max_r = _max_reach(limb)
        # must be captured before solve_ik_best, which leaves cm.data.qpos set to its
        # solution -- capturing it after made start_q == target_q, silently turning the
        # "interpolate from current pose to target" transition into a no-op (constant
        # ctrl at the target pose) with the limb then just sagging under gravity for the
        # full transition, unconstrained, since its grip was also released. That produced
        # a huge, spurious end-of-transition gap (~0.3-0.8 m) on every move, feasible or
        # not -- masked on feasible moves only because the subsequent grip-reconnect snap
        # (see model.set_limb_gain / _hold_geom_xml history) forcibly relocated the limb.
        pre_move_q = np.array([cm.data.qpos[a] for a in cm.limb_joint_qpos_adr[limb]])

        ik = solve_ik_best(cm, limb, target_pos, tol=REACH_TOL)
        if not ik.reached:
            deficit = ik.residual_m
            outcome = MoveOutcome(
                move=move, passed=False, phase="reachability",
                reason=(f"'{move.hold}' is not reachable by {limb}: "
                        f"{deficit * 100:.1f} cm short after IK convergence "
                        f"(currently {dist_from_anchor:.2f} m from the target hold, "
                        f"~{max_r:.2f} m is the limb's max extension from its shoulder/hip)"),
                detail=dict(residual_m=deficit, distance_m=dist_from_anchor, max_reach_m=max_r),
            )
            result.outcomes.append(outcome)
            break

        target_q = ik.q

        cm.set_grip_active(limb, False)
        cm.set_limb_gain(limb, A.JOINT_KP, A.JOINT_KV)
        start_q = pre_move_q
        # re-seed qpos back to the pre-move pose; IK is only used to compute the target
        for a, v in zip(cm.limb_joint_qpos_adr[limb], start_q):
            cm.data.qpos[a] = v

        n_steps = int(TRANSITION_S / DT)
        # Closed-loop re-aim, not a single open-loop plan: joint-space interpolation
        # computed once against the pre-move torso pose assumes that pose holds for the
        # whole transition. It doesn't -- dropping to 3-point support makes the torso
        # genuinely sag as it finds its new equilibrium (~0.3-0.5 m over ~1s during
        # development, then plateauing) -- so a plan aimed at the old pose drifts off the
        # real-world target by a similar margin. Every RESOLVE_EVERY steps we re-run a
        # fast local IK from the limb's actual current joint angles, against the same
        # fixed world-space hold position, which re-aims for whatever the torso is
        # actually doing right now (closer to how a climber continuously corrects a reach
        # using proprioception than a single ballistic plan).
        RESOLVE_EVERY = 50  # 0.1s at DT=0.002
        seg_start_q, seg_target_q, seg_start_step = start_q, target_q, 0
        failure = None
        for step in range(n_steps):
            if step > 0 and step % RESOLVE_EVERY == 0:
                cur_q = np.array([cm.data.qpos[a] for a in cm.limb_joint_qpos_adr[limb]])
                refined = solve_ik_best(cm, limb, target_pos, tol=REACH_TOL)
                # A re-solve that didn't converge is worse than no re-solve: an early
                # version used a cheap 2-seed search here and, once, it landed on an
                # unconverged *different* local minimum (shoulder rotation jumping ~34
                # degrees from the actual current pose) rather than refining toward the
                # real one. Chasing that bad target commanded a brief ~7 rad/s joint
                # velocity, whose reaction was what actually tripped a foothold's grip
                # check -- not the move itself. Using the full multi-seed search here and
                # discarding a non-converged result (keeping the previous target instead
                # of a wrong one) fixed it.
                if refined.reached:
                    # Rate-limited: if the torso has drifted since the last re-aim, the
                    # fresh solution can still be meaningfully far from cur_q, and closing
                    # that gap within one more RESOLVE_EVERY window would demand a sudden
                    # burst of joint velocity. Capping the per-axis step keeps every
                    # segment gentle; a target still far after one segment gets chased
                    # further next segment.
                    MAX_STEP = np.deg2rad(20.0)
                    step_vec = np.clip(refined.q - cur_q, -MAX_STEP, MAX_STEP)
                    seg_start_q, seg_target_q, seg_start_step = cur_q, cur_q + step_vec, step
                else:
                    for a, v in zip(cm.limb_joint_qpos_adr[limb], cur_q):
                        cm.data.qpos[a] = v  # undo solve_ik_best leaving qpos at its (bad) result
                    seg_start_q, seg_start_step = cur_q, step
                # mj_step recomputes the forward pass from qpos/qvel every call, so we
                # don't need to otherwise restore data here, but qvel is untouched and
                # still reflects real dynamics -- that's exactly what we want to keep.
            frac = min(1.0, (step - seg_start_step + 1) / RESOLVE_EVERY)
            # quintic ease (zero velocity *and* zero acceleration at both endpoints): a
            # cubic smoothstep here left a small acceleration discontinuity right at the
            # end of the move, which showed up as a brief (~60ms) reactive force spike on
            # the other anchored limbs as the moving limb "caught" its target -- enough to
            # trip a foothold's max-grip check on an otherwise clearly-fine move.
            smooth = frac ** 3 * (10 - 15 * frac + 6 * frac ** 2)
            q_cmd = seg_start_q + smooth * (seg_target_q - seg_start_q)
            for j, v in zip(LIMB_JOINTS[limb], q_cmd):
                aid = cm.model.actuator(f"act_{j}").id
                cm.data.ctrl[aid] = v
            mujoco.mj_step(cm.model, cm.data)
            if renderer is not None and step % frame_every == 0:
                result.frames.append(_capture(cm, renderer))
            failure = _check_all_anchored(cm, current_holds, exclude=(limb,))
            if failure is not None:
                break

        if failure is not None:
            failure.move = move
            failure.reason = f"during move to '{move.hold}': {failure.reason}"
            result.outcomes.append(failure)
            result.fell = True
            for held_limb in list(current_holds):
                if held_limb != limb:
                    hn = current_holds[held_limb]
                    hold = route.hold(hn)
                    gc = check_grip(held_limb, hold, cm.model, cm.data, cm.limb_eq_id[held_limb])
                    if not gc.ok:
                        cm.set_grip_active(held_limb, False)
            run_steps(int(1.2 / DT))
            break

        cm.set_limb_anchor(limb, target_pos)
        cm.set_grip_active(limb, True)
        cm.set_limb_gain(limb, A.JOINT_KP, A.JOINT_KV)
        _set_ctrl_from_qpos(cm, limb)
        current_holds[limb] = move.hold

        # Checked only at the end of the settle window, not every step: re-engaging a
        # grip constraint the instant the moving limb arrives is an idealization (a real
        # hand doesn't achieve full, precise grip force the instant it touches a hold
        # either), and produces a brief instantaneous-snap transient that isn't a genuine
        # failure -- what matters is whether the climber can hold the resulting position
        # once things settle.
        for step in range(int(SETTLE_S / DT)):
            mujoco.mj_step(cm.model, cm.data)
            if renderer is not None and step % frame_every == 0:
                result.frames.append(_capture(cm, renderer))
        settle_failure = _check_all_anchored(cm, current_holds)

        if settle_failure is not None:
            settle_failure.move = move
            settle_failure.phase = "settle"
            settle_failure.reason = f"move to '{move.hold}' reached but did not hold: {settle_failure.reason}"
            result.outcomes.append(settle_failure)
            result.fell = True
            run_steps(int(1.2 / DT))
            break

        result.outcomes.append(MoveOutcome(
            move=move, passed=True, phase="settle",
            reason=f"{limb} reached and held '{move.hold}'",
            detail=dict(ik_residual_m=ik.residual_m),
        ))

    return result
