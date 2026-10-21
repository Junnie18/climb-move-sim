"""Forward-dynamics route attempts with explicit contact and failure checks.

The controller is a bounded joint-position tracker, not a proof of human
climbability. Geometric bounds, numerical planning failures, tracking failures,
and contact overloads are reported separately. No IK result is applied to live
positions during a move, and no grip is attached before the limb arrives.
"""

from dataclasses import dataclass, field
from math import sqrt
from typing import List, Optional

import mujoco
import numpy as np

from climbsim import anthropometry as A
from climbsim.forces import check_grip, FRICTION_GATED_TYPES
from climbsim.kinematics import solve_ik_best, solve_pose_ik
from climbsim.model import ClimberModel, LIMB_JOINTS, arm_reach, leg_reach
from climbsim.route import LIMBS, Move, Route

DT = 0.002  # compatibility default; simulation loops use model.opt.timestep
INITIAL_SETTLE_S = 3.0
TRANSITION_S = 3.0
SETTLE_S = 3.0
REACH_TOL = 0.002
INITIAL_POSE_TOL = 0.0001
CONTACT_DISTANCE_M = 0.012
CONTACT_SPEED_M_S = 0.12
# A shoe or ungraspable surface must finish decelerating before touchdown.
# Attaching these unilateral contacts at hand-grasp speeds creates a braking
# impulse that can exceed the friction cone even when the final stance is
# supported. This is a stricter controller acquisition criterion, not a change
# to the physical slip/force limits checked on every attached sample.
UNILATERAL_CONTACT_SPEED_M_S = 0.02
REPLAN_S = 0.1
COMMAND_SPEED_RAD_S = 1.2
COMMAND_ACCEL_RAD_S2 = 4.0


def _max_reach(limb: str) -> float:
    return arm_reach() if "hand" in limb else leg_reach()


@dataclass
class MoveOutcome:
    move: Optional[Move]
    passed: bool
    phase: str
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
        completed_moves = sum(o.move is not None for o in self.outcomes)
        return completed_moves == len(self.route.moves) and bool(self.outcomes) and all(
            o.passed for o in self.outcomes) and not self.fell

    @property
    def first_failure(self) -> Optional[MoveOutcome]:
        return next((o for o in self.outcomes if not o.passed), None)


def _check_all_anchored(cm: ClimberModel, current_holds: dict, exclude=()) -> Optional[MoveOutcome]:
    for limb, hold_name in current_holds.items():
        if limb in exclude or hold_name is None or not cm.data.eq_active[cm.limb_eq_id[limb]]:
            continue
        hold = cm.route.hold(hold_name)
        gc = check_grip(limb, hold, cm.model, cm.data, cm.limb_eq_id[limb])
        if not gc.ok:
            category = "grip_strength_exceeded" if not gc.within_max_grip else "contact_cone_exceeded"
            reason = "max grip force exceeded" if not gc.within_max_grip else "hold slipped (contact cone exceeded)"
            return MoveOutcome(
                move=None, passed=False, phase="dynamic",
                reason=f"{reason} on {limb} at hold '{hold_name}'",
                detail=dict(category=category, limb=limb, hold=hold_name,
                            force_n=float(np.linalg.norm(gc.force)), normal_n=gc.normal_n,
                            tangential_n=gc.tangential_n, time_s=float(cm.data.time)),
            )
    return None


def _set_ctrl_from_qpos(cm: ClimberModel, limb: str) -> None:
    for j in LIMB_JOINTS[limb]:
        cm.data.ctrl[cm.model.actuator(f"act_{j}").id] = cm.data.qpos[cm.model.joint(j).qposadr[0]]


def _capture(cm: ClimberModel, renderer_factory) -> Optional[np.ndarray]:
    if renderer_factory is None:
        return None
    r = renderer_factory.bind(cm.model)
    r.update_scene(cm.data, camera="climb_cam")
    return r.render()


def _contact_state(cm: ClimberModel, limb: str, target_pos) -> tuple[float, float]:
    """Distance and world-relative site speed against a stationary hold."""
    mujoco.mj_kinematics(cm.model, cm.data)
    mujoco.mj_comPos(cm.model, cm.data)
    jac = np.zeros((3, cm.model.nv))
    mujoco.mj_jacSite(cm.model, cm.data, jac, None, cm.limb_site_id[limb])
    return (float(np.linalg.norm(cm.limb_site_pos(limb) - target_pos)),
            float(np.linalg.norm(jac @ cm.data.qvel)))


def _limb_base_pos(cm: ClimberModel, limb: str) -> np.ndarray:
    side = "L" if limb.startswith("left") else "R"
    body = ("upper_arm_" if "hand" in limb else "thigh_") + side
    return cm.data.xpos[cm.model.body(body).id].copy()



def _support_torques(cm: ClimberModel, current_holds: dict) -> np.ndarray:
    """Inverse-statics feedforward through the existing capped actuators.

    Alternate projection approximately allocates forces balancing the floating base's
    gravity wrench within conservative cones/caps. These are controller targets;
    only MuJoCo's actual forces determine acceptance. No external body force is
    applied, and no contact constraint is added by this controller.
    """
    contacts = [(limb, cm.route.hold(name)) for limb, name in current_holds.items()
                if name is not None and cm.data.eq_active[cm.limb_eq_id[limb]]]
    if not contacts:
        return cm.data.qfrc_bias.copy()
    jacobians = []
    for limb, _ in contacts:
        jac = np.zeros((3, cm.model.nv))
        mujoco.mj_jacSite(cm.model, cm.data, jac, None, cm.limb_site_id[limb])
        jacobians.append(jac)
    J = np.vstack(jacobians)
    base_map = J[:, :6].T
    target = cm.data.qfrc_bias[:6]
    inverse = np.linalg.pinv(base_map)
    forces = inverse @ target
    for projection_step in range(120):
        for index, (limb, hold) in enumerate(contacts):
            f = forces[3 * index:3 * index + 3]
            if limb.endswith("_foot") or hold.type in FRICTION_GATED_TYPES:
                normal = np.asarray(hold.normal)
                n = float(f @ normal)
                tangent = f - n * normal
                t = sqrt(float(tangent @ tangent))
                mu = hold.friction * 0.75  # controller reserve; physical limit stays unchanged
                if t > mu * n or n < 0:
                    projected_n = max(0.0, (n + mu * t) / (1 + mu * mu))
                    f[:] = projected_n * normal + tangent * (mu * projected_n / max(t, 1e-15))
            magnitude = sqrt(float(f @ f))
            if magnitude > 0.85 * hold.max_grip_n:
                f *= 0.85 * hold.max_grip_n / magnitude
        error = target - base_map @ forces
        if sqrt(float(error @ error)) < 1e-5:
            break
        if projection_step < 119:
            forces += inverse @ error
    # Finish on the cone/cap projection. If allocation cannot balance the
    # base, its residual is left on the unactuated root, never injected as
    # an external force. Real dynamics decides whether the stance survives.
    return cm.data.qfrc_bias - J.T @ forces

def attempt_route(route: Route, renderer=None, frame_every: int = 15, *,
                  timestep: float = 0.002, observer=None, integration_observer=None) -> AttemptResult:
    """Run one attempt. ``observer(cm, phase, move_index, current_holds)`` sees
    every integrated step, plus initialization/terminal states. Observers must
    treat the model/data as read-only. ``None`` denotes a detached limb.

    A failed controller/IK attempt is not a certificate of physical impossibility.
    Time windows use the compiled model timestep, including non-default values.
    ``integration_observer`` has the same signature, but sees the reaction forces
    actually used by mj_step before derived position caches are refreshed; it is
    intended for force/torque telemetry, not rendering.
    """
    if not isinstance(frame_every, int) or frame_every <= 0:
        raise ValueError("frame_every must be a positive integer")
    cm = ClimberModel.build(route, timestep=timestep)
    cm.init_qpos_standing()
    # The joint axes use the same sign on both sides. Mirror the lateral
    # seed angles so symmetric holds do not create an arbitrary sideways lean.
    for joint in ("shoulder_abd_R", "hip_abd_R"):
        adr = cm.model.joint(joint).qposadr[0]
        cm.data.qpos[adr] *= -1
    dt = float(cm.model.opt.timestep)
    current_holds = dict(route.start)
    result = AttemptResult(route=route)
    step_count = 0
    joint_ids = cm.model.actuator_trnid[:, 0]
    actuator_qadr = cm.model.jnt_qposadr[joint_ids]
    actuator_dofs = cm.model.jnt_dofadr[joint_ids]
    posture_targets = cm.data.qpos[actuator_qadr].copy()

    def apply_controls():
        compensation = _support_torques(cm, current_holds)
        cm.data.ctrl[:] = posture_targets + compensation[actuator_dofs] / A.JOINT_KP

    def emit(phase, move_index, force_capture=False):
        if observer is not None:
            observer(cm, phase, move_index, dict(current_holds))
        if numerical_failure(phase) is not None:
            return
        if renderer is not None and (force_capture or step_count % frame_every == 0):
            result.frames.append(_capture(cm, renderer))

    def numerical_failure(phase, expected_time=None):
        bad_fields = [name for name in ("qpos", "qvel", "ctrl", "qacc", "qfrc_bias")
                      if not np.all(np.isfinite(getattr(cm.data, name)))]
        warnings = [name for name in ("mjWARN_BADQPOS", "mjWARN_BADQVEL", "mjWARN_BADQACC", "mjWARN_BADCTRL")
                    if cm.data.warning[getattr(mujoco.mjtWarning, name)].number > 0]
        reset = expected_time is not None and not np.isclose(cm.data.time, expected_time, rtol=0, atol=1e-10)
        if bad_fields or warnings or reset:
            return MoveOutcome(None, False, phase,
                "numerical instability interrupted simulation; no physical feasibility conclusion",
                dict(category="numerical_instability", nonfinite_fields=bad_fields,
                     warnings=warnings, time_reset=bool(reset), time_s=float(cm.data.time)))
        return None

    def step(phase, move_index):
        nonlocal step_count
        invalid = numerical_failure(phase)
        if invalid is not None:
            return invalid
        apply_controls()
        invalid = numerical_failure(phase)
        if invalid is not None:
            return invalid
        expected_time = cm.data.time + dt
        mujoco.mj_step(cm.model, cm.data)
        step_count += 1
        invalid = numerical_failure(phase, expected_time)
        if invalid is not None:
            return invalid
        if integration_observer is not None:
            integration_observer(cm, phase, move_index, dict(current_holds))
        # efc_force here is the force actually used during the integration.
        failure = _check_all_anchored(cm, current_holds)
        # mj_step integrates qpos after computing its caches. Refresh them so
        # arrival checks, viewers and the next force check see the new state.
        mujoco.mj_forward(cm.model, cm.data)
        invalid = numerical_failure(phase)
        if invalid is not None:
            return invalid
        failure = failure or _check_all_anchored(cm, current_holds)
        emit(phase, move_index)
        if failure is not None:
            failure.phase = phase
        return failure

    def run_window(seconds, phase, move_index):
        for _ in range(int(np.ceil(seconds / dt))):
            failure = step(phase, move_index)
            if failure is not None:
                return failure
        return None

    def finish_failure(failure, move_index, fall=False):
        result.outcomes.append(failure)
        result.fell = fall and failure.detail.get("category") != "numerical_instability"
        if failure.detail.get("category") == "numerical_instability":
            # Preserve the last valid visual sample; invalid/reset engine state
            # must not be rendered as if it were a physical trajectory.
            return
        if fall:
            failed_limb = failure.detail.get("limb")
            if failed_limb:
                cm.set_grip_active(failed_limb, False)
                current_holds[failed_limb] = None
                mujoco.mj_forward(cm.model, cm.data)
        if numerical_failure(failure.phase) is None:
            emit("failure", move_index, force_capture=True)

    start_targets = {limb: route.hold(hn).pos for limb, hn in current_holds.items()}
    residual = solve_pose_ik(cm, start_targets, tol=INITIAL_POSE_TOL, max_iters=600)
    for limb in LIMBS:
        _set_ctrl_from_qpos(cm, limb)
    mujoco.mj_forward(cm.model, cm.data)
    invalid = numerical_failure("initialization")
    if invalid is not None:
        finish_failure(invalid, -1)
        return result
    emit("initialization", -1, force_capture=True)
    invalid = numerical_failure("initialization")
    if invalid is not None:
        finish_failure(invalid, -1)
        return result
    if not np.isfinite(residual) or residual > INITIAL_POSE_TOL:
        finish_failure(MoveOutcome(None, False, "initialization",
            "starting-pose IK did not converge; no physical feasibility conclusion",
            dict(category="initial_ik_nonconvergence", residual_m=float(residual), time_s=0.0)), -1)
        return result

    for limb, hn in current_holds.items():
        cm.set_limb_anchor(limb, route.hold(hn).pos)
        cm.set_grip_active(limb, True)
        cm.set_limb_gain(limb, A.JOINT_KP, A.JOINT_KV)
        _set_ctrl_from_qpos(cm, limb)
    posture_targets[:] = cm.data.qpos[actuator_qadr]
    mujoco.mj_forward(cm.model, cm.data)
    invalid = numerical_failure("initialization")
    if invalid is not None:
        finish_failure(invalid, -1)
        return result
    apply_controls()
    mujoco.mj_forward(cm.model, cm.data)
    bad = numerical_failure("initialization") or _check_all_anchored(cm, current_holds)
    if bad is not None:
        bad.phase = "initialization"
    else:
        bad = run_window(INITIAL_SETTLE_S, "initialization", -1)
    if bad is not None:
        bad.reason = "starting stance failed: " + bad.reason
        finish_failure(bad, -1, fall=True)
        return result
    if not route.moves:
        result.outcomes.append(MoveOutcome(None, True, "initialization",
            f"Starting stance held for {INITIAL_SETTLE_S:g} seconds",
            dict(category="completed", duration_s=INITIAL_SETTLE_S,
                 time_s=float(cm.data.time))))
        return result

    for move_index, move in enumerate(route.moves):
        limb = move.limb
        target_hold = route.hold(move.hold)
        target_pos = np.asarray(target_hold.pos, dtype=float)
        contact_distance = min(CONTACT_DISTANCE_M, target_hold.radius)
        contact_speed = (min(CONTACT_SPEED_M_S, UNILATERAL_CONTACT_SPEED_M_S)
                         if limb.endswith("_foot") or target_hold.type in FRICTION_GATED_TYPES
                         else CONTACT_SPEED_M_S)
        base_distance = float(np.linalg.norm(target_pos - _limb_base_pos(cm, limb)))
        distance = float(np.linalg.norm(target_pos - cm.limb_site_pos(limb)))
        max_reach = _max_reach(limb)
        if base_distance > max_reach + REACH_TOL:
            finish_failure(MoveOutcome(move, False, "reachability",
                f"'{move.hold}' exceeds {limb}'s maximum extension from the current stance",
                dict(category="geometric_reach_limit", distance_m=base_distance,
                     site_distance_m=distance, max_reach_m=max_reach,
                     residual_m=base_distance - max_reach)), move_index)
            break
        ik = solve_ik_best(cm, limb, target_pos, tol=REACH_TOL)
        if not ik.reached:
            finish_failure(MoveOutcome(move, False, "planning",
                f"IK did not converge for {limb} at '{move.hold}'; reachability is unresolved",
                dict(category="ik_nonconvergence", residual_m=ik.residual_m,
                     distance_m=distance, max_reach_m=max_reach)), move_index)
            break

        qadr = cm.limb_joint_qpos_adr[limb]
        actuator_ids = cm.actuator_ctrl_for_limb(limb)
        command_q = cm.data.qpos[qadr].copy()
        command_velocity = np.zeros_like(command_q)
        target_q = ik.q.copy()
        cm.set_grip_active(limb, False)
        current_holds[limb] = None
        mujoco.mj_forward(cm.model, cm.data)
        n_steps = int(np.ceil(TRANSITION_S / dt))
        replan_steps = max(1, int(round(REPLAN_S / dt)))
        failure = None
        arrived = False
        for index in range(n_steps):
            if index > 0 and index % replan_steps == 0:
                refined = solve_ik_best(cm, limb, target_pos, tol=REACH_TOL,
                                        n_random_seeds=0, max_iters=100)
                if refined.reached:
                    target_q = refined.q
            desired_velocity = np.clip((target_q - command_q) / dt,
                                       -COMMAND_SPEED_RAD_S, COMMAND_SPEED_RAD_S)
            command_velocity += np.clip(desired_velocity - command_velocity,
                                        -COMMAND_ACCEL_RAD_S2 * dt, COMMAND_ACCEL_RAD_S2 * dt)
            advance = command_velocity * dt
            crossing = np.abs(advance) >= np.abs(target_q - command_q)
            command_q = np.where(crossing, target_q, command_q + advance)
            command_velocity[crossing] = 0.0
            # Compensate the moving chain's gravity/Coriolis bias through the
            # same torque-limited position actuators. This removes static PD
            # tracking error without adding an unbounded external body force.
            posture_targets[actuator_ids] = command_q
            failure = step("dynamic", move_index)
            if failure is not None:
                break
            distance, speed = _contact_state(cm, limb, target_pos)
            if distance <= contact_distance and speed <= contact_speed:
                arrived = True
                break
        if failure is not None:
            failure.move = move
            failure.reason = f"during move to '{move.hold}': {failure.reason}"
            finish_failure(failure, move_index, fall=True)
            break
        if not arrived:
            distance, speed = _contact_state(cm, limb, target_pos)
            finish_failure(MoveOutcome(move, False, "dynamic",
                f"controller did not arrive at '{move.hold}' within the contact distance/speed limits",
                dict(category="contact_not_reached", distance_m=distance, speed_m_s=speed,
                     distance_limit_m=contact_distance, speed_limit_m_s=contact_speed,
                     ik_residual_m=ik.residual_m)), move_index)
            break

        # The nominal point describes a finite contact patch. Attach where
        # contact actually occurred; snapping to its center would inject an
        # artificial constraint impulse even after an otherwise valid arrival.
        cm.set_limb_anchor(limb, cm.limb_site_pos(limb))
        cm.set_grip_active(limb, True)
        posture_targets[actuator_ids] = cm.data.qpos[qadr]
        current_holds[limb] = move.hold
        mujoco.mj_forward(cm.model, cm.data)
        failure = numerical_failure("settle")
        if failure is not None:
            failure.move = move
            finish_failure(failure, move_index)
            break
        apply_controls()
        mujoco.mj_forward(cm.model, cm.data)
        failure = numerical_failure("settle")
        if failure is not None:
            failure.move = move
            finish_failure(failure, move_index)
            break
        emit("contact", move_index, force_capture=True)
        failure = _check_all_anchored(cm, current_holds)
        if failure is None:
            failure = run_window(SETTLE_S, "settle", move_index)
        if failure is not None:
            failure.move, failure.phase = move, "settle"
            failure.reason = f"after arrival at '{move.hold}': {failure.reason}"
            finish_failure(failure, move_index, fall=True)
            break
        result.outcomes.append(MoveOutcome(move, True, "settle",
            f"{limb} reached and held '{move.hold}'",
            dict(ik_residual_m=ik.residual_m, arrival_distance_m=distance,
                 arrival_speed_m_s=speed, speed_limit_m_s=contact_speed,
                 contact_radius_m=contact_distance, category="completed")))
    if result.success:
        emit("complete", len(route.moves) - 1, force_capture=True)
    return result
