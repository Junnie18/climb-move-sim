"""Numerical inverse kinematics for one limb at a time, via a damped-least-
squares Jacobian solve against the actual compiled MuJoCo model -- not a
closed-form approximation, so it automatically respects the model's real
joint axes, offsets and limits. A converged solve provides a candidate
reach in the current torso configuration; numerical non-convergence leaves
reachability unresolved and is not a proof of physical impossibility.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from climbsim.model import LIMB_JOINTS


@dataclass
class IKResult:
    q: np.ndarray  # solved joint angles for this limb's DOFs, in LIMB_JOINTS order
    residual_m: float  # final distance from target, meters
    reached: bool
    iterations: int


def solve_ik(cm, limb: str, target_pos, tol=0.015, max_iters=300, damping=0.15, gain=0.5,
             seed_q=None) -> IKResult:
    """Solve for `limb`'s joint angles that bring its grip site to `target_pos`,
    holding every other DOF (torso free joint + other three limbs) fixed at
    its current qpos. Planning uses isolated MjData; live positions, velocities
    and derived dynamics caches are never changed.

    `seed_q`, if given, is the starting joint configuration (defaults to the
    limb's current qpos). A single damped-least-squares run can stall at a
    local minimum -- most commonly a near-singular fully-extended elbow/knee,
    where the Jacobian has almost no response left along the reach direction
    -- so `solve_ik_best` below retries from alternate seeds when this stalls.
    """
    model = cm.model
    data = mujoco.MjData(model)
    data.qpos[:] = cm.data.qpos
    qadr = cm.limb_joint_qpos_adr[limb]
    dofadr = cm.limb_joint_dof_adr[limb]
    site_id = cm.limb_site_id[limb]
    joint_names = LIMB_JOINTS[limb]
    lo = np.array([model.joint(j).range[0] for j in joint_names])
    hi = np.array([model.joint(j).range[1] for j in joint_names])
    target = np.asarray(target_pos, dtype=float)

    jacp = np.zeros((3, model.nv))
    q = np.array([data.qpos[a] for a in qadr]) if seed_q is None else np.array(seed_q, dtype=float)
    best_q, best_residual = q.copy(), np.inf
    it = 0
    for it in range(1, max_iters + 1):
        for a, v in zip(qadr, q):
            data.qpos[a] = v
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        pos = data.site_xpos[site_id]
        err = target - pos
        residual = float(np.linalg.norm(err))
        if residual < best_residual:
            best_q, best_residual = q.copy(), residual
        if residual < tol:
            break
        mujoco.mj_jacSite(model, data, jacp, None, site_id)
        J = jacp[:, dofadr]  # 3 x 4 Jacobian restricted to this limb's DOFs
        JJt = J @ J.T + damping ** 2 * np.eye(3)
        dq = gain * (J.T @ np.linalg.solve(JJt, err))
        step = np.clip(dq, -0.15, 0.15)  # limit step size to avoid overshoot/oscillation
        q = np.clip(q + step, lo, hi)

    # Report the best configuration seen across the whole run, not just wherever the
    # last iteration happened to land -- damped least squares can overshoot and end an
    # unlucky run slightly worse than an earlier point it already passed through, which
    # previously showed up as a reachability check reporting a *larger* residual than the
    # limb's actual starting distance from the target (see solve_ik_best docstring).
    for a, v in zip(qadr, best_q):
        data.qpos[a] = v
    mujoco.mj_kinematics(model, data)
    return IKResult(q=best_q, residual_m=best_residual, reached=best_residual < tol, iterations=it)


def solve_ik_best(cm, limb: str, target_pos, tol=0.015, n_random_seeds=14, rng=None, **kwargs) -> IKResult:
    """Multi-start wrapper around `solve_ik`: tries the limb's current pose,
    then its neutral bind pose, then a handful of random within-range seeds,
    keeping the best (lowest-residual) result. Stops early once a seed
    reaches `tol`. Live simulation state is left unchanged.
    """
    rng = rng or np.random.default_rng(0)
    joint_names = LIMB_JOINTS[limb]
    lo = np.array([cm.model.joint(j).range[0] for j in joint_names])
    hi = np.array([cm.model.joint(j).range[1] for j in joint_names])
    current_seed = np.array([cm.data.qpos[a] for a in cm.limb_joint_qpos_adr[limb]])
    neutral_seed = np.array([cm.data.qpos[a] for a in cm.limb_joint_qpos_adr[limb]]) * 0.0 + \
        (lo + hi) * 0.3  # a mildly-bent, non-singular default rather than dead center

    # Bias the elbow/knee (last DOF) away from its range extremes: a fully straight or
    # fully folded limb is a Jacobian singularity (near-zero response along the reach
    # direction) that stalls the local solve -- this alone caused an under-counted
    # "unreachable" result for a target well inside the true reachable envelope during
    # development (confirmed reachable by brute-force random restarts).
    elbow_lo, elbow_hi = lo[-1], hi[-1]
    elbow_band = (elbow_lo + 0.2 * (elbow_hi - elbow_lo), elbow_lo + 0.8 * (elbow_hi - elbow_lo))

    seeds = [current_seed, neutral_seed]
    for _ in range(n_random_seeds):
        s = rng.uniform(lo, hi)
        s[-1] = rng.uniform(*elbow_band)
        seeds.append(s)

    best = None
    for seed in seeds:
        res = solve_ik(cm, limb, target_pos, tol=tol, seed_q=seed, **kwargs)
        if best is None or res.residual_m < best.residual_m:
            best = res
        if res.reached:
            break

    return best


def solve_pose_ik(cm, limb_targets: dict, tol=0.008, max_iters=200, damping=0.1, gain=0.5) -> float:
    """Whole-body IK: simultaneously adjusts the free-floating torso pose (6
    tangent-space DOF) *and* every named limb's joint angles so all given
    limb sites reach their targets together.

    Solving each limb independently against a fixed, assumed torso pose (as
    `solve_ik` does) leaves the four attachment points mutually
    inconsistent by however much that assumed pose was wrong -- centimeter-
    scale in practice. Under a stiff `equality/connect` grip constraint that
    residual alone produced spurious multi-kN "grip forces" on an otherwise
    motionless climber during development (see model.set_limb_gain
    docstring for the matching actuator-side issue). This solves for a
    torso pose that's actually consistent with all the holds at once, the
    way a climber's body settles into a stance rather than being pinned
    from an arbitrary guess.

    Returns the final max per-limb residual (meters).
    """
    model, data = cm.model, cm.data
    torso_dofs = list(range(6))  # free joint: dofs 0-2 linear, 3-5 angular (tangent space)
    dof_groups = {limb: cm.limb_joint_dof_adr[limb] for limb in limb_targets}
    all_dofs = torso_dofs + [d for dofs in dof_groups.values() for d in dofs]
    n = len(all_dofs)

    jac = np.zeros((3, model.nv))
    dq_full = np.zeros(model.nv)
    residual = np.inf
    for _ in range(max_iters):
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        errs, jrows = [], []
        for limb, target in limb_targets.items():
            site_id = cm.limb_site_id[limb]
            pos = data.site_xpos[site_id]
            errs.append(np.asarray(target, dtype=float) - pos)
            mujoco.mj_jacSite(model, data, jac, None, site_id)
            cols = torso_dofs + dof_groups[limb]
            row = np.zeros((3, n))
            for local_i, dof in enumerate(cols):
                row[:, all_dofs.index(dof)] = jac[:, dof]
            jrows.append(row)
        err = np.concatenate(errs)
        residual = max(np.linalg.norm(e) for e in errs)
        if residual < tol:
            break
        J = np.vstack(jrows)
        JJt = J @ J.T + damping ** 2 * np.eye(J.shape[0])
        dq = gain * (J.T @ np.linalg.solve(JJt, err))
        dq = np.clip(dq, -0.2, 0.2)
        dq_full[:] = 0.0
        for i, dof in enumerate(all_dofs):
            dq_full[dof] = dq[i]
        mujoco.mj_integratePos(model, data.qpos, dq_full, 1.0)
        for limb, dofs in dof_groups.items():
            lo = np.array([model.joint(j).range[0] for j in cm_limb_joint_names(limb)])
            hi = np.array([model.joint(j).range[1] for j in cm_limb_joint_names(limb)])
            for adr, l, h in zip(cm.limb_joint_qpos_adr[limb], lo, hi):
                data.qpos[adr] = np.clip(data.qpos[adr], l, h)
    mujoco.mj_kinematics(model, data)
    residual = max(
        float(np.linalg.norm(np.asarray(t) - data.site_xpos[cm.limb_site_id[limb]]))
        for limb, t in limb_targets.items()
    )
    return residual


def cm_limb_joint_names(limb: str):
    return LIMB_JOINTS[limb]
