"""Bounded geometric pose preview using the simulator's actual articulated model.

This solves joint positions only. A pose solution never certifies support,
grip strength, collision clearance, or a traversable sequence of moves.
"""

import numpy as np

from climbsim.feasibility import INITIAL_POSE_TOL
from climbsim.kinematics import solve_pose_ik
from climbsim.model import ClimberModel
from climbsim.route import LIMBS, Route
from climbsim.trajectory_export import BODY_NAMES, export_static_geometry


def build_pose_preview(route: Route) -> dict:
    """Solve the four ``route.start`` targets, with a fixed iteration limit.

    Unresolved poses still return their finite best geometry and target errors.
    Callers may preview a planned end stance by submitting its assignments as
    ``start``; this does not execute or validate the intervening moves.
    """
    cm = ClimberModel.build(route)
    cm.init_qpos_standing()
    for joint in ("shoulder_abd_R", "hip_abd_R"):
        cm.data.qpos[cm.model.joint(joint).qposadr[0]] *= -1
    targets = {limb: route.hold(route.start[limb]).pos for limb in LIMBS}
    solve_pose_ik(cm, targets, tol=INITIAL_POSE_TOL, max_iters=600)
    sites = np.asarray([cm.limb_site_pos(limb) for limb in LIMBS])
    target_positions = np.asarray([targets[limb] for limb in LIMBS])
    residuals = np.linalg.norm(sites - target_positions, axis=1)
    body_ids = [cm.model.body(name).id for name in BODY_NAMES]
    positions = cm.data.xpos[body_ids]
    quaternions = cm.data.xquat[body_ids]
    if not all(np.all(np.isfinite(value)) for value in
               (positions, quaternions, sites, target_positions, residuals)):
        raise ValueError("The pose solver did not produce a finite preview.")
    maximum = float(np.max(residuals))
    return {
        "geometry": export_static_geometry(route),
        "bodyNames": list(BODY_NAMES),
        "pos": positions.tolist(),
        "quat": quaternions.tolist(),
        "limbs": list(LIMBS),
        "sites": sites.tolist(),
        "targets": target_positions.tolist(),
        "residuals": residuals.tolist(),
        "converged": maximum <= INITIAL_POSE_TOL,
        "maxResidualM": maximum,
        "limitations": [
            "Kinematic pose preview only: support forces, grip strength and movement are not checked.",
            "An unresolved pose is a solver result, not proof that a person cannot reach the holds.",
            "The wall is visual only; body-wall collision response is not modeled.",
        ],
    }
