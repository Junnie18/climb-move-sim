"""Regression tests for the two real bugs the joint-axis convention and the IK solver
hit during development: an inverted flex-extension axis that made "reach toward the
wall" kinematically impossible regardless of joint range, and a damped-least-squares
solver that could stall at a near-singular fully-extended elbow/knee even when the
target was well within the limb's true reachable envelope.
"""

import numpy as np

from climbsim.holds import Hold, HoldType
from climbsim.kinematics import solve_ik_best, solve_pose_ik
from climbsim.model import ClimberModel, arm_reach
from climbsim.route import Route


def _simple_route(extra_holds=()):
    holds = {
        "lh0": Hold("lh0", (-0.22, 0.05, 1.55), HoldType.JUG),
        "rh0": Hold("rh0", (0.22, 0.05, 1.55), HoldType.JUG),
        "lf0": Hold("lf0", (-0.18, 0.05, 0.75), HoldType.FOOTHOLD),
        "rf0": Hold("rf0", (0.18, 0.05, 0.75), HoldType.FOOTHOLD),
    }
    for h in extra_holds:
        holds[h.name] = h
    start = {"left_hand": "lh0", "right_hand": "rh0", "left_foot": "lf0", "right_foot": "rf0"}
    return Route(name="test", holds=holds, start=start, moves=[])


def _settled_model(route):
    cm = ClimberModel.build(route)
    cm.init_qpos_standing()
    solve_pose_ik(cm, {limb: route.hold(hn).pos for limb, hn in route.start.items()})
    return cm


def test_right_hand_can_reach_a_hold_at_the_wall():
    """The historical bug: with the flex_ext axis inverted, IK could not find *any*
    joint configuration reaching toward the wall (-y), regardless of restart count,
    because the required rotation direction was outside every seed's search basin in
    the wrong half of the joint's range. A target directly in front of the shoulder,
    well within arm_reach(), must converge.
    """
    cm = _settled_model(_simple_route())
    shoulder = cm.data.xpos[cm.model.body("upper_arm_R").id]
    target = shoulder + np.array([0.0, -0.3, 0.05])  # toward the wall, modest reach
    result = solve_ik_best(cm, "right_hand", target, tol=0.02)
    assert result.reached, f"residual={result.residual_m:.3f}m"


def test_ik_does_not_stall_at_near_singular_extension():
    """A target ~79% of max arm extension away, at a shoulder-height horizontal
    reach, is the exact case that stalled a single-seed damped-least-squares solve at
    a fully-extended, near-singular elbow (residual ~13cm short) during development --
    confirmed reachable by brute-force multi-restart, which is what solve_ik_best now
    does by default.
    """
    cm = _settled_model(_simple_route())
    shoulder = cm.data.xpos[cm.model.body("upper_arm_R").id]
    reach = arm_reach()
    target = shoulder + np.array([reach * 0.5, -reach * 0.6, 0.0])
    assert np.linalg.norm(target - shoulder) < reach * 0.85
    result = solve_ik_best(cm, "right_hand", target, tol=0.02)
    assert result.reached, f"residual={result.residual_m:.3f}m"


def test_target_beyond_max_reach_correctly_fails():
    cm = _settled_model(_simple_route())
    shoulder = cm.data.xpos[cm.model.body("upper_arm_R").id]
    starting_distance = float(np.linalg.norm(np.asarray(cm.limb_site_pos("right_hand"))
                                              - (shoulder + np.array([0.0, -arm_reach() * 1.6, 0.0]))))
    target = shoulder + np.array([0.0, -arm_reach() * 1.6, 0.0])
    result = solve_ik_best(cm, "right_hand", target, tol=0.02)
    assert not result.reached
    # best-effort residual must never exceed the naive straight-line distance (the
    # residual of just doing nothing) -- a historical bug reported a *worse* result
    # than doing nothing, because solve_ik returned wherever the last iteration
    # landed instead of the best point visited along the way.
    assert result.residual_m <= starting_distance + 1e-6


def test_planning_does_not_modify_live_simulation_state():
    """Replanning a reach must not teleport the limb or invalidate live caches."""
    cm = _settled_model(_simple_route())
    shoulder = cm.data.xpos[cm.model.body("upper_arm_R").id].copy()
    target = shoulder + np.array([0.0, -0.3, 0.05])
    before = {name: getattr(cm.data, name).copy() for name in
              ("qpos", "qvel", "ctrl", "site_xpos", "cdof", "xpos")}
    result = solve_ik_best(cm, "right_hand", target, tol=0.002)
    assert result.reached
    for name, expected in before.items():
        np.testing.assert_array_equal(getattr(cm.data, name), expected,
                                      err_msg=f"planning changed live {name}")


def test_ik_jacobian_does_not_depend_on_stale_live_com_cache():
    """A stale cdof cache must not change the solution of a known reachable pose."""
    import mujoco
    from climbsim.kinematics import solve_ik

    cm = _settled_model(_simple_route())
    qadr = cm.limb_joint_qpos_adr["right_hand"]
    seed = cm.data.qpos[qadr].copy()
    cm.data.qpos[qadr] = seed + np.array([0.08, -0.04, 0.04, -0.05])
    mujoco.mj_kinematics(cm.model, cm.data)
    target = cm.limb_site_pos("right_hand")
    cm.data.qpos[qadr] = seed
    cm.data.cdof[:] = 0.0
    result = solve_ik(cm, "right_hand", target, tol=0.001)
    assert result.reached, f"known reachable target missed by {result.residual_m} m"
