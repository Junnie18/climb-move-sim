import numpy as np
import mujoco
import pytest

from climbsim import anthropometry as A
from climbsim.holds import Hold, HoldType
from climbsim.model import ClimberModel
from climbsim.route import Route


def _simple_route():
    holds = {
        "lh0": Hold("lh0", (-0.22, 0.05, 1.55), HoldType.JUG),
        "rh0": Hold("rh0", (0.22, 0.05, 1.55), HoldType.JUG),
        "lf0": Hold("lf0", (-0.18, 0.05, 0.75), HoldType.FOOTHOLD),
        "rf0": Hold("rf0", (0.18, 0.05, 0.75), HoldType.FOOTHOLD),
    }
    start = {"left_hand": "lh0", "right_hand": "rh0", "left_foot": "lf0", "right_foot": "rf0"}
    return Route(name="test", holds=holds, start=start, moves=[])


def test_model_compiles_with_expected_dofs():
    cm = ClimberModel.build(_simple_route())
    # free joint (6 dof, 7 qpos) + 4 limbs * 4 hinges each
    assert cm.model.nv == 6 + 4 * 4
    assert cm.model.nq == 7 + 4 * 4
    assert cm.model.nu == 4 * 4


def test_segment_mass_fractions_sum_to_total_body_mass():
    cm = ClimberModel.build(_simple_route())
    total = cm.model.body_mass.sum()
    # mocap anchor bodies contribute 0 mass; only the climber's segments should sum to
    # MASS_KG (world body itself also carries no mass)
    assert abs(total - A.MASS_KG) < 0.5


def test_torque_limits_applied_to_actuators():
    cm = ClimberModel.build(_simple_route())
    for aid in range(cm.model.nu):
        assert cm.model.actuator_forcelimited[aid] == 1
        lo, hi = cm.model.actuator_forcerange[aid]
        assert hi > 0 and lo == -hi


def test_requested_timestep_controls_elapsed_physics_time():
    cm = ClimberModel.build(_simple_route(), timestep=0.001)
    for _ in range(10):
        mujoco.mj_step(cm.model, cm.data)
    assert cm.data.time == pytest.approx(0.01)


@pytest.mark.parametrize("dt", [0, -0.001, float("nan"), float("inf")])
def test_invalid_timestep_rejected(dt):
    with pytest.raises(ValueError, match="timestep"):
        ClimberModel.build(_simple_route(), timestep=dt)


def test_reach_bounds_include_real_segment_offsets():
    from climbsim.model import arm_reach, leg_reach
    # Triangle-inequality bounds from the compiled geometry, including the
    # 23.94 cm ankle-to-site offset omitted by the old 65%-foot heuristic.
    assert arm_reach() == pytest.approx(0.6976301312, abs=1e-8)
    assert leg_reach() == pytest.approx(1.1001626068, abs=1e-8)
