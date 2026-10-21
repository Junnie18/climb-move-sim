"""Physics-correctness tests for grip-force extraction, using a case simple enough to
compute the expected answer by hand: a point mass hanging from a single `connect`
constraint must be held up by exactly gravity, in the correct direction. This is the
same check used during development to confirm `efc_force` extraction from MuJoCo's
solver is real physics, not a placeholder (see forces.py docstring).
"""

import mujoco
import numpy as np
import pytest

from climbsim.forces import check_grip
from climbsim.holds import Hold, HoldType


def constrained_mass(required_force):
    """One kg at rest; the connect force must oppose the specified gravity."""
    gravity = " ".join(str(-component) for component in required_force)
    model = mujoco.MjModel.from_xml_string(f"""
    <mujoco>
      <option gravity="{gravity}"/>
      <worldbody>
        <body name="mass"><freejoint/>
          <geom type="sphere" size="0.1" mass="1"/>
          <site name="limb"/>
        </body>
        <site name="hold"/>
      </worldbody>
      <equality><connect site1="limb" site2="hold" solref="0.01 1"/></equality>
    </mujoco>
    """)
    data = mujoco.MjData(model)
    for _ in range(300):
        mujoco.mj_step(model, data)
    return model, data


def test_sloper_cannot_pull_even_with_zero_shear():
    model, data = constrained_mass((0, -100, 0))
    result = check_grip("left_hand", Hold("s", (0, 0, 0), HoldType.SLOPER), model, data, 0)
    assert result.normal_n == pytest.approx(-100, abs=0.01)
    assert not result.friction_cone_ok


@pytest.mark.parametrize("hold_type", list(HoldType))
def test_feet_cannot_pull_on_any_hold_type(hold_type):
    force = (0, 0, -100) if hold_type == HoldType.FOOTHOLD else (0, -100, 0)
    model, data = constrained_mass(force)
    result = check_grip("left_foot", Hold("h", (0, 0, 0), hold_type), model, data, 0)
    assert not result.friction_cone_ok


def test_foothold_supports_bodyweight_in_upward_direction():
    model, data = constrained_mass((0, 0, 100))
    result = check_grip("right_foot", Hold("h", (0, 0, 0), HoldType.FOOTHOLD), model, data, 0)
    assert result.normal_n == pytest.approx(100, abs=0.01)
    assert result.tangential_n == pytest.approx(0, abs=0.01)
    assert result.ok


@pytest.mark.parametrize("shear, passes", [(40, True), (60, False)])
def test_foot_uses_configured_surface_normal_and_friction(shear, passes):
    model, data = constrained_mass((shear, 0, 100))
    hold = Hold("edge", (0, 0, 0), HoldType.JUG,
                normal=(0, 0, 2), friction_override=0.5)
    result = check_grip("right_foot", hold, model, data, 0)
    assert result.normal_n == pytest.approx(100, abs=0.01)
    assert result.tangential_n == pytest.approx(shear, abs=0.01)
    assert result.friction_cone_ok == passes


@pytest.mark.parametrize("hold_type", [HoldType.JUG, HoldType.CRIMP])
def test_positive_hand_grips_retain_effective_anchor_model(hold_type):
    model, data = constrained_mass((0, -100, 0))
    result = check_grip("right_hand", Hold("h", (0, 0, 0), hold_type), model, data, 0)
    assert result.ok


def test_grip_cap_override_is_enforced_on_actual_reaction_force():
    model, data = constrained_mass((0, 0, 100))
    hold = Hold("edge", (0, 0, 0), HoldType.CRIMP, max_grip_n_override=90)
    assert not check_grip("left_hand", hold, model, data, 0).within_max_grip


def test_force_extraction_matches_world_frame_gravity_on_each_axis():
    from climbsim.forces import grip_force

    model, data = constrained_mass((15, -20, 30))
    np.testing.assert_allclose(grip_force(model, data, 0), [15, -20, 30], atol=0.01)


def test_unknown_limb_is_rejected():
    model, data = constrained_mass((0, 0, 100))
    with pytest.raises(ValueError, match="limb"):
        check_grip("tail", Hold("h", (0, 0, 0), HoldType.JUG), model, data, 0)


def test_single_mass_hang_matches_gravity_exactly():
    mass = 1.0
    g = 9.81
    xml = f"""
    <mujoco>
      <option gravity="0 0 -{g}"/>
      <worldbody>
        <body name="a" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="{mass}"/>
          <site name="s1" pos="0 0 0"/>
        </body>
        <site name="s2" pos="0 0 1"/>
      </worldbody>
      <equality>
        <connect name="c1" site1="s1" site2="s2" active="true" solref="0.01 1"/>
      </equality>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    for _ in range(300):
        mujoco.mj_step(model, data)

    from climbsim.forces import grip_force
    eq_id = model.equality("c1").id
    force = grip_force(model, data, eq_id)
    assert abs(np.linalg.norm(force) - mass * g) < 0.05
    assert force[2] > 0  # holds the mass UP against gravity


def test_jug_within_friction_cone_and_grip_cap_passes():
    hold = Hold("h", (0, 0, 0), HoldType.JUG)

    class FakeData:
        efc_type = np.array([0, 0, 0])
        efc_id = np.array([0, 0, 0])
        efc_force = np.array([0.0, 50.0, 200.0])  # mostly "into the wall", modest shear

    class FakeModel:
        pass

    gc = check_grip("left_hand", hold, FakeModel(), FakeData(), eq_id=0)
    assert gc.ok


def test_sloper_with_no_normal_force_fails_friction_cone():
    """A sloper has no positive edge -- with zero force component pressing it into the
    wall, friction has nothing to grip with, so any nonzero shear must fail."""
    hold = Hold("h", (0, 0, 0), HoldType.SLOPER)

    class FakeData:
        efc_type = np.array([0, 0, 0])
        efc_id = np.array([0, 0, 0])
        efc_force = np.array([50.0, 0.0, 0.0])  # pure shear, zero normal component

    class FakeModel:
        pass

    gc = check_grip("left_hand", hold, FakeModel(), FakeData(), eq_id=0)
    assert not gc.ok
    assert not gc.friction_cone_ok


def test_force_exceeding_max_grip_fails_even_within_friction_cone():
    hold = Hold("h", (0, 0, 0), HoldType.CRIMP)  # max_grip_n = 550

    class FakeData:
        efc_type = np.array([0, 0, 0])
        efc_id = np.array([0, 0, 0])
        efc_force = np.array([0.0, 0.0, 900.0])  # pure normal, well within friction cone

    class FakeModel:
        pass

    gc = check_grip("left_hand", hold, FakeModel(), FakeData(), eq_id=0)
    assert gc.friction_cone_ok
    assert not gc.within_max_grip
    assert not gc.ok
