"""Physics-correctness tests for grip-force extraction, using a case simple enough to
compute the expected answer by hand: a point mass hanging from a single `connect`
constraint must be held up by exactly gravity, in the correct direction. This is the
same check used during development to confirm `efc_force` extraction from MuJoCo's
solver is real physics, not a placeholder (see forces.py docstring).
"""

import mujoco
import numpy as np

from climbsim.forces import check_grip
from climbsim.holds import Hold, HoldType


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
