"""Touch down slowly enough to avoid controller-induced foot slips.

The contact cones and actuator limits are checked on actual solver forces,
including the first attached sample. No force, state, or slip check is mocked.
"""

from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest

from climbsim.feasibility import attempt_route
from climbsim.route import Move, Route


SHOWCASE = Path(__file__).resolve().parents[1] / "data/routes/showcase"


def foot_step(limb, dx=0.0, dz=0.0):
    route = Route.from_json(SHOWCASE / "01_supported_steps.json")
    start = route.hold(route.start[limb])
    position = (start.pos[0] + dx, start.pos[1], start.pos[2] + dz)
    route.holds["target"] = replace(start, name="target", pos=position)
    route.moves = [Move(limb, "target")]
    return route


@pytest.mark.parametrize("limb,dx,dz", [
    ("left_foot", 0.0, 0.05),
    ("left_foot", 0.0, 0.10),
    ("left_foot", -0.05, 0.0),
    ("right_foot", 0.0, 0.05),
    ("right_foot", 0.0, 0.10),
    ("right_foot", 0.05, 0.0),
    # Additional displacements beyond those used to diagnose acquisition.
    ("left_foot", 0.0, 0.20),
    ("right_foot", 0.10, 0.0),
])
def test_foot_steps_decelerate_before_attachment_and_keep_admissible_loads(limb, dx, dz):
    route = foot_step(limb, dx, dz)
    acquisitions = []
    checked_samples = 0

    def audit(cm, phase, move_index, current_holds):
        nonlocal checked_samples
        assert not np.any(cm.data.qfrc_applied)
        assert not np.any(cm.data.xfrc_applied)
        assert np.all(cm.data.actuator_force <= cm.model.actuator_forcerange[:, 1] + 1e-8)
        assert np.all(cm.data.actuator_force >= cm.model.actuator_forcerange[:, 0] - 1e-8)
        for contact_limb, hold_name in current_holds.items():
            if hold_name is None or not cm.data.eq_active[cm.limb_eq_id[contact_limb]]:
                continue
            rows = ((cm.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                    & (cm.data.efc_id == cm.limb_eq_id[contact_limb]))
            force = cm.data.efc_force[rows]
            hold = route.hold(hold_name)
            assert force.shape == (3,)
            assert np.linalg.norm(force) <= hold.max_grip_n + 1e-6
            if contact_limb.endswith("_foot"):
                assert hold.normal == (0, 0, 1)
                assert force[2] >= -1e-6
                assert np.hypot(force[0], force[1]) <= hold.friction * force[2] + 1e-6
        checked_samples += 1

    def observe(cm, phase, move_index, current_holds):
        if phase == "contact":
            jacobian = np.zeros((3, cm.model.nv))
            mujoco.mj_jacSite(cm.model, cm.data, jacobian, None, cm.limb_site_id[limb])
            speed = float(np.linalg.norm(jacobian @ cm.data.qvel))
            acquisitions.append(speed)
            assert speed <= 0.02 + 1e-10, "a unilateral foot should finish decelerating before attachment"
            # The attachment sample is checked before any settling step.
            audit(cm, phase, move_index, current_holds)

    result = attempt_route(route, observer=observe, integration_observer=audit)
    assert result.success, result.first_failure
    assert len(acquisitions) == 1
    assert checked_samples > 3000
    assert result.outcomes[0].detail["arrival_speed_m_s"] <= 0.02
