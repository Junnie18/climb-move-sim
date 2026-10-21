"""Independently audit the executed dynamics of the supported showcase.

This spies on the real MuJoCo integrator; it never replaces a force or state.
The checks catch controller teleportation, uncapped external actuation, and
success reported despite an inadmissible support force.
"""

from pathlib import Path

import mujoco
import numpy as np

from climbsim.feasibility import attempt_route
from climbsim.route import LIMBS, Route


def test_supported_sequence_uses_admissible_contacts_and_capped_actuation(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    route = Route.from_json(root / "data/routes/showcase/01_supported_steps.json")
    assigned_holds = dict(route.start)
    last_integrated_qpos = None
    integrated_steps = 0
    acquisitions = 0
    checked_foot_samples = 0
    original_step = mujoco.mj_step

    def audit_loads(model, data):
        nonlocal checked_foot_samples
        # These are the solver outputs, not the controller's proposed loads.
        assert np.isfinite(data.efc_force).all()
        assert np.all(data.actuator_force <= model.actuator_forcerange[:, 1] + 1e-8)
        assert np.all(data.actuator_force >= model.actuator_forcerange[:, 0] - 1e-8)
        for limb in LIMBS:
            eq_id = model.equality(f"grip_{limb}").id
            if not data.eq_active[eq_id]:
                continue
            rows = ((data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                    & (data.efc_id == eq_id))
            force = data.efc_force[rows]
            assert force.shape == (3,), "an active connect must have three force rows"
            hold = route.hold(assigned_holds[limb])
            assert np.linalg.norm(force) <= hold.max_grip_n + 1e-6
            if limb.endswith("_foot"):
                # This fixture uses upward foot supports. The analytic Coulomb
                # test is independent of the production contact-check helper.
                assert hold.normal == (0, 0, 1)
                assert force[2] >= -1e-6
                assert np.hypot(force[0], force[1]) <= hold.friction * force[2] + 1e-6
                checked_foot_samples += 1

    def traced_step(model, data):
        nonlocal last_integrated_qpos, integrated_steps
        if last_integrated_qpos is not None:
            np.testing.assert_array_equal(data.qpos, last_integrated_qpos,
                                          err_msg="controller changed qpos between integration calls")
        assert not np.any(data.qfrc_applied), "controller bypassed actuator limits with generalized force"
        assert not np.any(data.xfrc_applied), "controller bypassed actuator limits with body force"
        original_step(model, data)
        integrated_steps += 1
        last_integrated_qpos = data.qpos.copy()
        audit_loads(model, data)

    def observe(cm, phase, move_index, current_holds):
        nonlocal acquisitions
        assigned_holds.update(current_holds)
        if cm.data.time > 0 and phase != "failure":
            audit_loads(cm.model, cm.data)
        if phase == "contact":
            acquisitions += 1
            move = route.moves[move_index]
            site = cm.limb_site_pos(move.limb)
            anchor = cm.data.mocap_pos[cm.limb_anchor_mocap_id[move.limb]]
            np.testing.assert_allclose(anchor, site, rtol=0, atol=1e-10,
                                       err_msg="attachment snapped to a remote anchor")

    monkeypatch.setattr(mujoco, "mj_step", traced_step)
    result = attempt_route(route, timestep=0.002, observer=observe)
    assert result.success, result.first_failure
    assert acquisitions == len(route.moves) == 4
    assert integrated_steps > 0 and checked_foot_samples > 0
