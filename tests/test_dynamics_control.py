"""Independent statics and numerical-boundary checks for the controller."""
from pathlib import Path

import mujoco
import numpy as np

from climbsim import feasibility
from climbsim.route import Route

ROUTES = Path(__file__).resolve().parents[1] / "data" / "routes"


def test_support_feedforward_balances_bodyweight_without_external_force(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.moves = []
    monkeypatch.setattr(feasibility, "INITIAL_SETTLE_S", 0.002)
    snapshots = []

    def observe(cm, phase, move_index, current_holds):
        if phase == "initialization" and cm.data.time > 0:
            torque = feasibility._support_torques(cm, current_holds)
            jacobians = []
            for limb in current_holds:
                jac = np.zeros((3, cm.model.nv))
                mujoco.mj_jacSite(cm.model, cm.data, jac, None, cm.limb_site_id[limb])
                jacobians.append(jac)
            jac = np.vstack(jacobians)
            # Recover the planned forces from the generalized wrench rather
            # than relying on the controller's internal contact vector.
            f = np.linalg.lstsq(jac.T, cm.data.qfrc_bias - torque, rcond=None)[0].reshape(-1, 3)
            snapshots.append((torque.copy(), f, cm.data.qfrc_applied.copy()))

    feasibility.attempt_route(route, observer=observe)
    assert len(snapshots) == 1
    torque, forces, externally_applied = snapshots[0]
    np.testing.assert_allclose(torque[:6], 0.0, atol=1e-5)
    np.testing.assert_allclose(forces.sum(axis=0), [0, 0, 70.0 * 9.81], atol=0.01)
    np.testing.assert_array_equal(externally_applied, 0.0)
    for limb, f in zip(route.start, forces):
        hold = route.hold(route.start[limb])
        assert np.linalg.norm(f) <= 0.85 * hold.max_grip_n + 1e-4
        if limb.endswith("foot"):
            normal = np.asarray(hold.normal)
            n = f @ normal
            assert n > 0
            assert np.linalg.norm(f - n * normal) <= 0.75 * hold.friction * n + 1e-4


def test_observer_and_duration_use_the_compiled_timestep(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.moves = []
    monkeypatch.setattr(feasibility, "INITIAL_SETTLE_S", 0.04)
    sample_times, force_times = [], []

    def observe(cm, phase, move_index, current_holds):
        if cm.data.time > 0:
            sample_times.append(cm.data.time)

    def observe_integration(cm, phase, move_index, current_holds):
        force_times.append(cm.data.time)

    feasibility.attempt_route(route, timestep=0.004, observer=observe,
                              integration_observer=observe_integration)
    np.testing.assert_allclose(sample_times, np.arange(1, 11) * .004, atol=1e-12)
    np.testing.assert_allclose(force_times, sample_times, atol=1e-12)


def test_engine_warning_reset_cannot_be_accepted_as_a_real_step(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    original_step = mujoco.mj_step
    injected = False

    def unstable_step(model, data):
        nonlocal injected
        if not injected:
            data.qvel[0] = 1e20  # finite, but MuJoCo resets it as an invalid velocity
            injected = True
        original_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", unstable_step)
    result = feasibility.attempt_route(route)
    assert result.first_failure.detail["category"] == "numerical_instability"
    assert result.first_failure.detail["warnings"]
    assert not result.fell


def test_invalid_refreshed_state_is_never_emitted_as_a_valid_sample(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    original_forward = mujoco.mj_forward
    emitted_invalid = []

    def invalid_forward(model, data):
        original_forward(model, data)
        if data.time >= 0.01:
            data.qacc[0] = np.nan

    def observe(cm, phase, move_index, current_holds):
        if not np.all(np.isfinite(cm.data.qacc)):
            emitted_invalid.append(phase)

    monkeypatch.setattr(mujoco, "mj_forward", invalid_forward)
    result = feasibility.attempt_route(route, observer=observe)
    assert result.first_failure.detail["category"] == "numerical_instability"
    assert not emitted_invalid
