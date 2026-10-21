"""Exported evidence must use simulation time and solver forces, not a UI clock."""
import importlib.util
from pathlib import Path

import mujoco
import numpy as np
import pytest

from climbsim.model import ClimberModel
from climbsim.route import LIMBS, Route

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("export_trajectory", ROOT / "scripts/export_trajectory.py")
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)


def test_recorded_time_is_the_physics_clock_and_includes_loads():
    route = Route.from_json(ROOT / "data/routes/easy_ladder.json")
    cm = ClimberModel.build(route)
    cm.init_qpos_standing()
    cm.data.time = 1.234
    recorder = export._TrajectoryRecorder(frame_dt=0.04)
    recorder.bind(cm.model)
    recorder.update_scene(cm.data)
    frame = recorder.render()
    assert frame["t"] == pytest.approx(1.234)
    assert np.asarray(frame["forces"]).shape == (4, 3)
    assert np.asarray(frame["com"]).shape == (3,)
    assert np.isfinite(frame["com"]).all()


def test_static_export_contains_contact_parameters_and_preserves_limb_order():
    route = Route.from_json(ROOT / "data/routes/easy_ladder.json")
    geometry = export.export_static_geometry(route)
    foot = next(h for h in geometry["holds"] if h["name"] == "lf0")
    assert foot["normal"] == [0, 0, 1]
    assert foot["maxGripN"] == 1200
    assert foot["friction"] == 1


def test_recording_does_not_change_simulation_state():
    route = Route.from_json(ROOT / "data/routes/easy_ladder.json")
    cm = ClimberModel.build(route)
    cm.init_qpos_standing()
    before = (cm.data.qpos.copy(), cm.data.qvel.copy(), cm.data.time)
    recorder = export._TrajectoryRecorder(frame_dt=0.04)
    recorder.bind(cm.model)
    recorder.observe(cm, "transition", 0, dict(route.start))
    recorder.update_scene(cm.data)
    frame = recorder.render()
    assert frame["phase"] == "transition"
    assert frame["moveIndex"] == 0
    assert frame["holdNames"] == [route.start[x] for x in LIMBS]
    assert np.array_equal(cm.data.qpos, before[0])
    assert np.array_equal(cm.data.qvel, before[1])
    assert cm.data.time == before[2]


def test_exported_attempt_has_finite_monotonic_frames_and_final_verdict():
    import json
    route = Route.from_json(ROOT / "data/routes/dyno_reach.json")
    payload = export.build_trajectory_payload(route, frame_every=25, timestep=0.001)
    assert payload["frames"]
    times = [frame["t"] for frame in payload["frames"]]
    assert all(b >= a for a, b in zip(times, times[1:]))
    assert times[-1] > 0
    assert not payload["success"]
    assert any(not outcome["passed"] for outcome in payload["outcomes"])
    assert payload["metadata"]["timestep"] == 0.001
    assert payload["metadata"]["massKg"] == pytest.approx(70)
    json.dumps(payload, allow_nan=False)


def test_peak_telemetry_matches_forces_used_by_integration(monkeypatch):
    """Preview/terminal forward solves must not contaminate executed peaks."""
    from climbsim.forces import grip_force
    actual_step = mujoco.mj_step
    force_peaks = np.zeros(4)
    torque_peaks = []

    def measured_step(model, data):
        nonlocal force_peaks
        actual_step(model, data)
        vectors = [grip_force(model, data, model.equality(f"grip_{limb}").id) for limb in LIMBS]
        force_peaks = np.maximum(force_peaks, np.linalg.norm(vectors, axis=1))
        torque_peaks.append(float(np.max(np.abs(data.actuator_force))))

    monkeypatch.setattr(mujoco, "mj_step", measured_step)
    route = Route.from_json(ROOT / "data/routes/dyno_reach.json")
    payload = export.build_trajectory_payload(route)
    np.testing.assert_allclose(payload["metadata"]["peakForceN"], force_peaks, atol=1e-9)
    assert payload["metadata"]["peakActuatorTorqueNm"] == pytest.approx(max(torque_peaks))


def test_initial_numerical_failure_exports_without_invalid_frames(monkeypatch):
    import json
    actual_forward = mujoco.mj_forward

    def invalid_forward(model, data):
        actual_forward(model, data)
        data.qacc[0] = np.nan

    monkeypatch.setattr(mujoco, "mj_forward", invalid_forward)
    route = Route.from_json(ROOT / "data/routes/dyno_reach.json")
    payload = export.build_trajectory_payload(route)
    assert not payload["success"]
    assert payload["frames"] == []
    assert payload["outcomes"][0]["detail"]["category"] == "numerical_instability"
    assert payload["metadata"]["massKg"] == pytest.approx(70)
    json.dumps(payload, allow_nan=False)
