"""The editor submits fresh physics jobs through a bounded local HTTP service."""

import copy
import importlib.util
import json
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from http.client import HTTPConnection
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def backend():
    assert importlib.util.find_spec("climbsim.editor_server"), "editor HTTP backend is missing"
    from climbsim import editor_server
    return editor_server


@pytest.fixture
def route_data():
    data = json.loads((ROOT / "data/routes/showcase/01_supported_steps.json").read_text())
    data["moves"] = data["moves"][:1]
    return data


@pytest.fixture
def server(tmp_path):
    api = backend()
    static = tmp_path / "public"
    static.mkdir()
    (static / "index.html").write_text("<title>Editor</title>")
    (tmp_path / "private.txt").write_text("private")
    (static / "leak.txt").symlink_to(tmp_path / "private.txt")
    httpd = api.make_server("127.0.0.1", 0, static_dir=static, timeout=30)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield httpd
    httpd.shutdown()
    httpd.server_close()
    thread.join(timeout=5)


def request(server, method="GET", path="/api/health", body=None, headers=None):
    conn = HTTPConnection(*server.server_address, timeout=40)
    try:
        conn.request(method, path, body, headers or {})
        response = conn.getresponse()
        return response.status, dict(response.headers), response.read()
    finally:
        conn.close()


def post(server, body):
    status, headers, data = request(server, "POST", "/api/simulate", json.dumps(body),
                                    {"Content-Type": "application/json"})
    return status, json.loads(data)


def test_service_health_static_files_and_path_boundaries(server):
    status, _, data = request(server)
    assert status == 200
    assert json.loads(data)["engine"] == "MuJoCo"
    assert request(server, path="/")[2] == b"<title>Editor</title>"
    for path in ("/../private.txt", "/%2e%2e/private.txt", "/leak.txt", "/api/unknown"):
        assert request(server, path=path)[0] == 404


def test_actual_edited_routes_produce_different_physics_and_original_definition(server, route_data):
    status, data = post(server, {"route": route_data, "frameEvery": 50})
    assert status == 200
    assert data["trajectory"]["success"] is True
    assert data["trajectory"]["frames"][-1]["t"] > 0
    edited = copy.deepcopy(route_data)
    edited["holds"][4]["pos"][2] = 4
    status, data = post(server, {"route": edited})
    assert status == 200
    trajectory = data["trajectory"]
    assert trajectory["success"] is False
    assert trajectory["outcomes"][-1]["detail"]["category"] == "geometric_reach_limit"
    assert trajectory["routeDefinition"]["holds"][4]["pos"][2] == 4


def test_busy_job_rejects_second_simulation_without_blocking_health(server, route_data):
    with ThreadPoolExecutor(max_workers=1) as pool:
        running = pool.submit(post, server, {"route": route_data})
        deadline = time.monotonic() + 5
        while not server.runner.busy and time.monotonic() < deadline:
            time.sleep(0.005)
        assert server.runner.busy
        assert post(server, {"route": route_data})[0] == 429
        assert request(server)[0] == 200
        assert running.result()[0] == 200
    assert not server.runner.busy


def test_timeout_terminates_child_and_releases_worker_slot(route_data, monkeypatch):
    api = backend()
    launched = []
    actual_popen = subprocess.Popen

    def record_popen(*args, **kwargs):
        process = actual_popen(*args, **kwargs)
        launched.append(process)
        return process

    monkeypatch.setattr(subprocess, "Popen", record_popen)
    runner = api.SimulationRunner(timeout=0.001)
    with pytest.raises(api.SimulationTimeout):
        runner.run(route_data, 20)
    assert not runner.busy
    assert len(launched) == 1
    assert launched[0].poll() is not None


@pytest.mark.parametrize("body", [None, [], {}, {"route": []}, {"route": {}}])
def test_invalid_route_shapes_are_rejected_as_user_input(server, body):
    status, data = post(server, body)
    assert status == 422
    assert isinstance(data["error"], str)


@pytest.mark.parametrize("change", [
    lambda d: d["holds"][0].update(pos=[0, 0, float("nan")]),
    lambda d: d["holds"][0].update(pos=[0, 0, 100]),
    lambda d: d["holds"][0].update(name='x"/><include file="secret"/>'),
    lambda d: d["holds"][0].update(radius=0.000001),
    lambda d: d.update(moves=d["moves"] * 17),
    lambda d: d.update(holds=d["holds"] * 9),
])
def test_unbounded_or_model_injecting_route_parameters_are_rejected(server, route_data, change):
    change(route_data)
    assert post(server, {"route": route_data})[0] in (400, 422)
    assert not server.runner.busy


@pytest.mark.parametrize("frame_every", [0, 1, 101, 2.5, True, "20"])
def test_frame_sampling_is_bounded(server, route_data, frame_every):
    assert post(server, {"route": route_data, "frameEvery": frame_every})[0] == 422


def test_malformed_oversized_and_wrong_content_type_are_rejected(server):
    assert request(server, "POST", "/api/simulate", "{", {"Content-Type": "application/json"})[0] == 400
    assert request(server, "POST", "/api/simulate", "x" * 140_000,
                   {"Content-Type": "application/json"})[0] == 413
    assert request(server, "POST", "/api/simulate", "{}", {"Content-Type": "text/plain"})[0] == 415


def test_cors_allows_project_page_but_rejects_arbitrary_origins(server):
    allowed = {"Origin": "https://junnie18.github.io", "Access-Control-Request-Method": "POST"}
    status, headers, _ = request(server, "OPTIONS", "/api/simulate", headers=allowed)
    assert status == 204
    assert headers["Access-Control-Allow-Origin"] == "https://junnie18.github.io"
    status, headers, _ = request(server, headers={"Origin": "https://unrelated.example"})
    assert status == 403
    assert "Access-Control-Allow-Origin" not in headers


def test_timeout_returns_http_error_not_a_physics_verdict(server, route_data):
    server.runner.timeout = 0.001
    status, data = post(server, {"route": route_data})
    assert status == 504
    assert "trajectory" not in data
    assert not server.runner.busy


def test_decimation_retains_endpoints_and_measured_physics_peaks():
    api = backend()
    payload = {"frames": [{"t": i / 100} for i in range(1001)],
               "metadata": {"peakForceN": [1600, 400, 600, 0]}}
    bounded = api._limit_frames(payload)
    assert len(bounded["frames"]) == 900
    assert bounded["frames"][0]["t"] == 0
    assert bounded["frames"][-1]["t"] == 10
    assert bounded["metadata"]["peakForceN"] == [1600, 400, 600, 0]
    assert bounded["metadata"]["renderingDecimated"] is True
    assert bounded["metadata"]["recordedFrameCount"] == 1001


def test_worker_exception_releases_slot_and_surfaces_error_without_fabricated_result():
    api = backend()
    runner = api.SimulationRunner(timeout=10)
    with pytest.raises(api.SimulationFailed, match="holds"):
        runner.run({"name": "broken"}, 20)
    assert not runner.busy


def test_invalid_content_length_and_chunked_requests_are_rejected(server):
    for headers in ({"Content-Length": "-1"}, {"Content-Length": "invalid"},
                    {"Transfer-Encoding": "chunked", "Content-Length": "0"}):
        headers["Content-Type"] = "application/json"
        assert request(server, "POST", "/api/simulate", headers=headers)[0] == 400


def test_worker_launch_failure_is_json_error_and_does_not_stick_busy(server, route_data, monkeypatch):
    def unavailable_start(*args, **kwargs):
        raise OSError("worker allocation unavailable")

    monkeypatch.setattr(subprocess, "Popen", unavailable_start)
    status, data = post(server, {"route": route_data})
    assert status == 500
    assert "worker" in data["error"].lower()
    assert not server.runner.busy


def test_explicit_worker_module_runs_without_importing_host_main(route_data):
    env = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
    completed = subprocess.run([sys.executable, "-m", "climbsim.editor_worker"],
        input=json.dumps({"route": route_data, "frameEvery": 50}),
        capture_output=True, text=True, env=env, timeout=30)
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result["ok"] is True
    assert result["trajectory"]["success"] is True
    assert result["trajectory"]["routeDefinition"]["moves"][0]["hold"] == "hand_step_0"


def test_child_uses_runtime_import_paths_and_single_thread_blas(route_data, monkeypatch, tmp_path):
    api = backend()
    extra_path = str(tmp_path / "injected-runtime-dependencies")
    monkeypatch.syspath_prepend(extra_path)
    observed = []
    actual_popen = subprocess.Popen

    def record_environment(*args, **kwargs):
        observed.append(kwargs["env"])
        return actual_popen(*args, **kwargs)

    monkeypatch.setattr(subprocess, "Popen", record_environment)
    result = api.SimulationRunner(timeout=30).run(route_data, 50)
    assert result["success"] is True
    assert extra_path in observed[0]["PYTHONPATH"].split(os.pathsep)
    for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        assert observed[0][name] == "1"
