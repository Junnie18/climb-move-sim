"""Bounded HTTP bridge from the route editor to real MuJoCo simulations.

The static editor is dependency-free. This service performs fresh simulation
work in an isolated child process, so a timed-out solve cannot occupy its one
worker forever. No user-supplied code or filesystem path is executed.
"""

import json
import math
import mimetypes
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

import mujoco

from climbsim.route import Route
from climbsim.pose_preview import build_pose_preview

MAX_BODY_BYTES = 128 * 1024
MAX_FRAMES = 900
MAX_RESPONSE_BYTES = 4_000_000
DEFAULT_ORIGINS = ("https://junnie18.github.io",)


class SimulationBusy(Exception):
    pass


class SimulationTimeout(Exception):
    pass


class SimulationFailed(Exception):
    pass


def validate_request(body):
    """Validate the editor's resource and physical-input bounds before spawn."""
    if not isinstance(body, dict) or not isinstance(body.get("route"), dict):
        raise ValueError("Provide a route object with name, holds, start and moves.")
    raw = body["route"]
    if not isinstance(raw.get("holds"), list) or not 1 <= len(raw["holds"]) <= 40:
        raise ValueError("Use between 1 and 40 holds.")
    if not isinstance(raw.get("moves", []), list) or len(raw.get("moves", [])) > 12:
        raise ValueError("Use at most 12 moves per attempt.")
    route = Route.from_dict(raw)
    if len(route.name) > 120:
        raise ValueError("Route names must be at most 120 characters.")
    for hold in route.holds.values():
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", hold.name):
            raise ValueError("Hold names must use 1–64 letters, digits, underscores or hyphens.")
        x, y, z = hold.pos
        if not (-3 <= x <= 3 and 0 <= y <= 1 and 0.05 <= z <= 5):
            raise ValueError("Hold coordinates must be x: −3 to 3 m, y: 0 to 1 m, z: 0.05 to 5 m.")
        if not (0 <= hold.friction <= 4 and 1 <= hold.max_grip_n <= 10_000
                and 0.005 <= hold.radius <= 0.3):
            raise ValueError("Use friction 0–4, grip capacity 1–10,000 N and hold radius 0.005–0.3 m.")
    frame_every = body.get("frameEvery", 20)
    if isinstance(frame_every, bool) or not isinstance(frame_every, int) or not 5 <= frame_every <= 100:
        raise ValueError("frameEvery must be an integer between 5 and 100.")
    return route.to_dict(), frame_every


def _limit_frames(payload):
    """Reduce rendering data only; the simulation and its peak measurements stay intact."""
    frames = payload["frames"]
    total = len(frames)
    if total > MAX_FRAMES:
        payload["frames"] = [frames[i * (total - 1) // (MAX_FRAMES - 1)]
                             for i in range(MAX_FRAMES)]
    payload["metadata"]["recordedFrameCount"] = total
    payload["metadata"]["renderedFrameCount"] = len(payload["frames"])
    payload["metadata"]["renderingDecimated"] = total > MAX_FRAMES
    return payload


class SimulationRunner:
    """One worker per service process, with an enforceable wall-clock deadline."""

    def __init__(self, timeout=90):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        self.timeout = timeout
        self._lock = threading.Lock()

    @property
    def busy(self):
        return self._lock.locked()

    def run(self, route_data, frame_every):
        if not self._lock.acquire(blocking=False):
            raise SimulationBusy("A simulation is already running. Try again when it finishes.")
        process = None
        try:
            env = os.environ.copy()
            # Serverless hosts may inject installed dependencies into sys.path
            # after interpreter startup. An explicit module worker must inherit
            # those paths without importing the host's __main__ bootstrap.
            import_paths = [path for path in sys.path if isinstance(path, str) and path]
            if env.get("PYTHONPATH"):
                import_paths.extend(env["PYTHONPATH"].split(os.pathsep))
            env["PYTHONPATH"] = os.pathsep.join(dict.fromkeys(import_paths))
            for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
                env[name] = "1"
            request = json.dumps({"route": route_data, "frameEvery": frame_every},
                                 allow_nan=False, separators=(",", ":")).encode()
            # Output is collected on disk, never into an unbounded pipe buffer.
            # The worker caps its response; this reader independently enforces it.
            with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
                try:
                    process = subprocess.Popen([sys.executable, "-m", "climbsim.editor_worker"],
                        stdin=subprocess.PIPE, stdout=output, stderr=errors, env=env,
                        start_new_session=True)
                except (OSError, RuntimeError) as exc:
                    raise SimulationFailed("The simulation worker is unavailable. Try again shortly.") from exc
                try:
                    process.communicate(input=request, timeout=self.timeout)
                except subprocess.TimeoutExpired as exc:
                    raise SimulationTimeout("The simulation exceeded its time limit. Try fewer moves or a simpler starting pose.") from exc
                if process.returncode != 0:
                    raise SimulationFailed(f"The simulation worker exited with code {process.returncode} before producing a result.")
                output.seek(0)
                raw = output.read(MAX_RESPONSE_BYTES + 1)
                if len(raw) > MAX_RESPONSE_BYTES:
                    raise SimulationFailed("The simulation worker produced an oversized result.")
                try:
                    result = json.loads(raw, parse_constant=_reject_json_constant)
                except (ValueError, RecursionError) as exc:
                    raise SimulationFailed("The simulation worker did not produce a valid result.") from exc
                if not isinstance(result, dict) or result.get("ok") is not True:
                    message = result.get("error") if isinstance(result, dict) else None
                    raise SimulationFailed(message[:500] if isinstance(message, str) else "The simulation worker did not produce a result.")
                if not isinstance(result.get("trajectory"), dict):
                    raise SimulationFailed("The simulation worker returned no trajectory.")
                return result["trajectory"]
        finally:
            if process is not None:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=5)
                if process.stdin is not None:
                    process.stdin.close()
            self._lock.release()


def _reject_json_constant(value):
    raise ValueError(f"JSON must not contain {value}.")


def make_handler(static_dir, *, runner=None, allowed_origins=DEFAULT_ORIGINS):
    """Return a normal BaseHTTPRequestHandler class, also usable by serverless hosts."""
    root = Path(static_dir).resolve()
    simulation_runner = runner or SimulationRunner()
    origins = frozenset(allowed_origins)

    class EditorHandler(BaseHTTPRequestHandler):
        runner = simulation_runner
        server_version = "AscentEditor/1.0"

        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def log_message(self, format, *args):
            # Avoid logging routes, submitted data or arbitrary request headers.
            return

        def _origin_allowed(self):
            origin = self.headers.get("Origin")
            if origin is None:
                return True
            host = self.headers.get("Host", "")
            return origin in origins or origin in (f"http://{host}", f"https://{host}")

        def _headers(self, status, content_type, size):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(size))
            self.send_header("Cache-Control", "no-store" if self.path.startswith("/api/") else "no-cache")
            self.send_header("X-Content-Type-Options", "nosniff")
            origin = self.headers.get("Origin")
            if origin is not None and self._origin_allowed():
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.end_headers()

        def _json(self, status, payload):
            encoded = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode()
            if len(encoded) > MAX_RESPONSE_BYTES:
                status = 500
                encoded = b'{"error":"The result is too large. Use fewer moves or a larger frameEvery."}'
            self._headers(status, "application/json; charset=utf-8", len(encoded))
            try:
                self.wfile.write(encoded)
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                pass

        def _check_origin(self):
            if self._origin_allowed():
                return True
            self._json(403, {"error": "This origin is not allowed to use the simulation service."})
            return False

        def do_OPTIONS(self):
            if not self._check_origin():
                return
            if urlsplit(self.path).path not in ("/api/health", "/api/simulate", "/api/preview"):
                self._json(404, {"error": "Not found."})
                return
            self.send_response(204)
            origin = self.headers.get("Origin")
            if origin is not None:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Max-Age", "600")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            if not self._check_origin():
                return
            path = urlsplit(self.path).path
            if path == "/api/health":
                self._json(200, {"status": "ok", "engine": "MuJoCo", "version": mujoco.__version__,
                                 "busy": self.runner.busy})
                return
            try:
                filename = root / ("index.html" if path == "/" else unquote(path).lstrip("/"))
                filename = filename.resolve()
                if not filename.is_relative_to(root) or not filename.is_file():
                    raise OSError("not a public file")
                content_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
                size = filename.stat().st_size
                source = filename.open("rb")
            except (OSError, ValueError):
                self._json(404, {"error": "Not found."})
                return
            with source:
                self._headers(200, content_type, size)
                try:
                    while block := source.read(64 * 1024):
                        self.wfile.write(block)
                except (BrokenPipeError, ConnectionResetError, socket.timeout):
                    pass

        def do_POST(self):
            if not self._check_origin():
                return
            path = urlsplit(self.path).path
            if path not in ("/api/simulate", "/api/preview"):
                self._json(404, {"error": "Not found."})
                return
            if self.headers.get_content_type() != "application/json":
                self._json(415, {"error": "Send application/json."})
                return
            lengths = self.headers.get_all("Content-Length", [])
            if len(lengths) != 1 or self.headers.get("Transfer-Encoding"):
                self._json(400, {"error": "Provide one Content-Length and an unchunked JSON body."})
                return
            try:
                length = int(lengths[0])
            except ValueError:
                self._json(400, {"error": "Content-Length must be a nonnegative integer."})
                return
            if length < 0:
                self._json(400, {"error": "Content-Length must be a nonnegative integer."})
                return
            if length > MAX_BODY_BYTES:
                self._json(413, {"error": "Route request exceeds 128 KiB."})
                return
            try:
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError("Incomplete request body.")
                body = json.loads(raw, parse_constant=_reject_json_constant)
            except socket.timeout:
                self._json(408, {"error": "Timed out reading the request body."})
                return
            except (ValueError, RecursionError):
                self._json(400, {"error": "Send valid finite JSON."})
                return
            try:
                route_data, frame_every = validate_request(body)
            except (ValueError, TypeError, OverflowError) as exc:
                self._json(422, {"error": str(exc)[:500]})
                return
            if path == "/api/preview":
                try:
                    pose = build_pose_preview(Route.from_dict(route_data))
                except Exception:
                    self._json(500, {"error": "The pose solver could not produce a preview. Try a different stance."})
                else:
                    self._json(200, {"preview": pose})
                return
            try:
                trajectory = self.runner.run(route_data, frame_every)
            except SimulationBusy as exc:
                self._json(429, {"error": str(exc)})
            except SimulationTimeout as exc:
                self._json(504, {"error": str(exc)})
            except SimulationFailed as exc:
                self._json(500, {"error": str(exc)})
            else:
                self._json(200, {"trajectory": trajectory})

    return EditorHandler


def make_server(host="127.0.0.1", port=8765, *, static_dir, timeout=90, allowed_origins=DEFAULT_ORIGINS):
    runner = SimulationRunner(timeout=timeout)
    handler = make_handler(static_dir, runner=runner, allowed_origins=allowed_origins)
    server = ThreadingHTTPServer((host, port), handler)
    server.runner = runner
    server.daemon_threads = True
    return server
