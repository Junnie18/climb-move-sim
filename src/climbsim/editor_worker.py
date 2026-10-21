"""Single isolated simulation job. Internal JSON stdin/stdout protocol."""

import contextlib
import json
import sys

from climbsim.editor_server import MAX_BODY_BYTES, MAX_RESPONSE_BYTES, _limit_frames, validate_request
from climbsim.route import Route
from climbsim.trajectory_export import build_trajectory_payload


def main():
    try:
        raw = sys.stdin.buffer.read(MAX_BODY_BYTES + 1)
        if len(raw) > MAX_BODY_BYTES:
            raise ValueError("Route request exceeds 128 KiB.")
        route_data, frame_every = validate_request(json.loads(raw))
        # Ordinary library diagnostics must not be mixed into the JSON response.
        with contextlib.redirect_stdout(sys.stderr):
            trajectory = build_trajectory_payload(Route.from_dict(route_data), frame_every=frame_every)
        result = {"ok": True, "trajectory": _limit_frames(trajectory)}
        encoded = json.dumps(result, allow_nan=False, separators=(",", ":")).encode()
        if len(encoded) > MAX_RESPONSE_BYTES:
            raise ValueError("The simulation result is too large. Use fewer moves or a larger frameEvery.")
    except Exception as exc:
        encoded = json.dumps({"ok": False, "error": f"Simulation could not run: {str(exc)[:300]}"}).encode()
    sys.stdout.buffer.write(encoded)
    sys.stdout.buffer.flush()


if __name__ == "__main__":
    main()
