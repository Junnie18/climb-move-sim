#!/usr/bin/env python3
"""Serve the interactive route editor and its real MuJoCo simulation API."""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from climbsim.editor_server import DEFAULT_ORIGINS, make_server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="listen address (default: local computer only)")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", "8765")))
    parser.add_argument("--timeout", type=float, default=90, help="seconds before a simulation is terminated")
    parser.add_argument("--static-dir", type=Path, default=ROOT / "docs/demo")
    parser.add_argument("--allow-origin", action="append", default=[], help="additional exact browser origin allowed by CORS")
    args = parser.parse_args()
    server = make_server(args.host, args.port, static_dir=args.static_dir, timeout=args.timeout,
                         allowed_origins=(*DEFAULT_ORIGINS, *args.allow_origin))
    print(f"Ascent editor: http://{args.host}:{server.server_port}", flush=True)
    print("Each run simulates the edited route. Press Ctrl-C to stop.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
