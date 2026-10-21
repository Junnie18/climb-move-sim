#!/usr/bin/env python3
"""Export a real route attempt as JSON for the interactive viewer."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from climbsim.feasibility import DT
from climbsim.route import Route
from climbsim.trajectory_export import (
    BODY_NAMES, _TrajectoryRecorder, build_trajectory_payload, export_static_geometry,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("route_json", help="path to a route JSON file")
    ap.add_argument("out_json", help="path to write the trajectory JSON to")
    ap.add_argument("--frame-every", type=int, default=20, help="sim steps per captured frame")
    ap.add_argument("--timestep", type=float, default=DT, help="physics timestep in seconds")
    args = ap.parse_args()
    route = Route.from_json(args.route_json)
    payload = build_trajectory_payload(route, frame_every=args.frame_every, timestep=args.timestep)
    Path(args.out_json).write_text(json.dumps(payload, allow_nan=False))
    print(f"Wrote {len(payload['frames'])} frames ({args.out_json})")
    print(f"Overall: {'COMPLETED IN SIMULATION' if payload['success'] else 'NOT COMPLETED'}"
          + (" (support failure)" if payload["fell"] else ""))


if __name__ == "__main__":
    main()
