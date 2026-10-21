#!/usr/bin/env python3
"""CLI: simulate a route under the model's assumptions and controller,
and optionally render the attempt to MP4.

    python scripts/run_route.py data/routes/easy_ladder.json
    python scripts/run_route.py data/routes/dyno_reach.json --render out.mp4
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from climbsim.route import Route
from climbsim.feasibility import attempt_route


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("route_json", help="path to a route JSON file")
    ap.add_argument("--render", metavar="OUT.mp4", help="render the attempt to this MP4 path")
    ap.add_argument("--fps", type=float, help="video frame rate (default: real simulation time)")
    ap.add_argument("--frame-every", type=int, default=15, help="sim steps per captured frame")
    ap.add_argument("--timestep", type=float, default=0.002, help="physics step in seconds")
    args = ap.parse_args()

    route = Route.from_json(args.route_json)
    renderer = None
    if args.render:
        from climbsim.render import make_renderer
        renderer = make_renderer()

    result = attempt_route(route, renderer=renderer, frame_every=args.frame_every,
                           timestep=args.timestep)

    print(f"Route: {route.name}")
    for outcome in result.outcomes:
        status = "PASS" if outcome.passed else "FAIL"
        move_desc = f"{outcome.move.limb} -> {outcome.move.hold}" if outcome.move else "(starting stance)"
        print(f"  [{status}] ({outcome.phase}) {move_desc}: {outcome.reason}")
    print(f"\nOverall: {'COMPLETED IN SIMULATION' if result.success else 'NOT COMPLETED'}"
          + (" (support failure)" if result.fell else ""))

    if args.render:
        from climbsim.render import write_mp4
        write_mp4(result.frames, args.render, fps=args.fps or 1 / (args.timestep * args.frame_every))
        print(f"Wrote {len(result.frames)} frames to {args.render}")

    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
