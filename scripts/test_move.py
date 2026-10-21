#!/usr/bin/env python3
"""Append a hypothetical move and rerun the whole route through the simulator.
Earlier failures prevent the appended move from being attempted.

    python scripts/test_move.py data/routes/easy_ladder.json left_hand lh1 out.json
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from climbsim.route import Move, Route
from export_trajectory import build_trajectory_payload


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("route_json", help="path to the base route JSON")
    ap.add_argument("limb", choices=["left_hand", "right_hand", "left_foot", "right_foot"])
    ap.add_argument("hold", help="target hold name, must exist in the base route's holds")
    ap.add_argument("out_json", help="path to write the trajectory JSON to")
    args = ap.parse_args()

    route = Route.from_json(args.route_json)
    if args.hold not in route.holds:
        sys.exit(f"error: '{args.hold}' is not a hold on route '{route.name}' "
                  f"(available: {', '.join(sorted(route.holds))})")

    route.moves = [*route.moves, Move(limb=args.limb, hold=args.hold)]
    payload = build_trajectory_payload(route)
    payload["route"] = f"{route.name} + {args.limb} -> {args.hold}"
    Path(args.out_json).write_text(json.dumps(payload, allow_nan=False))

    print(f"Wrote {len(payload['frames'])} frames ({args.out_json})")
    print(f"Overall: {'COMPLETED IN SIMULATION' if payload['success'] else 'NOT COMPLETED'}")
    last = payload["outcomes"][-1]
    print(f"Last outcome: [{'PASS' if last['passed'] else 'FAIL'}] {last['move']}: {last['reason']}")
    if len(payload["outcomes"]) < len(route.moves):
        print("The appended move was not attempted because the base route stopped earlier.")


if __name__ == "__main__":
    main()
