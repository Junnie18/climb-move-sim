#!/usr/bin/env python3
"""Appends one hypothetical move onto a base route's existing (already-
feasible) end stance and runs it through the real, unmodified physics
pipeline -- the "try a move" feature behind the 3D viewer's picker UI.

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
    ap.add_argument("route_json", help="path to the base route JSON (must already be feasible)")
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
    Path(args.out_json).write_text(json.dumps(payload))

    print(f"Wrote {len(payload['frames'])} frames ({args.out_json})")
    print(f"Overall: {'FEASIBLE' if payload['success'] else 'INFEASIBLE'}" + (" (fell)" if payload["fell"] else ""))
    last = payload["outcomes"][-1]
    print(f"Tested move: [{'PASS' if last['passed'] else 'FAIL'}] {last['move']}: {last['reason']}")


if __name__ == "__main__":
    main()
