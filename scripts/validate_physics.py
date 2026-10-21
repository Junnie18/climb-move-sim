#!/usr/bin/env python3
"""Reproduce timestep sensitivity evidence for the shipped showcase routes.

This measures numerical consistency, not agreement with real human climbing.
Run the analytic/regression tests separately with python -m pytest.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from export_trajectory import build_trajectory_payload
from climbsim.route import Route

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/validation.json")
    parser.add_argument("--timesteps", nargs="+", type=float, default=[0.002, 0.001, 0.0005])
    args = parser.parse_args()
    records = []
    paths = sorted((ROOT / "data/routes/showcase").glob("*.json"))
    if not paths:
        parser.error("no showcase routes found")
    for path in paths:
        results = []
        for dt in args.timesteps:
            payload = build_trajectory_payload(Route.from_json(path),
                frame_every=max(1, round(0.04 / dt)), timestep=dt)
            failed = next((o for o in payload["outcomes"] if not o["passed"]), None)
            entry = {
                "timestep_s": dt,
                "success": payload["success"],
                "category": failed["detail"].get("category") if failed else "completed",
                "reason": failed["reason"] if failed else "all requested moves completed",
                "end_time_s": payload["frames"][-1]["t"],
                "final_com_m": payload["frames"][-1]["com"],
                "peak_force_n": payload["metadata"]["peakForceN"],
                "minimum_wall_clearance_m": payload["metadata"]["minimumWallClearanceM"],
                "engine": payload["metadata"]["engine"],
            }
            results.append(entry)
            print(f"{path.stem:24} dt={dt:.4f} {entry['category']:28} "
                  f"t={entry['end_time_s']:.3f}s peak={max(entry['peak_force_n']):.1f}N")
        reference = results[-1]
        for result in results:
            result["final_com_delta_from_finest_m"] = float(np.linalg.norm(
                np.asarray(result["final_com_m"]) - np.asarray(reference["final_com_m"])))
            result["end_time_delta_from_finest_s"] = result["end_time_s"] - reference["end_time_s"]
        records.append({"route": str(path.relative_to(ROOT)), "runs": results,
            "classification_agrees": len({(r["success"], r["category"]) for r in results}) == 1})
    evidence = {
        "purpose": "Timestep sensitivity of deterministic synthetic scenarios; not real-world validation",
        "reference": "last timestep in --timesteps (default finest: 0.0005s)",
        "records": records,
        "all_classifications_agree": all(r["classification_agrees"] for r in records),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")
    print(f"Wrote {args.output}")
    if not evidence["all_classifications_agree"]:
        raise SystemExit("Timestep-sensitive classification: inspect validation.json before claiming robustness")


if __name__ == "__main__":
    main()
