#!/usr/bin/env python3
"""Exports a route attempt's per-frame body transforms + static geometry to
JSON, for a web-based 3D viewer. Reuses feasibility.attempt_route completely
unchanged: it accepts a "renderer factory" duck-typed like
render.make_renderer()'s (bind()/update_scene()/render()) and calls it the
same way whether it's recording pixels or, here, body world transforms --
so the physics/move logic is not touched or duplicated.

    python scripts/export_trajectory.py data/routes/easy_ladder.json out.json
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco
import numpy as np

from climbsim.feasibility import attempt_route, DT
from climbsim.model import ClimberModel, build_mjcf, WALL_Y
from climbsim.route import LIMBS, Route

BODY_NAMES = [
    "torso",
    "upper_arm_L", "forearm_L", "hand_L",
    "upper_arm_R", "forearm_R", "hand_R",
    "thigh_L", "shank_L", "foot_L",
    "thigh_R", "shank_R", "foot_R",
]

_GEOM_TYPE_NAMES = {
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
}


class _TrajectoryRecorder:
    """Duck-types render.make_renderer()'s factory: attempt_route calls
    `renderer.bind(model)` once (lazily, on first capture) then, per captured
    step, `.update_scene(data, camera=...)` followed by `.render()` -- whose
    return value it appends to result.frames. We return this instance from
    bind() and record real body/mocap/equality state instead of pixels."""

    def __init__(self, frame_dt: float):
        self.frame_dt = frame_dt
        self._t = 0.0
        self._body_ids = None
        self._eq_ids = None
        self._mocap_ids = None

    def bind(self, model):
        if self._body_ids is None:
            self._body_ids = [model.body(n).id for n in BODY_NAMES]
            self._eq_ids = [model.equality(f"grip_{limb}").id for limb in LIMBS]
            self._mocap_ids = [model.body(f"anchor_{limb}").mocapid[0] for limb in LIMBS]
        return self

    def update_scene(self, data, camera=None):
        self._pending = {
            "t": round(self._t, 4),
            "pos": [data.xpos[bid].tolist() for bid in self._body_ids],
            "quat": [data.xquat[bid].tolist() for bid in self._body_ids],  # (w, x, y, z)
            "grips": [bool(data.eq_active[eid]) for eid in self._eq_ids],  # order: LIMBS
            "anchors": [data.mocap_pos[mid].tolist() for mid in self._mocap_ids],  # order: LIMBS
        }
        self._t += self.frame_dt

    def render(self):
        return self._pending


def export_static_geometry(route: Route) -> dict:
    """A fresh, never-stepped ClimberModel purely to read compiled body-local
    geometry (capsule/sphere/box pos/quat/size) -- cheap (just an XML
    compile), and keeps this script from having to duplicate model.py's
    hardcoded segment layout."""
    model = mujoco.MjModel.from_xml_string(build_mjcf(route))
    bodies = []
    for name in BODY_NAMES:
        bid = model.body(name).id
        geoms = []
        for gid in range(model.ngeom):
            if model.geom_bodyid[gid] != bid:
                continue
            gtype = _GEOM_TYPE_NAMES.get(int(model.geom_type[gid]))
            if gtype is None:
                continue
            geoms.append({
                "type": gtype,
                "pos": model.geom_pos[gid].tolist(),
                "quat": model.geom_quat[gid].tolist(),  # (w, x, y, z), body-local
                "size": model.geom_size[gid].tolist(),
                "rgba": model.geom_rgba[gid].tolist(),
            })
        bodies.append({"name": name, "geoms": geoms})

    holds = [
        {"name": h.name, "type": h.type.value, "pos": list(h.pos), "radius": h.radius}
        for h in route.holds.values()
    ]
    wall = {"pos": [0.0, WALL_Y - 0.05, 2.0], "size": [3.0, 0.05, 3.0]}
    return {"bodies": bodies, "holds": holds, "wall": wall}


def build_trajectory_payload(route: Route, frame_every: int = 20) -> dict:
    """The shared core also used by test_move.py -- runs the real, unmodified
    attempt_route pipeline and shapes its result for the web viewer."""
    recorder = _TrajectoryRecorder(frame_dt=frame_every * DT)
    result = attempt_route(route, renderer=recorder, frame_every=frame_every)
    return {
        "route": route.name,
        "success": result.success,
        "fell": result.fell,
        "bodyNames": BODY_NAMES,
        "limbs": list(LIMBS),
        "outcomes": [
            {
                "passed": o.passed,
                "phase": o.phase,
                "move": (f"{o.move.limb} -> {o.move.hold}" if o.move else "(starting stance)"),
                "reason": o.reason,
            }
            for o in result.outcomes
        ],
        "geometry": export_static_geometry(route),
        "frames": result.frames,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("route_json", help="path to a route JSON file")
    ap.add_argument("out_json", help="path to write the trajectory JSON to")
    ap.add_argument("--frame-every", type=int, default=20, help="sim steps per captured frame")
    args = ap.parse_args()

    route = Route.from_json(args.route_json)
    payload = build_trajectory_payload(route, frame_every=args.frame_every)
    Path(args.out_json).write_text(json.dumps(payload))
    print(f"Wrote {len(payload['frames'])} frames ({args.out_json})")
    print(f"Overall: {'FEASIBLE' if payload['success'] else 'INFEASIBLE'}" + (" (fell)" if payload["fell"] else ""))


if __name__ == "__main__":
    main()
