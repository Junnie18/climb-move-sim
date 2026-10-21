"""Exports a route attempt's per-frame body transforms + static geometry to
JSON, for a web-based 3D viewer. Uses the real feasibility.attempt_route
pipeline with read-only observers for simulation metadata and executed
force peaks. Physics and move logic are not duplicated in the exporter.

    python scripts/export_trajectory.py data/routes/easy_ladder.json out.json
"""

import mujoco
import numpy as np

from climbsim.feasibility import attempt_route, DT
from climbsim.forces import grip_force
from climbsim.diagnostics import wall_clearance
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
        self._body_ids = None
        self._eq_ids = None
        self._mocap_ids = None
        self._phase = "initialization"
        self._move_index = -1
        self._holds = {}
        self._route = None
        self.peak_force_n = np.zeros(4)
        self.peak_torque_nm = 0.0
        self.minimum_wall_clearance_m = float("inf")

    def bind(self, model):
        if self._body_ids is None:
            self._model = model
            self._body_ids = [model.body(n).id for n in BODY_NAMES]
            self._eq_ids = [model.equality(f"grip_{limb}").id for limb in LIMBS]
            self._mocap_ids = [model.body(f"anchor_{limb}").mocapid[0] for limb in LIMBS]
        return self

    def observe(self, cm, phase, move_index, current_holds):
        """Current-state metadata. Never modify the integration state."""
        self.bind(cm.model)
        self._phase, self._move_index = phase, move_index
        self._holds, self._route = dict(current_holds), cm.route
        clearance = wall_clearance(cm.model, cm.data)["min_clearance_m"]
        if clearance is not None:
            self.minimum_wall_clearance_m = min(self.minimum_wall_clearance_m, clearance)

    def observe_integration(self, cm, phase, move_index, current_holds):
        """Forces actually used by mj_step, before refreshed/terminal solves."""
        self.bind(cm.model)
        forces = [grip_force(cm.model, cm.data, eid) for eid in self._eq_ids]
        self.peak_force_n = np.maximum(self.peak_force_n, np.linalg.norm(forces, axis=1))
        self.peak_torque_nm = max(self.peak_torque_nm, float(np.max(np.abs(cm.data.actuator_force))))

    def update_scene(self, data, camera=None):
        forces = [grip_force(self._model, data, eid) if data.eq_active[eid] else np.zeros(3)
                  for eid in self._eq_ids]
        hold_names = [self._holds.get(limb) for limb in LIMBS]
        utilization = [float(np.linalg.norm(force) / self._route.hold(hn).max_grip_n)
                       if hn is not None and self._route is not None else 0.0
                       for hn, force in zip(hold_names, forces)]
        self._pending = {
            "t": round(float(data.time), 6),
            "pos": [data.xpos[bid].tolist() for bid in self._body_ids],
            "quat": [data.xquat[bid].tolist() for bid in self._body_ids],  # (w, x, y, z)
            "grips": [bool(data.eq_active[eid]) for eid in self._eq_ids],  # order: LIMBS
            "anchors": [data.mocap_pos[mid].tolist() for mid in self._mocap_ids],  # order: LIMBS
            "forces": [f.tolist() for f in forces],
            "com": data.subtree_com[self._model.body("torso").id].tolist(),
            "phase": self._phase,
            "moveIndex": self._move_index,
            "holdNames": hold_names,
            "utilization": utilization,
            "wallClearanceM": wall_clearance(self._model, data)["min_clearance_m"],
        }

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
        {"name": h.name, "type": h.type.value, "pos": list(h.pos), "radius": h.radius,
         "normal": list(h.normal), "friction": h.friction, "maxGripN": h.max_grip_n}
        for h in route.holds.values()
    ]
    wall = {"pos": [0.0, WALL_Y - 0.05, 2.0], "size": [3.0, 0.05, 3.0]}
    return {"bodies": bodies, "holds": holds, "wall": wall,
            "massKg": float(model.body_mass.sum())}


def build_trajectory_payload(route: Route, frame_every: int = 20, *, timestep: float = DT) -> dict:
    """The shared core also used by test_move.py -- runs the real, unmodified
    attempt_route pipeline and shapes its result for the web viewer."""
    recorder = _TrajectoryRecorder(frame_dt=frame_every * timestep)
    result = attempt_route(route, renderer=recorder, frame_every=frame_every,
                           timestep=timestep, observer=recorder.observe,
                           integration_observer=recorder.observe_integration)
    geometry = export_static_geometry(route)
    return {
        "route": route.name,
        "routeDefinition": route.to_dict(),
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
                "detail": o.detail,
            }
            for o in result.outcomes
        ],
        "geometry": geometry,
        "frames": result.frames,
        "metadata": {
            "engine": "MuJoCo " + mujoco.__version__,
            "timestep": timestep,
            "massKg": geometry["massKg"],
            "peakForceN": recorder.peak_force_n.tolist(),
            "peakActuatorTorqueNm": recorder.peak_torque_nm,
            "minimumWallClearanceM": (recorder.minimum_wall_clearance_m
                                      if np.isfinite(recorder.minimum_wall_clearance_m) else None),
            "sampleInterval": timestep * frame_every,
            "limitations": [
                "Simulated outcomes for a simplified body and a specific controller, not measured climbing ability.",
                "Hold coefficients and joint strength limits are illustrative, not individually calibrated.",
                "The wall is visual only; body-wall collision, fingers, fatigue and dynamic jumps are not modeled.",
            ],
        },
    }


