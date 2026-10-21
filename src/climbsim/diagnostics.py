"""Read-only geometry diagnostics; these do not add wall collision forces."""

import mujoco
import numpy as np


def wall_clearance(model, data, wall_y: float = 0.0) -> dict:
    """Minimum surface clearance to the infinite plane y=``wall_y``.

    Positive clearance is in front of the plane; negative clearance means a
    shape crosses it. Uses current ``geom_xpos``/``geom_xmat`` caches, which
    callers must refresh with MuJoCo kinematics/forward after changing qpos.
    Sphere, capsule and box support extents are exact for the model's shapes;
    the shapes themselves are coarse body approximations. Static and mocap
    geometry is excluded. Jointless segments of an articulated body remain
    included. Unsupported dynamic shapes are counted rather than certified.

    This is a diagnostic against an infinite plane, not collision detection
    against the finite wall box and not a collision response or body repair.
    """
    result = dict(min_clearance_m=None, body=None, geom=None, geom_id=None,
                  unsupported_geom_count=0)
    dynamic = np.zeros(model.nbody, dtype=bool)
    for body_id in range(1, model.nbody):
        dynamic[body_id] = bool(model.body_jntnum[body_id]) or dynamic[model.body_parentid[body_id]]
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        if not dynamic[body_id]:
            continue
        kind = model.geom_type[geom_id]
        size = model.geom_size[geom_id]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            extent_y = size[0]
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            extent_y = size[0] + abs(rotation[1, 2]) * size[1]
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            extent_y = np.dot(np.abs(rotation[1]), size)
        else:
            result["unsupported_geom_count"] += 1
            continue
        clearance = float(data.geom_xpos[geom_id, 1] - extent_y - wall_y)
        if result["min_clearance_m"] is None or clearance < result["min_clearance_m"]:
            result.update(min_clearance_m=clearance, body=model.body(body_id).name or None,
                          geom=model.geom(geom_id).name or None, geom_id=geom_id)
    return result
