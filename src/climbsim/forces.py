"""Reading real constraint (grip) forces out of the physics engine, and
applying the Coulomb-friction-cone + max-grip-force slip test against a
hold's rated properties.

This is not a bolt-on heuristic: `data.efc_force` is MuJoCo's actual solved
constraint-space force for the active `equality/connect` rows this step, so
`grip_force` below is the real reaction force the solver computed the hold
needs to supply to keep the limb attached, given everything else happening
in the simulation this step (gravity, the other three limbs' loading, any
momentum). We just decompose it against the wall normal and check it against
the hold's rating.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from climbsim.holds import HoldType

WALL_NORMAL = np.array([0.0, 1.0, 0.0])  # points off the wall face, toward the climber

# Only SLOPER is friction-cone-gated: it has no positive edge, so holding on genuinely
# requires enough normal (into-the-wall) force to generate friction, which is the whole
# point of a sloper. Jugs/crimps/footholds are treated as small mechanical anchors --
# a positive incut/edge that a hand or foot can pull on from a range of directions -- so
# they're bounded only by their rated breaking/grip-strength force (max_grip_n). This is
# a simplification of real 3D hand-hold contact (see README "Assumptions").
FRICTION_GATED_TYPES = {HoldType.SLOPER}


@dataclass
class GripCheck:
    limb: str
    hold_name: str
    force: np.ndarray
    normal_n: float
    tangential_n: float
    friction_cone_ok: bool
    within_max_grip: bool

    @property
    def ok(self) -> bool:
        return self.friction_cone_ok and self.within_max_grip


def grip_force(model, data, eq_id: int) -> np.ndarray:
    """World-frame force (N) the equality constraint `eq_id` is currently applying,
    i.e. the force the hold must exert on the limb to keep it in place this step."""
    mask = (data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (data.efc_id == eq_id)
    rows = np.where(mask)[0]
    if rows.size == 0:
        return np.zeros(3)
    return data.efc_force[rows[:3]].copy()


def check_grip(limb: str, hold, model, data, eq_id: int) -> GripCheck:
    f = grip_force(model, data, eq_id)
    normal_n = float(np.dot(f, WALL_NORMAL))
    tangential = f - normal_n * WALL_NORMAL
    tangential_n = float(np.linalg.norm(tangential))
    if hold.type in FRICTION_GATED_TYPES:
        normal_component_for_friction = max(normal_n, 0.0)
        friction_ok = tangential_n <= hold.friction * normal_component_for_friction + 1e-6
    else:
        friction_ok = True
    mag = float(np.linalg.norm(f))
    within_max = mag <= hold.max_grip_n
    return GripCheck(
        limb=limb, hold_name=hold.name, force=f,
        normal_n=normal_n, tangential_n=tangential_n,
        friction_cone_ok=friction_ok, within_max_grip=within_max,
    )
