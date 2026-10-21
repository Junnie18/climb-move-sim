"""Evaluate solved attachment forces against a simplified contact model.

MuJoCo's bilateral connect equality supplies the force needed to keep an
attached limb in place. This module checks that force against a unilateral
Coulomb cone for feet and ungraspable surfaces, plus an effective force cap.
This is an attachment feasibility test, not a distributed hand/shoe contact
simulation: it does not solve sliding or contact pressure distributions.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from climbsim.holds import HoldType
from climbsim.route import LIMBS

# Positive jug/crimp HAND grips retain a force-capped grasp approximation.
# Shoes cannot pull on even an incut hold in this model (no toe/heel hooks).
FRICTION_GATED_TYPES = {HoldType.SLOPER, HoldType.FOOTHOLD}
FORCE_TOLERANCE_N = 1e-6  # numerical roundoff, not an adhesive force allowance


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
    """World-frame connect force (N) applied to site1 by site2.

The climber model always places the limb at site1 and the anchor at site2.
Connect constraints have three world-axis translational rows; their sign is
checked against a freely suspended mass with known gravity in the tests.
An inactive constraint has no rows and exerts zero force.
"""
    mask = (data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (data.efc_id == eq_id)
    rows = np.where(mask)[0]
    if rows.size == 0:
        return np.zeros(3)
    return data.efc_force[rows[:3]].copy()


def check_grip(limb: str, hold, model, data, eq_id: int) -> GripCheck:
    if limb not in LIMBS:
        raise ValueError(f"unknown limb {limb!r}, expected one of {LIMBS}")
    f = grip_force(model, data, eq_id)
    normal = np.asarray(hold.normal)
    normal_n = float(np.dot(f, normal))
    tangential = f - normal_n * normal
    tangential_n = float(np.linalg.norm(tangential))
    if limb.endswith("_foot") or hold.type in FRICTION_GATED_TYPES:
        friction_ok = (
            normal_n >= -FORCE_TOLERANCE_N
            and tangential_n <= hold.friction * max(normal_n, 0.0) + FORCE_TOLERANCE_N
        )
    else:
        friction_ok = True
    mag = float(np.linalg.norm(f))
    within_max = mag <= hold.max_grip_n
    return GripCheck(
        limb=limb, hold_name=hold.name, force=f,
        normal_n=normal_n, tangential_n=tangential_n,
        friction_cone_ok=friction_ok, within_max_grip=within_max,
    )
