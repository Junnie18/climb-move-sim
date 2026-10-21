"""Illustrative hold properties and configurable contact surfaces.

``normal`` points out of the support surface toward the limb. The wall frame
currently coincides with world axes: x across, y away from the wall, z up.
A foot chip defaults to an upward supporting surface; other holds default to
the outward wall normal. Supply an explicit normal for inclined surfaces.

``friction`` is a dimensionless Coulomb coefficient; ``max_grip_n`` caps the
total transmitted force. Defaults are illustrative, uncalibrated parameters,
not measured hold ratings or validated human strength limits. Overrides allow
measured inputs without changing the type defaults.
"""

from dataclasses import dataclass, field
from enum import Enum
import math
from numbers import Real


class HoldType(str, Enum):
    JUG = "jug"          # big, positive incut hold -- easy to hold from any angle
    CRIMP = "crimp"       # small edge, high friction but low grip-force ceiling
    SLOPER = "sloper"      # rounded, low friction, relies on body position/normal force
    FOOTHOLD = "foothold"   # small foot chip/edge with unilateral support


HOLD_PROPERTIES = {
    HoldType.JUG: dict(friction=1.3, max_grip_n=1400.0, radius=0.06),
    HoldType.CRIMP: dict(friction=1.1, max_grip_n=550.0, radius=0.03),
    HoldType.SLOPER: dict(friction=0.55, max_grip_n=900.0, radius=0.075),
    HoldType.FOOTHOLD: dict(friction=1.0, max_grip_n=1200.0, radius=0.035),
}


def _finite_vector(value, name):
    try:
        vector = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{name} must contain three finite coordinates") from exc
    if len(vector) != 3 or any(
        isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v)
        for v in vector
    ):
        raise ValueError(f"{name} must contain three finite coordinates")
    return tuple(float(v) for v in vector)


@dataclass(frozen=True)
class Hold:
    name: str
    pos: tuple  # (x, y, z) meters, in wall-local frame (see model.py for wall pose)
    type: HoldType
    normal: tuple | None = field(default=None, kw_only=True)
    friction_override: float | None = field(default=None, kw_only=True)
    max_grip_n_override: float | None = field(default=None, kw_only=True)
    radius_override: float | None = field(default=None, kw_only=True)

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("hold name must be a nonempty string")
        try:
            object.__setattr__(self, "type", HoldType(self.type))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"unknown hold type {self.type!r}") from exc
        object.__setattr__(self, "pos", _finite_vector(self.pos, "pos"))
        default_normal = (0.0, 0.0, 1.0) if self.type == HoldType.FOOTHOLD else (0.0, 1.0, 0.0)
        normal = _finite_vector(self.normal if self.normal is not None else default_normal, "normal")
        # Scale first so finite, very large/small normal vectors normalize safely.
        scale = max(abs(v) for v in normal)
        if scale == 0:
            raise ValueError("normal must be nonzero")
        scaled = tuple(v / scale for v in normal)
        norm = math.hypot(*scaled)
        object.__setattr__(self, "normal", tuple(v / norm for v in scaled))
        for name in ("friction_override", "max_grip_n_override", "radius_override"):
            value = getattr(self, name)
            if value is None:
                continue
            valid_number = isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
            if not valid_number or value < 0 or (name != "friction_override" and value == 0):
                qualifier = "nonnegative" if name == "friction_override" else "positive"
                raise ValueError(f"{name} must be finite and {qualifier}")
            object.__setattr__(self, name, float(value))

    @property
    def friction(self) -> float:
        return self.friction_override if self.friction_override is not None else HOLD_PROPERTIES[self.type]["friction"]

    @property
    def max_grip_n(self) -> float:
        return self.max_grip_n_override if self.max_grip_n_override is not None else HOLD_PROPERTIES[self.type]["max_grip_n"]

    @property
    def radius(self) -> float:
        return self.radius_override if self.radius_override is not None else HOLD_PROPERTIES[self.type]["radius"]
