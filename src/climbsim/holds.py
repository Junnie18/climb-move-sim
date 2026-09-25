"""Hold types and their grip properties.

`friction` is a Coulomb coefficient applied between the gripping limb and the
hold (governs how much tangential/shear force the grip can carry per unit of
normal/pull-in force -- e.g. a downward-pulling underclhing jug vs. an open
sloper you can barely apply normal force to). `max_grip_n` is a hard cap on
the total force the hold can transmit before it either breaks (a chipped
crimp) or the climber's grip fails (open-hand strength runs out on a sloper)
-- modeled as a slip once the constraint force magnitude exceeds this, which
is checked independently of the friction-cone test. Numbers are illustrative
or engineering estimates for climbing-wall holds, not measured from real
hardware -- see README "Assumptions".
"""

from dataclasses import dataclass
from enum import Enum


class HoldType(str, Enum):
    JUG = "jug"          # big, positive incut hold -- easy to hold from any angle
    CRIMP = "crimp"       # small edge, high friction but low grip-force ceiling
    SLOPER = "sloper"      # rounded, low friction, relies on body position/normal force
    FOOTHOLD = "foothold"   # small foot chip/edge -- treated like a shallow crimp for feet


HOLD_PROPERTIES = {
    HoldType.JUG: dict(friction=1.3, max_grip_n=1400.0, radius=0.06),
    HoldType.CRIMP: dict(friction=1.1, max_grip_n=550.0, radius=0.03),
    HoldType.SLOPER: dict(friction=0.55, max_grip_n=900.0, radius=0.075),
    # A stiff climbing shoe edging on a solid foot chip routinely takes well over 1x
    # bodyweight in a single-foot stance (the other foot momentarily unweighted while it
    # repositions) -- raised from an initial 800N after that exact scenario (~1.5x a 70kg
    # climber's bodyweight, ~1020N, during a normal, unhurried foot-swap on the
    # `easy_ladder` demo route) came out physically plausible but tripped too
    # conservative a cap.
    HoldType.FOOTHOLD: dict(friction=1.0, max_grip_n=1200.0, radius=0.035),
}


@dataclass(frozen=True)
class Hold:
    name: str
    pos: tuple  # (x, y, z) meters, in wall-local frame (see model.py for wall pose)
    type: HoldType

    @property
    def friction(self) -> float:
        return HOLD_PROPERTIES[self.type]["friction"]

    @property
    def max_grip_n(self) -> float:
        return HOLD_PROPERTIES[self.type]["max_grip_n"]

    @property
    def radius(self) -> float:
        return HOLD_PROPERTIES[self.type]["radius"]
