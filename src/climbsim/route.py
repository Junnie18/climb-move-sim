"""Route and move-sequence representation -- the "block coding" layer.

A route is a wall's worth of holds. A climb attempt is a sequence of moves;
each move is one "block": move exactly one limb to a named hold, with the
other three limbs staying put on whatever they were last assigned to. This
mirrors how a climber (and this project's future drag-and-drop UI) thinks
about a route: one limb repositioning at a time.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

from climbsim.holds import Hold, HoldType

LIMBS = ("left_hand", "right_hand", "left_foot", "right_foot")


@dataclass(frozen=True)
class Move:
    limb: str  # one of LIMBS
    hold: str  # hold name, must be a key in Route.holds

    def __post_init__(self):
        if self.limb not in LIMBS:
            raise ValueError(f"unknown limb '{self.limb}', expected one of {LIMBS}")


@dataclass
class Route:
    name: str
    holds: dict  # name -> Hold
    start: dict  # limb -> hold name, the climber's starting position on the wall
    moves: list = field(default_factory=list)  # list[Move], applied in order after `start`

    def hold(self, name: str) -> Hold:
        return self.holds[name]

    @classmethod
    def from_json(cls, path: str) -> "Route":
        data = json.loads(Path(path).read_text())
        holds = {
            h["name"]: Hold(name=h["name"], pos=tuple(h["pos"]), type=HoldType(h["type"]))
            for h in data["holds"]
        }
        moves = [Move(limb=m["limb"], hold=m["hold"]) for m in data["moves"]]
        return cls(name=data["name"], holds=holds, start=dict(data["start"]), moves=moves)

    def to_json(self, path: str) -> None:
        data = {
            "name": self.name,
            "holds": [dict(name=h.name, pos=list(h.pos), type=h.type.value) for h in self.holds.values()],
            "start": self.start,
            "moves": [dict(limb=m.limb, hold=m.hold) for m in self.moves],
        }
        Path(path).write_text(json.dumps(data, indent=2))
