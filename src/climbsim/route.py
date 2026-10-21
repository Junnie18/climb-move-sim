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

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("route name must be a nonempty string")
        if not isinstance(self.holds, dict) or not self.holds:
            raise ValueError("route holds must be a nonempty name-to-Hold mapping")
        for key, hold in self.holds.items():
            if not isinstance(hold, Hold) or key != hold.name:
                raise ValueError(f"hold key {key!r} must match its Hold name")
        if not isinstance(self.start, dict) or set(self.start) != set(LIMBS):
            raise ValueError(f"start must assign exactly these limbs: {LIMBS}")
        for limb, name in self.start.items():
            if not isinstance(name, str) or name not in self.holds:
                raise ValueError(f"start for {limb} references unknown hold {name!r}")
        for move in self.moves:
            if not isinstance(move, Move):
                raise ValueError("moves must contain Move objects")
            if not isinstance(move.hold, str) or move.hold not in self.holds:
                raise ValueError(f"move references unknown hold {move.hold!r}")

    def hold(self, name: str) -> Hold:
        return self.holds[name]

    @classmethod
    def from_json(cls, path: str) -> "Route":
        data = json.loads(Path(path).read_text())
        holds = {}
        for h in data["holds"]:
            hold = Hold(name=h["name"], pos=h["pos"], type=HoldType(h["type"]),
                        normal=h.get("normal"), friction_override=h.get("friction"),
                        max_grip_n_override=h.get("max_grip_n"), radius_override=h.get("radius"))
            if hold.name in holds:
                raise ValueError(f"duplicate hold name {hold.name!r}")
            holds[hold.name] = hold
        moves = [Move(limb=m["limb"], hold=m["hold"]) for m in data["moves"]]
        return cls(name=data["name"], holds=holds, start=dict(data["start"]), moves=moves)

    def to_json(self, path: str) -> None:
        holds = []
        for hold in self.holds.values():
            item = dict(name=hold.name, pos=list(hold.pos), type=hold.type.value,
                        normal=list(hold.normal))
            for prop in ("friction", "max_grip_n", "radius"):
                override = getattr(hold, f"{prop}_override")
                if override is not None:
                    item[prop] = override
            holds.append(item)
        data = {
            "name": self.name,
            "holds": holds,
            "start": self.start,
            "moves": [dict(limb=m.limb, hold=m.hold) for m in self.moves],
        }
        Path(path).write_text(json.dumps(data, indent=2))
