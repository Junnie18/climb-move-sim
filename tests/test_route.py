"""Routes reject broken references and retain explicit physical inputs on disk."""

import json

import pytest

from climbsim.holds import Hold, HoldType
from climbsim.route import LIMBS, Move, Route


def start_on(name="h"):
    return dict.fromkeys(LIMBS, name)


def test_route_roundtrip_preserves_physical_overrides(tmp_path):
    hold = Hold("h", (1, 2, 3), HoldType.JUG, normal=(0, 0, 1),
                friction_override=0.4, max_grip_n_override=200, radius_override=0.025)
    route = Route("custom", {"h": hold}, start_on(), [Move("left_foot", "h")])
    path = tmp_path / "route.json"
    route.to_json(path)
    encoded = json.loads(path.read_text())["holds"][0]
    assert encoded["normal"] == [0, 0, 1]
    assert encoded["friction"] == 0.4
    assert encoded["max_grip_n"] == 200
    assert encoded["radius"] == 0.025
    assert Route.from_json(path) == route


def test_legacy_route_json_uses_defaults_without_explicit_overrides(tmp_path):
    path = tmp_path / "route.json"
    path.write_text(json.dumps({"name": "old", "holds": [
        {"name": "h", "pos": [0, 0, 0], "type": "foothold"}],
        "start": start_on(), "moves": []}))
    route = Route.from_json(path)
    assert route.hold("h").normal == (0, 0, 1)
    assert route.hold("h").friction_override is None


@pytest.mark.parametrize("start", [
    {"left_hand": "h"}, {**start_on(), "tail": "h"},
    {**start_on(), "right_foot": "missing"},
])
def test_route_rejects_incomplete_invalid_or_dangling_start(start):
    with pytest.raises(ValueError, match="start"):
        Route("bad", {"h": Hold("h", (0, 0, 0), HoldType.JUG)}, start)


def test_route_rejects_dangling_move_destination():
    with pytest.raises(ValueError, match="missing"):
        Route("bad", {"h": Hold("h", (0, 0, 0), HoldType.JUG)},
              start_on(), [Move("left_hand", "missing")])


def test_route_rejects_hold_key_name_mismatch():
    with pytest.raises(ValueError, match="name"):
        Route("bad", {"h": Hold("different", (0, 0, 0), HoldType.JUG)}, start_on())


def test_route_json_rejects_duplicate_names_before_they_are_overwritten(tmp_path):
    path = tmp_path / "route.json"
    path.write_text(json.dumps({"name": "bad", "holds": [
        {"name": "h", "pos": [0, 0, 0], "type": "jug"},
        {"name": "h", "pos": [1, 0, 0], "type": "sloper"}],
        "start": start_on(), "moves": []}))
    with pytest.raises(ValueError, match="duplicate"):
        Route.from_json(path)


def test_dictionary_roundtrip_is_detached_and_preserves_contact_inputs():
    route = Route("custom", {"h": Hold("h", (1, 2, 3), HoldType.SLOPER,
                  normal=(0, 0, 2), friction_override=0.4,
                  max_grip_n_override=200, radius_override=0.025)}, start_on())
    encoded = route.to_dict()
    assert encoded["holds"][0] == {"name": "h", "pos": [1, 2, 3],
        "type": "sloper", "normal": [0, 0, 1], "friction": 0.4,
        "max_grip_n": 200, "radius": 0.025}
    assert Route.from_dict(encoded) == route
    encoded["start"]["left_hand"] = "different"
    assert route.start["left_hand"] == "h"


@pytest.mark.parametrize("value", [None, [], {}, {"name": "x", "holds": None},
    {"name": "x", "holds": [None]},
    {"name": "x", "holds": [{"name": "h", "pos": [0, 0, 0], "type": "jug"}],
     "start": start_on(), "moves": [None]},
])
def test_dictionary_validation_returns_actionable_value_errors(value):
    with pytest.raises(ValueError):
        Route.from_dict(value)
