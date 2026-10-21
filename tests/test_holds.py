"""Validate physical inputs before they enter MuJoCo or force decomposition."""

import math

import pytest

from climbsim.holds import Hold, HoldType


@pytest.mark.parametrize("pos", [(0, 0), (0, 0, 0, 0), (0, math.nan, 0), (0, math.inf, 0)])
def test_hold_rejects_invalid_coordinates(pos):
    with pytest.raises(ValueError, match="pos"):
        Hold("h", pos, HoldType.JUG)


@pytest.mark.parametrize("normal", [(0, 0, 0), (0, 0), (math.nan, 0, 1), (0, math.inf, 0)])
def test_hold_rejects_invalid_support_normal(normal):
    with pytest.raises(ValueError, match="normal"):
        Hold("h", (0, 0, 0), HoldType.SLOPER, normal=normal)


@pytest.mark.parametrize("field, value", [
    ("friction_override", -0.1), ("friction_override", math.nan),
    ("max_grip_n_override", 0), ("max_grip_n_override", math.inf),
    ("radius_override", -0.1), ("radius_override", math.nan),
])
def test_hold_rejects_invalid_physical_overrides(field, value):
    with pytest.raises(ValueError, match=field):
        Hold("h", (0, 0, 0), HoldType.JUG, **{field: value})


def test_hold_converts_normal_to_unit_vector_and_coordinates_to_immutable_tuple():
    pos = [1, 2, 3]
    hold = Hold("h", pos, HoldType.SLOPER, normal=(0, 3, 4))
    pos[0] = 9
    assert hold.pos == (1, 2, 3)
    assert hold.normal == pytest.approx((0, 0.6, 0.8))


def test_zero_friction_is_valid_for_frictionless_support():
    hold = Hold("h", (0, 0, 0), HoldType.FOOTHOLD, friction_override=0)
    assert hold.friction == 0


@pytest.mark.parametrize("name", ["", "   ", None])
def test_hold_rejects_empty_name(name):
    with pytest.raises(ValueError, match="name"):
        Hold(name, (0, 0, 0), HoldType.JUG)


def test_hold_rejects_unknown_type():
    with pytest.raises(ValueError, match="type"):
        Hold("h", (0, 0, 0), "magnet")
