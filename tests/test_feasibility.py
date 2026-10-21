"""End-to-end route checks distinguish controller outcomes, contact failure,
and a geometric reach bound in the current stance. The legacy foot ladder
is rejected because unilateral feet cannot supply its requested shear.
"""

from pathlib import Path

from climbsim.feasibility import attempt_route
from climbsim.route import Route

ROUTES_DIR = Path(__file__).resolve().parents[1] / "data" / "routes"


def test_legacy_ladder_rejects_the_second_foot_contact_cone_violation():
    route = Route.from_json(str(ROUTES_DIR / "easy_ladder.json"))
    result = attempt_route(route, renderer=None)
    assert not result.success
    assert result.outcomes[0].passed
    failure = result.first_failure
    assert failure.move == route.moves[1]
    assert failure.detail["category"] == "contact_cone_exceeded"
    assert failure.detail["tangential_n"] > failure.detail["normal_n"]


def test_dyno_reach_route_is_correctly_infeasible():
    route = Route.from_json(str(ROUTES_DIR / "dyno_reach.json"))
    result = attempt_route(route, renderer=None)
    assert not result.success
    failure = result.first_failure
    assert failure.phase == "reachability"
    assert failure.detail["distance_m"] > failure.detail["max_reach_m"]
    # The reported deficit is a geometric bound from the current limb base.
    assert failure.detail["residual_m"] > 0.15


def test_modest_symmetric_hand_sequence_has_a_real_dynamic_solution():
    from climbsim.holds import Hold, HoldType
    from climbsim.route import Move

    route = Route.from_json(str(ROUTES_DIR / "easy_ladder.json"))
    route.moves = []
    for i, limb in enumerate(("left_hand", "right_hand", "left_hand", "right_hand")):
        start = route.hold(route.start[limb])
        name = f"hand_step_{i}"
        route.holds[name] = Hold(name, (start.pos[0], 0.05, start.pos[2] + .05 * (i // 2 + 1)), HoldType.JUG)
        route.moves.append(Move(limb, name))
    result = attempt_route(route)
    assert result.success, [(o.phase, o.reason) for o in result.outcomes]
    assert len(result.outcomes) == 4
    assert all(o.detail["arrival_distance_m"] <= o.detail["contact_radius_m"] for o in result.outcomes)
