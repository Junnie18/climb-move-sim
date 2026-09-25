"""End-to-end checks against the two shipped demo routes -- the actual deliverable of
this project: a clearly-feasible route passes for physically real reasons (every move's
grip stays within its hold's friction cone and rated strength throughout forward
simulation, not just at rest), and a route with one dynamically-unreachable move is
correctly rejected by genuine inverse kinematics, not a hardcoded distance check.
"""

from pathlib import Path

from climbsim.feasibility import attempt_route
from climbsim.route import Route

ROUTES_DIR = Path(__file__).resolve().parents[1] / "data" / "routes"


def test_easy_ladder_route_is_feasible():
    route = Route.from_json(str(ROUTES_DIR / "easy_ladder.json"))
    result = attempt_route(route, renderer=None)
    assert result.success, [(o.phase, o.reason) for o in result.outcomes if not o.passed]
    assert len(result.outcomes) == len(route.moves)
    assert all(o.passed for o in result.outcomes)


def test_dyno_reach_route_is_correctly_infeasible():
    route = Route.from_json(str(ROUTES_DIR / "dyno_reach.json"))
    result = attempt_route(route, renderer=None)
    assert not result.success
    failure = result.first_failure
    assert failure.phase == "reachability"
    assert failure.detail["distance_m"] > failure.detail["max_reach_m"]
    # infeasibility here must come from real IK non-convergence, not a stub
    assert failure.detail["residual_m"] > 0.15
