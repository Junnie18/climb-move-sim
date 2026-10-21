"""Representative static stances that the contact model can actually support.

These checks initialize a candidate pose and hold it under MuJoCo dynamics.
They do not assert that the movement controller can reach the same pose from
another stance, or that a human can perform it. Keeping those questions apart
prevents a failed movement attempt from becoming a blanket rejection of a pose.
"""

from pathlib import Path

import pytest

from climbsim.feasibility import INITIAL_SETTLE_S, attempt_route
from climbsim.route import Route


BASELINE = Path(__file__).resolve().parents[1] / "data/routes/showcase/01_supported_steps.json"


@pytest.mark.parametrize(
    "positions",
    [
        {"lf0": (-0.18, 0.05, 0.85)},
        {"lh0": (-0.52, 0.05, 1.55)},
        {"lh0": (-0.22, 0.05, 1.10), "rh0": (0.22, 0.05, 1.10),
         "lf0": (-0.18, 0.05, 0.80), "rf0": (0.18, 0.05, 0.80)},
        {"lh0": (-0.40, 0.05, 1.10), "rh0": (0.40, 0.05, 1.10),
         "lf0": (-0.40, 0.05, 0.80), "rf0": (0.40, 0.05, 0.80)},
        {"lh0": (-0.05, 0.05, 1.55), "rh0": (0.05, 0.05, 1.55),
         "lf0": (-0.05, 0.05, 0.35), "rf0": (0.05, 0.05, 0.35)},
    ],
    ids=["left-foot-up-10cm", "left-hand-out-30cm", "deep-crouch", "wide-crouch", "narrow-stance"],
)
def test_varied_static_stances_hold_under_real_dynamics(positions):
    payload = Route.from_json(str(BASELINE)).to_dict()
    payload["moves"] = []
    for hold in payload["holds"]:
        if hold["name"] in positions:
            hold["pos"] = list(positions[hold["name"]])

    result = attempt_route(Route.from_dict(payload))

    assert result.success, [(outcome.reason, outcome.detail) for outcome in result.outcomes]
    assert not result.fell
    assert result.first_failure is None
    assert result.outcomes[-1].detail["duration_s"] == INITIAL_SETTLE_S
    assert result.outcomes[-1].detail["time_s"] >= INITIAL_SETTLE_S - 1e-9
