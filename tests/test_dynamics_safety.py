"""Regressions for false success caused by ignored physics and grip snaps."""
from pathlib import Path

import numpy as np

from climbsim import feasibility
from climbsim.holds import Hold, HoldType, HOLD_PROPERTIES
from climbsim.route import Route, Move

ROUTES = Path(__file__).resolve().parents[1] / "data" / "routes"


def test_initial_pose_ik_failure_is_not_reported_as_physical_impossibility():
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.holds["rh0"] = Hold("rh0", (10.0, 0.05, 1.55), HoldType.JUG)
    result = feasibility.attempt_route(route)
    assert result.first_failure.phase == "initialization"
    assert result.first_failure.detail["category"] == "initial_ik_nonconvergence"
    assert not result.fell


def test_unarrived_limb_cannot_activate_target_constraint(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    monkeypatch.setattr(feasibility, "TRANSITION_S", 0.002)
    result = feasibility.attempt_route(route)
    assert not result.success, "a 30 cm reach cannot finish in one 2 ms step"
    assert result.first_failure.detail["category"] == "contact_not_reached"
    assert result.first_failure.detail["distance_m"] > 0.1


def test_starting_stance_force_limit_is_checked_during_transient(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.moves = []
    # Apply one brief external downward velocity perturbation during settling.
    # The 400 N contacts can support rest with feedforward, but cannot absorb
    # this transient. End-only validation would miss the overload.
    props = dict(HOLD_PROPERTIES[HoldType.FOOTHOLD], max_grip_n=400.0)
    monkeypatch.setitem(HOLD_PROPERTIES, HoldType.FOOTHOLD, props)
    import mujoco
    original_step = mujoco.mj_step
    disturbed = False

    def disturbed_step(model, data):
        nonlocal disturbed
        if not disturbed and data.time >= 0.02:
            data.qvel[2] -= 0.3
            disturbed = True
        original_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", disturbed_step)
    result = feasibility.attempt_route(route)
    assert result.first_failure is not None
    assert result.first_failure.phase == "initialization"
    assert result.first_failure.detail["category"] == "grip_strength_exceeded"
    assert result.first_failure.detail["time_s"] < feasibility.INITIAL_SETTLE_S


def test_contact_acquisition_never_snaps_to_nominal_hold_center():
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.moves = route.moves[:1]
    acquired = []

    def observe(cm, phase, move_index, current_holds):
        if phase == "contact":
            limb = route.moves[move_index].limb
            anchor = cm.data.mocap_pos[cm.limb_anchor_mocap_id[limb]]
            acquired.append(float(np.linalg.norm(anchor - cm.limb_site_pos(limb))))

    feasibility.attempt_route(route, observer=observe)
    assert acquired, "an arrived limb must emit its contact-acquisition state"
    assert max(acquired) < 1e-10, "new equality anchor must use the actual contact point"


def test_symmetric_start_does_not_inherit_one_sided_posture_bias(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.moves = []
    monkeypatch.setattr(feasibility, "INITIAL_SETTLE_S", 0.0)
    initial = []

    def observe(cm, phase, move_index, current_holds):
        if phase == "initialization":
            initial.append(cm.data.qpos[0])

    feasibility.attempt_route(route, observer=observe)
    assert initial
    assert abs(initial[0]) < 0.001


def test_nonfinite_state_is_a_numerical_failure_not_a_contact_failure():
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    injected = False

    def inject(cm, phase, move_index, current_holds):
        nonlocal injected
        if phase == "initialization" and not injected:
            cm.data.qvel[0] = np.nan
            injected = True

    result = feasibility.attempt_route(route, observer=inject)
    assert result.first_failure.detail["category"] == "numerical_instability"
    assert "numerical" in result.first_failure.reason


def test_fast_limb_cannot_attach_even_inside_the_contact_patch(monkeypatch):
    route = Route.from_json(str(ROUTES / "easy_ladder.json"))
    route.moves = route.moves[:1]
    monkeypatch.setattr(feasibility, "CONTACT_SPEED_M_S", 0.0)
    result = feasibility.attempt_route(route)
    assert result.first_failure.detail["category"] == "contact_not_reached"
    assert result.first_failure.detail["distance_m"] < feasibility.CONTACT_DISTANCE_M
    assert result.first_failure.detail["speed_m_s"] > 0


def test_settle_force_limit_is_checked_before_the_window_ends(monkeypatch):
    import mujoco

    route = Route.from_json(str(ROUTES / "showcase" / "01_supported_steps.json"))
    route.moves = route.moves[:1]
    original_step = mujoco.mj_step
    disturbed = False

    def disturbed_step(model, data):
        nonlocal disturbed
        # A brief external disturbance after the moving hand has reattached.
        # Its overload occurs well before the 3 s settle window finishes.
        if not disturbed and data.time > 3.5 and np.count_nonzero(data.eq_active) == 4:
            data.qvel[2] -= 1.0
            disturbed = True
        original_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", disturbed_step)
    result = feasibility.attempt_route(route)
    assert disturbed
    assert not result.success
    assert result.first_failure.phase == "settle"
    assert result.first_failure.detail["category"] in {"grip_strength_exceeded", "contact_cone_exceeded"}
    assert result.first_failure.detail["time_s"] < 3.6
