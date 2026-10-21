"""Geometric wall-clearance diagnostics, independent of collision solving."""

from pathlib import Path

import mujoco
import pytest

from climbsim.diagnostics import wall_clearance
from climbsim.kinematics import solve_pose_ik
from climbsim.model import ClimberModel
from climbsim.route import Route


@pytest.mark.parametrize("geom, expected", [
    ('type="sphere" size="0.2"', 0.8),
    ('type="capsule" size="0.1 0.4"', 0.9),
    ('type="capsule" size="0.1 0.4" euler="90 0 0"', 0.5),
    ('type="capsule" size="0.1 0.4" euler="30 0 0"', 0.7),
    ('type="box" size="0.1 0.2 0.3"', 0.8),
    ('type="box" size="0.1 0.2 0.3" euler="90 0 0"', 0.7),
    # World y projection: .2*cos(30 deg) + .3*sin(30 deg).
    ('type="box" size="0.1 0.2 0.3" euler="30 0 0"', 0.6767949192431123),
])
def test_rotated_shape_clearance_matches_analytic_support_extent(geom, expected):
    model = mujoco.MjModel.from_xml_string(f"""
    <mujoco><worldbody><body name="moving" pos="0 1 0">
      <freejoint/><geom name="shape" {geom}/>
    </body></worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    report = wall_clearance(model, data)
    assert report["min_clearance_m"] == pytest.approx(expected)
    assert report["body"] == "moving"
    assert report["geom"] == "shape"


def test_world_pose_includes_body_rotation_and_jointless_child_geometry():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco><worldbody>
      <geom type="sphere" pos="0 -10 0" size="1"/>
      <body mocap="true" pos="0 -20 0"><geom type="sphere" size="1"/></body>
      <body name="moving" pos="0 1 0" euler="90 0 0"><freejoint/>
        <body name="child"><geom type="capsule" size="0.1 0.4"/></body>
      </body>
    </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    report = wall_clearance(model, data, wall_y=0.25)
    assert report["min_clearance_m"] == pytest.approx(0.25)
    assert report["body"] == "child"


def test_absence_of_dynamic_geometry_has_no_finite_clearance():
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size="1"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    report = wall_clearance(model, data)
    assert report["min_clearance_m"] is None
    assert report["body"] is None


def test_unsupported_dynamic_shapes_are_reported_as_incomplete_coverage():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body><freejoint/>
      <geom type="ellipsoid" size="0.1 0.2 0.3"/>
    </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert wall_clearance(model, data)["unsupported_geom_count"] == 1


def test_legacy_ladder_start_exposes_wall_penetration_despite_site_alignment():
    path = Path(__file__).resolve().parents[1] / "data/routes/easy_ladder.json"
    route = Route.from_json(path)
    cm = ClimberModel.build(route)
    cm.init_qpos_standing()
    residual = solve_pose_ik(cm, {limb: route.hold(name).pos for limb, name in route.start.items()},
                             tol=0.0001, max_iters=600)
    mujoco.mj_forward(cm.model, cm.data)
    assert residual < 0.0001
    report = wall_clearance(cm.model, cm.data)
    assert report["min_clearance_m"] < -0.06
    assert report["body"].startswith("shank")
    assert report["unsupported_geom_count"] == 0
