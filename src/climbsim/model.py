"""Builds the MJCF (MuJoCo XML) for a climber + wall + route, and wraps the
compiled model with the bookkeeping (per-limb site/joint/equality indices)
the rest of the package needs.

Kinematic tree: a free-floating torso (6-DOF free joint -- nothing pins the
climber to the wall except grip forces, as in reality) with four limbs, each
attached at a shoulder/hip site. Shoulders and hips are modeled as three
serial hinge joints on a single body (the standard MuJoCo trick for a
limited, torque-actuated ball joint -- see e.g. dm_control's humanoid.xml),
followed by one hinge for the elbow/knee. Hands and feet are not separately
articulated (wrist/ankle are treated as rigid extensions of the fore-limb);
the grip point is a site at the tip.

Holds are static geoms on the wall. Gripping is modeled with an
`equality/connect` constraint from each limb's end-effector site to a small
kinematic "anchor" (mocap) body that we teleport to whatever hold that limb
is currently assigned to -- so the *set* of holds doesn't have to be fixed
at compile time, only the wall + the four anchors are.
"""

from dataclasses import dataclass, field
from typing import Dict

import mujoco
import numpy as np

from climbsim import anthropometry as A
from climbsim.route import LIMBS, Route

WALL_Y = 0.0  # wall plane at y=0, holds protrude toward +y, climber faces -y
HOLD_STANDOFF = 0.04  # how far a hold's grip site sits off the wall face


def _limb_geometry():
    """Segment lengths (m) derived from height fractions, shared by model + IK."""
    h = A.HEIGHT_M
    return dict(
        trunk=A.LENGTH_FRAC["trunk"] * h,
        upper_arm=A.LENGTH_FRAC["upper_arm"] * h,
        forearm=A.LENGTH_FRAC["forearm"] * h,
        hand=A.LENGTH_FRAC["hand"] * h,
        thigh=A.LENGTH_FRAC["thigh"] * h,
        shank=A.LENGTH_FRAC["shank"] * h,
        foot=A.LENGTH_FRAC["foot"] * h,
    )


def arm_reach(geom=None) -> float:
    """Max distance from shoulder site to hand grip site (fully extended)."""
    geom = geom or _limb_geometry()
    return geom["upper_arm"] + geom["forearm"] + 0.65 * geom["hand"]


def leg_reach(geom=None) -> float:
    """Max distance from hip site to foot grip site (fully extended)."""
    geom = geom or _limb_geometry()
    return geom["thigh"] + geom["shank"] + 0.65 * geom["foot"]


def _joint_xml(spec: A.JointSpec, name: str) -> str:
    ax = " ".join(str(a) for a in spec.axis)
    lo, hi = spec.range_deg
    return (f'<joint name="{name}" type="hinge" axis="{ax}" '
            f'range="{lo} {hi}" damping="0.6" armature="0.02"/>')


def _actuator_xml(joint_name: str) -> str:
    return (f'<position name="act_{joint_name}" joint="{joint_name}" '
            f'kp="{A.JOINT_KP}" kv="{A.JOINT_KV}" ctrlrange="-6.3 6.3" forcelimited="false"/>')


def _arm_xml(side: str, geom: dict, z_offset: float) -> str:
    sign = -1 if side == "L" else 1
    ua, fa, hand_r = geom["upper_arm"], geom["forearm"], geom["hand"]
    j = A.SHOULDER_JOINTS
    joints = "".join(
        _joint_xml(spec, f"shoulder_{n}_{side}")
        for n, spec in zip(("flex", "abd", "rot"), j)
    )
    elbow = _joint_xml(A.ELBOW_JOINT, f"elbow_{side}")
    return f"""
      <body name="upper_arm_{side}" pos="{sign * A.SHOULDER_WIDTH_M / 2} 0 {z_offset}">
        {joints}
        <geom type="capsule" fromto="0 0 0 0 0.04 {-ua}" size="{A.SEGMENT_RADIUS['upper_arm']}"
              mass="{A.MASS_FRAC['upper_arm'] * A.MASS_KG}"/>
        <body name="forearm_{side}" pos="0 0.04 {-ua}">
          {elbow}
          <geom type="capsule" fromto="0 0 0 0 0.02 {-fa}" size="{A.SEGMENT_RADIUS['forearm']}"
                mass="{A.MASS_FRAC['forearm'] * A.MASS_KG}"/>
          <body name="hand_{side}" pos="0 0.02 {-fa}">
            <geom type="sphere" size="0.045" pos="0 0 {-0.6 * hand_r}"
                  mass="{A.MASS_FRAC['hand'] * A.MASS_KG}"/>
            <site name="site_{'left' if side == 'L' else 'right'}_hand" pos="0 0 {-0.6 * hand_r}" size="0.01"/>
          </body>
        </body>
      </body>"""


def _leg_xml(side: str, geom: dict, z_offset: float) -> str:
    sign = -1 if side == "L" else 1
    th, sh, foot_l = geom["thigh"], geom["shank"], geom["foot"]
    j = A.HIP_JOINTS
    joints = "".join(
        _joint_xml(spec, f"hip_{n}_{side}")
        for n, spec in zip(("flex", "abd", "rot"), j)
    )
    knee = _joint_xml(A.KNEE_JOINT, f"knee_{side}")
    return f"""
      <body name="thigh_{side}" pos="{sign * A.HIP_WIDTH_M / 2} 0 {z_offset}">
        {joints}
        <geom type="capsule" fromto="0 0 0 0 0.03 {-th}" size="{A.SEGMENT_RADIUS['thigh']}"
              mass="{A.MASS_FRAC['thigh'] * A.MASS_KG}"/>
        <body name="shank_{side}" pos="0 0.03 {-th}">
          {knee}
          <geom type="capsule" fromto="0 0 0 0 0.02 {-sh}" size="{A.SEGMENT_RADIUS['shank']}"
                mass="{A.MASS_FRAC['shank'] * A.MASS_KG}"/>
          <body name="foot_{side}" pos="0 0.02 {-sh}">
            <geom type="box" size="0.035 {0.55 * foot_l} 0.025" pos="0 {0.55 * foot_l} 0"
                  mass="{A.MASS_FRAC['foot'] * A.MASS_KG}"/>
            <site name="site_{'left' if side == 'L' else 'right'}_foot" pos="0 {0.9 * foot_l} 0" size="0.01"/>
          </body>
        </body>
      </body>"""


def _hold_geom_xml(name: str, pos, color: str, radius: float) -> str:
    x, y, z = pos
    return (f'<geom name="hold_{name}" type="sphere" pos="{x} {y} {z}" size="{radius}" '
            f'rgba="{color}" contype="2" conaffinity="1"/>'
            f'<site name="hsite_{name}" pos="{x} {y} {z}" size="0.005"/>')


_HOLD_COLOR = {
    "jug": "0.75 0.35 0.15 1",
    "crimp": "0.55 0.55 0.6 1",
    "sloper": "0.3 0.5 0.75 1",
    "foothold": "0.35 0.4 0.35 1",
}


def build_mjcf(route: Route, torso_start_pos=(0.0, 0.35, 1.55)) -> str:
    geom = _limb_geometry()
    tl = geom["trunk"]
    holds_xml = "\n      ".join(
        _hold_geom_xml(h.name, h.pos, _HOLD_COLOR[h.type.value], h.radius)
        for h in route.holds.values()
    )
    x0, y0, z0 = torso_start_pos

    anchors_xml = "\n    ".join(
        f'<body name="anchor_{limb}" mocap="true" pos="0 0 0">'
        f'<site name="asite_{limb}" size="0.008"/></body>'
        for limb in LIMBS
    )
    connects_xml = "\n    ".join(
        f'<connect name="grip_{limb}" site1="site_{limb}" site2="asite_{limb}" '
        f'active="false" solref="0.006 1" solimp="0.95 0.99 0.001"/>'
        for limb in LIMBS
    )

    xml = f"""
<mujoco model="climber">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic"/>
  <default>
    <geom friction="0.9 0.02 0.002" solref="0.01 1"/>
  </default>
  <visual>
    <headlight ambient="0.4 0.4 0.4"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.65 0.75 0.9" rgb2="1 1 1" width="128" height="128"/>
    <texture name="wall_tex" type="2d" builtin="flat" rgb1="0.82 0.78 0.72" width="64" height="64"/>
    <material name="wall_mat" texture="wall_tex" texrepeat="4 4"/>
  </asset>
  <worldbody>
    <light directional="true" pos="1 -2 3" dir="-0.3 0.6 -0.9" diffuse="0.8 0.8 0.8"/>
    <geom name="wall" type="box" pos="0 {WALL_Y - 0.05} 2" size="3 0.05 3" material="wall_mat"/>
    {holds_xml}
    {anchors_xml}
    <body name="torso" pos="{x0} {y0} {z0}">
      <freejoint name="root"/>
      <geom type="capsule" fromto="0 0 {-tl / 2} 0 0 {tl / 2}" size="{A.SEGMENT_RADIUS['trunk']}"
            mass="{A.MASS_FRAC['trunk'] * A.MASS_KG}"/>
      <geom type="sphere" pos="0 0 {tl / 2 + 0.14}" size="0.1"
            mass="{A.MASS_FRAC['head_neck'] * A.MASS_KG}"/>
      <site name="torso_site" pos="0 0 0" size="0.01"/>
      {_arm_xml('L', geom, tl / 2)}
      {_arm_xml('R', geom, tl / 2)}
      {_leg_xml('L', geom, -tl / 2)}
      {_leg_xml('R', geom, -tl / 2)}
    </body>
  </worldbody>
  <equality>
    {connects_xml}
  </equality>
  <actuator>
    {_actuator_xml('shoulder_flex_L')}
    {_actuator_xml('shoulder_abd_L')}
    {_actuator_xml('shoulder_rot_L')}
    {_actuator_xml('elbow_L')}
    {_actuator_xml('shoulder_flex_R')}
    {_actuator_xml('shoulder_abd_R')}
    {_actuator_xml('shoulder_rot_R')}
    {_actuator_xml('elbow_R')}
    {_actuator_xml('hip_flex_L')}
    {_actuator_xml('hip_abd_L')}
    {_actuator_xml('hip_rot_L')}
    {_actuator_xml('knee_L')}
    {_actuator_xml('hip_flex_R')}
    {_actuator_xml('hip_abd_R')}
    {_actuator_xml('hip_rot_R')}
    {_actuator_xml('knee_R')}
  </actuator>
</mujoco>
"""
    return xml


LIMB_TO_SITE = {
    "left_hand": "site_left_hand", "right_hand": "site_right_hand",
    "left_foot": "site_left_foot", "right_foot": "site_right_foot",
}
LIMB_JOINTS = {
    "left_hand": ["shoulder_flex_L", "shoulder_abd_L", "shoulder_rot_L", "elbow_L"],
    "right_hand": ["shoulder_flex_R", "shoulder_abd_R", "shoulder_rot_R", "elbow_R"],
    "left_foot": ["hip_flex_L", "hip_abd_L", "hip_rot_L", "knee_L"],
    "right_foot": ["hip_flex_R", "hip_abd_R", "hip_rot_R", "knee_R"],
}


@dataclass
class ClimberModel:
    route: Route
    model: mujoco.MjModel
    data: "mujoco.MjData"
    limb_joint_qpos_adr: Dict[str, list] = field(default_factory=dict)
    limb_joint_dof_adr: Dict[str, list] = field(default_factory=dict)
    limb_site_id: Dict[str, int] = field(default_factory=dict)
    limb_anchor_mocap_id: Dict[str, int] = field(default_factory=dict)
    limb_eq_id: Dict[str, int] = field(default_factory=dict)

    @classmethod
    def build(cls, route: Route) -> "ClimberModel":
        xml = build_mjcf(route)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        self = cls(route=route, model=model, data=data)
        for limb in LIMBS:
            self.limb_site_id[limb] = model.site(LIMB_TO_SITE[limb]).id
            self.limb_joint_qpos_adr[limb] = [model.joint(j).qposadr[0] for j in LIMB_JOINTS[limb]]
            self.limb_joint_dof_adr[limb] = [model.joint(j).dofadr[0] for j in LIMB_JOINTS[limb]]
            self.limb_anchor_mocap_id[limb] = model.body(f"anchor_{limb}").mocapid[0]
            self.limb_eq_id[limb] = model.equality(f"grip_{limb}").id
        return self

    def set_limb_anchor(self, limb: str, pos) -> None:
        self.data.mocap_pos[self.limb_anchor_mocap_id[limb]] = np.array(pos)

    def set_grip_active(self, limb: str, active: bool) -> None:
        self.data.eq_active[self.limb_eq_id[limb]] = active

    def limb_site_pos(self, limb: str) -> np.ndarray:
        return self.data.site_xpos[self.limb_site_id[limb]].copy()

    def actuator_ctrl_for_limb(self, limb: str) -> np.ndarray:
        names = LIMB_JOINTS[limb]
        ids = [self.model.actuator(f"act_{n}").id for n in names]
        return ids

    def init_qpos_standing(self) -> None:
        """A neutral bind pose: arms roughly forward/up, legs roughly forward -- a
        reasonable IK seed, not meant to be biomechanically final."""
        mujoco.mj_resetData(self.model, self.data)
        for limb, joints in LIMB_JOINTS.items():
            is_arm = "hand" in limb
            vals = [40, 20, 0, 60] if is_arm else [30, 10, 0, 40]
            for jname, v in zip(joints, vals):
                self.data.qpos[self.model.joint(jname).qposadr[0]] = np.deg2rad(v)
        mujoco.mj_forward(self.model, self.data)
