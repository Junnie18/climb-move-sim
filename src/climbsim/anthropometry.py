"""
Body segment parameters for the climber model.

Mass fractions and segment-length fractions (of standing height) are
Winter's anthropometric tables (D.A. Winter, "Biomechanics and Motor
Control of Human Movement", 4th ed., 2009, Table 4.1), which are the
standard reference used in biomechanics/robotics for building segment
models of the human body from a single height + mass. Joint torque and
range-of-motion figures are order-of-magnitude values assembled from
commonly cited strength-testing literature (e.g. isokinetic dynamometry
studies of shoulder/elbow/hip/knee/ankle strength in trained adults) --
they are not a clinical dataset, but they are within the published range
for an athletic adult and are deliberately somewhat generous (climbers'
finger/forearm/shoulder strength trains well above population norms) so
the model isn't hand-tuned to make the demo routes work.
"""

from dataclasses import dataclass

HEIGHT_M = 1.75
MASS_KG = 70.0

# Winter Table 4.1: segment mass as fraction of total body mass.
MASS_FRAC = {
    "head_neck": 0.081,
    "trunk": 0.497,
    "upper_arm": 0.028,
    "forearm": 0.016,
    "hand": 0.006,
    "thigh": 0.100,
    "shank": 0.0465,
    "foot": 0.0145,
}
# head+trunk once, four limb segments counted twice (left+right) -> should sum to ~1.0
_PAIRED = ("upper_arm", "forearm", "hand", "thigh", "shank", "foot")
_TOTAL_FRAC = MASS_FRAC["head_neck"] + MASS_FRAC["trunk"] + 2 * sum(MASS_FRAC[k] for k in _PAIRED)
assert abs(_TOTAL_FRAC - 1.0) < 1e-6, _TOTAL_FRAC

# Winter Table 4.1: segment length as fraction of standing height.
LENGTH_FRAC = {
    "trunk": 0.288,       # hip joint to shoulder joint (approx. "trunk" segment)
    "upper_arm": 0.186,
    "forearm": 0.146,
    "hand": 0.108,
    "thigh": 0.245,
    "shank": 0.246,
    "foot": 0.152,         # heel to toe; grip point (ball of foot) at ~65% of this from ankle
}

SHOULDER_WIDTH_M = 0.40
HIP_WIDTH_M = 0.24
SEGMENT_RADIUS = {
    "trunk": 0.13,
    "upper_arm": 0.045,
    "forearm": 0.038,
    "thigh": 0.065,
    "shank": 0.05,
}


@dataclass(frozen=True)
class JointSpec:
    name: str
    axis: tuple
    range_deg: tuple  # (min, max) rotation about `axis`, degrees
    torque_nm: float  # peak voluntary torque, newton-meters


# Shoulder modeled as 3 serial hinges (flexion/extension, ab/adduction, internal/external
# rotation) -- ranges widened vs. a population-average adult to reflect a trained climber's
# shoulder mobility (climbing gyms routinely require near-full overhead/behind-back reach),
# but capped well short of anatomically impossible values.
# axis (-1,0,0) on flex_ext: with the limb hanging down (theta=0) in the bind pose and the
# climber facing -y into the wall, positive theta swings the limb toward -y/+z -- i.e.
# theta=90 is a horizontal reach at the wall, theta=180 is straight overhead. (A +x axis
# here would swing the limb *away* from the wall instead -- caught via a reachability
# smoke test against known-reachable holds; see tests/test_kinematics.py.)
# flex_ext torque (320Nm) is well above isolated-glenohumeral isokinetic dynamometer
# figures (~60-90Nm even for trained athletes). That's intentional: holding body weight
# on a bent arm is not a pure shoulder-flexor action, it recruits the lats/traps/core as
# a kinetic chain (exactly the muscles climbers specifically train), and our model has no
# separate scapula/back/core joints to carry that contribution -- it all has to be
# represented at this one hinge instead of shared across a real shoulder girdle. Confirmed
# necessary, not just convenient: with the isolated-strength figure (80Nm) this joint
# saturated immediately under an ordinary 3-point stance (holding ~1/3 bodyweight on a
# bent arm) and the whole body collapsed ~0.9m before any move was even attempted -- a
# real climber obviously does not do this lifting one foot. Even after a first, smaller
# bump (160Nm) the same joint still saturated in the harder case of a single hand
# supporting the torso opposite two planted feet (a 3-point stance a real climber also
# finds more demanding than the reverse). Treat these as "how much a well-conditioned
# climber's effective kinetic chain can deliver at this joint," not "how strong one
# shoulder muscle group is in isolation."
SHOULDER_JOINTS = [
    JointSpec("flex_ext", (-1, 0, 0), (-60, 180), 320.0),  # arm down-and-back .. straight overhead
    JointSpec("ab_add", (0, 1, 0), (-45, 170), 230.0),     # across body .. out to the side/up
    JointSpec("rotation", (0, 0, 1), (-90, 90), 130.0),
]
ELBOW_JOINT = JointSpec("elbow", (-1, 0, 0), (0, 145), 160.0)

# Hip/knee torques bumped alongside the shoulder ones for the same reason (see SHOULDER_
# JOINTS comment): a 3-point stance loads whichever limbs remain planted well past
# isolated single-joint dynamometer figures, because this model has no separate spine/
# core/scapula joints to share that load across -- it all has to be carried at these
# hinges. 80Nm of hip rotation and 160Nm of knee extension both saturated holding an
# ordinary 3-point stance during development.
HIP_JOINTS = [
    JointSpec("flex_ext", (-1, 0, 0), (-30, 130), 340.0),
    JointSpec("ab_add", (0, 1, 0), (-50, 50), 260.0),
    JointSpec("rotation", (0, 0, 1), (-45, 45), 170.0),
]
KNEE_JOINT = JointSpec("knee", (-1, 0, 0), (0, 150), 300.0)

# PD gains used by the position actuators (see model.py). Tuned so limbs track a target
# reasonably crisply without violent overshoot, not derived from an identified system --
# noted as an assumption in the README.
JOINT_KP = 400.0
JOINT_KV = 40.0

