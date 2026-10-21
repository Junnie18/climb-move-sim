# Physics and validation

This project studies whether a particular controller can execute a sequence of
climbing contacts in a simplified rigid-body model. It is not a predictor of an
individual climber's ability or a climbing-safety tool.

## Dynamics

The MuJoCo model has a free-floating root and 16 actuated hinge joints: 22 velocity
degrees of freedom. Gravity acts on a 70 kg segmented body. Joint position servos
have explicit torque limits. The moving limb receives gravity/Coriolis bias
compensation through those same limited actuators, rather than an unlimited
external force.

Initial stance IK adjusts the root and limbs together. During movement, IK works
on separate `MjData`; planning cannot teleport the live body. Each Jacobian is
evaluated after both kinematics and center-of-mass quantities are refreshed.
The controller periodically replans and rate-limits its joint target. Support
feedforward approximately distributes the root's gravity wrench over active
contacts, projects forces into their allowed cones/caps, then maps them through
the site Jacobians into joint torque offsets. These offsets also enter the
torque-limited actuators. The allocation is a control target, not an accepted
force measurement: actual MuJoCo constraint forces still determine every result.

Position control is deliberately a baseline. A failed tracking attempt does not
establish that an optimal human or robot controller could not do the move.

## Contacts

An active hand/foot attachment is a MuJoCo connect equality. Its three solved
constraint forces are interpreted in world coordinates with force on the limb
at site1. Analytic suspended-mass tests check both sign and magnitude, including
gravity that is not aligned with a coordinate axis.

For a unit outward surface normal **n** and solved force **f**:

```
normal force       Fn = dot(f, n)
tangential force   Ft = norm(f - Fn*n)
unilateral support Fn >= 0
Coulomb friction   Ft <= mu*Fn
effective capacity norm(f) <= max_grip_n
```

Feet and ungraspable surfaces must satisfy all three inequalities, up to a 1e-6 N
roundoff tolerance. Feet therefore cannot act as glued anchors that pull on the
wall. Footholds default to upward normals; slopers default to the outward wall
normal. Inclined surfaces accept explicit normals. Positive jug/crimp **hand**
grasps retain the simpler force-cap approximation, since fingers can grasp an
edge. Toe/heel hooks are not represented.

The inequalities check attachment validity; they are not a full contact-pressure
or sliding-friction solver. A violated attachment is released and the attempt
stops. The legacy `fell` field denotes support failure, not a simulated fall.

A new grip requires both proximity to a finite contact patch and low relative
site speed. The equality anchors at the actual reached point rather than pulling
the body to the hold center. Contact loads are checked immediately and throughout
settling, including the forces used by integration. There is no grace period
that silently ignores overloaded attachments.

The controller waits until site speed is at most 0.02 m/s before attaching a
foot or an ungraspable contact. Jug/crimp hands retain a 0.12 m/s ceiling.
These are illustrative acquisition settings, not experimentally fitted human
parameters. The slower touchdown avoids braking impulses that previously made
5–10 cm foot moves fail immediately at attachment. Friction, grip capacities
and the per-sample force checks are unchanged.

## Interpreting results

| Result | What it establishes |
|---|---|
| Completed | This controller executed the requested sequence under the model assumptions. |
| Pose held | The displayed contacts supported the model through three seconds of settling. Getting into the pose was not tested. |
| Geometric reach limit | The target exceeds a conservative segment-length bound from the current stance. Repositioning the body could change this. |
| IK did not converge | The numerical planner did not find a solution; reachability remains unresolved. |
| Contact not reached | The controller failed the distance/speed gate within its movement window. |
| Contact cone exceeded | The attachment would require tension or more friction than the configured surface supplies. |
| Grip strength exceeded | The required force exceeds the configured effective capacity. |

The editor offers separate pose and movement checks. Pose checks initialize
whole-body IK at the displayed contacts and run the usual dynamic settling and
force checks. Movement checks start from the route's original stance and execute
its sequence. Their per-limb IK holds the torso fixed while planning; it does
not search body-repositioning strategies. For example, a foot target 5 cm below
its initial position can fail the movement planner while the same final contacts
pass the pose check. This is a planner limitation, not evidence of an impossible
stance.

## Known approximation: wall intersections

The wall is a visual plane, not a collision-solving surface. The coarse hands,
feet and limbs can intersect it. The replay displays the signed minimum surface
clearance for sphere, capsule and box geometry at the current pose. Negative
clearance is an intersection, not an allowed physical contact. The run summary
also records the worst clearance.

Adding collision response without first correcting the body/contact geometry
would create contradictory constraints. A future model needs collision-aware
posture planning, anatomically better foot placement and calibrated contact
patches. Current completed routes remain subject to this limitation.

## What is and is not validated

`python -m pytest` checks analytic force cases, friction directionality, geometric
clearance, planning-state isolation, Jacobian consistency, contact gating,
input validation, exported telemetry and end-to-end scenarios. These establish
internal correctness for the tested cases.

[The stance audit](stance_validation.json) records 28 supported synthetic poses,
including raised/spread contacts, crouches and translated stances. Five varied
poses are retained as end-to-end regression tests. Contact-acquisition tests
cover eight small foot movements and audit actual forces throughout integration,
including the first newly attached sample. Genuine reach and grip-limit failures
remain covered. These are model/controller checks, not human climbing trials.

`python scripts/validate_physics.py` repeats the showcase at 2 ms, 1 ms and 0.5 ms.
It records outcome categories, event times, force peaks, final center of mass and
wall clearance. The finest run is the numerical reference, not empirical truth.
Classification disagreement causes the script to exit unsuccessfully.

The inherited segment proportions are attributed to Winter's anthropometric
tables. Joint limits, effective torques, hold coefficients and controller gains
are engineering assumptions. None has been fitted to measured climbing trials.
The model has no articulated spine, fingers, wrists, ankles, fatigue, muscle
activation dynamics, skin deformation or dynamic jump planner. No percentage
claim about real-world accuracy is supported.

## Primary references

- [MuJoCo simulation pipeline](https://mujoco.readthedocs.io/en/stable/programming/simulation.html): integration order and state caches.
- [MuJoCo constraint model](https://mujoco.readthedocs.io/en/stable/computation/index.html): equality constraints, force vectors and friction cones.
- [MuJoCo API functions](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html): kinematics, center-of-mass updates and Jacobians.
- [MuJoCo constraint reference](https://mujoco.readthedocs.io/en/stable/modeling.html#reference): compliance and the timestep/time-constant relationship.
- D. A. Winter, *Biomechanics and Motor Control of Human Movement*, 4th ed.
  (2009), Table 4.1: provenance recorded for the inherited segment table, not
  empirical validation of this implementation.
