# Independent physics audit

The four-move supported showcase and its weak-grip counterexample were audited
on 2026-10-21 using MuJoCo 3.14.0 and 3.16.0 at 2 ms and 1 ms timesteps.
Both engine versions produced identical reported values for these cases.
These are internal simulation checks, not validation against human climbing.

## Method

A read-only wrapper around the real `mujoco.mj_step` recorded the solved forces
and actuator torques actually used for integration. Before each later step it
compared live `qpos` with the preceding integrated `qpos`, catching any controller
teleportation between steps. It also checked external generalized/body force
arrays, joint-limit constraint activity, and each active foot's Coulomb margin.
The normal force sign is independently covered by suspended-mass tests.

The ordinary observer inspected refreshed world geometry and each new contact.
At acquisition, the equality anchor had to coincide with the actual site; nominal
hold-center distance and site speed were recorded separately. No force or state
was mocked. The persistent regression is `tests/test_physics_audit.py`; it runs
one real supported attempt and checks both integration and refreshed contact loads.

## Supported sequence

`01_supported_steps.json` requests four alternating hand movements, each 5 cm
upward, for 10 cm total nominal height gain per hand. Both feet remain on their
starting supports. All four moves completed at both timesteps.

| Measurement | 2 ms | 1 ms |
|---|---:|---:|
| Largest integrated attachment force | 616.96 N | 616.96 N |
| Largest grip-cap utilization | 51.41% | 51.41% |
| Smallest foot normal force | 136.27 N | 136.56 N |
| Smallest foot friction margin, `mu*Fn - Ft` | 40.06 N | 41.36 N |
| Largest integrated actuator torque | 169.75 Nm | 169.75 Nm |
| Largest actuator-cap utilization | 53.05% | 53.05% |
| Worst wall-plane clearance | -67.03 mm | -67.03 mm |

No external force was applied through `qfrc_applied` or `xfrc_applied`. There
were no live position changes between integration calls and no active joint-limit
constraint samples. Thus this particular successful run did not depend on
teleportation, an uncapped external force, or support from joint stops.

All four acquired anchors coincided with their sites to numerical precision.
The sites were 11.79–11.99 mm from the nominal hold centers and moving at
0.1044–0.1074 m/s, below the configured 12 mm and 0.12 m/s acquisition limits.
The controller attaches within a finite patch; it does not require reaching the
patch center exactly.

## Weak-grip counterexample

`03_weak_grip.json` reduces the first target's effective force cap to 30 N.
The limb reaches the target patch, then the attempt fails during settling at
3.418 s at both timesteps. The refreshed constraint demand is 30.5364 N at 2 ms
and 30.6453 N at 1 ms. The attempt stops before integrating that overloaded
state. The preceding integrated loads on that target were at most 25.3198 N
and 28.9155 N, respectively. This distinguishes an executed load from the
current-state demand that triggers the conservative rejection.

## Limits of acceptance

The supported run still intersects the visual wall by approximately 67 mm.
The clearance diagnostic exposes this geometry error; it does not resolve
collision or make the posture physically valid. Jug/crimp hand grasps remain
an effective force-cap model, and body/hold parameters are uncalibrated.
The audit therefore supports the statement that the controller completes the
sequence under the documented model assumptions. It does not establish that
a human can perform it, that the wall intersections are acceptable, or that
the model predicts real climbing performance.

Peak telemetry must distinguish integrated dynamics from initialization previews
and terminal re-solves. The audit values above were captured at the integrator
boundary rather than inferred from subsampled replay frames.
