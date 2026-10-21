# Presenting the project

Open `docs/demo/index.html`. The complete demo works offline and starts paused
at a real, supported frame. No Python installation is needed to show the replay.

## A 90-second walkthrough

1. **Supported hand sequence.** Start playback. The model moves one hand at a
   time, in four 5 cm steps. Explain that the other contacts must support the
   body while the moving limb is detached. Show the changing force traces and
   center of mass. The physics engine produces every pose and load.
2. **Weak target grip.** Switch scenarios. This is deliberately the same kind
   of movement with an illustrative 30 N target-grip capacity. The limb reaches
   the target, but the required support load exceeds that capacity. Geometry
   alone would have missed this failure.
3. **Current-stance reach limit.** Switch again. The requested target exceeds
   the limb's geometric bound from the present stance, so it is rejected before
   movement. This is a fixed-stance result, not proof that a human could never
   reach it by repositioning or jumping.
4. **Show the engineering evidence.** Open `docs/validation.json` and
   `docs/PHYSICS.md`. Explain the analytic force tests, isolated planning state,
   numerical failure handling and 2/1/0.5 ms sensitivity study. Show the wall
   intersection warning and explain the collision model's current limitation.

## Resume bullet

Built a 22-DOF rock-climbing simulator in Python and MuJoCo with Jacobian-based
inverse kinematics, contact-aware support control, torque-limited actuation and
friction-cone checks; developed an offline 3D replay with force telemetry and
validated behavior using analytic physics and timestep-sensitivity tests.

## Interview explanation

The hardest bug was that inverse kinematics was editing the live simulator
state during planning. A successful plan could therefore inject motion without
the dynamics ever producing it. The solution was to isolate planning state and
test that live positions change only through integration. Stricter unilateral
foot contacts then exposed weaknesses in the old controller. Support
feedforward improved bodyweight distribution while preserving real actuator
limits, instead of making the grips arbitrarily stronger.

Keep the claim precise: this is a tested simulation and control project. It is
not experimentally calibrated biomechanics or a validated climbing coach.
