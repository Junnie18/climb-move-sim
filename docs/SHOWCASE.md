# Presenting the project

Open https://ascent-climbing-lab.vercel.app/ or start
`python scripts/serve_editor.py` and visit http://127.0.0.1:8765/.
The editor runs new simulations through Python. Opening `docs/demo/index.html`
offline still supports route editing and recorded replays.

## Show the interactive editor

1. Choose **Supported hand sequence** and run it unchanged. The controller
   should complete its four small hand moves. Open the replay to inspect loads.
2. Return to the editor. Drag a target hand hold much higher. The previous
   verdict is cleared immediately. Run again and inspect the returned failure.
3. Reset the route, clear the moves, choose **Place limbs** and **Add move**, then
   drag a hand handle onto the next nearby hold. The explicit move list is what
   will be simulated. **Start pose** instead changes the initial stance.
4. Show grip capacity and friction in the hold inspector. Explain that reaching
   a point is only part of the problem: the remaining contacts must support the
   body, and the arriving limb must establish a valid contact.

This is model/controller feasibility, with illustrative strengths and no
body-wall collision response. A failed solve is not proof of human impossibility.

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
friction-cone checks; built a drag-and-drop route editor backed by isolated
simulation workers and a 3D force-telemetry replay, with analytic physics and
timestep-sensitivity validation.

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
