# Ascent · Climbing Physics Lab

A climbing-move simulator with a 22-DOF MuJoCo body, numerical inverse
kinematics, torque-limited control and force-checked contacts. Build a route
by dragging holds and assigning hands and feet, simulate the move sequence,
then inspect what happened in a 3D replay with force telemetry.

The interesting question is not just “can the hand reach?” It is also “can
the other three contacts support the body while it moves?”

## Interactive route editor

**[Open the live climbing lab](https://ascent-climbing-lab.vercel.app/)**

1. Pick a route template. In **Move holds**, drag a hold to change its position.
2. Switch to **Place limbs**. **Start pose** assigns the four starting contacts;
   **Add move** appends a hand or foot move to the sequence.
3. Drag a labeled limb handle onto a hold, or select a limb and target in the
   inspector. Hold coordinates, type, grip capacity and friction are editable.
4. Click **Run simulation**. The Python service simulates the edited route in
   MuJoCo and returns a new verdict and trajectory. Open **View replay** to
   inspect the motion, contacts and force history.

Edits clear the previous verdict. Undo and reset also require a fresh run. The
planning figure illustrates contact assignments; it is not a solved body pose.
Use **Export JSON** to keep a route and **Import route** to reopen it. The
service does not maintain a route database.

To run the full app locally after installing the Python dependencies below:

```bash
python scripts/serve_editor.py
```

Open **http://127.0.0.1:8765/**. The local server serves both the editor and
simulation API. No GPU, API key or paid model service is required.

The **[GitHub Pages copy](https://junnie18.github.io/climb-move-sim/)** and a
downloaded **[docs/demo/index.html](docs/demo/index.html)** support editing and
recorded replay. Fresh simulations require the hosted service or a local Python
server; they do not run inside the browser.

## Inspect a simulation

- Orbit the climber or switch between front and side views.
- Play, pause, change speed and scrub through the actual simulation.
- Inspect per-limb force vectors, capacity use and force histories.
- Track the center of mass and the current move/phase.
- Compare completed movements with reach, tracking and grip failures.
- Import a trajectory JSON produced by your own simulation.

The replay uses MuJoCo states returned by a fresh run or bundled with the three
recorded examples. **Restart** returns to simulation time zero.

## Run your own simulation

Python 3.12 is the tested setup. Install into a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
```

Run a route and export its replay:

```bash
python scripts/run_route.py data/routes/showcase/01_supported_steps.json
python scripts/export_trajectory.py data/routes/showcase/01_supported_steps.json attempt.json
```

Open the demo and use **Import trajectory** to inspect `attempt.json`.
To regenerate the bundled scenarios or compare numerical resolution:

```bash
python scripts/build_demo.py
python scripts/validate_physics.py
python -m pytest -q
node --test docs/demo/*.test.cjs
```

An optional MP4 renderer requires a working OpenGL environment:

```bash
python scripts/run_route.py data/routes/showcase/01_supported_steps.json --render attempt.mp4
```

## What changed in v0.2

| Earlier behavior | Current behavior |
|---|---|
| Replanning could overwrite the live limb pose. | IK uses isolated state; only physics integration moves the body. |
| A grip could attach to a distant target. | Distance and relative-speed gates verify actual arrival. |
| Feet acted like bidirectional glued anchors. | Feet require compressive support within a surface-specific Coulomb cone. |
| Settle overloads could be missed. | Loads are checked throughout motion and settling. |
| Unsuccessful IK was described as impossible. | Geometric, numerical-planning, controller and contact failures are distinct. |
| Render timing used an artificial frame counter. | Replay timestamps come from the simulation clock. |
| Body-wall intersections were invisible in the result. | Signed clearance is measured and displayed explicitly. |

Contact surfaces support configurable normals, friction coefficients and force
limits. Configuration is validated before compiling a model.

## Define a route

The wall frame uses x across the wall, y out from the wall and z upward, in meters.
A route supplies four starting contacts and a list of single-limb moves. A hold
can override the illustrative defaults:

```json
{
  "name": "foot_edge",
  "pos": [0.18, 0.05, 0.75],
  "type": "foothold",
  "normal": [0, 0, 1],
  "friction": 1.0,
  "max_grip_n": 1200,
  "radius": 0.035
}
```

See [data/routes](data/routes) for complete examples. To append a hypothetical
move and export the entire resulting attempt:

```bash
python scripts/test_move.py path/to/route.json left_hand target_hold attempt.json
```

## What the results mean

“Controller completed” means that the requested sequence succeeded in this
simplified simulation. It does **not** establish that a real person can climb it.
An unsuccessful controller or IK attempt does not prove that nobody could.

Joint strengths and hold parameters are uncalibrated engineering assumptions.
The wall is visual only; body-wall collision response is not implemented, and
the replay flags intersections of the coarse body geometry. Fingers, fatigue,
muscle dynamics and dynamic jumps are outside this model.

The project emphasizes visible assumptions, reproducible trajectories and tests
against analytic physics. Read [the physics and validation notes](docs/PHYSICS.md)
and [timestep sensitivity results](docs/validation.json) before interpreting
results as evidence about human movement.

[Showcase walkthrough and resume bullet](docs/SHOWCASE.md) ·
[Independent integration audit](docs/PHYSICS_AUDIT.md)

## Implementation

```
src/climbsim/
  anthropometry.py     Segment properties and effective actuator limits
  model.py             MuJoCo model, geometry and actuator construction
  kinematics.py        Whole-body initialization and isolated limb IK
  forces.py            Constraint force extraction and contact checks
  diagnostics.py       Signed body-to-wall clearance
  feasibility.py       Controller, contact lifecycle and failure categories
  holds.py / route.py  Validated route and surface configuration
  trajectory_export.py Shared geometry, states and force telemetry export
  editor_server.py    Bounded HTTP API with isolated simulation workers
scripts/
  serve_editor.py     Start the complete local editor and simulation API
  run_route.py         Simulate and optionally render MP4
  export_trajectory.py Export real geometry, states and force telemetry
  build_demo.py        Regenerate the offline showcase
  validate_physics.py  Timestep sensitivity study
docs/demo/             Dependency-free route editor and 3D replay
api/index.py           Hosted Python API adapter
tests/                 Analytic, regression and integration checks
```

Built with Python, NumPy and MuJoCo. The browser renderer uses Canvas and
quaternion-transformed model geometry with no external JavaScript dependencies.
