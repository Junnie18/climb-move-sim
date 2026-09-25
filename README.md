# climb-move-sim

A physics-based feasibility checker for rock climbing moves. Like block-based coding,
but each "block" is a climbing move -- assign one limb (a hand or foot) to a hold on a
route -- and the system calculates, from real body-mechanics simulation, whether that
move is actually achievable, and can render the attempt.

A route is a start stance plus a sequence of moves, where each move repositions exactly
one limb at a time (the standard way both climbers and this project's future block-based
UI think about a route). For each move, the pipeline:

1. **Kinematic reachability.** Can the moving limb's end-effector reach the target hold
   at all, given the model's real joint geometry and range-of-motion limits, with the
   other three limbs held fixed? Solved with numerical inverse kinematics (damped least
   squares against the actual compiled [MuJoCo](https://mujoco.org/) model via its
   Jacobian) -- not a distance heuristic.
2. **Forward-dynamics simulation of the transition.** The target limb's grip releases
   and it's driven toward the IK solution by torque-limited PD position actuators while
   gravity acts throughout and the other three limbs stay passively anchored. Every
   step, each anchored limb's real constraint force (read directly from MuJoCo's
   solved `efc_force`, not estimated) is checked against its hold's friction cone and
   rated breaking/grip-strength.
3. **Settle.** Once the moving limb's new grip engages, a short settle window is
   simulated and all four grips are re-checked -- the static-equilibrium check, done via
   genuine forward dynamics rather than a separate analytic solve.

Any friction-cone or max-grip violation, at any point, fails the move.

## Demo results

| Route | Result | Why |
|---|---|---|
| [`easy_ladder`](data/routes/easy_ladder.json) | **Feasible** | Both feet step up onto slightly higher footholds. Every grip force stays well inside its hold's rating throughout (e.g. jugs settle around 150-450N of a 1400N rating). |
| [`dyno_reach`](data/routes/dyno_reach.json) | **Infeasible** | A single reach for a hold 1.13m away -- ~24.5cm beyond the arm's ~0.70m maximum extension. Multi-restart IK cannot find *any* joint configuration that closes the gap; this is a hard kinematic bound, not a tuning artifact. |

Rendered MP4s of both attempts: [`data/renders/easy_ladder.mp4`](data/renders/easy_ladder.mp4),
[`data/renders/dyno_reach.mp4`](data/renders/dyno_reach.mp4).

## Running it

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e .

python scripts/run_route.py data/routes/easy_ladder.json
python scripts/run_route.py data/routes/dyno_reach.json --render out.mp4
```

Requires Python >= 3.10 (MuJoCo's Python bindings dropped 3.9; this repo was built
against 3.12). `pip install -e .` pulls in `mujoco`, `numpy`, `imageio`.

Run the test suite with `pytest`.

## Project structure

```
src/climbsim/
  anthropometry.py   Body segment masses/lengths (Winter's tables) and joint torque/ROM
  holds.py            Hold types (jug/crimp/sloper/foothold) and their friction/grip ratings
  route.py            Route + Move ("block") representation, JSON load/save
  model.py             Builds the MJCF humanoid + wall + holds; per-limb bookkeeping
  kinematics.py       Numerical IK (single-limb and whole-body variants)
  forces.py           Reads real per-limb grip forces from the solver; friction-cone /
                       max-grip check
  feasibility.py       Orchestrates a full route attempt, move by move
  render.py            MuJoCo offscreen rendering to MP4

scripts/run_route.py  CLI: check a route, optionally render it
data/routes/           The two demo routes (JSON)
data/renders/           Their rendered MP4s
tests/                 pytest suite (model, kinematics, forces, end-to-end feasibility)
```

## The climber model

A free-floating torso (nothing pins the climber to the wall except grip forces) with
four limbs. Each shoulder/hip is three serial hinge joints on one body (the standard
MuJoCo pattern for a torque-limited, range-limited ball joint); each elbow/knee is one
more hinge. Hands and feet are not separately articulated -- wrist/ankle are treated as
rigid extensions of the fore-limb, since their contribution to whether a *reach* is
possible is secondary to the shoulder/hip/elbow/knee chain that actually determines it.

Segment masses and lengths come from Winter's anthropometric tables (a 1.75m, 70kg
adult) via a standard "sum of segment mass fractions" model -- these are not invented
numbers, though the joint torque and range-of-motion figures below are engineering
estimates rather than a clinical dataset (see "Assumptions").

## Assumptions

- **Anthropometry (mass/length):** Winter's Table 4.1 segment mass and length
  fractions, a well-established biomechanics reference.
- **Joint torque limits are climber-specific, not population-average.** Isolated
  glenohumeral-flexor dynamometer figures (~60-90Nm) turned out to be far too low: with
  them, a climber's shoulder saturated just holding a 3-point stance and the whole body
  collapsed ~0.9m before any move was even attempted. Real climbing recruits a kinetic
  chain (lats/traps/core) that this model has no separate joints to represent, so those
  contributions have to be folded into the one hinge that's actually there. The shipped
  values (up to 320Nm shoulder flexion, 340Nm hip flexion) were tuned to be the smallest
  bump that held an ordinary engaged stance without saturating -- read them as "what a
  well-conditioned climber's effective kinetic chain can deliver at this joint," not
  "isolated single-muscle strength."
- **Hold friction/grip ratings are illustrative**, not measured from real hardware.
  Jugs and crimps are modeled as small mechanical anchors (bounded mainly by a rated
  breaking/grip-strength force, since a positive edge resists a range of pull
  directions); slopers are the one type actually friction-cone-gated, since holding on
  without a positive edge genuinely requires enough force pressed into the wall to
  generate friction.
- **PD gains (kp=400, kv=40) are hand-picked**, not system-identified, chosen so limbs
  track a target crisply without violent overshoot.
- **Single global wall-normal friction model.** Real hand/hold contact is much richer
  3D contact mechanics; this treats every hold's friction cone relative to one fixed
  wall-normal direction.

## Known limitations / v2

- **Longer move chains stress the current controller.** Two sequential moves (proven,
  see `easy_ladder`) track cleanly (end-of-transition gaps of 3-7cm). A third move that
  drops support to a single hand plus two feet -- the more demanding of the two
  3-point-stance configurations, both in this model and for a real climber -- pushed
  joint-space tracking error up to 30-60cm in testing even after raising torque limits
  substantially and adding closed-loop re-aiming (periodic IK re-solves against the
  torso's actual, continuously-sagging pose, not a single open-loop plan). The `dyno_reach`
  route sidesteps this by failing at the reachability check, before any dynamics run.
  A proper whole-body trajectory optimizer (rather than PD-tracking a joint-space plan)
  is the fix; not built here.
- **No drag-and-drop block-editor UI yet.** v1's "block" is a JSON `{"limb", "hold"}`
  entry; a visual editor is future work built on the same `Route`/`Move` API.
- **Synthetic/hand-authored routes only.** No import from real gym/outdoor route data.
- **No momentum-driven dynos.** The dynamic-transition controller is a slow, controlled
  reach; a deliberately explosive "jump for it and catch a hold moving" move isn't
  separately modeled (the current `dyno_reach` demo fails purely on static reach, which
  is sufficient to demonstrate a real dyno is required, but doesn't simulate attempting
  the dyno itself).
