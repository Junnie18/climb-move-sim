# Editor usability repair

The editor must make one experiment obvious: move a limb to a hold, run the
current route, watch what happens, and adjust it using the result.

## Observed failures

- Visible limb handles only responded in a separate tool mode. A normal drag
  panned the wall instead of moving the selected limb.
- The default edited the starting stance while retaining four hidden preset
  moves. The experiment users ran differed from the action they intended.
- Fresh simulation replays opened paused, making completed movement hard to see.
- The planning figure stretched to arbitrary targets without solving joints.
- Stable starting stances with no moves returned an unsuccessful result.
- A large header and overlapping 5 cm targets obscured the main interaction.

## Revised interaction

Start with four contacts, two clearly separated hand targets and no queued moves.
Dragging a limb adds a move; dragging a hold changes the route. Keep the move
list and test button beside the wall. Starting contacts and detailed hold
parameters are secondary controls. An example button performs a visible 20 cm
hand move through the same simulation service.

Use actual bounded whole-body inverse kinematics for a geometric preview.
Its convergence says whether this solver found a pose, not whether the contacts
can support it. The full MuJoCo attempt remains the source of motion, loads,
contact failures and the final result.

Fresh results autoplay after the initial settling period. Show the failed move,
what happened and an actionable adjustment. Preserve raw diagnostics for
inspection. Editing invalidates the previous result visibly.

## Verification

Cover direct pointer interaction, move bookkeeping, stale responses, replay
playback, stable zero-move routes and initialization failures. Check the actual
hosted API and browser flow. Any controller optimization must preserve complete
trajectory results on supported, unreachable, weak-grip and slipping examples.

The model still omits wall collision response, fingers, fatigue and dynamic
catches. A successful controller attempt is a simulated outcome, not a
guarantee about a real climber.

## Recorded checks

- 136 Python tests pass on MuJoCo 3.14.0 and 3.16.0.
- The controller norm optimization preserves the complete trajectory payload
  for the supported, unreachable, weak-grip and foot-slip examples. In one
  local measurement the supported sequence took 6.73 seconds instead of 7.48.
  These are local timings, not a hosted latency guarantee.
- Browser checks exercised the actual local service: the 20 cm example
  completed and selecting a target at 4 m returned the reach-limit explanation.
- Node interaction tests execute the shipped editor and replay handlers,
  including direct pointer drags, cancelled drops, asynchronous result fencing
  and playback progression. Native automation could click controls and inspect
  screenshots, but its pointer-drag API failed at the window-capture layer;
  direct dragging is covered by the handler tests rather than a native drag.
