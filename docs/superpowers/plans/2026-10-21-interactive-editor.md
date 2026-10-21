# Interactive climbing editor

The user wants a polished route editor: move holds, move hands and feet to holds, and evaluate the edited climb using the actual simulator.

## Design

- Keep the existing 3D replay and force telemetry. Add a front-view planning canvas with draggable holds and four distinct limb handles.
- Separate starting-pose assignment from appending a move. Display the resulting sequence explicitly.
- Provide numeric coordinate inputs and click-based assignment as alternatives to dragging. Include undo, reset, JSON export, and loading a template.
- An edit invalidates the previous result. A result received after subsequent edits cannot be presented as the current route's verdict.
- Run edited routes through a Python HTTP service. `POST /api/simulate` accepts `{route, frameEvery:20}` and returns `{trajectory}`. `GET /api/health` identifies the available runtime. The service serves the editor at its root for local use.
- The static GitHub Pages site requires a configured simulation service. Explain this dependency and show connection failures clearly. Recorded examples must remain identified as recorded examples.
- Validate and bound requests. Allow one simulation at a time; terminate jobs that exceed the timeout.
- Report controller completion and concrete failure categories. Keep the existing body-wall collision limitation visible. Do not promise real human feasibility.

## Implementation

1. Backend: route serialization, shared trajectory export, isolated simulation worker, HTTP service, input and integration tests.
2. Frontend: route state, drag interaction, sequence editing, route inspector, request lifecycle, replay integration, state regression tests.
3. Integration: regenerate examples with their route definitions; run full Python and browser-state tests; exercise a real edit and simulation in the UI.
4. Delivery: update usage documentation and CI, publish the updated static site and source, and start a local service. Deploy a public simulation service only when an authenticated existing hosting path is available.

## Acceptance checks

- Moving a hold changes exported coordinates and invalidates results.
- Reassigning a starting limb differs from appending a move.
- Undo and reset restore route state without reusing a stale verdict.
- Running the supported example returns actual successful simulation telemetry.
- An unreachable edit returns the simulator's failure and a useful explanation.
- Malformed routes, empty-frame failures, busy workers, timeouts, and unavailable service produce usable errors.
- Existing physics tests and replay behavior remain intact.
