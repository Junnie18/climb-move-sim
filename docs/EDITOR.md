# Interactive editor and simulation service

The browser owns an editable route, a monotonically increasing revision and
the latest simulation result. Changes invalidate results, including undo and
reset. Each request captures a route snapshot and revision; a response for an
older revision cannot become the current verdict.

## Planning and simulation

The front-view planner edits hold positions in the wall's x/z plane. Y remains
the hold's stand-off from the wall. Its figure illustrates the assigned contacts,
not a validated body pose. **Start pose** changes `route.start`; **Add move**
appends to `route.moves`. The explicit move list is the solver's input.

The server uses the same `Route`, controller and trajectory exporter as the
command-line tools. It does not substitute a prerecorded trajectory or infer a
pass from the planner drawing. Success means this controller completed under
the model's assumptions. Body-wall contact response and experimentally
calibrated human strength remain outside the model.

## HTTP interface

- `GET /api/health`: runtime status, engine version and worker availability.
- `POST /api/simulate`: JSON `{ "route": { ... }, "frameEvery": 20 }`.
  A successful HTTP response is `{ "trajectory": { ... } }`; the simulation
  itself can still report `trajectory.success: false`.
- API errors return JSON `{ "error": "..." }`, including invalid input,
  busy workers and timeouts.

Requests are limited to 128 KiB, 40 holds and 12 single-limb moves. Coordinates
are bounded to x ±3 m, y 0–1 m and z 0.05–5 m. Hold names use letters, digits,
hyphens and underscores. The service checks finite numeric values and hold
properties before compiling the model.

A child process performs each simulation. The default deadline is 90 seconds;
the service terminates a timed-out worker. A process-local lock rejects concurrent
work with HTTP 429. Hosted instances can scale independently, so this is not a
global admission limit or a durable job queue.

Responses contain at most 900 rendered frames and are limited to 4 MB. Frame
decimation preserves the first and last state. Force peaks come from the full
integration trace, not from the decimated display samples. Metadata reports the
original and returned frame counts.

## Local use

```bash
python scripts/serve_editor.py
```

Open http://127.0.0.1:8765/. The default binds only to loopback. Run `--help` for
host, port, timeout and additional allowed-origin settings. The server exposes
the demo asset directory, not the whole repository. The HTTP API accepts
same-origin browser requests and the GitHub Pages origin; other frontends must
be explicitly allowed. CORS is a browser policy, not API authentication.

## Hosted use

`api/index.py` adapts the same service for Vercel. `vercel.json` serves the demo
assets and routes `/api/*` to Python. No account keys or model API credentials
are required by the application. Submitted routes are held in memory for the
request and no application route database is maintained. Use **Export JSON**
to retain your work and **Import route** to reopen it. Imported route files
are validated and limited to 128 KiB before replacing the current draft.

GitHub Pages and file-based copies can edit routes and play bundled examples.
They require a reachable simulation service for a fresh solve. The connection
settings show the configured endpoint and connectivity errors.
