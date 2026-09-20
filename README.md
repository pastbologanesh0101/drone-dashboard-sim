# Autonomous Drone Dashboard (Simulated)

**This project is a software simulation only.** There is no real drone, no
real flight controller, no radio telemetry link, and no real GPS receiver
anywhere in this codebase. `drone/simulation.py` synthesizes a plausible
drone's position, battery level, and GPS coordinates using simple kinematics
and bounded random noise, and drives it through an autonomy state machine.
The Flask backend exposes that simulated state over HTTP, and a small
vanilla-JS dashboard polls and visualizes it. If you are looking for
something that talks to actual hardware (MAVLink, PX4, ArduPilot, DJI SDKs,
etc.), this is not that — it's a self-contained backend/frontend exercise in
telemetry APIs, state machines, and live dashboards.

## What it actually does

- `drone/simulation.py` — a dependency-free `Drone` class: 3D position and
  velocity, battery level (drains over time, faster while flying), a
  simulated GPS lat/lon derived from position plus bounded jitter, a mission
  (an ordered list of waypoints), and a `DroneState` state machine.
- `drone/app.py` — a Flask app factory exposing the simulation over HTTP.
- `run.py` — starts a background thread that steps the simulation forward
  in real time, then serves the Flask app + dashboard.
- `templates/index.html`, `static/dashboard.js`, `static/style.css` — a
  single-page dashboard: current telemetry as text, and a top-down 2D
  canvas plot of the drone's position relative to its waypoints and home.

## Autonomy state machine

```
        set_mission(start=True)
IDLE ─────────────────────────────► TAKING_OFF
                                         │ reaches cruise altitude
                                         ▼
                    ┌──────────────► EN_ROUTE ─────┐
                    │  more waypoints left         │ reaches current
                    │                               │ waypoint (within
                HOVERING ◄─────────────────────────┘ tolerance)
                    │  hover_time elapsed,
                    │  no waypoints left
                    ▼
             RETURNING_HOME ───────────────────────► LANDED
                    ▲            reaches home (within tolerance)
                    │
   (from TAKING_OFF / EN_ROUTE / HOVERING)
     battery <= low_battery_threshold
              -> LOW_BATTERY_ABORT
              -> forced immediately to RETURNING_HOME
              (also triggered manually by POST /abort)
```

Key rules, enforced in `Drone.step()` / `Drone.set_mission()` / `Drone.abort()`:

- `set_mission(waypoints, start=True)` replaces the waypoint list and moves
  `IDLE`/`LANDED` → `TAKING_OFF` (or redirects an already-airborne drone
  straight to `EN_ROUTE` toward the new first waypoint).
- Each simulation step moves the drone toward its current target
  (cruise altitude during takeoff, current waypoint while `EN_ROUTE`, home
  while `RETURNING_HOME`) at a configured speed, and drains battery
  proportional to elapsed time, with an extra drain rate while actually
  flying.
- Reaching a waypoint within `waypoint_tolerance` triggers `HOVERING` for
  `hover_time` seconds, then either the next waypoint or, if that was the
  last one, `RETURNING_HOME`.
- **Whenever battery crosses `low_battery_threshold`** while airborne
  (`TAKING_OFF`, `EN_ROUTE`, or `HOVERING`), the drone enters
  `LOW_BATTERY_ABORT`, which unconditionally and immediately forces a
  transition to `RETURNING_HOME` — regardless of how much of the mission is
  left. `LOW_BATTERY_ABORT` is therefore never actually observed in
  telemetry; it exists to make the "why did it turn back" reason explicit
  and unit-testable (`Drone._force_low_battery_abort`,
  `telemetry()["last_abort_reason"]`).
- `abort()` forces the exact same `LOW_BATTERY_ABORT` → `RETURNING_HOME`
  transition on demand, from any state except `LANDED`.
- Reaching home (within tolerance) while `RETURNING_HOME` transitions to
  `LANDED`. If every waypoint had been visited first, `mission_complete`
  is set `True`.

## Running it

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python run.py
```

Then open <http://127.0.0.1:5000> in a browser. Pass `--battery 15` to
start the simulation already low on battery (handy for exercising the
low-battery return-home behavior without waiting), or `--port 8080` to
serve on a different port; run `python run.py --help` for details.

The dashboard polls
`GET /telemetry` once a second and redraws the position plot. Enter
waypoints in the text box as `x,y` (or `x,y,z`) pairs, one per line, and
click "Set mission & start". "Abort / return home" forces an immediate
return regardless of progress.

## HTTP API

- `GET /telemetry` — current simulated telemetry snapshot (see example
  below).
- `POST /mission` — body `{"waypoints": [{"x": 20, "y": 0}, {"x": 20, "y": 20}]}`
  (`z` optional per waypoint, defaults to cruise altitude). Replaces the
  waypoint list and starts the mission. Returns `400` if `waypoints` is
  missing, empty, has non-numeric coordinates, or has non-finite (`NaN`/
  `Infinity`) coordinates.
- `POST /abort` — no body required. Forces an immediate transition to
  `returning_home`.

### Example `GET /telemetry` response

```json
{
  "timestamp": 1731000000.123,
  "mission_time_s": 12.5,
  "state": "en_route",
  "position": {"x": 8.42, "y": 1.10, "z": 30.0},
  "velocity": {"vx": 5.9, "vy": 0.8, "vz": 0.0},
  "battery_percent": 91.35,
  "gps": {"lat": 37.774978, "lon": -122.419328},
  "altitude_m": 30.0,
  "mission": {
    "waypoints": [[20.0, 0.0, 30.0], [20.0, 20.0, 30.0]],
    "current_waypoint_index": 0,
    "complete": false
  },
  "home": {"x": 0.0, "y": 0.0, "lat": 37.7749, "lon": -122.4194},
  "last_abort_reason": null
}
```

## Tests

```bash
pip install -r requirements.txt pytest
pytest -v
```

`tests/test_simulation.py` calls the `Drone` class directly (takeoff
transitions, waypoint advancement, battery drain rates, low-battery forced
abort from multiple states, mission completion/landing, GPS drift bounds).
`tests/test_app.py` drives the same behavior through Flask's test client
(telemetry JSON shape, mission replacement, malformed-input rejection,
abort endpoint). CI (`.github/workflows/tests.yml`) runs the full suite on
Python 3.11, 3.12, and 3.13 on every push and pull request.

## Project layout

```
drone/
  simulation.py   # Drone model + state machine (framework-agnostic, unit-testable)
  app.py          # Flask app factory / HTTP routes
run.py            # entry point: background sim loop + Flask server
templates/index.html
static/dashboard.js, static/style.css
tests/test_simulation.py, tests/test_app.py
.github/workflows/tests.yml
```

## Troubleshooting / FAQ

**Why does the battery percentage drop faster once the drone takes off?**
`Drone._drain_battery()` applies `idle_drain_rate` (default `0.05` %/s) at
all times, plus `moving_drain_rate` (default `0.6` %/s) on top of that
whenever the drone is in a "flying" state (`TAKING_OFF`, `EN_ROUTE`,
`HOVERING`, `RETURNING_HOME`). So a drone just sitting `IDLE` on the pad
drains at 0.05 %/s (~33 minutes to empty), while an airborne one drains at
0.65 %/s (~2.5 minutes to empty) — both tunable via `SimConfig`.

**What actually happens at 0% battery?**
Two different things, and it's a common point of confusion. First, while
airborne, crossing `low_battery_threshold` (default 20%) forces an
automatic `LOW_BATTERY_ABORT` → `RETURNING_HOME` transition (see the state
machine above) — that's the "safety" behavior, and it happens at 20%, not
0%. Second, `_drain_battery()` clamps the raw percentage with
`max(0.0, ...)`, so the number itself never goes negative — but nothing in
`step()` re-checks battery once `RETURNING_HOME` (it's not in
`_ABORTABLE_STATES`), so a drone that somehow reaches literal 0% mid-return
just keeps returning at normal speed. There's no "falls out of the sky"
model here; battery is a depleting counter and a trigger for one
state transition, not a real energy budget that caps movement.

**Why is there no real GPS (or any real hardware)?**
Because this project is a dashboard/backend exercise, not a flight
controller. `Drone._update_gps()` derives `lat`/`lon` purely from the
simulated local `x`/`y` position plus bounded random jitter
(`SimConfig.gps_noise_m`, default ±1.5m) — there's no receiver, no NMEA
parsing, no satellites. If you came here looking for MAVLink/PX4/ArduPilot
integration, see the disclaimer below; this repo is deliberately just the
software layers around a synthetic telemetry feed.

**Why did my `POST /mission` get rejected with a 400?**
`waypoints` must be a non-empty list, each entry needs numeric `x` and `y`
(and optional numeric `z`), and coordinates must be finite — `NaN`/
`Infinity` are rejected too (a waypoint the drone can never measure itself
as "within tolerance" of would otherwise fly forever). See the HTTP API
section above for the exact error messages.

## Disclaimer

Again, to be explicit: this is a simulated telemetry feed and a simulated
autonomy state machine running entirely in software, built as a portfolio
project. It does not control, connect to, or represent any real unmanned
aircraft.
