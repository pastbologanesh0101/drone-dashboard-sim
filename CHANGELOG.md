# Changelog

All notable changes to this project are documented here.

## [0.1.0] - Initial release

The first published version of the Autonomous Drone Dashboard, a
software-only simulated drone telemetry system (no real hardware — see the
README's disclaimer). This release included:

- `drone/simulation.py` — a dependency-free `Drone` class: 3D kinematics
  (position/velocity), a battery model that drains at different rates
  while idle vs. flying, simulated GPS lat/lon derived from local position
  plus bounded random jitter, an ordered waypoint mission list, and a
  `DroneState` autonomy state machine (`IDLE` → `TAKING_OFF` → `EN_ROUTE`
  → `HOVERING` → `RETURNING_HOME` → `LANDED`, with a `LOW_BATTERY_ABORT`
  transition reachable from any airborne state).
- `drone/app.py` — a Flask application factory exposing the simulation
  over HTTP: `GET /telemetry`, `POST /mission`, `POST /abort`, plus the
  `/` route serving the dashboard page.
- `run.py` — an entry point that starts a background thread stepping the
  simulation forward in real time, then serves the Flask app.
- A single-page vanilla-JS dashboard (`templates/index.html`,
  `static/dashboard.js`, `static/style.css`) that polls `/telemetry` and
  renders current values plus a 2D top-down canvas plot of the drone's
  position relative to its waypoints and home.
- `tests/test_simulation.py` and `tests/test_app.py` — unit tests covering
  state transitions, battery drain rates, low-battery forced abort from
  multiple states, mission completion/landing, GPS drift bounds, and the
  HTTP layer's request/response handling (22 tests total at release).
- `.github/workflows/tests.yml` — CI running the full test suite on
  Python 3.11 and 3.12 for every push and pull request.
- MIT `LICENSE`.

[0.1.0]: https://github.com/pastbologanesh0101/drone-dashboard-sim/commit/1ea0515
