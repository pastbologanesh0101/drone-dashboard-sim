# Contributing

Thanks for looking at this project. It's a small, software-only drone
telemetry simulation (no real hardware — see the README's disclaimer), so
the bar for contributions is: keep it dependency-free where it already is,
keep it tested, and keep the "this is simulated, not real" framing intact.

## Running tests

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt pytest
pytest -v
```

`tests/test_simulation.py` exercises the `Drone` class directly (no Flask).
`tests/test_app.py` drives the same behavior through Flask's test client.
Both must pass before you open a pull request — CI runs the same command
on every push and pull request (see `.github/workflows/tests.yml`).

## Code style

- `drone/simulation.py` stays **stdlib-only**. Don't add a dependency to it
  just to simplify some math — it's kept dependency-free on purpose so it's
  trivially unit-testable and portable.
- Match the existing style: type hints on public methods, docstrings that
  explain *why* (especially around state machine transitions), and small,
  single-purpose helper functions (`_move_toward`, `_distance3`, etc.)
  rather than long inline blocks.
- New `DroneState` transitions or validation rules should be documented in
  the docstring on `DroneState` and in the README's state machine section,
  not just in code comments.
- Flask routes in `drone/app.py` should validate their input and return a
  `400` with a clear JSON `{"error": "..."}` message on bad input, rather
  than letting an exception surface as a 500.

## Submitting changes

1. Fork the repo and create a branch for your change.
2. Add or update tests for whatever you change — a behavior change without
   a test won't be merged.
3. Run `pytest -v` locally and make sure it's green.
4. Open a pull request with a short description of *why* the change is
   needed, not just what it does. Reference any relevant state machine
   transition or endpoint by name.

Keep pull requests small and focused; it's much easier to review one
behavior change than five bundled together.
