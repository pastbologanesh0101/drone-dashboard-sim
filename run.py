"""
Entry point for running the Autonomous Drone Dashboard locally.

Starts a background thread that advances the simulated drone's physics
every `interval` seconds, then serves the Flask app. This is a SOFTWARE
SIMULATION -- there is no real drone or flight controller anywhere in this
project. See README.md for details.

Usage:
    python run.py
Then open http://127.0.0.1:5000 in a browser.
"""

from __future__ import annotations

import threading
import time

from drone.app import create_app

SIM_DT = 0.5        # seconds of simulated time advanced per tick
SIM_INTERVAL = 0.5  # real seconds between ticks


def start_background_simulation(app, dt: float = SIM_DT, interval: float = SIM_INTERVAL) -> threading.Thread:
    """Start a daemon thread that repeatedly steps app.drone forward."""

    stop_event = threading.Event()

    def _loop():
        while not stop_event.is_set():
            time.sleep(interval)
            app.drone.step(dt)

    thread = threading.Thread(target=_loop, name="drone-sim-loop", daemon=True)
    thread.stop_event = stop_event  # type: ignore[attr-defined]
    thread.start()
    return thread


def main() -> None:
    app = create_app()
    start_background_simulation(app)
    app.run(host="127.0.0.1", port=5000, debug=False)


if __name__ == "__main__":
    main()
