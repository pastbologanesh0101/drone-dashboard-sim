"""
Entry point for running the Autonomous Drone Dashboard locally.

Starts a background thread that advances the simulated drone's physics
every `interval` seconds, then serves the Flask app. This is a SOFTWARE
SIMULATION -- there is no real drone or flight controller anywhere in this
project. See README.md for details.

Usage:
    python run.py
    python run.py --battery 15      # start already low on battery
    python run.py --port 8000
Then open http://127.0.0.1:<port> in a browser.
"""

from __future__ import annotations

import argparse
import threading
import time

from drone.app import create_app
from drone.simulation import Drone, SimConfig

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


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--battery",
        type=float,
        default=100.0,
        metavar="PERCENT",
        help="starting battery percentage, 0-100 (default: 100). Useful for "
        "testing low-battery/return-home behavior without waiting for a "
        "real drain.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="port to serve the dashboard on (default: 5000)",
    )
    args = parser.parse_args(argv)
    if not 0.0 <= args.battery <= 100.0:
        parser.error("--battery must be between 0 and 100")
    return args


def main(argv=None) -> None:
    args = parse_args(argv)
    drone = Drone(config=SimConfig())
    drone.battery = args.battery
    app = create_app(drone=drone)
    start_background_simulation(app)
    app.run(host="127.0.0.1", port=args.port, debug=False)


if __name__ == "__main__":
    main()
