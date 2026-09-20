"""
Flask backend for the Autonomous Drone Dashboard.

This module ONLY wires HTTP routes to the simulation in drone/simulation.py.
No real drone hardware, radio link, or GPS receiver is involved -- see
simulation.py and the project README for details.
"""

from __future__ import annotations

import math
import os

from flask import Flask, jsonify, render_template, request

from .simulation import Drone, SimConfig

_BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def create_app(drone: Drone | None = None) -> Flask:
    """Application factory. Pass a pre-built Drone (useful for tests) or
    leave it None to create a fresh one with default simulation settings.
    """
    app = Flask(
        __name__,
        template_folder=os.path.join(_BASE_DIR, "templates"),
        static_folder=os.path.join(_BASE_DIR, "static"),
    )
    app.drone = drone or Drone(config=SimConfig())

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/telemetry", methods=["GET"])
    def telemetry():
        return jsonify(app.drone.telemetry())

    @app.route("/mission", methods=["POST"])
    def mission():
        payload = request.get_json(silent=True) or {}
        raw_waypoints = payload.get("waypoints")

        if not isinstance(raw_waypoints, list) or len(raw_waypoints) == 0:
            return jsonify({"error": "waypoints must be a non-empty list"}), 400

        try:
            parsed = []
            for wp in raw_waypoints:
                x = float(wp["x"])
                y = float(wp["y"])
                z = float(wp.get("z", app.drone.config.cruise_altitude))
                parsed.append((x, y, z))
        except (KeyError, TypeError, ValueError):
            return (
                jsonify({"error": "each waypoint needs numeric 'x' and 'y' (optional 'z')"}),
                400,
            )

        # float("nan")/float("inf") succeed above (they're valid floats) but
        # would silently break the simulation's distance math -- a waypoint
        # at NaN/Infinity is never "within tolerance" of anything, so the
        # drone would fly toward it forever. Reject those explicitly with a
        # clearer error than a downstream math exception would give.
        if not all(math.isfinite(coord) for wp in parsed for coord in wp):
            return (
                jsonify({"error": "waypoint coordinates must be finite numbers (got NaN/Infinity)"}),
                400,
            )

        app.drone.set_mission(parsed, start=True)
        return jsonify(app.drone.telemetry()), 200

    @app.route("/abort", methods=["POST"])
    def abort():
        app.drone.abort()
        return jsonify(app.drone.telemetry()), 200

    return app
