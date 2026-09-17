"""
Tests for the Flask HTTP layer (drone/app.py), exercised via Flask's test
client. These check the wiring between HTTP and the simulation module, not
the simulation physics itself (see tests/test_simulation.py for that).
"""

import unittest

from drone.app import create_app
from drone.simulation import Drone, DroneState, SimConfig


def make_test_app():
    config = SimConfig(
        cruise_altitude=10.0,
        cruise_speed=5.0,
        climb_rate=5.0,
        waypoint_tolerance=0.5,
        hover_time=0.0,
        idle_drain_rate=0.01,
        moving_drain_rate=0.3,
        low_battery_threshold=15.0,
        gps_noise_m=1.0,
    )
    drone = Drone(config=config)
    app = create_app(drone=drone)
    app.testing = True
    return app, drone


class TestTelemetryEndpoint(unittest.TestCase):
    def setUp(self):
        self.app, self.drone = make_test_app()
        self.client = self.app.test_client()

    def test_telemetry_returns_200_and_well_formed_json(self):
        resp = self.client.get("/telemetry")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.content_type, "application/json")

        data = resp.get_json()
        for key in (
            "state",
            "position",
            "velocity",
            "battery_percent",
            "gps",
            "altitude_m",
            "mission",
            "home",
        ):
            self.assertIn(key, data)
        for key in ("x", "y", "z"):
            self.assertIn(key, data["position"])
        for key in ("lat", "lon"):
            self.assertIn(key, data["gps"])
        self.assertEqual(data["state"], "idle")


class TestMissionEndpoint(unittest.TestCase):
    def setUp(self):
        self.app, self.drone = make_test_app()
        self.client = self.app.test_client()

    def test_post_mission_sets_waypoints_and_starts_flight(self):
        resp = self.client.post(
            "/mission", json={"waypoints": [{"x": 10, "y": 0}, {"x": 10, "y": 10}]}
        )
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(len(self.drone.waypoints), 2)
        self.assertEqual(self.drone.state, DroneState.TAKING_OFF)

    def test_post_mission_replaces_existing_waypoint_list(self):
        self.client.post("/mission", json={"waypoints": [{"x": 10, "y": 0}, {"x": 10, "y": 10}]})
        self.assertEqual(len(self.drone.waypoints), 2)

        resp = self.client.post("/mission", json={"waypoints": [{"x": 50, "y": 50, "z": 15}]})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(len(self.drone.waypoints), 1)
        self.assertEqual(self.drone.waypoints[0], (50.0, 50.0, 15.0))
        self.assertEqual(self.drone.current_index, 0)

    def test_post_mission_rejects_empty_waypoint_list(self):
        resp = self.client.post("/mission", json={"waypoints": []})
        self.assertEqual(resp.status_code, 400)

    def test_post_mission_rejects_malformed_waypoint(self):
        resp = self.client.post("/mission", json={"waypoints": [{"x": "not-a-number", "y": 0}]})
        self.assertEqual(resp.status_code, 400)


class TestAbortEndpoint(unittest.TestCase):
    def setUp(self):
        self.app, self.drone = make_test_app()
        self.client = self.app.test_client()

    def test_abort_forces_returning_home_regardless_of_progress(self):
        self.client.post("/mission", json={"waypoints": [{"x": 100, "y": 100}]})
        for _ in range(5):
            self.drone.step(0.5)
        self.assertNotEqual(self.drone.state, DroneState.RETURNING_HOME)

        resp = self.client.post("/abort")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(self.drone.state, DroneState.RETURNING_HOME)
        data = resp.get_json()
        self.assertEqual(data["state"], "returning_home")


if __name__ == "__main__":
    unittest.main()
