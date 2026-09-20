"""
Unit tests for the pure-Python drone simulation module (drone/simulation.py).

These talk directly to the Drone class -- no Flask involved -- so they run
fast and pin down the autonomy state machine's transition rules precisely.
"""

import math
import unittest

from drone.simulation import Drone, DroneState, SimConfig, METERS_PER_DEG_LAT


def make_config(**overrides) -> SimConfig:
    base = dict(
        cruise_altitude=10.0,
        cruise_speed=5.0,
        climb_rate=5.0,
        waypoint_tolerance=0.5,
        hover_time=0.0,
        idle_drain_rate=0.02,
        moving_drain_rate=0.5,
        low_battery_threshold=20.0,
        gps_noise_m=1.5,
    )
    base.update(overrides)
    return SimConfig(**base)


class TestTakeoffAndRouting(unittest.TestCase):
    def setUp(self):
        self.drone = Drone(config=make_config())

    def test_initial_state_is_idle(self):
        self.assertEqual(self.drone.state, DroneState.IDLE)

    def test_set_mission_transitions_idle_to_taking_off(self):
        self.drone.set_mission([(10, 0, 10)], start=True)
        self.assertEqual(self.drone.state, DroneState.TAKING_OFF)

    def test_taking_off_transitions_to_en_route_after_reaching_altitude(self):
        self.drone.set_mission([(10, 0, 10)], start=True)
        for _ in range(30):
            self.drone.step(0.5)
            if self.drone.state != DroneState.TAKING_OFF:
                break
        self.assertEqual(self.drone.state, DroneState.EN_ROUTE)

    def test_drone_moves_toward_current_waypoint(self):
        # Waypoint is far enough away that the drone won't have already
        # reached it and turned back within this sampling window.
        self.drone.set_mission([(500, 0, 10)], start=True)
        for _ in range(10):
            self.drone.step(0.5)  # clear takeoff
        self.assertEqual(self.drone.state, DroneState.EN_ROUTE)
        x_before = self.drone.x
        self.drone.step(1.0)
        self.assertGreater(self.drone.x, x_before)

    def test_drone_advances_to_next_waypoint_when_within_tolerance(self):
        self.drone.set_mission([(1, 0, 10), (2, 0, 10)], start=True)
        reached_second = False
        for _ in range(200):
            self.drone.step(0.2)
            if self.drone.current_index >= 1:
                reached_second = True
                break
            if self.drone.state == DroneState.RETURNING_HOME:
                break
        self.assertTrue(reached_second, "drone never advanced past the first waypoint")


class TestBattery(unittest.TestCase):
    def test_battery_drains_over_time_while_idle(self):
        drone = Drone(config=make_config())
        start_battery = drone.battery
        for _ in range(10):
            drone.step(1.0)
        self.assertLess(drone.battery, start_battery)

    def test_battery_drains_faster_while_moving_than_while_idle(self):
        idle_drone = Drone(config=make_config())
        idle_start = idle_drone.battery
        idle_drone.step(5.0)
        idle_drain = idle_start - idle_drone.battery

        moving_drone = Drone(config=make_config())
        moving_drone.set_mission([(200, 0, 10)], start=True)
        moving_start = moving_drone.battery
        for _ in range(10):
            moving_drone.step(0.5)  # takeoff + en_route, definitely "moving"
        moving_drain = moving_start - moving_drone.battery

        self.assertGreater(moving_drain, idle_drain)

    def test_low_battery_forces_returning_home_from_en_route(self):
        drone = Drone(config=make_config())
        drone.set_mission([(1000, 0, 10)], start=True)
        for _ in range(30):
            drone.step(0.5)
        self.assertIn(drone.state, (DroneState.TAKING_OFF, DroneState.EN_ROUTE))

        drone.battery = 5.0
        drone.step(0.1)
        self.assertEqual(drone.state, DroneState.RETURNING_HOME)

    def test_low_battery_forces_returning_home_from_hovering(self):
        drone = Drone(config=make_config(hover_time=5.0))
        drone.set_mission([(1, 0, 10), (2, 0, 10)], start=True)
        reached_hover = False
        for _ in range(200):
            drone.step(0.2)
            if drone.state == DroneState.HOVERING:
                reached_hover = True
                break
        self.assertTrue(reached_hover)

        drone.battery = 5.0
        drone.step(0.1)
        self.assertEqual(drone.state, DroneState.RETURNING_HOME)

    def test_low_battery_forces_returning_home_from_taking_off(self):
        drone = Drone(config=make_config())
        drone.set_mission([(10, 0, 10)], start=True)
        self.assertEqual(drone.state, DroneState.TAKING_OFF)

        drone.battery = 5.0
        drone.step(0.1)
        self.assertEqual(drone.state, DroneState.RETURNING_HOME)

    def test_low_battery_has_no_effect_once_landed(self):
        drone = Drone(config=make_config())
        drone.state = DroneState.LANDED
        drone.battery = 5.0
        drone.step(0.1)
        self.assertEqual(drone.state, DroneState.LANDED)


class TestAbortAndMissionCompletion(unittest.TestCase):
    def test_abort_is_a_noop_once_landed(self):
        # abort() explicitly early-returns for LANDED (see Drone.abort). A
        # ground crew hitting "abort" after the drone has already landed
        # shouldn't re-trigger a low_battery_abort reason or move anything.
        drone = Drone(config=make_config())
        drone.state = DroneState.LANDED
        drone.abort()
        self.assertEqual(drone.state, DroneState.LANDED)
        self.assertIsNone(drone._last_abort_reason)

    def test_set_mission_rejects_empty_waypoint_list(self):
        # set_mission() raises directly on an empty list; the Flask layer
        # (tests/test_app.py) covers the HTTP 400 wrapping of this, but the
        # underlying Drone-level contract wasn't pinned down on its own.
        drone = Drone(config=make_config())
        with self.assertRaises(ValueError):
            drone.set_mission([], start=True)
        # Rejecting the mission must not have side effects on drone state.
        self.assertEqual(drone.state, DroneState.IDLE)
        self.assertEqual(drone.waypoints, [])

    def test_abort_forces_return_home_regardless_of_progress(self):
        drone = Drone(config=make_config())
        drone.set_mission([(1, 0, 10), (500, 500, 10)], start=True)
        for _ in range(10):
            drone.step(0.5)
        self.assertNotEqual(drone.state, DroneState.RETURNING_HOME)

        drone.abort()
        self.assertEqual(drone.state, DroneState.RETURNING_HOME)
        # Progress toward the mission waypoints is irrelevant to the abort itself.
        self.assertLess(drone.current_index, len(drone.waypoints))

    def test_mission_completes_and_lands_after_visiting_all_waypoints(self):
        config = make_config(
            cruise_altitude=5.0,
            cruise_speed=10.0,
            climb_rate=10.0,
            hover_time=0.0,
            idle_drain_rate=0.0,
            moving_drain_rate=0.0,
            low_battery_threshold=5.0,
        )
        drone = Drone(config=config)
        drone.set_mission([(5, 0, 5), (5, 5, 5)], start=True)

        landed = False
        for _ in range(3000):
            drone.step(0.1)
            if drone.state == DroneState.LANDED:
                landed = True
                break

        self.assertTrue(landed, "drone never reached LANDED state")
        self.assertTrue(drone.mission_complete)
        self.assertAlmostEqual(drone.x, 0.0, delta=0.6)
        self.assertAlmostEqual(drone.y, 0.0, delta=0.6)
        self.assertAlmostEqual(drone.z, 0.0, delta=0.6)


class TestTelemetryAndGps(unittest.TestCase):
    def test_telemetry_contains_expected_fields(self):
        drone = Drone(config=make_config())
        t = drone.telemetry()
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
            self.assertIn(key, t)
        for key in ("x", "y", "z"):
            self.assertIn(key, t["position"])
        for key in ("lat", "lon"):
            self.assertIn(key, t["gps"])

    def test_gps_drift_stays_within_bounded_noise_range(self):
        config = make_config(gps_noise_m=2.0)
        drone = Drone(config=config)
        drone.x, drone.y = 100.0, 50.0
        drone._update_gps()

        ideal_lat = drone.home_lat + (50.0 / METERS_PER_DEG_LAT)
        ideal_lon = drone.home_lon + (
            100.0 / (METERS_PER_DEG_LAT * math.cos(math.radians(drone.home_lat)))
        )

        lat_diff_m = abs(drone.lat - ideal_lat) * METERS_PER_DEG_LAT
        lon_diff_m = abs(drone.lon - ideal_lon) * METERS_PER_DEG_LAT * math.cos(
            math.radians(drone.home_lat)
        )

        self.assertLessEqual(lat_diff_m, config.gps_noise_m + 1e-6)
        self.assertLessEqual(lon_diff_m, config.gps_noise_m + 1e-6)

    def test_gps_drift_never_produces_wild_jumps_over_many_steps(self):
        drone = Drone(config=make_config(gps_noise_m=2.0))
        drone.set_mission([(20, 0, 10)], start=True)
        max_jump_m = 0.0
        prev_lat, prev_lon = drone.lat, drone.lon
        for _ in range(50):
            drone.step(0.2)
            dlat_m = abs(drone.lat - prev_lat) * METERS_PER_DEG_LAT
            dlon_m = abs(drone.lon - prev_lon) * METERS_PER_DEG_LAT * math.cos(
                math.radians(drone.home_lat)
            )
            max_jump_m = max(max_jump_m, dlat_m, dlon_m)
            prev_lat, prev_lon = drone.lat, drone.lon
        # True motion per 0.2s tick is small (<= cruise_speed*dt) and noise is
        # bounded by +/- gps_noise_m on each axis independently, so the max
        # possible per-tick jump is bounded well under an arbitrary large value.
        self.assertLess(max_jump_m, 10.0)


if __name__ == "__main__":
    unittest.main()
