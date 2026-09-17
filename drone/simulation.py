"""
Simulated drone telemetry + autonomy state machine.

IMPORTANT: This module is a pure-software simulation. It does not talk to any
real drone, flight controller, radio, or GPS receiver. Positions, battery
level, and GPS coordinates are all synthesized by simple kinematics and
random noise so that the rest of the project (Flask backend, dashboard) has
something realistic-looking to display and control.

The simulation is intentionally kept dependency-free (stdlib only) and
separate from Flask so it can be unit tested directly.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

# Meters per degree of latitude is ~constant on Earth's surface.
METERS_PER_DEG_LAT = 111_320.0

# Default simulated "home" launch point (an arbitrary point in San Francisco).
DEFAULT_HOME_LAT = 37.7749
DEFAULT_HOME_LON = -122.4194


class DroneState(Enum):
    """Autonomy state machine states.

    Transition rules (enforced in Drone.step / Drone.set_mission / Drone.abort):

      IDLE            -- set_mission(start=True) --> TAKING_OFF
      TAKING_OFF      -- reaches cruise altitude --> EN_ROUTE
      EN_ROUTE        -- reaches a waypoint (within tolerance) --> HOVERING
      HOVERING        -- hover_time elapsed, more waypoints left --> EN_ROUTE
      HOVERING        -- hover_time elapsed, no waypoints left --> RETURNING_HOME
      EN_ROUTE         -- last waypoint reached directly (hover_time == 0) --> RETURNING_HOME
      RETURNING_HOME  -- reaches home position (within tolerance) --> LANDED
      * (any airborne  -- battery <= low_battery_threshold --> LOW_BATTERY_ABORT
         state)                                             --> (forced immediately) RETURNING_HOME
      * (any non-LANDED state) -- abort() called --> LOW_BATTERY_ABORT --> RETURNING_HOME

    LOW_BATTERY_ABORT is intentionally momentary: entering it *always* forces
    an immediate transition to RETURNING_HOME. It exists as a named state so
    the "why did we start returning home" reason is explicit and testable,
    even though telemetry snapshots will only ever observe RETURNING_HOME
    right after.
    """

    IDLE = "idle"
    TAKING_OFF = "taking_off"
    EN_ROUTE = "en_route"
    HOVERING = "hovering"
    RETURNING_HOME = "returning_home"
    LANDED = "landed"
    LOW_BATTERY_ABORT = "low_battery_abort"


# States in which the drone is actually airborne / consuming "movement" power.
_FLYING_STATES = (
    DroneState.TAKING_OFF,
    DroneState.EN_ROUTE,
    DroneState.HOVERING,
    DroneState.RETURNING_HOME,
)

# States from which a low-battery event should force a return-home.
_ABORTABLE_STATES = (
    DroneState.TAKING_OFF,
    DroneState.EN_ROUTE,
    DroneState.HOVERING,
)


@dataclass
class SimConfig:
    """Tunable simulation parameters."""

    cruise_altitude: float = 30.0      # meters, target altitude while flying a mission
    cruise_speed: float = 8.0          # m/s horizontal speed toward a waypoint
    climb_rate: float = 4.0            # m/s vertical speed during takeoff/landing
    waypoint_tolerance: float = 1.5    # meters; "close enough" to a waypoint/home
    hover_time: float = 3.0            # seconds spent hovering at each waypoint
    idle_drain_rate: float = 0.05      # %/s battery drain just from electronics being on
    moving_drain_rate: float = 0.6     # %/s additional battery drain while flying
    low_battery_threshold: float = 20.0  # % battery at which low_battery_abort triggers
    gps_noise_m: float = 1.5           # meters; max simulated GPS jitter magnitude
    max_battery: float = 100.0


DEFAULT_CONFIG = SimConfig()

Waypoint = Tuple[float, float, float]  # (x, y, z) meters, relative to home


def _distance3(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def _move_toward(
    x: float, y: float, z: float, tx: float, ty: float, tz: float, speed: float, dt: float
) -> Tuple[float, float, float, float, float, float]:
    """Move (x, y, z) toward (tx, ty, tz) at `speed` m/s for `dt` seconds.

    Returns the new (x, y, z, vx, vy, vz). Clamps so the point never
    overshoots the target.
    """
    dist = _distance3(x, y, z, tx, ty, tz)
    if dist < 1e-9:
        return tx, ty, tz, 0.0, 0.0, 0.0
    step_dist = min(speed * dt, dist)
    ux, uy, uz = (tx - x) / dist, (ty - y) / dist, (tz - z) / dist
    nx, ny, nz = x + ux * step_dist, y + uy * step_dist, z + uz * step_dist
    if dt > 0:
        vx, vy, vz = (nx - x) / dt, (ny - y) / dt, (nz - z) / dt
    else:
        vx, vy, vz = 0.0, 0.0, 0.0
    return nx, ny, nz, vx, vy, vz


class Drone:
    """A single simulated drone: state, kinematics, battery, and GPS."""

    def __init__(
        self,
        config: Optional[SimConfig] = None,
        home_lat: float = DEFAULT_HOME_LAT,
        home_lon: float = DEFAULT_HOME_LON,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.config = config or SimConfig()
        self.home_lat = home_lat
        self.home_lon = home_lon
        self._rng = rng or random.Random()

        self.state: DroneState = DroneState.IDLE

        # Position/velocity in a local ENU-style frame, meters relative to home.
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Simulated GPS (with noise applied on top of true x/y).
        self.lat = home_lat
        self.lon = home_lon

        self.battery = self.config.max_battery

        self.waypoints: List[Waypoint] = []
        self.current_index = 0
        self.mission_complete = False
        self._hover_remaining = 0.0
        self._last_abort_reason: Optional[str] = None

        self.mission_time = 0.0
        self._update_gps()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_mission(self, waypoints: List[Waypoint], start: bool = True) -> None:
        """Replace the waypoint list and (optionally) begin flying it.

        waypoints: list of (x, y, z) tuples, meters relative to home.
        """
        if not waypoints:
            raise ValueError("mission requires at least one waypoint")

        self.waypoints = [
            (float(wp[0]), float(wp[1]), float(wp[2]) if len(wp) > 2 else self.config.cruise_altitude)
            for wp in waypoints
        ]
        self.current_index = 0
        self.mission_complete = False
        self._hover_remaining = 0.0

        if not start:
            return

        if self.state in (DroneState.IDLE, DroneState.LANDED):
            self.state = DroneState.TAKING_OFF
        elif self.state == DroneState.LOW_BATTERY_ABORT:
            # Shouldn't normally be observable, but handle defensively.
            self.state = DroneState.RETURNING_HOME
        else:
            # Already airborne (en_route/hovering/returning_home): redirect
            # immediately toward the new first waypoint.
            self.state = DroneState.EN_ROUTE

    def abort(self) -> None:
        """Force an immediate return-to-home, regardless of mission progress."""
        if self.state == DroneState.LANDED:
            return
        self._last_abort_reason = "manual_abort"
        self._force_low_battery_abort()

    def step(self, dt: float) -> None:
        """Advance the simulation by dt seconds."""
        if dt <= 0:
            return

        if self.state == DroneState.LANDED:
            # Motors off; nothing moves, nothing drains.
            self._update_gps()
            return

        moving = self.state in _FLYING_STATES
        self._drain_battery(dt, moving)

        if (
            self.state in _ABORTABLE_STATES
            and self.battery <= self.config.low_battery_threshold
        ):
            self._last_abort_reason = "low_battery"
            self._force_low_battery_abort()
            # Don't also run the newly-entered state's movement this same
            # tick -- that would let a just-triggered abort immediately
            # evaluate (and possibly satisfy) the "arrived home" check
            # before the drone has taken a single step toward home.
        elif self.state == DroneState.TAKING_OFF:
            self._step_taking_off(dt)
        elif self.state == DroneState.EN_ROUTE:
            self._step_en_route(dt)
        elif self.state == DroneState.HOVERING:
            self._step_hovering(dt)
        elif self.state == DroneState.RETURNING_HOME:
            self._step_returning_home(dt)

        self._update_gps()
        self.mission_time += dt

    def telemetry(self) -> dict:
        """Return a JSON-serializable snapshot of the current drone state."""
        return {
            "timestamp": time.time(),
            "mission_time_s": round(self.mission_time, 2),
            "state": self.state.value,
            "position": {"x": round(self.x, 3), "y": round(self.y, 3), "z": round(self.z, 3)},
            "velocity": {"vx": round(self.vx, 3), "vy": round(self.vy, 3), "vz": round(self.vz, 3)},
            "battery_percent": round(self.battery, 2),
            "gps": {"lat": round(self.lat, 6), "lon": round(self.lon, 6)},
            "altitude_m": round(self.z, 3),
            "mission": {
                "waypoints": [list(wp) for wp in self.waypoints],
                "current_waypoint_index": self.current_index,
                "complete": self.mission_complete,
            },
            "home": {"x": 0.0, "y": 0.0, "lat": self.home_lat, "lon": self.home_lon},
            "last_abort_reason": self._last_abort_reason,
        }

    # ------------------------------------------------------------------
    # Internal step helpers
    # ------------------------------------------------------------------

    def _drain_battery(self, dt: float, moving: bool) -> None:
        drain = self.config.idle_drain_rate * dt
        if moving:
            drain += self.config.moving_drain_rate * dt
        self.battery = max(0.0, min(self.config.max_battery, self.battery - drain))

    def _force_low_battery_abort(self) -> None:
        """Entering LOW_BATTERY_ABORT always immediately resolves to
        RETURNING_HOME -- this is the one required, non-optional transition.
        """
        self.state = DroneState.LOW_BATTERY_ABORT
        self.state = DroneState.RETURNING_HOME

    def _step_taking_off(self, dt: float) -> None:
        target_z = self.config.cruise_altitude
        self.x, self.y, self.z, self.vx, self.vy, self.vz = _move_toward(
            self.x, self.y, self.z, 0.0, 0.0, target_z, self.config.climb_rate, dt
        )
        if abs(self.z - target_z) <= self.config.waypoint_tolerance:
            self.z = target_z
            self.state = DroneState.EN_ROUTE

    def _current_waypoint(self) -> Optional[Waypoint]:
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def _step_en_route(self, dt: float) -> None:
        wp = self._current_waypoint()
        if wp is None:
            self.state = DroneState.RETURNING_HOME
            return

        tx, ty, tz = wp
        self.x, self.y, self.z, self.vx, self.vy, self.vz = _move_toward(
            self.x, self.y, self.z, tx, ty, tz, self.config.cruise_speed, dt
        )

        if _distance3(self.x, self.y, self.z, tx, ty, tz) <= self.config.waypoint_tolerance:
            self._arrive_at_waypoint()

    def _arrive_at_waypoint(self) -> None:
        if self.config.hover_time > 0:
            self.state = DroneState.HOVERING
            self._hover_remaining = self.config.hover_time
        else:
            self._advance_waypoint()

    def _step_hovering(self, dt: float) -> None:
        self.vx = self.vy = self.vz = 0.0
        self._hover_remaining -= dt
        if self._hover_remaining <= 0:
            self._advance_waypoint()

    def _advance_waypoint(self) -> None:
        self.current_index += 1
        if self.current_index >= len(self.waypoints):
            self.state = DroneState.RETURNING_HOME
        else:
            self.state = DroneState.EN_ROUTE

    def _step_returning_home(self, dt: float) -> None:
        # First close the horizontal distance at cruise altitude, then descend.
        horiz_dist = math.sqrt(self.x ** 2 + self.y ** 2)
        if horiz_dist > self.config.waypoint_tolerance:
            self.x, self.y, self.z, self.vx, self.vy, self.vz = _move_toward(
                self.x, self.y, self.z, 0.0, 0.0, self.z, self.config.cruise_speed, dt
            )
            return

        self.x, self.y, self.z, self.vx, self.vy, self.vz = _move_toward(
            self.x, self.y, self.z, 0.0, 0.0, 0.0, self.config.climb_rate, dt
        )
        if math.sqrt(self.x ** 2 + self.y ** 2) <= self.config.waypoint_tolerance and self.z <= (
            self.config.waypoint_tolerance
        ):
            self.x, self.y, self.z = 0.0, 0.0, 0.0
            self.vx = self.vy = self.vz = 0.0
            self.state = DroneState.LANDED
            if self.current_index >= len(self.waypoints):
                self.mission_complete = True

    def _update_gps(self) -> None:
        """Derive simulated lat/lon from local x/y plus bounded random noise."""
        noise_x = self._rng.uniform(-self.config.gps_noise_m, self.config.gps_noise_m)
        noise_y = self._rng.uniform(-self.config.gps_noise_m, self.config.gps_noise_m)
        noisy_x = self.x + noise_x
        noisy_y = self.y + noise_y
        self.lat = self.home_lat + (noisy_y / METERS_PER_DEG_LAT)
        self.lon = self.home_lon + (
            noisy_x / (METERS_PER_DEG_LAT * math.cos(math.radians(self.home_lat)))
        )
