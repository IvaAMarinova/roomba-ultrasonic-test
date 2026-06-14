"""
Core navigation logic: turn the five ultrasonic readings into a single drive
command. This module is pure (no hardware, no I/O) so it can be unit-tested and
simulated on a laptop before ever touching the car.

Inside a closed rectangle there is only one obstacle the car ever meets:

  1. Wall straight ahead -> the lane has ended, rotate to start the next lane.

Everything else is "cruise forward", with a gentle steering trim that keeps the
car parallel to and a fixed distance from the right wall.
"""

from dataclasses import dataclass
from enum import Enum, auto

import config as default_config


INF = float("inf")


class Action(Enum):
    FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    STOP = auto()


class Turn(Enum):
    LEFT = auto()
    RIGHT = auto()


@dataclass
class Command:
    """What the controller wants the car to do this tick."""
    action: Action
    speed: float = 0.0
    steer: float = 0.0       # only used with FORWARD: -1 = full left .. +1 = full right
    reason: str = ""         # human-readable explanation, handy for logs/tests


class NavigationController:
    def __init__(self, cfg=default_config):
        self.cfg = cfg
        # "continue route" stub: serpentine sweeps alternate their turn
        # direction at each end wall so the car snakes across the arena.
        self._next_serpentine_turn = Turn.RIGHT

    # -- public API ---------------------------------------------------------

    def decide(self, readings):
        """Map a dict of {sensor_name: distance_cm} to a Command.

        Missing or out-of-range sensors should be passed as float('inf').
        """
        cfg = self.cfg
        front = self._front_distance(readings)
        rf = readings.get("right_front", INF)
        rr = readings.get("right_rear", INF)

        front_blocked = front <= cfg.FRONT_STOP_DISTANCE_CM
        right_wall = min(rf, rr) <= cfg.RIGHT_WALL_DISTANCE_CM

        # 1) Wall straight ahead: the lane is finished, rotate in place.
        if front_blocked:
            turn = self._choose_front_turn(right_wall)
            self._advance_serpentine()
            action = Action.TURN_LEFT if turn is Turn.LEFT else Action.TURN_RIGHT
            return Command(action, speed=cfg.TURN_SPEED,
                           reason="wall ahead: end of lane")

        # 2) Otherwise cruise forward, trimming heading against the right wall.
        speed = cfg.DRIVE_SPEED
        if front <= cfg.FRONT_SLOW_DISTANCE_CM:
            speed = cfg.SLOW_SPEED
        steer = self._wall_follow_trim(rf, rr)
        return Command(Action.FORWARD, speed=speed, steer=steer,
                       reason="cruising")

    # -- internals ----------------------------------------------------------

    def _front_distance(self, readings):
        """Nearest obstacle seen by any of the three forward sensors."""
        return min(readings.get(name, INF) for name in self.cfg.FRONT_SENSORS)

    def _choose_front_turn(self, right_wall):
        # In a closed rectangle the sweep must alternate its turn direction at
        # each end wall -- that alternation is what walks the car across the
        # arena -- so the serpentine schedule is the primary source of truth.
        scheduled = self._next_serpentine_turn
        # We only have right-side sensors, so the one case we can positively
        # veto is "schedule says turn right, but there's a wall on the right"
        # (the car has reached the far edge of the arena): turn left instead.
        if scheduled is Turn.RIGHT and right_wall:
            return Turn.LEFT
        return scheduled

    def _advance_serpentine(self):
        self._next_serpentine_turn = (
            Turn.LEFT if self._next_serpentine_turn is Turn.RIGHT else Turn.RIGHT
        )

    def _wall_follow_trim(self, rf, rr):
        """Keep parallel to, and RIGHT_TARGET_DISTANCE_CM from, the right wall.

        Returns a steer trim in [-MAX_STEER_TRIM, +MAX_STEER_TRIM] where
        positive steers right (toward the wall).
        """
        cfg = self.cfg
        if rf == INF or rr == INF:
            return 0.0
        # Too far from the wall -> steer right (toward it).
        dist_err = ((rf + rr) / 2.0) - cfg.RIGHT_TARGET_DISTANCE_CM
        # Nose angled away from the wall (rf > rr) -> steer right to straighten.
        angle_err = rf - rr
        trim = cfg.STEER_CORRECTION_GAIN * (dist_err + angle_err)
        return max(-cfg.MAX_STEER_TRIM, min(cfg.MAX_STEER_TRIM, trim))
