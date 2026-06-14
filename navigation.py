"""
Core navigation logic: turn the five ultrasonic readings into a single drive
command. This module is pure (no hardware, no I/O) so it can be unit-tested and
simulated on a laptop before ever touching the car.

Inside a closed rectangle there is only one obstacle the car ever meets:

  1. Wall straight ahead -> the lane has ended, rotate to start the next lane.

Everything else is "cruise forward", with a gentle steering trim that keeps the
car parallel to and a fixed distance from the right wall.
"""

import statistics
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
        # First turn is LEFT, then it alternates left/right/left/...
        self._next_serpentine_turn = Turn.LEFT

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

        # 1) Wall straight ahead: the lane is finished, rotate in place.
        #    Turns simply alternate left/right (starting left), which is what
        #    walks the serpentine across the arena.
        if front_blocked:
            turn = self._next_serpentine_turn
            self._advance_serpentine()
            action = Action.TURN_LEFT if turn is Turn.LEFT else Action.TURN_RIGHT
            return Command(action, speed=cfg.TURN_SPEED,
                           reason=f"wall ahead: end of lane, turn {turn.name.lower()}")

        # 2) Otherwise cruise forward, trimming heading against the right wall.
        speed = cfg.DRIVE_SPEED
        if front <= cfg.FRONT_SLOW_DISTANCE_CM:
            speed = cfg.SLOW_SPEED
        steer = self._wall_follow_trim(rf, rr)
        return Command(Action.FORWARD, speed=speed, steer=steer,
                       reason="cruising")

    # -- internals ----------------------------------------------------------

    def _front_distance(self, readings):
        """Distance to a wall ahead, robust to one bad sensor.

        Uses the MEDIAN of the front sensors, so a single sensor that drifts
        (reads too short, or drops out to 'inf') is ignored -- the car only
        acts on a wall when at least two of the three front sensors agree.
        (For 3 sensors, median <= X is exactly "at least two read <= X".)
        """
        values = [readings.get(name, INF) for name in self.cfg.FRONT_SENSORS]
        return statistics.median(values)

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
