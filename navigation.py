"""
Core navigation logic as a small state machine driven by the IMU + odometry,
with the ultrasonic sensors as a safety fallback.

  Modes:
    DRIVING   -- cruise forward, holding the lane's target heading with the IMU.
    TURNING   -- end-of-lane U-turn (executed as a blocking maneuver in main.py).
    DISPOSING -- reached the known pit: dump the collected blocks out the back.

Instead of following the right wall and watching the front sensors for the end
of a lane, the car now:
  * holds a straight heading using IMU feedback (HEADING_HOLD_GAIN), and
  * tracks how far it has driven down the lane by dead-reckoning
    (DRIVE_CM_PER_S x time), turning when it has covered the arena length.
The front ultrasonic stop is only a fallback for when a wall appears sooner than
odometry expects. Position is integrated in a start-relative frame so the car
knows when it has reached the fixed pit location (PIT_X_CM, PIT_Y_CM).

This module is still pure (no hardware, no I/O beyond the passed-in readings /
yaw), so it can be unit-tested and simulated on a laptop.
"""

import math
import statistics
from dataclasses import dataclass
from enum import Enum, auto

import config as default_config
from actuators import Collector


INF = float("inf")


def angle_diff(a, b):
    """Shortest signed difference a - b, normalized to (-180, 180] degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


class Action(Enum):
    FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    DISPOSE = auto()
    STOP = auto()


class Mode(Enum):
    DRIVING = auto()
    TURNING = auto()
    DISPOSING = auto()


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
    def __init__(self, cfg=default_config, collector=None):
        self.cfg = cfg
        self.collector = collector if collector is not None else Collector(cfg)

        # -- pose, in the start-relative frame (see config.START_*/PIT_*) -------
        # heading 0 = the car's initial facing; forward unit vector is
        # (sin(heading), cos(heading)), so +y is straight ahead at start and +x
        # is to the car's right. Position is in centimetres.
        self.x = cfg.START_X_CM
        self.y = cfg.START_Y_CM
        self.heading_rel = 0.0       # current heading, degrees, relative to start
        self.target_heading = 0.0    # heading we want to hold down this lane
        self.lane_distance = 0.0     # cm driven down the current lane so far
        self.mode = Mode.DRIVING

        # Heading reference: origin yaw captured by set_origin(); until then (or
        # with no IMU) we have no absolute heading, so heading-hold is disabled
        # and the car cruises open-loop straight.
        self._yaw0 = None
        self._has_heading = False

        # Serpentine sweep: alternate the U-turn direction at each end wall so the
        # car snakes across the arena. The first turn (config.SERPENTINE_FIRST_TURN)
        # sets which way it steps sideways -- RIGHT from a bottom-left start.
        first = getattr(cfg, "SERPENTINE_FIRST_TURN", "left").lower()
        self._next_serpentine_turn = Turn.RIGHT if first == "right" else Turn.LEFT
        self._last_turn = None       # direction of the turn currently being executed

        # Pit hysteresis: once we dump we don't re-trigger until we leave the zone.
        self._pit_handled = False

        # Odometry bookkeeping: remember what we last commanded so the next tick
        # can integrate how far/where the car moved during dt.
        self._last_speed = 0.0
        self._last_action = Action.STOP

    # -- setup --------------------------------------------------------------

    def set_origin(self, yaw0):
        """Record the startup heading as 0. yaw0=None -> no IMU, heading-hold off."""
        if yaw0 is None:
            self._has_heading = False
            print("[nav] no IMU heading at startup -> open-loop straight driving")
            return
        self._yaw0 = yaw0
        self.heading_rel = 0.0
        self._has_heading = True
        print(f"[nav] heading origin set (yaw0={yaw0:.1f}deg -> heading 0)")

    # -- public API ---------------------------------------------------------

    def decide(self, readings, yaw=None, dt=0.0):
        """Map sensors + IMU yaw + elapsed dt to a Command, updating pose/mode.

        readings: {sensor_name: distance_cm} (missing/out-of-range = float('inf'))
        yaw:      IMU heading in degrees, or None if unavailable this tick
        dt:       seconds since the previous decide() call (for odometry)
        """
        cfg = self.cfg
        self._update_pose(yaw, dt)

        front = self._front_distance(readings)

        # 1) At the pit? Highest priority: stop sweeping and dispose.
        if self._at_pit() and self._should_dispose():
            self.mode = Mode.DISPOSING
            self._pit_handled = True
            return self._remember(Command(
                Action.DISPOSE, speed=0.0,
                reason=f"reached pit ({self.collector}) -> dispose"))

        # 2) End of lane? Either odometry says we've covered the arena length, or
        #    the front sensors (fallback) see a wall closer than expected.
        lane_done = self.lane_distance >= (cfg.ARENA_LENGTH_CM - cfg.LANE_END_MARGIN_CM)
        wall_ahead = front <= cfg.FRONT_STOP_DISTANCE_CM
        if lane_done or wall_ahead:
            turn = self._next_serpentine_turn
            self._advance_serpentine()
            self._last_turn = turn
            self.mode = Mode.TURNING
            action = Action.TURN_LEFT if turn is Turn.LEFT else Action.TURN_RIGHT
            trigger = ("odometry: lane length reached" if lane_done
                       else f"FALLBACK front wall {front:.0f}cm")
            return self._remember(Command(
                action, speed=cfg.TURN_SPEED,
                reason=f"end of lane ({trigger}), turn {turn.name.lower()}"))

        # 3) Cruise forward, holding heading (IMU) unless wall-follow is selected.
        self.mode = Mode.DRIVING
        speed = cfg.SLOW_SPEED if front <= cfg.FRONT_SLOW_DISTANCE_CM else cfg.DRIVE_SPEED
        steer = self._cruise_trim(readings)
        return self._remember(Command(
            Action.FORWARD, speed=speed, steer=steer, reason="cruising"))

    def complete_turn(self):
        """Call after main.py finishes the blocking U-turn maneuver.

        Folds the maneuver's net effect into the pose: heading reverses ~180 deg,
        the car has shifted one LANE_WIDTH_CM sideways, and a fresh lane starts.
        (The turn runs as a blocking maneuver with no decide() ticks, so its
        motion is applied here rather than integrated tick-by-tick.)
        """
        cfg = self.cfg
        pre = self.target_heading
        # First 90deg spin sets the intermediate heading the lane-shift is driven
        # along; turn_right rotates clockwise (heading +90), turn_left -90.
        step = 90.0 if self._last_turn is Turn.RIGHT else -90.0
        mid = pre + step
        self.x += cfg.LANE_WIDTH_CM * math.sin(math.radians(mid))
        self.y += cfg.LANE_WIDTH_CM * math.cos(math.radians(mid))
        self.target_heading = angle_diff(pre + 180.0, 0.0)
        self.lane_distance = 0.0
        self.mode = Mode.DRIVING

    def complete_dispose(self):
        """Call after main.py finishes the disposal maneuver: bucket emptied."""
        self.collector.reset()
        self.mode = Mode.DRIVING

    def rel_heading(self, yaw):
        """Convert a raw IMU yaw to a start-relative heading, or None if no IMU."""
        if yaw is None or not self._has_heading:
            return None
        return angle_diff(yaw, self._yaw0)

    def bearing_to_pit(self):
        """Heading (start-relative deg) pointing from the car toward the pit."""
        dx = self.cfg.PIT_X_CM - self.x
        dy = self.cfg.PIT_Y_CM - self.y
        return math.degrees(math.atan2(dx, dy))  # matches forward=(sin,cos)

    def pose_str(self):
        return (f"pos=({self.x:6.1f},{self.y:6.1f}) hdg={self.heading_rel:6.1f} "
                f"tgt={self.target_heading:6.1f} lane={self.lane_distance:5.1f}")

    # -- internals ----------------------------------------------------------

    def _remember(self, cmd):
        """Record the command so the next tick can integrate the motion it caused."""
        self._last_action = cmd.action
        self._last_speed = cmd.speed if cmd.action is Action.FORWARD else 0.0
        return cmd

    def _update_pose(self, yaw, dt):
        """Advance heading from the IMU and position from the last drive command."""
        cfg = self.cfg
        if yaw is not None and self._has_heading:
            self.heading_rel = angle_diff(yaw, self._yaw0)

        # Only forward motion moves the car between ticks (turns are blocking and
        # handled in complete_turn). Convert the last speed fraction to cm using
        # the measured DRIVE_CM_PER_S (calibrated at DRIVE_SPEED).
        if self._last_action is Action.FORWARD and dt > 0.0 and cfg.DRIVE_SPEED > 0.0:
            dist = cfg.DRIVE_CM_PER_S * (self._last_speed / cfg.DRIVE_SPEED) * dt
            self.x += dist * math.sin(math.radians(self.heading_rel))
            self.y += dist * math.cos(math.radians(self.heading_rel))
            self.lane_distance += dist

        # Leaving the pit zone re-arms disposal for the next visit.
        if self._pit_handled and not self._at_pit():
            self._pit_handled = False

    def _at_pit(self):
        dx = self.cfg.PIT_X_CM - self.x
        dy = self.cfg.PIT_Y_CM - self.y
        return math.hypot(dx, dy) <= self.cfg.PIT_ARRIVAL_RADIUS_CM

    def _should_dispose(self):
        """Whether to dump now that we're at the pit.

        TODO(servo): gate on self.collector.is_full() once the collection servo
        reports a real block count. For now the count is a stub (always 0), so we
        dispose on arrival at the pit as long as we haven't already this visit.
        """
        return not self._pit_handled

    def _front_distance(self, readings):
        """Median of the front sensors -- robust to one drifting/dropped sensor."""
        values = [readings.get(name, INF) for name in self.cfg.FRONT_SENSORS]
        return statistics.median(values)

    def _advance_serpentine(self):
        self._next_serpentine_turn = (
            Turn.LEFT if self._next_serpentine_turn is Turn.RIGHT else Turn.RIGHT
        )

    def _cruise_trim(self, readings):
        """Steering trim while cruising: IMU heading-hold (default) or wall-follow."""
        cfg = self.cfg
        if cfg.USE_WALL_FOLLOW:
            return self._wall_follow_trim(readings.get("right_front", INF),
                                          readings.get("right_rear", INF))
        if not self._has_heading:
            return 0.0  # no IMU -> can't hold a heading; drive open-loop straight
        # Positive steer = toward the car's right. If our heading is left of the
        # target (target - heading > 0), steer right to come back.
        err = angle_diff(self.target_heading, self.heading_rel)
        trim = cfg.HEADING_HOLD_GAIN * err
        return max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM, trim))

    def _wall_follow_trim(self, rf, rr):
        """Legacy: keep parallel to, and RIGHT_TARGET_DISTANCE_CM from, the right wall.

        Returns a steer trim in [-MAX_STEER_TRIM, +MAX_STEER_TRIM] where positive
        steers right (toward the wall). Only used when USE_WALL_FOLLOW is True.
        """
        cfg = self.cfg
        if rf == INF or rr == INF:
            return 0.0
        dist_err = ((rf + rr) / 2.0) - cfg.RIGHT_TARGET_DISTANCE_CM
        angle_err = rf - rr
        trim = cfg.STEER_CORRECTION_GAIN * (dist_err + angle_err)
        return max(-cfg.MAX_STEER_TRIM, min(cfg.MAX_STEER_TRIM, trim))
