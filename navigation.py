"""
Core navigation logic as a small state machine that FUSES the IMU and the front
ultrasonics for localization, so position doesn't depend on wheel odometry over a
bumpy floor.

  Modes:
    DRIVING   -- cruise forward, holding the lane's target heading with the IMU.
    TURNING   -- end-of-lane U-turn (executed as a blocking maneuver in main.py).
    DISPOSING -- reached the known pit: dump the collected blocks out the back.

Localization model (start-relative frame: start bottom-left (0,0), heading 0 = +y
along ARENA_LENGTH, +x = right across ARENA_WIDTH):

  * heading        -- IMU yaw (reliable, slip-proof).
  * cross-lane x   -- lane counting: x = START_X + sweep_sign * lane_index * LANE_WIDTH.
                      Each U-turn steps exactly one lane, bump or no bump.
  * along-lane y   -- the FRONT WALL: the gap the front sensors measure to the end
                      wall IS the position. A close reading is only believed to be
                      the wall when K-of-N front sensors AGREE (rejects a narrow
                      block) AND it is near where odometry expects the wall (rejects
                      a mid-lane object). When no wall is seen we BRIDGE with
                      odometry (DRIVE_CM_PER_S x time) for a bounded time.
  * end of lane    -- primary: the believed wall within FRONT_STOP_DISTANCE_CM,
                      held WALL_PERSIST_TICKS ticks. Backstop: odometry distance if
                      the wall is never seen at all.

Time (DRIVE_CM_PER_S) is thus only a bridge + a rough prior, never the sole source
of position. This module is pure (no hardware, no I/O beyond the passed-in
readings / yaw), so it can be unit-tested and simulated on a laptop.
"""

import math
import statistics
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import config as default_config
from actuators import Collector


INF = float("inf")


def angle_diff(a, b):
    """Shortest signed difference a - b, normalized to (-180, 180] degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


class Action(Enum):
    FORWARD = auto()
    REVERSE = auto()         # back straight up (benchmark return home, no 180)
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    SPIN_LEFT = auto()       # 90 deg left only (hill: wall stop -> spin -> next leg)
    FACE_HEADING = auto()    # blocking spin to Command.face_heading (e.g. 180 deg)
    DISPOSE = auto()
    ALIGN_CENTER = auto()    # blocking: square ±x, balance side sensors, then face_heading
    ALIGN_PIT = auto()       # blocking: align at start wall, then dump
    STOP = auto()


class Mode(Enum):
    DRIVING = auto()
    TURNING = auto()
    DISPOSING = auto()
    DONE = auto()


class Phase(Enum):
    """Macro stages for HILL_MODE (slope -> sweep -> descend -> pit)."""
    CLIMB_FIRST = auto()
    APPROACH_FAR_WALL = auto()
    APPROACH_LEFT_WALL = auto()
    SWEEP = auto()
    APPROACH_HILL_CENTER = auto()
    DESCEND = auto()
    ALIGN_AT_FAR_WALL = auto()   # blocking lateral align before turnaround at the far wall
    BENCHMARK_OUT = auto()       # flat: centre line to far wall, collecting
    BENCHMARK_RETURN = auto()    # reverse straight back to the start wall (no 180)
    BENCHMARK_ALIGN_PIT = auto() # at start wall: face ±x, center with side sensors


class Turn(Enum):
    LEFT = auto()
    RIGHT = auto()


@dataclass
class Command:
    """What the controller wants the car to do this tick."""
    action: Action
    speed: float = 0.0
    steer: float = 0.0       # FORWARD/REVERSE only: -1 = full left .. +1 = full right
    reason: str = ""         # human-readable explanation, handy for logs/tests
    wall_stop: bool = False    # pause + front scoop lift before this maneuver
    face_heading: Optional[float] = None  # for FACE_HEADING: start-relative target deg


class NavigationController:
    def __init__(self, cfg=default_config, collector=None):
        self.cfg = cfg
        self.collector = collector if collector is not None else Collector(cfg)

        # -- pose, in the start-relative frame ---------------------------------
        hill = getattr(cfg, "HILL_MODE", False)
        self.x = (getattr(cfg, "HILL_CLIMB_X_CM", cfg.START_X_CM) if hill
                  else cfg.START_X_CM)
        self.y = cfg.START_Y_CM
        self.heading_rel = 0.0       # current heading, degrees, relative to start
        self.target_heading = 0.0    # heading we want to hold down this lane
        self.lane_distance = 0.0     # cm along the current lane (wall-anchored when a wall is seen, else odometry)
        self.mode = Mode.DRIVING

        # Heading reference (set by set_origin()); until then heading-hold is off.
        self._yaw0 = None
        self._has_heading = False
        self._prev_heading_rel = 0.0

        # Serpentine sweep + which way it steps sideways. From a bottom-left start
        # the first turn is RIGHT and lanes march +x; from bottom-right, LEFT / -x.
        first = getattr(cfg, "SERPENTINE_FIRST_TURN", "left").lower()
        self._next_serpentine_turn = Turn.RIGHT if first == "right" else Turn.LEFT
        self._sweep_sign = 1.0 if first == "right" else -1.0
        self._last_turn = None
        self._lane_index = 0

        # Which lane the pit sits on (it's on the start wall at PIT_X). Disposal is
        # gated to this lane so a generous arrival radius can't fire on a neighbour.
        self._pit_lane = (round((cfg.PIT_X_CM - cfg.START_X_CM)
                                / (self._sweep_sign * cfg.LANE_WIDTH_CM))
                          if cfg.LANE_WIDTH_CM else 0)

        # Pit hysteresis: once we dump we don't re-trigger until we leave the zone.
        self._pit_handled = False
        self._return_origin_y = None   # y at far wall when benchmark return begins
        self._rear_persist = 0         # consecutive ticks the rear wall-stop has held
        self._post_align_heading = None
        self._post_align_phase = None

        # Coverage: once the last lane (NUM_LANES-1) is finished, we're done.
        self._done = False

        # Hill strategy phase (None when HILL_MODE is off).
        self.phase = Phase.CLIMB_FIRST if getattr(cfg, "HILL_MODE", False) else None

        # Sideways hill sweep: lanes run along ARENA_WIDTH, step along +y.
        self._sweep_transverse = False
        self._sweep_origin_y = cfg.START_Y_CM

        # Wall-fusion state (updated every tick by _update_pose).
        self.front_wall_cm = INF     # believed distance to the end wall, or INF
        self.front_agree = 0         # how many front sensors agreed this tick
        self.pos_source = "BRIDGE"   # "WALL" = y from a believed wall, "BRIDGE" = from odometry
        self._wall_persist = 0       # consecutive ticks the wall-stop has held

        # Odometry bookkeeping for the bridge / expectation prior.
        self._last_speed = 0.0
        self._last_action = Action.STOP

    @property
    def collecting(self):
        """True when the front scoop should run lift cycles."""
        if getattr(self.cfg, "HILL_BENCHMARK_MODE", False):
            return self.phase is Phase.BENCHMARK_OUT
        if not getattr(self.cfg, "HILL_MODE", False):
            return True
        return self.phase is Phase.SWEEP

    def _benchmark_mode(self):
        return (getattr(self.cfg, "HILL_MODE", False)
                and getattr(self.cfg, "HILL_BENCHMARK_MODE", False))

    @property
    def wants_climb_shovel(self):
        """Intermediate scoop height for slope legs (FRONT_SERVO_CLIMB_PULSE_MS)."""
        if not getattr(self.cfg, "HILL_MODE", False):
            return False
        return self.phase in (Phase.CLIMB_FIRST, Phase.DESCEND,
                              Phase.BENCHMARK_RETURN)

    @property
    def wants_full_up_shovel(self):
        """Scoop fully raised when a hill phase needs maximum clearance."""
        return False

    def note_blocking_maneuver(self):
        """Call after a blocking U-turn / dispose so the next tick does not odometry-bridge a long gap."""
        self._last_action = Action.STOP
        self._last_speed = 0.0
        self._wall_persist = 0
        self._rear_persist = 0
        self.pos_source = "BRIDGE"

    # -- setup --------------------------------------------------------------

    def set_origin(self, yaw0):
        """Record the startup heading as 0. yaw0=None -> no IMU, heading-hold off."""
        if yaw0 is None:
            self._has_heading = False
            print("[nav] no IMU heading at startup -> open-loop straight driving")
            return
        self._yaw0 = yaw0
        self.heading_rel = 0.0
        self._prev_heading_rel = 0.0
        self._has_heading = True
        print(f"[nav] heading origin set (yaw0={yaw0:.1f}deg -> heading 0)")

    # -- public API ---------------------------------------------------------

    def decide(self, readings, yaw=None, dt=0.0):
        """Map sensors + IMU yaw + elapsed dt to a Command, updating pose/mode."""
        self._update_pose(readings, yaw, dt)
        if getattr(self.cfg, "HILL_MODE", False):
            return self._decide_hill(readings, yaw, dt)
        return self._decide_serpentine(readings)

    def _decide_serpentine(self, readings):
        """Original full-arena serpentine with mid-sweep pit disposal."""
        cfg = self.cfg

        # 0) Whole arena swept? Latch STOP and stay stopped.
        if self._done:
            self.mode = Mode.DONE
            return self._remember(Command(Action.STOP, reason="coverage complete"))

        # 1) At the pit? Highest priority: stop sweeping and dispose.
        if self._at_pit() and self._should_dispose():
            self.mode = Mode.DISPOSING
            self._pit_handled = True
            return self._remember(Command(
                Action.DISPOSE, speed=0.0,
                reason=f"reached pit ({self.collector}) -> dispose"))

        wall_trigger, odo_backstop, end_dist, end_agree, trigger = (
            self._lane_end_state(readings))

        if wall_trigger or odo_backstop:
            expected_gap = max(0.0, cfg.ARENA_LENGTH_CM - self.lane_distance)
            wall_plausible = (end_dist >= expected_gap - cfg.WALL_EXPECT_TOL_CM)
            standoff_aligned = end_dist >= expected_gap - cfg.FRONT_STOP_DISTANCE_CM
            inferred_ld = max(0.0, min(cfg.ARENA_LENGTH_CM - end_dist, cfg.ARENA_LENGTH_CM))
            inferred_near_far_end = (inferred_ld >= (cfg.ARENA_LENGTH_CM
                                                     - cfg.LANE_END_MARGIN_CM
                                                     - cfg.FRONT_STOP_DISTANCE_CM))
            odo_matches_inferred = (abs(inferred_ld - self.lane_distance)
                                    <= cfg.WALL_EXPECT_TOL_CM)
            inferred_standoff = (end_dist <= cfg.FRONT_STOP_DISTANCE_CM
                                 and inferred_near_far_end and odo_matches_inferred)
            if wall_trigger and not wall_plausible and not standoff_aligned and not inferred_standoff:
                self._reanchor_lane_from_wall(end_dist)
            # Was that the end of the LAST lane? Then the arena is swept -> done.
            if self._lane_index >= cfg.NUM_LANES - 1:
                self._done = True
                self.mode = Mode.DONE
                return self._remember(Command(
                    Action.STOP,
                    reason=f"coverage complete: swept all {cfg.NUM_LANES} lanes"))
            turn = self._next_serpentine_turn
            self._advance_serpentine()
            self._last_turn = turn
            self.mode = Mode.TURNING
            action = Action.TURN_LEFT if turn is Turn.LEFT else Action.TURN_RIGHT
            return self._remember(Command(
                action, speed=cfg.TURN_SPEED,
                reason=f"end of lane ({trigger}), turn {turn.name.lower()}",
                wall_stop=wall_trigger))

        return self._cmd_cruise(readings, reason="cruising")

    def _decide_hill(self, readings, yaw, dt):
        """Centre climb -> sideways sweep -> centre descend -> dump at pit."""
        cfg = self.cfg

        if self._done:
            self.mode = Mode.DONE
            return self._remember(Command(Action.STOP, reason="run complete"))

        if self.phase is Phase.CLIMB_FIRST:
            if self.y >= cfg.HILL_TOP_Y_CM:
                self.lane_distance = self.y - cfg.START_Y_CM
                if self._benchmark_mode():
                    self.phase = Phase.BENCHMARK_OUT
                    print(f"[benchmark] top of slope (y={self.y:.0f}) -> far wall")
                else:
                    self.phase = Phase.APPROACH_FAR_WALL
                    print(f"[hill] top of slope (y={self.y:.0f}) -> drive to wall")
            return self._cmd_cruise(readings, reason="climbing slope (centre)",
                                     hold_heading=False)

        if self.phase is Phase.BENCHMARK_OUT:
            return self._benchmark_out(readings)

        if self.phase is Phase.BENCHMARK_RETURN:
            return self._benchmark_return(readings)

        if self.phase is Phase.BENCHMARK_ALIGN_PIT:
            if self._pit_handled:
                self._done = True
                self.mode = Mode.DONE
                return self._remember(Command(Action.STOP, reason="benchmark dump done"))
            return self._remember(Command(
                Action.ALIGN_PIT, reason="benchmark align pit center"))

        if self.phase in (Phase.APPROACH_FAR_WALL, Phase.APPROACH_LEFT_WALL):
            return self._hill_wall_then_spin_left(readings)

        if self.phase is Phase.SWEEP:
            if (self._lane_index == 0 and self.lane_distance < 10.0
                    and abs(angle_diff(self.target_heading, self.heading_rel))
                    > getattr(cfg, "ORIENT_SKIP_DEG", 15.0)):
                return self._hill_spin_left_cmd("align for sideways sweep")
            wall_trigger, odo_backstop, end_dist, end_agree, trigger = (
                self._lane_end_state(readings))
            if wall_trigger or odo_backstop:
                if self._at_right_edge(readings) and self._at_sweep_half():
                    self.phase = Phase.APPROACH_HILL_CENTER
                    self._wall_persist = 0
                    print("[hill] right edge at mid-length -> drive to hill centre")
                    return self._hill_spin_left_cmd("right edge -> spin left")
                if wall_trigger and trigger.startswith("wall") and "contact" not in trigger:
                    self._reanchor_lane_from_wall(end_dist)
                if self._lane_index >= (
                        getattr(cfg, "HILL_SWEEP_NUM_LANES", cfg.NUM_LANES) - 1):
                    self.phase = Phase.APPROACH_HILL_CENTER
                    self._wall_persist = 0
                    print("[hill] swept to mid-length -> drive to hill centre")
                    return self._hill_spin_left_cmd("sweep half done -> spin left")
                turn = self._next_serpentine_turn
                self._advance_serpentine()
                self.mode = Mode.TURNING
                action = (Action.TURN_LEFT if turn is Turn.LEFT
                          else Action.TURN_RIGHT)
                return self._remember(Command(
                    action, speed=cfg.TURN_SPEED,
                    reason=f"end of lane ({trigger}), turn {turn.name.lower()}",
                    wall_stop=wall_trigger))
            return self._cmd_cruise(readings, reason="sweeping sideways")

        if self.phase is Phase.APPROACH_HILL_CENTER:
            return self._hill_approach_center(readings)

        if self.phase is Phase.DESCEND:
            wall_trigger, odo_backstop, end_dist, end_agree, trigger = (
                self._lane_end_state(readings))
            at_start = (self.y <= cfg.FRONT_STOP_DISTANCE_CM + 5.0
                        or (wall_trigger and self.y <= cfg.HILL_TOP_Y_CM))
            if at_start or (wall_trigger and self.target_heading == 180.0
                            and self.y <= cfg.HILL_TOP_Y_CM):
                self._pit_handled = True
                self.mode = Mode.DISPOSING
                print(f"[hill] start wall (y={self.y:.0f}) -> turn 180 and dump")
                return self._remember(Command(
                    Action.DISPOSE, reason="start wall -> dump", wall_stop=True))
            return self._cmd_cruise(readings, reason="descending slope (centre)")

        return self._cmd_cruise(readings, reason="cruising")

    def _benchmark_out(self, readings):
        """Centre line along +y to the far wall; scoop down and collecting."""
        cfg = self.cfg
        end_dist, end_agree = self._lane_end_wall(readings)
        if (end_agree >= cfg.FRONT_AGREE_MIN_COUNT
                and end_dist <= cfg.FRONT_STOP_DISTANCE_CM):
            self._reanchor_lane_from_wall(end_dist)
            self.phase = Phase.BENCHMARK_RETURN
            self.lane_distance = 0.0
            self._return_origin_y = self.y
            self.target_heading = 0.0    # stay facing the far wall; reverse home
            self.note_blocking_maneuver()
            print(f"[benchmark] far wall (y={self.y:.0f}) -> reverse home (no 180)")
            return self._remember(Command(
                Action.STOP, reason=f"benchmark far wall {end_dist:.0f}cm",
                wall_stop=True))
        return self._cmd_cruise(readings, reason="benchmark: to far wall",
                                hold_heading=False)

    def _cmd_reverse_cruise(self, reason=""):
        """Back straight up at reverse speed, open loop (no heading trim)."""
        cfg = self.cfg
        self.mode = Mode.DRIVING
        speed = getattr(cfg, "BENCHMARK_REVERSE_SPEED", cfg.SLOW_SPEED)
        return self._remember(Command(
            Action.REVERSE, speed=speed, reason=reason))

    def _rear_wall(self, readings):
        """Distance to the wall behind + agreeing count, from the back sensors."""
        cfg = self.cfg
        finite = []
        for name in getattr(cfg, "BACK_SENSORS", ()):
            spec = cfg.SENSORS.get(name, {})
            if not spec.get("enabled", True):
                continue
            d = readings.get(name, INF)
            if d != INF:
                finite.append(d)
        min_count = getattr(cfg, "BACK_AGREE_MIN_COUNT", 2)
        if len(finite) < min_count:
            return INF, len(finite)
        tol = getattr(cfg, "BACK_AGREE_TOL_CM", cfg.FRONT_AGREE_TOL_CM)
        if max(finite) - min(finite) <= tol:
            return statistics.median(finite), len(finite)
        return INF, len(finite)

    def _benchmark_return(self, readings):
        """Reverse straight home, nose still on the far wall, IMU holding heading 0.

        No 180 turn: the back sensors watch for the start wall (K-of-2 agree,
        held REAR_WALL_PERSIST_TICKS, only after BENCHMARK_MIN_RETURN_CM of
        travel so junk at departure can't fire it), with odometry y as the
        backstop if they never agree. Front readings are ignored here -- they
        see the far wall, which is behind us in travel terms.
        """
        cfg = self.cfg
        rear_stop = getattr(cfg, "BACK_STOP_DISTANCE_CM", 25.0)
        min_return = getattr(cfg, "BENCHMARK_MIN_RETURN_CM", 55.0)
        rear_dist, rear_agree = self._rear_wall(readings)
        rear_hold = (rear_agree >= getattr(cfg, "BACK_AGREE_MIN_COUNT", 2)
                     and rear_dist <= rear_stop
                     and self.lane_distance >= min_return)
        self._rear_persist = self._rear_persist + 1 if rear_hold else 0
        wall_home = (self._rear_persist
                     >= getattr(cfg, "REAR_WALL_PERSIST_TICKS", 2))
        odo_home = self.y <= rear_stop
        if wall_home or odo_home:
            src = (f"rear wall {rear_dist:.0f}cm (x{rear_agree} agree)"
                   if wall_home else "odometry backstop")
            self.phase = Phase.BENCHMARK_ALIGN_PIT
            self.mode = Mode.DISPOSING
            print(f"[benchmark] start wall ({src}, y={self.y:.0f}, "
                  f"ld={self.lane_distance:.0f}, "
                  f"blocks={self.collector.count}) -> align pit")
            return self._remember(Command(
                Action.ALIGN_PIT, reason=f"benchmark align pit ({src})"))
        return self._cmd_reverse_cruise(reason="benchmark: reverse home")

    def _hill_wall_then_spin_left(self, readings):
        """Drive until front sensors see the wall, then spin 90 deg left.

        Sensor-only stop: hill approach legs are short and odometry is still
        climbing-relative, so the full-lane fusion in _lane_end_state would
        reject a real wall contact and let the car ram it.
        """
        cfg = self.cfg
        end_dist, end_agree = self._lane_end_wall(readings)
        if (end_agree >= cfg.FRONT_AGREE_MIN_COUNT
                and end_dist <= cfg.FRONT_STOP_DISTANCE_CM):
            self._reanchor_lane_from_wall(end_dist)
            trigger = f"wall {end_dist:.0f}cm (x{end_agree} agree)"
            if self.phase is Phase.APPROACH_FAR_WALL:
                self.phase = Phase.APPROACH_LEFT_WALL
                print("[hill] wall ahead -> spin left, seek left wall")
            else:
                self.reset_sweep_transverse(origin_y=self.y)
                self.phase = Phase.SWEEP
                print(f"[hill] left wall (x={self.x:.0f}) -> sideways sweep")
            return self._hill_spin_left_cmd(trigger, wall_stop=True)
        return self._cmd_cruise(readings, reason="driving to wall", hold_heading=False)

    def _hill_odometry_only(self):
        """Hill climb/approach: trust integrated odometry, not wall re-anchors."""
        cfg = self.cfg
        if not getattr(cfg, "HILL_MODE", False):
            return False
        return self.phase in (Phase.CLIMB_FIRST, Phase.APPROACH_FAR_WALL,
                              Phase.APPROACH_LEFT_WALL, Phase.BENCHMARK_OUT,
                              Phase.BENCHMARK_RETURN)

    def _hill_spin_left_cmd(self, reason, wall_stop=False):
        self.mode = Mode.TURNING
        return self._remember(Command(
            Action.SPIN_LEFT, speed=self.cfg.TURN_SPEED, reason=reason,
            wall_stop=wall_stop))

    def _hill_approach_center(self, readings):
        """From the right edge: spin left to face -x, creep to hill centre, spin to descend."""
        cfg = self.cfg
        center_x = getattr(cfg, "HILL_CLIMB_X_CM", cfg.ARENA_WIDTH_CM / 2.0)

        if abs(angle_diff(self.target_heading, -90.0)) > 5.0:
            return self._hill_spin_left_cmd("face -x for hill centre")

        if self.x > center_x + 5.0:
            return self._cmd_cruise(readings, reason="driving to hill centre")

        self._sweep_transverse = False
        self.target_heading = 180.0
        if abs(angle_diff(self.heading_rel, 180.0)) > cfg.ORIENT_SKIP_DEG:
            return self._hill_spin_left_cmd("face downhill")
        self.lane_distance = cfg.ARENA_LENGTH_CM - (self.y - cfg.START_Y_CM)
        self.phase = Phase.DESCEND
        self.note_blocking_maneuver()
        print(f"[hill] at centre (x={self.x:.0f}, y={self.y:.0f}) -> descend")
        return self._cmd_cruise(readings, reason="descending slope (centre)")

    def _cruise_speed(self):
        """Full speed only on the hill climb; everything else is slow."""
        cfg = self.cfg
        if getattr(cfg, "HILL_MODE", False) and self.phase is Phase.CLIMB_FIRST:
            return cfg.DRIVE_SPEED
        return cfg.SLOW_SPEED

    def _cmd_cruise(self, readings, reason="", extra_steer=0.0, hold_heading=True):
        """Forward at cruise speed with IMU heading-hold (+ optional wall-hug trim)."""
        cfg = self.cfg
        self.mode = Mode.DRIVING
        speed = self._cruise_speed()
        trim = self._cruise_trim(readings) if hold_heading else 0.0
        steer = trim + extra_steer
        steer = max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM, steer))
        return self._remember(Command(
            Action.FORWARD, speed=speed, steer=steer, reason=reason))

    def _lane_axis_length(self):
        """Lane length along the current heading (width when driving ±x, length when ±y)."""
        cfg = self.cfg
        if self._heading_is_across():
            return cfg.ARENA_WIDTH_CM
        return cfg.ARENA_LENGTH_CM

    def _heading_is_across(self):
        return (abs(math.sin(math.radians(self.target_heading)))
                >= abs(math.cos(math.radians(self.target_heading))))

    def _is_transverse_sweep(self):
        return self._sweep_transverse and self.phase is Phase.SWEEP

    def _at_sweep_half(self):
        half_y = getattr(self.cfg, "HILL_SWEEP_HALF_Y_CM",
                         self.cfg.ARENA_LENGTH_CM / 2.0)
        return self.y >= half_y - self.cfg.LANE_WIDTH_CM

    def _lane_end_state(self, readings):
        """Return (wall_trigger, odo_backstop, end_dist, end_agree, trigger_reason)."""
        cfg = self.cfg
        lane_len = self._lane_axis_length()
        end_dist, end_agree = self._lane_end_wall(readings)
        expected_gap = max(0.0, lane_len - self.lane_distance)
        wall_plausible = (end_dist >= expected_gap - cfg.WALL_EXPECT_TOL_CM)
        heading_ok = (not self._has_heading
                      or abs(angle_diff(self.target_heading, self.heading_rel))
                      <= getattr(cfg, "WALL_HEADING_ALIGN_DEG", 30.0))
        contact_close = getattr(cfg, "WALL_CONTACT_STOP_CM", 25.0)
        standoff_aligned = end_dist >= expected_gap - cfg.FRONT_STOP_DISTANCE_CM
        inferred_ld = max(0.0, min(lane_len - end_dist, lane_len))
        inferred_near_far_end = (inferred_ld >= (lane_len
                                                 - cfg.LANE_END_MARGIN_CM
                                                 - cfg.FRONT_STOP_DISTANCE_CM))
        odo_matches_inferred = (abs(inferred_ld - self.lane_distance)
                                <= cfg.WALL_EXPECT_TOL_CM)
        inferred_standoff = (end_dist <= cfg.FRONT_STOP_DISTANCE_CM
                             and inferred_near_far_end
                             and odo_matches_inferred)
        near_far_end = (self.lane_distance + end_dist
                        >= lane_len - cfg.WALL_EXPECT_TOL_CM)
        odo_still_near_start = (self.lane_distance < lane_len * 0.35)
        contact_hold = (end_dist <= contact_close
                        and (near_far_end or odo_still_near_start))
        end_hold = (end_agree >= cfg.FRONT_AGREE_MIN_COUNT
                    and end_dist <= cfg.FRONT_STOP_DISTANCE_CM
                    and heading_ok
                    and not self._at_pit()
                    and (wall_plausible or standoff_aligned or contact_hold
                         or inferred_standoff))
        self._wall_persist = self._wall_persist + 1 if end_hold else 0
        wall_trigger = self._wall_persist >= cfg.WALL_PERSIST_TICKS
        odo_backstop = (self.pos_source != "WALL"
                        and self.lane_distance >= (lane_len - cfg.LANE_END_MARGIN_CM)
                        and heading_ok)
        if wall_trigger:
            if wall_plausible or standoff_aligned or inferred_standoff:
                trigger = f"wall {end_dist:.0f}cm (x{end_agree} agree)"
            elif contact_hold:
                trigger = (f"wall contact {end_dist:.0f}cm "
                           f"(x{end_agree} agree, odometry lag)")
            else:
                trigger = f"wall {end_dist:.0f}cm (x{end_agree} agree)"
        elif odo_backstop:
            trigger = "odometry backstop (no wall seen)"
        else:
            trigger = ""
        return wall_trigger, odo_backstop, end_dist, end_agree, trigger

    def _reanchor_lane_from_wall(self, end_dist):
        cfg = self.cfg
        lane_len = self._lane_axis_length()
        self.lane_distance = max(0.0, min(lane_len - end_dist, lane_len))
        if self._is_transverse_sweep() or self._heading_is_across():
            self._sync_lateral_pose()
            return
        forward = math.cos(math.radians(self.target_heading)) >= 0.0
        self.y = (cfg.START_Y_CM + self.lane_distance if forward
                  else cfg.START_Y_CM + cfg.ARENA_LENGTH_CM - self.lane_distance)

    def _at_right_edge(self, readings):
        cfg = self.cfg
        if self._is_transverse_sweep() and math.sin(math.radians(self.target_heading)) >= 0:
            end_dist, agree = self._lane_end_wall(readings)
            if (agree >= cfg.FRONT_AGREE_MIN_COUNT
                    and self.lane_distance + end_dist
                    >= cfg.ARENA_WIDTH_CM - cfg.RIGHT_EDGE_MARGIN_CM):
                return True
        if self.x + cfg.LANE_WIDTH_CM >= cfg.ARENA_WIDTH_CM - cfg.RIGHT_EDGE_MARGIN_CM:
            return True
        if readings:
            d = readings.get("front_right", INF)
            if d != INF and d <= cfg.RIGHT_WALL_STOP_CM:
                return True
        return False

    def complete_face_heading(self, target_heading):
        """Call after a blocking FACE_HEADING spin."""
        self.target_heading = target_heading
        self.lane_distance = 0.0
        self.mode = Mode.DRIVING
        self.note_blocking_maneuver()
        self._prev_heading_rel = self.heading_rel

    def complete_align_center(self, face_heading):
        """Call after lateral wall align + spin to the departure heading."""
        self.target_heading = face_heading
        self.complete_face_heading(face_heading)
        if self._post_align_phase is not None:
            self.phase = self._post_align_phase
            self._post_align_phase = None
        self._post_align_heading = None
        if self.phase is Phase.BENCHMARK_RETURN:
            self.lane_distance = 0.0

    def complete_spin_left(self):
        """Call after a 90 deg left spin: update heading and start a fresh lane leg."""
        self.target_heading = angle_diff(self.target_heading - 90.0, 0.0)
        self.lane_distance = 0.0
        self.mode = Mode.DRIVING
        self.note_blocking_maneuver()
        self._prev_heading_rel = self.heading_rel

    def complete_turn(self):
        """Call after main.py finishes the blocking U-turn maneuver."""
        cfg = self.cfg
        self._lane_index += 1
        if self._is_transverse_sweep():
            self.y += self._sweep_sign * cfg.LANE_WIDTH_CM
            self.target_heading = angle_diff(self.target_heading + 180.0, 0.0)
            along = self.x - cfg.START_X_CM
            east = math.sin(math.radians(self.target_heading)) >= 0.0
            if east:
                self.lane_distance = max(0.0, min(along, cfg.ARENA_WIDTH_CM))
            else:
                self.lane_distance = max(0.0, min(cfg.ARENA_WIDTH_CM - along,
                                                  cfg.ARENA_WIDTH_CM))
        else:
            self.x += self._sweep_sign * cfg.LANE_WIDTH_CM
            self.target_heading = angle_diff(self.target_heading + 180.0, 0.0)
            along = self.y - cfg.START_Y_CM
            forward = math.cos(math.radians(self.target_heading)) >= 0.0
            if forward:
                self.lane_distance = max(0.0, min(along, cfg.ARENA_LENGTH_CM))
            else:
                self.lane_distance = max(0.0, min(cfg.ARENA_LENGTH_CM - along,
                                                  cfg.ARENA_LENGTH_CM))
        self.mode = Mode.DRIVING
        self.note_blocking_maneuver()

    def complete_dispose(self):
        """Call after main.py finishes the disposal maneuver: bucket emptied."""
        self.collector.reset()
        self.mode = Mode.DRIVING
        self.note_blocking_maneuver()

    def rel_heading(self, yaw):
        """Convert a raw IMU yaw to a start-relative heading, or None if no IMU."""
        if yaw is None or not self._has_heading:
            return None
        return angle_diff(yaw, self._yaw0)

    def front_wall(self, readings):
        """Public wrapper: (believed wall distance, agreeing count). See _front_wall.

        Used by main.py's blocking return-to-pit maneuver to drive up to a wall.
        """
        return self._front_wall(readings)

    def end_wall_ahead(self, readings):
        """Distance to a believable end wall (for drive-to-wall during return)."""
        return self._lane_end_wall(readings)

    def set_pose(self, x=None, y=None, lane_distance=None, target_heading=None):
        """Overwrite pose after a blocking sensor-referenced maneuver (return legs)."""
        cfg = self.cfg
        if x is not None:
            self.x = x
        if target_heading is not None:
            self.target_heading = target_heading
        lane_len = self._lane_axis_length()
        if lane_distance is not None:
            self.lane_distance = max(0.0, min(lane_distance, lane_len))
        elif self._is_transverse_sweep() and x is not None:
            along = x - cfg.START_X_CM
            east = math.sin(math.radians(self.target_heading)) >= 0.0
            if east:
                self.lane_distance = max(0.0, min(along, cfg.ARENA_WIDTH_CM))
            else:
                self.lane_distance = max(0.0, min(cfg.ARENA_WIDTH_CM - along,
                                                  cfg.ARENA_WIDTH_CM))
        elif y is not None:
            forward = math.cos(math.radians(self.target_heading)) >= 0.0
            along = y - cfg.START_Y_CM
            if forward:
                self.lane_distance = max(0.0, min(along, cfg.ARENA_LENGTH_CM))
            else:
                self.lane_distance = max(0.0, min(cfg.ARENA_LENGTH_CM - along,
                                                  cfg.ARENA_LENGTH_CM))
        if y is not None:
            self.y = y
        self._wall_persist = 0

    def reset_sweep_transverse(self, origin_y=None):
        """Start sideways serpentine: lanes along width, stepping +y from left wall."""
        cfg = self.cfg
        self._sweep_transverse = True
        self._lane_index = 0
        self._next_serpentine_turn = Turn.RIGHT
        self._sweep_sign = 1.0
        self._last_turn = None
        self._pit_handled = False
        self._sweep_origin_y = origin_y if origin_y is not None else self.y
        self.target_heading = 90.0
        standoff = cfg.FRONT_STOP_DISTANCE_CM
        self.x = cfg.START_X_CM + standoff
        self.y = self._sweep_origin_y
        self.lane_distance = standoff
        self.note_blocking_maneuver()

    def bearing_to_pit(self):
        """Heading (start-relative deg) pointing from the car toward the pit."""
        dx = self.cfg.PIT_X_CM - self.x
        dy = self.cfg.PIT_Y_CM - self.y
        return math.degrees(math.atan2(dx, dy))  # matches forward=(sin,cos)

    def dispose_face_heading(self):
        """Heading to face so the car's BACK points at the pit before reversing in.

        On a start-wall or far-wall pit, face along the lane (+y or -y). The
        generic bearing+180° rule fails when dy≈0 (car on the pit's wall) and
        would command a sideways heading.
        """
        cfg = self.cfg
        if abs(cfg.PIT_Y_CM - cfg.START_Y_CM) < 1.0:
            return 0.0
        far_y = cfg.START_Y_CM + cfg.ARENA_LENGTH_CM
        if abs(cfg.PIT_Y_CM - far_y) < 1.0:
            return 180.0
        return angle_diff(self.bearing_to_pit() + 180.0, 0.0)

    def pose_str(self):
        wall = "--" if self.front_wall_cm == INF else f"{self.front_wall_cm:5.1f}"
        phase = "" if self.phase is None else f" ph={self.phase.name}"
        return (f"pos=({self.x:6.1f},{self.y:6.1f}) hdg={self.heading_rel:6.1f} "
                f"tgt={self.target_heading:6.1f} lane#{self._lane_index} "
                f"ld={self.lane_distance:5.1f} y_src={self.pos_source:6s} "
                f"wall={wall}(x{self.front_agree}){phase}")

    # -- internals ----------------------------------------------------------

    def _remember(self, cmd):
        """Record the command so the next tick can integrate the motion it caused."""
        self._last_action = cmd.action
        self._last_speed = (cmd.speed
                            if cmd.action in (Action.FORWARD, Action.REVERSE)
                            else 0.0)
        return cmd

    def _update_pose(self, readings, yaw, dt):
        """Fuse IMU heading + front wall + odometry into the pose estimate."""
        cfg = self.cfg
        if yaw is not None and self._has_heading:
            self._update_heading_rel(yaw)

        # Never bridge more than a couple of control ticks -- a blocking dispose,
        # U-turn, or front-scoop cycle can leave dt at many seconds.
        max_dt = 2.0 / cfg.CONTROL_LOOP_HZ if cfg.CONTROL_LOOP_HZ > 0 else 0.1
        if dt > max_dt:
            dt = max_dt

        # Odometry accumulator: the bridge value + the "where's the wall" prior.
        # REVERSE (benchmark return) accumulates the same way -- lane_distance
        # there means "cm travelled backward from the far wall".
        if (self._last_action in (Action.FORWARD, Action.REVERSE) and dt > 0.0
                and cfg.DRIVE_SPEED > 0.0):
            self.lane_distance += (cfg.DRIVE_CM_PER_S
                                   * (self._last_speed / cfg.DRIVE_SPEED) * dt)

        heading_ok = (not self._has_heading
                      or abs(angle_diff(self.target_heading, self.heading_rel))
                      <= getattr(cfg, "WALL_HEADING_ALIGN_DEG", 30.0))

        # Front wall: distance + how many sensors agree on it.
        front_dist, agree = self._front_wall(readings)
        self.front_agree = agree
        lane_len = self._lane_axis_length()
        if agree >= cfg.FRONT_AGREE_MIN_COUNT:
            expected_gap = max(0.0, lane_len - self.lane_distance)
            plausible = (front_dist <= lane_len
                         and abs(expected_gap - front_dist) <= cfg.WALL_EXPECT_TOL_CM)
            near_start = self.lane_distance < lane_len * 0.35
            far_phantom = front_dist > lane_len * 0.75
            if (plausible and not (near_start and far_phantom) and heading_ok
                    and not self._hill_odometry_only()):
                self.lane_distance = max(0.0, min(lane_len - front_dist, lane_len))
                self.front_wall_cm = front_dist
                self.pos_source = "WALL"
            else:
                self.front_wall_cm = front_dist
                self.pos_source = "BRIDGE"
        else:
            self.front_wall_cm = INF
            self.pos_source = "BRIDGE"

        # While bridging, show a close agreed wall for slowing (never min-of-disagree).
        near_dist, near_agree = self._lane_end_wall(readings)
        if self.pos_source == "BRIDGE" and near_agree >= cfg.FRONT_AGREE_MIN_COUNT:
            self.front_wall_cm = near_dist
            self.front_agree = near_agree

        self.lane_distance = max(0.0, min(self.lane_distance, lane_len))

        if self._is_transverse_sweep() or self._heading_is_across():
            self._sync_lateral_pose()
        elif (self.phase is Phase.BENCHMARK_RETURN
              and self._return_origin_y is not None):
            # Reversing home while still facing the far wall (target heading 0):
            # lane_distance counts backward travel, so y shrinks from the origin.
            self.y = max(cfg.START_Y_CM,
                         self._return_origin_y - self.lane_distance)
        else:
            forward = math.cos(math.radians(self.target_heading)) >= 0.0
            self.y = (cfg.START_Y_CM + self.lane_distance if forward
                      else cfg.START_Y_CM + cfg.ARENA_LENGTH_CM - self.lane_distance)

        # Leaving the pit zone re-arms disposal for the next visit.
        if self._pit_handled and not self._at_pit():
            self._pit_handled = False

    def _sync_lateral_pose(self):
        """Map lane_distance (+ lane index during sweep) to (x, y) when driving ±x."""
        cfg = self.cfg
        east = math.sin(math.radians(self.target_heading)) >= 0.0
        if east:
            self.x = cfg.START_X_CM + self.lane_distance
        else:
            self.x = cfg.START_X_CM + cfg.ARENA_WIDTH_CM - self.lane_distance
        if self._is_transverse_sweep():
            self.y = (self._sweep_origin_y
                      + self._lane_index * self._sweep_sign * cfg.LANE_WIDTH_CM)

    def _enabled_front_distances(self, readings):
        cfg = self.cfg
        out = []
        for name in cfg.FRONT_SENSORS:
            spec = cfg.SENSORS.get(name, {})
            if not spec.get("enabled", True):
                continue
            d = readings.get(name, INF)
            if d != INF:
                out.append(d)
        return out

    def _front_wall(self, readings):
        """Believed distance to the wall ahead + how many front sensors agree.

        A real end wall is flat and spans the whole front, so all the (edge-to-edge)
        front sensors read nearly the same distance; a narrow block/bump shows up on
        one and is outvoted. Returns (median_of_the_agreeing_cluster, cluster_size),
        or (INF, <count>) when fewer than the required number agree.
        """
        cfg = self.cfg
        finite = sorted(self._enabled_front_distances(readings))
        if len(finite) < cfg.FRONT_AGREE_MIN_COUNT:
            return INF, len(finite)
        candidate = statistics.median(finite)
        cluster = [d for d in finite if abs(d - candidate) <= cfg.FRONT_AGREE_TOL_CM]
        if len(cluster) >= cfg.FRONT_AGREE_MIN_COUNT:
            return statistics.median(cluster), len(cluster)
        return INF, len(cluster)

    def _lane_end_wall(self, readings):
        """End-wall distance when sensors actually agree (cluster or tight spread).

        Does NOT take min(front_left, front_right) when they disagree -- that was
        stopping at ~30 cm because the left sensor saw the side wall while the
        right still saw down the lane.
        """
        dist, agree = self._front_wall(readings)
        if agree >= self.cfg.FRONT_AGREE_MIN_COUNT:
            return dist, agree
        finite = self._enabled_front_distances(readings)
        if len(finite) >= self.cfg.FRONT_AGREE_MIN_COUNT:
            spread = max(finite) - min(finite)
            if spread <= self.cfg.FRONT_AGREE_TOL_CM:
                return min(finite), len(finite)
        return INF, len(finite)

    def _at_pit(self):
        if self._lane_index != self._pit_lane:
            return False  # only the pit's own lane can be "at the pit"
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

    def _advance_serpentine(self):
        self._next_serpentine_turn = (
            Turn.LEFT if self._next_serpentine_turn is Turn.RIGHT else Turn.RIGHT
        )

    def _update_heading_rel(self, yaw):
        """Fuse IMU yaw into heading_rel, rejecting single-tick glitches while cruising."""
        cfg = self.cfg
        raw = angle_diff(yaw, self._yaw0)
        step = angle_diff(raw, self._prev_heading_rel)
        max_step = getattr(cfg, "IMU_GLITCH_MAX_STEP_DEG", 45.0)
        cruising = (self.mode is Mode.DRIVING
                    and self._last_action in (Action.FORWARD, Action.REVERSE))
        if cruising and abs(step) > max_step:
            return
        self.heading_rel = raw
        self._prev_heading_rel = raw

    def _cruise_trim(self, readings):
        """Steering trim while cruising: IMU heading-hold toward the lane heading."""
        cfg = self.cfg
        if not self._has_heading:
            return 0.0
        err = angle_diff(self.target_heading, self.heading_rel)
        deadband = getattr(cfg, "HEADING_HOLD_DEADBAND_DEG", 0.0)
        gain = cfg.HEADING_HOLD_GAIN
        max_trim = cfg.MAX_HEADING_TRIM
        if abs(err) <= deadband:
            return 0.0
        # Sign matches _drive_to_wall in main.py and this rover's IMU/motor frame.
        trim = -gain * err
        return max(-max_trim, min(max_trim, trim))
