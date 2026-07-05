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
    DONE = auto()


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

        # -- pose, in the start-relative frame ---------------------------------
        self.x = cfg.START_X_CM
        self.y = cfg.START_Y_CM
        self.heading_rel = 0.0       # current heading, degrees, relative to start
        self.target_heading = 0.0    # heading we want to hold down this lane
        self.lane_distance = 0.0     # cm along the current lane (wall-anchored when a wall is seen, else odometry)
        self.mode = Mode.DRIVING

        # Heading reference (set by set_origin()); until then heading-hold is off.
        self._yaw0 = None
        self._has_heading = False

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

        # Coverage: once the last lane (NUM_LANES-1) is finished, we're done.
        self._done = False

        # Wall-fusion state (updated every tick by _update_pose).
        self.front_wall_cm = INF     # believed distance to the end wall, or INF
        self.front_agree = 0         # how many front sensors agreed this tick
        self.pos_source = "BRIDGE"   # "WALL" = y from a believed wall, "BRIDGE" = from odometry
        self.x_source = "LANE"       # "WALL" = x re-zeroed off a side wall, "LANE" = from lane counting
        self._wall_persist = 0       # consecutive ticks the wall-stop has held

        # Odometry bookkeeping for the bridge / expectation prior.
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
        """Map sensors + IMU yaw + elapsed dt to a Command, updating pose/mode."""
        cfg = self.cfg
        self._update_pose(readings, yaw, dt)

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

        # 2) End of lane? Primary = a believed wall within the standoff, held for
        #    a few ticks. Backstop = odometry distance if the wall is never seen.
        wall_stop = (self.pos_source == "WALL"
                     and self.front_wall_cm <= cfg.FRONT_STOP_DISTANCE_CM)
        self._wall_persist = self._wall_persist + 1 if wall_stop else 0
        wall_trigger = self._wall_persist >= cfg.WALL_PERSIST_TICKS
        # Odometry backstop only applies when we have NO believed wall (total
        # dropout). With a wall in view the persistence-gated trigger governs, so
        # a close wall re-anchoring lane_distance can't skip persistence.
        odo_backstop = (self.pos_source != "WALL"
                        and self.lane_distance >= (cfg.ARENA_LENGTH_CM - cfg.LANE_END_MARGIN_CM))

        if wall_trigger or odo_backstop:
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
            trigger = (f"wall {self.front_wall_cm:.0f}cm (x{self.front_agree} agree)"
                       if wall_trigger else "odometry backstop (no wall seen)")
            return self._remember(Command(
                action, speed=cfg.TURN_SPEED,
                reason=f"end of lane ({trigger}), turn {turn.name.lower()}"))

        # 3) Cruise forward, holding the lane heading with the IMU.
        self.mode = Mode.DRIVING
        speed = (cfg.SLOW_SPEED if self.front_wall_cm <= cfg.FRONT_SLOW_DISTANCE_CM
                 else cfg.DRIVE_SPEED)
        steer = self._cruise_trim(readings)
        return self._remember(Command(
            Action.FORWARD, speed=speed, steer=steer, reason="cruising"))

    def complete_turn(self):
        """Call after main.py finishes the blocking U-turn maneuver.

        Steps to the next lane (heading reversed ~180, one lane over) and resets
        the per-lane fusion state. Cross-lane x comes from the lane index, so the
        sideways step is exact regardless of how the physical shift went.
        """
        self._lane_index += 1
        self.x += self._sweep_sign * self.cfg.LANE_WIDTH_CM   # step one lane sideways
        self.target_heading = angle_diff(self.target_heading + 180.0, 0.0)
        self.lane_distance = 0.0
        self._wall_persist = 0
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

    def front_wall(self, readings):
        """Public wrapper: (believed wall distance, agreeing count). See _front_wall.

        Used by main.py's blocking return-to-pit maneuver to drive up to a wall.
        """
        return self._front_wall(readings)

    def bearing_to_pit(self):
        """Heading (start-relative deg) pointing from the car toward the pit."""
        dx = self.cfg.PIT_X_CM - self.x
        dy = self.cfg.PIT_Y_CM - self.y
        return math.degrees(math.atan2(dx, dy))  # matches forward=(sin,cos)

    def pose_str(self):
        wall = "--" if self.front_wall_cm == INF else f"{self.front_wall_cm:5.1f}"
        return (f"pos=({self.x:6.1f},{self.y:6.1f}) hdg={self.heading_rel:6.1f} "
                f"tgt={self.target_heading:6.1f} lane#{self._lane_index} "
                f"ld={self.lane_distance:5.1f} y_src={self.pos_source:6s} "
                f"x_src={self.x_source:4s} wall={wall}(x{self.front_agree})")

    # -- internals ----------------------------------------------------------

    def _remember(self, cmd):
        """Record the command so the next tick can integrate the motion it caused."""
        self._last_action = cmd.action
        self._last_speed = cmd.speed if cmd.action is Action.FORWARD else 0.0
        return cmd

    def _update_pose(self, readings, yaw, dt):
        """Fuse IMU heading + front wall + odometry into the pose estimate."""
        cfg = self.cfg
        if yaw is not None and self._has_heading:
            self.heading_rel = angle_diff(yaw, self._yaw0)

        # Odometry accumulator: the bridge value + the "where's the wall" prior.
        if (self._last_action is Action.FORWARD and dt > 0.0
                and cfg.DRIVE_SPEED > 0.0):
            self.lane_distance += (cfg.DRIVE_CM_PER_S
                                   * (self._last_speed / cfg.DRIVE_SPEED) * dt)

        forward = math.cos(math.radians(self.target_heading)) >= 0.0

        # Front wall: distance + how many sensors agree on it.
        front_dist, agree = self._front_wall(readings)
        self.front_agree = agree
        if agree >= cfg.FRONT_AGREE_MIN_COUNT:
            # Believable ONLY if it's near where odometry expects the end wall --
            # this rejects a mid-lane object (a block) that fooled the agreement.
            expected_gap = max(0.0, cfg.ARENA_LENGTH_CM - self.lane_distance)
            if abs(expected_gap - front_dist) <= cfg.WALL_EXPECT_TOL_CM:
                self.lane_distance = cfg.ARENA_LENGTH_CM - front_dist  # re-anchor to the wall
                self.front_wall_cm = front_dist
                self.pos_source = "WALL"
            else:
                # Agreed, but not where a wall belongs -> an obstacle to collect,
                # not the lane end. Report it (so we slow) but don't trust it as position.
                self.front_wall_cm = front_dist
                self.pos_source = "BRIDGE"
        else:
            self.front_wall_cm = INF
            self.pos_source = "BRIDGE"

        # Along-lane y from lane_distance in the current lane's direction. Cross-lane
        # x carries over from the last turn (lane counting), corrected below.
        self.y = (cfg.START_Y_CM + self.lane_distance if forward
                  else cfg.START_Y_CM + cfg.ARENA_LENGTH_CM - self.lane_distance)

        # Edge re-zero: near a side wall (outer lanes), re-anchor x to the measured
        # wall -- but only if it agrees with the lane-counting prior (rejects a block).
        self.x_source = "LANE"
        measured_x = self._side_x(readings, forward)
        if measured_x is not None:
            prior = cfg.START_X_CM + self._sweep_sign * self._lane_index * cfg.LANE_WIDTH_CM
            if abs(measured_x - prior) <= cfg.SIDE_EXPECT_TOL_CM:
                self.x = measured_x
                self.x_source = "WALL"

        # Leaving the pit zone re-arms disposal for the next visit.
        if self._pit_handled and not self._at_pit():
            self._pit_handled = False

    def _front_wall(self, readings):
        """Believed distance to the wall ahead + how many front sensors agree.

        A real end wall is flat and spans the whole front, so all the (edge-to-edge)
        front sensors read nearly the same distance; a narrow block/bump shows up on
        one and is outvoted. Returns (median_of_the_agreeing_cluster, cluster_size),
        or (INF, <count>) when fewer than the required number agree.
        """
        cfg = self.cfg
        finite = sorted(d for d in (readings.get(n, INF) for n in cfg.FRONT_SENSORS)
                        if d != INF)
        if len(finite) < cfg.FRONT_AGREE_MIN_COUNT:
            return INF, len(finite)
        candidate = statistics.median(finite)
        cluster = [d for d in finite if abs(d - candidate) <= cfg.FRONT_AGREE_TOL_CM]
        if len(cluster) >= cfg.FRONT_AGREE_MIN_COUNT:
            return statistics.median(cluster), len(cluster)
        return INF, len(cluster)

    def _side_x(self, readings, forward):
        """Cross-lane x measured off a NEAR side wall (edge re-zero), or None.

        The sensor pair facing the arena's -x wall depends on lane direction: a
        body-left sensor faces arena -x while driving +y, but arena +x while driving
        -y. Both sensors on a side must agree (rejects a block on one), and the wall
        must be nearer than SIDE_WALL_TRUST_CM (only the outer lanes). The nearer of
        the two walls wins. None if no side wall is reliably in range (e.g. a middle
        lane, or the side sensors are disabled -> all readings inf).
        """
        cfg = self.cfg
        # Which physical pair faces the arena's left (-x) vs right (+x) wall now.
        left_names, right_names = ((cfg.LEFT_SENSORS, cfg.RIGHT_SENSORS) if forward
                                   else (cfg.RIGHT_SENSORS, cfg.LEFT_SENSORS))
        best = None  # (distance, x_estimate); arena left wall at x=0, right at WIDTH
        dl = self._pair_distance(readings, left_names)
        if dl is not None and dl <= cfg.SIDE_WALL_TRUST_CM:
            best = (dl, dl + cfg.SIDE_SENSOR_OFFSET_CM)
        dr = self._pair_distance(readings, right_names)
        if (dr is not None and dr <= cfg.SIDE_WALL_TRUST_CM
                and (best is None or dr < best[0])):
            best = (dr, cfg.ARENA_WIDTH_CM - dr - cfg.SIDE_SENSOR_OFFSET_CM)
        return best[1] if best else None

    def _pair_distance(self, readings, names):
        """Median distance if BOTH sensors in the pair see something and agree, else None."""
        vals = [readings.get(n, INF) for n in names]
        finite = [d for d in vals if d != INF]
        if len(finite) < len(names):          # need both sensors of the pair
            return None
        if max(finite) - min(finite) > self.cfg.FRONT_AGREE_TOL_CM:
            return None                        # disagree -> likely a block on one
        return statistics.median(finite)

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

    def _cruise_trim(self, readings):
        """Steering trim while cruising: IMU heading-hold toward the lane heading."""
        cfg = self.cfg
        if not self._has_heading:
            return 0.0  # no IMU -> can't hold a heading; drive open-loop straight
        # Positive steer = toward the car's right. If our heading is left of the
        # target (target - heading > 0), steer right to come back.
        err = angle_diff(self.target_heading, self.heading_rel)
        trim = cfg.HEADING_HOLD_GAIN * err
        return max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM, trim))
