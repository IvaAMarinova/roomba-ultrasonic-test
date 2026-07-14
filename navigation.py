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


class Phase(Enum):
    """Macro stages for HILL_MODE (slope -> sweep -> descend -> pit)."""
    CLIMB_FIRST = auto()
    REPOSITION_TO_LEFT = auto()
    SWEEP = auto()
    REPOSITION_FOR_DESCEND = auto()
    DESCEND = auto()
    TO_PIT = auto()


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

        # Hill strategy phase (None when HILL_MODE is off).
        self.phase = Phase.CLIMB_FIRST if getattr(cfg, "HILL_MODE", False) else None

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
        if not getattr(self.cfg, "HILL_MODE", False):
            return True
        return (self.phase is Phase.SWEEP
                and self.y >= self.cfg.COLLECTION_START_Y_CM)

    @property
    def wants_climb_shovel(self):
        if not getattr(self.cfg, "HILL_MODE", False):
            return False
        return self.phase in (Phase.CLIMB_FIRST, Phase.DESCEND)

    def note_blocking_maneuver(self):
        """Call after a blocking U-turn / dispose so the next tick does not odometry-bridge a long gap."""
        self._last_action = Action.STOP
        self._last_speed = 0.0
        self._wall_persist = 0
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
                reason=f"end of lane ({trigger}), turn {turn.name.lower()}"))

        return self._cmd_cruise(readings, reason="cruising")

    def _decide_hill(self, readings, yaw, dt):
        """Slope climb -> end-zone sweep -> descend -> pit (no mid-run disposal)."""
        cfg = self.cfg

        if self._done:
            self.mode = Mode.DONE
            return self._remember(Command(Action.STOP, reason="run complete"))

        if self.phase is Phase.TO_PIT:
            self.mode = Mode.DONE
            return self._remember(Command(Action.STOP, reason="at start wall -> pit"))

        if self.phase is Phase.CLIMB_FIRST:
            if self.y >= cfg.HILL_TOP_Y_CM:
                self.phase = Phase.REPOSITION_TO_LEFT
                print(f"[hill] top of slope (y={self.y:.0f}) -> reposition to left wall")
            return self._cmd_cruise(readings, reason="climbing slope (centre)")

        if self.phase is Phase.REPOSITION_TO_LEFT:
            self.mode = Mode.DONE
            return self._remember(Command(
                Action.STOP, reason="reposition to left wall"))

        if self.phase is Phase.SWEEP:
            wall_trigger, odo_backstop, end_dist, end_agree, trigger = (
                self._lane_end_state(readings))
            if wall_trigger or odo_backstop:
                if self._at_right_edge(readings):
                    self.phase = Phase.REPOSITION_FOR_DESCEND
                    self._wall_persist = 0
                    print("[hill] right wall -> reposition to centre for descend")
                    self.mode = Mode.DONE
                    return self._remember(Command(
                        Action.STOP, reason="right wall -> reposition for descend"))
                if wall_trigger and trigger.startswith("wall") and "contact" not in trigger:
                    self._reanchor_lane_from_wall(end_dist)
                turn = self._next_serpentine_turn
                self._advance_serpentine()
                self.mode = Mode.TURNING
                action = Action.TURN_LEFT if turn is Turn.LEFT else Action.TURN_RIGHT
                return self._remember(Command(
                    action, speed=cfg.TURN_SPEED,
                    reason=f"end of lane ({trigger}), turn {turn.name.lower()}"))
            return self._cmd_cruise(readings, reason="sweeping")

        if self.phase is Phase.REPOSITION_FOR_DESCEND:
            self.mode = Mode.DONE
            return self._remember(Command(
                Action.STOP, reason="reposition for descend"))

        if self.phase is Phase.DESCEND:
            wall_trigger, odo_backstop, end_dist, end_agree, trigger = (
                self._lane_end_state(readings))
            at_start = (self.y <= cfg.FRONT_STOP_DISTANCE_CM + 5.0
                        or (wall_trigger and self.y <= cfg.HILL_TOP_Y_CM))
            if at_start or (wall_trigger and self.target_heading == 180.0
                            and self.y <= cfg.HILL_TOP_Y_CM):
                self.phase = Phase.TO_PIT
                print(f"[hill] start wall (y={self.y:.0f}) -> pit approach")
                self.mode = Mode.DONE
                return self._remember(Command(
                    Action.STOP, reason="at start wall -> pit"))
            return self._cmd_cruise(readings, reason="descending slope (centre)")

        return self._cmd_cruise(readings, reason="cruising")

    def _cmd_cruise(self, readings, reason="", extra_steer=0.0):
        """Forward at cruise speed with IMU heading-hold (+ optional wall-hug trim)."""
        cfg = self.cfg
        self.mode = Mode.DRIVING
        speed = (cfg.SLOW_SPEED if self.front_wall_cm <= cfg.FRONT_SLOW_DISTANCE_CM
                 else cfg.DRIVE_SPEED)
        steer = self._cruise_trim(readings) + extra_steer
        steer = max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM, steer))
        return self._remember(Command(
            Action.FORWARD, speed=speed, steer=steer, reason=reason))

    def _lane_end_state(self, readings):
        """Return (wall_trigger, odo_backstop, end_dist, end_agree, trigger_reason)."""
        cfg = self.cfg
        end_dist, end_agree = self._lane_end_wall(readings)
        expected_gap = max(0.0, cfg.ARENA_LENGTH_CM - self.lane_distance)
        wall_plausible = (end_dist >= expected_gap - cfg.WALL_EXPECT_TOL_CM)
        heading_ok = (not self._has_heading
                      or abs(angle_diff(self.target_heading, self.heading_rel))
                      <= getattr(cfg, "WALL_HEADING_ALIGN_DEG", 30.0))
        contact_close = getattr(cfg, "WALL_CONTACT_STOP_CM", 25.0)
        standoff_aligned = end_dist >= expected_gap - cfg.FRONT_STOP_DISTANCE_CM
        inferred_ld = max(0.0, min(cfg.ARENA_LENGTH_CM - end_dist,
                                   cfg.ARENA_LENGTH_CM))
        inferred_near_far_end = (inferred_ld >= (cfg.ARENA_LENGTH_CM
                                                 - cfg.LANE_END_MARGIN_CM
                                                 - cfg.FRONT_STOP_DISTANCE_CM))
        odo_matches_inferred = (abs(inferred_ld - self.lane_distance)
                                <= cfg.WALL_EXPECT_TOL_CM)
        inferred_standoff = (end_dist <= cfg.FRONT_STOP_DISTANCE_CM
                             and inferred_near_far_end
                             and odo_matches_inferred)
        near_far_end = (self.lane_distance + end_dist
                        >= cfg.ARENA_LENGTH_CM - cfg.WALL_EXPECT_TOL_CM)
        odo_still_near_start = (self.lane_distance
                                < cfg.ARENA_LENGTH_CM * 0.35)
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
                        and self.lane_distance >= (cfg.ARENA_LENGTH_CM - cfg.LANE_END_MARGIN_CM)
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
        self.lane_distance = max(0.0, min(cfg.ARENA_LENGTH_CM - end_dist,
                                          cfg.ARENA_LENGTH_CM))
        forward = math.cos(math.radians(self.target_heading)) >= 0.0
        self.y = (cfg.START_Y_CM + self.lane_distance if forward
                  else cfg.START_Y_CM + cfg.ARENA_LENGTH_CM - self.lane_distance)

    def _at_right_edge(self, readings):
        cfg = self.cfg
        if self.x + cfg.LANE_WIDTH_CM >= cfg.ARENA_WIDTH_CM - cfg.RIGHT_EDGE_MARGIN_CM:
            return True
        if readings:
            d = readings.get("front_right", INF)
            if d != INF and d <= cfg.RIGHT_WALL_STOP_CM:
                return True
        return False

    def complete_turn(self):
        """Call after main.py finishes the blocking U-turn maneuver.

        Steps to the next lane (heading reversed ~180, one lane over) and resets
        the per-lane fusion state. Cross-lane x comes from the lane index, so the
        sideways step is exact regardless of how the physical shift went.
        """
        cfg = self.cfg
        self._lane_index += 1
        self.x += self._sweep_sign * cfg.LANE_WIDTH_CM   # step one lane sideways
        self.target_heading = angle_diff(self.target_heading + 180.0, 0.0)
        # Keep along-lane position across the U-turn: resetting to 0 would snap y
        # to the start wall whenever the next lane drives +y (the usual case after
        # turning at y=0), falsely placing the car at the pit.
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
        if lane_distance is not None:
            self.lane_distance = max(0.0, min(lane_distance, cfg.ARENA_LENGTH_CM))
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

    def reset_sweep_from_left(self):
        """Re-anchor serpentine state after blocking drive to the left wall."""
        cfg = self.cfg
        first = getattr(cfg, "SERPENTINE_FIRST_TURN", "left").lower()
        self._lane_index = 0
        self._next_serpentine_turn = Turn.RIGHT if first == "right" else Turn.LEFT
        self._sweep_sign = 1.0 if first == "right" else -1.0
        self._last_turn = None
        self._pit_handled = False
        self.target_heading = 0.0
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
        self._last_speed = cmd.speed if cmd.action is Action.FORWARD else 0.0
        return cmd

    def _update_pose(self, readings, yaw, dt):
        """Fuse IMU heading + front wall + odometry into the pose estimate."""
        cfg = self.cfg
        if yaw is not None and self._has_heading:
            self.heading_rel = angle_diff(yaw, self._yaw0)

        # Never bridge more than a couple of control ticks -- a blocking dispose,
        # U-turn, or front-scoop cycle can leave dt at many seconds.
        max_dt = 2.0 / cfg.CONTROL_LOOP_HZ if cfg.CONTROL_LOOP_HZ > 0 else 0.1
        if dt > max_dt:
            dt = max_dt

        # Odometry accumulator: the bridge value + the "where's the wall" prior.
        if (self._last_action is Action.FORWARD and dt > 0.0
                and cfg.DRIVE_SPEED > 0.0):
            self.lane_distance += (cfg.DRIVE_CM_PER_S
                                   * (self._last_speed / cfg.DRIVE_SPEED) * dt)

        forward = math.cos(math.radians(self.target_heading)) >= 0.0
        heading_ok = (not self._has_heading
                      or abs(angle_diff(self.target_heading, self.heading_rel))
                      <= getattr(cfg, "WALL_HEADING_ALIGN_DEG", 30.0))

        # Front wall: distance + how many sensors agree on it.
        front_dist, agree = self._front_wall(readings)
        self.front_agree = agree
        if agree >= cfg.FRONT_AGREE_MIN_COUNT:
            # Believable ONLY if it's near where odometry expects the end wall --
            # this rejects a mid-lane object (a block) that fooled the agreement.
            expected_gap = max(0.0, cfg.ARENA_LENGTH_CM - self.lane_distance)
            plausible = (front_dist <= cfg.ARENA_LENGTH_CM
                         and abs(expected_gap - front_dist) <= cfg.WALL_EXPECT_TOL_CM)
            # Reject a far phantom wall when odometry says we're still near the
            # start of the lane (e.g. pit gap / open area behind the rover).
            near_start = self.lane_distance < cfg.ARENA_LENGTH_CM * 0.35
            far_phantom = front_dist > cfg.ARENA_LENGTH_CM * 0.75
            if plausible and not (near_start and far_phantom) and heading_ok:
                self.lane_distance = max(0.0, min(cfg.ARENA_LENGTH_CM - front_dist,
                                                  cfg.ARENA_LENGTH_CM))
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

        # While bridging, show a close agreed wall for slowing (never min-of-disagree).
        near_dist, near_agree = self._lane_end_wall(readings)
        if self.pos_source == "BRIDGE" and near_agree >= cfg.FRONT_AGREE_MIN_COUNT:
            self.front_wall_cm = near_dist
            self.front_agree = near_agree

        self.lane_distance = max(0.0, min(self.lane_distance, cfg.ARENA_LENGTH_CM))

        # Along-lane y from lane_distance in the current lane's direction. Cross-lane
        # x comes purely from lane counting (set in complete_turn); the IMU
        # heading-hold keeps the car square so it stays centred in the lane.
        self.y = (cfg.START_Y_CM + self.lane_distance if forward
                  else cfg.START_Y_CM + cfg.ARENA_LENGTH_CM - self.lane_distance)

        # Leaving the pit zone re-arms disposal for the next visit.
        if self._pit_handled and not self._at_pit():
            self._pit_handled = False

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

    def _cruise_trim(self, readings):
        """Steering trim while cruising: IMU heading-hold toward the lane heading."""
        cfg = self.cfg
        if not self._has_heading:
            return 0.0  # no IMU -> can't hold a heading; drive open-loop straight
        # Positive steer = toward the car's right. If our heading is left of the
        # target (target - heading > 0), steer right to come back.
        err = angle_diff(self.target_heading, self.heading_rel)
        trim = -cfg.HEADING_HOLD_GAIN * err
        return max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM, trim))
