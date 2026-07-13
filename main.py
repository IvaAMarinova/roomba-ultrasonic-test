"""
Entry point. Two modes, selected by config.USE_SENSORS:

  USE_SENSORS = False  -> open-loop drive test: run the hardcoded maneuver
                          script (config.DRIVE_TEST_SEQUENCE) so you can check
                          the motors / H-bridges / turning on the real car.
                          The sensors are never touched.
  USE_SENSORS = True   -> IMU + odometry navigation: hold heading with the IMU,
                          track position by dead-reckoning against the known
                          arena size, use the ultrasonics only as a wall-stop
                          fallback, and dispose collected blocks at the pit.

Run on the Pi with:  python3 main.py
"""

import time
import argparse

import config
from actuators import Disposer, FrontServo
from log import Logger
from motors import MotorDriver
from navigation import NavigationController, Action, Mode, Phase, angle_diff


def _spin_timed(logger, motors, cfg, direction):
    """Original open-loop ~90 deg in-place rotation (no IMU)."""
    (motors.turn_left if direction == "left" else motors.turn_right)(logger, cfg.TURN_SPEED)
    time.sleep(cfg.TURN_TIME_S)
    motors.stop(logger)


def _spin_imu(logger, motors, cfg, direction, imu):
    """Rotate in place until the measured heading change reaches the target.

    Accumulates the signed per-sample yaw delta (wrap-safe) so it works in
    either direction and across the +/-180 wrap. Corrupted single-sample jumps
    are ignored, and a hard timeout guarantees the spin always ends. Returns the
    degrees actually turned, or None if no heading could be read at all (so the
    caller can fall back to the timed spin).
    """
    start = imu.yaw()
    if start is None:
        return None

    turn_fn = motors.turn_left if direction == "left" else motors.turn_right
    target = cfg.TURN_ANGLE_DEG - cfg.IMU_TURN_TOLERANCE_DEG
    prev = start
    turned = 0.0

    # Stall recovery: if we've been turning for boost_after seconds but the IMU
    # still shows less than half the goal, the tires are likely binding -- bump
    # the spin speed up to break through the friction. Boost once, then hold.
    speed = cfg.TURN_SPEED
    boost_after = getattr(cfg, "IMU_TURN_BOOST_AFTER_S", None)
    half_goal = cfg.TURN_ANGLE_DEG / 2.0
    boosted = False

    turn_fn(logger, speed)
    start_t = time.time()
    deadline = start_t + cfg.IMU_TURN_TIMEOUT_S
    while time.time() < deadline:
        time.sleep(cfg.IMU_TURN_POLL_S)
        cur = imu.yaw()
        if cur is None:
            continue
        step = angle_diff(cur, prev)
        prev = cur
        if abs(step) > cfg.IMU_GLITCH_MAX_STEP_DEG:
            continue  # corrupted I2C read -- ignore this sample
        turned += step
        if abs(turned) >= target:
            break
        if (not boosted and boost_after is not None
                and time.time() - start_t >= boost_after
                and abs(turned) < half_goal):
            speed = min(1.0, cfg.TURN_SPEED * cfg.IMU_TURN_BOOST_FACTOR)
            turn_fn(logger, speed)
            boosted = True
            logger.log("uturn", step="stall_boost", turned=abs(turned),
                       window_s=boost_after, target=cfg.TURN_ANGLE_DEG,
                       speed_from=cfg.TURN_SPEED, speed_to=speed)
    motors.stop(logger)
    return abs(turned)


def _spin_90(logger, motors, cfg, direction, imu=None):
    """Rotate ~TURN_ANGLE_DEG in place.

    Uses IMU heading feedback when an IMU is present and USE_IMU_TURN is set;
    otherwise (or if the IMU yields no reading) falls back to the timed spin.
    """
    if imu is not None and imu.available and getattr(cfg, "USE_IMU_TURN", False):
        turned = _spin_imu(logger, motors, cfg, direction, imu)
        if turned is not None:
            logger.log("uturn", step="imu_spin", direction=direction, turned=turned,
                       target=cfg.TURN_ANGLE_DEG)
            return
        logger.log("uturn", step="timed_fallback", reason="IMU gave no heading")
    _spin_timed(logger, motors, cfg, direction)


def _heading_aligned(imu, nav, target_rel, tol):
    """True when IMU heading is within `tol` degrees of `target_rel`."""
    cur = nav.rel_heading(imu.yaw()) if imu is not None else None
    if cur is None:
        return False
    return abs(angle_diff(target_rel, cur)) <= tol


def _spin_to_heading(logger, motors, cfg, imu, nav, target_rel):
    """Rotate in place until the car's heading ~= target_rel (start-relative deg).

    Accumulates signed heading change (like U-turn spins) so a 3° correction
    stops after ~3°, not after the full IMU timeout. Spins one way without
    per-tick direction flips.
    """
    cur = nav.rel_heading(imu.yaw()) if imu is not None else None
    if cur is None:
        logger.log("orient", step="skip", reason="no IMU heading")
        return False
    err0 = angle_diff(target_rel, cur)
    if abs(err0) <= cfg.IMU_TURN_TOLERANCE_DEG:
        logger.log("orient", step="skip", target=target_rel, current=cur, error=err0)
        return True
    logger.log("orient", step="turn", target=target_rel, current=cur, error=err0)

    direction = "right" if err0 > 0 else "left"
    turn_fn = motors.turn_right if direction == "right" else motors.turn_left
    goal = max(0.0, abs(err0) - cfg.IMU_TURN_TOLERANCE_DEG)
    timeout = cfg.IMU_TURN_TIMEOUT_S * max(1.0, abs(err0) / cfg.TURN_ANGLE_DEG)
    deadline = time.time() + timeout

    turn_fn(logger, cfg.TURN_SPEED)
    prev_rel = cur
    turned = 0.0
    while time.time() < deadline:
        time.sleep(cfg.IMU_TURN_POLL_S)
        cur_rel = nav.rel_heading(imu.yaw())
        if cur_rel is None:
            continue
        if abs(angle_diff(target_rel, cur_rel)) <= cfg.IMU_TURN_TOLERANCE_DEG:
            break
        step = angle_diff(cur_rel, prev_rel)
        if abs(step) > cfg.IMU_GLITCH_MAX_STEP_DEG:
            continue
        prev_rel = cur_rel
        turned += step
        if abs(turned) >= goal:
            break
    motors.stop(logger)

    aligned = _heading_aligned(imu, nav, target_rel, cfg.IMU_TURN_TOLERANCE_DEG)
    if not aligned:
        cur = nav.rel_heading(imu.yaw())
        err = angle_diff(target_rel, cur) if cur is not None else None
        if cur is not None and err is not None and abs(err) > cfg.IMU_TURN_TOLERANCE_DEG:
            fine = motors.turn_left if err < 0 else motors.turn_right
            fine(logger, cfg.TURN_SPEED * 0.5)
            fine_deadline = time.time() + cfg.IMU_TURN_TIMEOUT_S * 0.25
            prev_rel = cur
            turned = 0.0
            fine_goal = max(0.0, abs(err) - cfg.IMU_TURN_TOLERANCE_DEG)
            while time.time() < fine_deadline:
                time.sleep(cfg.IMU_TURN_POLL_S)
                cur_rel = nav.rel_heading(imu.yaw())
                if cur_rel is None:
                    continue
                if abs(angle_diff(target_rel, cur_rel)) <= cfg.IMU_TURN_TOLERANCE_DEG:
                    break
                step = angle_diff(cur_rel, prev_rel)
                if abs(step) > cfg.IMU_GLITCH_MAX_STEP_DEG:
                    continue
                prev_rel = cur_rel
                turned += step
                if abs(turned) >= fine_goal:
                    break
            motors.stop(logger)
        aligned = _heading_aligned(imu, nav, target_rel, cfg.IMU_TURN_TOLERANCE_DEG)
        if not aligned:
            cur = nav.rel_heading(imu.yaw())
            logger.log("orient", step="incomplete", target=target_rel,
                       current=cur, error=angle_diff(target_rel, cur) if cur is not None else None)
    return aligned


def _advance_one_lane(logger, motors, cfg):
    """Drive straight by LANE_WIDTH_CM (the sideways shift into the next lane)."""
    motors.drive(logger, cfg.DRIVE_SPEED, 0.0)
    time.sleep(cfg.LANE_WIDTH_CM / cfg.DRIVE_CM_PER_S)
    motors.stop(logger)


def _u_turn(logger, motors, cfg, direction, imu, nav):
    """End-of-lane U-turn: spin 90, shift one lane width, spin 90 -- one decision.

    No sensors are read while it runs; the steps are logged so the second spin
    isn't a surprise. Afterwards nav.complete_turn() folds the net motion (heading
    reversed, one lane shifted) back into the odometry pose.
    """
    shift_s = cfg.LANE_WIDTH_CM / cfg.DRIVE_CM_PER_S
    logger.log("uturn", step="spin", direction=direction, deg=cfg.TURN_ANGLE_DEG)
    _spin_90(logger, motors, cfg, direction, imu)
    logger.log("uturn", step="lane_shift", cm=cfg.LANE_WIDTH_CM, seconds=shift_s)
    _advance_one_lane(logger, motors, cfg)
    logger.log("uturn", step="spin", direction=direction, deg=cfg.TURN_ANGLE_DEG)
    _spin_90(logger, motors, cfg, direction, imu)
    nav.complete_turn()
    logger.log("uturn", step="done", x=nav.x, y=nav.y, heading=nav.heading_rel)


def _drive_distance(logger, motors, cfg, distance_cm, speed):
    """Drive straight for a fixed distance (cm) at `speed` (signed: <0 = reverse).

    Open-loop timed off DRIVE_CM_PER_S (measured at DRIVE_SPEED). Short move, so
    the timing is good enough even on a bumpy floor.
    """
    cm_per_s = cfg.DRIVE_CM_PER_S * (abs(speed) / cfg.DRIVE_SPEED)
    seconds = distance_cm / cm_per_s if cm_per_s > 0 else 0.0
    motors.drive(logger, speed, 0.0)
    time.sleep(seconds)
    motors.stop(logger)
    return seconds


def _dispose(logger, motors, cfg, imu, nav, disposer, face_heading=None):
    """Disposal maneuver for a SMALL (car-sized) pit: seat the rear over it, dump.

      1. Turn so the car's BACK faces the pit.
      2. Lift the rear door and reverse DISPOSE_REVERSE_CM together so the back
         seats over the pit with the door already opening.
      3. Hold with the door open, then close it (actuators.Disposer).
      4. Pull the same distance forward to get clear of the pit.
      5. Re-orient to the lane heading so the sweep resumes cleanly.

    face_heading: the heading to face so the back points at the pit. If None (the
    normal in-sweep pass) it is computed from the live pose (nav.bearing_to_pit).
    The final return-to-pit dump passes it explicitly, because after the blocking
    return legs the pose estimate is stale. Afterwards nav.complete_dispose()
    empties the (stub) bucket and returns to DRIVING.
    """
    if cfg.DISPOSE_BACK_INTO_PIT:
        target = (face_heading if face_heading is not None
                  else nav.dispose_face_heading())
        cur = nav.rel_heading(imu.yaw() if imu is not None else None)
        if (cur is None
                or abs(angle_diff(target, cur)) > cfg.ORIENT_SKIP_DEG):
            _spin_to_heading(logger, motors, cfg, imu, nav, target)

    door_thread = disposer.start_opening()
    time.sleep(0.1) # Need to back up first
    rev_s = _drive_distance(logger, motors, cfg, cfg.DISPOSE_REVERSE_CM, -cfg.DISPOSE_REVERSE_SPEED)
    disposer.join_opening(door_thread)
    logger.log("dispose", step="reverse", cm=cfg.DISPOSE_REVERSE_CM, seconds=rev_s)

    logger.log("dispose", step="dump", hold_s=cfg.DISPOSE_HOLD_S)
    disposer.dump()

    _drive_distance(logger, motors, cfg, cfg.DISPOSE_REVERSE_CM, cfg.DISPOSE_REVERSE_SPEED)
    logger.log("dispose", step="clear", cm=cfg.DISPOSE_REVERSE_CM)

    # Resume the lane heading only if disposal left us meaningfully misaligned
    # (small errors are fine -- spinning for 3° often overshoots and wrecks pose).
    cur = nav.rel_heading(imu.yaw() if imu is not None else None)
    if (cur is not None
            and abs(angle_diff(nav.target_heading, cur)) > cfg.ORIENT_SKIP_DEG):
        _spin_to_heading(logger, motors, cfg, imu, nav, nav.target_heading)
    nav.complete_dispose()
    logger.log("dispose", step="done")


def _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading,
                   stop_distance=None):
    """Turn to `target_heading`, then drive (slow, heading-held) toward a wall.

    Stops when a believed FRONT wall is within `stop_distance` (default the normal
    FRONT_STOP standoff), or a safety timeout (~2 arena lengths) elapses so it can
    never drive forever. Front-wall-referenced (the car turns to face whichever wall
    it wants and reads it with the front sensors), so it lands at a consistent gap
    regardless of how far it had to go -- pass a larger stop_distance to stop the car
    at a known distance FROM a wall (e.g. to reach the pit's middle off the far wall).

    Requires the car to be oriented within tolerance before a close-wall reading
    counts as "arrived", and enforces a short minimum drive time so a wall that is
    already in front (e.g. the far wall while still facing +y) cannot skip the leg.
    """
    if stop_distance is None:
        stop_distance = cfg.FRONT_STOP_DISTANCE_CM
    tol = cfg.IMU_TURN_TOLERANCE_DEG
    if not _spin_to_heading(logger, motors, cfg, imu, nav, target_heading):
        logger.log("drive_wall", step="warn", reason="orient incomplete",
                   target=target_heading)
    nav.target_heading = target_heading
    cm_s = cfg.DRIVE_CM_PER_S * (cfg.SLOW_SPEED / cfg.DRIVE_SPEED) if cfg.DRIVE_SPEED else 0.0
    deadline = time.time() + (2.0 * cfg.ARENA_LENGTH_CM / cm_s if cm_s > 0 else 30.0)
    min_ticks = max(5, int(0.5 / period))
    ticks = 0
    while time.time() < deadline:
        readings = sensors.read_all()
        front, agree = nav.end_wall_ahead(readings)
        ticks += 1
        if (ticks >= min_ticks
                and agree >= cfg.FRONT_AGREE_MIN_COUNT
                and front <= stop_distance
                and _heading_aligned(imu, nav, target_heading, tol)):
            logger.log("drive_wall", step="stop", front=front, agree=agree,
                       heading=target_heading)
            break
        cur = nav.rel_heading(imu.yaw() if imu is not None else None)
        steer = 0.0
        if cur is not None:
            steer = max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM,
                        cfg.HEADING_HOLD_GAIN * angle_diff(target_heading, cur)))
        motors.drive(logger, cfg.SLOW_SPEED, steer)
        time.sleep(period)
    motors.stop(logger)


def _approach_pit(logger, motors, cfg, imu, nav, disposer, sensors, period):
    """From the start wall: drive along it to pit x, face the pit, dump."""
    print("[pit] along start wall -> centre -> dump")

    if nav.x > cfg.PIT_X_CM + 5.0:
        along = -90.0
        dist = nav.x - cfg.PIT_X_CM
    else:
        along = 90.0
        dist = cfg.PIT_X_CM - nav.x

    _spin_to_heading(logger, motors, cfg, imu, nav, along)
    nav.target_heading = along
    if dist > 5.0:
        print(f"[pit] drive {dist:.0f} cm along wall to pit x")
        _drive_distance(logger, motors, cfg, dist, cfg.SLOW_SPEED)
    nav.set_pose(x=cfg.PIT_X_CM, y=cfg.FRONT_STOP_DISTANCE_CM, target_heading=along)

    _spin_to_heading(logger, motors, cfg, imu, nav, 0.0)
    nav.target_heading = 0.0
    _dispose(logger, motors, cfg, imu, nav, disposer, face_heading=0.0)
    nav.set_pose(x=cfg.PIT_X_CM, y=cfg.FRONT_STOP_DISTANCE_CM, target_heading=0.0)
    nav._done = True
    print("[pit] dump complete -- run finished.")


def _return_to_pit_and_dispose(logger, motors, cfg, imu, nav, disposer, sensors, period):
    """After the sweep: go back to the pit (mid start wall) and dump the remainder.

    Sensor-referenced legs re-anchor nav pose after each wall so stale odometry
    from the sweep cannot confuse later legs. The pit is on the start wall (y=0):
      1. Face the start wall and drive to it (from wherever the sweep ended).
      2. Face -x to the left wall, then +x until the right wall marks pit x.
      3. Face +y, reverse into the pit, and dump.
    """
    print("[return] coverage complete -> returning to the pit for the final dump")

    standoff = cfg.FRONT_STOP_DISTANCE_CM

    print("[return] leg 1: drive to the start wall (the pit's side)")
    _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading=180.0)
    nav.set_pose(y=standoff, target_heading=180.0)

    print("[return] leg 2a: face -x, drive to the left wall (front sensors)")
    _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading=-90.0)
    nav.set_pose(x=cfg.START_X_CM + standoff, target_heading=-90.0)

    print("[return] leg 2b: face +x, drive until the right wall marks the pit's middle")
    _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading=90.0,
                   stop_distance=cfg.ARENA_WIDTH_CM - cfg.PIT_X_CM)
    nav.set_pose(x=cfg.PIT_X_CM, target_heading=90.0)

    print("[return] leg 3: final dump into the pit")
    nav.target_heading = 0.0
    _dispose(logger, motors, cfg, imu, nav, disposer, face_heading=0.0)
    nav.set_pose(x=cfg.PIT_X_CM, y=standoff, target_heading=0.0)
    print("[return] final dump complete -- arena swept and emptied.")


def execute(logger, cmd, motors, cfg, imu, nav, disposer):
    if cmd.action is Action.FORWARD:
        motors.drive(logger, cmd.speed, cmd.steer)
    elif cmd.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
        direction = "left" if cmd.action is Action.TURN_LEFT else "right"
        _u_turn(logger, motors, cfg, direction, imu, nav)
    elif cmd.action is Action.DISPOSE:
        _dispose(logger, motors, cfg, imu, nav, disposer)
    elif cmd.action is Action.STOP:
        motors.stop(logger)


def _log_status(logger, nav, readings, yaw, cmd):
    """One structured line per control tick: mode, pose, sensors, IMU, action."""
    logger.log("nav", mode=nav.mode.name, action=cmd.action.name, reason=cmd.reason,
               x=nav.x, y=nav.y, heading=nav.heading_rel,
               target_heading=nav.target_heading, lane=nav._lane_index,
               lane_distance=nav.lane_distance, front_wall=nav.front_wall_cm,
               front_agree=nav.front_agree, yaw=yaw, blocks=nav.collector.count,
               **readings)


def run_drive_test(logger, motors, cfg, imu=None):
    """Blind, open-loop maneuver script -- no ultrasonic sensors.

    Forward/stop steps still run for their scripted number of seconds, but the
    "left"/"right" turn steps now rotate by TURN_ANGLE_DEG using IMU heading
    feedback (falling back to the timed spin if no IMU is present), instead of
    spinning for the hardcoded step duration.
    """
    turn_mode = ("IMU heading" if (imu is not None and imu.available
                 and getattr(cfg, "USE_IMU_TURN", False)) else f"timed {cfg.TURN_TIME_S}s")
    print(f"USE_SENSORS = False -> open-loop drive test (turns: {turn_mode})")
    for action, seconds in cfg.DRIVE_TEST_SEQUENCE:
        if action in ("left", "right"):
            logger.log("drive_test", step="turn", direction=action, deg=cfg.TURN_ANGLE_DEG)
            _spin_90(logger, motors, cfg, action, imu)
            continue
        logger.log("drive_test", step=action, seconds=seconds)
        if action == "forward":
            motors.drive(logger, cfg.DRIVE_SPEED, 0.0)
        else:  # "stop"
            motors.stop(logger)
        time.sleep(seconds)
    motors.stop(logger)


def _sync_shovel(front_servo, nav, last_phase):
    """Move the scoop when the hill phase changes."""
    if front_servo is None or not getattr(nav.cfg, "HILL_MODE", False):
        return last_phase
    phase = nav.phase
    if phase is last_phase:
        return last_phase
    if phase is Phase.CLIMB_FIRST:
        front_servo.climb()
    elif phase is Phase.SWEEP:
        front_servo.lower()
    elif phase in (Phase.CLIMB_LAST, Phase.DESCEND):
        front_servo.climb()
    return phase


def run_navigation(logger, motors, cfg, imu=None, front_servo=None, babysit=False):
    """IMU + odometry navigation loop with block disposal at the pit."""
    # Imported here so the drive-test mode never needs the sensor stack.
    from sensors import UltrasonicArray

    sensors = UltrasonicArray(cfg)
    disposer = Disposer(cfg)
    nav = NavigationController(cfg)
    period = 1.0 / cfg.CONTROL_LOOP_HZ

    # Zero the heading on the current facing so lane targets are start-relative.
    nav.set_origin(imu.yaw() if imu is not None else None)

    last_phase = None
    if front_servo is not None and getattr(cfg, "HILL_MODE", False):
        last_phase = Phase.CLIMB_FIRST

    turn_mode = "IMU heading" if (imu is not None and imu.available
                                  and cfg.USE_IMU_TURN) else f"timed {cfg.TURN_TIME_S}s"
    drive_mode = "IMU heading-hold"
    print("=" * 78)
    hill = getattr(cfg, "HILL_MODE", False)
    mode_label = ("hill: climb -> sweep end -> descend -> pit"
                  if hill else "IMU + wall-referenced navigation with disposal")
    print(f"USE_SENSORS = True -> {mode_label}")
    print(f"  arena         : {cfg.ARENA_WIDTH_CM:.0f} x {cfg.ARENA_LENGTH_CM:.0f} cm, "
          f"{cfg.NUM_LANES} lanes x {cfg.LANE_WIDTH_CM:.0f} cm (stop when all swept)")
    print(f"  start pose    : ({cfg.START_X_CM:.0f}, {cfg.START_Y_CM:.0f}) cm, heading 0")
    print(f"  pit           : ({cfg.PIT_X_CM:.0f}, {cfg.PIT_Y_CM:.0f}) cm, "
          f"arrive within {cfg.PIT_ARRIVAL_RADIUS_CM:.0f} cm")
    print(f"  collection cap: {cfg.COLLECTION_CAPACITY_BLOCKS} blocks")
    print(f"  front scoop   : start up {cfg.FRONT_SERVO_START_UP_S:.1f}s, then lift to "
          f"{cfg.FRONT_SERVO_UP_PULSE_MS:.3f} ms every {cfg.FRONT_SERVO_INTERVAL_S:.0f}s driving, "
          f"hold {cfg.FRONT_SERVO_HOLD_S:.1f}s, move {cfg.FRONT_SERVO_MOVE_S:.1f}s down<->up "
          f"(down {cfg.FRONT_SERVO_DOWN_PULSE_MS:.3f} ms)")
    print(f"  driving       : {drive_mode}   turns: {turn_mode}")
    print(f"  end of lane   : wall standoff {cfg.FRONT_STOP_DISTANCE_CM:.0f} cm, "
          f">={cfg.FRONT_AGREE_MIN_COUNT}/3 agree, hold {cfg.WALL_PERSIST_TICKS} ticks "
          f"(odometry backstop {cfg.LANE_END_MARGIN_CM:.0f} cm)")
    print(f"  sensors (hw)  : {sensors.using_hardware}")
    print("=" * 78)

    last_t = time.monotonic()
    drive_elapsed = 0.0          # driving time since the last front-scoop lift
    try:
        while True:
            now = time.monotonic()
            dt = now - last_t
            last_t = now

            readings = sensors.read_all()
            yaw = imu.yaw() if imu is not None else None
            cmd = nav.decide(readings, yaw, dt)
            last_phase = _sync_shovel(front_servo, nav, last_phase)
            _log_status(logger, nav, readings, yaw, cmd)

            if babysit:
                motors.stop(logger)
                result = input('Execute? (Y/n): ')
                if result.strip().lower() not in ('y', ''):
                    logger.log('babysit', ending_prematurely=True, reason='command rejected. input: ' + result)
                    break

            execute(logger, cmd, motors, cfg, imu, nav, disposer)

            if cmd.action is not Action.FORWARD:
                last_t = time.monotonic()   # dispose / U-turn blocked -- drop stale dt

            # Scoop only in the collection zone (hill mode) or on interval (legacy).
            if cmd.action is Action.FORWARD and front_servo is not None and nav.collecting:
                drive_elapsed += dt
                if drive_elapsed >= cfg.FRONT_SERVO_INTERVAL_S:
                    motors.stop(logger)  # HACK: this probably breaks more than it fixes
                    front_servo.lift_cycle()
                    drive_elapsed = 0.0
                    last_t = time.monotonic()   # scoop blocks -- don't bridge that gap

            if nav.phase is Phase.TO_PIT:
                print("[nav] start wall reached -> pit approach")
                _approach_pit(logger, motors, cfg, imu, nav, disposer, sensors, period)
                break
            if nav.mode is Mode.DONE and not hill:
                print(f"[nav] coverage complete: swept all {cfg.NUM_LANES} lanes")
                _return_to_pit_and_dispose(logger, motors, cfg, imu, nav, disposer, sensors, period)
                break
            time.sleep(period)
    finally:
        disposer.cleanup()
        sensors.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-format',
                        type=str, choices=('pretty', 'json'), default='pretty')
    parser.add_argument('--shell', action='store_true')
    parser.add_argument('--babysit', action='store_true')
    args = parser.parse_args()

    cfg = config
    logger = Logger(format=args.log_format)

    motors = MotorDriver(cfg)
    front_servo = FrontServo(cfg)
    # IMU is optional: if absent/disabled, IMU.available stays False, yaw()
    # returns None, and turns fall back to the timed spin while driving stays
    # open-loop straight (no heading-hold).
    imu = None
    if getattr(cfg, "USE_IMU_TURN", False):
        from imu import IMU
        imu = IMU(logger, cfg)
    try:
        front_servo.startup() if not cfg.HILL_MODE else front_servo.climb()
        if args.shell:
            breakpoint()
        elif cfg.USE_SENSORS:
            run_navigation(logger, motors, cfg, imu, front_servo, babysit=args.babysit)
        else:
            run_drive_test(logger, motors, cfg, imu)
    except KeyboardInterrupt:
        pass
    finally:
        motors.stop(logger)
        front_servo.cleanup()
        motors.cleanup()


if __name__ == "__main__":
    main()
