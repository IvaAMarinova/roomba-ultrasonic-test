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
from navigation import NavigationController, Action, Mode, angle_diff


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


def _spin_to_heading(logger, motors, cfg, imu, nav, target_rel):
    """Rotate in place until the car's heading ~= target_rel (start-relative deg).

    Re-picks the turn direction each poll so a small overshoot is corrected. No-op
    (and returns False) if no IMU heading is available.
    """
    cur = nav.rel_heading(imu.yaw()) if imu is not None else None
    if cur is None:
        logger.log("orient", step="skip", reason="no IMU heading")
        return False
    err0 = angle_diff(target_rel, cur)
    logger.log("orient", step="turn", target=target_rel, current=cur, error=err0)
    deadline = time.time() + cfg.IMU_TURN_TIMEOUT_S
    while time.time() < deadline:
        cur = nav.rel_heading(imu.yaw())
        if cur is not None:
            err = angle_diff(target_rel, cur)
            if abs(err) <= cfg.IMU_TURN_TOLERANCE_DEG:
                break
            (motors.turn_right if err > 0 else motors.turn_left)(logger, cfg.TURN_SPEED)
        time.sleep(cfg.IMU_TURN_POLL_S)
    motors.stop(logger)
    return True


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
      2. Reverse DISPOSE_REVERSE_CM to place the rear directly over the pit.
      3. Dump (placeholder until the disposal servo lands -- see actuators.Disposer).
      4. Pull the same distance forward to get clear of the pit.
      5. Re-orient to the lane heading so the sweep resumes cleanly.

    face_heading: the heading to face so the back points at the pit. If None (the
    normal in-sweep pass) it is computed from the live pose (nav.bearing_to_pit).
    The final return-to-pit dump passes it explicitly, because after the blocking
    return legs the pose estimate is stale. Afterwards nav.complete_dispose()
    empties the (stub) bucket and returns to DRIVING.
    """
    if cfg.DISPOSE_BACK_INTO_PIT:
        # Face AWAY from the pit so the back (the tip side) points into it.
        target = (face_heading if face_heading is not None
                  else angle_diff(nav.bearing_to_pit() + 180.0, 0.0))
        _spin_to_heading(logger, motors, cfg, imu, nav, target)

    rev_s = _drive_distance(motors, cfg, cfg.DISPOSE_REVERSE_CM, -cfg.DISPOSE_REVERSE_SPEED)
    logger.log("dispose", step="reverse", cm=cfg.DISPOSE_REVERSE_CM, seconds=rev_s)

    logger.log("dispose", step="dump", hold_s=cfg.DISPOSE_HOLD_S)
    disposer.dump()

    _drive_distance(motors, cfg, cfg.DISPOSE_REVERSE_CM, cfg.DISPOSE_REVERSE_SPEED)
    logger.log("dispose", step="clear", cm=cfg.DISPOSE_REVERSE_CM)

    # Point back down the lane before handing control back to the sweep.
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
    """
    if stop_distance is None:
        stop_distance = cfg.FRONT_STOP_DISTANCE_CM
    _spin_to_heading(logger, motors, cfg, imu, nav, target_heading)
    nav.target_heading = target_heading
    cm_s = cfg.DRIVE_CM_PER_S * (cfg.SLOW_SPEED / cfg.DRIVE_SPEED) if cfg.DRIVE_SPEED else 0.0
    deadline = time.time() + (2.0 * cfg.ARENA_LENGTH_CM / cm_s if cm_s > 0 else 30.0)
    while time.time() < deadline:
        readings = sensors.read_all()
        front, agree = nav.front_wall(readings)
        if agree >= cfg.FRONT_AGREE_MIN_COUNT and front <= stop_distance:
            break
        cur = nav.rel_heading(imu.yaw() if imu is not None else None)
        steer = 0.0
        if cur is not None:
            steer = max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM,
                        cfg.HEADING_HOLD_GAIN * angle_diff(target_heading, cur)))
        motors.drive(logger, cfg.SLOW_SPEED, steer)
        time.sleep(period)
    motors.stop(logger)


def _return_to_pit_and_dispose(logger, motors, cfg, imu, nav, disposer, sensors, period):
    """After the sweep: go back to the pit (mid start wall) and dump the remainder.

    The pit is in the MIDDLE of the start wall (y=0), where no wall marks its
    sideways position -- so we reach the left/right walls with the FRONT sensors
    (turning to face each) and measure across:
      1. Drive to the start wall (the pit's side), wherever the sweep ended.
      2. Drive to the LEFT wall (exact x), then drive PIT_X across to the pit's middle.
      3. Face the start wall so the back points at the pit, reverse in, and dump.
    """
    print("[return] coverage complete -> returning to the pit for the final dump")

    # 1) Onto the pit's wall (start wall, y=0). heading 180 = -y.
    print("[return] leg 1: drive to the start wall (the pit's side)")
    _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading=180.0)

    # 2) To the pit's middle -- fully front-wall-referenced (no odometry):
    #    go to the left wall, then face +x and drive until the FAR (right) wall is
    #    (WIDTH - PIT_X) away, which puts the car at x = PIT_X.
    print("[return] leg 2a: face -x, drive to the left wall (front sensors)")
    _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading=-90.0)
    print("[return] leg 2b: face +x, drive until the right wall marks the pit's middle")
    _drive_to_wall(logger, motors, cfg, imu, nav, sensors, period, target_heading=90.0,
                   stop_distance=cfg.ARENA_WIDTH_CM - cfg.PIT_X_CM)

    # 3) Final dump: face +y so the BACK points at the start-wall pit, reverse in.
    print("[return] leg 3: final dump into the pit")
    _dispose(logger, motors, cfg, imu, nav, disposer, face_heading=0.0)
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


def run_navigation(logger, motors, cfg, imu=None, front_servo=None):
    """IMU + odometry navigation loop with block disposal at the pit."""
    # Imported here so the drive-test mode never needs the sensor stack.
    from sensors import UltrasonicArray

    sensors = UltrasonicArray(cfg)
    disposer = Disposer(cfg)
    nav = NavigationController(cfg)
    period = 1.0 / cfg.CONTROL_LOOP_HZ

    # Zero the heading on the current facing so lane targets are start-relative.
    nav.set_origin(imu.yaw() if imu is not None else None)

    turn_mode = "IMU heading" if (imu is not None and imu.available
                                  and cfg.USE_IMU_TURN) else f"timed {cfg.TURN_TIME_S}s"
    drive_mode = "IMU heading-hold"
    print("=" * 78)
    print("USE_SENSORS = True -> IMU + wall-referenced navigation with disposal")
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
            _log_status(logger, nav, readings, yaw, cmd)
            execute(logger, cmd, motors, cfg, imu, nav, disposer)

            # Periodically raise the front scoop while cruising. The timer counts
            # only FORWARD time, so it pauses through the (blocking) U-turns and
            # disposal maneuvers rather than firing right after one.
            if cmd.action is Action.FORWARD and front_servo is not None:
                drive_elapsed += dt
                if drive_elapsed >= cfg.FRONT_SERVO_INTERVAL_S:
                    front_servo.lift_cycle()
                    drive_elapsed = 0.0

            if nav.mode is Mode.DONE:
                print(f"[nav] coverage complete: swept all {cfg.NUM_LANES} lanes")
                _return_to_pit_and_dispose(motors, cfg, imu, nav, disposer, sensors, period)
                break
            time.sleep(period)
    finally:
        disposer.cleanup()
        sensors.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-format',
                        type=str, choices=('pretty', 'json'), default='pretty')
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
        front_servo.startup()
        if cfg.USE_SENSORS:
            run_navigation(logger, motors, cfg, imu, front_servo)
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
