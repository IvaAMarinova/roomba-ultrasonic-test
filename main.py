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

import config
from actuators import Disposer
from motors import MotorDriver
from navigation import NavigationController, Action, Mode, angle_diff


def _spin_timed(motors, cfg, direction):
    """Original open-loop ~90 deg in-place rotation (no IMU)."""
    (motors.turn_left if direction == "left" else motors.turn_right)(cfg.TURN_SPEED)
    time.sleep(cfg.TURN_TIME_S)
    motors.stop()


def _spin_imu(motors, cfg, direction, imu):
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

    turn_fn(speed)
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
            turn_fn(speed)
            boosted = True
            print(f"    [u-turn] stall: only {abs(turned):.1f}deg in "
                  f"{boost_after:.1f}s (< half of {cfg.TURN_ANGLE_DEG:.0f}) "
                  f"-> boost spin speed {cfg.TURN_SPEED:.2f} -> {speed:.2f}")
    motors.stop()
    return abs(turned)


def _spin_90(motors, cfg, direction, imu=None):
    """Rotate ~TURN_ANGLE_DEG in place.

    Uses IMU heading feedback when an IMU is present and USE_IMU_TURN is set;
    otherwise (or if the IMU yields no reading) falls back to the timed spin.
    """
    if imu is not None and imu.available and getattr(cfg, "USE_IMU_TURN", False):
        turned = _spin_imu(motors, cfg, direction, imu)
        if turned is not None:
            print(f"    [u-turn] IMU spin {direction} {turned:.1f}deg "
                  f"(target {cfg.TURN_ANGLE_DEG:.0f})")
            return
        print("    [u-turn] IMU gave no heading -> timed spin fallback")
    _spin_timed(motors, cfg, direction)


def _spin_to_heading(motors, cfg, imu, nav, target_rel):
    """Rotate in place until the car's heading ~= target_rel (start-relative deg).

    Re-picks the turn direction each poll so a small overshoot is corrected. No-op
    (and returns False) if no IMU heading is available.
    """
    cur = nav.rel_heading(imu.yaw()) if imu is not None else None
    if cur is None:
        print("[orient] no IMU heading -> skipping orient")
        return False
    err0 = angle_diff(target_rel, cur)
    print(f"[orient] to {target_rel:.0f}deg (cur {cur:.0f}, err {err0:.0f})")
    deadline = time.time() + cfg.IMU_TURN_TIMEOUT_S
    while time.time() < deadline:
        cur = nav.rel_heading(imu.yaw())
        if cur is not None:
            err = angle_diff(target_rel, cur)
            if abs(err) <= cfg.IMU_TURN_TOLERANCE_DEG:
                break
            (motors.turn_right if err > 0 else motors.turn_left)(cfg.TURN_SPEED)
        time.sleep(cfg.IMU_TURN_POLL_S)
    motors.stop()
    return True


def _advance_one_lane(motors, cfg):
    """Drive straight by LANE_WIDTH_CM (the sideways shift into the next lane)."""
    motors.drive(cfg.DRIVE_SPEED, 0.0)
    time.sleep(cfg.LANE_WIDTH_CM / cfg.DRIVE_CM_PER_S)
    motors.stop()


def _u_turn(motors, cfg, direction, imu, nav):
    """End-of-lane U-turn: spin 90, shift one lane width, spin 90 -- one decision.

    No sensors are read while it runs; the steps are logged so the second spin
    isn't a surprise. Afterwards nav.complete_turn() folds the net motion (heading
    reversed, one lane shifted) back into the odometry pose.
    """
    shift_s = cfg.LANE_WIDTH_CM / cfg.DRIVE_CM_PER_S
    print(f"    [u-turn] spin {direction} {cfg.TURN_ANGLE_DEG:.0f}deg")
    _spin_90(motors, cfg, direction, imu)
    print(f"    [u-turn] forward {cfg.LANE_WIDTH_CM:.0f}cm lane shift ({shift_s:.2f}s)")
    _advance_one_lane(motors, cfg)
    print(f"    [u-turn] spin {direction} {cfg.TURN_ANGLE_DEG:.0f}deg")
    _spin_90(motors, cfg, direction, imu)
    nav.complete_turn()
    print(f"    [u-turn] done, resuming sweep ({nav.pose_str()})")


def _drive_distance(motors, cfg, distance_cm, speed):
    """Drive straight for a fixed distance (cm) at `speed` (signed: <0 = reverse).

    Open-loop timed off DRIVE_CM_PER_S (measured at DRIVE_SPEED). Short move, so
    the timing is good enough even on a bumpy floor.
    """
    cm_per_s = cfg.DRIVE_CM_PER_S * (abs(speed) / cfg.DRIVE_SPEED)
    seconds = distance_cm / cm_per_s if cm_per_s > 0 else 0.0
    motors.drive(speed, 0.0)
    time.sleep(seconds)
    motors.stop()
    return seconds


def _dispose(motors, cfg, imu, nav, disposer, face_heading=None):
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
        _spin_to_heading(motors, cfg, imu, nav, target)

    rev_s = _drive_distance(motors, cfg, cfg.DISPOSE_REVERSE_CM, -cfg.DISPOSE_REVERSE_SPEED)
    print(f"[dispose] reversed {cfg.DISPOSE_REVERSE_CM:.0f}cm ({rev_s:.2f}s) "
          f"to seat the rear over the pit")

    print(f"[dispose] holding {cfg.DISPOSE_HOLD_S:.1f}s while dumping")
    time.sleep(cfg.DISPOSE_HOLD_S)
    disposer.dump()

    _drive_distance(motors, cfg, cfg.DISPOSE_REVERSE_CM, cfg.DISPOSE_REVERSE_SPEED)
    print(f"[dispose] pulled {cfg.DISPOSE_REVERSE_CM:.0f}cm forward, clear of the pit")

    # Point back down the lane before handing control back to the sweep.
    _spin_to_heading(motors, cfg, imu, nav, nav.target_heading)
    nav.complete_dispose()
    print("[dispose] done, resuming sweep")


def _drive_to_wall(motors, cfg, imu, nav, sensors, period, target_heading):
    """Turn to `target_heading`, then drive (slow, heading-held) up to a wall.

    Stops when a believed front wall is within the standoff, or a safety timeout
    (~2 arena lengths) elapses so it can never drive forever. Wall-referenced, so
    it lands at a consistent standoff regardless of how far it had to go.
    """
    _spin_to_heading(motors, cfg, imu, nav, target_heading)
    nav.target_heading = target_heading
    cm_s = cfg.DRIVE_CM_PER_S * (cfg.SLOW_SPEED / cfg.DRIVE_SPEED) if cfg.DRIVE_SPEED else 0.0
    deadline = time.time() + (2.0 * cfg.ARENA_LENGTH_CM / cm_s if cm_s > 0 else 30.0)
    while time.time() < deadline:
        readings = sensors.read_all()
        front, agree = nav.front_wall(readings)
        if agree >= cfg.FRONT_AGREE_MIN_COUNT and front <= cfg.FRONT_STOP_DISTANCE_CM:
            break
        cur = nav.rel_heading(imu.yaw() if imu is not None else None)
        steer = 0.0
        if cur is not None:
            steer = max(-cfg.MAX_HEADING_TRIM, min(cfg.MAX_HEADING_TRIM,
                        cfg.HEADING_HOLD_GAIN * angle_diff(target_heading, cur)))
        motors.drive(cfg.SLOW_SPEED, steer)
        time.sleep(period)
    motors.stop()


def _return_to_pit_and_dispose(motors, cfg, imu, nav, disposer, sensors, period):
    """After the sweep: go back to the pit (mid start wall) and dump the remainder.

    The pit is in the MIDDLE of the start wall (y=0), where no wall marks its
    sideways position -- so we reference the side wall and measure across:
      1. Drive to the start wall (the pit's side), wherever the sweep ended.
      2. Drive to the LEFT side wall (exact x), then drive PIT_X across to the pit's
         middle.
      3. Face the start wall so the back points at the pit, reverse in, and dump.
    """
    print("[return] coverage complete -> returning to the pit for the final dump")

    # 1) Onto the pit's wall (start wall, y=0). heading 180 = -y.
    print("[return] leg 1: drive to the start wall (the pit's side)")
    _drive_to_wall(motors, cfg, imu, nav, sensors, period, target_heading=180.0)

    # 2) To the pit's middle, measured from the left wall. heading -90 = -x.
    print("[return] leg 2: reference the left wall, then measure across to the middle")
    _drive_to_wall(motors, cfg, imu, nav, sensors, period, target_heading=-90.0)
    across = cfg.PIT_X_CM - cfg.FRONT_STOP_DISTANCE_CM  # left-wall standoff ~ car x
    if across > 0:
        _spin_to_heading(motors, cfg, imu, nav, 90.0)     # face +x
        nav.target_heading = 90.0
        secs = _drive_distance(motors, cfg, across, cfg.SLOW_SPEED)
        print(f"[return] moved {across:.0f}cm from the left wall to the pit's middle ({secs:.2f}s)")

    # 3) Final dump: face +y so the BACK points at the start-wall pit, reverse in.
    print("[return] leg 3: final dump into the pit")
    _dispose(motors, cfg, imu, nav, disposer, face_heading=0.0)
    print("[return] final dump complete -- arena swept and emptied.")


def execute(cmd, motors, cfg, imu, nav, disposer):
    if cmd.action is Action.FORWARD:
        motors.drive(cmd.speed, cmd.steer)
    elif cmd.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
        direction = "left" if cmd.action is Action.TURN_LEFT else "right"
        _u_turn(motors, cfg, direction, imu, nav)
    elif cmd.action is Action.DISPOSE:
        _dispose(motors, cfg, imu, nav, disposer)
    elif cmd.action is Action.STOP:
        motors.stop()


def _format(readings):
    return " ".join(
        f"{name}={('--' if dist == float('inf') else f'{dist:5.1f}')}"
        for name, dist in readings.items()
    )


def _log_status(nav, readings, yaw, cmd):
    """One structured line per control tick: mode, pose, sensors, IMU, action."""
    yaw_s = "--" if yaw is None else f"{yaw:6.1f}"
    print(f"MODE={nav.mode.name:<9} | {nav.pose_str()} | {_format(readings)} "
          f"| yaw={yaw_s} | {nav.collector} | -> {cmd.action.name:<10} ({cmd.reason})")


def run_drive_test(motors, cfg, imu=None):
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
            print(f"[drive-test] turn {action} {cfg.TURN_ANGLE_DEG:.0f}deg")
            _spin_90(motors, cfg, action, imu)
            continue
        print(f"[drive-test] {action:<7} for {seconds:.2f}s")
        if action == "forward":
            motors.drive(cfg.DRIVE_SPEED, 0.0)
        else:  # "stop"
            motors.stop()
        time.sleep(seconds)
    motors.stop()


def run_navigation(motors, cfg, imu=None):
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
    drive_mode = "wall-follow" if cfg.USE_WALL_FOLLOW else "IMU heading-hold"
    print("=" * 78)
    print("USE_SENSORS = True -> IMU + wall-referenced navigation with disposal")
    print(f"  arena         : {cfg.ARENA_WIDTH_CM:.0f} x {cfg.ARENA_LENGTH_CM:.0f} cm, "
          f"{cfg.NUM_LANES} lanes x {cfg.LANE_WIDTH_CM:.0f} cm (stop when all swept)")
    print(f"  start pose    : ({cfg.START_X_CM:.0f}, {cfg.START_Y_CM:.0f}) cm, heading 0")
    print(f"  pit           : ({cfg.PIT_X_CM:.0f}, {cfg.PIT_Y_CM:.0f}) cm, "
          f"arrive within {cfg.PIT_ARRIVAL_RADIUS_CM:.0f} cm")
    print(f"  collection cap: {cfg.COLLECTION_CAPACITY_BLOCKS} blocks")
    print(f"  driving       : {drive_mode}   turns: {turn_mode}")
    print(f"  end of lane   : wall standoff {cfg.FRONT_STOP_DISTANCE_CM:.0f} cm, "
          f">={cfg.FRONT_AGREE_MIN_COUNT}/3 agree, hold {cfg.WALL_PERSIST_TICKS} ticks "
          f"(odometry backstop {cfg.LANE_END_MARGIN_CM:.0f} cm)")
    print(f"  sensors (hw)  : {sensors.using_hardware}")
    print("=" * 78)

    last_t = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            dt = now - last_t
            last_t = now

            readings = sensors.read_all()
            yaw = imu.yaw() if imu is not None else None
            cmd = nav.decide(readings, yaw, dt)
            _log_status(nav, readings, yaw, cmd)
            execute(cmd, motors, cfg, imu, nav, disposer)
            if nav.mode is Mode.DONE:
                print(f"[nav] coverage complete: swept all {cfg.NUM_LANES} lanes")
                _return_to_pit_and_dispose(motors, cfg, imu, nav, disposer, sensors, period)
                break
            time.sleep(period)
    finally:
        sensors.cleanup()


def main():
    cfg = config
    motors = MotorDriver(cfg)
    # IMU is optional: if absent/disabled, IMU.available stays False, yaw()
    # returns None, and turns fall back to the timed spin while driving stays
    # open-loop straight (no heading-hold).
    imu = None
    if getattr(cfg, "USE_IMU_TURN", False):
        from imu import IMU
        imu = IMU(cfg)
    try:
        if cfg.USE_SENSORS:
            run_navigation(motors, cfg, imu)
        else:
            run_drive_test(motors, cfg, imu)
    except KeyboardInterrupt:
        pass
    finally:
        motors.stop()
        motors.cleanup()


if __name__ == "__main__":
    main()
