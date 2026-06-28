"""
Entry point. Two modes, selected by config.USE_SENSORS:

  USE_SENSORS = False  -> open-loop drive test: run the hardcoded maneuver
                          script (config.DRIVE_TEST_SEQUENCE) so you can check
                          the motors / H-bridges / turning on the real car.
                          The sensors are never touched.
  USE_SENSORS = True   -> sensor-driven navigation: read sensors -> decide ->
                          drive, repeat.

Run on the Pi with:  python3 main.py
"""

import time

import config
from motors import MotorDriver
from navigation import NavigationController, Action


def _angle_diff(a, b):
    """Shortest signed difference a - b, normalized to (-180, 180] degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


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
        step = _angle_diff(cur, prev)
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


def _advance_one_lane(motors, cfg):
    """Drive straight by LANE_WIDTH_CM (the sideways shift into the next lane)."""
    motors.drive(cfg.DRIVE_SPEED, 0.0)
    time.sleep(cfg.LANE_WIDTH_CM / cfg.DRIVE_CM_PER_S)
    motors.stop()


def execute(cmd, motors, cfg, imu=None):
    if cmd.action is Action.FORWARD:
        motors.drive(cmd.speed, cmd.steer)
    elif cmd.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
        # End-of-lane U-turn: rotate 90 deg, shift one lane width to the side,
        # rotate 90 deg the same way to face back down the next lane. This whole
        # maneuver is ONE decision; the steps below are logged so the second
        # spin isn't a surprise (no sensors are read while it runs).
        direction = "left" if cmd.action is Action.TURN_LEFT else "right"
        shift_s = cfg.LANE_WIDTH_CM / cfg.DRIVE_CM_PER_S
        print(f"    [u-turn] spin {direction} {cfg.TURN_ANGLE_DEG:.0f}deg")
        _spin_90(motors, cfg, direction, imu)
        print(f"    [u-turn] forward {cfg.LANE_WIDTH_CM:.0f}cm lane shift ({shift_s:.2f}s)")
        _advance_one_lane(motors, cfg)
        print(f"    [u-turn] spin {direction} {cfg.TURN_ANGLE_DEG:.0f}deg")
        _spin_90(motors, cfg, direction, imu)
        print("    [u-turn] done, resuming sensing")
    elif cmd.action is Action.STOP:
        motors.stop()


def _format(readings):
    return " ".join(
        f"{name}={('--' if dist == float('inf') else f'{dist:5.1f}')}"
        for name, dist in readings.items()
    )


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
    """Sensor-driven navigation loop."""
    # Imported here so the drive-test mode never needs the sensor stack.
    from sensors import UltrasonicArray

    sensors = UltrasonicArray(cfg)
    nav = NavigationController(cfg)
    period = 1.0 / cfg.CONTROL_LOOP_HZ
    turn_mode = "IMU heading" if (imu is not None and imu.available
                                  and cfg.USE_IMU_TURN) else f"timed {cfg.TURN_TIME_S}s"
    print(f"USE_SENSORS = True -> navigation (hardware sensors: "
          f"{sensors.using_hardware}, turns: {turn_mode})")
    try:
        while True:
            readings = sensors.read_all()
            cmd = nav.decide(readings)
            print(f"{_format(readings)} -> {cmd.action.name:<10} ({cmd.reason})")
            execute(cmd, motors, cfg, imu)
            time.sleep(period)
    finally:
        sensors.cleanup()


def main():
    cfg = config
    motors = MotorDriver(cfg)
    # IMU is optional: if absent/disabled, IMU.available stays False and turns
    # transparently fall back to the timed TURN_TIME_S spin.
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
