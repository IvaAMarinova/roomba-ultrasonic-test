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


def execute(cmd, motors, cfg):
    if cmd.action is Action.FORWARD:
        motors.drive(cmd.speed, cmd.steer)
    elif cmd.action is Action.TURN_LEFT:
        motors.turn_left(cmd.speed)
        time.sleep(cfg.TURN_TIME_S)   # open-loop 90 deg turn, no IMU
        motors.stop()
    elif cmd.action is Action.TURN_RIGHT:
        motors.turn_right(cmd.speed)
        time.sleep(cfg.TURN_TIME_S)   # open-loop 90 deg turn, no IMU
        motors.stop()
    elif cmd.action is Action.STOP:
        motors.stop()


def _format(readings):
    return " ".join(
        f"{name}={('--' if dist == float('inf') else f'{dist:5.1f}')}"
        for name, dist in readings.items()
    )


def run_drive_test(motors, cfg):
    """Blind, open-loop maneuver script -- no sensors, just exercise the drive."""
    moves = {
        "forward": lambda: motors.drive(cfg.DRIVE_SPEED, 0.0),
        "left":    lambda: motors.turn_left(cfg.TURN_SPEED),
        "right":   lambda: motors.turn_right(cfg.TURN_SPEED),
        "stop":    motors.stop,
    }
    print("USE_SENSORS = False -> running open-loop drive test")
    for action, seconds in cfg.DRIVE_TEST_SEQUENCE:
        print(f"[drive-test] {action:<7} for {seconds:.2f}s")
        moves[action]()
        time.sleep(seconds)
    motors.stop()


def run_navigation(motors, cfg):
    """Sensor-driven navigation loop."""
    # Imported here so the drive-test mode never needs the sensor stack.
    from sensors import UltrasonicArray

    sensors = UltrasonicArray(cfg)
    nav = NavigationController(cfg)
    period = 1.0 / cfg.CONTROL_LOOP_HZ
    print(f"USE_SENSORS = True -> navigation (hardware sensors: "
          f"{sensors.using_hardware})")
    try:
        while True:
            readings = sensors.read_all()
            cmd = nav.decide(readings)
            print(f"{_format(readings)} -> {cmd.action.name:<10} ({cmd.reason})")
            execute(cmd, motors, cfg)
            time.sleep(period)
    finally:
        sensors.cleanup()


def main():
    cfg = config
    motors = MotorDriver(cfg)
    try:
        if cfg.USE_SENSORS:
            run_navigation(motors, cfg)
        else:
            run_drive_test(motors, cfg)
    except KeyboardInterrupt:
        pass
    finally:
        motors.stop()
        motors.cleanup()


if __name__ == "__main__":
    main()
