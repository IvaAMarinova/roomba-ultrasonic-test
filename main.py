"""
Main control loop: read sensors -> decide -> drive, repeat.

Run on the Pi with:   python3 main.py
Off the Pi it still runs (sensors read as 'inf', motors print) so you can sanity
check the loop, but for real logic testing use simulate.py / test_navigation.py.
"""

import time

import config
from sensors import UltrasonicArray
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


def main():
    cfg = config
    sensors = UltrasonicArray(cfg)
    motors = MotorDriver(cfg)
    nav = NavigationController(cfg)
    period = 1.0 / cfg.CONTROL_LOOP_HZ

    print(f"hardware sensors: {sensors.using_hardware}")
    try:
        while True:
            readings = sensors.read_all()
            cmd = nav.decide(readings)
            print(f"{_format(readings)} -> {cmd.action.name:<10} ({cmd.reason})")
            execute(cmd, motors, cfg)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        motors.stop()
        motors.cleanup()
        sensors.cleanup()


if __name__ == "__main__":
    main()
