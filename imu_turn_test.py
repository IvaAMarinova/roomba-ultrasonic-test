"""
Bench test for the IMU closed-loop turn logic -- NO MOTORS DRIVEN.

Reads the BNO086 heading and accumulates rotation with the exact same wrap-safe,
glitch-rejecting math as main._spin_imu, but driven by you turning the robot BY
HAND. Rotate the car and it reports the measured angle, announcing when the
TURN_ANGLE_DEG target is reached -- so you can trust the heading math before it
ever commands the motors.

Run:  ~/robot_env/bin/python3 imu_turn_test.py
"""

import time

import config
from imu import IMU


def _angle_diff(a, b):
    """Shortest signed difference a - b, normalized to (-180, 180] degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


def main():
    imu = IMU(config)
    if not imu.available:
        print("No IMU available -> cannot bench-test closed-loop turn.")
        raise SystemExit(1)

    target = config.TURN_ANGLE_DEG - config.IMU_TURN_TOLERANCE_DEG
    print(f"\nRotate the robot by hand. Target {config.TURN_ANGLE_DEG:.0f} deg "
          f"(stops {config.IMU_TURN_TOLERANCE_DEG:.0f} short, at {target:.0f}). "
          f"Ctrl-C to quit.\n")

    while True:
        prev = imu.yaw()
        if prev is None:
            time.sleep(0.05)
            continue
        turned = 0.0
        print("--- new turn: start rotating now (20s window) ---")
        deadline = time.time() + 20.0
        reached = False
        while time.time() < deadline:
            time.sleep(config.IMU_TURN_POLL_S)
            cur = imu.yaw()
            if cur is None:
                continue
            step = _angle_diff(cur, prev)
            prev = cur
            if abs(step) > config.IMU_GLITCH_MAX_STEP_DEG:
                continue  # corrupted I2C read -- ignore
            turned += step
            print(f"  turned {turned:+7.1f} deg", end="\r", flush=True)
            if abs(turned) >= target:
                print(f"\n  >>> reached {turned:+.1f} deg -> motors would STOP here <<<\n")
                reached = True
                break
        if not reached:
            print(f"\n  (window elapsed at {turned:+.1f} deg, restarting)\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
