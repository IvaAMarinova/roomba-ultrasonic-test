"""
Continuous front scoop lift test.

Runs the same startup sequence and lift cycle as main.py: raise to
FRONT_SERVO_UP_PULSE_MS, hold FRONT_SERVO_HOLD_S, lower to FRONT_SERVO_DOWN_PULSE_MS,
then pause FRONT_SERVO_HOLD_S before the next cycle.

Run on the Pi:  python3 front_servo_test.py
                (requires pigpiod — actuators.py selects it automatically)
Stop with:     Ctrl-C
"""

import time

import config
from actuators import FrontServo


def main():
    cfg = config
    front_servo = FrontServo(cfg)

    print("Front scoop lift test (same sequence as main.py)")
    print(f"  up pulse   : {cfg.FRONT_SERVO_UP_PULSE_MS:.3f} ms")
    print(f"  down pulse : {cfg.FRONT_SERVO_DOWN_PULSE_MS:.3f} ms")
    print(f"  hold at top: {cfg.FRONT_SERVO_HOLD_S:.1f} s")
    print(f"  move speed : {cfg.FRONT_SERVO_MOVE_S:.1f} s full down<->up")
    print(f"  pause at down: {cfg.FRONT_SERVO_HOLD_S:.1f} s")
    print("Ctrl-C to stop\n")

    try:
        front_servo.startup()
        while True:
            front_servo.lift_cycle()
            time.sleep(cfg.FRONT_SERVO_HOLD_S)
    except KeyboardInterrupt:
        pass
    finally:
        front_servo.cleanup()


if __name__ == "__main__":
    main()
