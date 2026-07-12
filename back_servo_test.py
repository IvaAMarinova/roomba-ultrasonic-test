"""
Continuous rear dump door test.

Runs the same open/hold/close sequence as Disposer.dump() at the pit: open to
BACK_SERVO_OPEN_PULSE_MS, hold DISPOSE_HOLD_S, close to BACK_SERVO_CLOSED_PULSE_MS,
then pause DISPOSE_HOLD_S before the next cycle.

Run on the Pi:  python3 back_servo_test.py
                (requires pigpiod — actuators.py selects it automatically)
Stop with:     Ctrl-C
"""

import time

import config
from actuators import Disposer


def main():
    cfg = config
    disposer = Disposer(cfg)

    print("Rear dump door test (same sequence as main.py disposal)")
    print(f"  open pulse  : {cfg.BACK_SERVO_OPEN_PULSE_MS:.3f} ms")
    print(f"  closed pulse: {cfg.BACK_SERVO_CLOSED_PULSE_MS:.3f} ms")
    print(f"  open hold   : {cfg.DISPOSE_HOLD_S:.1f} s")
    print(f"  move speed  : {cfg.BACK_SERVO_MOVE_S:.1f} s full closed<->open")
    print(f"  pause closed: {cfg.DISPOSE_HOLD_S:.1f} s")
    print("Ctrl-C to stop\n")

    try:
        while True:
            disposer.dump_cycle()
            time.sleep(cfg.DISPOSE_HOLD_S)
    except KeyboardInterrupt:
        pass
    finally:
        disposer.cleanup()


if __name__ == "__main__":
    main()
