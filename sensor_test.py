"""
Standalone sensor read-out. No motors, no navigation -- just poll every
ultrasonic sensor and print its distance. Use it to check wiring and aim each
sensor at a wall to confirm the readings make sense.

Run with:  python3 sensor_test.py
Stop with: Ctrl-C

Off a Raspberry Pi the readings show as '--' (no GPIO), so this is mainly useful
on the car itself.
"""

import time

import config
from sensors import UltrasonicArray


def format_value(dist, enabled):
    if not enabled:
        return "off"
    return "--" if dist == float("inf") else f"{dist:.1f}"


def main():
    cfg = config
    sensors = UltrasonicArray(cfg)
    names = list(cfg.SENSORS)
    period = 1.0 / cfg.CONTROL_LOOP_HZ

    print(f"hardware sensors: {sensors.using_hardware}  (Ctrl-C to stop)")
    print("distances in cm; '--' = out of range / no echo; 'off' = disabled\n")
    print("  ".join(f"{n:>12}" for n in names))
    try:
        while True:
            readings = sensors.read_all()
            print("  ".join(
                f"{format_value(readings[n], sensors.is_enabled(n)):>12}"
                for n in names))
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        sensors.cleanup()


if __name__ == "__main__":
    main()
