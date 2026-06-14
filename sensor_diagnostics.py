"""
Per-sensor diagnostics: print each sensor's pin assignment and the raw data
coming back on it. More verbose than sensor_test.py -- use this to debug wiring,
see which pings time out, and check sample-to-sample noise.

For every sensor it shows:
    TRIG pin, ECHO pin, enabled flag,
    the individual ping samples (cm, or 'timeout' for no echo),
    and the median distance the navigation code would actually use.

Run with:  python3 sensor_diagnostics.py
Stop with: Ctrl-C
"""

import time

import config
from sensors import UltrasonicArray

REFRESH_S = 0.5  # slow enough to read the per-sensor lines


def fmt_sample(s):
    return "timeout" if s is None else f"{s:.1f}"


def fmt_distance(d):
    return "--" if d == float("inf") else f"{d:.1f} cm"


def print_pin_map(cfg):
    print("Sensor pin map (BCM numbering):")
    for name, spec in cfg.SENSORS.items():
        state = "enabled" if spec.get("enabled", True) else "DISABLED"
        print(f"  {name:<13} trig=GPIO{spec['trig']:<3} echo=GPIO{spec['echo']:<3} [{state}]")
    print()


def main():
    cfg = config
    sensors = UltrasonicArray(cfg)

    print_pin_map(cfg)
    print(f"hardware sensors: {sensors.using_hardware}  (Ctrl-C to stop)\n")

    try:
        while True:
            for name, spec in cfg.SENSORS.items():
                raw = sensors.read_raw(name)
                dist = sensors.read(name)
                samples = "[" + ", ".join(fmt_sample(s) for s in raw) + "]"
                print(f"{name:<13} trig={spec['trig']:<3} echo={spec['echo']:<3} "
                      f"samples={samples:<28} -> {fmt_distance(dist)}")
            print("-" * 72)
            time.sleep(REFRESH_S)
    except KeyboardInterrupt:
        pass
    finally:
        sensors.cleanup()


if __name__ == "__main__":
    main()
