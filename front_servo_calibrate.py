"""
Interactive front scoop servo calibration.

Adjust the pulse width with +/- (or arrow keys) and read the current value in ms.
Uses the hardware pulse range for exploration; d/u jump to configured down/up.

Run on the Pi:  python3 front_servo_calibrate.py
                GPIOZERO_PIN_FACTORY=pigpio python3 front_servo_calibrate.py
Quit with:     q
"""

import curses

import config
from actuators import FrontServo, pulse_ms_to_value

FINE_STEP_MS = 0.010
COARSE_STEP_MS = 0.050


def _draw(stdscr, servo, pulse_ms, value, status):
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    if height < 16 or width < 48:
        stdscr.addstr(0, 0, "Terminal too small (need at least 48x16).")
        stdscr.refresh()
        return

    cfg = servo.cfg
    mode = "dry run" if servo.dry_run else f"GPIO {cfg.FRONT_SERVO_PIN}"

    lines = [
        "Front scoop servo calibration",
        "",
        f"  pulse width  {pulse_ms:7.3f} ms",
        f"  servo value  {value:+7.3f}",
        f"  backend      {mode}",
        "",
        "  config.py reference:",
        f"    FRONT_SERVO_DOWN_PULSE_MS = {cfg.FRONT_SERVO_DOWN_PULSE_MS:.3f}",
        f"    FRONT_SERVO_UP_PULSE_MS   = {cfg.FRONT_SERVO_UP_PULSE_MS:.3f}",
        "",
        "  +/- or Left/Right     step 0.010 ms",
        "  Up/Down               step 0.050 ms",
        "  d / u                 jump to configured down / up",
        "  q                     quit",
    ]
    if status:
        lines.extend(["", f"  {status}"])

    for row, line in enumerate(lines):
        if row >= height - 1:
            break
        stdscr.addnstr(row, 0, line, width - 1)

    stdscr.refresh()


def _run(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.keypad(True)

    servo = FrontServo(config, calibration=True)
    pulse_ms = config.FRONT_SERVO_DOWN_PULSE_MS
    value = servo.move_to_pulse_ms(pulse_ms, log=False)
    status = ""

    try:
        while True:
            _draw(stdscr, servo, pulse_ms, value, status)
            key = stdscr.getch()

            if key in (ord("q"), ord("Q")):
                break

            changed = False
            if key in (ord("+"), ord("=")):
                pulse_ms += FINE_STEP_MS
                changed = True
            elif key == ord("-"):
                pulse_ms -= FINE_STEP_MS
                changed = True
            elif key == curses.KEY_RIGHT:
                pulse_ms += FINE_STEP_MS
                changed = True
            elif key == curses.KEY_LEFT:
                pulse_ms -= FINE_STEP_MS
                changed = True
            elif key == curses.KEY_UP:
                pulse_ms += COARSE_STEP_MS
                changed = True
            elif key == curses.KEY_DOWN:
                pulse_ms -= COARSE_STEP_MS
                changed = True
            elif key in (ord("d"), ord("D")):
                pulse_ms = config.FRONT_SERVO_DOWN_PULSE_MS
                status = "jumped to configured down pulse"
                changed = True
            elif key in (ord("u"), ord("U")):
                pulse_ms = config.FRONT_SERVO_UP_PULSE_MS
                status = "jumped to configured up pulse"
                changed = True
            else:
                status = ""

            if changed:
                pulse_ms = max(servo._min_pulse_ms, min(servo._max_pulse_ms, pulse_ms))
                value = pulse_ms_to_value(pulse_ms, servo._min_pulse_ms, servo._max_pulse_ms)
                servo.move_to_pulse_ms(pulse_ms, log=False)
                if not status:
                    status = "updated"
    finally:
        servo.cleanup()


def main():
    curses.wrapper(_run)


if __name__ == "__main__":
    main()
