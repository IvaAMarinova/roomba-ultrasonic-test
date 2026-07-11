"""
Interactive front scoop servo calibration.

Adjust the scoop angle with +/- (or arrow keys) and read the matching gpiozero
servo value and pulse width. Uses the same PWM mapping as actuators.FrontServo.

Run on the Pi:  python3 front_servo_calibrate.py
                GPIOZERO_PIN_FACTORY=pigpio python3 front_servo_calibrate.py
Quit with:     q
"""

import curses

import config
from actuators import FrontServo, _SERVO_MAX_PULSE_S, _SERVO_MIN_PULSE_S

FINE_STEP_DEG = 1.0
COARSE_STEP_DEG = 5.0


def _pulse_ms(value):
    """Convert gpiozero Servo value [-1, 1] to pulse width in milliseconds."""
    t = (value + 1.0) / 2.0
    return (_SERVO_MIN_PULSE_S + t * (_SERVO_MAX_PULSE_S - _SERVO_MIN_PULSE_S)) * 1000.0


def _apply(servo, angle_deg):
    value = servo._deg_to_value(angle_deg)
    servo.angle = angle_deg
    if not servo.dry_run and servo._servo is not None:
        servo._servo.value = value
    return value


def _draw(stdscr, servo, angle_deg, value, status):
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    if height < 16 or width < 48:
        stdscr.addstr(0, 0, "Terminal too small (need at least 48x16).")
        stdscr.refresh()
        return

    cfg = servo.cfg
    pulse = _pulse_ms(value)
    clamped = angle_deg < cfg.FRONT_SERVO_DOWN_DEG or angle_deg > cfg.FRONT_SERVO_UP_DEG
    mode = "dry run" if servo.dry_run else f"GPIO {cfg.FRONT_SERVO_PIN}"

    lines = [
        "Front scoop servo calibration",
        "",
        f"  angle        {angle_deg:7.1f} deg",
        f"  servo value  {value:+7.3f}",
        f"  pulse width  {pulse:7.3f} ms",
        f"  backend      {mode}",
        "",
        "  config.py reference:",
        f"    FRONT_SERVO_DOWN_DEG = {cfg.FRONT_SERVO_DOWN_DEG:.1f}",
        f"    FRONT_SERVO_UP_DEG   = {cfg.FRONT_SERVO_UP_DEG:.1f}",
        "",
    ]
    if clamped:
        lines.append("  (servo value clamped outside configured down/up range)")
        lines.append("")

    lines.extend([
        "  +/- or Left/Right     step 1 deg",
        "  Up/Down               step 5 deg",
        "  d / u                 jump to configured down / up",
        "  q                     quit",
    ])
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

    servo = FrontServo(config)
    angle_deg = config.FRONT_SERVO_DOWN_DEG
    value = _apply(servo, angle_deg)
    status = ""

    try:
        while True:
            _draw(stdscr, servo, angle_deg, value, status)
            key = stdscr.getch()

            if key in (ord("q"), ord("Q")):
                break

            changed = False
            if key in (ord("+"), ord("=")):
                angle_deg += FINE_STEP_DEG
                changed = True
            elif key == ord("-"):
                angle_deg -= FINE_STEP_DEG
                changed = True
            elif key == curses.KEY_RIGHT:
                angle_deg += FINE_STEP_DEG
                changed = True
            elif key == curses.KEY_LEFT:
                angle_deg -= FINE_STEP_DEG
                changed = True
            elif key == curses.KEY_UP:
                angle_deg += COARSE_STEP_DEG
                changed = True
            elif key == curses.KEY_DOWN:
                angle_deg -= COARSE_STEP_DEG
                changed = True
            elif key in (ord("d"), ord("D")):
                angle_deg = config.FRONT_SERVO_DOWN_DEG
                status = "jumped to configured down angle"
                changed = True
            elif key in (ord("u"), ord("U")):
                angle_deg = config.FRONT_SERVO_UP_DEG
                status = "jumped to configured up angle"
                changed = True
            else:
                status = ""

            if changed:
                value = _apply(servo, angle_deg)
                if not status:
                    status = "updated"
    finally:
        servo.cleanup()


def main():
    curses.wrapper(_run)


if __name__ == "__main__":
    main()
