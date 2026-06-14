"""
Tank / skid-steer motor driver for two H-bridges.

Each side (left, right) is one H-bridge with two direction pins (IN1, IN2) and
an optional enable pin:

    forward : IN1=1, IN2=0
    reverse : IN1=0, IN2=1
    stop    : IN1=0, IN2=0

If a side's `en` pin is configured, speed is controlled with PWM on that pin.
If `en` is None (enable tied high), the driver runs direction-only: any side
command past MOTOR_DEADZONE drives that side at full speed, which still gives
bang-bang steering (the inside wheel cuts out on a hard trim) and clean
in-place turns.

Off a Raspberry Pi (no RPi.GPIO) it runs in "dry" mode and prints intent so the
navigation logic can be exercised without hardware.
"""

import config as default_config

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except (ImportError, RuntimeError):
    GPIO = None
    _HAS_GPIO = False


def _clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


class MotorDriver:
    def __init__(self, cfg=default_config, dry_run=None):
        self.cfg = cfg
        self.dry_run = (not _HAS_GPIO) if dry_run is None else dry_run
        self._pwm = {}  # side -> PWM object (only for sides with an enable pin)
        if not self.dry_run:
            self._setup_gpio()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        for side, spec in self.cfg.MOTORS.items():
            GPIO.setup(spec["in1"], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(spec["in2"], GPIO.OUT, initial=GPIO.LOW)
            if spec.get("en") is not None:
                GPIO.setup(spec["en"], GPIO.OUT, initial=GPIO.LOW)
                pwm = GPIO.PWM(spec["en"], self.cfg.MOTOR_PWM_HZ)
                pwm.start(0)
                self._pwm[side] = pwm

    # -- high-level intents -------------------------------------------------

    def drive(self, speed, steer=0.0):
        """Move forward at `speed`, biasing with `steer` (-1 left .. +1 right)."""
        # Steering right slows the right side; steering left slows the left side.
        self._set_side("left", speed + steer)
        self._set_side("right", speed - steer)

    def turn_left(self, speed):
        self._set_side("left", -speed)
        self._set_side("right", +speed)

    def turn_right(self, speed):
        self._set_side("left", +speed)
        self._set_side("right", -speed)

    def stop(self):
        self._set_side("left", 0.0)
        self._set_side("right", 0.0)

    # -- hardware boundary --------------------------------------------------

    def _set_side(self, side, command):
        """Drive one side from a signed command in -1..1 (sign = direction)."""
        command = _clamp(command)
        magnitude = abs(command)
        if magnitude < self.cfg.MOTOR_DEADZONE:
            direction = 0          # stop
        elif command > 0:
            direction = +1         # forward
        else:
            direction = -1         # reverse

        spec = self.cfg.MOTORS[side]
        has_pwm = spec.get("en") is not None
        # Without PWM, anything not stopped runs at full speed.
        duty = magnitude * 100.0 if has_pwm else (100.0 if direction else 0.0)

        if self.dry_run:
            name = {0: "stop", 1: "fwd", -1: "rev"}[direction]
            print(f"[motor] {side:<5} {name:<4} duty={duty:5.1f}%")
            return

        in1 = 1 if direction > 0 else 0
        in2 = 1 if direction < 0 else 0
        GPIO.output(spec["in1"], in1)
        GPIO.output(spec["in2"], in2)
        if has_pwm:
            self._pwm[side].ChangeDutyCycle(duty)

    def cleanup(self):
        if self.dry_run:
            return
        for pwm in self._pwm.values():
            pwm.stop()
        GPIO.cleanup()
