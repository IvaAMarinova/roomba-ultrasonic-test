"""
Tank / skid-steer motor driver: one DIR pin + one PWM pin per motor.

Each side (left, right) has:
    dir  - a single direction line: forward = HIGH, reverse = LOW
           (flipped per side by the 'invert' flag in config.MOTORS)
    pwm  - speed as a PWM duty cycle, 0..100%

A side is commanded with a signed value in -1..1: the sign picks the direction
pin level, the magnitude becomes the PWM duty. Below MOTOR_DEADZONE the side is
stopped (0% duty).

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
        self._pwm = {}  # side -> PWM object
        if not self.dry_run:
            self._setup_gpio()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        for side, spec in self.cfg.MOTORS.items():
            GPIO.setup(spec["dir"], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(spec["pwm"], GPIO.OUT, initial=GPIO.LOW)
            pwm = GPIO.PWM(spec["pwm"], self.cfg.MOTOR_PWM_HZ)
            pwm.start(0)
            self._pwm[side] = pwm

    # -- high-level intents -------------------------------------------------

    def drive(self, logger, speed, steer=0.0):
        """Move at `speed` (signed: <0 = reverse), biasing with `steer` (-1 left .. +1 right)."""
        if speed > 0:
            steer += self.cfg.FORWARD_STEER_TRIM
        elif speed < 0:
            steer += getattr(self.cfg, "REVERSE_STEER_TRIM", 0.0)
        # Steering right slows the right side; steering left slows the left side.
        self._set_side(logger, "left", speed + steer)
        self._set_side(logger, "right", speed - steer)

    def turn_left(self, logger, speed):
        self._set_side(logger, "left", -speed)
        self._set_side(logger, "right", +speed)

    def turn_right(self, logger, speed):
        self._set_side(logger, "left", +speed)
        self._set_side(logger, "right", -speed)

    def stop(self, logger):
        try:
            self._set_side(logger, "left", 0.0)
            self._set_side(logger, "right", 0.0)
        except RuntimeError:
            pass  # GPIO already torn down at interpreter exit

    # -- hardware boundary --------------------------------------------------

    def _set_side(self, logger, side, command):
        """Drive one side from a signed command in -1..1 (sign = direction)."""
        command = _clamp(command)
        magnitude = abs(command)
        spec = self.cfg.MOTORS[side]

        if magnitude < self.cfg.MOTOR_DEADZONE:
            forward, duty = True, 0.0          # stopped
        else:
            forward, duty = (command > 0), magnitude * 100.0
        if spec.get("invert"):
            forward = not forward

        if self.dry_run:
            label = "stop" if duty == 0.0 else ("fwd" if forward else "rev")
            logger.log("motor", side=side, mode=label, duty=duty)
            return

        GPIO.output(spec["dir"], GPIO.HIGH if forward else GPIO.LOW)
        self._pwm[side].ChangeDutyCycle(duty)

    def cleanup(self):
        if self.dry_run or not self._pwm:
            return
        for pwm in self._pwm.values():
            pwm.stop()
        # Drop PWM refs before GPIO.cleanup(); otherwise PWM.__del__ runs at
        # interpreter shutdown and tries to stop() on an already-freed handle.
        self._pwm.clear()
        if _HAS_GPIO and GPIO.getmode() is not None:
            GPIO.cleanup()
