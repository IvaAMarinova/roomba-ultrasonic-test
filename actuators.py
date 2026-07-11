"""
Actuators: collection bucket state, front scoop servo, and disposal mechanism.

Mirrors motors.py / imu.py's hardware-boundary philosophy -- hardware drivers
fall back to dry-run logging off-Pi or when dependencies are missing.

  Collector  -- knows how many blocks are in the bucket and when it is full.
  FrontServo -- drives the front scoop on GPIO 18 (gpiozero Servo PWM).
  Disposer   -- performs the actual dump at the pit (the back-of-car "tip").
"""

import time

import config as default_config

try:
    from gpiozero import Servo
    _HAS_SERVO = True
except (ImportError, RuntimeError):
    Servo = None
    _HAS_SERVO = False

# Wide pulse range for bench calibration (front_servo_calibrate.py).
SERVO_HW_MIN_PULSE_S = 0.0005
SERVO_HW_MAX_PULSE_S = 0.0025


def pulse_ms_to_value(pulse_ms, min_pulse_ms, max_pulse_ms):
    """Map a pulse width in ms to gpiozero Servo value [-1.0, 1.0]."""
    span = max_pulse_ms - min_pulse_ms
    if span == 0:
        return 0.0
    t = (pulse_ms - min_pulse_ms) / span
    t = max(0.0, min(1.0, t))
    return -1.0 + 2.0 * t


class Collector:
    """The front collection bucket and its (future) block-count servo.

    TODO(servo): the block count will come from the collection servo/sensor. For
    now `count` is a stub -- it stays 0 unless something calls add()/reset(), so
    is_full() is effectively off and disposal triggers purely on reaching the pit
    (see NavigationController). Wiring the real count in is then a one-liner.
    """

    def __init__(self, cfg=default_config):
        self.cfg = cfg
        self.capacity = cfg.COLLECTION_CAPACITY_BLOCKS
        self.count = 0

    def add(self, n=1):
        """Register that `n` more blocks entered the bucket (future: from servo)."""
        self.count += n

    def is_full(self):
        return self.count >= self.capacity

    def reset(self):
        """Bucket emptied (after a dump)."""
        self.count = 0

    def __str__(self):
        return f"blocks={self.count}/{self.capacity}{' FULL' if self.is_full() else ''}"


class FrontServo:
    """The front collection scoop ("багер") servo on FRONT_SERVO_PIN.

    Down and up positions are configured as pulse widths in milliseconds.
    gpiozero min/max pulse widths are set to those endpoints so value -1.0 is
    down and +1.0 is up. Each move sets the target once (no sweep).
    """

    def __init__(self, cfg=default_config, dry_run=None, calibration=False):
        self.cfg = cfg
        self.pulse_ms = cfg.FRONT_SERVO_UP_PULSE_MS
        self.calibration = calibration
        self.dry_run = (not _HAS_SERVO) if dry_run is None else dry_run
        self._servo = None
        if not self.dry_run:
            if calibration:
                min_pulse_s = SERVO_HW_MIN_PULSE_S
                max_pulse_s = SERVO_HW_MAX_PULSE_S
                initial = pulse_ms_to_value(
                    cfg.FRONT_SERVO_DOWN_PULSE_MS,
                    min_pulse_s * 1000.0,
                    max_pulse_s * 1000.0,
                )
            else:
                min_pulse_s = cfg.FRONT_SERVO_DOWN_PULSE_MS / 1000.0
                max_pulse_s = cfg.FRONT_SERVO_UP_PULSE_MS / 1000.0
                initial = 1.0
            self._servo = Servo(
                cfg.FRONT_SERVO_PIN,
                min_pulse_width=min_pulse_s,
                max_pulse_width=max_pulse_s,
                initial_value=initial,
            )
        self._min_pulse_ms = (
            SERVO_HW_MIN_PULSE_S * 1000.0 if calibration
            else cfg.FRONT_SERVO_DOWN_PULSE_MS
        )
        self._max_pulse_ms = (
            SERVO_HW_MAX_PULSE_S * 1000.0 if calibration
            else cfg.FRONT_SERVO_UP_PULSE_MS
        )

    def move_to_pulse_ms(self, pulse_ms, *, log=True):
        """Command the servo to `pulse_ms` with a single PWM target (no sweep)."""
        pulse_ms = max(self._min_pulse_ms, min(self._max_pulse_ms, pulse_ms))
        self.pulse_ms = pulse_ms
        value = pulse_ms_to_value(pulse_ms, self._min_pulse_ms, self._max_pulse_ms)
        if self.dry_run:
            if log:
                print(f"[front-servo] -> {pulse_ms:.3f} ms (dry run, value={value:+.3f})")
            return value
        self._servo.value = value
        if log:
            print(f"[front-servo] -> {pulse_ms:.3f} ms")
        return value

    def raise_up(self):
        """Lift the scoop to the raised pulse width (FRONT_SERVO_UP_PULSE_MS)."""
        self.move_to_pulse_ms(self.cfg.FRONT_SERVO_UP_PULSE_MS)

    def lower(self):
        """Return the scoop to the resting pulse width (FRONT_SERVO_DOWN_PULSE_MS)."""
        self.move_to_pulse_ms(self.cfg.FRONT_SERVO_DOWN_PULSE_MS)

    def lift_cycle(self):
        """Raise, hold at the top, then lower -- same sequence as main.py while driving."""
        self.raise_up()
        time.sleep(self.cfg.FRONT_SERVO_HOLD_S)
        self.lower()

    def startup(self):
        """Hold the scoop raised at launch, then lower to the collecting position.

        The scoop is already commanded up on init. Waits FRONT_SERVO_START_UP_S
        (0 = lower immediately) before moving to the down/collecting position.
        """
        self.raise_up()
        hold_s = self.cfg.FRONT_SERVO_START_UP_S
        if hold_s > 0:
            print(f"[front-servo] holding up for {hold_s:.1f}s (regulation start size)")
            time.sleep(hold_s)
        self.lower()

    def cleanup(self):
        """Stop PWM pulses and release the GPIO pin."""
        if self._servo is not None:
            self._servo.detach()
            self._servo.close()
            self._servo = None


class Disposer:
    """The back-of-car disposal mechanism (waste-truck style tip into the pit).

    TODO(servo): drive the disposal servo here to tip the bucket. Today it just
    logs and dwells, so the DISPOSING state machine can be exercised without
    hardware. Dry-run safe off-Pi like the other drivers.
    """

    def __init__(self, cfg=default_config):
        self.cfg = cfg

    def dump(self):
        """Tip the collected blocks out the back. Placeholder until the servo lands."""
        print("[disposer] DUMP: tipping bucket out the back into the pit "
              "(placeholder -- no servo yet)")
