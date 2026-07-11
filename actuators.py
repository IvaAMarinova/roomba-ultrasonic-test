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

# Match the working roomba servo_test.py pulse widths (0.5 ms .. 2.5 ms).
_SERVO_MIN_PULSE_S = 0.0005
_SERVO_MAX_PULSE_S = 0.0025


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

    Uses gpiozero Servo with the same 0.5–2.5 ms pulse range as servo_test.py.
    Configured angles map linearly to servo values: DOWN -> -1.0 (MIN pulse),
    UP -> +1.0 (MAX pulse). Each move_to() sets the target once (no sweep).
    """

    def __init__(self, cfg=default_config, dry_run=None):
        self.cfg = cfg
        self.angle = cfg.FRONT_SERVO_UP_DEG
        self.dry_run = (not _HAS_SERVO) if dry_run is None else dry_run
        self._servo = None
        if not self.dry_run:
            self._servo = Servo(
                cfg.FRONT_SERVO_PIN,
                min_pulse_width=_SERVO_MIN_PULSE_S,
                max_pulse_width=_SERVO_MAX_PULSE_S,
                initial_value=self._deg_to_value(cfg.FRONT_SERVO_UP_DEG),
            )

    def _deg_to_value(self, deg):
        """Map configured scoop angle to gpiozero Servo value in [-1.0, 1.0]."""
        down = self.cfg.FRONT_SERVO_DOWN_DEG
        up = self.cfg.FRONT_SERVO_UP_DEG
        span = up - down
        if span == 0:
            return 0.0
        t = (deg - down) / span
        t = max(0.0, min(1.0, t))
        return -1.0 + 2.0 * t

    def move_to(self, deg):
        """Command the servo to `deg` with a single PWM target (no sweep)."""
        self.angle = deg
        value = self._deg_to_value(deg)
        if self.dry_run:
            print(f"[front-servo] -> {deg:.0f}deg (dry run, value={value:+.2f})")
            return
        self._servo.value = value
        print(f"[front-servo] -> {deg:.0f}deg")

    def raise_up(self):
        """Lift the scoop to the raised angle (FRONT_SERVO_UP_DEG)."""
        self.move_to(self.cfg.FRONT_SERVO_UP_DEG)

    def lower(self):
        """Return the scoop to its resting angle (FRONT_SERVO_DOWN_DEG)."""
        self.move_to(self.cfg.FRONT_SERVO_DOWN_DEG)

    def lift_cycle(self):
        """Raise, hold at the top, then lower -- same sequence as main.py while driving."""
        self.raise_up()
        time.sleep(self.cfg.FRONT_SERVO_HOLD_S)
        self.lower()

    def startup(self):
        """Hold the scoop raised at launch, then lower to the collecting position.

        The scoop is already commanded up on init. Waits FRONT_SERVO_START_UP_S
        (0 = lower immediately) before moving to the down/collecting angle.
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
