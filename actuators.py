"""
Future-servo placeholders: the collection bucket ("багер") and the disposal
mechanism. Mirrors motors.py / imu.py's hardware-boundary philosophy -- right
now these do nothing but track state and log intent, so the navigation state
machine can be wired and tested before any servo exists. When the servos land,
only the bodies of the methods here change; the callers don't.

  Collector -- knows how many blocks are in the bucket and when it is full.
  Disposer  -- performs the actual dump at the pit (the back-of-car "tip").
"""

import config as default_config


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
    """The front collection scoop ("багер") servo.

    While driving, the scoop is raised to FRONT_SERVO_UP_DEG on a timer and then
    returned to FRONT_SERVO_DOWN_DEG (the periodic lift is driven by main's loop;
    this class just holds the target angle and logs each move).

    TODO(servo): drive the real servo PWM on FRONT_SERVO_PIN here. Today it only
    tracks/logs the angle so the timing can be exercised without hardware. Dry-run
    safe off-Pi like the other drivers.
    """

    def __init__(self, cfg=default_config):
        self.cfg = cfg
        self.angle = cfg.FRONT_SERVO_DOWN_DEG

    def move_to(self, deg):
        """Command the servo to `deg`. Placeholder until the servo lands."""
        self.angle = deg
        print(f"[front-servo] -> {deg:.0f}deg (placeholder -- no servo yet)")

    def raise_up(self):
        """Lift the scoop to the raised angle (FRONT_SERVO_UP_DEG)."""
        self.move_to(self.cfg.FRONT_SERVO_UP_DEG)

    def lower(self):
        """Return the scoop to its resting angle (FRONT_SERVO_DOWN_DEG)."""
        self.move_to(self.cfg.FRONT_SERVO_DOWN_DEG)


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
