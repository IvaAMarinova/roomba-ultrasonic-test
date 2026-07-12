"""
Actuators: collection bucket state, front scoop servo, and disposal mechanism.

Mirrors motors.py / imu.py's hardware-boundary philosophy -- hardware drivers
fall back to dry-run logging off-Pi or when dependencies are missing.

  Collector  -- knows how many blocks are in the bucket and when it is full.
  FrontServo -- drives the front scoop on GPIO 18 (gpiozero Servo PWM).
  BackServo  -- opens/closes the rear dump door.
  Disposer   -- dump sequence at the pit (open door, hold, close).
"""

import threading
import time

import config as default_config

try:
    from gpiozero import Servo
    _HAS_SERVO = True
except (ImportError, RuntimeError):
    Servo = None
    _HAS_SERVO = False

# Wide pulse range for bench calibration scripts.
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


class PulseWidthServo:
    """gpiozero Servo driver with two calibrated endpoint pulse widths in ms.

    Endpoints may be in either order (e.g. back door closed can be higher pulse
    than open). gpiozero always gets min/max of the two endpoint values.
    """

    def __init__(
        self,
        cfg,
        *,
        pin,
        endpoint_a_ms,
        endpoint_b_ms,
        move_s,
        ramp_step_s,
        log_name,
        initial_pulse_ms=None,
        dry_run=None,
        calibration=False,
    ):
        self.cfg = cfg
        self.pin = pin
        self.log_name = log_name
        self.calibration = calibration
        self.move_s = move_s
        self.ramp_step_s = ramp_step_s
        self.dry_run = (not _HAS_SERVO) if dry_run is None else dry_run
        self._servo = None
        if initial_pulse_ms is None:
            initial_pulse_ms = endpoint_a_ms
        self.pulse_ms = initial_pulse_ms

        if calibration:
            self._min_pulse_ms = SERVO_HW_MIN_PULSE_S * 1000.0
            self._max_pulse_ms = SERVO_HW_MAX_PULSE_S * 1000.0
        else:
            self._min_pulse_ms = min(endpoint_a_ms, endpoint_b_ms)
            self._max_pulse_ms = max(endpoint_a_ms, endpoint_b_ms)

        if not self.dry_run:
            if calibration:
                min_pulse_s = SERVO_HW_MIN_PULSE_S
                max_pulse_s = SERVO_HW_MAX_PULSE_S
                initial = pulse_ms_to_value(
                    initial_pulse_ms,
                    self._min_pulse_ms,
                    self._max_pulse_ms,
                )
            else:
                min_pulse_s = self._min_pulse_ms / 1000.0
                max_pulse_s = self._max_pulse_ms / 1000.0
                initial = pulse_ms_to_value(
                    initial_pulse_ms,
                    self._min_pulse_ms,
                    self._max_pulse_ms,
                )
            self._servo = Servo(
                pin,
                min_pulse_width=min_pulse_s,
                max_pulse_width=max_pulse_s,
                initial_value=initial,
            )

    def _write_pulse_ms(self, pulse_ms):
        self.pulse_ms = pulse_ms
        return pulse_ms_to_value(pulse_ms, self._min_pulse_ms, self._max_pulse_ms)

    def move_to_pulse_ms(self, pulse_ms, *, log=True, ramp=None):
        """Command the servo to `pulse_ms`, ramping unless in calibration mode."""
        pulse_ms = max(self._min_pulse_ms, min(self._max_pulse_ms, pulse_ms))
        if ramp is None:
            ramp = not self.calibration and self.move_s > 0

        start = self.pulse_ms
        if not ramp or abs(pulse_ms - start) < 1e-9:
            value = self._write_pulse_ms(pulse_ms)
            if self.dry_run:
                if log:
                    print(f"[{self.log_name}] -> {pulse_ms:.3f} ms "
                          f"(dry run, value={value:+.3f})")
                return value
            if self._servo is not None:
                self._servo.value = value
            if log:
                print(f"[{self.log_name}] -> {pulse_ms:.3f} ms")
            return value

        span = self._max_pulse_ms - self._min_pulse_ms
        if span <= 0:
            move_duration_s = 0.0
        else:
            move_duration_s = self.move_s * abs(pulse_ms - start) / span
        steps = max(1, round(move_duration_s / self.ramp_step_s))
        step_delay = move_duration_s / steps

        for i in range(1, steps + 1):
            t = i / steps
            intermediate = start + (pulse_ms - start) * t
            value = self._write_pulse_ms(intermediate)
            if not self.dry_run and self._servo is not None:
                self._servo.value = value
            if i < steps:
                time.sleep(step_delay)

        if log:
            print(f"[{self.log_name}] -> {pulse_ms:.3f} ms ({move_duration_s:.2f}s ramp)")
        return value

    def cleanup(self):
        if self._servo is not None:
            self._servo.detach()
            self._servo.close()
            self._servo = None


class FrontServo:
    """The front collection scoop ("багер") servo on FRONT_SERVO_PIN."""

    def __init__(self, cfg=default_config, dry_run=None, calibration=False):
        self.cfg = cfg
        self._driver = PulseWidthServo(
            cfg,
            pin=cfg.FRONT_SERVO_PIN,
            endpoint_a_ms=cfg.FRONT_SERVO_DOWN_PULSE_MS,
            endpoint_b_ms=cfg.FRONT_SERVO_UP_PULSE_MS,
            move_s=cfg.FRONT_SERVO_MOVE_S,
            ramp_step_s=cfg.FRONT_SERVO_RAMP_STEP_S,
            log_name="front-servo",
            initial_pulse_ms=cfg.FRONT_SERVO_UP_PULSE_MS,
            dry_run=dry_run,
            calibration=calibration,
        )

    @property
    def dry_run(self):
        return self._driver.dry_run

    @property
    def pulse_ms(self):
        return self._driver.pulse_ms

    @property
    def _min_pulse_ms(self):
        return self._driver._min_pulse_ms

    @property
    def _max_pulse_ms(self):
        return self._driver._max_pulse_ms

    def move_to_pulse_ms(self, pulse_ms, *, log=True, ramp=None):
        return self._driver.move_to_pulse_ms(pulse_ms, log=log, ramp=ramp)

    def raise_up(self):
        self.move_to_pulse_ms(self.cfg.FRONT_SERVO_UP_PULSE_MS)

    def lower(self):
        self.move_to_pulse_ms(self.cfg.FRONT_SERVO_DOWN_PULSE_MS)

    def lift_cycle(self):
        self.raise_up()
        time.sleep(self.cfg.FRONT_SERVO_HOLD_S)
        self.lower()

    def startup(self):
        self.raise_up()
        hold_s = self.cfg.FRONT_SERVO_START_UP_S
        if hold_s > 0:
            print(f"[front-servo] holding up for {hold_s:.1f}s (regulation start size)")
            time.sleep(hold_s)
        self.lower()

    def cleanup(self):
        self._driver.cleanup()


class BackServo:
    """Rear dump door servo on BACK_SERVO_PIN."""

    def __init__(self, cfg=default_config, dry_run=None, calibration=False):
        self.cfg = cfg
        self._driver = PulseWidthServo(
            cfg,
            pin=cfg.BACK_SERVO_PIN,
            endpoint_a_ms=cfg.BACK_SERVO_CLOSED_PULSE_MS,
            endpoint_b_ms=cfg.BACK_SERVO_OPEN_PULSE_MS,
            move_s=cfg.BACK_SERVO_MOVE_S,
            ramp_step_s=cfg.BACK_SERVO_RAMP_STEP_S,
            log_name="back-servo",
            initial_pulse_ms=cfg.BACK_SERVO_CLOSED_PULSE_MS,
            dry_run=dry_run,
            calibration=calibration,
        )

    @property
    def dry_run(self):
        return self._driver.dry_run

    @property
    def pulse_ms(self):
        return self._driver.pulse_ms

    @property
    def _min_pulse_ms(self):
        return self._driver._min_pulse_ms

    @property
    def _max_pulse_ms(self):
        return self._driver._max_pulse_ms

    def move_to_pulse_ms(self, pulse_ms, *, log=True, ramp=None):
        return self._driver.move_to_pulse_ms(pulse_ms, log=log, ramp=ramp)

    def close_door(self):
        self.move_to_pulse_ms(self.cfg.BACK_SERVO_CLOSED_PULSE_MS)

    def open_door(self):
        self.move_to_pulse_ms(self.cfg.BACK_SERVO_OPEN_PULSE_MS)

    def cleanup(self):
        self._driver.cleanup()


class Disposer:
    """The back-of-car disposal mechanism: open the rear door to dump into the pit."""

    def __init__(self, cfg=default_config):
        self.cfg = cfg
        self._door = BackServo(cfg)

    def start_opening(self):
        """Begin opening the rear door on a background thread (ramped move).

        Pair with reverse driving so the door lifts while backing over the pit.
        Call join_opening() before hold_open_and_close().
        """
        print("[disposer] opening rear door")
        thread = threading.Thread(target=self._door.open_door, daemon=True)
        thread.start()
        return thread

    def join_opening(self, thread):
        """Wait until start_opening()'s ramp finishes."""
        if thread is not None:
            thread.join()

    def hold_open_and_close(self):
        """Hold with the rear door open, then close it."""
        time.sleep(self.cfg.DISPOSE_HOLD_S)
        print("[disposer] closing rear door")
        self._door.close_door()

    def dump_cycle(self):
        """Open the rear door, hold while balls fall out, then close."""
        self.start_opening().join()
        self.hold_open_and_close()

    def dump(self):
        """Hold with door open, then close (door should already be opening/open)."""
        print("[disposer] DUMP")
        self.hold_open_and_close()

    def cleanup(self):
        self._door.cleanup()
