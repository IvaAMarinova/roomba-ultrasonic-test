"""
HC-SR04 ultrasonic sensor driver.

On a Raspberry Pi (with RPi.GPIO available) this reads real sensors. On any
other machine it transparently falls back to returning float('inf') for every
sensor, or to a supplied `simulator` callable, so the navigation logic can be
exercised without hardware.
"""

import time
import statistics

import config as default_config

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except (ImportError, RuntimeError):
    GPIO = None
    _HAS_GPIO = False

INF = float("inf")


class UltrasonicArray:
    def __init__(self, cfg=default_config, simulator=None):
        """
        cfg:        configuration module (see config.py)
        simulator:  optional callable(name) -> distance_cm. When given, no GPIO
                    is touched and this is used as the source of readings.
        """
        self.cfg = cfg
        self.simulator = simulator
        self._gpio_ready = False
        if simulator is None and _HAS_GPIO:
            self._setup_gpio()

    @property
    def using_hardware(self):
        return self._gpio_ready

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        for spec in self.cfg.SENSORS.values():
            GPIO.setup(spec["trig"], GPIO.OUT)
            GPIO.setup(spec["echo"], GPIO.IN)
            GPIO.output(spec["trig"], False)
        time.sleep(0.05)  # let the sensors settle
        self._gpio_ready = True

    def read_all(self):
        """Return {sensor_name: distance_cm} for every configured sensor."""
        return {name: self.read(name) for name in self.cfg.SENSORS}

    def read(self, name):
        """Distance in cm for one sensor, or float('inf') if unknown/out of range."""
        if self.simulator is not None:
            return self.simulator(name)
        if not self._gpio_ready:
            return INF
        samples = [self._read_once(name) for _ in range(self.cfg.SENSOR_SAMPLES)]
        samples = [s for s in samples if s is not None]
        if not samples:
            return INF
        return statistics.median(samples)

    def _read_once(self, name):
        """A single trigger/echo cycle. Returns cm or None on timeout/out-of-range."""
        spec = self.cfg.SENSORS[name]
        trig, echo = spec["trig"], spec["echo"]

        # 10 microsecond trigger pulse.
        GPIO.output(trig, True)
        time.sleep(1e-5)
        GPIO.output(trig, False)

        # Wait for the echo to go high (start of the return pulse).
        start = time.monotonic()
        deadline = start + self.cfg.SENSOR_TIMEOUT_S
        while GPIO.input(echo) == 0:
            start = time.monotonic()
            if start > deadline:
                return None

        # Wait for the echo to go low again (end of the return pulse).
        end = start
        deadline = start + self.cfg.SENSOR_TIMEOUT_S
        while GPIO.input(echo) == 1:
            end = time.monotonic()
            if end > deadline:
                return None

        dist = (end - start) * self.cfg.SOUND_SPEED_CM_PER_S / 2.0
        if dist < self.cfg.SENSOR_MIN_RANGE_CM or dist > self.cfg.SENSOR_MAX_RANGE_CM:
            return None
        return dist

    def cleanup(self):
        if self._gpio_ready:
            GPIO.cleanup()
