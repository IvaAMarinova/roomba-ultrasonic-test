"""
BNO086 IMU wrapper, used for closed-loop (heading-feedback) turns.

Exposes a single useful reading: yaw() -- the car's heading about the vertical
axis, in degrees. The navigation code accumulates yaw change during a spin so it
can rotate by a measured angle instead of for a hardcoded number of seconds.

Mirrors motors.py's philosophy: if the IMU stack or hardware isn't present this
runs in a disabled mode (``available == False``) and yaw() returns None, so the
caller transparently falls back to the original timed turn.

Driver note: adafruit_bno08x 1.3.3 chokes on some BNO086 packets the Pi's
hardware I2C delivers corrupted (UNKNOWN report type / KeyError on report_id 0).
This is usually clock-stretching on the Pi -- see Adafruit's guide and add
``dtparam=i2c_arm_baudrate=400000`` to /boot/firmware/config.txt, or use
software I2C (i2c-gpio overlay + adafruit-extended-bus).

We monkey-patch _handle_packet() to skip command-channel traffic and to drop
corrupted/unknown sensor batches instead of crashing the turn loop.
"""

import math
import time

try:
    import board
    import busio
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO08X,
        BNO_CHANNEL_EXE,
        BNO_CHANNEL_SHTP_COMMAND,
    )
    import adafruit_bno08x as _bno08x
    from adafruit_bno08x.i2c import BNO08X_I2C

    _HAS_IMU_LIBS = True
except Exception:  # noqa: BLE001 - board/busio raise NotImplementedError off-Pi
    _HAS_IMU_LIBS = False
    _bno08x = None


if _HAS_IMU_LIBS:
    def _handle_packet_tolerant(self, packet):
        """Skip command traffic and drop corrupted I2C batches (don't crash)."""
        if packet.channel_number in (BNO_CHANNEL_SHTP_COMMAND, BNO_CHANNEL_EXE):
            return
        try:
            _bno08x._separate_batch(packet, self._packet_slices)
            while len(self._packet_slices) > 0:
                self._process_report(*self._packet_slices.pop())
        except KeyError:
            # Unknown report_id -- garbled packet from Pi hardware I2C.
            self._packet_slices.clear()
        except RuntimeError as error:
            self._packet_slices.clear()
            # Incomplete batch during enable_feature(); init retries handle that.
            if not (error.args and error.args[0] == "Unprocessable Batch bytes"):
                raise

    BNO08X._handle_packet = _handle_packet_tolerant


class IMU:
    """Heading source backed by a BNO086 over I2C (pins 3/5)."""

    def __init__(self, logger, cfg=None, dry_run=None):
        self.available = False
        self._bno = None
        if dry_run or (dry_run is None and not _HAS_IMU_LIBS):
            logger.log("imu", status="absent", fallback="timed turns")
            return
        try:
            self._init_sensor()
            self.available = True
            logger.log("imu", status="ready")
        except Exception as e:  # noqa: BLE001 - sensor raises bare RuntimeError
            logger.log("imu", status="init_failed", error=str(e), fallback="timed turns")
            self.available = False

    def _init_sensor(self, max_attempts=8):
        i2c = busio.I2C(board.SCL, board.SDA)
        last_error = None
        for attempt in range(1, max_attempts + 1):
            try:
                try:
                    bno = BNO08X_I2C(i2c, address=0x4B)
                except (ValueError, RuntimeError):
                    bno = BNO08X_I2C(i2c, address=0x4A)
                bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                _ = bno.quaternion  # confirm a real sample before declaring ready
                self._bno = bno
                return
            except Exception as e:  # noqa: BLE001
                last_error = e
                time.sleep(0.5)
        raise RuntimeError(f"BNO08x init failed after {max_attempts} attempts: {last_error}")

    def yaw(self):
        """Heading about the vertical axis, degrees in (-180, 180], or None.

        None signals a read that couldn't be turned into a heading (no sample /
        transient error); callers should skip the sample, not treat it as 0.
        """
        if not self.available:
            return None
        try:
            quat = self._bno.quaternion
            x, y, z, w = quat[0], quat[1], quat[2], quat[3]
        except (KeyError, TypeError, ValueError, RuntimeError, OSError):
            # KeyError / OSError: garbled I2C traffic on the Pi hardware bus.
            # Skip the sample and let the turn loop keep polling.
            return None
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))
