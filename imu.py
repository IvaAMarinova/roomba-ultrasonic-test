"""
BNO086 IMU wrapper, used for closed-loop (heading-feedback) turns.

Exposes a single useful reading: yaw() -- the car's heading about the vertical
axis, in degrees. The navigation code accumulates yaw change during a spin so it
can rotate by a measured angle instead of for a hardcoded number of seconds.

Mirrors motors.py's philosophy: if the IMU stack or hardware isn't present this
runs in a disabled mode (``available == False``) and yaw() returns None, so the
caller transparently falls back to the original timed turn.

Driver note: adafruit_bno08x 1.3.3 chokes on the BNO086's post-reset
command-channel packets during enable_feature() ("Unprocessable Batch bytes").
The driver's own _wait_for_packet_type() already skips those channels; we apply
the same skip to _handle_packet() instead of editing the installed library.
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
    from adafruit_bno08x.i2c import BNO08X_I2C

    _HAS_IMU_LIBS = True
except Exception:  # noqa: BLE001 - board/busio raise NotImplementedError off-Pi
    _HAS_IMU_LIBS = False


if _HAS_IMU_LIBS:
    _orig_handle_packet = BNO08X._handle_packet

    def _handle_packet_skip_command(self, packet):
        if packet.channel_number in (BNO_CHANNEL_SHTP_COMMAND, BNO_CHANNEL_EXE):
            return
        return _orig_handle_packet(self, packet)

    BNO08X._handle_packet = _handle_packet_skip_command


class IMU:
    """Heading source backed by a BNO086 over I2C (pins 3/5)."""

    def __init__(self, cfg=None, dry_run=None):
        self.available = False
        self._bno = None
        if dry_run or (dry_run is None and not _HAS_IMU_LIBS):
            print("[imu] no IMU stack present -> timed turns will be used")
            return
        try:
            self._init_sensor()
            self.available = True
            print("[imu] BNO086 ready -> closed-loop turns enabled")
        except Exception as e:  # noqa: BLE001 - sensor raises bare RuntimeError
            print(f"[imu] init failed ({e}); falling back to timed turns")
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
        except (TypeError, ValueError, RuntimeError, OSError):
            # OSError covers the transient I2C TimeoutError ([Errno 110]) the
            # BNO086 throws under clock stretching -- skip the sample, don't crash.
            return None
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))
