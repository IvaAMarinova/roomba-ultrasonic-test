"""
UVC camera capture for one-shot stills (teleop snapshots, arena mapping).

Uses ffmpeg against a V4L2 device — the same stack verified on the rover Pi
(LG AN-VC500 at /dev/video0). Off-Pi or without ffmpeg the driver stays disabled
and capture() returns None.
"""

import os
import shutil
import subprocess

import config as default_config


class Camera:
    def __init__(self, logger, cfg=None, dry_run=None):
        self.cfg = cfg or default_config
        self.available = False
        if dry_run:
            logger.log("camera", status="absent", reason="dry_run")
            return
        if not os.path.exists(self.cfg.CAMERA_DEVICE):
            logger.log(
                "camera",
                status="absent",
                reason="no_device",
                device=self.cfg.CAMERA_DEVICE,
            )
            return
        if shutil.which("ffmpeg") is None:
            logger.log("camera", status="absent", reason="ffmpeg_missing")
            return
        self.available = True
        logger.log("camera", status="ready", device=self.cfg.CAMERA_DEVICE)

    def capture(self, path: str) -> str | None:
        """Grab one JPEG frame. Returns `path` on success, else None."""
        if not self.available:
            return None
        attempts = [
            (self.cfg.CAMERA_INPUT_FORMAT, self.cfg.CAMERA_WIDTH, self.cfg.CAMERA_HEIGHT),
            ("mjpeg", 640, 480),
        ]
        for fmt, width, height in attempts:
            if self._try_capture(path, fmt, width, height):
                return path
        return None

    def _try_capture(self, path: str, fmt: str, width: int, height: int) -> bool:
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "v4l2",
            "-input_format",
            fmt,
            "-video_size",
            f"{width}x{height}",
            "-i",
            self.cfg.CAMERA_DEVICE,
            "-frames:v",
            "1",
            "-y",
            path,
        ]
        try:
            subprocess.run(cmd, check=True, timeout=10, capture_output=True)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
            return False
        return os.path.isfile(path) and os.path.getsize(path) > 0
