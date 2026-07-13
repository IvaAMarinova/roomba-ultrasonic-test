#!/usr/bin/env python3
"""
Manual teleoperation controller for the lunar rover.

Opens a tkinter window (works over X11 forwarding) and lets you
drive the rover with WASD / arrow keys or on-screen buttons.

In hardware mode the rear dump door servo is held at the closed pulse via
pigpiod for the whole session so it does not buzz or drift while driving.

Usage:
    python teleop.py                 # mock mode (prints to terminal)
    python teleop.py --hardware      # drives real motors via GPIO

Structured JSON lines go to stdout and teleop_recordings/<session>.jsonl.
Each line has monotonic t (seconds), pose (x/y/heading), all ultrasonic
readings, IMU yaw, and derived wall summaries. Space or --snapshot-interval
captures camera JPEGs keyed by snapshot_id.
"""

import argparse
import math
import os
import threading
import time

# Match actuators.py: servos need pigpiod; set before any gpiozero import.
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "pigpio")

import sys
import tkinter as tk


def _format_distance(dist, enabled: bool = True) -> str:
    if not enabled:
        return "off"
    if dist is None or dist == float("inf"):
        return "--"
    return f"{dist:.1f} cm"


def _navigation_dir() -> str:
    """Directory containing actuators.py / config.py."""
    here = os.path.dirname(os.path.abspath(__file__))
    nested = os.path.join(here, "roomba-ultrasonic-test")
    if os.path.isfile(os.path.join(nested, "actuators.py")):
        return nested
    if os.path.isfile(os.path.join(here, "actuators.py")):
        return here
    return nested


def _create_mock_robot():
    class _MockRobot:
        def forward(self, speed: float) -> None:
            print(f"[MOCK] forward  speed={speed:.2f}")

        def backward(self, speed: float) -> None:
            print(f"[MOCK] backward speed={speed:.2f}")

        def left(self, speed: float) -> None:
            print(f"[MOCK] left     speed={speed:.2f}")

        def right(self, speed: float) -> None:
            print(f"[MOCK] right    speed={speed:.2f}")

        def stop(self) -> None:
            print("[MOCK] stop")

    return _MockRobot()


class _DriveRobot:
    """Optional direction inversion between UI labels and gpiozero Robot."""

    def __init__(self, inner, *, invert_forward: bool = False, invert_turn: bool = False) -> None:
        self._inner = inner
        self._invert_forward = invert_forward
        self._invert_turn = invert_turn

    def forward(self, speed: float) -> None:
        fn = self._inner.backward if self._invert_forward else self._inner.forward
        fn(speed)

    def backward(self, speed: float) -> None:
        fn = self._inner.forward if self._invert_forward else self._inner.backward
        fn(speed)

    def left(self, speed: float) -> None:
        fn = self._inner.right if self._invert_turn else self._inner.left
        fn(speed)

    def right(self, speed: float) -> None:
        fn = self._inner.left if self._invert_turn else self._inner.right
        fn(speed)

    def stop(self) -> None:
        self._inner.stop()


def _create_hardware_robot(cfg):
    from gpiozero import Robot, PhaseEnableMotor

    left = cfg.MOTORS["left"]
    right = cfg.MOTORS["right"]
    inner = Robot(
        left=PhaseEnableMotor(left["dir"], left["pwm"]),
        right=PhaseEnableMotor(right["dir"], right["pwm"]),
    )
    # gpiozero forward/back sense is opposite to driving intent on this chassis.
    return _DriveRobot(inner, invert_forward=True)


def _run_async(target, *args, **kwargs) -> None:
    threading.Thread(target=target, args=args, kwargs=kwargs, daemon=True).start()


class _ArenaRecorder:
    """Dead-reckoning pose + monotonic session stamps for arena mapping logs."""

    def __init__(self, cfg, session_id: str) -> None:
        from navigation import angle_diff

        self._angle_diff = angle_diff
        self.cfg = cfg
        self.session_id = session_id
        self.t0 = time.monotonic()
        self.tick = 0
        self.x = cfg.START_X_CM
        self.y = cfg.START_Y_CM
        self.heading = 0.0
        self._yaw0: float | None = None
        self._last_t = self.t0

    def set_origin(self, yaw_abs: float | None) -> None:
        if yaw_abs is not None:
            self._yaw0 = yaw_abs
            self.heading = 0.0

    def stamp(self) -> dict:
        return {
            "session": self.session_id,
            "t": round(time.monotonic() - self.t0, 4),
        }

    def next_tick(self) -> dict:
        self.tick += 1
        return {**self.stamp(), "tick": self.tick}

    def update_motion(self, direction: str | None, speed: float, yaw_abs: float | None) -> float:
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        max_dt = 2.0 / self.cfg.CONTROL_LOOP_HZ if self.cfg.CONTROL_LOOP_HZ > 0 else 0.1
        if dt > max_dt:
            dt = max_dt

        if yaw_abs is not None and self._yaw0 is not None:
            self.heading = self._angle_diff(yaw_abs, self._yaw0)

        if direction in ("forward", "backward") and speed > 0 and dt > 0:
            sign = 1.0 if direction == "forward" else -1.0
            dist = (
                self.cfg.DRIVE_CM_PER_S
                * (speed / self.cfg.DRIVE_SPEED)
                * dt
                * sign
            )
            rad = math.radians(self.heading)
            self.x += dist * math.sin(rad)
            self.y += dist * math.cos(rad)
        return dt

    def pose_fields(self, yaw_abs: float | None = None) -> dict:
        fields = {"x": round(self.x, 2), "y": round(self.y, 2), "heading": round(self.heading, 2)}
        if yaw_abs is not None:
            fields["yaw"] = yaw_abs
        return fields


def _session_metadata(cfg, session_id: str, **extra) -> dict:
    return {
        "session": session_id,
        "arena_width_cm": cfg.ARENA_WIDTH_CM,
        "arena_length_cm": cfg.ARENA_LENGTH_CM,
        "pit_x_cm": cfg.PIT_X_CM,
        "pit_y_cm": cfg.PIT_Y_CM,
        "start_x_cm": cfg.START_X_CM,
        "start_y_cm": cfg.START_Y_CM,
        "robot_width_cm": cfg.ROBOT_WIDTH_CM,
        "lane_width_cm": cfg.LANE_WIDTH_CM,
        "drive_cm_per_s": cfg.DRIVE_CM_PER_S,
        "drive_speed": cfg.DRIVE_SPEED,
        "front_sensors": list(cfg.FRONT_SENSORS),
        "back_sensors": list(cfg.BACK_SENSORS),
        "sensors": {
            name: {"trig": spec["trig"], "echo": spec["echo"], "enabled": spec.get("enabled", True)}
            for name, spec in cfg.SENSORS.items()
        },
        **extra,
    }


def _front_wall_summary(cfg, readings: dict[str, float]) -> dict:
    dists = []
    for name in cfg.FRONT_SENSORS:
        spec = cfg.SENSORS.get(name, {})
        if not spec.get("enabled", True):
            continue
        dist = readings.get(name)
        if dist is not None and math.isfinite(dist):
            dists.append(dist)
    if not dists:
        return {"front_wall_cm": None, "front_agree": 0}
    median = sorted(dists)[len(dists) // 2]
    tol = cfg.FRONT_AGREE_TOL_CM
    agree = sum(1 for d in dists if abs(d - median) <= tol)
    return {"front_wall_cm": round(median, 2), "front_agree": agree}


def _back_wall_summary(cfg, readings: dict[str, float]) -> dict:
    dists = []
    for name in cfg.BACK_SENSORS:
        spec = cfg.SENSORS.get(name, {})
        if not spec.get("enabled", True):
            continue
        dist = readings.get(name)
        if dist is not None and math.isfinite(dist):
            dists.append(dist)
    if not dists:
        return {"back_wall_cm": None, "back_agree": 0}
    median = sorted(dists)[len(dists) // 2]
    tol = cfg.FRONT_AGREE_TOL_CM
    agree = sum(1 for d in dists if abs(d - median) <= tol)
    return {"back_wall_cm": round(median, 2), "back_agree": agree}


class TeleopApp:
    KEYS_FORWARD = {"w", "Up"}
    KEYS_BACKWARD = {"s", "Down"}
    KEYS_LEFT = {"a", "Left"}
    KEYS_RIGHT = {"d", "Right"}

    def __init__(
        self,
        robot,
        *,
        cfg,
        logger,
        recorder: _ArenaRecorder,
        front_servo=None,
        disposer=None,
        sensors=None,
        imu=None,
        camera=None,
        record_dir: str = "teleop_recordings",
        tick_ms: int = 50,
        snapshot_interval_s: float = 0.0,
    ) -> None:
        self._cfg = cfg
        self._robot = robot
        self._logger = logger
        self._recorder = recorder
        self._front_servo = front_servo
        self._disposer = disposer
        self._sensors = sensors
        self._imu = imu
        self._camera = camera
        self._record_dir = record_dir
        self._tick_ms = tick_ms
        self._snapshot_interval_s = snapshot_interval_s
        self._last_auto_snapshot_t = 0.0
        self._speed: float = 0.5
        self._direction: str | None = None
        self._held: set[str] = set()
        self._servo_busy = False

        self._root = tk.Tk()
        self._root.title("ROOMBA Teleop")
        self._root.configure(bg="#1e1e2e")
        self._root.resizable(False, False)

        self._build_ui()

        self._root.bind("<KeyPress>", self._on_key_press)
        self._root.bind("<KeyRelease>", self._on_key_release)
        self._root.protocol("WM_DELETE_WINDOW", self._quit)
        self._root.after(self._tick_ms, self._record_tick)

    def _log(self, event: str, **fields) -> None:
        fields.update(self._recorder.stamp())
        yaw = self._imu.yaw() if self._imu is not None else None
        fields.update(self._recorder.pose_fields(yaw))
        self._logger.log(event, **fields)

    def _sensor_fields(self) -> dict:
        if self._sensors is None:
            return {}
        readings = self._sensors.read_all()
        fields = dict(readings)
        fields.update(_front_wall_summary(self._cfg, readings))
        fields.update(_back_wall_summary(self._cfg, readings))
        return fields

    def _build_ui(self) -> None:
        style = {"bg": "#1e1e2e", "fg": "#cdd6f4"}
        btn_style = {
            "bg": "#313244",
            "fg": "#cdd6f4",
            "activebackground": "#45475a",
            "activeforeground": "#cdd6f4",
            "relief": "flat",
            "font": ("monospace", 14, "bold"),
            "width": 4,
            "height": 2,
        }

        title = tk.Label(
            self._root,
            text="ROOMBA Teleop",
            font=("monospace", 16, "bold"),
            **style,
        )
        title.pack(pady=(12, 4))

        hint = tk.Label(
            self._root,
            text=(
                "WASD / arrows = drive   |   Space = stop + snapshot   |   Q = quit\n"
                "U/J = front up/down   K = scoop cycle   "
                "O/C = door open/close   X = dispose"
            ),
            font=("monospace", 9),
            justify="center",
            **style,
        )
        hint.pack(pady=(0, 8))

        button_frame = tk.Frame(self._root, bg="#1e1e2e")
        button_frame.pack(pady=4)

        self._btn_fwd = tk.Button(button_frame, text="W\n▲", **btn_style)
        self._btn_fwd.grid(row=0, column=1, padx=4, pady=4)

        self._btn_left = tk.Button(button_frame, text="A\n◀", **btn_style)
        self._btn_left.grid(row=1, column=0, padx=4, pady=4)

        self._btn_stop = tk.Button(
            button_frame,
            text="STOP",
            bg="#f38ba8",
            fg="#1e1e2e",
            activebackground="#eba0ac",
            activeforeground="#1e1e2e",
            relief="flat",
            font=("monospace", 11, "bold"),
            width=4,
            height=2,
        )
        self._btn_stop.grid(row=1, column=1, padx=4, pady=4)

        self._btn_right = tk.Button(button_frame, text="D\n▶", **btn_style)
        self._btn_right.grid(row=1, column=2, padx=4, pady=4)

        self._btn_bwd = tk.Button(button_frame, text="S\n▼", **btn_style)
        self._btn_bwd.grid(row=2, column=1, padx=4, pady=4)

        for btn, press, release in [
            (self._btn_fwd, lambda: self._start("forward"), self._stop_motors),
            (self._btn_bwd, lambda: self._start("backward"), self._stop_motors),
            (self._btn_left, lambda: self._start("left"), self._stop_motors),
            (self._btn_right, lambda: self._start("right"), self._stop_motors),
            (self._btn_stop, self._stop_motors, None),
        ]:
            btn.bind("<ButtonPress-1>", lambda e, fn=press: fn())
            if release:
                btn.bind("<ButtonRelease-1>", lambda e, fn=release: fn())

        speed_frame = tk.Frame(self._root, bg="#1e1e2e")
        speed_frame.pack(pady=(8, 4))

        tk.Label(
            speed_frame, text="Speed:", font=("monospace", 11), **style
        ).pack(side="left", padx=(8, 4))

        self._speed_var = tk.DoubleVar(value=self._speed)
        speed_scale = tk.Scale(
            speed_frame,
            from_=0.1,
            to=1.0,
            resolution=0.05,
            orient="horizontal",
            variable=self._speed_var,
            command=self._on_speed_change,
            length=200,
            bg="#1e1e2e",
            fg="#cdd6f4",
            troughcolor="#313244",
            highlightthickness=0,
            font=("monospace", 10),
        )
        speed_scale.pack(side="left", padx=4)

        servo_frame = tk.Frame(self._root, bg="#1e1e2e")
        servo_frame.pack(pady=(4, 4))

        servo_btn = {
            "bg": "#45475a",
            "fg": "#cdd6f4",
            "activebackground": "#585b70",
            "activeforeground": "#cdd6f4",
            "relief": "flat",
            "font": ("monospace", 9, "bold"),
            "width": 8,
        }

        tk.Button(
            servo_frame, text="Scoop UP", command=self._front_raise, **servo_btn
        ).grid(row=0, column=0, padx=3, pady=2)
        tk.Button(
            servo_frame, text="Scoop DN", command=self._front_lower, **servo_btn
        ).grid(row=0, column=1, padx=3, pady=2)
        tk.Button(
            servo_frame, text="Scoop cycle", command=self._front_cycle, **servo_btn
        ).grid(row=0, column=2, padx=3, pady=2)
        tk.Button(
            servo_frame, text="Door OPEN", command=self._back_open, **servo_btn
        ).grid(row=1, column=0, padx=3, pady=2)
        tk.Button(
            servo_frame, text="Door CLOSE", command=self._back_close, **servo_btn
        ).grid(row=1, column=1, padx=3, pady=2)
        tk.Button(
            servo_frame,
            text="DISPOSE",
            command=self._dispose,
            bg="#fab387",
            fg="#1e1e2e",
            activebackground="#f9c89b",
            activeforeground="#1e1e2e",
            relief="flat",
            font=("monospace", 9, "bold"),
            width=8,
        ).grid(row=1, column=2, padx=3, pady=2)

        self._status_var = tk.StringVar(value="STOPPED")
        status = tk.Label(
            self._root,
            textvariable=self._status_var,
            font=("monospace", 12, "bold"),
            bg="#1e1e2e",
            fg="#a6e3a1",
        )
        status.pack(pady=(4, 4))

        self._servo_status_var = tk.StringVar(value="servos: ready")
        servo_status = tk.Label(
            self._root,
            textvariable=self._servo_status_var,
            font=("monospace", 10),
            bg="#1e1e2e",
            fg="#89b4fa",
        )
        servo_status.pack(pady=(0, 12))

    def _set_servo_status(self, text: str) -> None:
        self._servo_status_var.set(text)

    def _run_servo_action(self, label: str, action: str, fn) -> None:
        if self._servo_busy:
            self._set_servo_status(f"servos: busy ({label} ignored)")
            return

        def worker() -> None:
            self._servo_busy = True
            self._root.after(0, lambda: self._set_servo_status(f"servos: {label}..."))
            try:
                fn()
                yaw = self._imu.yaw() if self._imu is not None else None
                self._logger.log(
                    "teleop",
                    action=action,
                    **self._recorder.stamp(),
                    **self._recorder.pose_fields(yaw),
                )
            finally:
                self._servo_busy = False
                self._root.after(0, lambda: self._set_servo_status("servos: ready"))

        _run_async(worker)

    def _front_raise(self) -> None:
        if self._front_servo is None:
            return
        self._run_servo_action("scoop up", "front_up", self._front_servo.raise_up)

    def _front_lower(self) -> None:
        if self._front_servo is None:
            return
        self._run_servo_action("scoop down", "front_down", self._front_servo.lower)

    def _front_cycle(self) -> None:
        if self._front_servo is None:
            return

        def cycle() -> None:
            # Same as run_navigation: stop driving before the blocking scoop move.
            self._held.clear()
            self._robot.stop()
            self._direction = None
            self._root.after(0, lambda: self._status_var.set("STOPPED"))
            self._log("teleop", action="stop")
            self._front_servo.lift_cycle()

        self._run_servo_action("scoop cycle", "front_cycle", cycle)

    def _back_open(self) -> None:
        if self._disposer is None:
            return
        self._run_servo_action(
            "door open",
            "back_open",
            lambda: self._disposer.join_opening(self._disposer.start_opening()),
        )

    def _back_close(self) -> None:
        if self._disposer is None:
            return
        self._run_servo_action(
            "door close",
            "back_close",
            self._disposer._door.close_door,
        )

    def _dispose(self) -> None:
        if self._disposer is None:
            return
        self._run_servo_action("disposing", "dispose", self._disposer.dump_cycle)

    def _direction_for_key(self, key: str) -> str | None:
        if key in self.KEYS_FORWARD:
            return "forward"
        if key in self.KEYS_BACKWARD:
            return "backward"
        if key in self.KEYS_LEFT:
            return "left"
        if key in self.KEYS_RIGHT:
            return "right"
        return None

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym
        if key == "space":
            self._held.clear()
            self._stop_motors()
            self._capture_snapshot()
            return
        if key == "q":
            self._quit()
            return
        if key == "u":
            self._front_raise()
            return
        if key == "j":
            self._front_lower()
            return
        if key == "k":
            self._front_cycle()
            return
        if key == "o":
            self._back_open()
            return
        if key == "c":
            self._back_close()
            return
        if key == "x":
            self._dispose()
            return
        direction = self._direction_for_key(key)
        if direction and key not in self._held:
            self._held.add(key)
            self._start(direction)

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym
        self._held.discard(key)
        if not self._held:
            self._stop_motors()
        else:
            remaining = next(iter(self._held))
            direction = self._direction_for_key(remaining)
            if direction:
                self._start(direction)

    def _start(self, direction: str) -> None:
        fn = getattr(self._robot, direction)
        fn(self._speed)
        self._direction = direction
        label = direction.upper()
        self._status_var.set(f"{label}  ({self._speed:.2f})")
        self._log("teleop", action=direction, speed=self._speed)

    def _stop_motors(self) -> None:
        self._robot.stop()
        self._direction = None
        self._status_var.set("STOPPED")
        self._log("teleop", action="stop")

    def _on_speed_change(self, _value: str) -> None:
        self._speed = self._speed_var.get()
        if self._direction is not None:
            getattr(self._robot, self._direction)(self._speed)
            self._status_var.set(f"{self._direction.upper()}  ({self._speed:.2f})")
            self._log("teleop", action=self._direction, speed=self._speed)

    def _record_tick(self) -> None:
        if not self._root.winfo_exists():
            return
        yaw = self._imu.yaw() if self._imu is not None else None
        dt = self._recorder.update_motion(self._direction, self._speed, yaw)
        fields: dict = {
            **self._recorder.next_tick(),
            "dt": round(dt, 4),
            "direction": self._direction or "stop",
            "speed": self._speed,
            **self._recorder.pose_fields(yaw),
        }
        fields.update(self._sensor_fields())
        if self._front_servo is not None:
            fields["front_servo_pulse_ms"] = round(self._front_servo.pulse_ms, 3)
        if self._disposer is not None:
            fields["back_servo_pulse_ms"] = round(self._disposer._door.pulse_ms, 3)
        self._logger.log("teleop_tick", **fields)

        if self._snapshot_interval_s > 0:
            elapsed = self._recorder.stamp()["t"]
            if elapsed - self._last_auto_snapshot_t >= self._snapshot_interval_s:
                self._last_auto_snapshot_t = elapsed
                _run_async(lambda: self._capture_snapshot(trigger="interval", quiet=True))

        self._root.after(self._tick_ms, self._record_tick)

    def _capture_snapshot(self, *, trigger: str = "manual", quiet: bool = False) -> None:
        snapshot_id = time.strftime("%Y%m%d-%H%M%S")
        yaw = self._imu.yaw() if self._imu is not None else None
        fields: dict = {
            **self._recorder.stamp(),
            "snapshot_id": snapshot_id,
            "trigger": trigger,
            "direction": self._direction or "stop",
            "speed": self._speed,
            **self._recorder.pose_fields(yaw),
        }
        fields.update(self._sensor_fields())

        image_path = os.path.join(self._record_dir, f"{snapshot_id}.jpg")
        if self._camera is not None and self._camera.available:
            captured = self._camera.capture(image_path)
            if captured:
                fields["image"] = captured
            else:
                fields["image_error"] = "capture_failed"
                image_path = None
        else:
            image_path = None

        self._logger.log("teleop_snapshot", **fields)

        if quiet:
            return

        readings = self._sensors.read_all() if self._sensors is not None else {}
        print(f"\n=== snapshot {snapshot_id} ({trigger}) ===")
        print(f"           pos: ({fields.get('x')}, {fields.get('y')}) cm  "
              f"hdg={fields.get('heading')}°  t={fields.get('t')}s")
        if self._sensors is not None:
            for name in sorted(readings):
                enabled = self._sensors.is_enabled(name)
                print(f"  {name:>14}: {_format_distance(readings[name], enabled)}")
        else:
            print("        sensors: (disabled)")

        if self._imu is not None:
            if yaw is None:
                print("           yaw: --")
            else:
                print(f"           yaw: {yaw:.1f}°")
        else:
            print("           imu: (disabled)")

        if self._camera is None:
            print("        camera: (disabled)")
        elif image_path:
            print(f"         image: {image_path}")
        elif self._camera.available:
            print("         image: capture failed")
        else:
            print("         image: (no camera)")
        print(flush=True)

    def _quit(self) -> None:
        self._robot.stop()
        self._log("teleop", phase="end")
        if self._front_servo is not None:
            self._front_servo.cleanup()
        if self._disposer is not None:
            self._disposer._door.hold_closed()
            self._disposer.cleanup()
        if self._sensors is not None:
            self._sensors.cleanup()
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual teleop with structured recording.")
    parser.add_argument("--hardware", action="store_true", help="Drive real motors via GPIO")
    parser.add_argument(
        "--log-format",
        choices=("json", "pretty"),
        default="json",
        help="Log format (default: json for capture / Kibana)",
    )
    parser.add_argument(
        "--no-sensors",
        action="store_true",
        help="Skip ultrasonic + IMU polling (drive commands only)",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Skip UVC camera capture on Space snapshots",
    )
    parser.add_argument(
        "--record-dir",
        default="teleop_recordings",
        help="Directory for session JSONL, camera JPEGs (default: teleop_recordings)",
    )
    parser.add_argument(
        "--snapshot-interval",
        type=float,
        default=30.0,
        metavar="SEC",
        help="Auto snapshot+camera every SEC seconds (0=Space only, default: 30)",
    )
    args = parser.parse_args()

    nav_dir = _navigation_dir()
    if nav_dir not in sys.path:
        sys.path.insert(0, nav_dir)
    import config
    from log import Logger

    session_id = time.strftime("%Y%m%d-%H%M%S")
    record_dir = os.path.abspath(args.record_dir)
    os.makedirs(record_dir, exist_ok=True)
    log_path = os.path.join(record_dir, f"{session_id}.jsonl")
    with open(log_path, "w", encoding="utf-8") as log_file:
        logger = Logger(format=args.log_format, log_file=log_file)
        tick_ms = max(1, int(1000.0 / config.CONTROL_LOOP_HZ))
        recorder = _ArenaRecorder(config, session_id)

        sensors = None
        imu = None
        if not args.no_sensors:
            from sensors import UltrasonicArray
            from imu import IMU

            sensors = UltrasonicArray(config)
            imu = IMU(logger, config)
            recorder.set_origin(imu.yaw() if imu is not None else None)

        camera = None
        if not args.no_camera:
            from camera import Camera

            camera = Camera(logger, config)

        from actuators import Disposer, FrontServo

        front_servo = FrontServo(config)
        disposer = Disposer(config)
        if args.hardware:
            print(
                f"[back-servo] holding closed at {config.BACK_SERVO_CLOSED_PULSE_MS:.3f} ms "
                f"(GPIO {config.BACK_SERVO_PIN})"
            )
            front_servo.startup()

        if args.hardware:
            print("Starting teleop in HARDWARE mode")
            robot = _create_hardware_robot(config)
        else:
            print("Starting teleop in MOCK mode (use --hardware for real motors)")
            robot = _create_mock_robot()

        logger.log(
            "teleop_session",
            **_session_metadata(
                config,
                session_id,
                log_file=log_path,
                record_dir=record_dir,
                hardware=args.hardware,
                sensors=sensors is not None and sensors.using_hardware,
                imu=imu is not None and imu.available,
                camera=camera is not None and camera.available,
                snapshot_interval_s=args.snapshot_interval,
                control_loop_hz=config.CONTROL_LOOP_HZ,
            ),
        )
        logger.log(
            "teleop",
            phase="start",
            session=session_id,
            hardware=args.hardware,
            sensors=sensors is not None and sensors.using_hardware,
            imu=imu is not None and imu.available,
            camera=camera is not None and camera.available,
            record_dir=record_dir,
            log_file=log_path,
        )
        print(f"Session {session_id} -> {log_path}")

        app = TeleopApp(
            robot,
            cfg=config,
            logger=logger,
            recorder=recorder,
            front_servo=front_servo,
            disposer=disposer,
            sensors=sensors,
            imu=imu,
            camera=camera,
            record_dir=record_dir,
            tick_ms=tick_ms,
            snapshot_interval_s=max(0.0, args.snapshot_interval),
        )
        app.run()


if __name__ == "__main__":
    main()
