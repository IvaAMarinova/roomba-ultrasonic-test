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

Structured JSON lines (event teleop / teleop_tick / teleop_snapshot) go to stdout.
Press Space to stop and capture a snapshot: sensor distances, IMU yaw, and a
camera JPEG saved under teleop_recordings/ (or --record-dir).
"""

import argparse
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


class TeleopApp:
    KEYS_FORWARD = {"w", "Up"}
    KEYS_BACKWARD = {"s", "Down"}
    KEYS_LEFT = {"a", "Left"}
    KEYS_RIGHT = {"d", "Right"}

    def __init__(
        self,
        robot,
        *,
        logger,
        front_servo=None,
        back_servo=None,
        dispose_hold_s: float = 2.0,
        sensors=None,
        imu=None,
        camera=None,
        record_dir: str = "teleop_recordings",
        tick_ms: int = 50,
    ) -> None:
        self._robot = robot
        self._logger = logger
        self._front_servo = front_servo
        self._back_servo = back_servo
        self._dispose_hold_s = dispose_hold_s
        self._sensors = sensors
        self._imu = imu
        self._camera = camera
        self._record_dir = record_dir
        self._tick_ms = tick_ms
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
                self._logger.log("teleop", action=action)
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
        self._run_servo_action("scoop cycle", "front_cycle", self._front_servo.lift_cycle)

    def _back_open(self) -> None:
        if self._back_servo is None:
            return
        self._run_servo_action("door open", "back_open", self._back_servo.open_door)

    def _back_close(self) -> None:
        if self._back_servo is None:
            return
        self._run_servo_action("door close", "back_close", self._back_servo.close_door)

    def _dispose(self) -> None:
        if self._back_servo is None:
            return

        def cycle() -> None:
            self._back_servo.open_door()
            time.sleep(self._dispose_hold_s)
            self._back_servo.close_door()

        self._run_servo_action("disposing", "dispose", cycle)

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
        self._logger.log("teleop", action=direction, speed=self._speed)

    def _stop_motors(self) -> None:
        self._robot.stop()
        self._direction = None
        self._status_var.set("STOPPED")
        self._logger.log("teleop", action="stop")

    def _on_speed_change(self, _value: str) -> None:
        self._speed = self._speed_var.get()
        if self._direction is not None:
            getattr(self._robot, self._direction)(self._speed)
            self._status_var.set(f"{self._direction.upper()}  ({self._speed:.2f})")
            self._logger.log("teleop", action=self._direction, speed=self._speed)

    def _record_tick(self) -> None:
        if not self._root.winfo_exists():
            return
        fields: dict = {
            "direction": self._direction or "stop",
            "speed": self._speed,
        }
        if self._imu is not None:
            fields["yaw"] = self._imu.yaw()
        if self._sensors is not None:
            fields.update(self._sensors.read_all())
        self._logger.log("teleop_tick", **fields)
        self._root.after(self._tick_ms, self._record_tick)

    def _capture_snapshot(self) -> None:
        snapshot_id = time.strftime("%Y%m%d-%H%M%S")
        fields: dict = {"snapshot_id": snapshot_id}

        readings: dict[str, float] = {}
        if self._sensors is not None:
            readings = self._sensors.read_all()
            fields.update(readings)

        yaw = None
        if self._imu is not None:
            yaw = self._imu.yaw()
            fields["yaw"] = yaw

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

        print(f"\n=== snapshot {snapshot_id} ===")
        if self._sensors is not None:
            for name in sorted(readings):
                enabled = self._sensors.is_enabled(name)
                print(f"  {name:>14}: {_format_distance(readings[name], enabled)}")
        elif not readings:
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
        self._logger.log("teleop", phase="end")
        if self._front_servo is not None:
            self._front_servo.cleanup()
        if self._back_servo is not None:
            self._back_servo.hold_closed()
            self._back_servo.cleanup()
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
        help="Directory for Space-triggered camera JPEGs (default: teleop_recordings)",
    )
    args = parser.parse_args()

    nav_dir = _navigation_dir()
    if nav_dir not in sys.path:
        sys.path.insert(0, nav_dir)
    import config
    from log import Logger

    logger = Logger(format=args.log_format)
    tick_ms = max(1, int(1000.0 / config.CONTROL_LOOP_HZ))
    record_dir = os.path.abspath(args.record_dir)
    os.makedirs(record_dir, exist_ok=True)

    sensors = None
    imu = None
    if not args.no_sensors:
        from sensors import UltrasonicArray
        from imu import IMU

        sensors = UltrasonicArray(config)
        imu = IMU(logger, config)

    camera = None
    if not args.no_camera:
        from camera import Camera

        camera = Camera(logger, config)

    from actuators import BackServo, FrontServo

    front_servo = FrontServo(config)
    back_servo = BackServo(config)
    if args.hardware:
        print(
            f"[back-servo] holding closed at {config.BACK_SERVO_CLOSED_PULSE_MS:.3f} ms "
            f"(GPIO {config.BACK_SERVO_PIN})"
        )

    if args.hardware:
        print("Starting teleop in HARDWARE mode")
        robot = _create_hardware_robot(config)
    else:
        print("Starting teleop in MOCK mode (use --hardware for real motors)")
        robot = _create_mock_robot()

    logger.log(
        "teleop",
        phase="start",
        hardware=args.hardware,
        sensors=sensors is not None and sensors.using_hardware,
        imu=imu is not None and imu.available,
        camera=camera is not None and camera.available,
        record_dir=record_dir,
    )

    app = TeleopApp(
        robot,
        logger=logger,
        front_servo=front_servo,
        back_servo=back_servo,
        dispose_hold_s=config.DISPOSE_HOLD_S,
        sensors=sensors,
        imu=imu,
        camera=camera,
        record_dir=record_dir,
        tick_ms=tick_ms,
    )
    app.run()


if __name__ == "__main__":
    main()
