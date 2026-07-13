#!/usr/bin/env python3
"""
Playback visualizer for teleop arena-mapping sessions (runs on a PC).

Loads teleop_recordings/<session>.jsonl plus any snapshot JPEGs in the same
directory and shows:
  - top-down arena map with trajectory, robot pose, ultrasonic rays
  - sensor distance bars at the current time
  - camera snapshot when the scrubber crosses a snapshot event

Usage:
    python visualize_teleop.py teleop_recordings/20260713-190430.jsonl
    python visualize_teleop.py --dir teleop_recordings          # newest session
    python visualize_teleop.py --dir teleop_recordings --play
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
from matplotlib.widgets import Button, Slider

try:
    import config as default_config
except ImportError:
    default_config = None

SENSOR_NAMES = (
    "front_left",
    "front_center",
    "front_right",
    "back_left",
    "back_right",
)
SENSOR_COLORS = {
    "front_left": "#89b4fa",
    "front_center": "#a6e3a1",
    "front_right": "#89dceb",
    "back_left": "#f9e2af",
    "back_right": "#fab387",
}


@dataclass
class TeleopSession:
    path: Path
    meta: dict = field(default_factory=dict)
    ticks: list[dict] = field(default_factory=list)
    snapshots: list[dict] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)

    @property
    def duration(self) -> float:
        if not self.ticks:
            return 0.0
        return float(self.ticks[-1].get("t", 0.0))

    @property
    def record_dir(self) -> Path:
        return self.path.parent


def load_session(path: Path) -> TeleopSession:
    session = TeleopSession(path=path.resolve())
    with path.open(encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: {exc}") from exc
            event = rec.get("event")
            if event == "teleop_session":
                session.meta.update(rec)
            elif event == "teleop_tick":
                session.ticks.append(rec)
            elif event == "teleop_snapshot":
                session.snapshots.append(rec)
            elif event in ("teleop",):
                session.events.append(rec)
    if not session.ticks and not session.snapshots:
        raise ValueError(f"{path}: no teleop_tick or teleop_snapshot records found")
    session.ticks.sort(key=lambda r: r.get("t", 0.0))
    session.snapshots.sort(key=lambda r: r.get("t", 0.0))
    return session


def _meta_value(session: TeleopSession, key: str, fallback):
    if key in session.meta:
        return session.meta[key]
    if default_config is not None and hasattr(default_config, key.upper()):
        return getattr(default_config, key.upper())
    return fallback


def _arena_meta(session: TeleopSession) -> dict:
    return {
        "width": float(_meta_value(session, "arena_width_cm", 150.0)),
        "length": float(_meta_value(session, "arena_length_cm", 212.0)),
        "pit_x": float(_meta_value(session, "pit_x_cm", 75.0)),
        "pit_y": float(_meta_value(session, "pit_y_cm", 0.0)),
        "robot_w": float(_meta_value(session, "robot_width_cm", 50.0)),
    }


def _heading_unit(heading_deg: float) -> tuple[float, float]:
    rad = math.radians(heading_deg)
    return math.sin(rad), math.cos(rad)


def _body_to_world(cx: float, cy: float, heading: float, bx: float, by: float) -> tuple[float, float]:
    fx, fy = _heading_unit(heading)
    rx, ry = math.cos(math.radians(heading)), -math.sin(math.radians(heading))
    return cx + bx * fx + by * rx, cy + bx * fy + by * ry


def _robot_polygon(x: float, y: float, heading: float, width: float, length: float | None = None):
    hl = (length or width) / 2.0
    hw = width / 2.0
    pts = [
        _body_to_world(x, y, heading, -hl, -hw),
        _body_to_world(x, y, heading, -hl, hw),
        _body_to_world(x, y, heading, hl, hw),
        _body_to_world(x, y, heading, hl, -hw),
        _body_to_world(x, y, heading, -hl, -hw),
    ]
    return pts


def _sensor_mounts(width: float, length: float | None = None) -> dict[str, tuple[float, float, float]]:
    """Body-frame (forward, right) positions and ray heading offset (deg)."""
    hl = (length or width) / 2.0
    hw = width / 2.0
    return {
        "front_left": (hl, -hw * 0.85, -18.0),
        "front_center": (hl, 0.0, 0.0),
        "front_right": (hl, hw * 0.85, 18.0),
        "back_left": (-hl, -hw * 0.85, 180.0 - 18.0),
        "back_right": (-hl, hw * 0.85, 180.0 + 18.0),
    }


def _tick_at_time(session: TeleopSession, t: float) -> dict | None:
    if not session.ticks:
        return None
    times = [float(r.get("t", 0.0)) for r in session.ticks]
    idx = bisect.bisect_right(times, t) - 1
    idx = max(0, min(idx, len(session.ticks) - 1))
    return session.ticks[idx]


def _snapshot_at_time(session: TeleopSession, t: float, tol: float = 0.15) -> dict | None:
    best = None
    best_dt = tol
    for snap in session.snapshots:
        dt = abs(float(snap.get("t", 0.0)) - t)
        if dt <= best_dt:
            best_dt = dt
            best = snap
    return best


def _resolve_image(session: TeleopSession, snap: dict) -> Path | None:
    image = snap.get("image")
    if not image:
        return None
    path = Path(image)
    if path.is_file():
        return path
    candidate = session.record_dir / path.name
    if candidate.is_file():
        return candidate
    sid = snap.get("snapshot_id")
    if sid:
        candidate = session.record_dir / f"{sid}.jpg"
        if candidate.is_file():
            return candidate
    return None


def _pick_session(path: Path | None, directory: Path | None) -> Path:
    if path is not None:
        if not path.is_file():
            raise SystemExit(f"not a file: {path}")
        return path
    if directory is None:
        directory = Path("teleop_recordings")
    if not directory.is_dir():
        raise SystemExit(f"not a directory: {directory}")
    candidates = sorted(directory.glob("*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        raise SystemExit(f"no .jsonl sessions in {directory}")
    return candidates[0]


class TeleopVisualizer:
    def __init__(self, session: TeleopSession, *, autoplay: bool = False) -> None:
        self.session = session
        self.arena = _arena_meta(session)
        self.mounts = _sensor_mounts(self.arena["robot_w"])
        self.playing = autoplay
        self.play_speed = 1.0

        self.fig = plt.figure(figsize=(14, 8), facecolor="#1e1e2e")
        self.fig.canvas.manager.set_window_title(f"Teleop replay — {session.path.name}")
        gs = self.fig.add_gridspec(3, 2, width_ratios=(1.4, 1.0), height_ratios=(2.2, 1.0, 0.12))

        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_sensors = self.fig.add_subplot(gs[1, 0])
        self.ax_image = self.fig.add_subplot(gs[0:2, 1])
        self.ax_slider = self.fig.add_subplot(gs[2, :])

        self._style_axes()
        self._draw_static()
        self._init_dynamic_artists()

        self.slider = Slider(
            self.ax_slider,
            "t (s)",
            0.0,
            max(session.duration, 0.05),
            valinit=0.0,
            color="#89b4fa",
        )
        self.slider.label.set_color("#cdd6f4")
        self.slider.valtext.set_color("#cdd6f4")
        self.slider.on_changed(self._on_slider)

        ax_play = self.fig.add_axes([0.86, 0.02, 0.06, 0.04])
        ax_pause = self.fig.add_axes([0.93, 0.02, 0.06, 0.04])
        self.btn_play = Button(ax_play, "Play", color="#313244", hovercolor="#45475a")
        self.btn_pause = Button(ax_pause, "Pause", color="#313244", hovercolor="#45475a")
        self.btn_play.label.set_color("#cdd6f4")
        self.btn_pause.label.set_color("#cdd6f4")
        self.btn_play.on_clicked(lambda _e: self._set_playing(True))
        self.btn_pause.on_clicked(lambda _e: self._set_playing(False))

        self._timer = self.fig.canvas.new_timer(interval=50)
        self._timer.add_callback(self._on_timer)
        if self.playing:
            self._timer.start()

        self._update(0.0)

    def _style_axes(self) -> None:
        for ax in (self.ax_map, self.ax_sensors, self.ax_image):
            ax.set_facecolor("#181825")
            ax.tick_params(colors="#a6adc8")
            for spine in ax.spines.values():
                spine.set_color("#45475a")
            ax.title.set_color("#cdd6f4")
            ax.xaxis.label.set_color("#bac2de")
            ax.yaxis.label.set_color("#bac2de")
        self.ax_slider.set_facecolor("#181825")
        self.ax_slider.tick_params(colors="#a6adc8")

    def _draw_static(self) -> None:
        w, h = self.arena["width"], self.arena["length"]
        self.ax_map.add_patch(
            mpatches.Rectangle(
                (0, 0), w, h, fill=False, edgecolor="#585b70", linewidth=2, linestyle="-"
            )
        )
        pit_r = self.arena["robot_w"] / 2.0
        self.ax_map.add_patch(
            mpatches.Circle(
                (self.arena["pit_x"], self.arena["pit_y"]),
                pit_r,
                fill=True,
                facecolor="#f38ba8",
                edgecolor="#eba0ac",
                alpha=0.35,
                linewidth=1.5,
            )
        )
        xs = [float(t.get("x", 0.0)) for t in self.session.ticks if t.get("x") is not None]
        ys = [float(t.get("y", 0.0)) for t in self.session.ticks if t.get("y") is not None]
        if xs and ys:
            self.ax_map.plot(xs, ys, color="#45475a", linewidth=1.5, alpha=0.8, zorder=1)
            self.ax_map.scatter(xs[:: max(1, len(xs) // 200)], ys[:: max(1, len(ys) // 200)],
                                s=8, color="#6c7086", alpha=0.6, zorder=1)
        snap_xs = [float(s.get("x", 0.0)) for s in self.session.snapshots if s.get("x") is not None]
        snap_ys = [float(s.get("y", 0.0)) for s in self.session.snapshots if s.get("y") is not None]
        if snap_xs:
            self.ax_map.scatter(snap_xs, snap_ys, s=40, marker="*", c="#fab387", zorder=4,
                                label="snapshots")
        margin = max(w, h) * 0.08
        self.ax_map.set_xlim(-margin, w + margin)
        self.ax_map.set_ylim(-margin, h + margin)
        self.ax_map.set_aspect("equal")
        self.ax_map.set_xlabel("x (cm)")
        self.ax_map.set_ylabel("y (cm)")
        self.ax_map.set_title("Arena map")
        if snap_xs:
            self.ax_map.legend(facecolor="#313244", edgecolor="#45475a", labelcolor="#cdd6f4")

        self.ax_sensors.set_title("Ultrasonic readings (cm)")
        self.ax_sensors.set_ylim(0, max(h, w) * 0.6)
        self.ax_sensors.set_xticks(range(len(SENSOR_NAMES)))
        self.ax_sensors.set_xticklabels([n.replace("_", "\n") for n in SENSOR_NAMES], fontsize=8)
        self.ax_image.set_title("Camera snapshot")
        self.ax_image.axis("off")

    def _init_dynamic_artists(self) -> None:
        self.robot_patch = mpatches.Polygon([[0, 0]], closed=True, facecolor="#a6e3a1",
                                            edgecolor="#cdd6f4", alpha=0.75, linewidth=1.5, zorder=3)
        self.ax_map.add_patch(self.robot_patch)
        self.heading_line, = self.ax_map.plot([], [], color="#f9e2af", linewidth=2, zorder=3)
        self.rays = LineCollection([], linewidths=2, zorder=2)
        self.ax_map.add_collection(self.rays)
        self.time_text = self.ax_map.text(0.02, 0.98, "", transform=self.ax_map.transAxes,
                                          va="top", color="#cdd6f4", fontsize=10,
                                          bbox=dict(boxstyle="round", facecolor="#313244", alpha=0.8))
        self.sensor_bars = self.ax_sensors.bar(
            range(len(SENSOR_NAMES)), [0] * len(SENSOR_NAMES),
            color=[SENSOR_COLORS[n] for n in SENSOR_NAMES], edgecolor="#1e1e2e"
        )
        self.sensor_cap = self.ax_sensors.axhline(self.arena["length"], color="#f38ba8",
                                                  linestyle="--", linewidth=1, alpha=0.5,
                                                  label="arena length")
        self.ax_sensors.legend(facecolor="#313244", edgecolor="#45475a", labelcolor="#cdd6f4",
                               fontsize=8)
        self.image_artist = None
        self.status_text = self.ax_image.text(
            0.5, 0.5, "no snapshot", transform=self.ax_image.transAxes,
            ha="center", va="center", color="#a6adc8", fontsize=11,
        )

    def _set_playing(self, playing: bool) -> None:
        self.playing = playing
        if playing:
            self._timer.start()
        else:
            self._timer.stop()

    def _on_timer(self) -> None:
        if not self.playing:
            return
        dt = 0.05 * self.play_speed
        new_t = min(self.slider.val + dt, self.session.duration)
        self.slider.set_val(new_t)
        if new_t >= self.session.duration:
            self._set_playing(False)

    def _on_slider(self, val: float) -> None:
        self._update(float(val))

    def _update(self, t: float) -> None:
        tick = _tick_at_time(self.session, t)
        if tick is None:
            return

        x = float(tick.get("x", 0.0))
        y = float(tick.get("y", 0.0))
        heading = float(tick.get("heading", 0.0))
        robot_pts = _robot_polygon(x, y, heading, self.arena["robot_w"])
        self.robot_patch.set_xy(robot_pts)

        fx, fy = _heading_unit(heading)
        head_len = self.arena["robot_w"] * 0.6
        self.heading_line.set_data([x, x + fx * head_len], [y, y + fy * head_len])

        segments = []
        colors = []
        for name in SENSOR_NAMES:
            dist = tick.get(name)
            if dist is None or not isinstance(dist, (int, float)):
                continue
            mount = self.mounts.get(name)
            if mount is None:
                continue
            bx, by, offset = mount
            sx, sy = _body_to_world(x, y, heading, bx, by)
            ray_h = heading + offset
            rx, ry = _heading_unit(ray_h)
            ex, ey = sx + rx * dist, sy + ry * dist
            segments.append([(sx, sy), (ex, ey)])
            colors.append(SENSOR_COLORS[name])
        self.rays.set_segments(segments)
        self.rays.set_color(colors)

        for bar, name in zip(self.sensor_bars, SENSOR_NAMES):
            dist = tick.get(name)
            bar.set_height(float(dist) if isinstance(dist, (int, float)) else 0.0)

        direction = tick.get("direction", "stop")
        speed = tick.get("speed", 0.0)
        front = tick.get("front_wall_cm")
        back = tick.get("back_wall_cm")
        self.time_text.set_text(
            f"t={t:.2f}s  tick={tick.get('tick', '?')}\n"
            f"pos=({x:.1f}, {y:.1f}) hdg={heading:.1f}°\n"
            f"drive={direction} @ {speed:.2f}\n"
            f"front_wall={front}  back_wall={back}"
        )

        snap = _snapshot_at_time(self.session, t)
        self._show_snapshot(snap)
        self.fig.canvas.draw_idle()

    def _show_snapshot(self, snap: dict | None) -> None:
        if self.image_artist is not None:
            self.image_artist.remove()
            self.image_artist = None
        if snap is None:
            self.status_text.set_visible(True)
            self.status_text.set_text("no snapshot at this time")
            return
        path = _resolve_image(self.session, snap)
        if path is None:
            self.status_text.set_visible(True)
            self.status_text.set_text(f"missing image\n{snap.get('snapshot_id', '?')}")
            return
        img = plt.imread(path)
        self.status_text.set_visible(False)
        self.image_artist = self.ax_image.imshow(img)
        trigger = snap.get("trigger", "?")
        sid = snap.get("snapshot_id", "?")
        self.ax_image.set_title(f"Camera — {sid} ({trigger})")

    def run(self) -> None:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize a teleop arena-mapping session.")
    parser.add_argument("session", nargs="?", type=Path, help="Path to session .jsonl file")
    parser.add_argument(
        "--dir",
        type=Path,
        default=None,
        help="Directory to search for the newest .jsonl (default: teleop_recordings)",
    )
    parser.add_argument("--play", action="store_true", help="Start playback immediately")
    args = parser.parse_args()

    try:
        path = _pick_session(args.session, args.dir)
    except SystemExit as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(1) from exc

    session = load_session(path)
    print(f"Loaded {path.name}: {len(session.ticks)} ticks, "
          f"{len(session.snapshots)} snapshots, {session.duration:.1f}s")
    TeleopVisualizer(session, autoplay=args.play).run()


if __name__ == "__main__":
    main()
