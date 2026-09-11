"""Renders a flown mission as a 3-D animation and a static trajectory plot.

Coordinates are NED internally; everything here flips to a conventional
east-north-up view, because a plot with altitude pointing down is needlessly
hard to read.
"""

from __future__ import annotations

import numpy as np
import matplotlib
matplotlib.use("Agg")                      # headless: no window, just files
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

INK = "#12151a"
GRID = "#2a3038"
PATH = "#4db8ff"
DRONE = "#ffb020"
ACCENT = "#5ee08a"


def _style(ax) -> None:
    ax.set_facecolor(INK)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.set_pane_color((0.07, 0.08, 0.10, 1.0))
        pane._axinfo["grid"].update(color=GRID, linewidth=0.5)
    ax.tick_params(colors="#8b97a6", labelsize=8)
    for label in (ax.set_xlabel, ax.set_ylabel, ax.set_zlabel):
        pass
    ax.set_xlabel("East (m)", color="#8b97a6", fontsize=9)
    ax.set_ylabel("North (m)", color="#8b97a6", fontsize=9)
    ax.set_zlabel("Altitude (m)", color="#8b97a6", fontsize=9)


def _bounds(path: np.ndarray):
    east, north, alt = path[:, 1], path[:, 0], -path[:, 2]
    pad = 3.0
    r = max(np.ptp(east), np.ptp(north), 10.0) / 2 + pad
    cx, cy = east.mean(), north.mean()
    return (cx - r, cx + r), (cy - r, cy + r), (0.0, max(alt.max() * 1.25, 5.0))


def plot_trajectory(mission, out: str, title: str = "Flight path") -> str:
    """Static 3-D trajectory with the command boundaries marked."""
    path = mission.path()
    fig = plt.figure(figsize=(9, 6.5), facecolor=INK)
    ax = fig.add_subplot(111, projection="3d")
    _style(ax)

    east, north, alt = path[:, 1], path[:, 0], -path[:, 2]
    ax.plot(east, north, alt, color=PATH, linewidth=1.6, alpha=0.95)
    ax.plot(east, north, np.zeros_like(alt), color=PATH, linewidth=0.7,
            alpha=0.22)                                     # ground shadow

    for idx, label in mission.events:
        idx = min(idx, len(path) - 1)
        ax.scatter(path[idx, 1], path[idx, 0], -path[idx, 2],
                   s=34, color=ACCENT, depthshade=False, zorder=5)
        ax.text(path[idx, 1], path[idx, 0], -path[idx, 2] + 0.9, label,
                color=ACCENT, fontsize=8)

    ax.scatter([east[0]], [north[0]], [alt[0]], s=60, color="#ffffff",
               marker="^", depthshade=False, label="launch")
    xl, yl, zl = _bounds(path)
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_zlim(*zl)
    ax.view_init(elev=26, azim=-58)
    ax.set_title(title, color="#e6edf3", fontsize=12, pad=14)
    fig.tight_layout()
    fig.savefig(out, dpi=140, facecolor=INK)
    plt.close(fig)
    return out


def animate(mission, out: str, seconds: float = 14.0, fps: int = 30,
            title: str = "LLM-commanded flight", spin: float = 26.0) -> str:
    """Animate the flight, compressed into `seconds` of video.

    `spin` sweeps the camera azimuth by that many degrees over the clip. Set it
    to 0 for GIF output: a moving camera changes every pixel in every frame,
    which defeats the frame-delta compression a GIF relies on.
    """
    path = mission.path()
    frames = int(seconds * fps)
    # The sim runs at 200 Hz; sample it down to video rate.
    idx = np.linspace(0, len(path) - 1, frames).astype(int)

    fig = plt.figure(figsize=(9, 6.5), facecolor=INK)
    ax = fig.add_subplot(111, projection="3d")
    _style(ax)
    xl, yl, zl = _bounds(path)
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_zlim(*zl)
    ax.set_title(title, color="#e6edf3", fontsize=12, pad=14)

    trail, = ax.plot([], [], [], color=PATH, linewidth=1.7)
    shadow, = ax.plot([], [], [], color=PATH, linewidth=0.7, alpha=0.22)
    marker = ax.scatter([], [], [], s=70, color=DRONE, depthshade=False)
    hud = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                    color="#e6edf3", fontsize=10, family="monospace")

    # Which command is active at each sampled frame.
    marks = [(i, lab) for i, lab in mission.events]
    def phase_at(i: int) -> str:
        name = "takeoff"
        for at, lab in marks:
            if i >= at:
                name = lab
        return name

    def update(f: int):
        i = idx[f]
        seg = path[: i + 1]
        e, n, a = seg[:, 1], seg[:, 0], -seg[:, 2]
        trail.set_data(e, n); trail.set_3d_properties(a)
        shadow.set_data(e, n); shadow.set_3d_properties(np.zeros_like(a))
        marker._offsets3d = ([e[-1]], [n[-1]], [a[-1]])
        hud.set_text(f"{phase_at(i):8}  N {n[-1]:6.1f}  E {e[-1]:6.1f}  alt {a[-1]:5.1f} m")
        if spin:
            ax.view_init(elev=26, azim=-58 + spin * f / max(frames - 1, 1))
        return trail, shadow, marker, hud

    anim = FuncAnimation(fig, update, frames=frames, interval=1000 / fps, blit=False)
    if out.endswith(".gif"):
        anim.save(out, writer=PillowWriter(fps=fps))
    else:
        anim.save(out, writer=FFMpegWriter(fps=fps, bitrate=3600))
    plt.close(fig)
    return out
