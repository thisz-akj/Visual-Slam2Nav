"""Figures for the project guide PDF (docs/figures/*.png).

    .venv/bin/python docs/make_figures.py
"""
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, Polygon
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "figures"
OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(ROOT / "tools"))

INK, QUIET, EDGE = "#1f2328", "#5b636e", "#9aa3ad"
BLUE, BLUE_T = "#2f6fdf", "#e6eefc"
GREEN, GREEN_T = "#2e8b57", "#e5f4ea"
ORANGE, ORANGE_T = "#d9822b", "#fbeedd"
RED, RED_T = "#c94a4a", "#f9e3e3"
GREY_T = "#f2f4f6"
plt.rcParams.update({"font.family": ["Noto Sans", "DejaVu Sans"], "font.size": 11, "axes.edgecolor": EDGE,
                     "axes.labelcolor": INK, "xtick.color": QUIET, "ytick.color": QUIET})


def save(fig, name):
    fig.savefig(OUT / name, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", name)


def box(ax, x, y, w, h, title, sub=None, fc=GREY_T, ec=EDGE, tsize=11.5, weight="bold"):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec=ec, lw=1.4))
    if sub:
        ax.text(x, y + 0.13 * h, title, ha="center", va="center", fontsize=tsize, weight="bold", color=INK)
        ax.text(x, y - 0.2 * h, sub, ha="center", va="center", fontsize=9, color=QUIET)
    else:
        ax.text(x, y, title, ha="center", va="center", fontsize=tsize, weight=weight, color=INK)


def arrow(ax, x0, y0, x1, y1, text=None, color=EDGE, off=(0, 0.12), style="-|>", ls="-"):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle=style, color=color, lw=1.6, ls=ls, shrinkA=0, shrinkB=0))
    if text:
        ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], text, ha="center", va="bottom",
                fontsize=8.5, color=QUIET)


def canvas(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.axis("off")
    return fig, ax


# ------------------------------------------------------------------ diagrams
def fig_pipeline():
    fig, ax = canvas(11, 5.2)
    ax.text(0.2, 4.95, "REAL WORLD  (recorded video)", fontsize=9, color=BLUE, weight="bold")
    ax.add_patch(Rectangle((0.1, 2.95), 10.8, 1.85, fc="#f7f9fd", ec="#d6e2f7", lw=1))
    ax.text(0.2, 2.62, "SIMULATION  (Gazebo)", fontsize=9, color=GREEN, weight="bold")
    ax.add_patch(Rectangle((0.1, 0.1), 10.8, 2.4, fc="#f6fbf7", ec="#d3eadb", lw=1))
    ys = 3.85
    xs = [1.25, 3.95, 6.65, 9.35]
    items = [("1. RGB-D video", "colour + depth images"), ("2. ORB-SLAM3", "where was the camera?"),
             ("3. Dense map", "3D mesh of the room"), ("4. Digital twin", "level, simplify, texture")]
    for x, (t, s) in zip(xs, items):
        box(ax, x, ys, 2.2, 1.05, t, s, fc="white", ec=BLUE)
    for a, b in zip(xs[:-1], xs[1:]):
        arrow(ax, a + 1.1, ys, b - 1.1, ys, color=BLUE)
    box(ax, 9.35, 1.55, 2.2, 1.05, "5. Gazebo world", "apartment + robot", fc=GREEN_T, ec=GREEN)
    arrow(ax, 9.35, ys - 0.53, 9.35, 1.55 + 0.53, "export", color=GREEN, off=(0.45, -0.05))
    box(ax, 5.6, 2.0, 2.9, 0.85, "6. SLAM test drive", "score vs perfect ground truth", fc="white", ec=GREEN)
    box(ax, 5.6, 0.85, 2.9, 0.85, "7. Autonomous navigation", "click a goal, robot drives", fc="white", ec=GREEN)
    arrow(ax, 8.25, 1.75, 7.05, 2.0, color=GREEN)
    arrow(ax, 8.25, 1.35, 7.05, 0.85, color=GREEN)
    box(ax, 1.9, 1.45, 3.0, 1.4, "Results", "SLAM error, tracking losses,\nroutes reached", fc=ORANGE_T, ec=ORANGE)
    arrow(ax, 4.15, 2.0, 3.4, 1.7, color=ORANGE)
    arrow(ax, 4.15, 0.85, 3.4, 1.2, color=ORANGE)
    save(fig, "pipeline.png")


def fig_pinhole():
    fig, ax = canvas(8, 3.6)
    cx, cy = 1.0, 1.8
    ax.add_patch(Circle((cx, cy), 0.09, color=INK))
    ax.text(cx, cy - 0.35, "camera\ncentre", ha="center", va="top", fontsize=9, color=QUIET)
    ax.plot([2.6, 2.6], [0.4, 3.2], color=BLUE, lw=3)
    ax.text(2.6, 3.35, "image plane\n(the photo)", ha="center", fontsize=9, color=BLUE)
    P = (7.0, 3.0)
    ax.add_patch(Circle(P, 0.12, color=ORANGE))
    ax.text(P[0], P[1] + 0.25, "3D point (X, Y, Z)", ha="center", fontsize=10, color=INK)
    ax.plot([cx, P[0]], [cy, P[1]], color=ORANGE, lw=1.3, ls="--")
    t = (2.6 - cx) / (P[0] - cx)
    py = cy + t * (P[1] - cy)
    ax.add_patch(Circle((2.6, py), 0.08, color=ORANGE))
    ax.text(2.75, py + 0.05, "pixel (u, v)", fontsize=10, color=INK, va="bottom")
    ax.annotate("", xy=(2.6, 0.6), xytext=(cx, 0.6), arrowprops=dict(arrowstyle="<->", color=QUIET))
    ax.text((cx + 2.6) / 2, 0.68, "focal length f", ha="center", fontsize=9, color=QUIET)
    ax.annotate("", xy=(P[0], 0.25), xytext=(cx, 0.25), arrowprops=dict(arrowstyle="<->", color=QUIET))
    ax.text((cx + P[0]) / 2, 0.05, "depth Z (what the depth camera measures)", ha="center", fontsize=9, color=QUIET)
    ax.text(4.6, 2.05, "u = f · X / Z + cx", fontsize=12, color=INK, family="Noto Sans Mono")
    ax.text(4.6, 1.6, "v = f · Y / Z + cy", fontsize=12, color=INK, family="Noto Sans Mono")
    save(fig, "pinhole.png")


def fig_slam_idea():
    fig, ax = canvas(10, 3.4)
    box(ax, 1.4, 1.7, 2.3, 1.1, "New camera image", "features detected", fc="white", ec=EDGE)
    box(ax, 4.6, 2.5, 2.6, 0.95, "Match to map", "which known points?", fc=BLUE_T, ec=BLUE)
    box(ax, 4.6, 0.9, 2.6, 0.95, "Estimate pose", "where am I now?", fc=BLUE_T, ec=BLUE)
    box(ax, 8.2, 1.7, 2.8, 1.1, "Update map", "add new 3D points,\nrefine old ones", fc=GREEN_T, ec=GREEN)
    arrow(ax, 2.55, 1.9, 3.3, 2.4)
    arrow(ax, 4.6, 2.02, 4.6, 1.38)
    arrow(ax, 5.9, 0.9, 6.8, 1.5)
    ax.annotate("", xy=(5.9, 2.6), xytext=(8.2, 2.25),
                arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.5, connectionstyle="arc3,rad=0.25"))
    ax.text(7.35, 2.85, "better map = better next pose", fontsize=9, color=GREEN, ha="center")
    save(fig, "slam_idea.png")


def fig_orbslam_threads():
    fig, ax = canvas(12.4, 4.6)
    rows = [(3.75, "Tracking", "every frame, real time", BLUE_T, BLUE,
             ["find ORB\nfeatures", "predict pose\n(constant speed)", "match to\nmap points", "solve pose\n(PnP + BA)"]),
            (2.3, "Local mapping", "every new keyframe", GREEN_T, GREEN,
             ["create new\n3D points", "remove\nbad points", "local bundle\nadjustment", "drop redundant\nkeyframes"]),
            (0.85, "Loop closing", "when a place is seen again", ORANGE_T, ORANGE,
             ["recognise place\n(bag of words)", "check\ngeometry", "correct drift\n(pose graph)", "merge maps,\nfull BA"])]
    bw, step, x0 = 1.95, 2.3, 4.0
    for y, name, when, fc, ec, steps in rows:
        box(ax, 1.4, y, 2.5, 1.05, name, when, fc=fc, ec=ec)
        arrow(ax, 2.65, y, x0 - bw / 2, y, color=ec)
        for i, st in enumerate(steps):
            x = x0 + i * step
            box(ax, x, y, bw, 0.9, st, fc="white", ec=ec, tsize=9.2, weight="normal")
            if i:
                arrow(ax, x - step + bw / 2, y, x - bw / 2, y, color=ec)
    for ya, yb in [(3.22, 2.83), (1.77, 1.38)]:
        ax.annotate("", xy=(1.4, yb), xytext=(1.4, ya), arrowprops=dict(arrowstyle="-|>", color=QUIET))
        ax.text(1.52, (ya + yb) / 2, "keyframe", fontsize=8.5, color=QUIET, va="center")
    save(fig, "orbslam_threads.png")


def fig_tsdf():
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), gridspec_kw={"width_ratios": [1.15, 1]})
    ax = axs[0]
    n = 12
    xs = np.arange(n) + 0.5
    surf = lambda x: 5.2 + 1.3 * np.sin(x / 2.2)
    for i in range(n):
        for j in range(9):
            d = surf(xs[i]) - (j + 0.5)
            t = np.clip(d / 2.0, -1, 1)
            col = (1 - max(t, 0) * 0.6, 1 - abs(t) * 0.35, 1 - max(-t, 0) * 0.6) if abs(d) < 2 else (0.93, 0.93, 0.93)
            ax.add_patch(Rectangle((i, j), 1, 1, fc=col, ec="white", lw=0.8))
            if abs(d) < 2:
                ax.text(i + 0.5, j + 0.5, f"{d:+.1f}", ha="center", va="center", fontsize=6.3, color=INK)
    xx = np.linspace(0, n, 200)
    ax.plot(xx, surf(xx), color=INK, lw=2.2)
    ax.text(n + 0.2, surf(n) - 0.1, "real\nsurface", fontsize=9, color=INK, va="center")
    ax.set_xlim(0, n + 1.6)
    ax.set_ylim(0, 9)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("Each voxel stores its distance to the surface\n(+ = camera side, − = behind it, grey = far, ignored)",
                 fontsize=10, color=INK)
    ax.annotate("camera\nlooks up", xy=(1.0, 0.2), xytext=(1.0, -1.4), ha="center", fontsize=8.5, color=QUIET,
                arrowprops=dict(arrowstyle="-|>", color=QUIET), annotation_clip=False)
    ax = axs[1]
    ax.axis("off")
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 4.2)
    steps = [("Depth image + camera pose", "from ORB-SLAM3"), ("Update voxels near surface", "running average"),
             ("Repeat for 1000s of frames", "noise averages out"), ("Marching cubes", "zero crossing → triangles")]
    for i, (t, s) in enumerate(steps):
        y = 3.7 - i * 1.05
        box(ax, 2.5, y, 4.4, 0.8, t, s, fc=BLUE_T if i < 3 else GREEN_T, ec=BLUE if i < 3 else GREEN, tsize=10)
        if i:
            arrow(ax, 2.5, y + 0.65, 2.5, y + 0.42)
    save(fig, "tsdf.png")


def fig_twin_steps():
    fig, ax = canvas(11, 2.3)
    steps = [("Clean", "drop floating bits"), ("Level", "floor flat at z = 0"), ("Square", "walls along x / y"),
             ("Simplify", "4M → 1M triangles"), ("Texture", "colours → image"), ("Package", "Gazebo model + world")]
    for i, (t, s) in enumerate(steps):
        x = 0.95 + i * 1.82
        box(ax, x, 1.15, 1.6, 1.0, t, s, fc=BLUE_T if i < 5 else GREEN_T, ec=BLUE if i < 5 else GREEN, tsize=10.5)
        if i:
            arrow(ax, x - 1.82 + 0.8, 1.15, x - 0.8, 1.15)
    save(fig, "twin_steps.png")


def fig_gazebo_arch():
    fig, ax = canvas(11, 5.0)
    ax.add_patch(FancyBboxPatch((0.2, 0.3), 6.2, 4.4, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc="#f6fbf7", ec=GREEN, lw=1.3))
    ax.text(0.45, 4.42, "Gazebo simulator", fontsize=11, weight="bold", color=GREEN)
    box(ax, 1.8, 3.55, 2.6, 0.85, "Apartment twin", "static mesh + collision", fc="white", ec=GREEN, tsize=10)
    box(ax, 4.75, 3.55, 2.6, 0.85, "Physics (DART)", "wheels, contacts, 1 ms", fc="white", ec=GREEN, tsize=10)
    ax.add_patch(FancyBboxPatch((0.5, 1.5), 5.6, 1.2, boxstyle="round,pad=0.02,rounding_size=0.12", fc="white", ec=GREEN, lw=1.4))
    ax.text(3.3, 2.45, "scout_bot robot", ha="center", va="center", fontsize=10, weight="bold", color=INK)
    for i, lab in enumerate(["DiffDrive", "Lidar", "RGB-D cam", "IMU", "Pose pub."]):
        box(ax, 1.05 + i * 1.12, 1.85, 1.0, 0.45, lab, fc=GREEN_T, ec=GREEN, tsize=7.8)
    box(ax, 3.3, 0.75, 5.6, 0.6, "Rendering (ogre2) for camera + lidar", fc="white", ec=GREEN, tsize=9.5)
    topics = [("/cmd_vel", "drive commands", True), ("/odom", "wheel odometry", False), ("/scan", "360° lidar", False),
              ("/camera/image", "colour image", False), ("/camera/depth_image", "depth image", False),
              ("/imu", "acceleration, rotation", False), ("/model/scout_bot/pose", "true pose (ground truth)", False)]
    ax.text(7.1, 4.42, "gz-transport topics", fontsize=11, weight="bold", color=BLUE)
    for i, (t, s, into) in enumerate(topics):
        y = 3.95 - i * 0.5
        ax.text(7.1, y, t, fontsize=9.2, color=INK, family="Noto Sans Mono")
        ax.text(9.25, y, s, fontsize=8.6, color=QUIET)
        if into:
            arrow(ax, 7.0, y + 0.05, 6.45, 2.4, color=BLUE)
        else:
            arrow(ax, 6.45, 2.1 - 0.03 * i, 7.0, y + 0.05, color=EDGE)
    ax.text(7.1, 0.35, "Our Python tools publish / subscribe to these\n(sim_collect.py, navigate.py)",
            fontsize=9, color=QUIET)
    save(fig, "gazebo_arch.png")


def fig_robot():
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
    ax = axs[0]
    ax.set_title("Top view", fontsize=10, color=INK)
    ax.add_patch(FancyBboxPatch((-0.15, -0.12), 0.30, 0.24, boxstyle="round,pad=0,rounding_size=0.02",
                                fc=BLUE_T, ec=BLUE, lw=1.5))
    for s in (1, -1):
        ax.add_patch(Rectangle((0.0, s * 0.14 - 0.015), 0.10, 0.03, fc=INK))
    ax.add_patch(Circle((-0.11, 0), 0.02, fc=QUIET))
    ax.add_patch(Circle((0, 0), 0.035, fc="#333"))
    ax.add_patch(Rectangle((0.10, -0.045), 0.03, 0.09, fc="#222"))
    ax.annotate("", xy=(0.32, 0), xytext=(0.17, 0), arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2))
    ax.text(0.25, 0.03, "forward (x)", fontsize=8.5, color=ORANGE)
    for txt, xy, xyt in [("drive wheels\nr = 5 cm", (0.05, 0.155), (0.16, 0.24)), ("lidar", (0, 0.03), (-0.05, 0.25)),
                         ("camera", (0.12, -0.04), (0.18, -0.22)), ("caster", (-0.12, -0.02), (-0.27, -0.2))]:
        ax.annotate(txt, xy=xy, xytext=xyt, fontsize=8.5, color=INK, arrowprops=dict(arrowstyle="-", color=QUIET, lw=0.8))
    ax.text(0, -0.31, "0.30 m x 0.24 m, 3 kg", ha="center", fontsize=9, color=QUIET)
    ax.set_xlim(-0.35, 0.4)
    ax.set_ylim(-0.35, 0.32)
    ax.set_aspect("equal")
    ax.axis("off")
    ax = axs[1]
    ax.set_title("Side view: two camera heights tested", fontsize=10, color=INK)
    ax.add_patch(Rectangle((-0.15, 0.03), 0.30, 0.08, fc=BLUE_T, ec=BLUE))
    ax.add_patch(Circle((0.05, 0.05), 0.05, fc=INK))
    ax.plot([0.1, 0.1], [0.11, 1.2], color=QUIET, lw=2)
    ax.plot([-0.4, 0.6], [0, 0], color=EDGE, lw=2)
    for h, lab, c in [(0.37, "scout_bot: 0.37 m", ORANGE), (1.20, "scout_bot_tall: 1.20 m", GREEN)]:
        ax.add_patch(Rectangle((0.09, h - 0.025), 0.05, 0.05, fc=c))
        ax.plot([0.14, 0.55], [h, h - 0.12], color=c, lw=1, ls="--")
        ax.plot([0.14, 0.55], [h, h + 0.12], color=c, lw=1, ls="--")
        ax.text(-0.38, h, lab, fontsize=9, color=c, va="center")
    ax.plot([-0.4, 0.6], [1.45, 1.45], color=QUIET, lw=1, ls=":")
    ax.text(-0.38, 1.5, "human scanner height 1.45 m", fontsize=8.5, color=QUIET)
    ax.set_xlim(-0.4, 0.6)
    ax.set_ylim(-0.05, 1.65)
    ax.axis("off")
    save(fig, "robot.png")


def fig_sim_experiment():
    fig, ax = canvas(11, 2.6)
    steps = [("Plan a loop", "A* on floor plan"), ("Drive robot", "pure pursuit"), ("Record", "images + true pose"),
             ("Run ORB-SLAM3", "on the images only"), ("Compare", "estimate vs truth")]
    for i, (t, s) in enumerate(steps):
        x = 1.05 + i * 2.2
        fc, ec = (GREEN_T, GREEN) if i < 3 else (BLUE_T, BLUE)
        box(ax, x, 1.3, 1.9, 1.0, t, s, fc=fc, ec=ec, tsize=10.5)
        if i:
            arrow(ax, x - 2.2 + 0.95, 1.3, x - 0.95, 1.3)
    ax.text(3.25, 0.35, "inside Gazebo", ha="center", fontsize=9, color=GREEN)
    ax.text(8.55, 0.35, "same SLAM code as for real video", ha="center", fontsize=9, color=BLUE)
    save(fig, "sim_experiment.png")


def fig_nav_loop():
    fig, ax = canvas(11, 4.2)
    box(ax, 1.5, 2.1, 2.4, 1.0, "You click a goal", "on the 2D floor plan", fc=ORANGE_T, ec=ORANGE)
    box(ax, 4.5, 3.2, 2.4, 0.95, "Plan (A*)", "shortest safe route", fc=BLUE_T, ec=BLUE)
    box(ax, 7.7, 3.2, 2.4, 0.95, "Follow (pure pursuit)", "steer to a point ahead", fc=BLUE_T, ec=BLUE)
    box(ax, 7.7, 1.0, 2.4, 0.95, "Watch lidar", "anything < 30 cm ahead?", fc=GREEN_T, ec=GREEN)
    box(ax, 4.5, 1.0, 2.4, 0.95, "Add obstacle to map", "stop, back off", fc=RED_T, ec=RED)
    box(ax, 10.2, 2.1, 1.4, 0.8, "Arrived", fc=GREEN_T, ec=GREEN, tsize=10)
    arrow(ax, 2.7, 2.35, 3.3, 3.0)
    arrow(ax, 5.7, 3.2, 6.5, 3.2)
    arrow(ax, 7.7, 2.72, 7.7, 1.48, "every 50 ms", off=(0.55, -0.1))
    arrow(ax, 6.5, 1.0, 5.7, 1.0, "yes", off=(0, 0.05))
    arrow(ax, 4.5, 1.48, 4.5, 2.72, "replan", off=(-0.45, -0.1))
    arrow(ax, 8.9, 1.2, 9.5, 1.9, "goal\nreached", off=(0.35, -0.3))
    ax.text(4.5, 0.2, "goal cut off → stop and report BLOCKED (never pick another goal)", ha="center",
            fontsize=9, color=RED)
    save(fig, "nav_loop.png")


def fig_pure_pursuit():
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    t = np.linspace(0, 3.4, 300)
    px, py = t, 0.55 * np.sin(t * 1.1) + 0.25 * t
    ax.plot(px, py, color=GREEN, lw=2.5, label="planned route")
    rob = np.array([0.35, -0.45])
    L = 0.9
    d = np.hypot(px - rob[0], py - rob[1])
    k = np.where((px > rob[0]) & (d >= L))[0][0]
    tgt = np.array([px[k], py[k]])
    dx, dy = tgt - rob                                    # robot heading = +x
    R = (dx ** 2 + dy ** 2) / (2 * dy)                    # circle tangent to heading through the target
    c = rob + [0, R]
    a0, a1 = -np.pi / 2, np.arctan2(tgt[1] - c[1], tgt[0] - c[0])
    ang = np.linspace(a0, a1, 60)
    ax.plot(c[0] + R * np.cos(ang), c[1] + R * np.sin(ang), color=BLUE, lw=1.8, ls=":", label="steering arc")
    ax.add_patch(Circle(rob, L, fill=False, ec=QUIET, ls="--", lw=1))
    ax.add_patch(Circle(rob, 0.09, fc=BLUE))
    ax.annotate("", xy=rob + [0.32, 0], xytext=rob, arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2))
    ax.add_patch(Circle(tgt, 0.07, fc=ORANGE, zorder=5))
    ax.text(tgt[0] + 0.5, tgt[1] - 0.02, "look-ahead point\n(35 cm ahead on route)", fontsize=9, color=ORANGE, va="top")
    ax.text(rob[0], rob[1] - 0.22, "robot", fontsize=9, color=BLUE, ha="center", va="top")
    ax.text(rob[0] - 0.55, rob[1] + 0.95, "look-ahead\ncircle", fontsize=8.5, color=QUIET, ha="center")
    ax.text(1.75, -0.95, "turn rate = 2 v sin(α) / L\nrecomputed 20 times a second", fontsize=9, color=QUIET)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-0.75, 3.5)
    ax.set_ylim(-1.45, 1.75)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    save(fig, "pure_pursuit.png")


# ------------------------------------------------------------------ real-data figures
def fig_rgbd_frames():
    d = ROOT / "data/rgbd_dataset_freiburg3_long_office_household"
    row = open(ROOT / "third_party/ORB_SLAM3/Examples/RGB-D/associations/fr3_office.txt").readlines()[600].split()
    c = cv2.cvtColor(cv2.imread(str(d / row[1])), cv2.COLOR_BGR2RGB)
    dep = cv2.imread(str(d / row[3]), -1) / 5000.0
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.9))
    axs[0].imshow(c)
    axs[0].set_title("Colour image", fontsize=10)
    im = axs[1].imshow(np.where(dep > 0, dep, np.nan), cmap="turbo", vmin=0.5, vmax=3.5)
    axs[1].set_title("Depth image (metres; black = no reading)", fontsize=10)
    axs[1].set_facecolor("black")
    for a in axs:
        a.axis("off")
    fig.colorbar(im, ax=axs[1], fraction=0.035, pad=0.02)
    save(fig, "rgbd_frame.png")


def fig_orb():
    d = ROOT / "data/rgbd_dataset_freiburg3_long_office_household"
    rows = open(ROOT / "third_party/ORB_SLAM3/Examples/RGB-D/associations/fr3_office.txt").readlines()
    a = cv2.imread(str(d / rows[600].split()[1]))
    b = cv2.imread(str(d / rows[615].split()[1]))
    orb = cv2.ORB_create(800)
    ka, da = orb.detectAndCompute(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), None)
    kb, db = orb.detectAndCompute(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), None)
    m = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
    m = sorted(m, key=lambda x: x.distance)[:70]
    kp_img = a.copy()
    for k in ka:
        cv2.circle(kp_img, (int(k.pt[0]), int(k.pt[1])), 3, (0, 210, 255), -1, cv2.LINE_AA)
    mt = cv2.drawMatches(a, ka, b, kb, m, None, matchColor=(60, 220, 60), singlePointColor=None,
                         flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    fig, axs = plt.subplots(2, 1, figsize=(9, 8.6), gridspec_kw={"height_ratios": [1, 0.62]})
    axs[0].imshow(cv2.cvtColor(kp_img, cv2.COLOR_BGR2RGB))
    axs[0].set_title(f"{len(ka)} ORB features (yellow dots): they gather on textured objects, not on blank walls", fontsize=10)
    axs[1].imshow(cv2.cvtColor(mt, cv2.COLOR_BGR2RGB))
    axs[1].set_title("Matching the same features in a frame half a second later (70 best matches)", fontsize=10)
    for x in axs:
        x.axis("off")
    save(fig, "orb_features.png")


def copy_existing():
    for src, dst in [("runs/fr3_office/map_topdown.png", "fr3_map_topdown.png"),
                     ("runs/fr3_office/trajectory_vs_gt.png", "fr3_traj_vs_gt.png"),
                     ("runs/apt0/map_topdown.png", "apt0_map_topdown.png")]:
        im = cv2.imread(str(ROOT / src))
        cv2.imwrite(str(OUT / dst), im)
        print("copied", dst)


def fig_apt0_frame():
    d = ROOT / "data/apt0/apt0"
    c = cv2.cvtColor(cv2.imread(str(d / "frame-002400.color.jpg")), cv2.COLOR_BGR2RGB)
    dep = cv2.imread(str(d / "frame-002400.depth.png"), -1) / 1000.0
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.9))
    axs[0].imshow(c)
    axs[0].set_title("apt0 colour (Structure Sensor)", fontsize=10)
    axs[1].imshow(np.where(dep > 0, dep, np.nan), cmap="turbo", vmin=0.4, vmax=4)
    axs[1].set_facecolor("black")
    axs[1].set_title("apt0 depth", fontsize=10)
    for a in axs:
        a.axis("off")
    save(fig, "apt0_frame.png")


def fig_level():
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(ROOT / "runs/apt0/dense_mesh.ply"))
    P = np.asarray(m.sample_points_uniformly(60000).points)
    T = np.loadtxt(ROOT / "gazebo/models/apt0/T_slam_to_gazebo.txt")
    Q = P @ T[:3, :3].T + T[:3, 3]
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6))
    axs[0].scatter(P[:, 2], -P[:, 1], s=0.2, c=QUIET)
    axs[0].set_title("Before: SLAM's frame (tilted with the first camera)", fontsize=10)
    axs[1].scatter(Q[:, 0], Q[:, 2], s=0.2, c=QUIET)
    axs[1].axhline(0, color=GREEN, lw=1.5)
    axs[1].text(Q[:, 0].min(), -0.45, "floor at z = 0", color=GREEN, fontsize=9)
    axs[1].set_ylim(-0.7, 3.0)
    axs[1].set_title("After: floor level at z = 0, ceiling at 2.75 m", fontsize=10)
    for a in axs:
        a.set_aspect("equal")
        a.set_xlabel("m")
    save(fig, "leveling.png")


def fig_sim_views():
    def frame(d, frac):
        A = [l.split() for l in open(ROOT / d / "associations.txt")]
        return cv2.cvtColor(cv2.imread(str(ROOT / d / A[int(frac * len(A))][1])), cv2.COLOR_BGR2RGB)
    shots = [("data/sim_apt0_loop", 0.13, "Run A: camera at 0.37 m, 2 cm twin\n(holes, undersides never scanned)"),
             ("data/sim_apt0_tall", 0.15, "Run D: camera at 1.2 m, 2 cm twin\n(sees what the scanner saw; blurry)"),
             ("data/sim_apt0hd_tall", 0.15, "Run C: camera at 1.2 m, 1 cm HD twin\n(sharp: poster text readable)"),
             ("data/INVALID_sim_apt0hd_tall_broken_texture", 0.15,
              "INVALID: texture bug (\"confetti\")\nfake detail fooled the score")]
    fig, axs = plt.subplots(2, 2, figsize=(10, 7.6))
    for a, (d, f, t) in zip(axs.ravel(), shots):
        a.imshow(frame(d, f))
        a.set_title(t, fontsize=9.5)
        a.axis("off")
    save(fig, "sim_views.png")


def fig_results_chart():
    runs = ["A\n0.37 m cam", "B\nslow turns", "D\n1.2 m cam", "C\n1.2 m + HD twin"]
    losses = [5, 5, 3, 0]
    longest = [20, 20, 36, 100]
    kp10 = [45, None, 228, 1037]
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.4))
    cols = [EDGE, EDGE, "#8fb3ee", BLUE]
    axs[0].bar(runs, losses, color=cols)
    axs[0].set_title("Tracking losses (lower is better)", fontsize=10)
    axs[1].bar(runs, longest, color=cols)
    axs[1].set_title("Longest continuous tracking, % of loop", fontsize=10)
    axs[1].set_ylim(0, 110)
    for i, v in enumerate(longest):
        axs[1].text(i, v + 3, f"{v}%", ha="center", fontsize=9)
    for i, v in enumerate(losses):
        axs[0].text(i, v + 0.1, str(v), ha="center", fontsize=9)
    r2 = [r for r, v in zip(runs, kp10) if v is not None]
    v2 = [v for v in kp10 if v is not None]
    axs[2].bar(r2, v2, color=[EDGE, "#8fb3ee", BLUE])
    axs[2].axhline(583, color=ORANGE, ls="--", lw=1.3)
    axs[2].text(-0.4, 610, "real footage: 583", color=ORANGE, fontsize=8.5)
    axs[2].set_title("Features in the worst 10% of frames", fontsize=10)
    for a in axs:
        a.tick_params(labelsize=8)
        a.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    save(fig, "sim_results.png")


def fig_nav_maps():
    from sim_collect import Grid, plan_loop, save_plan
    g = Grid(ROOT / "gazebo/models/apt0_hd/meshes/apt0_hd_visual.obj", z_obst=(0.04, 0.5))
    # 1) floor plan layers
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.6))
    img = np.full(g.blocked.shape + (3,), 1.0)
    img[~g.free] = (0.82, 0.82, 0.82)
    img[g.blocked] = (0.28, 0.28, 0.28)
    ext = [g.lo[0], g.lo[0] + g.blocked.shape[0] * 0.05, g.lo[1], g.lo[1] + g.blocked.shape[1] * 0.05]
    axs[0].imshow(np.transpose(img, (1, 0, 2))[::-1], extent=ext)
    axs[0].set_title("Floor plan from the scan\nwhite = drivable, light grey = too close, dark = obstacle", fontsize=9.5)
    clear = np.ma.masked_where(g.blocked, g.clear)
    im = axs[1].imshow(clear.T[::-1], extent=ext, cmap="viridis")
    axs[1].set_title("Clearance: distance to nearest obstacle (m)", fontsize=9.5)
    fig.colorbar(im, ax=axs[1], fraction=0.04)
    for a in axs:
        a.set_xlabel("x (m)")
    save(fig, "nav_grid.png")
    # 2) a planned route between two rooms
    s, t = tuple(g.to_ij([0.55, -0.28])), tuple(g.to_ij([-1.67, 0.41]))
    path = g.to_xy(np.array(g.astar(s, t)))
    fig, ax = plt.subplots(figsize=(5.4, 4.8))
    ax.imshow(np.transpose(img, (1, 0, 2))[::-1], extent=ext)
    ax.plot(path[:, 0], path[:, 1], color=GREEN, lw=2.5, label="A* route")
    ax.plot(*path[0], "o", color=BLUE, ms=9, label="robot")
    ax.plot(*path[-1], "o", color=RED, ms=9, label="goal (click)")
    L = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
    ax.set_title(f"A* route between rooms: {L:.1f} m, centred in the corridors", fontsize=9.5)
    ax.legend(loc="lower right", fontsize=8.5)
    ax.set_xlabel("x (m)")
    save(fig, "nav_route.png")
    # 3) the recording loop
    p, w = plan_loop(g, np.array([0.55, -0.28]), 8, 0.35, np.random.default_rng(0))
    save_plan(g, p, w, OUT / "sim_plan.png")


if __name__ == "__main__":
    which = sys.argv[1:] or ["all"]
    fns = [fig_pipeline, fig_pinhole, fig_slam_idea, fig_orbslam_threads, fig_tsdf, fig_twin_steps,
           fig_gazebo_arch, fig_robot, fig_sim_experiment, fig_nav_loop, fig_pure_pursuit, fig_rgbd_frames,
           fig_orb, copy_existing, fig_apt0_frame, fig_level, fig_sim_views, fig_results_chart, fig_nav_maps]
    for f in fns:
        if "all" in which or f.__name__ in which:
            f()
