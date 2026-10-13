"""Drive scout_bot through a Gazebo digital twin and record a TUM-format RGB-D dataset
with perfect ground truth.

Pipeline
    1. occupancy grid from the twin's collision mesh (floor = free, 0.04-0.5 m = obstacle)
    2. pick K well-spread waypoints in open space, order them as a loop back to the start
    3. A* between waypoints (clearance-weighted), pure-pursuit control on the GT pose
    4. record /camera/image + /camera/depth_image + GT camera pose, write TUM files

Output (in --out): rgb/ depth/ rgb.txt depth.txt associations.txt groundtruth.txt
                   camera.yaml (ORB-SLAM3 config), plan.png, collect_log.txt

Usage:
    .venv/bin/python tools/sim_collect.py --out data/sim_apt0_loop          # headless
    .venv/bin/python tools/sim_collect.py --out data/sim_apt0_loop --gui    # watch it drive
"""
import argparse
import heapq
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial.transform import Rotation

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.twist_pb2 import Twist

ROOT = Path(__file__).resolve().parents[1]
RES = 0.05                     # occupancy grid resolution (m)
ROBOT_RADIUS = 0.20            # chassis half-diagonal + margin
DEPTH_FACTOR = 5000.0          # TUM convention

# camera pose in the model frame (base_link at z=0.07 + sensor offset); set by --cam-z
T_MODEL_CAM = np.eye(4)
T_MODEL_CAM[:3, 3] = [0.12, 0.0, 0.37]
# Gazebo camera frame (x fwd, y left, z up) -> optical frame (x right, y down, z fwd)
T_CAM_OPT = np.eye(4)
T_CAM_OPT[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]


# ------------------------------------------------------------------ planning
class Grid:
    def __init__(self, mesh_path, z_obst=(0.04, 0.5)):
        mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        o3d.utility.random.seed(0)                                    # repeatable plans
        P = np.asarray(mesh.sample_points_uniformly(1_500_000).points)
        self.lo = P[:, :2].min(0) - 0.5
        shape = np.ceil((P[:, :2].max(0) + 0.5 - self.lo) / RES).astype(int)
        ij = self.to_ij(P[:, :2])
        floor = np.zeros(shape, bool)
        obst = np.zeros(shape, bool)
        f = P[:, 2] < z_obst[0]
        o = (P[:, 2] >= z_obst[0]) & (P[:, 2] < z_obst[1])
        floor[ij[f, 0], ij[f, 1]] = True
        obst[ij[o, 0], ij[o, 1]] = True
        floor = ndimage.binary_closing(floor, iterations=2)          # fill small scan holes
        self.blocked = obst | ~floor                                  # unscanned floor is not trusted
        self.update(np.zeros_like(self.blocked))

    def update(self, extra_blocked):
        """Recompute clearance with additional blocked cells (e.g. obstacles seen by the lidar)."""
        blocked = self.blocked | extra_blocked
        self.clear = ndimage.distance_transform_edt(~blocked) * RES   # distance to nearest blocked cell
        self.free = self.clear >= ROBOT_RADIUS

    def to_ij(self, xy):
        return np.floor((np.asarray(xy) - self.lo) / RES).astype(int)

    def to_xy(self, ij):
        return (np.asarray(ij) + 0.5) * RES + self.lo

    def component(self, start_ij):
        lab, _ = ndimage.label(self.free)
        return lab == lab[tuple(start_ij)]

    def astar(self, a, b):
        """8-connected A*; cost grows near obstacles to keep the robot centred."""
        cost = 1.0 + 3.0 * np.exp(-(self.clear - ROBOT_RADIUS) / 0.15)
        H, W = self.free.shape
        steps = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        g = {a: 0.0}
        parent = {a: None}
        pq = [(0.0, a)]
        while pq:
            _, cur = heapq.heappop(pq)
            if cur == b:
                break
            for di, dj, dl in steps:
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < H and 0 <= nb[1] < W) or not self.free[nb]:
                    continue
                ng = g[cur] + dl * cost[nb]
                if ng < g.get(nb, np.inf):
                    g[nb], parent[nb] = ng, cur
                    heapq.heappush(pq, (ng + np.hypot(nb[0] - b[0], nb[1] - b[1]), nb))
        if b not in parent:
            return None
        path, n = [], b
        while n is not None:
            path.append(n)
            n = parent[n]
        return path[::-1]


def plan_loop(grid, start_xy, k, min_clear, rng):
    s = tuple(grid.to_ij(start_xy))
    if not grid.free[s]:                                   # snap start to nearest free cell
        free_ij = np.argwhere(grid.free)
        s = tuple(free_ij[np.argmin(np.linalg.norm(free_ij - np.array(s), axis=1))])
    comp = grid.component(s)
    cand = np.argwhere(comp & (grid.clear >= min_clear))
    # farthest-point sampling from the start: spreads waypoints over the whole space
    pts = [np.array(s)]
    d = np.linalg.norm(cand - pts[0], axis=1)
    for _ in range(k):
        i = int(np.argmax(d + rng.uniform(0, 1e-3, len(d))))
        pts.append(cand[i])
        d = np.minimum(d, np.linalg.norm(cand - cand[i], axis=1))
    # order as a nearest-neighbour tour and return home (gives loop closures)
    order, left = [0], list(range(1, len(pts)))
    while left:
        j = min(left, key=lambda j: np.linalg.norm(pts[order[-1]] - pts[j]))
        order.append(j)
        left.remove(j)
    order.append(0)
    tour = [tuple(pts[i]) for i in order]
    path = []
    for a, b in zip(tour[:-1], tour[1:]):
        seg = grid.astar(a, b)
        if seg is None:
            continue
        path.extend(seg if not path else seg[1:])
    xy = grid.to_xy(np.array(path))
    # resample every ~0.1 m
    keep = [0]
    for i in range(1, len(xy)):
        if np.linalg.norm(xy[i] - xy[keep[-1]]) >= 0.1:
            keep.append(i)
    return xy[keep], grid.to_xy(np.array(tour))


def save_plan(grid, path, waypoints, out):
    img = np.where(grid.free, 255, np.where(grid.clear > 0, 170, 60)).astype(np.uint8)
    img = cv2.cvtColor(img.T[::-1].copy(), cv2.COLOR_GRAY2BGR)       # x right, y up
    H = img.shape[0]
    pix = lambda xy: (int((xy[0] - grid.lo[0]) / RES), H - 1 - int((xy[1] - grid.lo[1]) / RES))
    for a, b in zip(path[:-1], path[1:]):
        cv2.line(img, pix(a), pix(b), (0, 0, 255), 1)
    for i, w in enumerate(waypoints[:-1]):
        cv2.circle(img, pix(w), 3, (0, 160, 0), -1)
        cv2.putText(img, str(i), pix(w), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 100, 0), 1)
    cv2.imwrite(str(out), cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))


# ------------------------------------------------------------------ recording
class Recorder:
    def __init__(self, out):
        self.out = out
        (out / "rgb").mkdir(parents=True, exist_ok=True)
        (out / "depth").mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.pose = None                  # latest (t, T_world_model)
        self.poses = []                   # all (t, T_world_model)
        self.rgb, self.depth = {}, {}     # stamp -> filename
        self.recording = False

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    def on_pose(self, msg):
        if msg.name != "scout_bot":
            return
        T = np.eye(4)
        o = msg.orientation
        T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        T[:3, 3] = [msg.position.x, msg.position.y, msg.position.z]
        t = self.stamp(msg)
        with self.lock:
            self.pose = (t, T)
            if self.recording:
                self.poses.append((t, T))

    def on_rgb(self, msg):
        if not self.recording:
            return
        t = self.stamp(msg)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        name = f"rgb/{t:.6f}.png"
        cv2.imwrite(str(self.out / name), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        with self.lock:
            self.rgb[round(t, 6)] = name

    def on_depth(self, msg):
        if not self.recording:
            return
        t = self.stamp(msg)
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        d = np.where(np.isfinite(d) & (d > 0.1) & (d < 8.0), d, 0.0)
        name = f"depth/{t:.6f}.png"
        cv2.imwrite(str(self.out / name), (d * DEPTH_FACTOR).astype(np.uint16))
        with self.lock:
            self.depth[round(t, 6)] = name

    def write(self, fx, fy, cx, cy, fps):
        out = self.out
        rgb = sorted(self.rgb.items())
        depth = sorted(self.depth.items())
        (out / "rgb.txt").write_text("".join(f"{t:.6f} {f}\n" for t, f in rgb))
        (out / "depth.txt").write_text("".join(f"{t:.6f} {f}\n" for t, f in depth))
        both = sorted(set(self.rgb) & set(self.depth))          # same sensor: identical stamps
        (out / "associations.txt").write_text(
            "".join(f"{t:.6f} {self.rgb[t]} {t:.6f} {self.depth[t]}\n" for t in both))
        lines = ["# ground truth: optical camera frame in Gazebo world (TUM format)\n"]
        for t, T in self.poses:
            Tc = T @ T_MODEL_CAM @ T_CAM_OPT
            q = Rotation.from_matrix(Tc[:3, :3]).as_quat()
            lines.append(f"{t:.6f} {Tc[0,3]:.6f} {Tc[1,3]:.6f} {Tc[2,3]:.6f} "
                         f"{q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}\n")
        (out / "groundtruth.txt").write_text("".join(lines))
        (out / "camera.yaml").write_text(CAMERA_YAML.format(fx=fx, fy=fy, cx=cx, cy=cy, fps=fps,
                                                             depth=DEPTH_FACTOR))
        return len(both), len(self.poses)


CAMERA_YAML = """%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: {fx}
Camera1.fy: {fy}
Camera1.cx: {cx}
Camera1.cy: {cy}
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: {fps}
Camera.RGB: 0
Stereo.ThDepth: 40.0
Stereo.b: 0.07
RGBD.DepthMapFactor: {depth}
ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""


# ------------------------------------------------------------------ control
def yaw_of(T):
    return np.arctan2(T[1, 0], T[0, 0])


def follow(path, rec, pub, speed, log, timeout_s, max_turn):
    """Pure pursuit on the GT pose. Returns when the end of the path is reached."""
    lookahead, idx = 0.35, 0
    t0 = time.time()
    last_prog, last_prog_t = 0, time.time()
    while True:
        with rec.lock:
            pose = rec.pose
        if pose is None:
            time.sleep(0.05)
            continue
        T = pose[1]
        p, yaw = T[:2, 3], yaw_of(T)
        # advance the target index to the first path point beyond the lookahead
        d = np.linalg.norm(path[idx:] - p, axis=1)
        idx += int(np.argmin(d[:8]))    # nearest point within 0.8 m ahead: never skip where the path doubles back
        tgt = idx
        while tgt < len(path) - 1 and np.linalg.norm(path[tgt] - p) < lookahead:
            tgt += 1
        if idx >= len(path) - 2 and np.linalg.norm(path[-1] - p) < 0.15:
            break
        if idx > last_prog:
            last_prog, last_prog_t = idx, time.time()
        if time.time() - last_prog_t > 15:
            log(f"  stuck at path index {idx}/{len(path)}; skipping ahead")
            idx = min(idx + 10, len(path) - 1)
            last_prog_t = time.time()
        if time.time() - t0 > timeout_s:
            log("  timeout")
            break
        v = path[tgt] - p
        alpha = np.arctan2(v[1], v[0]) - yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        cmd = Twist()
        if abs(alpha) > 0.7:                                # turn in place first
            cmd.angular.z = max_turn * np.sign(alpha)
        else:
            cmd.linear.x = speed * (1 - 0.6 * abs(alpha) / 0.7)
            cmd.angular.z = float(np.clip(2 * speed * np.sin(alpha) / lookahead, -max_turn, max_turn))
        pub.publish(cmd)
        time.sleep(0.05)
    pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--world", default="apt0_robot")
    ap.add_argument("--map", default="apt0", help="model whose collision mesh is used for planning")
    ap.add_argument("--start", type=float, nargs=2, default=[0.55, -0.28])
    ap.add_argument("--waypoints", type=int, default=8)
    ap.add_argument("--min-clear", type=float, default=0.35, help="waypoint clearance (m)")
    ap.add_argument("--speed", type=float, default=0.25)
    ap.add_argument("--max-turn", type=float, default=0.9, help="max turn rate (rad/s)")
    ap.add_argument("--cam-z", type=float, default=0.37, help="camera height above floor in the robot model (m)")
    ap.add_argument("--obst-top", type=float, default=0.5,
                    help="geometry below this height blocks the robot; >= camera height for tall masts")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gui", action="store_true", help="show the Gazebo GUI while driving")
    ap.add_argument("--timeout", type=float, default=900, help="max driving time (s)")
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    logf = open(out / "collect_log.txt", "w")

    def log(s):
        print(s, flush=True)
        logf.write(s + "\n")
        logf.flush()

    # ---- plan
    # visual mesh: it still has the scanned floor (the collision mesh has a flat ground plane instead)
    T_MODEL_CAM[2, 3] = args.cam_z
    grid = Grid(ROOT / "gazebo/models" / args.map / "meshes" / f"{args.map}_visual.obj",
                z_obst=(0.04, args.obst_top))
    path, wps = plan_loop(grid, np.array(args.start), args.waypoints, args.min_clear,
                          np.random.default_rng(args.seed))
    length = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
    log(f"plan: {len(wps) - 2} waypoints, path {length:.1f} m, ~{length / args.speed / 60:.1f} min of driving")
    save_plan(grid, path, wps, out / "plan.png")

    # ---- simulator
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=str(ROOT / "gazebo/models"))
    world = str(ROOT / "gazebo/worlds" / f"{args.world}.sdf")
    cmd = ["gz", "sim", "-r", world] if args.gui else ["gz", "sim", "-s", "-r", "--headless-rendering", world]
    sim = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           start_new_session=True)
    try:
        node = Node()
        rec = Recorder(out)
        node.subscribe(Pose, "/model/scout_bot/pose", rec.on_pose)
        node.subscribe(Image, "/camera/image", rec.on_rgb)
        node.subscribe(Image, "/camera/depth_image", rec.on_depth)
        pub = node.advertise("/cmd_vel", Twist)
        log("waiting for the simulator...")
        t0 = time.time()
        while rec.pose is None:
            if time.time() - t0 > 180:                   # large twins take a while to load
                raise RuntimeError("no pose from Gazebo after 180 s")
            time.sleep(0.2)
        time.sleep(3.0)                                  # let the robot settle on the floor
        rec.recording = True
        log("recording + driving")
        t_drive = time.time()
        follow(path, rec, pub, args.speed, log, args.timeout, args.max_turn)
        time.sleep(1.0)
        rec.recording = False
        n_frames, n_poses = rec.write(554.38271282264407, 554.38271282264407, 320.0, 240.0, 15)
        sim_dt = rec.poses[-1][0] - rec.poses[0][0] if rec.poses else 0
        gt = np.array([T[:2, 3] for _, T in rec.poses])
        driven = np.linalg.norm(np.diff(gt, axis=0), axis=1).sum() if len(gt) > 1 else 0
        log(f"done: {n_frames} RGB-D frames, {n_poses} GT poses, {sim_dt:.0f} s sim time "
            f"({time.time() - t_drive:.0f} s wall), drove {driven:.1f} m, "
            f"end-to-start distance {np.linalg.norm(gt[-1] - gt[0]):.2f} m")
        log(f"wrote {out}")
    finally:
        os.killpg(sim.pid, signal.SIGINT)
        try:
            sim.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(sim.pid, signal.SIGKILL)


if __name__ == "__main__":
    main()
