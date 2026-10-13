"""Click-to-go autonomous navigation for scout_bot in a Gazebo digital twin.

A 2D map window shows the apartment floor plan (from the scanned mesh) with the robot live.
Click a point -> A* plans a route -> pure pursuit drives it. The route is also drawn in the
Gazebo 3D view. The lidar watches the way ahead: if something blocks it that the map did not
show, the robot stops, adds it to the map and replans.

Localization: the simulator's ground-truth pose (a stand-in for a localization system).

Usage:
    .venv/bin/python tools/navigate.py                       # starts Gazebo + map window
    .venv/bin/python tools/navigate.py --goal -1.5 0.4       # drive to x y right away
    .venv/bin/python tools/navigate.py --attach              # Gazebo is already running

Map window:  left-click = go there,  S / Space = stop,  C = clear lidar obstacles,  Q / Esc = quit
"""
import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_collect import RES, ROBOT_RADIUS, Grid  # noqa: E402

from gz.transport13 import Node  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.laserscan_pb2 import LaserScan  # noqa: E402
from gz.msgs10.marker_pb2 import Marker  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCALE = 5                      # map window pixels per grid cell
LOOKAHEAD = 0.35               # pure pursuit (m)
STOP_DIST = 0.30               # lidar: obstacle closer than this in front -> stop (m)
STOP_HALF_ANGLE = np.radians(25)
GOAL_TOL = 0.12                # arrived when this close (m)
SNAP_TOL = 0.30                # a goal moved further than this to reach free space is reported


class Robot:
    """Latest ground-truth pose and lidar scan (filled by gz-transport callbacks)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.T = None
        self.scan = None

    def on_pose(self, msg):
        if msg.name != "scout_bot":
            return
        T = np.eye(4)
        o = msg.orientation
        T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        T[:3, 3] = [msg.position.x, msg.position.y, msg.position.z]
        with self.lock:
            self.T = T

    def on_scan(self, msg):
        a = msg.angle_min + msg.angle_step * np.arange(msg.count)
        r = np.array(msg.ranges)
        with self.lock:
            self.scan = (a, r)

    def state(self):
        with self.lock:
            if self.T is None:
                return None
            return self.T[0, 3], self.T[1, 3], np.arctan2(self.T[1, 0], self.T[0, 0]), self.scan


class Navigator:
    def __init__(self, grid, node, max_speed, max_turn):
        self.grid, self.node = grid, node
        self.max_speed, self.max_turn = max_speed, max_turn
        self.cmd_pub = node.advertise("/cmd_vel", Twist)
        self.extra = np.zeros_like(grid.blocked)    # obstacles found by the lidar
        self.path, self.goal, self.idx = None, None, 0
        self.target = None                          # the goal the user asked for (self.goal = snapped to free space)
        self.status = "IDLE - click on the map to set a goal"
        self.replans = 0

    # ------------------------------------------------------------ planning
    def nearest_free(self, ij, comp=None):
        mask = self.grid.free if comp is None else (self.grid.free & comp)
        cand = np.argwhere(mask)
        if len(cand) == 0:
            return None
        return tuple(cand[np.argmin(np.linalg.norm(cand - np.array(ij), axis=1))])

    def plan(self, x, y, goal_xy):
        g = self.grid
        g.update(self.extra)
        s = tuple(g.to_ij([x, y]))
        if not (0 <= s[0] < g.free.shape[0] and 0 <= s[1] < g.free.shape[1]) or not g.free[s]:
            s = self.nearest_free(s)              # robot is inside the safety margin: start from closest free cell
            if s is None:
                return None
        comp = g.component(s)
        t = self.nearest_free(g.to_ij(goal_xy), comp)   # snap goal to a reachable free cell
        if t is None:
            return None
        cells = g.astar(s, t)
        if cells is None:
            return None
        xy = g.to_xy(np.array(cells))
        keep = [0]
        for i in range(1, len(xy)):
            if np.linalg.norm(xy[i] - xy[keep[-1]]) >= 0.1:
                keep.append(i)
        if keep[-1] != len(xy) - 1:
            keep.append(len(xy) - 1)
        return np.vstack([[x, y], xy[keep]])

    def set_goal(self, goal_xy, st):
        x, y, _, _ = st
        self.target = np.array(goal_xy, float)
        self.replans = 0
        self.path = self.plan(x, y, self.target)
        self.idx = 0
        if self.path is None:
            self.status = f"NO ROUTE to ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})"
            self.goal = None
            self.stop()
        else:
            self.goal = self.path[-1]              # snapped to free space
            L = np.linalg.norm(np.diff(self.path, axis=0), axis=1).sum()
            off = np.linalg.norm(self.goal - self.target)
            if off > SNAP_TOL:
                self.status = (f"DRIVING to closest reachable point ({self.goal[0]:.2f}, {self.goal[1]:.2f}), "
                               f"{off:.2f} m from the click - route {L:.1f} m")
            else:
                self.status = f"DRIVING to ({self.goal[0]:.2f}, {self.goal[1]:.2f}) - route {L:.1f} m"
        self.draw_markers()

    # ------------------------------------------------------------ control
    def stop(self):
        self.cmd_pub.publish(Twist())

    def cancel(self, why="STOPPED"):
        self.path, self.goal = None, None
        self.stop()
        self.status = why
        self.draw_markers()

    def obstacle_ahead(self, st):
        x, y, yaw, scan = st
        if scan is None:
            return False
        a, r = scan
        front = (np.abs(a) < STOP_HALF_ANGLE) & np.isfinite(r)
        return bool(np.any(r[front] < STOP_DIST))

    def add_scan_obstacles(self, st):
        """Mark NEW lidar hits (in front, within 1 m, not already on the map) as blocked (kept until 'C')."""
        x, y, yaw, scan = st
        a, r = scan
        ok = np.isfinite(r) & (r < 1.0) & (np.abs(a) < np.radians(60))
        px = x + r[ok] * np.cos(yaw + a[ok])
        py = y + r[ok] * np.sin(yaw + a[ok])
        ij = self.grid.to_ij(np.c_[px, py])
        H, W = self.extra.shape
        ij = ij[(ij[:, 0] >= 0) & (ij[:, 0] < H) & (ij[:, 1] >= 0) & (ij[:, 1] < W)]
        known = self.grid.clear[ij[:, 0], ij[:, 1]] < 2 * RES            # already a mapped obstacle
        ij = ij[~known]
        self.extra[ij[:, 0], ij[:, 1]] = True

    def step(self, st):
        if self.path is None:
            return
        x, y, yaw, _ = st
        p = np.array([x, y])
        if np.linalg.norm(self.goal - p) < GOAL_TOL:
            self.cancel(f"ARRIVED at ({self.goal[0]:.2f}, {self.goal[1]:.2f}) - click a new goal")
            return
        if self.obstacle_ahead(st):
            self.stop()
            self.add_scan_obstacles(st)
            self.replans += 1
            if self.replans > 5:
                self.cancel("BLOCKED - could not get around the obstacle")
                return
            self.path = self.plan(x, y, self.goal)
            self.idx = 0
            # never silently swap in a different destination: the old goal must still be reachable
            if self.path is None or np.linalg.norm(self.path[-1] - self.goal) > SNAP_TOL:
                self.cancel("BLOCKED - obstacle cuts off the goal, no route around it")
                return
            self.status = f"REPLANNED around an obstacle ({self.replans})"
            self.draw_markers()
            # back off a little so the new route starts with room to turn
            back = Twist()
            back.linear.x = -0.1
            self.cmd_pub.publish(back)
            time.sleep(0.4)
            return
        path = self.path
        d = np.linalg.norm(path[self.idx:] - p, axis=1)
        self.idx += int(np.argmin(d[:8]))
        tgt = self.idx
        while tgt < len(path) - 1 and np.linalg.norm(path[tgt] - p) < LOOKAHEAD:
            tgt += 1
        v = path[tgt] - p
        alpha = (np.arctan2(v[1], v[0]) - yaw + np.pi) % (2 * np.pi) - np.pi
        cmd = Twist()
        if abs(alpha) > 0.7:
            cmd.angular.z = self.max_turn * np.sign(alpha)
        else:
            dist_goal = np.linalg.norm(self.goal - p)
            speed = min(self.max_speed, 0.6 * dist_goal + 0.05)       # slow down near the goal
            cmd.linear.x = speed * (1 - 0.6 * abs(alpha) / 0.7)
            cmd.angular.z = float(np.clip(2 * speed * np.sin(alpha) / LOOKAHEAD, -self.max_turn, self.max_turn))
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------ Gazebo 3D markers
    def draw_markers(self):
        clear = Marker()
        clear.ns = "nav"
        clear.action = Marker.DELETE_ALL
        self.node.request("/marker", clear, Marker, Boolean, 1000)
        if self.path is None:
            return
        line = Marker()
        line.ns, line.id = "nav", 1
        line.action, line.type = Marker.ADD_MODIFY, Marker.LINE_STRIP
        line.visibility = Marker.GUI
        for c, v in [(line.material.ambient, (0, 0.8, 0.2, 1)), (line.material.diffuse, (0, 0.8, 0.2, 1)),
                     (line.material.emissive, (0, 0.8, 0.2, 1))]:
            c.r, c.g, c.b, c.a = v
        for px, py in self.path:
            pt = line.point.add()
            pt.x, pt.y, pt.z = px, py, 0.05
        self.node.request("/marker", line, Marker, Boolean, 1000)
        goal = Marker()
        goal.ns, goal.id = "nav", 2
        goal.action, goal.type = Marker.ADD_MODIFY, Marker.SPHERE
        goal.visibility = Marker.GUI
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.goal[0], self.goal[1], 0.1
        goal.scale.x = goal.scale.y = goal.scale.z = 0.15
        for c in (goal.material.ambient, goal.material.diffuse, goal.material.emissive):
            c.r, c.g, c.b, c.a = 1, 0.1, 0.1, 1
        self.node.request("/marker", goal, Marker, Boolean, 1000)


# ------------------------------------------------------------------ map window
class MapView:
    def __init__(self, grid):
        self.grid = grid
        self.H = grid.blocked.shape[1] * SCALE
        self.click = None

    def to_px(self, x, y):
        return (int((x - self.grid.lo[0]) / RES * SCALE), int(self.H - (y - self.grid.lo[1]) / RES * SCALE))

    def to_xy(self, u, v):
        return (self.grid.lo[0] + u / SCALE * RES, self.grid.lo[1] + (self.H - v) / SCALE * RES)

    def on_mouse(self, event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = self.to_xy(u, v)

    def render(self, nav, st):
        g = self.grid
        base = np.full(g.blocked.shape, 255, np.uint8)
        base[~g.free] = 205                                         # too close to obstacles for the robot
        base[g.blocked] = 70                                        # walls / furniture / unscanned
        img = cv2.cvtColor(base.T[::-1].copy(), cv2.COLOR_GRAY2BGR)
        img[nav.extra.T[::-1]] = (0, 140, 255)                      # lidar-found obstacles
        img = cv2.resize(img, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
        if nav.path is not None:
            pts = np.array([self.to_px(*p) for p in nav.path], np.int32)
            cv2.polylines(img, [pts], False, (40, 170, 40), 2)
            cv2.circle(img, self.to_px(*nav.goal), 7, (30, 30, 230), -1)
        if st is not None:
            x, y, yaw, scan = st
            if scan is not None:
                a, r = scan
                ok = np.isfinite(r) & (r < 8)
                for px, py in zip(x + r[ok] * np.cos(yaw + a[ok]), y + r[ok] * np.sin(yaw + a[ok])):
                    cv2.circle(img, self.to_px(px, py), 1, (200, 0, 200), -1)
            c = self.to_px(x, y)
            cv2.circle(img, c, int(ROBOT_RADIUS / RES * SCALE * 0.75), (200, 90, 20), -1)
            tip = self.to_px(x + 0.3 * np.cos(yaw), y + 0.3 * np.sin(yaw))
            cv2.line(img, c, tip, (255, 255, 255), 2)
            cv2.putText(img, f"robot ({x:.2f}, {y:.2f})", (8, img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1)
        bar = np.full((52, img.shape[1], 3), 245, np.uint8)
        cv2.putText(bar, nav.status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1)
        cv2.putText(bar, "click = go   S/space = stop   C = clear lidar obstacles   Q = quit", (8, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1)
        return np.vstack([bar, img])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="apt0_hd_robot")
    ap.add_argument("--map", default="apt0_hd", help="twin model used to build the floor plan")
    ap.add_argument("--obst-top", type=float, default=0.5, help="geometry below this height blocks the robot (m)")
    ap.add_argument("--goal", type=float, nargs=2, help="drive to this x y as soon as the robot is ready")
    ap.add_argument("--attach", action="store_true", help="use an already running Gazebo")
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--max-turn", type=float, default=0.9)
    args = ap.parse_args()

    print("building floor plan from the scanned mesh...")
    grid = Grid(ROOT / "gazebo/models" / args.map / "meshes" / f"{args.map}_visual.obj",
                z_obst=(0.04, args.obst_top))

    sim = None
    if not args.attach:
        env = dict(os.environ, GZ_SIM_RESOURCE_PATH=str(ROOT / "gazebo/models"))
        # opencv-python points Qt at its own bundled plugins on import; Gazebo's Qt GUI must not inherit that
        for k in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
            env.pop(k, None)
        sim = subprocess.Popen(["gz", "sim", "-r", str(ROOT / "gazebo/worlds" / f"{args.world}.sdf")],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               start_new_session=True)
    node = Node()
    robot = Robot()
    node.subscribe(Pose, "/model/scout_bot/pose", robot.on_pose)
    node.subscribe(LaserScan, "/scan", robot.on_scan)
    nav = Navigator(grid, node, args.speed, args.max_turn)
    view = MapView(grid)
    win = "scout_bot navigation (click to set goal)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, lambda e, u, v, f, p: view.on_mouse(e, u, v - 52, f, p))

    print("waiting for the robot...")
    pending_goal = args.goal
    try:
        while True:
            st = robot.state()
            if st is None:
                nav.status = "waiting for Gazebo..."
            else:
                if pending_goal is not None:
                    nav.set_goal(pending_goal, st)
                    print(nav.status)
                    pending_goal = None
                if view.click is not None:
                    nav.set_goal(view.click, st)
                    print(nav.status)
                    view.click = None
                prev = nav.status
                nav.step(st)
                if nav.status != prev:
                    print(nav.status)
            cv2.imshow(win, view.render(nav, st))
            k = cv2.waitKey(50) & 0xFF
            if k in (ord("q"), 27):
                break
            if k in (ord("s"), ord(" ")):
                nav.cancel("STOPPED - click a new goal")
            if k == ord("c"):
                nav.extra[:] = False
                grid.update(nav.extra)
                nav.status = "lidar obstacles cleared"
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        nav.stop()
        cv2.destroyAllWindows()
        if sim is not None:
            os.killpg(sim.pid, signal.SIGINT)
            try:
                sim.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(sim.pid, signal.SIGKILL)


if __name__ == "__main__":
    main()
