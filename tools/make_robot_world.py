"""Create a Gazebo world with scout_bot for an exported twin, spawning the robot in open space.

    .venv/bin/python tools/make_robot_world.py --map apt0_splg          # -> gazebo/worlds/apt0_splg_robot.sdf
    .venv/bin/python tools/make_robot_world.py --map apt0_splg --tall   # camera on the 1.2 m mast

The spawn point is the most open spot on the scanner's walked path (camera_path_gz.txt written by
export_gazebo.py), so the robot never starts inside furniture.
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sim_collect import Grid  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="model name used with export_gazebo.py --name")
    ap.add_argument("--tall", action="store_true", help="use scout_bot_tall (camera at 1.2 m)")
    args = ap.parse_args()
    model = ROOT / "gazebo/models" / args.map

    grid = Grid(model / "meshes" / f"{args.map}_visual.obj", z_obst=(0.04, 1.3 if args.tall else 0.5))
    cams = np.loadtxt(model / "camera_path_gz.txt")[::10, :2]
    ij = grid.to_ij(cams)
    H, W = grid.free.shape
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < H) & (ij[:, 1] >= 0) & (ij[:, 1] < W)
    clear = np.zeros(len(cams))
    clear[ok] = grid.clear[ij[ok, 0], ij[ok, 1]]
    i = int(np.argmax(clear))
    x, y = cams[i]

    template = (ROOT / "gazebo/worlds" / ("apt0_robot_tall.sdf" if args.tall else "apt0_robot.sdf")).read_text()
    world = template.replace("<uri>model://apt0</uri>", f"<uri>model://{args.map}</uri>")
    world = re.sub(r"(<name>scout_bot</name>\s*<pose>)[^<]+(</pose>)", rf"\g<1>{x:.2f} {y:.2f} 0.02 0 0 0\g<2>", world)
    out = ROOT / "gazebo/worlds" / f"{args.map}_robot{'_tall' if args.tall else ''}.sdf"
    out.write_text(world)
    print(f"spawn ({x:.2f}, {y:.2f}), {clear[i]:.2f} m clearance")
    print(f"wrote {out}")
    print(f"navigate:  .venv/bin/python tools/navigate.py --world {out.stem} --map {args.map}")


if __name__ == "__main__":
    main()
