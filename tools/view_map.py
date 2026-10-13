"""Open a SLAM map in an interactive 3D window.

Usage:
    .venv/bin/python tools/view_map.py                     # dense mesh + trajectory
    .venv/bin/python tools/view_map.py --what cloud        # dense coloured point cloud
    .venv/bin/python tools/view_map.py --what sparse       # ORB-SLAM3 sparse map points
    .venv/bin/python tools/view_map.py --run runs/other    # a different run

Mouse: left-drag rotate, right-drag / ctrl+drag pan, wheel zoom.
Keys:  +/- point size, R reset view, H help (printed in terminal), Q quit.
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def trajectory_lineset(path, color=(1.0, 0.0, 0.0)):
    a = np.loadtxt(path, comments="#")
    pts = a[:, 1:4]
    lines = np.c_[np.arange(len(pts) - 1), np.arange(1, len(pts))]
    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(pts), o3d.utility.Vector2iVector(lines))
    ls.paint_uniform_color(color)
    return ls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="runs/fr3_office", type=Path)
    ap.add_argument("--what", choices=["mesh", "cloud", "sparse"], default="mesh")
    ap.add_argument("--no-traj", action="store_true", help="hide the camera trajectory")
    args = ap.parse_args()

    if args.what == "mesh":
        geom = o3d.io.read_triangle_mesh(str(args.run / "dense_mesh.ply"))
        geom.compute_vertex_normals()
    elif args.what == "cloud":
        geom = o3d.io.read_point_cloud(str(args.run / "dense_map.ply"))
    else:
        geom = o3d.io.read_point_cloud(str(args.run / "sparse_map.ply"))
        P = np.asarray(geom.points)
        lo, hi = np.percentile(P, [1, 99], axis=0)               # drop far outliers
        geom = geom.select_by_index(np.where(((P >= lo) & (P <= hi)).all(1))[0])
        geom.paint_uniform_color((0.2, 0.4, 0.8))

    shown = [geom]
    if not args.no_traj:
        shown.append(trajectory_lineset(args.run / "CameraTrajectory.txt"))
    shown.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3))   # first camera pose

    print(f"{args.what}: {args.run}  (red = camera path, axes = first camera: x red, y green, z blue/forward)")
    o3d.visualization.draw_geometries(shown, window_name=f"vslam map - {args.what}",
                                      width=1400, height=900)


if __name__ == "__main__":
    main()
