"""Turn an ORB-SLAM3 RGB-D run into maps + an accuracy check.

Inputs (from the ORB-SLAM3 run directory): CameraTrajectory.txt, MapPoints.txt
Outputs: sparse_map.ply, dense_map.ply (TSDF-fused coloured cloud), dense_mesh.ply,
         map_topdown.png, eval.txt

Usage:
    .venv/bin/python tools/build_map.py --data data/rgbd_dataset_freiburg3_long_office_household \
        --run runs/fr3_office --assoc third_party/ORB_SLAM3/Examples/RGB-D/associations/fr3_office.txt
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate import umeyama  # noqa: E402



def load_camera(cfg):
    """Read intrinsics + depth factor from an ORB-SLAM3 yaml."""
    import cv2
    fs = cv2.FileStorage(str(cfg), cv2.FILE_STORAGE_READ)
    g = lambda k: fs.getNode(k).real()
    return dict(fx=g("Camera1.fx"), fy=g("Camera1.fy"), cx=g("Camera1.cx"), cy=g("Camera1.cy"),
                depth_scale=g("RGBD.DepthMapFactor"))


def load_tum(path):
    """TUM format 'ts tx ty tz qx qy qz qw' -> (ts (N,), T_wc (N,4,4))."""
    a = np.loadtxt(path, comments="#")
    T = np.tile(np.eye(4), (len(a), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(a[:, 4:8]).as_matrix()
    T[:, :3, 3] = a[:, 1:4]
    return a[:, 0], T


def associate(ts_a, ts_b, max_dt=0.02):
    j = np.clip(np.searchsorted(ts_b, ts_a), 1, len(ts_b) - 1)
    j = np.where(np.abs(ts_b[j - 1] - ts_a) < np.abs(ts_b[j] - ts_a), j - 1, j)
    ok = np.abs(ts_b[j] - ts_a) < max_dt
    return np.where(ok)[0], j[ok]


def evaluate(ts, T, gt_path, out):
    gts, gT = load_tum(gt_path)
    i, j = associate(ts, gts)
    est, gt = T[i, :3, 3], gT[j, :3, 3]
    lines = []
    for name, scale in [("SE(3) (metric, RGB-D)", False), ("Sim(3)", True)]:
        s, R, t = umeyama(est, gt, with_scale=scale)
        err = np.linalg.norm((s * (R @ est.T)).T + t - gt, axis=1)
        lines.append(f"ATE {name}: RMSE {np.sqrt((err**2).mean())*100:.2f} cm, "
                     f"median {np.median(err)*100:.2f} cm, max {err.max()*100:.2f} cm, scale {s:.4f}")
    path_len = np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()
    lines.append(f"frames evaluated: {len(i)}   GT path length: {path_len:.2f} m")
    (out / "eval.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    s, R, t = umeyama(est, gt, with_scale=False)
    return (R @ est.T).T + t, gt


def fuse(data, assoc, ts, T, out, every, voxel, cam, depth_max):
    """TSDF fusion with Open3D's tensor VoxelBlockGrid.

    (The legacy ScalableTSDFVolume returns empty clouds in Open3D 0.20.)
    """
    import open3d.core as o3c
    K = o3c.Tensor([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]], o3c.float64)
    vbg = o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"), attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
        attr_channels=((1), (1), (3)), voxel_size=voxel, block_resolution=8,
        block_count=200000, device=o3c.Device("CPU:0"))
    rows = [l.split() for l in Path(assoc).read_text().splitlines() if l.strip() and not l.startswith("#")]
    pose = {round(t, 6): Tk for t, Tk in zip(ts, T)}
    used = 0
    for k, (t_rgb, f_rgb, _, f_d) in enumerate(rows):
        Tk = pose.get(round(float(t_rgb), 6))
        if Tk is None or k % every:
            continue
        depth = o3d.t.io.read_image(str(data / f_d))
        color = o3d.t.io.read_image(str(data / f_rgb))
        E = o3c.Tensor(np.linalg.inv(Tk))                   # extrinsic = T_cw
        blk = vbg.compute_unique_block_coordinates(depth, K, E, cam["depth_scale"], depth_max)
        vbg.integrate(blk, depth, color, K, E, cam["depth_scale"], depth_max)
        used += 1
        if used % 100 == 0:
            print(f"  fused {used} frames")
    print(f"fused {used} frames")
    pcd = vbg.extract_point_cloud(weight_threshold=3.0).to_legacy()
    mesh = vbg.extract_triangle_mesh(weight_threshold=3.0).to_legacy()
    mesh.compute_vertex_normals()
    # TSDF colour averaging can overshoot 1.0 slightly
    pcd.colors = o3d.utility.Vector3dVector(np.clip(np.asarray(pcd.colors), 0, 1))
    mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(np.asarray(mesh.vertex_colors), 0, 1))
    o3d.io.write_point_cloud(str(out / "dense_map.ply"), pcd)
    o3d.io.write_triangle_mesh(str(out / "dense_mesh.ply"), mesh)
    print(f"dense map: {len(pcd.points):,} points, mesh {len(mesh.triangles):,} triangles")
    return pcd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, type=Path)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--assoc", required=True)
    ap.add_argument("--every", type=int, default=3, help="fuse every k-th tracked frame")
    ap.add_argument("--voxel", type=float, default=0.01, help="TSDF voxel size (m)")
    ap.add_argument("--config", default="third_party/ORB_SLAM3/Examples/RGB-D/TUM3.yaml",
                    help="ORB-SLAM3 yaml with the camera intrinsics and depth factor")
    ap.add_argument("--depth-max", type=float, default=3.0, help="ignore depth beyond this (m)")
    args = ap.parse_args()
    out = args.run

    ts, T = load_tum(out / "CameraTrajectory.txt")
    print(f"trajectory: {len(ts)} frames")
    est_aligned, gt = evaluate(ts, T, args.data / "groundtruth.txt", out)

    mp = np.loadtxt(out / "MapPoints.txt")
    sparse = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(mp[:, 1:4]))
    o3d.io.write_point_cloud(str(out / "sparse_map.ply"), sparse)
    print(f"sparse map: {len(mp):,} ORB map points in {len(np.unique(mp[:, 0]))} map(s)")

    pcd = fuse(args.data, args.assoc, ts, T, out, args.every, args.voxel,
               load_camera(args.config), args.depth_max)

    # top-down preview (TUM world: z is up for fr3 mocap; pick the two widest axes)
    P, C = np.asarray(pcd.points), np.asarray(pcd.colors)
    sel = np.random.default_rng(0).choice(len(P), min(len(P), 400_000), replace=False)
    ax_ = np.argsort(np.ptp(T[:, :3, 3], axis=0))[::-1][:2]
    fig, axs = plt.subplots(1, 2, figsize=(18, 8))
    axs[0].scatter(P[sel, ax_[0]], P[sel, ax_[1]], c=C[sel], s=0.2)
    axs[0].plot(T[:, ax_[0], 3], T[:, ax_[1], 3], "r-", lw=1)
    axs[0].set_title("Dense map (TSDF-fused) + ORB-SLAM3 trajectory")
    axs[1].scatter(mp[:, 1 + ax_[0]], mp[:, 1 + ax_[1]], s=0.3, c="steelblue")
    axs[1].plot(T[:, ax_[0], 3], T[:, ax_[1], 3], "r-", lw=1, label="estimate")
    axs[1].set_title("Sparse ORB map points (1-99th percentile extent)")
    lo, hi = np.percentile(mp[:, 1:4], [1, 99], axis=0)
    axs[1].set_xlim(lo[ax_[0]], hi[ax_[0]])
    axs[1].set_ylim(lo[ax_[1]], hi[ax_[1]])
    for a in axs:
        a.set_aspect("equal"); a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "map_topdown.png", dpi=110)

    fig, a = plt.subplots(figsize=(8, 8))
    a.plot(gt[:, 0], gt[:, 1], "k-", label="ground truth")
    a.plot(est_aligned[:, 0], est_aligned[:, 1], "r-", lw=1, label="ORB-SLAM3 (SE3-aligned)")
    a.set_aspect("equal"); a.grid(alpha=0.3); a.legend()
    fig.savefig(out / "trajectory_vs_gt.png", dpi=110)
    print(f"wrote outputs to {out}/")


if __name__ == "__main__":
    main()
