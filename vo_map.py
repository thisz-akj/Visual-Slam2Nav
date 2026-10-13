"""Phase 2: map-based monocular visual odometry (no ground truth used for tracking).

Usage:
    python vo_map.py --data rgbd_dataset_freiburg1_xyz --show
"""
import argparse
import time
from pathlib import Path
import numpy as np
import cv2
import matplotlib
import matplotlib.pyplot as plt

from dataset import TUMDataset, rot_to_quat
from frontend import Frontend
from tracker import Tracker, Frame
from evaluate import ate


def draw(img, frame, tracker):
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for p, m in zip(frame.pts, frame.mp):
        color = (0, 255, 0) if m >= 0 else (90, 90, 90)
        cv2.circle(vis, (int(p[0]), int(p[1])), 2 if m < 0 else 3, color, -1)
    txt = (f"{tracker.state}  frame {frame.idx}  tracked {int((frame.mp >= 0).sum())}  "
           f"map {len(tracker.map.X)}  KFs {len(tracker.keyframes)}")
    cv2.putText(vis, txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--out", default="results_map")
    ap.add_argument("--init-parallax", type=float, default=20.0,
                    help="median pixel flow required before two-view initialization")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-ba", action="store_true", help="disable bundle adjustment (A/B test)")
    ap.add_argument("--sigma", type=float, default=1.5, help="keypoint noise in px; sets all gates")
    args = ap.parse_args()

    ds = TUMDataset(args.data)
    fe = Frontend(ds.K, ds.dist, n_features=2000)
    cv2.setRNGSeed(args.seed)              # RANSAC is random: make runs comparable
    tr = Tracker(ds.K)
    tr.init_min_parallax = args.init_parallax
    tr.use_ba = not args.no_ba
    tr.px_sigma = args.sigma
    out = Path(args.out)
    out.mkdir(exist_ok=True)

    n = len(ds) if args.max_frames <= 0 else min(len(ds), args.max_frames)
    t0 = time.time()
    for i in range(n):
        img = ds.image(i)
        pts, des = fe.detect(img)
        frame = Frame(i, ds.timestamp(i), pts, des)
        tr.process(frame)
        if tr.state == "LOST":
            print(f"Tracking lost at frame {i}")
            break
        if args.show:
            cv2.imshow("Phase 3", draw(img, frame, tr))
            if cv2.waitKey(1) == 27:
                break
    cv2.destroyAllWindows()
    dt = time.time() - t0

    # ---------------- evaluation (ground truth used ONLY here)
    stamps, est, gt = [], [], []
    for ts, T in tr.trajectory:
        j = np.argmin(np.abs(ds.gt_t - ts))
        if abs(ds.gt_t[j] - ts) < 0.02:
            stamps.append(ts)
            est.append(T[:3, 3])
            gt.append(ds.gt[j, :3])
    est, gt = np.array(est), np.array(gt)
    rmse, aligned, s = ate(est, gt)

    print(f"\nFrames processed: {n}  tracked: {len(tr.trajectory)}  keyframes: {len(tr.keyframes)}  "
          f"map points: {tr.map.n_good} good / {len(tr.map.X)} created")
    print(f"ATE RMSE (Sim3-aligned): {rmse * 100:.2f} cm   scale: {s:.3f}   "
          f"speed: {n / dt:.1f} fps")

    with open(out / "trajectory.txt", "w") as f:
        for ts, T in tr.trajectory:
            q = rot_to_quat(T[:3, :3])
            f.write(f"{ts:.6f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

    # keyframe trajectory AFTER bundle adjustment (what the back end believes now)
    kf_est, kf_gt = [], []
    with open(out / "kf_trajectory.txt", "w") as f:
        for kf in tr.keyframes:
            T = kf.T_wc
            q = rot_to_quat(T[:3, :3])
            f.write(f"{kf.ts:.6f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
            j = np.argmin(np.abs(ds.gt_t - kf.ts))
            if abs(ds.gt_t[j] - kf.ts) < 0.02:
                kf_est.append(T[:3, 3]); kf_gt.append(ds.gt[j, :3])
    if len(kf_est) > 3:
        kf_rmse, _, _ = ate(np.array(kf_est), np.array(kf_gt))
        print(f"Keyframe ATE after BA: {kf_rmse * 100:.2f} cm   ({len(kf_est)} keyframes)")

    import csv
    with open(out / "log.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(tr.log[0].keys()))
        w.writeheader()
        w.writerows(tr.log)
    paths = [r["path"] for r in tr.log]
    print("Tracking paths: " + "  ".join(f"{p}={paths.count(p)}" for p in sorted(set(paths))))

    # map points in the ground-truth frame, using the same Sim3 alignment
    from evaluate import umeyama
    s_, R_, t_ = umeyama(est, gt)
    M = (s_ * (R_ @ tr.map.X[~tr.map.bad].T)).T + t_
    near = np.linalg.norm(M - gt.mean(0), axis=1) < 4.0          # drop far outliers for display

    fig = plt.figure(figsize=(16, 5))
    for k, (a, b, name) in enumerate([(0, 1, "x-y"), (0, 2, "x-z"), (1, 2, "y-z")]):
        ax = fig.add_subplot(1, 4, k + 1)
        ax.plot(gt[:, a], gt[:, b], "k-", label="ground truth")
        ax.plot(aligned[:, a], aligned[:, b], "r-", lw=1, label="estimate")
        ax.set_title(name); ax.axis("equal"); ax.grid(alpha=0.3)
    fig.axes[0].legend()
    ax = fig.add_subplot(1, 4, 4, projection="3d")
    ax.scatter(*M[near].T, s=0.3, c="steelblue")
    ax.plot(*aligned.T, "r-")
    ax.set_title("map + trajectory")
    fig.suptitle(f"Phase 3 mono VO + local BA  |  ATE {rmse*100:.2f} cm")
    fig.tight_layout()
    fig.savefig(out / "trajectory.png", dpi=120)
    print(f"Saved {out/'trajectory.png'} and {out/'trajectory.txt'}")
    if matplotlib.get_backend().lower() != "agg":
        plt.show()


if __name__ == "__main__":
    main()
