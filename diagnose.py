"""Diagnose a trajectory: is the error from scale drift, rotation drift, or initialization?

Usage:
    python diagnose.py --data rgbd_dataset_freiburg1_xyz --traj results_map/trajectory.txt
"""
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from dataset import quat_to_rot
from evaluate import ate


def load_tum(path):
    rows = [l.split() for l in open(path) if l.strip() and not l.startswith("#")]
    a = np.array(rows, dtype=float)
    Rs = np.array([quat_to_rot(*r[4:8]) for r in a])
    return a[:, 0], a[:, 1:4], Rs


def rot_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def rot_axis_angle(R):
    """Rotation vector (axis * angle in rad)."""
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w / (2 * np.sin(th)) * th


def estimate_frame_offset(dRe_list, dRg_list):
    """Find fixed rotation X with dRe = X^T dRg X by aligning rotation axes (Kabsch).

    If our camera frame and the ground-truth body frame differ, X is that difference.
    """
    a = np.array([rot_axis_angle(R) for R in dRg_list])   # gt axes
    b = np.array([rot_axis_angle(R) for R in dRe_list])   # est axes, = X^T a
    w = np.linalg.norm(a, axis=1) > np.radians(2)          # only informative windows
    if w.sum() < 5:
        return np.eye(3)
    H = b[w].T @ a[w]
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return (U @ D @ Vt).T         # X such that b ~= X^T a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--traj", required=True)
    ap.add_argument("--delta", type=float, default=1.0, help="window in seconds")
    ap.add_argument("--out", default=None)
    ap.add_argument("--log", default=None, help="log.csv from vo_map.py (adds a tracker panel)")
    args = ap.parse_args()

    te, pe, Re = load_tum(args.traj)
    tg, pg, Rg = load_tum(f"{args.data}/groundtruth.txt")

    # associate
    idx = np.array([np.argmin(np.abs(tg - t)) for t in te])
    ok = np.abs(tg[idx] - te) < 0.02
    te, pe, Re = te[ok], pe[ok], Re[ok]
    pg, Rg = pg[idx[ok]], Rg[idx[ok]]
    t = te - te[0]

    # 1. ATE on growing prefixes -> where does error appear?
    print("ATE on growing prefixes (Sim3 aligned per prefix):")
    for frac in [0.1, 0.25, 0.5, 0.75, 1.0]:
        n = max(10, int(len(pe) * frac))
        r, _, s = ate(pe[:n], pg[:n])
        print(f"  first {t[n-1]:5.1f}s ({n:4d} frames): ATE {r*100:6.2f} cm   scale {s:.3f}")

    # 2. windowed relative rotation error and local scale ratio
    rot_err, scale, tw, mag_err, gt_rot = [], [], [], [], []
    dRes, dRgs = [], []
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + args.delta)
        if j >= len(t):
            break
        dRe = Re[i].T @ Re[j]
        dRg = Rg[i].T @ Rg[j]
        dRes.append(dRe); dRgs.append(dRg)
        rot_err.append(rot_angle_deg(dRg.T @ dRe))
        mag_err.append(abs(rot_angle_deg(dRe) - rot_angle_deg(dRg)))   # convention-free
        gt_rot.append(rot_angle_deg(dRg))
        dg = np.linalg.norm(pg[j] - pg[i])
        de = np.linalg.norm(pe[j] - pe[i])
        scale.append(de / dg if dg > 0.05 else np.nan)   # only where real motion happened
        tw.append(t[i])
    rot_err, scale, tw = map(np.array, (rot_err, scale, tw))
    mag_err, gt_rot = np.array(mag_err), np.array(gt_rot)
    valid = np.isfinite(scale)
    print(f"\nTrue rotation per {args.delta}s window (ground truth): mean {gt_rot.mean():.2f} deg")
    print(f"Relative rotation error (assumes same body frame): "
          f"mean {rot_err.mean():.2f} deg, max {rot_err.max():.2f} deg")
    print(f"Rotation ANGLE error (frame-convention free):      "
          f"mean {mag_err.mean():.2f} deg, max {mag_err.max():.2f} deg")
    X = estimate_frame_offset(dRes, dRgs)
    rot_err_x = np.array([rot_angle_deg((X.T @ g @ X).T @ e) for e, g in zip(dRes, dRgs)])
    print(f"Estimated fixed offset between camera frame and GT frame: {rot_angle_deg(X):.1f} deg")
    print(f"Relative rotation error after removing that offset:  "
          f"mean {rot_err_x.mean():.2f} deg, max {rot_err_x.max():.2f} deg")
    print("  -> offset near 0 deg: frames agree, rotation errors are real."
          "\n  -> large offset: the GT body frame differs; trust the corrected numbers.")
    rot_err = rot_err_x
    if valid.sum() > 2:
        print(f"Local scale (est/gt): first quarter {np.nanmedian(scale[:len(scale)//4]):.3f}, "
              f"last quarter {np.nanmedian(scale[-len(scale)//4:]):.3f}, "
              f"spread (p90/p10) {np.nanpercentile(scale, 90) / np.nanpercentile(scale, 10):.2f}x")

    _, aligned, _ = ate(pe, pg)
    nrow = 4 if args.log else 3
    fig, ax = plt.subplots(nrow, 1, figsize=(11, 3 * nrow), sharex=True)
    for k, c in enumerate("xyz"):
        ax[0].plot(t, pg[:, k], "k-", lw=1)
        ax[0].plot(t, aligned[:, k], "--", lw=1, label=f"{c} est")
    ax[0].set_ylabel("position (m)"); ax[0].legend(ncol=3); ax[0].set_title("aligned position vs ground truth (black)")
    ax[1].plot(tw, scale, "b.", ms=3)
    ax[1].set_ylabel("local scale est/gt"); ax[1].set_title("flat line = no scale drift")
    ax[2].plot(tw, rot_err, "r.", ms=3)
    ax[2].set_ylabel("rot err (deg)"); ax[2].set_xlabel("time (s)")
    ax[2].set_title(f"relative rotation error over {args.delta}s windows")
    if args.log:
        import csv
        rows = list(csv.DictReader(open(args.log)))
        lt = np.array([float(r["ts"]) for r in rows]) - te[0]
        trk = np.array([int(r["tracked"]) for r in rows])
        path = np.array([r["path"] for r in rows])
        ax[3].plot(lt, trk, "k-", lw=0.8, label="tracked points")
        for p, c in [("wide", "orange"), ("desc", "red"), ("lost", "magenta")]:
            m = path == p
            if m.any():
                ax[3].plot(lt[m], np.zeros(m.sum()), "|", color=c, ms=12, label=p)
        kf = np.array([int(r["kf"]) for r in rows]) == 1
        ax[3].plot(lt[kf], trk[kf], "g^", ms=4, label="keyframe")
        ax[3].set_ylabel("tracked"); ax[3].legend(ncol=5, fontsize=8)
        ax[3].set_title("tracker: orange=wide search, red=descriptor fallback, magenta=lost")
        ax[3].set_xlabel("time (s)")
        bad = np.where(rot_err > 5)[0]
        if len(bad):
            print(f"First window with rotation error > 5 deg starts at t = {tw[bad[0]]:.1f}s")
        for p in ["proj", "wide", "desc", "lost"]:
            print(f"  frames on path '{p}': {(path == p).sum()}")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    out = args.out or args.traj.replace(".txt", "_diagnose.png")
    fig.savefig(out, dpi=110)
    print(f"\nSaved {out}")
    if matplotlib.get_backend().lower() != "agg":
        plt.show()


if __name__ == "__main__":
    main()
