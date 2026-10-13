"""Evaluate a SLAM run that may have split into several maps (tracking losses).

Each map has its own frame, so a single global ATE is meaningless after a loss.
This reports: number of tracking breaks, per-segment SE(3) ATE, the share of
the trajectory covered by the longest segment, and the global ATE (valid only
if there is one segment).

Usage:
    .venv/bin/python tools/eval_segments.py --run runs/sim_apt0_loop --gt data/sim_apt0_loop/groundtruth.txt
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate import umeyama  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--gt", required=True, type=Path)
    ap.add_argument("--jump", type=float, default=0.2, help="pose jump (m) treated as a map break")
    args = ap.parse_args()

    est = np.loadtxt(args.run / "CameraTrajectory.txt")
    gt = np.loadtxt(args.gt, comments="#")
    j = np.searchsorted(gt[:, 0], est[:, 0]).clip(1, len(gt) - 1)
    j = np.where(np.abs(gt[j - 1, 0] - est[:, 0]) < np.abs(gt[j, 0] - est[:, 0]), j - 1, j)
    G, E = gt[j, 1:4], est[:, 1:4]
    cuts = np.r_[0, np.where(np.linalg.norm(np.diff(E, axis=0), axis=1) > args.jump)[0] + 1, len(E)]

    total = np.linalg.norm(np.diff(G, axis=0), axis=1).sum()
    lines = [f"run: {args.run}", f"frames {len(E)}, GT path {total:.2f} m, tracking breaks: {len(cuts) - 2}"]
    longest = 0.0
    for a, b in zip(cuts[:-1], cuts[1:]):
        L = np.linalg.norm(np.diff(G[a:b], axis=0), axis=1).sum()
        longest = max(longest, L)
        if b - a < 20:
            lines.append(f"  segment {a:5d}-{b:5d}: {b - a} frames (too short to evaluate)")
            continue
        _, R, t = umeyama(E[a:b], G[a:b], with_scale=False)
        e = np.linalg.norm((R @ E[a:b].T).T + t - G[a:b], axis=1)
        rmse = np.sqrt((e ** 2).mean())
        lines.append(f"  segment {a:5d}-{b:5d}: {b - a:5d} frames, {L:5.2f} m, ATE {rmse * 100:5.2f} cm "
                     f"({rmse / max(L, 1e-9) * 100:.2f}% of distance)")
    lines.append(f"longest continuous segment: {longest:.2f} m ({longest / total * 100:.0f}% of the path)")
    if len(cuts) == 2:
        _, R, t = umeyama(E, G, with_scale=False)
        e = np.linalg.norm((R @ E.T).T + t - G, axis=1)
        lines.append(f"global ATE (single map): {np.sqrt((e ** 2).mean()) * 100:.2f} cm, max {e.max() * 100:.2f} cm")
    (args.run / "eval_segments.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
