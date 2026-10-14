"""Compare ORB-SLAM3 and SuperPoint+LightGlue runs from tools/compare_all.sh.

Writes results/COMPARISON.md and docs/figures/cmp_*.png.

    .venv/bin/python tools/compare_report.py
"""
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from evaluate import umeyama  # noqa: E402

CMP = ROOT / "runs" / "cmp"
FIG = ROOT / "docs" / "figures"
DATASETS = [  # name, label, ground truth, ground-truth kind
    ("fr3_office", "TUM fr3 office (real)", "data/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt", "motion capture"),
    ("sim_A", "Twin run A: cam 0.37 m, 2 cm", "data/sim_apt0_loop/groundtruth.txt", "exact (Gazebo)"),
    ("sim_B", "Twin run B: + slow turns", "data/sim_apt0_slowturn/groundtruth.txt", "exact (Gazebo)"),
    ("sim_D", "Twin run D: cam 1.2 m, 2 cm", "data/sim_apt0_tall/groundtruth.txt", "exact (Gazebo)"),
    ("sim_C", "Twin run C: cam 1.2 m, HD", "data/sim_apt0hd_tall/groundtruth.txt", "exact (Gazebo)"),
    ("apt0", "BundleFusion apt0 (real)", "data/apt0/apt0/groundtruth.txt", "reference poses"),
]
SHORT = {"fr3_office": "TUM office\n(real)", "sim_A": "Twin A\n0.37 m cam", "sim_B": "Twin B\nslow turns",
         "sim_D": "Twin D\n1.2 m cam", "sim_C": "Twin C\n1.2 m + HD", "apt0": "apt0\n(real)"}
PIPES = [("orbslam3", "ORB-SLAM3", "#9aa3ad"), ("splg", "SuperPoint+LightGlue", "#2f6fdf")]
plt.rcParams.update({"font.family": ["Noto Sans", "DejaVu Sans"], "font.size": 10})


def load(path):
    return np.loadtxt(path, comments="#")


def evaluate(run, gt_path, n_frames):
    est = load(run / "CameraTrajectory.txt")
    gt = load(ROOT / gt_path)
    j = np.searchsorted(gt[:, 0], est[:, 0]).clip(1, len(gt) - 1)
    j = np.where(np.abs(gt[j - 1, 0] - est[:, 0]) < np.abs(gt[j, 0] - est[:, 0]), j - 1, j)
    E, G = est[:, 1:4], gt[j, 1:4]
    cuts = np.r_[0, np.where(np.linalg.norm(np.diff(E, axis=0), axis=1) > 0.2)[0] + 1, len(E)]
    total = np.linalg.norm(np.diff(G, axis=0), axis=1).sum()
    segs, longest = [], 0.0
    for a, b in zip(cuts[:-1], cuts[1:]):
        L = np.linalg.norm(np.diff(G[a:b], axis=0), axis=1).sum()
        longest = max(longest, L)
        if b - a >= 20:
            _, R, t = umeyama(E[a:b], G[a:b], with_scale=False)
            e = np.linalg.norm((R @ E[a:b].T).T + t - G[a:b], axis=1)
            segs.append((b - a, np.sqrt((e ** 2).mean())))
    out = {"tracked": len(E) / n_frames, "breaks": len(cuts) - 2, "longest": longest / total, "path": total,
           "seg_ate": np.average([s[1] for s in segs], weights=[s[0] for s in segs]) if segs else np.nan,
           "ate": np.nan, "max": np.nan, "aligned": None, "gt": G}
    if len(cuts) == 2:
        _, R, t = umeyama(E, G, with_scale=False)
        A = (R @ E.T).T + t
        e = np.linalg.norm(A - G, axis=1)
        out.update(ate=np.sqrt((e ** 2).mean()), max=e.max(), aligned=A)
    return out


def log_stats(run):
    txt = (run / "run.log").read_text(errors="ignore")
    m = re.search(r"median tracking time: ([\d.]+)", txt)
    loops = len(re.findall(r"Loop detected", txt))
    merges = len(re.findall(r"Merge detected", txt))
    maps = len(re.findall(r"New Map created|New map \d", txt))
    if "LightGlue calls" in txt:                      # our pipeline counts maps via "New map"
        maps = len(re.findall(r"New map \d", txt))
    return {"ms": float(m.group(1)) * 1000 if m else np.nan, "loops": loops, "merges": merges,
            "maps": max(maps, 1)}


def fmt_cm(v):
    return "-" if np.isnan(v) else f"{v * 100:.2f}"


def main():
    rows, res = [], {}
    for name, label, gt, kind in DATASETS:
        if not (CMP / "assoc" / f"{name}.txt").exists():
            continue
        n = sum(1 for _ in open(CMP / "assoc" / f"{name}.txt"))
        for key, plabel, _ in PIPES:
            run = CMP / f"{key}_{name}"
            if not (run / "CameraTrajectory.txt").exists():
                continue
            r = evaluate(run, gt, n) | log_stats(run)
            res[(name, key)] = r
            rows.append(f"| {label} | {plabel} | {n} | {r['tracked'] * 100:.0f}% | {r['breaks']} | "
                        f"{r['longest'] * 100:.0f}% | {fmt_cm(r['ate'])} | {fmt_cm(r['max'])} | {fmt_cm(r['seg_ate'])} | "
                        f"{r['loops']} | {r['merges']} | {r['ms']:.0f} |")

    md = ["# ORB-SLAM3 vs SuperPoint + LightGlue",
          "",
          "Both pipelines see exactly the same frames (`runs/cmp/assoc/`): every 2nd frame of the 30 Hz real sequences, "
          "every frame of the 15 Hz twin recordings. Both use RGB-D, keyframes, PnP tracking, loop closure and "
          "pose-graph optimization, so the main difference is the features: hand-crafted ORB + bag of words vs "
          "learned SuperPoint + LightGlue. Errors are SE(3)-aligned ATE in cm.",
          "",
          "| Dataset | Pipeline | Frames | Tracked | Breaks | Longest continuous | ATE | Max | ATE per segment | Loops | Merges | ms / frame |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|",
          *rows,
          "",
          "- **Tracked**: share of frames with a pose. **Breaks**: jumps > 0.2 m, i.e. a new map was started.",
          "- **ATE**: only when the whole run is one map. **ATE per segment**: frame-weighted mean over segments.",
          "- **ms / frame**: median tracking time on this CPU-only laptop (Intel Core Ultra 5 225H). ORB-SLAM3 runs "
          "mapping and loop closing in other threads; our pipeline runs everything in one thread.",
          "- apt0 errors are against BundleFusion's reference poses, not true ground truth.",
          ""]
    (ROOT / "results" / "comparison_table.md").write_text("\n".join(md))   # COMPARISON.md = written analysis
    print("\n".join(md))

    # ---- chart: longest continuous tracking + ATE per dataset
    names = [d for d in DATASETS if all((d[0], p[0]) in res for p in PIPES)]
    if names:
        x = np.arange(len(names))
        fig, axs = plt.subplots(1, 2, figsize=(12, 3.8))
        for k, (key, plabel, col) in enumerate(PIPES):
            lon = [res[(d[0], key)]["longest"] * 100 for d in names]
            ate = [res[(d[0], key)]["ate"] * 100 if not np.isnan(res[(d[0], key)]["ate"])
                   else res[(d[0], key)]["seg_ate"] * 100 for d in names]
            axs[0].bar(x + (k - 0.5) * 0.38, lon, 0.38, color=col, label=plabel)
            bars = axs[1].bar(x + (k - 0.5) * 0.38, ate, 0.38, color=col, label=plabel)
            for i, (b, d) in enumerate(zip(bars, names)):
                if np.isnan(res[(d[0], key)]["ate"]):
                    b.set_hatch("///")
                    b.set_alpha(0.6)
        for a in axs:
            a.set_xticks(x)
            a.set_xticklabels([SHORT[d[0]] for d in names], fontsize=8.5)
            a.spines[["top", "right"]].set_visible(False)
        axs[0].set_title("Longest continuous tracking (% of path, higher is better)")
        axs[0].set_ylim(0, 110)
        axs[1].set_title("Position error, cm (lower is better; hatched = several maps,\nmean of segments)")
        axs[0].legend(frameon=False, fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
        fig.tight_layout()
        fig.savefig(FIG / "cmp_summary.png", dpi=200, facecolor="white", bbox_inches="tight")
        plt.close(fig)
        print("wrote cmp_summary.png")

    # ---- trajectory overlays
    for name in ["sim_A", "fr3_office"]:
        if not all((name, p[0]) in res for p in PIPES):
            continue
        fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
        for a, (key, plabel, col) in zip(axs, PIPES):
            r = res[(name, key)]
            G = r["gt"]
            a.plot(G[:, 0], G[:, 1], color="black", lw=2, label="ground truth")
            if r["aligned"] is not None:
                a.plot(r["aligned"][:, 0], r["aligned"][:, 1], color=col if key == "splg" else "#d9822b", lw=1.3,
                       label=f"{plabel}: one map, {r['ate'] * 100:.1f} cm")
            else:
                est = load(CMP / f"{key}_{name}" / "CameraTrajectory.txt")[:, 1:4]
                cuts = np.r_[0, np.where(np.linalg.norm(np.diff(est, axis=0), axis=1) > 0.2)[0] + 1, len(est)]
                for s, (p, q) in enumerate(zip(cuts[:-1], cuts[1:])):
                    if q - p < 20:
                        continue
                    _, R, t = umeyama(est[p:q], G[p:q], with_scale=False)
                    A = (R @ est[p:q].T).T + t
                    a.plot(A[:, 0], A[:, 1], lw=1.6, label=f"{plabel}: map piece {s + 1}" if s < 6 else None)
                for c in cuts[1:-1]:
                    a.plot(*G[c, :2], "x", color="#c94a4a", ms=9, mew=2)
            a.set_aspect("equal")
            a.set_title(f"{plabel}: {r['breaks']} tracking breaks, longest {r['longest'] * 100:.0f}%", fontsize=10)
            a.legend(fontsize=7.5, frameon=False, loc="best")
            a.set_xlabel("x (m)")
        fig.suptitle(dict((d[0], d[1]) for d in DATASETS)[name], fontsize=11)
        fig.tight_layout()
        fig.savefig(FIG / f"cmp_traj_{name}.png", dpi=200, facecolor="white", bbox_inches="tight")
        plt.close(fig)
        print(f"wrote cmp_traj_{name}.png")


if __name__ == "__main__":
    main()
