"""Run SuperPoint + LightGlue RGB-D SLAM on a TUM-format sequence.

Writes the same files as ORB-SLAM3 (CameraTrajectory.txt, KeyFrameTrajectory.txt, MapPoints.txt,
run.log) so tools/build_map.py and tools/eval_segments.py work unchanged.

    .venv/bin/python -m splg_slam.run --data DIR --assoc FILE --config ORBSLAM3.yaml --out runs/X [--stride 2]
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from build_map import load_camera  # noqa: E402

from .frontend import Frontend  # noqa: E402
from .slam import SLAM  # noqa: E402


def write_tum(path, rows):
    with open(path, "w") as f:
        for ts, T in rows:
            q = Rotation.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{ts:.6f} {T[0,3]:.9f} {T[1,3]:.9f} {T[2,3]:.9f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, type=Path)
    ap.add_argument("--assoc", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--stride", type=int, default=1, help="process every n-th frame")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--max-kp", type=int, default=1024)
    ap.add_argument("--resize", type=int, default=480, help="SuperPoint input size (longer side, px)")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    logf = open(args.out / "run.log", "w")

    def log(s):
        print(s, flush=True)
        logf.write(s + "\n")
        logf.flush()

    cam = load_camera(args.config)
    K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]])
    rows = [l.split() for l in Path(args.assoc).read_text().splitlines() if l.strip() and not l.startswith("#")]
    rows = rows[::args.stride]
    if args.max_frames:
        rows = rows[:args.max_frames]
    fe = Frontend(K, cam["depth_scale"], max_kp=args.max_kp, resize=args.resize)
    slam = SLAM(fe, K, log)
    log(f"SuperPoint+LightGlue RGB-D SLAM: {len(rows)} frames (stride {args.stride}), {args.data}")

    times = []
    t_all = time.time()
    for n, (t_rgb, f_rgb, _, f_d) in enumerate(rows):
        rgb = cv2.imread(str(args.data / f_rgb))[:, :, ::-1].copy()
        depth = cv2.imread(str(args.data / f_d), cv2.IMREAD_UNCHANGED)
        t0 = time.time()
        f = fe.extract(rgb, depth)
        slam.process(f, float(t_rgb))
        times.append(time.time() - t0)
        if n % 100 == 0:
            s = slam.stats
            log(f"[{n}/{len(rows)}] KFs {s['kfs']} maps {s['new_maps']} loops {s['loops']} merges {s['merges']} "
                f"lost {s['lost']}  {np.mean(times[-100:]) * 1000:.0f} ms/frame")

    write_tum(args.out / "CameraTrajectory.txt", slam.trajectory())
    write_tum(args.out / "KeyFrameTrajectory.txt", [(k.ts, k.T_wc) for k in slam.kfs])
    np.savetxt(args.out / "MapPoints.txt", slam.map_points(), fmt="%d %.6f %.6f %.6f %d")
    s = slam.stats
    log(f"done: {len(rows)} frames, tracked {s['tracked']}, lost {s['lost']}, relocalized {s['relocalized']}, "
        f"maps {s['new_maps']}, KFs {s['kfs']}, loops {s['loops']}, merges {s['merges']}, "
        f"LightGlue calls {fe.n_match_calls}")
    log(f"median tracking time: {np.median(times):.4f}")
    log(f"mean tracking time: {np.mean(times):.4f}")
    log(f"total {time.time() - t_all:.0f} s")


if __name__ == "__main__":
    main()
