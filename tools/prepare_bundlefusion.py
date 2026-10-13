"""Convert a Stanford BundleFusion sequence (apt0, office3, ...) to what ORB-SLAM3's rgbd_tum expects.

Writes into the sequence folder:
    associations.txt   "t rgb_file t depth_file" per frame (timestamps synthesised at 30 Hz)
    groundtruth.txt    TUM format, from BundleFusion's own poses (a reference, not mocap GT)
and next to it:
    <name>.yaml        ORB-SLAM3 camera config built from info.txt

Usage:
    .venv/bin/python tools/prepare_bundlefusion.py data/apt0/apt0
"""
import re
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

FPS = 30.0

YAML = """%YAML:1.0
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
Camera.width: {w}
Camera.height: {h}
Camera.fps: 30
Camera.RGB: 0
Stereo.ThDepth: 40.0
Stereo.b: 0.07
RGBD.DepthMapFactor: {depth_shift}
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


def main():
    seq = Path(sys.argv[1]).resolve()
    info = dict(re.findall(r"([\w.]+) = (.*)", (seq / "info.txt").read_text()))
    Kc = np.array(info["m_calibrationColorIntrinsic"].split(), float).reshape(4, 4)
    n = int(info["m_frames.size"])

    assoc, gt, bad = [], [], 0
    for i in range(n):
        t = f"{i / FPS:.6f}"
        assoc.append(f"{t} frame-{i:06d}.color.jpg {t} frame-{i:06d}.depth.png")
        T = np.loadtxt(seq / f"frame-{i:06d}.pose.txt")     # camera-to-world
        if not np.isfinite(T).all():
            bad += 1
            continue
        q = Rotation.from_matrix(T[:3, :3]).as_quat()       # x y z w
        gt.append(f"{t} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")

    (seq / "associations.txt").write_text("\n".join(assoc) + "\n")
    (seq / "groundtruth.txt").write_text("# BundleFusion reference poses\n" + "\n".join(gt) + "\n")
    cfg = seq.parent / f"{seq.name}.yaml"
    cfg.write_text(YAML.format(fx=Kc[0, 0], fy=Kc[1, 1], cx=Kc[0, 2], cy=Kc[1, 2],
                               w=info["m_colorWidth"], h=info["m_colorHeight"],
                               depth_shift=float(info["m_depthShift"])))
    print(f"{n} frames ({n / FPS / 60:.1f} min at 30 Hz), {bad} without reference pose")
    print(f"wrote {seq/'associations.txt'}\n      {seq/'groundtruth.txt'}\n      {cfg}")


if __name__ == "__main__":
    main()
