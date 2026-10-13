"""TUM RGB-D dataset loader: images, timestamps, ground-truth poses."""
from pathlib import Path
import numpy as np
import cv2

# Freiburg1 calibration (from the TUM website)
K_FR1 = np.array([[517.3, 0.0, 318.6],
                  [0.0, 516.5, 255.3],
                  [0.0, 0.0, 1.0]])
DIST_FR1 = np.array([0.2624, -0.9531, -0.0054, 0.0026, 1.1633])


def quat_to_rot(qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz]) / np.linalg.norm([qw, qx, qy, qz])
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    """Returns (qx, qy, qz, qw)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw, qx = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        qy, qz = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw, qx = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        qy, qz = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw, qx = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        qy, qz = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw, qx = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        qy, qz = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return qx, qy, qz, qw


def _read_list(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            rows.append(line.split())
    return rows


class TUMDataset:
    def __init__(self, root, K=K_FR1, dist=DIST_FR1, max_dt=0.02):
        self.root = Path(root)
        self.K, self.dist, self.max_dt = K, dist, max_dt
        self.frames = [(float(t), f) for t, f in _read_list(self.root / "rgb.txt")]
        gt = np.array(_read_list(self.root / "groundtruth.txt"), dtype=float)
        self.gt_t, self.gt = gt[:, 0], gt[:, 1:]

    def __len__(self):
        return len(self.frames)

    def timestamp(self, i):
        return self.frames[i][0]

    def image(self, i, gray=True):
        flag = cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR
        return cv2.imread(str(self.root / self.frames[i][1]), flag)

    def gt_pose(self, i):
        """4x4 camera-to-world pose T_wc nearest in time, or None if too far."""
        j = np.argmin(np.abs(self.gt_t - self.timestamp(i)))
        if abs(self.gt_t[j] - self.timestamp(i)) > self.max_dt:
            return None
        tx, ty, tz, qx, qy, qz, qw = self.gt[j]
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
        T[:3, 3] = [tx, ty, tz]
        return T
