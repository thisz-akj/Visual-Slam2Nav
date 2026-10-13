"""Front end: feature detection, matching, and two-view relative pose."""
import numpy as np
import cv2


def undistort_points(pts, K, dist, iters=30):
    """Invert the Brown-Conrady model by fixed-point iteration (portable, converged).

    Distortion maps ideal normalized coords x -> x_d = x * radial + tangential.
    We solve x = (x_d - tangential(x)) / radial(x), starting from x = x_d.
    OpenCV's default does only 5 iterations, too few for fr1's large k3.
    """
    k1, k2, p1, p2, k3 = dist
    xd = (pts[:, 0] - K[0, 2]) / K[0, 0]
    yd = (pts[:, 1] - K[1, 2]) / K[1, 1]
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return np.c_[x * K[0, 0] + K[0, 2], y * K[1, 1] + K[1, 2]].astype(np.float32)


class Frontend:
    def __init__(self, K, dist, n_features=2000):
        self.K, self.dist = K, dist
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def detect(self, img):
        """Returns undistorted keypoint pixels (N,2) and ORB descriptors (N,32)."""
        kps, des = self.orb.detectAndCompute(img, None)
        if des is None:
            return np.zeros((0, 2), np.float32), None
        pts = np.float32([k.pt for k in kps]).reshape(-1, 1, 2)
        # Undistort only the keypoints (cheaper than undistorting the image)
        pts_u = undistort_points(pts.reshape(-1, 2), self.K, self.dist)
        return pts_u, des

    def match(self, des1, des2, ratio=0.75):
        """Lowe ratio test + mutual check. Returns index arrays (i1, i2)."""
        if des1 is None or des2 is None:
            return np.array([], int), np.array([], int)
        fwd = self.matcher.knnMatch(des1, des2, k=2)
        bwd = {m.queryIdx: m.trainIdx for m in self.matcher.match(des2, des1)}
        i1, i2 = [], []
        for pair in fwd:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < ratio * n.distance and bwd.get(m.trainIdx) == m.queryIdx:
                i1.append(m.queryIdx)
                i2.append(m.trainIdx)
        return np.array(i1, int), np.array(i2, int)

    def relative_pose(self, p1, p2, thresh_px=1.0):
        """Essential matrix + cheirality check.

        Returns R, t (unit norm), inlier mask, with the convention
        X_cam2 = R @ X_cam1 + t.
        """
        E, mask = cv2.findEssentialMat(p1, p2, self.K, method=cv2.RANSAC,
                                       prob=0.999, threshold=thresh_px)
        if E is None or E.shape != (3, 3):
            return None, None, np.zeros(len(p1), bool)
        _, R, t, mask_pose = cv2.recoverPose(E, p1, p2, self.K, mask=mask.copy())
        return R, t.ravel(), mask_pose.ravel() > 0
