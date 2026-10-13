"""Phase 2: map-based monocular tracking.

INIT  : two-view initialization (essential matrix + triangulation), scale = median depth 1
OK    : constant-velocity prediction -> search by projection -> PnP RANSAC -> refine
KF    : insert keyframe when tracking weakens, triangulate new map points
"""
import numpy as np
import cv2
from scipy.spatial import cKDTree

from ba import bundle_adjust


# ---------------------------------------------------------------- geometry
def inv_T(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def project(K, T_cw, X):
    Xc = X @ T_cw[:3, :3].T + T_cw[:3, 3]
    z = Xc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = Xc[:, :2] / z[:, None] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    return uv, z


def triangulate(K, T_cw1, T_cw2, p1, p2):
    Xh = cv2.triangulatePoints(K @ T_cw1[:3], K @ T_cw2[:3],
                               p1.T.astype(np.float64), p2.T.astype(np.float64))
    with np.errstate(divide="ignore", invalid="ignore"):
        return (Xh[:3] / Xh[3]).T


def good_triangulation(K, T_cw1, T_cw2, X, p1, p2, max_err=2.0, min_angle_deg=1.0):
    """Cheirality + reprojection error + parallax angle."""
    uv1, z1 = project(K, T_cw1, X)
    uv2, z2 = project(K, T_cw2, X)
    e1 = np.linalg.norm(uv1 - p1, axis=1)
    e2 = np.linalg.norm(uv2 - p2, axis=1)
    r1 = X - inv_T(T_cw1)[:3, 3]
    r2 = X - inv_T(T_cw2)[:3, 3]
    cos = (r1 * r2).sum(1) / (np.linalg.norm(r1, axis=1) * np.linalg.norm(r2, axis=1) + 1e-12)
    ok = np.isfinite(X).all(1) & (z1 > 0) & (z2 > 0) & (e1 < max_err) & (e2 < max_err)
    return ok & (cos < np.cos(np.deg2rad(min_angle_deg)))


def sampson_ok(K, T_cw1, T_cw2, p1, p2, thresh_px2=4.0):
    """Epipolar check using the fundamental matrix implied by the two known poses."""
    T21 = T_cw2 @ inv_T(T_cw1)
    t = T21[:3, 3]
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    Ki = np.linalg.inv(K)
    F = Ki.T @ tx @ T21[:3, :3] @ Ki
    x1 = np.c_[p1, np.ones(len(p1))]
    x2 = np.c_[p2, np.ones(len(p2))]
    Fx1, Ftx2 = x1 @ F.T, x2 @ F
    num = (x2 * Fx1).sum(1) ** 2
    den = Fx1[:, 0] ** 2 + Fx1[:, 1] ** 2 + Ftx2[:, 0] ** 2 + Ftx2[:, 1] ** 2
    return num / (den + 1e-12) < thresh_px2


def hamming_pairs(da, db):
    return np.unpackbits(np.bitwise_xor(da, db), axis=1).sum(1)


def match_descriptors(matcher, d1, d2, ratio=0.75):
    if d1 is None or d2 is None or len(d1) < 2 or len(d2) < 2:
        return np.array([], int), np.array([], int)
    bwd = {m.queryIdx: m.trainIdx for m in matcher.match(d2, d1)}
    i1, i2 = [], []
    for pair in matcher.knnMatch(d1, d2, k=2):
        if len(pair) == 2:
            m, n = pair
            if m.distance < ratio * n.distance and bwd.get(m.trainIdx) == m.queryIdx:
                i1.append(m.queryIdx)
                i2.append(m.trainIdx)
    return np.array(i1, int), np.array(i2, int)


# ---------------------------------------------------------------- data structures
class Frame:
    def __init__(self, idx, ts, pts, des):
        self.idx, self.ts = idx, ts
        self.pts = pts.astype(np.float64)
        self.des = des
        self.mp = -np.ones(len(pts), int)      # keypoint -> map point id (-1 = none)
        self.T_cw = None

    @property
    def T_wc(self):
        return inv_T(self.T_cw)


class Map:
    def __init__(self):
        self.X = np.zeros((0, 3))
        self.des = np.zeros((0, 32), np.uint8)
        self.bad = np.zeros(0, bool)          # culled points are flagged, never reused
        self.created = np.zeros(0, int)       # keyframe count when the point was born

    def add(self, X, des, n_kf=0):
        ids = np.arange(len(self.X), len(self.X) + len(X))
        self.X = np.vstack([self.X, X])
        self.des = np.vstack([self.des, des])
        self.bad = np.r_[self.bad, np.zeros(len(X), bool)]
        self.created = np.r_[self.created, np.full(len(X), n_kf)]
        return ids

    @property
    def n_good(self):
        return int((~self.bad).sum())


# ---------------------------------------------------------------- tracker
class Tracker:
    def __init__(self, K, width=640, height=480):
        self.K, self.w, self.h = K, width, height
        self.map = Map()
        self.keyframes = []
        self.state = "INIT"
        self.init_ref = None
        self.last = None
        self.velocity = np.eye(4)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.trajectory = []        # (timestamp, T_wc) of every tracked frame
        self.log = []               # per-frame diagnostics
        self.lost_frames = 0

        # parameters
        self.init_min_parallax = 20.0   # px
        self.init_min_points = 100      # matches needed to attempt init
        self.init_min_triangulated = 150
        self.radius = 15.0              # px, projection search window
        self.max_hamming = 64           # of 256 bits
        self.min_inliers = 30
        self.n_local_kf = 10
        self.kf_ratio = 0.7
        self.kf_min_gap = 3             # frames between keyframes (unless urgent)
        self.kf_urgent = 80             # tracked points below this -> keyframe now
        self.tri_n_kf = 5               # triangulate against this many past keyframes
        self.tri_min_baseline = 0.02    # baseline / median depth
        self.px_sigma = 1.5             # keypoint noise (px); all gates derive from it
        self.use_ba = True
        self.local_map_all = True       # test: search every good point
        self.ba_window = 10             # keyframes optimized in local BA
        self.ba_fixed = 10              # max older keyframes held fixed
        self.cull_age = 3               # keyframes after birth before culling check
        self.cull_min_obs = 3           # keyframes that must observe a point to keep it

    # ------------------------------------------------ public
    def process(self, frame):
        self._rec = {"idx": frame.idx, "ts": frame.ts, "path": "", "cand": 0,
                     "pnp_inl": 0, "tracked": 0, "kf": 0, "new_pts": 0}
        ok = self._initialize(frame) if self.state == "INIT" else self._track(frame)
        if self._rec["path"] == "":
            self._rec["path"] = "init_wait" if self.state == "INIT" else "lost"
        self.log.append(self._rec)
        return ok

    @property
    def _gate(self):
        """95% chi-square gate for a 2D reprojection error: sqrt(5.991) * sigma."""
        return 2.45 * self.px_sigma

    @property
    def n_tracked(self):
        return int((self.last.mp >= 0).sum()) if self.last is not None else 0

    # ------------------------------------------------ initialization
    def _initialize(self, frame):
        if self.init_ref is None:
            if len(frame.pts) > 300:
                self.init_ref = frame
            return False
        ref = self.init_ref
        i1, i2 = match_descriptors(self.matcher, ref.des, frame.des)
        if len(i1) < self.init_min_points:
            self.init_ref = frame if len(frame.pts) > 300 else None  # reference too old
            return False
        p1, p2 = ref.pts[i1], frame.pts[i2]
        if np.median(np.linalg.norm(p1 - p2, axis=1)) < self.init_min_parallax:
            return False

        E, mask = cv2.findEssentialMat(p1, p2, self.K, cv2.RANSAC, 0.999, 1.0)
        if E is None or E.shape != (3, 3):
            return False
        _, R, t, mask = cv2.recoverPose(E, p1, p2, self.K, mask=mask.copy())
        inl = mask.ravel() > 0
        T1, T2 = np.eye(4), np.eye(4)
        T2[:3, :3], T2[:3, 3] = R, t.ravel()

        X = triangulate(self.K, T1, T2, p1[inl], p2[inl])
        ok = good_triangulation(self.K, T1, T2, X, p1[inl], p2[inl], max_err=self._gate)
        if ok.sum() < self.init_min_triangulated:
            return False

        # Fix the gauge: world = first camera, median scene depth = 1
        s = 1.0 / np.median(X[ok, 2])
        X = X[ok] * s
        T2[:3, 3] *= s
        ref.T_cw, frame.T_cw = T1, T2

        a, b = i1[inl][ok], i2[inl][ok]
        ids = self.map.add(X, frame.des[b])
        ref.mp[a] = ids
        frame.mp[b] = ids

        self.keyframes = [ref, frame]
        if self.use_ba:                       # refine the 2-view map, keep KF0 fixed
            out = bundle_adjust(self.K, [frame], [ref], self.map.X, ids, max_nfev=30, sigma=self.px_sigma)
            self._remove_outlier_obs(out)
            z = (self.map.X[ids] @ frame.T_cw[:3, :3].T + frame.T_cw[:3, 3])[:, 2]
            s2 = 1.0 / np.median(z)
            self.map.X[ids] *= s2
            frame.T_cw[:3, 3] *= s2
        self.trajectory = [(ref.ts, ref.T_wc), (frame.ts, frame.T_wc)]
        self.last, self.state = frame, "OK"
        print(f"[init] frames {ref.idx}->{frame.idx}: {len(ids)} map points")
        self._rec.update(path="init", tracked=len(ids), kf=1, new_pts=len(ids))
        return True

    # ------------------------------------------------ tracking
    def _local_map_ids(self):
        if self.local_map_all:
            return np.where(~self.map.bad)[0]
        ids = np.concatenate([kf.mp[kf.mp >= 0] for kf in self.keyframes[-self.n_local_kf:]]
                             + [self.last.mp[self.last.mp >= 0]]).astype(int)
        ids = np.unique(ids)
        return ids[~self.map.bad[ids]]

    def search_by_projection(self, frame, T_cw, mp_ids, radius):
        """Project map points with a pose guess, match to nearby free keypoints."""
        uv, z = project(self.K, T_cw, self.map.X[mp_ids])
        vis = (z > 1e-3) & (uv[:, 0] >= 0) & (uv[:, 0] < self.w) & (uv[:, 1] >= 0) & (uv[:, 1] < self.h)
        mp_ids, uv = mp_ids[vis], uv[vis]
        free = np.where(frame.mp < 0)[0]
        if len(mp_ids) == 0 or len(free) == 0:
            return np.array([], int), np.array([], int)

        # KD-tree over free keypoints: up to 8 candidates within the radius per map point
        tree = cKDTree(frame.pts[free])
        dist, nn = tree.query(uv, k=min(8, len(free)), distance_upper_bound=radius)
        dist, nn = dist.reshape(len(uv), -1), nn.reshape(len(uv), -1)
        mi, col = np.nonzero(np.isfinite(dist))
        if len(mi) == 0:
            return np.array([], int), np.array([], int)
        ki = nn[mi, col]
        ham = hamming_pairs(self.map.des[mp_ids[mi]], frame.des[free[ki]])

        # best keypoint per map point, then enforce one-to-one by distance
        order = np.lexsort((ham, mi))
        mi, ki, ham = mi[order], ki[order], ham[order]
        first = np.r_[True, mi[1:] != mi[:-1]]
        mi, ki, ham = mi[first], ki[first], ham[first]
        keep = ham < self.max_hamming
        mi, ki, ham = mi[keep], ki[keep], ham[keep]
        order = np.argsort(ham, kind="stable")
        _, uniq = np.unique(ki[order], return_index=True)
        sel = order[uniq]
        return mp_ids[mi[sel]], free[ki[sel]]

    def _pnp(self, frame, mids, kidx, T_guess):
        obj = self.map.X[mids].astype(np.float64)
        img = frame.pts[kidx].astype(np.float64)
        rvec, _ = cv2.Rodrigues(T_guess[:3, :3])
        tvec = T_guess[:3, 3].copy().reshape(3, 1)
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            obj, img, self.K, None, rvec, tvec, useExtrinsicGuess=True,
            iterationsCount=200, reprojectionError=self._gate, confidence=0.999)
        if not ok or inl is None or len(inl) < self.min_inliers:
            return None, None
        inl = inl.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], self.K, None, rvec, tvec)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
        return T, inl

    def _reproj_inliers(self, frame, mids, kidx, T_cw, thresh=3.0):
        uv, z = project(self.K, T_cw, self.map.X[mids])
        err = np.linalg.norm(uv - frame.pts[kidx], axis=1)
        return (z > 0) & (err < thresh)

    def _track(self, frame):
        T_pred = self.velocity @ self.last.T_cw
        local = self._local_map_ids()

        path = "proj"
        mids, kidx = self.search_by_projection(frame, T_pred, local, self.radius)
        if len(mids) < 2 * self.min_inliers:             # prediction poor: wider search
            path = "wide"
            mids, kidx = self.search_by_projection(frame, T_pred, local, 3 * self.radius)
        if len(mids) < 2 * self.min_inliers:             # fall back to pure descriptor matching
            path = "desc"
            a, b = match_descriptors(self.matcher, self.map.des[local], frame.des)
            mids, kidx = local[a], b
        self._rec["cand"] = len(mids)

        T, inl = (None, None) if len(mids) < self.min_inliers else self._pnp(frame, mids, kidx, T_pred)
        if T is None:
            self.lost_frames += 1
            self.velocity = np.eye(4)
            if self.lost_frames > 30:
                self.state = "LOST"
            return False
        self.lost_frames = 0
        self._rec["path"], self._rec["pnp_inl"] = path, len(inl)
        mids, kidx = mids[inl], kidx[inl]
        frame.mp[kidx] = mids

        # second pass: with an accurate pose, find more matches in a tight window.
        # Reject outliers BEFORE refining - plain LM is not robust.
        rest = np.setdiff1d(local, mids)
        m2, k2 = self.search_by_projection(frame, T, rest, self.radius / 3)
        if len(m2):
            g2 = self._reproj_inliers(frame, m2, k2, T, thresh=self._gate)
            m2, k2 = m2[g2], k2[g2]
        mids, kidx = np.r_[mids, m2], np.r_[kidx, k2]
        rvec, _ = cv2.Rodrigues(T[:3, :3])
        rvec, tvec = cv2.solvePnPRefineLM(self.map.X[mids], frame.pts[kidx], self.K, None,
                                          rvec, T[:3, 3].reshape(3, 1).copy())
        T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
        good = self._reproj_inliers(frame, mids, kidx, T, thresh=self._gate)
        frame.mp[:] = -1
        frame.mp[kidx[good]] = mids[good]

        frame.T_cw = T
        self.velocity = T @ inv_T(self.last.T_cw)
        self.last = frame
        self.trajectory.append((frame.ts, frame.T_wc))

        n_ref = (self.keyframes[-1].mp >= 0).sum()
        n_cur = (frame.mp >= 0).sum()
        self._rec["tracked"] = int(n_cur)
        gap = frame.idx - self.keyframes[-1].idx
        if (n_cur < self.kf_ratio * n_ref and gap >= self.kf_min_gap) or n_cur < self.kf_urgent:
            self._rec["kf"] = 1
            self._rec["new_pts"] = self._insert_keyframe(frame)
        return True

    # ------------------------------------------------ mapping
    def _insert_keyframe(self, frame):
        new_total = 0
        _, z = project(self.K, frame.T_cw, self.map.X[frame.mp[frame.mp >= 0]])
        med_depth = np.median(z) if len(z) else 1.0
        c_new = frame.T_wc[:3, 3]
        for other in self.keyframes[-self.tri_n_kf:][::-1]:
            baseline = np.linalg.norm(other.T_wc[:3, 3] - c_new)
            if baseline / med_depth < self.tri_min_baseline:
                continue                  # too little baseline: depth would be garbage
            a = np.where(other.mp < 0)[0]
            b = np.where(frame.mp < 0)[0]
            i1, i2 = match_descriptors(self.matcher, other.des[a], frame.des[b])
            if len(i1) == 0:
                continue
            i1, i2 = a[i1], b[i2]
            p1, p2 = other.pts[i1], frame.pts[i2]
            epi = sampson_ok(self.K, other.T_cw, frame.T_cw, p1, p2, thresh_px2=3.84 * self.px_sigma ** 2)
            i1, i2, p1, p2 = i1[epi], i2[epi], p1[epi], p2[epi]
            if len(i1) == 0:
                continue
            X = triangulate(self.K, other.T_cw, frame.T_cw, p1, p2)
            ok = good_triangulation(self.K, other.T_cw, frame.T_cw, X, p1, p2, max_err=self._gate)
            ids = self.map.add(X[ok], frame.des[i2[ok]], len(self.keyframes))
            other.mp[i1[ok]] = ids
            frame.mp[i2[ok]] = ids
            new_total += len(ids)
        self.keyframes.append(frame)
        if self.use_ba:
            self._local_ba()
        self._cull_points()
        return new_total

    # ------------------------------------------------ back end
    def _remove_outlier_obs(self, outliers):
        for kf in self.keyframes:
            k = outliers.get(id(kf))
            if k is not None and len(k):
                kf.mp[k] = -1

    def _local_ba(self):
        opt = self.keyframes[-self.ba_window:]
        pids = np.unique(np.concatenate([kf.mp[kf.mp >= 0] for kf in opt]))
        pids = pids[~self.map.bad[pids]]
        older = self.keyframes[:-self.ba_window]
        mask = np.zeros(len(self.map.X), bool)
        mask[pids] = True
        fixed = [kf for kf in older if mask[kf.mp[kf.mp >= 0]].any()][-self.ba_fixed:]
        if not fixed:                         # early on: pin the first two keyframes (gauge + scale)
            fixed, opt = opt[:2], opt[2:]
        out = bundle_adjust(self.K, opt, fixed, self.map.X, pids, sigma=self.px_sigma)
        self._remove_outlier_obs(out)
        # the current frame is the newest keyframe: keep motion model consistent
        if len(self.keyframes) >= 2:
            self.last = self.keyframes[-1]

    def _cull_points(self):
        counts = np.bincount(np.concatenate([kf.mp[kf.mp >= 0] for kf in self.keyframes]),
                             minlength=len(self.map.X))
        age = len(self.keyframes) - self.map.created
        cull = (~self.map.bad) & (age >= self.cull_age) & (counts < self.cull_min_obs)
        self.map.bad |= cull
        for kf in self.keyframes:             # drop associations to culled points
            m = kf.mp >= 0
            kf.mp[m & self.map.bad[np.where(m, kf.mp, 0)]] = -1
