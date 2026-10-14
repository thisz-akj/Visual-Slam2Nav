"""RGB-D SLAM with SuperPoint + LightGlue.

Same architecture as ORB-SLAM3 (so the comparison isolates the features):
  tracking      match to the reference keyframe, PnP-RANSAC on its depth-lifted points
  keyframes     new KF when overlap drops or the camera moves; extra edges to nearby KFs
  relocalize    when tracking fails, search KFs by global-descriptor similarity
  new map       after repeated failure, start a new map (like ORB-SLAM3's Atlas)
  loop / merge  verified matches to old KFs -> pose-graph optimization; across maps -> merge
Frames are stored relative to their reference keyframe, so optimizations move them too.
"""
import cv2
import numpy as np

from .posegraph import optimize


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rel_motion(Ta, Tb):
    """Translation (m) and rotation (deg) between two poses."""
    D = inv(Ta) @ Tb
    ang = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    return np.linalg.norm(D[:3, 3]), ang


class KeyFrame:
    def __init__(self, kid, map_id, feats, T_wc, ts):
        self.id, self.map_id, self.f, self.T_wc, self.ts = kid, map_id, feats, T_wc, ts


class SLAM:
    def __init__(self, frontend, K, log=print):
        self.fe, self.K, self.log = frontend, K, log
        self.kfs, self.edges = [], []          # edges: (kf_i, kf_j, Z_ij = T_i^-1 T_j, weight)
        self.frames = []                       # (ts, kf_id, T_rel)  with  T_wc = kf.T_wc @ T_rel
        self.map_id = -1
        self.ref = None
        self.last = None                       # (feats, T_wc) of the last tracked frame
        self.velocity = np.eye(4)
        self.lost = 0
        self.stats = {"tracked": 0, "lost": 0, "relocalized": 0, "new_maps": 0, "loops": 0, "merges": 0,
                      "kfs": 0}
        # parameters
        self.min_inliers = 25
        self.kf_inlier_ratio = 0.5
        self.kf_dist, self.kf_angle = 0.30, 20.0
        self.loop_min_inliers = 45
        self.loop_gap = 25                     # a loop candidate must be this many KFs older

    # ------------------------------------------------------------ geometry
    def pnp(self, src, dst, guess_T_dst_src=None, min_inl=None):
        """Pose of camera `dst` relative to `src` (T_dst_src) from src's depth-lifted points.

        Returns (T_dst_src, n_inliers) or (None, n)."""
        ia, ib = self.fe.match(src, dst)
        ok = src.valid[ia]
        ia, ib = ia[ok], ib[ok]
        min_inl = min_inl or self.min_inliers
        if len(ia) < min_inl:
            return None, len(ia)
        obj, img = src.p3[ia], dst.kp[ib]
        kw = dict(iterationsCount=300, reprojectionError=3.0, confidence=0.999, flags=cv2.SOLVEPNP_ITERATIVE)
        if guess_T_dst_src is not None:
            rv, _ = cv2.Rodrigues(guess_T_dst_src[:3, :3])
            tv = guess_T_dst_src[:3, 3].reshape(3, 1).copy()
            ok, rv, tv, inl = cv2.solvePnPRansac(obj, img, self.K, None, rv, tv, useExtrinsicGuess=True, **kw)
        else:
            ok, rv, tv, inl = cv2.solvePnPRansac(obj, img, self.K, None, **kw)
        if not ok or inl is None or len(inl) < min_inl:
            return None, 0 if inl is None else len(inl)
        inl = inl.ravel()
        rv, tv = cv2.solvePnPRefineLM(obj[inl], img[inl], self.K, None, rv, tv)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = cv2.Rodrigues(rv)[0], tv.ravel()
        return T, len(inl)

    # ------------------------------------------------------------ keyframes / maps
    def add_kf(self, f, T_wc, ts):
        kf = KeyFrame(len(self.kfs), self.map_id, f, T_wc, ts)
        kf.n_valid = int(f.valid.sum())
        self.kfs.append(kf)
        self.stats["kfs"] += 1
        return kf

    def new_map(self, f, ts):
        self.map_id += 1
        self.stats["new_maps"] += 1
        kf = self.add_kf(f, np.eye(4), ts)
        self.log(f"New map {self.map_id} at KF {kf.id} ({int(f.valid.sum())} points with depth)")
        self.ref, self.last, self.velocity, self.lost = kf, (f, kf.T_wc), np.eye(4), 0
        self.frames.append((ts, kf.id, np.eye(4)))

    def map_kfs(self, m):
        return [k for k in self.kfs if k.map_id == m]

    # ------------------------------------------------------------ main entry
    def process(self, f, ts):
        if self.ref is None:
            if f.valid.sum() >= 100:
                self.new_map(f, ts)
            return
        T_pred = self.last[1] @ self.velocity                 # constant velocity (in world frame)
        T_wc, inl, ref = self.track(f, T_pred)
        if T_wc is None:
            T_wc, inl, ref = self.relocalize(f)
            if T_wc is not None:
                self.stats["relocalized"] += 1
                self.log(f"  relocalized against KF {ref.id} ({inl} inliers)")
        if T_wc is None:
            self.lost += 1
            self.stats["lost"] += 1
            self.velocity = np.eye(4)
            if self.lost >= 3 and f.valid.sum() >= 100:
                self.new_map(f, ts)
            return
        self.lost = 0
        self.stats["tracked"] += 1
        self.velocity = inv(self.last[1]) @ T_wc
        self.last = (f, T_wc)
        self.ref = ref
        d, a = rel_motion(ref.T_wc, T_wc)
        if inl < max(60, self.kf_inlier_ratio * min(ref.n_valid, 500)) or d > self.kf_dist or a > self.kf_angle:
            kf = self.add_kf(f, T_wc, ts)
            self.edges.append((ref.id, kf.id, inv(ref.T_wc) @ T_wc, 1.0))
            self.ref = kf
            self.connect_neighbours(kf)
            self.detect_loop(kf)
            self.frames.append((ts, kf.id, np.eye(4)))
        else:
            self.frames.append((ts, ref.id, inv(ref.T_wc) @ T_wc))

    def track(self, f, T_pred):
        ref = self.ref
        T, inl = self.pnp(ref.f, f, inv(T_pred) @ ref.T_wc)
        if T is not None:
            return ref.T_wc @ inv(T), inl, ref
        lf, lT = self.last                                    # fallback: the previous frame
        T, inl = self.pnp(lf, f, inv(T_pred) @ lT)
        if T is not None:
            return lT @ inv(T), inl, ref
        return None, 0, None

    def relocalize(self, f):
        cands = [k for k in self.kfs if k.map_id == self.map_id]
        if not cands:
            return None, 0, None
        sims = np.array([k.f.gdesc @ f.gdesc for k in cands])
        order = list(np.argsort(-sims)[:4]) + list(range(max(0, len(cands) - 3), len(cands)))
        best = (None, 0, None)
        for i in dict.fromkeys(order):
            kf = cands[i]
            T, inl = self.pnp(kf.f, f, min_inl=40)
            if T is not None and inl > best[1]:
                best = (kf.T_wc @ inv(T), inl, kf)
        return best

    # ------------------------------------------------------------ back end
    def connect_neighbours(self, kf):
        """Extra edges to nearby keyframes of the same map: redundancy that limits drift."""
        near = []
        for k in self.map_kfs(kf.map_id):
            if k.id >= kf.id - 1:
                continue
            d, a = rel_motion(k.T_wc, kf.T_wc)
            if d < 1.2 and a < 35:
                near.append((d, k))
        for _, k in sorted(near, key=lambda x: x[0])[:2]:
            T, inl = self.pnp(k.f, kf.f, inv(kf.T_wc) @ k.T_wc, min_inl=30)
            if T is not None:
                self.edges.append((k.id, kf.id, inv(T), 1.0))

    def detect_loop(self, kf):
        old = [k for k in self.kfs if k.id < kf.id - self.loop_gap]
        if not old:
            return
        sims = np.array([k.f.gdesc @ kf.f.gdesc for k in old])
        cands = list(np.argsort(-sims)[:3])
        same = [i for i, k in enumerate(old) if k.map_id == kf.map_id]
        if same:
            dist = [np.linalg.norm(old[i].T_wc[:3, 3] - kf.T_wc[:3, 3]) for i in same]
            cands += [same[i] for i in np.argsort(dist)[:2] if dist[i] < 1.5]
        for i in dict.fromkeys(cands):
            k = old[i]
            T, inl = self.pnp(k.f, kf.f, min_inl=self.loop_min_inliers)
            if T is None:
                continue
            Z = inv(T)                                        # T_k^-1 T_kf
            if k.map_id != kf.map_id:
                self.merge(k, kf, Z, inl)
            else:
                self.edges.append((k.id, kf.id, Z, 1.0))
                self.stats["loops"] += 1
                self.log(f"Loop detected: KF {kf.id} <-> KF {k.id} ({inl} inliers)")
                self.optimize(kf.map_id)
            return

    def merge(self, k_old, kf, Z, inl):
        """Bring the current map into the old map's frame, then optimize the joined map."""
        T_align = (k_old.T_wc @ Z) @ inv(kf.T_wc)
        cur = kf.map_id
        for k in self.map_kfs(cur):
            k.T_wc = T_align @ k.T_wc
            k.map_id = k_old.map_id
        self.last = (self.last[0], T_align @ self.last[1])
        self.map_id = k_old.map_id
        self.edges.append((k_old.id, kf.id, Z, 1.0))
        self.stats["merges"] += 1
        self.log(f"Merge detected: map {cur} -> map {k_old.map_id} via KF {kf.id} <-> KF {k_old.id} ({inl} inliers)")
        self.optimize(k_old.map_id)

    def optimize(self, m):
        ids = [k.id for k in self.map_kfs(m)]
        if len(ids) < 3:
            return
        pos = {kid: i for i, kid in enumerate(ids)}
        edges = [(pos[i], pos[j], Z, w) for i, j, Z, w in self.edges if i in pos and j in pos]
        before = self.last[1] @ np.eye(4)
        ref_before = self.ref.T_wc.copy()
        new, res = optimize([self.kfs[i].T_wc for i in ids], edges, fixed=(0,))
        for kid, T in zip(ids, new):
            self.kfs[kid].T_wc = T
        # keep the tracker consistent with the corrected reference keyframe
        self.last = (self.last[0], self.ref.T_wc @ inv(ref_before) @ before)
        self.log(f"  pose graph: {len(ids)} KFs, {len(edges)} edges, cost {res.cost:.1f}")

    # ------------------------------------------------------------ output
    def trajectory(self):
        return [(ts, self.kfs[k].T_wc @ T_rel) for ts, k, T_rel in self.frames]

    def map_points(self, step=4):
        out = []
        for k in self.kfs:
            P = k.f.p3[k.f.valid][::step]
            Pw = P @ k.T_wc[:3, :3].T + k.T_wc[:3, 3]
            out.append(np.c_[np.full(len(Pw), k.map_id), Pw, np.ones(len(Pw))])
        return np.vstack(out) if out else np.zeros((0, 5))
