"""Phase 3: sparse bundle adjustment.

Minimizes   sum_obs  huber( || project(K, T_k X_j) - u_kj ||^2 )
over the poses of the 'optimized' keyframes and all points they observe.
'Fixed' keyframes contribute observations but don't move: they pin the gauge
(position, rotation and - for monocular - scale).
"""
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


def _rodrigues_batch(r):
    """(N,3) axis-angle -> (N,3,3) rotation matrices."""
    theta = np.linalg.norm(r, axis=1)
    out = np.tile(np.eye(3), (len(r), 1, 1))
    m = theta > 1e-12
    if m.any():
        k = r[m] / theta[m, None]
        Kx = np.zeros((m.sum(), 3, 3))
        Kx[:, 0, 1], Kx[:, 0, 2] = -k[:, 2], k[:, 1]
        Kx[:, 1, 0], Kx[:, 1, 2] = k[:, 2], -k[:, 0]
        Kx[:, 2, 0], Kx[:, 2, 1] = -k[:, 1], k[:, 0]
        s, c = np.sin(theta[m])[:, None, None], np.cos(theta[m])[:, None, None]
        out[m] = np.eye(3) + s * Kx + (1 - c) * (Kx @ Kx)
    return out


def bundle_adjust(K, kfs_opt, kfs_fixed, X_all, pids, max_nfev=15, sigma=1.0):
    """Refine kf.T_cw (for kfs_opt) and X_all[pids] in place.

    Returns per-keyframe lists of keypoint indices whose reprojection error is
    an outlier after optimization (caller decides what to do with them).
    """
    if len(pids) == 0 or len(kfs_opt) == 0:
        return {}
    lut = -np.ones(len(X_all), int)
    lut[pids] = np.arange(len(pids))
    kfs = list(kfs_opt) + list(kfs_fixed)
    n_opt = len(kfs_opt)

    ok_, op_, ouv, okp = [], [], [], []
    for j, kf in enumerate(kfs):
        k = np.where(kf.mp >= 0)[0]
        k = k[lut[kf.mp[k]] >= 0]
        ok_.append(np.full(len(k), j))
        op_.append(lut[kf.mp[k]])
        ouv.append(kf.pts[k])
        okp.append(k)
    obs_kf, obs_pt = np.concatenate(ok_), np.concatenate(op_)
    obs_uv, obs_kp = np.concatenate(ouv), np.concatenate(okp)
    if len(obs_kf) < 20:
        return {}

    fixed_R = np.array([kf.T_cw[:3, :3] for kf in kfs_fixed]).reshape(-1, 3, 3)
    fixed_t = np.array([kf.T_cw[:3, 3] for kf in kfs_fixed]).reshape(-1, 3)
    x0 = np.concatenate(
        [np.r_[cv2.Rodrigues(kf.T_cw[:3, :3])[0].ravel(), kf.T_cw[:3, 3]] for kf in kfs_opt]
        + [X_all[pids].ravel()])
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    def unpack(x):
        p = x[:6 * n_opt].reshape(-1, 6)
        R = np.concatenate([_rodrigues_batch(p[:, :3]), fixed_R])
        t = np.concatenate([p[:, 3:], fixed_t])
        return R, t, x[6 * n_opt:].reshape(-1, 3)

    def residuals(x):
        R, t, Xp = unpack(x)
        Xc = np.einsum("nij,nj->ni", R[obs_kf], Xp[obs_pt]) + t[obs_kf]
        z = np.maximum(Xc[:, 2], 1e-3)
        u = fx * Xc[:, 0] / z + cx
        v = fy * Xc[:, 1] / z + cy
        return np.c_[u - obs_uv[:, 0], v - obs_uv[:, 1]].ravel()

    # Jacobian sparsity: each residual pair touches one pose (if optimized) and one point
    n_obs = len(obs_kf)
    A = lil_matrix((2 * n_obs, len(x0)), dtype=int)
    rows = np.arange(n_obs)
    for d in range(3):
        A[2 * rows, 6 * n_opt + 3 * obs_pt + d] = 1
        A[2 * rows + 1, 6 * n_opt + 3 * obs_pt + d] = 1
    opt = obs_kf < n_opt
    for d in range(6):
        A[2 * rows[opt], 6 * obs_kf[opt] + d] = 1
        A[2 * rows[opt] + 1, 6 * obs_kf[opt] + d] = 1

    res = least_squares(residuals, x0, jac_sparsity=A, method="trf", loss="huber",
                        f_scale=sigma, x_scale="jac", max_nfev=max_nfev)

    R, t, Xp = unpack(res.x)
    for j, kf in enumerate(kfs_opt):
        kf.T_cw = np.eye(4)
        kf.T_cw[:3, :3], kf.T_cw[:3, 3] = R[j], t[j]
    X_all[pids] = Xp

    # outlier observations after optimization (chi2 95% gate for 2 dof)
    err = np.linalg.norm(residuals(res.x).reshape(-1, 2), axis=1)
    bad = err > 2.45 * sigma
    out = {}
    for j in np.unique(obs_kf[bad]):
        out[id(kfs[j])] = obs_kp[bad & (obs_kf == j)]
    return out
