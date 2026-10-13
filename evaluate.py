"""Trajectory evaluation: Umeyama alignment + Absolute Trajectory Error."""
import numpy as np


def umeyama(src, dst, with_scale=True):
    """Find s, R, t minimizing || dst - (s R src + t) ||.  src, dst: (N,3)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, D, Vt = np.linalg.svd(xd.T @ xs / len(src))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / xs.var(0).sum() if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def ate(est, gt, with_scale=True):
    """Returns RMSE (m), aligned estimate, and the scale used."""
    s, R, t = umeyama(est, gt, with_scale)
    aligned = (s * (R @ est.T)).T + t
    err = np.linalg.norm(aligned - gt, axis=1)
    return np.sqrt((err ** 2).mean()), aligned, s
