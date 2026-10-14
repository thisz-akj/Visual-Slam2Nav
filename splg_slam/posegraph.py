"""SE(3) pose-graph optimization over keyframe poses (T_wc), solved with scipy least squares.

Residuals are computed for all edges at once (numpy batch ops): identical maths to a per-edge loop,
but fast enough to run after every loop closure on graphs with hundreds of keyframes.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


def to_vec(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def to_mat(x):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
    T[:3, 3] = x[3:]
    return T


def optimize(poses, edges, fixed=(0,), sigma_t=0.02, sigma_r=0.01, iters=50):
    """poses: list of 4x4 T_wc. edges: (i, j, Z_ij, weight) with Z_ij = T_i^-1 T_j measured.

    Returns optimized poses. Robust (soft-L1) loss so a wrong loop edge cannot wreck the map.
    """
    n = len(poses)
    fixed = set(fixed)
    free = [k for k in range(n) if k not in fixed]
    idx = {k: i for i, k in enumerate(free)}
    x0 = np.concatenate([to_vec(poses[k]) for k in free])
    P = np.array(poses)
    I = np.array([e[0] for e in edges])
    J = np.array([e[1] for e in edges])
    RZt = np.array([e[2][:3, :3].T for e in edges])
    tZ = np.array([e[2][:3, 3] for e in edges])
    w = np.array([e[3] for e in edges])[:, None]
    free_arr = np.array(free)

    def all_poses(x):
        R = P[:, :3, :3].copy()
        t = P[:, :3, 3].copy()
        v = x.reshape(-1, 6)
        R[free_arr] = Rotation.from_rotvec(v[:, :3]).as_matrix()
        t[free_arr] = v[:, 3:]
        return R, t

    def residuals(x):
        R, t = all_poses(x)
        Rij = np.einsum("eji,ejk->eik", R[I], R[J])                    # R_i^T R_j
        tij = np.einsum("eji,ej->ei", R[I], t[J] - t[I])               # R_i^T (t_j - t_i)
        RE = RZt @ Rij                                                 # Z^-1 (T_i^-1 T_j)
        tE = np.einsum("eij,ej->ei", RZt, tij - tZ)
        r = np.c_[Rotation.from_matrix(RE).as_rotvec() / sigma_r, tE / sigma_t] * w
        return r.ravel()

    A = lil_matrix((6 * len(edges), len(x0)), dtype=int)
    for e, (i, j, _, _) in enumerate(edges):
        for k in (i, j):
            if k not in fixed:
                A[6 * e: 6 * e + 6, 6 * idx[k]: 6 * idx[k] + 6] = 1
    res = least_squares(residuals, x0, jac_sparsity=A, method="trf", loss="soft_l1", f_scale=3.0,
                        x_scale="jac", max_nfev=iters)
    R, t = all_poses(res.x)
    out = []
    for k in range(n):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R[k], t[k]
        out.append(T)
    return out, res
