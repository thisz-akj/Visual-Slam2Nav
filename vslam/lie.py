"""Lab 0: Lie groups SO(3) and SE(3).

Conventions (see ROADMAP.md):
    se(3) vector  xi = [rho, phi]   (translation part first)
    left perturbation  T <- exp(xi^) @ T
All functions take/return float numpy arrays; no in-place modification of inputs.
"""
import numpy as np

SMALL_ANGLE = 1e-8


# ----------------------------------------------------------------------- SO(3)
def hat(phi):
    """(3,) -> (3,3) skew-symmetric matrix with hat(phi) @ a == cross(phi, a)."""
    raise NotImplementedError("TODO Lab 0")


def vee(Phi):
    """(3,3) skew-symmetric -> (3,). Inverse of hat."""
    raise NotImplementedError("TODO Lab 0")


def so3_exp(phi):
    """Rotation vector (3,) -> rotation matrix (3,3) via Rodrigues.

    Must be accurate for ||phi|| -> 0 (use the Taylor form below SMALL_ANGLE).
    """
    raise NotImplementedError("TODO Lab 0")


def so3_log(R):
    """Rotation matrix (3,3) -> rotation vector (3,) with norm in [0, pi].

    Must be accurate near theta = 0 AND near theta = pi (see lab notes, Part A).
    Clip the arccos argument: numerical noise can push trace slightly out of range.
    """
    raise NotImplementedError("TODO Lab 0")


def so3_left_jacobian(phi):
    """(3,) -> (3,3) left Jacobian J_l(phi) of SO(3)."""
    raise NotImplementedError("TODO Lab 0")


def so3_left_jacobian_inv(phi):
    """(3,) -> (3,3) inverse of J_l(phi). Closed form, or a linear solve; both are fine."""
    raise NotImplementedError("TODO Lab 0")


# ----------------------------------------------------------------------- SE(3)
def se3_hat(xi):
    """(6,) [rho, phi] -> (4,4) Lie algebra matrix [[phi^, rho], [0, 0]]."""
    raise NotImplementedError("TODO Lab 0")


def se3_exp(xi):
    """(6,) [rho, phi] -> (4,4) homogeneous transform."""
    raise NotImplementedError("TODO Lab 0")


def se3_log(T):
    """(4,4) -> (6,) [rho, phi]. Note rho != translation in general."""
    raise NotImplementedError("TODO Lab 0")


def se3_inv(T):
    """(4,4) -> (4,4) inverse, using the structure of SE(3) (no np.linalg.inv)."""
    raise NotImplementedError("TODO Lab 0")


def se3_adjoint(T):
    """(4,4) -> (6,6) Ad_T such that T exp(xi^) T^-1 == exp((Ad_T xi)^)."""
    raise NotImplementedError("TODO Lab 0")


def se3_interpolate(T0, T1, s):
    """Pose a fraction s in [0,1] of the way from T0 to T1, moving along the
    constant-twist (screw) path:  exp(s * log(T1 T0^-1)) T0.  Returns (4,4).

    Used for evaluation: ground truth is 100 Hz, camera frames are 30 Hz.
    """
    raise NotImplementedError("TODO Lab 0")


def transform_point_jacobian(T, p):
    """d(exp(delta^) @ T @ p)/d(delta) at delta = 0, for p (3,).  Returns (3,6).

    This is the building block of every PnP / BA Jacobian (chained in Lab 1).
    """
    raise NotImplementedError("TODO Lab 0")
