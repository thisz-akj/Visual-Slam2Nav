"""Numerical differentiation helpers (plumbing, provided).

Use these to check every analytic Jacobian you write in this project.
"""
import numpy as np


def numeric_jacobian(f, x, eps=1e-6):
    """Central-difference Jacobian of f: R^n -> R^m at x.  Returns (m, n)."""
    x = np.asarray(x, float)
    f0 = np.atleast_1d(f(x))
    J = np.zeros((f0.size, x.size))
    for i in range(x.size):
        d = np.zeros_like(x)
        d[i] = eps
        J[:, i] = (np.atleast_1d(f(x + d)) - np.atleast_1d(f(x - d))).ravel() / (2 * eps)
    return J
