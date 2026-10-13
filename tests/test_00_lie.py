"""Lab 0 tests. Ground truth comes from scipy (expm/logm/Rotation), never from vslam itself."""
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

from vslam import lie
from vslam.numdiff import numeric_jacobian

rng = np.random.default_rng(0)


def _ref_hat(p):
    return np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])


def _ref_se3_hat(xi):
    M = np.zeros((4, 4))
    M[:3, :3], M[:3, 3] = _ref_hat(xi[3:]), xi[:3]
    return M


def _rand_rotvec(max_angle=np.pi - 1e-3):
    a = rng.normal(size=3)
    return a / np.linalg.norm(a) * rng.uniform(0, max_angle)


def _rand_T():
    return expm(_ref_se3_hat(np.r_[rng.normal(size=3), _rand_rotvec()]))


def _rot_dist(R1, R2):
    return Rotation.from_matrix(R1 @ R2.T).magnitude()


# ------------------------------------------------------------------ SO(3)
def test_hat_vee():
    for _ in range(20):
        a, b = rng.normal(size=3), rng.normal(size=3)
        assert np.allclose(lie.hat(a) @ b, np.cross(a, b))
        assert np.allclose(lie.hat(a), -lie.hat(a).T)
        assert np.allclose(lie.vee(lie.hat(a)), a)


def test_so3_exp_matches_scipy():
    for _ in range(50):
        phi = _rand_rotvec()
        R = lie.so3_exp(phi)
        assert np.allclose(R, Rotation.from_rotvec(phi).as_matrix(), atol=1e-12)
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(R), 1.0)


def test_so3_exp_small_angles():
    assert np.allclose(lie.so3_exp(np.zeros(3)), np.eye(3))
    for s in [1e-6, 1e-9, 1e-12, 1e-15]:
        phi = np.array([1.0, -2.0, 0.5]) * s
        R = lie.so3_exp(phi)
        assert np.all(np.isfinite(R)), f"NaN/inf at |phi|~{s}"
        assert np.allclose(R, Rotation.from_rotvec(phi).as_matrix(), atol=1e-14)


def test_so3_log_roundtrip():
    for _ in range(50):
        phi = _rand_rotvec()
        assert np.allclose(lie.so3_log(lie.so3_exp(phi)), phi, atol=1e-9)
    assert np.allclose(lie.so3_log(np.eye(3)), 0)


def test_so3_log_small_angles():
    for s in [1e-6, 1e-9, 1e-12]:
        phi = np.array([0.3, -0.2, 0.9]) * s
        out = lie.so3_log(Rotation.from_rotvec(phi).as_matrix())
        assert np.all(np.isfinite(out))
        assert np.allclose(out, phi, atol=1e-14, rtol=1e-4), f"|phi|~{s}"


def test_so3_log_near_pi():
    for theta in [np.pi, np.pi - 1e-6, np.pi - 1e-3]:
        for _ in range(10):
            a = rng.normal(size=3)
            a /= np.linalg.norm(a)
            R = Rotation.from_rotvec(a * theta).as_matrix()
            phi = lie.so3_log(R)
            assert np.all(np.isfinite(phi)), f"NaN at theta={theta}"
            assert np.isclose(np.linalg.norm(phi), theta, atol=1e-6), f"theta={theta}"
            # at exactly pi, +a and -a are both valid; round-trip must still hold
            assert _rot_dist(Rotation.from_rotvec(phi).as_matrix(), R) < 1e-6, f"theta={theta}"


def test_so3_left_jacobian_bch():
    """exp((phi + d)^) ~= exp((J_l(phi) d)^) exp(phi^) to first order in d."""
    for _ in range(20):
        phi = _rand_rotvec(3.0)
        J = lie.so3_left_jacobian(phi)
        f = lambda d: Rotation.from_matrix(
            Rotation.from_rotvec(phi + d).as_matrix() @ Rotation.from_rotvec(phi).as_matrix().T
        ).as_rotvec()
        assert np.allclose(J, numeric_jacobian(f, np.zeros(3)), atol=1e-6)


def test_so3_left_jacobian_small_and_inverse():
    assert np.allclose(lie.so3_left_jacobian(np.zeros(3)), np.eye(3))
    assert np.allclose(lie.so3_left_jacobian(np.full(3, 1e-12)), np.eye(3))
    for _ in range(20):
        phi = _rand_rotvec(3.0)
        assert np.allclose(lie.so3_left_jacobian_inv(phi) @ lie.so3_left_jacobian(phi), np.eye(3), atol=1e-9)
    assert np.allclose(lie.so3_left_jacobian_inv(np.zeros(3)), np.eye(3))


# ------------------------------------------------------------------ SE(3)
def test_se3_hat():
    xi = rng.normal(size=6)
    assert np.allclose(lie.se3_hat(xi), _ref_se3_hat(xi))


def test_se3_exp_matches_expm():
    for _ in range(50):
        xi = np.r_[rng.normal(size=3), _rand_rotvec()]
        assert np.allclose(lie.se3_exp(xi), expm(_ref_se3_hat(xi)), atol=1e-10)
    assert np.allclose(lie.se3_exp(np.zeros(6)), np.eye(4))
    xi = np.r_[1.0, 2.0, 3.0, 1e-13, 0, 0]          # pure translation limit
    assert np.allclose(lie.se3_exp(xi)[:3, 3], [1, 2, 3])


def test_se3_log_roundtrip():
    for _ in range(50):
        xi = np.r_[rng.normal(size=3), _rand_rotvec()]
        assert np.allclose(lie.se3_log(lie.se3_exp(xi)), xi, atol=1e-8)
    T = np.eye(4)
    T[:3, 3] = [0.5, -1, 2]
    assert np.allclose(lie.se3_log(T), [0.5, -1, 2, 0, 0, 0])


def test_rho_is_not_translation():
    xi = np.r_[1.0, 0, 0, 0, 0, np.pi / 2]
    T = lie.se3_exp(xi)
    assert not np.allclose(T[:3, 3], xi[:3]), "rho should NOT equal t when rotating"


def test_se3_inv():
    for _ in range(20):
        T = _rand_T()
        assert np.allclose(lie.se3_inv(T) @ T, np.eye(4), atol=1e-12)


def test_se3_adjoint():
    for _ in range(20):
        T, xi = _rand_T(), rng.normal(size=6) * 0.3
        lhs = T @ expm(_ref_se3_hat(xi)) @ np.linalg.inv(T)
        rhs = expm(_ref_se3_hat(lie.se3_adjoint(T) @ xi))
        assert np.allclose(lhs, rhs, atol=1e-9)


def test_transform_point_jacobian():
    for _ in range(20):
        T, p = _rand_T(), rng.normal(size=3) * 3
        f = lambda d: (expm(_ref_se3_hat(d)) @ T @ np.r_[p, 1])[:3]
        J = lie.transform_point_jacobian(T, p)
        assert J.shape == (3, 6)
        assert np.allclose(J, numeric_jacobian(f, np.zeros(6)), atol=1e-6)


def test_se3_interpolate():
    for _ in range(20):
        T0, T1 = _rand_T(), _rand_T()
        assert np.allclose(lie.se3_interpolate(T0, T1, 0.0), T0, atol=1e-9)
        assert np.allclose(lie.se3_interpolate(T0, T1, 1.0), T1, atol=1e-8)
        # half-way twice == all the way (constant twist)
        H = lie.se3_interpolate(T0, T1, 0.5) @ np.linalg.inv(T0)
        assert np.allclose(H @ H @ T0, T1, atol=1e-8)
    # pure translation interpolates linearly
    T0, T1 = np.eye(4), np.eye(4)
    T1[:3, 3] = [2, 0, -4]
    assert np.allclose(lie.se3_interpolate(T0, T1, 0.25)[:3, 3], [0.5, 0, -1])
