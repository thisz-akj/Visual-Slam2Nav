# Lab 0: Rigid motion and Lie groups

**Read first:** 14L ch3 (rigid-body motion) and ch4 (Lie groups/algebras).
Solà, Deray, Atchuthan 2018, *A micro Lie theory for state estimation in
robotics* (arXiv:1812.01537), sections I–IV. It's short and the clearest treatment available.

**You build:** `vslam/lie.py`  **Verify:** `python check.py 0`

---

## Why this is Lab 0

Every later lab optimizes camera poses: PnP (Lab 4), BA (Lab 5), pose graphs (Lab 8).
An optimizer takes a step `x ← x + Δx`, but a rotation matrix plus a small matrix
is no longer a rotation. Each fix you might try fails in its own way:

| Parameterization | Problem |
|---|---|
| 9 entries of R | 6 constraints (RᵀR = I, det = 1) that the optimizer doesn't know about |
| Euler angles | Gimbal lock: the Jacobian loses rank at ±90° pitch |
| Quaternion | 4 numbers for 3 DoF, needs a unit-norm constraint, and q and −q are the same rotation |
| Axis-angle as the state (what your old `ba.py` does) | Singular at θ = π, and the Jacobians are messy |

**The Lie-group answer:** keep the state as a proper matrix `T ∈ SE(3)`, but optimize
a small 6-vector `δ` *around* the current estimate:

```
T  ←  exp(δ^) · T          (left perturbation, our project convention)
```

Near `δ = 0` the map `exp` is smooth and has full rank, so you never hit a
singularity. After each step, `δ` goes back to zero. This is what ORB-SLAM, g2o,
Ceres (manifolds) and GTSAM all do.

---

## Part A: SO(3)

**Hat and vee.** `φ^` is the skew-symmetric matrix with `φ^ a = φ × a`. `vee` undoes it.

**Exponential map (Rodrigues).** With `θ = ‖φ‖` and `a = φ/θ`:

```
exp(φ^) = cos θ · I + (1 − cos θ) · a aᵀ + sin θ · a^
```

*Numerics:* for tiny `θ`, dividing by `θ` blows up. Use the Taylor form
`I + φ^ + ½(φ^)²` when `θ < 1e-8`. Think about why this is accurate enough.

**Logarithm map.** You derive this one. Hints:
1. Take the trace of the Rodrigues formula. What does `tr(R)` tell you about `θ`?
2. The antisymmetric part `R − Rᵀ` contains `a sin θ`.
3. **Edge case `θ → π`:** `sin θ → 0`, so step 2 divides 0 by 0. Look at the
   *symmetric* part of R near π instead. Work out what `(R + I)/2` equals when
   `θ = π` exactly. Then ask what changes at `θ = π − 1e-6`, where `R` is no longer
   symmetric. The test checks both cases and needs ~1e-6 accuracy. A version that
   is correct at exactly π can still fail just below it. Also: how do you pick
   the *sign* of the axis when θ < π?

**Left Jacobian of SO(3).** It shows up in two places: the SE(3) exponential, and the
first-order BCH approximation `exp((φ + δφ)^) ≈ exp((J_l(φ) δφ)^) · exp(φ^)`.
14L eq. 4.39 gives:

```
J_l(φ) = (sin θ / θ) I + (1 − sin θ / θ) a aᵀ + ((1 − cos θ) / θ) a^
```

*Derivation exercise (paper only):* write `J_l = Σ_{n≥0} (φ^)ⁿ / (n+1)!` and
collapse the series using `(a^)² = a aᵀ − I` and `(a^)³ = −a^`.

## Part B: SE(3)

```
T = [R t; 0 1],   xi = [rho, phi] ∈ R⁶,   xi^ = [phi^ rho; 0 0] (4×4)
exp(xi^) = [ exp(phi^)   J_l(phi)·rho ; 0  1 ]
```

**Important:** `rho` is *not* the translation. The translation is `t = J_l(phi) rho`.
So the log map needs `J_l⁻¹`. Either solve the 3×3 linear system or use the
closed form in 14L eq. 4.42.

**Adjoint.** `Ad_T` is the 6×6 matrix that moves a perturbation across T:

```
T · exp(xi^) · T⁻¹ = exp((Ad_T xi)^)          Ad_T = [ R   t^R ; 0   R ]
```

(This form is for `xi = [rho, phi]`. Check the block placement yourself.)

## Part C: the Jacobian you'll use 100× later

For a point `p` and `q = T p`, find the derivative of `q` with respect to a left perturbation:

```
∂(exp(δ^) T p) / ∂δ  at δ = 0     → 3×6 matrix
```

Derive it by expanding `exp(δ^) ≈ I + δ^`. You should get `[ I  −q^ ]`.
In Lab 1 you chain this with the projection Jacobian to get the BA/PnP Jacobian
(14L eq. 7.46).

---

## Part D: pose interpolation (`se3_interpolate`)

`T(s) = exp(s · log(T₁T₀⁻¹)) · T₀`. This moves along a *constant twist*, the
screw motion a rigid body follows at constant velocity. Compare it with
interpolating the rotation (slerp) and the translation (lerp) separately: do they give the same path?

---

## fr1_xyz milestone: evaluate ground truth properly

Ground truth is recorded at 100 Hz and images at 30 Hz, so their timestamps never
line up. Your old `dataset.py:gt_pose` takes the *nearest* GT sample and accepts a
gap of up to 20 ms. Write `labs/answers/00_milestone.py` to:

1. For every RGB timestamp, find the two GT samples that bracket it and use
   `se3_interpolate` to get the pose at exactly that time.
2. Compare with the nearest-neighbour pose. Report the mean and max of the translation
   error (mm) and rotation error (deg). Plot the error over time against the camera speed.
3. Your prototype reaches an ATE of a few cm. Is nearest-neighbour GT error negligible
   against that? What about for a system that reaches about 1 cm?
4. Bonus: the TUM camera and mocap clocks may be offset. Search a time offset
   `Δt ∈ [−30, 30] ms` that minimizes your prototype's ATE. Is there one?

Put the numbers in `results/REPORT.md` under *Lab 0*.

## Experiments (write `labs/answers/00_experiments.py`)

1. **Why manifolds matter.** Compose 100 000 random small rotations (θ ~ 0.01 rad)
   in two ways: (a) `R ← R + R @ hat(φ)` (first-order "Euler" update, no exp), and
   (b) `R ← R @ so3_exp(φ)`. Plot `‖RᵀR − I‖` against step for both.
2. **BCH error.** Fix a random `a` with ‖a‖ ≈ 1 rad and sweep a small `b`
   (‖b‖ from 1e-6 to 1e-1). Compare `exp((a+b)^)` with two approximations:
   (i) the naive `exp(b^)·exp(a^)`, and (ii) the first-order BCH `exp((J_l(a) b)^)·exp(a^)`.
   Plot `‖log(approx · exp((a+b)^)ᵀ)‖` against `‖b‖` on log-log axes. What slopes
   do you expect for (i) and (ii), and why?
3. **Interpolation.** Interpolate between two poses using `exp(s·log(T₂T₁⁻¹))·T₁`
   for s ∈ [0,1], then naively by linearly blending the matrices. Plot both camera
   paths and the determinant of the blended R.

## Questions (answer in `labs/answers/00.md`)

1. Why is `so3_log` ambiguous at exactly θ = π, and why doesn't that ambiguity
   matter for optimization done with left perturbations?
2. With a left perturbation `exp(δ^)·T_cw`, what frame is `δ` expressed in: the
   world or the camera? What changes if you use `T_cw·exp(δ^)` instead?
3. Your old `ba.py` optimizes `rvec` (axis-angle) directly. Give one concrete
   situation where that parameterization behaves badly, and explain why the
   perturbation approach avoids it.
4. Why is the SE(3) log translation `rho` different from `t`? When are they equal?
5. Name two places in a SLAM system where you need the adjoint. (Hint: covariances,
   and pose-graph edge Jacobians in Lab 8.)
6. A pose has 6 DoF and a monocular *similarity* transform has 7. What is the
   7th, and what Lie group will we need in Lab 8?
