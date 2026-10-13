# vslam: a complete visual SLAM system on TUM fr1_xyz

**The project:** build a keyframe SLAM system (ORB-SLAM-style) from first principles and
evaluate it thoroughly on the one dataset we have, `rgbd_dataset_freiburg1_xyz`.

**Final deliverables**
1. `python -m vslam.run --mode mono|rgbd` runs the whole system on fr1_xyz: tracking, local
   mapping, relocalization and loop closure.
2. `results/REPORT.md` contains an evaluation table (ATE, RPE, tracked %, fps) across system
   configurations and stress tests, plus plots of the trajectory and map.
3. Your own implementation of every geometric and optimization core, each one tested
   against ground truth.

## What fr1_xyz gives us (measured)

| Data | Used for |
|---|---|
| 798 RGB frames, 640×480, 30 Hz, 30 s | Everything |
| 798 registered depth frames | Metric scale, **true correspondences** for testing matchers, RGB-D mode |
| Ground truth at 100 Hz (motion capture) | Evaluation, and labels for every lab's test |
| Accelerometer at ~500 Hz | Gravity direction, so the map can be aligned upright (Lab 9) |
| 9.2 m path in a ~0.5 m box, rotation up to 29° | Short baselines make monocular initialization and scale hard |
| **43 revisits** (>5 s apart, <3 cm, <5°) | Ground-truth labels for place recognition and loop closure |

**One sequence, many test conditions.** We make harder variants by transforming the
sequence instead of downloading new data:

| Stress test | How | Tests |
|---|---|---|
| `skip-k` | Use every k-th frame (k = 2, 3, 5) | Fast motion and wide-baseline matching |
| `blackout` | Replace 30 frames with black images | Relocalization |
| `blur` / `noise` | Gaussian blur or noise on images | Feature robustness, and direct vs feature methods |
| `reverse` | Play the sequence backwards | A sanity check that nothing is overfit to the motion |
| `no-depth` vs `depth` | Mono vs RGB-D | What metric scale buys you |

**Rule: no black boxes.** You re-implement every OpenCV/SciPy routine that does real
geometry. The library versions stay only as references in the tests.

**Rule: synthetic, then real.** Every lab is verified on a synthetic world first, where
a bug shows up as a wrong number, and then ends with a **fr1_xyz milestone**.

## How each lab works
1. **Read** the listed chapters (`14L` = Gao Xiang, *14 Lectures on Visual SLAM*, 2nd ed.,
   `H&Z` = Hartley & Zisserman, *Multiple View Geometry*, 2nd ed.).
2. **Read** `labs/NN_*.md` for the theory, conventions and derivations.
3. **Implement** the `TODO`s in `vslam/*.py`.
4. **Verify** with `python check.py NN`.
5. **Hit the fr1_xyz milestone** and record the numbers in `results/REPORT.md`.
6. **Answer** the lab questions in `labs/answers/NN.md`, then ask Claude for a review.

## Labs

| # | Lab | You build | Read | fr1_xyz milestone |
|---|-----|-----------|------|-------------------|
| 0 | Rigid motion & Lie groups | `lie.py`: SO(3)/SE(3) exp/log, Jacobians, adjoint, pose interpolation | 14L ch3–4 · Solà 2018 *micro Lie theory* | Interpolate ground truth on SE(3) at camera timestamps; measure the error of nearest-neighbour GT |
| 1 | Cameras & depth | `camera.py`: projection, distortion and its inverse, back-projection, **projection Jacobians** | 14L ch5 · H&Z ch6–7 | Fuse 50 depth frames with GT poses into one coloured point cloud. Sharp edges mean calibration and conventions are right |
| 2 | Features & matching | `features.py`: grid-bucketed ORB, Hamming matching, ratio + mutual test | 14L ch7.1–7.2 · Rublee 2011 | **Precision/recall of your matcher**, scored against true matches derived from depth + GT |
| 3 | Two-view geometry | `twoview.py`: normalized 8-pt, E→(R,t)×4, cheirality, DLT triangulation, RANSAC | H&Z ch9–12 · Hartley 1997 · Nistér 2004 | Relative rotation error and translation *direction* error against frame gap (1…30) |
| 4 | Least squares & PnP | `optim.py`: GN, LM, IRLS. `pnp.py`: DLT + GN on SE(3), PnP-RANSAC | 14L ch6, 7.7 · Madsen et al. · EPnP 2009 | **First odometry:** frame-to-frame RGB-D odometry (depth gives the 3D points, PnP the pose). ATE/RPE |
| 5 | Bundle adjustment | `ba.py`: sparse normal equations, Schur complement, gauge fixing, robust kernels | 14L ch9 · Triggs 2000 | Windowed BA on Lab 4's odometry: ATE before/after, and runtime against the old scipy BA |
| 6 | Monocular VO system | `tracking.py`, `mapping.py`: frames relative to keyframes, covisibility graph, KF policy, local BA | ORB-SLAM 2015 §III–VI · 14L ch13 | Mono VO beats your prototype's ATE. Run `skip-2/3/5` |
| 7 | Place recognition & relocalization | `bow.py`: vocabulary, TF-IDF, inverted index. Relocalization | DBoW2 2012 · Nistér & Stewénius 2006 · 14L ch11 | **PR curve on the 43 GT revisits.** Survive `blackout` |
| 8 | Loop closure | `sim3.py`, `posegraph.py`: Sim(3) RANSAC, Sim(3) pose graph, map fusion | Strasdat 2010 · ORB-SLAM §VII · 14L ch10 | Scale drift before/after loops; ATE of mono VO vs SLAM |
| 9 | Full system & report | Global BA, RGB-D mode, gravity alignment from accelerometer, `vslam.run`, `vslam.eval` | Sturm 2012 · ORB-SLAM2 2017 | `REPORT.md`: every config × every stress test |
| 10 | *(stretch)* Direct methods | `direct.py`: photometric SE(3) alignment, DSO-lite tracking | Engel 2017 · Baker & Matthews 2004 | Features vs direct under `blur` and `noise` |

The labs build on each other: Lab 4 gives you a working odometry with real numbers early,
and every later lab improves a system that already runs.

## Survey reading (any time)
- Cadena et al. 2016, *Past, Present, and Future of SLAM*
- Dellaert & Kaess, *Factor Graphs for Robot Perception*
- Barfoot, *State Estimation for Robotics* (ch7–8)
- After the project: DROID-SLAM (2021), MASt3R-SLAM (2025), for contrast with learned methods

## Layout
```
ROADMAP.md          this file
check.py            test runner: python check.py <lab>
labs/NN_*.md        theory notes, derivations, milestone, questions
labs/answers/NN.md  your answers (reviewed by Claude)
vslam/              the package you build
tests/test_NN_*.py  ground-truth tests per lab
results/REPORT.md   the running evaluation table (grows each lab)
*.py (root)         earlier VO prototype; kept as the baseline to beat, ported in Lab 6
```

## Conventions (fixed for the whole project)
- `T_cw` maps world → camera: `X_c = R_cw X_w + t_cw`. `T_wc = T_cw⁻¹` is the camera pose in the world.
- se(3) vector `xi = [rho, phi]` (translation part first, as in 14L).
- **Left perturbation:** `T ← exp(δ^) · T`. Every Jacobian in the project uses this.
- Pixels: `u` = column, `v` = row. Camera frame: x right, y down, z forward.
- TUM depth: `depth_png / 5000` = metres (0 = missing).
