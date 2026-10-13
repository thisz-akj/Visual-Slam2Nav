# vslam: evaluation report on TUM fr1_xyz

All ATE values are RMSE after Sim(3) alignment (Umeyama), in cm, unless marked metric (SE(3)).
Each row is added when its lab is done. **Never delete rows:** the history is the point.

## Baseline: original prototype (`vo_map.py`, root dir)

Run 2026-10-13, seed 0, all 798 frames.

| Config | Tracked | KFs | Frame ATE | KF ATE | Scale | fps | Notes |
|---|---|---|---|---|---|---|---|
| mono, no BA (`--no-ba`) | 795/798 | 48 | 9.19 | 10.94 | 0.839 | 22.5 | |
| mono, local BA (scipy) | 781/798 | 53 | **8.19** | 8.57 | 0.647 | 15.5 | 14 frames lost |

Open questions this baseline raises (answer them as the labs progress):
- With BA, the Sim(3) scale drops from 0.84 to 0.65. Is the map shrinking over time (scale drift)? → Labs 6 and 8
- BA loses 14 frames that the no-BA run tracks. Why? → Lab 6 (BA changes poses under the motion model)
- Frame poses are never corrected after BA (see the earlier review). How much of the 8.19 is that? → Lab 6

## Reference system: ORB-SLAM3 RGB-D on fr3_long_office_household

Run 2026-10-13. 2,488 frames, 22.1 m path, one loop detected and closed. Outputs are in `runs/fr3_office/`.

| System | Mode | ATE RMSE (SE3) | median | max | tracking time | sparse pts | dense map |
|---|---|---|---|---|---|---|---|
| ORB-SLAM3 | RGB-D | **1.01 cm** | 0.86 cm | 3.00 cm | 34 ms/frame | 14,578 | 580k pts / 1.14M tris (1 cm TSDF, 830 frames) |

This is the target our own system works towards (Labs 6–9).

## Digital twin: ORB-SLAM3 on scout_bot driving the scanned apartment (Gazebo)

Twin is built from the apt0 ORB-SLAM3 run (2 cm TSDF, 300k-triangle visual mesh, 4K baked texture).
The robot camera is at 0.37 m; the human scanner was at about 1.45 m. The route is a 17 m loop
back to the start. Ground truth is exact (from Gazebo). Run with
`tools/sim_collect.py` + `tools/run_orbslam3.sh` + `tools/eval_segments.py`.

| Run | Change | Frames | Breaks | Longest segment | Per-segment ATE |
|---|---|---|---|---|---|
| A | baseline, turns ≤ 0.9 rad/s | 1311 | 5 | 3.4 m (20%) | 0.8–1.7 cm |
| B | turns ≤ 0.4 rad/s | 1496 | 5 | 3.5 m (20%) | 0.7–2.1 cm |
| D | camera at 1.2 m (scanner height), turns ≤ 0.9 rad/s | 1172 | 3 (+2 map merges) | 5.5 m (36%) | 0.7–1.4 cm; one merged segment drifts to 35 cm |

| C | D + sharper twin `apt0_hd` (1 cm TSDF, 1M tris, 8K per-triangle texture from the 4M-tri mesh) | 1136 | **0** | **14.9 m (100%)** | **2.08 cm global ATE** (max 5.5 cm, 0.14% of distance) |

Run D: ORB keypoints/frame went from a median of 529 to 1247 (real: 1363), and the 10th percentile from 45 to 228. Place recognition
merged maps for the first time. **Conclusion: the twin is only reliable from the viewpoints it was scanned
from.** About 72 frames still have fewer than 100 keypoints, so texture (run C) is the next suspect.

Run C: keypoints median 1473, 10th percentile 1037, 1 frame under 100. One continuous map for the whole loop.
**Conclusion: viewpoint coverage (D) plus texture resolution (C) together close the gap.** Caveat: the twin is
now *easier* than reality at the low end (10th percentile 1037 against 583 real), because the simulated
camera has no motion blur, noise or exposure changes. The next realism step is to model those.
No loop closure fired: the loop ends 0.26 m from the start, but the drift stayed small (max 5.5 cm).

*Invalid first attempt at C (kept in `*/INVALID_sim_apt0hd_tall_broken_texture`):* the atlas UVs had
the wrong v orientation for Gazebo, so every triangle showed another triangle's colours. The resulting
"confetti" texture gave 1500 keypoints in every frame and a falsely perfect 3.1 cm. It was caught by
comparing keypoint statistics with the real data and looking at the frames. Fixed by using top-down v
(Gazebo convention) and a Morton-ordered atlas.

**Findings**
- While tracking, ORB-SLAM3 is accurate (about 1 cm, ~0.5% of distance): perfect sim depth helps.
- It is not robust. Tracking breaks 5× and the maps are never merged, so no loop closure.
- The breaks happen at the **same places** in A and B (within 1–9 cm), so motion is not the cause; the scene is.
- Texture: sim frames have a median of 529 ORB keypoints (10th percentile 45) against 1363 (10th percentile 583) on the real apt0 frames.
- The frames before each break show scan defects: holes, jagged edges, blurry baked texture, and
  undersides the 1.45 m-high scanner never saw.

Next: C = sharper twin (1 cm voxels, more triangles, 8K texture), D = camera at scanner height (~1.2 m).

## Lab 0: ground-truth interpolation
| Method | mean trans. err (mm) | max trans. err (mm) | mean rot. err (deg) | max rot. err (deg) |
|---|---|---|---|---|
| nearest GT sample | | | | |
| best clock offset Δt | | | | |

## Lab 1: calibration check
## Lab 2: matcher precision/recall
## Lab 3: two-view accuracy vs frame gap
## Lab 4: RGB-D frame-to-frame odometry
## Lab 5: windowed BA
## Lab 6: monocular VO system
## Lab 7: place recognition (43 GT revisits)
## Lab 8: loop closure
## Lab 9: full system × stress tests
