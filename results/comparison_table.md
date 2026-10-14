# ORB-SLAM3 vs SuperPoint + LightGlue

Both pipelines see exactly the same frames (`runs/cmp/assoc/`): every 2nd frame of the 30 Hz real sequences, every frame of the 15 Hz twin recordings. Both use RGB-D, keyframes, PnP tracking, loop closure and pose-graph optimization, so the main difference is the features: hand-crafted ORB + bag of words vs learned SuperPoint + LightGlue. Errors are SE(3)-aligned ATE in cm.

| Dataset | Pipeline | Frames | Tracked | Breaks | Longest continuous | ATE | Max | ATE per segment | Loops | Merges | ms / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|
| TUM fr3 office (real) | ORB-SLAM3 | 1244 | 100% | 0 | 100% | 1.56 | 7.57 | 1.56 | 1 | 0 | 18 |
| TUM fr3 office (real) | SuperPoint+LightGlue | 1244 | 100% | 0 | 100% | 2.66 | 5.72 | 2.66 | 34 | 0 | 296 |
| Twin run A: cam 0.37 m, 2 cm | ORB-SLAM3 | 1311 | 100% | 5 | 20% | - | - | 1.22 | 0 | 0 | 13 |
| Twin run A: cam 0.37 m, 2 cm | SuperPoint+LightGlue | 1311 | 100% | 0 | 100% | 2.07 | 4.79 | 2.07 | 10 | 0 | 457 |
| Twin run B: + slow turns | ORB-SLAM3 | 1496 | 99% | 6 | 20% | - | - | 1.39 | 0 | 0 | 13 |
| Twin run B: + slow turns | SuperPoint+LightGlue | 1496 | 100% | 0 | 100% | 1.30 | 3.71 | 1.30 | 11 | 0 | 486 |
| Twin run D: cam 1.2 m, 2 cm | ORB-SLAM3 | 1172 | 100% | 4 | 36% | - | - | 0.94 | 0 | 2 | 16 |
| Twin run D: cam 1.2 m, 2 cm | SuperPoint+LightGlue | 1172 | 100% | 0 | 100% | 1.33 | 4.15 | 1.33 | 18 | 1 | 539 |
| Twin run C: cam 1.2 m, HD | ORB-SLAM3 | 1136 | 100% | 0 | 100% | 2.75 | 6.16 | 2.75 | 0 | 0 | 16 |
| Twin run C: cam 1.2 m, HD | SuperPoint+LightGlue | 1136 | 100% | 0 | 100% | 1.19 | 3.11 | 1.19 | 17 | 0 | 569 |
| BundleFusion apt0 (real) | ORB-SLAM3 | 4280 | 100% | 0 | 100% | 10.75 | 76.81 | 10.75 | 5 | 0 | 25 |
| BundleFusion apt0 (real) | SuperPoint+LightGlue | 4280 | 100% | 0 | 100% | 8.29 | 17.24 | 8.29 | 643 | 0 | 452 |

- **Tracked**: share of frames with a pose. **Breaks**: jumps > 0.2 m, i.e. a new map was started.
- **ATE**: only when the whole run is one map. **ATE per segment**: frame-weighted mean over segments.
- **ms / frame**: median tracking time on this CPU-only laptop (Intel Core Ultra 5 225H). ORB-SLAM3 runs mapping and loop closing in other threads; our pipeline runs everything in one thread.
- apt0 errors are against BundleFusion's reference poses, not true ground truth.
