#!/usr/bin/env bash
# Run ORB-SLAM3 RGB-D on a TUM sequence, then build the maps.
#   tools/run_orbslam3.sh                       # fr3_long_office_household, live viewer
#   ORB_NO_VIEWER=1 tools/run_orbslam3.sh       # headless
#   tools/run_orbslam3.sh <dataset_dir> <assoc.txt> <TUMx.yaml> <out_dir>
set -euo pipefail
V="$(cd "$(dirname "$0")/.." && pwd)"
O="$V/third_party/ORB_SLAM3"
DATA="$(realpath "${1:-$V/data/rgbd_dataset_freiburg3_long_office_household}")"
ASSOC="$(realpath "${2:-$O/Examples/RGB-D/associations/fr3_office.txt}")"
CFG="$(realpath "${3:-$O/Examples/RGB-D/TUM3.yaml}")"
OUT="${4:-$V/runs/fr3_office}"

export LD_LIBRARY_PATH="$V/third_party/install/lib:$O/lib:$O/Thirdparty/DBoW2/lib:$O/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$OUT" && cd "$OUT"
# ORB-SLAM3 sometimes crashes while closing the viewer, after everything is saved: don't stop on that
"$O/Examples/RGB-D/rgbd_tum" "$O/Vocabulary/ORBvoc.txt" "$CFG" "$DATA" "$ASSOC" 2>&1 | tee run.log \
  || echo "warning: ORB-SLAM3 exited with an error (often harmless at shutdown)"
[ -s CameraTrajectory.txt ] || { echo "no trajectory saved, stopping"; exit 1; }

cd "$V"
.venv/bin/python tools/build_map.py --data "$DATA" --run "$OUT" --assoc "$ASSOC" --config "$CFG" ${MAP_ARGS:-}
