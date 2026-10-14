#!/usr/bin/env bash
# Run both pipelines (ORB-SLAM3 and SuperPoint+LightGlue) on the same frames of every dataset.
# Results: runs/cmp/<pipeline>_<dataset>/ ; summary via tools/compare_report.py
set -uo pipefail
V="$(cd "$(dirname "$0")/.." && pwd)"
cd "$V"
O="$V/third_party/ORB_SLAM3"
OUT="$V/runs/cmp"
mkdir -p "$OUT/assoc"
export ORB_NO_VIEWER=1
export LD_LIBRARY_PATH="$V/third_party/install/lib:$O/lib:$O/Thirdparty/DBoW2/lib:$O/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

# name | data dir | association file | camera yaml | stride
DATASETS=(
  "fr3_office|data/rgbd_dataset_freiburg3_long_office_household|$O/Examples/RGB-D/associations/fr3_office.txt|$O/Examples/RGB-D/TUM3.yaml|2"
  "sim_A|data/sim_apt0_loop|data/sim_apt0_loop/associations.txt|data/sim_apt0_loop/camera.yaml|1"
  "sim_B|data/sim_apt0_slowturn|data/sim_apt0_slowturn/associations.txt|data/sim_apt0_slowturn/camera.yaml|1"
  "sim_D|data/sim_apt0_tall|data/sim_apt0_tall/associations.txt|data/sim_apt0_tall/camera.yaml|1"
  "sim_C|data/sim_apt0hd_tall|data/sim_apt0hd_tall/associations.txt|data/sim_apt0hd_tall/camera.yaml|1"
  "apt0|data/apt0/apt0|data/apt0/apt0/associations.txt|data/apt0/apt0.yaml|2"
)

for entry in "${DATASETS[@]}"; do
  IFS='|' read -r name data assoc cfg stride <<< "$entry"
  a="$OUT/assoc/$name.txt"
  grep -v '^#' "$assoc" | awk -v s="$stride" 'NR % s == 1 || s == 1' > "$a"     # identical frames for both
  echo "=== $name: $(wc -l < "$a") frames (stride $stride)"

  d="$OUT/orbslam3_$name"
  if [ ! -s "$d/CameraTrajectory.txt" ]; then
    mkdir -p "$d" && cfg_abs="$(realpath "$cfg")"; data_abs="$(realpath "$data")"
    mkdir -p "$d" && (cd "$d" && "$O/Examples/RGB-D/rgbd_tum" "$O/Vocabulary/ORBvoc.txt" "$cfg_abs" "$data_abs" "$a" > run.log 2>&1)
  fi
  .venv/bin/python tools/eval_segments.py --run "$d" --gt "$data/groundtruth.txt" | tail -2

  d="$OUT/splg_$name"
  if [ ! -s "$d/CameraTrajectory.txt" ]; then
    .venv/bin/python -m splg_slam.run --data "$data" --assoc "$a" --config "$cfg" --out "$d" 2>&1 \
      | grep -E "^(New map|Loop|Merge|done|median|total)"
  fi
  .venv/bin/python tools/eval_segments.py --run "$d" --gt "$data/groundtruth.txt" | tail -2
done
echo "ALL DONE"
