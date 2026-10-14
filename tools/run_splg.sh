#!/usr/bin/env bash
# Pipeline 2: SuperPoint + LightGlue RGB-D SLAM on a TUM-format sequence, then dense map + evaluation.
#   tools/run_splg.sh <dataset_dir> <assoc.txt> <camera.yaml> <out_dir> [stride]
# (same arguments as tools/run_orbslam3.sh, so both pipelines run on identical inputs)
set -euo pipefail
V="$(cd "$(dirname "$0")/.." && pwd)"
DATA="$(realpath "$1")"; ASSOC="$(realpath "$2")"; CFG="$(realpath "$3")"; OUT="$4"; STRIDE="${5:-1}"
cd "$V"
.venv/bin/python -m splg_slam.run --data "$DATA" --assoc "$ASSOC" --config "$CFG" --out "$OUT" --stride "$STRIDE"
.venv/bin/python tools/build_map.py --data "$DATA" --run "$OUT" --assoc "$ASSOC" --config "$CFG" ${MAP_ARGS:-}
.venv/bin/python tools/eval_segments.py --run "$OUT" --gt "$DATA/groundtruth.txt"
