#!/usr/bin/env bash
# Build Pangolin v0.8 + ORB-SLAM3 (RGB-D TUM example only) into third_party/, no sudo needed.
# Requires: build-essential cmake libeigen3-dev libopencv-dev libglew-dev libepoxy-dev libboost-serialization-dev libssl-dev
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TP="$ROOT/third_party"
PREFIX="$TP/install"
J=$(nproc)

echo "== Pangolin"
cmake -S "$TP/Pangolin" -B "$TP/Pangolin/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_CXX_FLAGS="-include cstdint" -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF \
      -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_FFMPEG=OFF
cmake --build "$TP/Pangolin/build" -j"$J"
cmake --install "$TP/Pangolin/build"

O="$TP/ORB_SLAM3"
# our changes: C++17, GCC 13 fix in LoopClosing.h, SaveMapPoints(), ORB_NO_VIEWER switch
if git -C "$O" apply --check "$ROOT/tools/orbslam3_ubuntu24.patch" 2>/dev/null; then
  echo "== applying tools/orbslam3_ubuntu24.patch"
  git -C "$O" apply "$ROOT/tools/orbslam3_ubuntu24.patch"
fi
for lib in DBoW2 g2o Sophus; do
  echo "== $lib"
  cmake -S "$O/Thirdparty/$lib" -B "$O/Thirdparty/$lib/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
  cmake --build "$O/Thirdparty/$lib/build" -j"$J"
done

echo "== vocabulary"
[ -f "$O/Vocabulary/ORBvoc.txt" ] || tar xzf "$O/Vocabulary/ORBvoc.txt.tar.gz" -C "$O/Vocabulary"

echo "== ORB-SLAM3"
cmake -S "$O" -B "$O/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$O/build" --target rgbd_tum -j"$J"
ls -la "$O/Examples/RGB-D/rgbd_tum"
echo "BUILD OK"
