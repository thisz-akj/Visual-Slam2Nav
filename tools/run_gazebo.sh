#!/usr/bin/env bash
# Open a scanned map in Gazebo:  tools/run_gazebo.sh [world_name]   (default: apt0)
V="$(cd "$(dirname "$0")/.." && pwd)"
export GZ_SIM_RESOURCE_PATH="$V/gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
exec gz sim "$V/gazebo/worlds/${1:-apt0}.sdf"
