#!/usr/bin/env bash
# P5 — 真车（必须显式 REAL_ROBOT_ENABLED=true）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ROOT}/ros2_ws"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

if [[ "${REAL_ROBOT_ENABLED:-false}" != "true" ]]; then
  echo "Refusing real stack: set REAL_ROBOT_ENABLED=true first."
  exit 1
fi

export ROS_DOMAIN_ID=30
export AGV_BACKEND=real
export REAL_ROBOT_ENABLED=true

exec ros2 launch agv_bringup agv_real_bringup.launch.py \
  agv_host:="${AGV_HOST:-192.168.192.5}"
