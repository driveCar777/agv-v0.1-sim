#!/usr/bin/env bash
# 全量仿真 — 底盘 + 传感器 + 视觉 Mock + Mission + Web
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ROOT}/ros2_ws"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

export ROS_DOMAIN_ID=30
export AGV_BACKEND=mock
export REAL_ROBOT_ENABLED=false
export DELIVERY_ENV=mock
export ARM_MODE=simulation
export ARM_REAL_MOTION=0
export USE_SIM_CAMERAS=1
export AGV_MAP_YAML="${ROOT}/maps/empty_room.yaml"

exec ros2 launch agv_bringup agv_full_sim_bringup.launch.py
