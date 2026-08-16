#!/usr/bin/env bash
# 离线单测（可在无 ROS2 环境下先跑纯逻辑；有 ROS2 时再 source install）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ROOT}/ros2_ws"
cd "$WS"

export PYTHONPATH="\
${WS}/src/agv_control:\
${WS}/src/agv_mission:\
${WS}/src/agv_lidar:\
${WS}/src/agv_ultrasonic:\
${WS}/src/agv_simulation:\
${WS}/src/agv_vision_mock:\
${WS}/src/agv_tf:\
${WS}/src/agv_safety:\
${PYTHONPATH:-}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi

python3 -m pytest \
  src/agv_control/agv_control/tests \
  src/agv_mission/agv_mission/tests \
  src/agv_lidar/agv_lidar/tests \
  src/agv_ultrasonic/agv_ultrasonic/tests \
  src/agv_simulation/agv_simulation/tests \
  src/agv_vision_mock/agv_vision_mock/tests \
  src/agv_safety/agv_safety/tests \
  src/agv_tf/agv_tf/tests \
  -v --tb=short
