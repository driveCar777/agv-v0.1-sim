#!/usr/bin/env bash
# 编译 V0.1 仿真工作空间（完全离线包）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ROOT}/ros2_ws"
source /opt/ros/humble/setup.bash
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
colcon build \
  --packages-select \
    delivery_interfaces delivery_web agv_bridge camera_bridge \
    agv_control agv_navigation agv_safety agv_mission agv_bringup \
    agv_tf agv_lidar agv_ultrasonic agv_simulation agv_vision_mock \
  --symlink-install
echo ""
echo "Build OK. Source:"
echo "  source ${WS}/install/setup.bash"
echo ""
echo "Then:"
echo "  bash scripts/start_mock_stack.sh       # P1 底盘+Web"
echo "  bash scripts/start_nav_sim_stack.sh    # P2 +Nav2+Safety"
echo "  bash scripts/start_full_sim_stack.sh   # 全量仿真"
