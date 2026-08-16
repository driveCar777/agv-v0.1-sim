#!/usr/bin/env bash
# 模式二：Dev Sim — 旧 agv_sim_node（站点导航，已 DEPRECATED）
# 仅用于 legacy 回归。新开发请用 start_mock_stack.sh / start_full_sim_stack.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ROOT}/ros2_ws"
MAPS="${ROOT}/maps/stations_from_smap.json"
CFG="${ROOT}/config/devices.sim.yaml"
LOGDIR="${ROOT}/logs"
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

export ROS_DOMAIN_ID=30
export DELIVERY_ENV=dev
export ARM_MODE=simulation
export ARM_REAL_MOTION=0
export USE_SIM_CAMERAS=1
export DELIVERY_DEVICES_YAML="$CFG"
export MAPS_RW_DIR="${ROOT}/maps"
export ENABLE_LEGACY_BRIDGE=true
export AGV_BACKEND=mock
export REAL_ROBOT_ENABLED=false

echo "[warn] start_dev_sim_stack uses DEPRECATED agv_sim_node (LM stations)."
echo "[warn] Prefer: bash scripts/start_full_sim_stack.sh"

if ! pgrep -f "robokit_mock_server" >/dev/null 2>&1; then
  nohup ros2 run agv_bridge robokit_mock_server --ros-args -- --host 127.0.0.1 \
    > "${LOGDIR}/robokit_mock.log" 2>&1 &
  sleep 1
fi

if ! pgrep -f "agv_sim_node" >/dev/null 2>&1; then
  nohup ros2 run agv_bridge agv_sim_node \
    --ros-args -p stations_file:="${MAPS}" \
    > "${LOGDIR}/agv_sim_node.log" 2>&1 &
  sleep 1
fi

exec ros2 run delivery_web dashboard_node \
  --ros-args \
  -p http_host:=0.0.0.0 \
  -p demo_port:=19999 \
  -p debug_port:=1999 \
  -p stations_file:="${MAPS}"
