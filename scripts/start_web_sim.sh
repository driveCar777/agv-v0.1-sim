#!/usr/bin/env bash
# 启动 API 仿真 Web（无需 ROS2）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="${ROOT}/ros2_ws/src/agv_bridge:${PYTHONPATH:-}"
exec python3 scripts/run_web_sim.py
