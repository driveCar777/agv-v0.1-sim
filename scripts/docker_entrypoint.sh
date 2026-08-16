#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
export AGV_BACKEND="${AGV_BACKEND:-tcp}"
export AGV_HOST="${AGV_HOST:-host.docker.internal}"
echo "=== AGV V0.1 ROS2 Humble ==="
echo "AGV_BACKEND=$AGV_BACKEND  AGV_HOST=$AGV_HOST"
echo "Tip: colcon build --symlink-install"
echo "     ros2 run agv_control agv_control_bridge --ros-args -p backend:=tcp -p agv_host:=$AGV_HOST"
exec "$@"
