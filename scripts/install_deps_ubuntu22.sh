#!/usr/bin/env bash
# Ubuntu 22.04 — 安装 ROS2 Humble 及仿真所需依赖
set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
  SUDO=""
else
  SUDO="sudo"
fi

echo "==> ROS2 Humble (若已安装可跳过)"
if ! command -v ros2 >/dev/null 2>&1; then
  $SUDO apt-get update
  $SUDO apt-get install -y curl gnupg lsb-release
  $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  $SUDO apt-get update
  $SUDO apt-get install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    $SUDO rosdep init || true
    rosdep update
  fi
fi

echo "==> 构建依赖"
$SUDO apt-get install -y \
  python3-pip python3-opencv \
  ros-humble-cv-bridge ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-geometry-msgs \
  ros-humble-std-srvs

echo "Done. Source: source /opt/ros/humble/setup.bash"
