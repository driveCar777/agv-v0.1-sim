# ROS2 Humble（Docker）+ 实车切换

本机已有 `agv_ros2_ws/docker`（Humble + Nav2 + Gazebo），但镜像未预先构建。  
V0.1 使用更轻量的 **`docker/Dockerfile`**（Humble + Nav2，无 Gazebo），专注 **control/planning → 3055**，便于上实车。

## 架构（仿真 = 实车协议）

```text
Nav2 / teleop
    → /cmd_vel
    → agv_control_bridge (backend=tcp|real)
    → Robokit TCP 3055/3056 + 4005 lock
    → Mock(127.0.0.1)  或  真车(AGV_HOST)
```

Web 真场景仿真（宿主机）：`scripts/start_web_sim.bat` → :19999 + Mock :19204-19207

## 1. 启动 Docker Desktop

确保 Docker Desktop 运行中。

## 2. 宿主机先起 Mock（推荐）

```bat
scripts\start_web_sim.bat
```

浏览器：http://127.0.0.1:19999/

## 3. 导出办公室地图给 Nav2

```bat
set PYTHONPATH=%CD%\ros2_ws\src\agv_bridge
python scripts\export_smap_to_navmap.py --out maps\office_nav
```

## 4. 进入 Humble 容器并编译

```bat
scripts\start_ros2_docker.bat
```

容器内：

```bash
cd /ws
colcon build --symlink-install --packages-select agv_control agv_bridge agv_navigation
source install/setup.bash

# 控制桥：连宿主机 Mock（验证与真车相同的 3055）
ros2 run agv_control agv_control_bridge --ros-args \
  -p backend:=tcp -p agv_host:=host.docker.internal

# 另开终端发速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

## 5. 上实车测试（同一套代码）

1. 车与电脑同一网段，确认 Robokit API 可达（常用 `192.168.192.5`）。
2. **停掉 Mock**，避免端口/误连。
3. 启动：

```bash
export REAL_ROBOT_ENABLED=true
export AGV_BACKEND=real
export AGV_HOST=192.168.192.5   # 改成实车 IP

ros2 run agv_control agv_control_bridge --ros-args \
  -p backend:=real -p agv_host:=$AGV_HOST
```

或 Windows 宿主机（有 ROS2 时）同样环境变量。

安全：`REAL_ROBOT_ENABLED` 默认 false；真车必须显式打开。

## 6. 旧 agv_ros2_ws 全量镜像（可选）

若需要 Gazebo Harmonic 完整栈：

```bat
cd agv_ros2_ws
docker compose -f docker\docker-compose.yml build
```

首次构建较久。V0.1 实车验证优先用上面的轻量 Humble + TCP 3055 路径。
