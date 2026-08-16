# AGV 离线部署手册

**适用**：Ubuntu 22.04 + ROS2 Humble（裸机 / VM / WSL2）

## 0. 系统要求

| 项目 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 LTS |
| ROS | Humble Desktop |
| Python | 3.10+ |
| 内存 | ≥ 8 GB |
| 磁盘 | ≥ 5 GB |

## 1. 一键启动（完整仿真 + Nav2 + Safety）

```bash
cd AGV_Web_V0.51.4_Sim_20260815

# 安装依赖（仅首次）
bash scripts/install_deps_ubuntu22.sh

# 编译
source /opt/ros/humble/setup.bash
bash scripts/build.sh

# 启动 P1（仅底盘 + Web）
bash scripts/start_mock_stack.sh
# 浏览器：http://127.0.0.1:19999/

# 启动 P2（+ Nav2 + Safety）
bash scripts/start_nav_sim_stack.sh
```

## 2. 验证

```bash
# 命令行验证 cmd_vel → odom
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}' -r 10
ros2 topic echo /odom --once
# 预期：x 单调递增

# 查看 TF 树
ros2 run tf2_tools view_frames

# 看 Nav2
ros2 topic list | grep -E '(nav|cmd_vel|odom)'
ros2 action send_goal /navigate_to_pose nav2_msgs/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}}}}"

# 看 sensor
ros2 topic echo /scan_front --once
ros2 topic echo /scan_rear --once
ros2 topic echo /ultrasonic/01 --once
ros2 topic echo /face/detections --once
```

## 3. 单测

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/agv_control/tests/ \
  src/agv_ultrasonic/tests/ \
  src/agv_lidar/tests/ \
  src/agv_mission/tests/ \
  src/agv_simulation/tests/ \
  src/agv_safety/tests/ \
  src/agv_vision_mock/tests/ \
  -v
# 预期：46 passed
```

## 4. 切换真车（REAL_ROBOT_ENABLED）

⚠️ **默认禁止**。必须显式开启，且必须先把 `读写寄存器[4x].docx` 里的寄存器地址填到 `agv_control/backend/real.py` 中。

```bash
export REAL_ROBOT_ENABLED=true
export AGV_BACKEND=real
bash scripts/start_real_stack.sh
```

## 5. 故障排查

| 现象 | 排查 |
|---|---|
| `agv_bridge_node` 启动失败 | 正常，已 DEPRECATED，必须 `ENABLE_LEGACY_BRIDGE=true` 才允许 |
| `robokit_mock_server` 端口占用 | `pkill -f robokit_mock_server` |
| Nav2 启动失败 | 确认已 `source install/setup.bash` |
| `/odom` 无输出 | 确认 `agv_control_bridge` 在运行 |
| `/scan_front` 无输出 | 确认 `agv_mock_lidar` 节点在运行 |

## 6. 卸载

```bash
cd AGV_Web_V0.51.4_Sim_20260815/ros2_ws
rm -rf build install log
```

## 7. 完全离线的保证

✅ 零外网依赖：build / launch / test 全部本地完成
✅ 零硬件依赖：不连 AGV、不连相机、不连 Jetson
✅ 零云端依赖：不调用任何 cloud API
✅ 零地图下载：使用本地 smap / yaml
✅ 零模型下载：vision 全部 mock

唯一的"在线"是首次 `apt install ros-humble-*` —— 装完之后再无外网需求。