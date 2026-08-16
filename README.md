# AGV V0.1 仿真版

**版本**：0.1.0-sim  
**基线**：AGV_Web_V0.51.4_Sim_20260815 重构整理版  
**原则**：完全离线 · 默认 Mock · 不连真车 / Leo / Zack / Jetson

## 架构

```text
Nav2 / Web
  → /cmd_vel
  → agv_control_bridge (backend=tcp|real)
  → Robokit TCP 3055/3056   ← 仿真 Mock 与真车同一协议
  → Mock(127.0.0.1) 或 真车(AGV_HOST)
```

旧「LM1→LM10 站点导航」已标记 DEPRECATED，默认不启用。

## ROS2 Humble（本机 Docker，可上实车）

本机已有 `agv_ros2_ws/docker`（Humble 全量+Gazebo），V0.1 另提供轻量镜像专注控制/规划：

```bat
scripts\start_web_sim.bat          # 宿主机 Mock + Web
scripts\start_ros2_docker.bat      # 构建并进入 Humble 容器
```

详见 [`docs/ROS2_DOCKER_REAL_ROBOT.md`](docs/ROS2_DOCKER_REAL_ROBOT.md)。

真车：

```bash
REAL_ROBOT_ENABLED=true AGV_BACKEND=real AGV_HOST=192.168.192.5 \
  ros2 run agv_control agv_control_bridge --ros-args -p backend:=real -p agv_host:=$AGV_HOST
```

## Windows / 无 ROS2：真场景仿真 Web（推荐）

选型：**真实办公室 `.smap` 占用栅格 + 双雷达 ray-cast + Three.js 第三人称**  
（非 Gazebo；协议仍走 Robokit 实车 API，Windows 可离线开发）

```bat
scripts\start_web_sim.bat
```

浏览器：**http://127.0.0.1:19999/**

| 操作 | 说明 |
|------|------|
| 主视口 | 第三人称；底层为车周实时/全景点云；AMB-150 车模；规划引导带 |
| 滚轮 | 贴近（有最近距）/ 拉远至约 100m 俯视 |
| 地图组件 | 路段缩略图；未导航=全景点云；室内黄线 / 室外蓝线 |
| 导航组件 | 搜点、设点 → 规划 → **车上弹窗点「开始」** 才行动 |
| 场景切换 | 室内办公室（`agv_downloaded` smap）/ 室外公开园区 |

控制链路：`规划 → 确认 → API 3055 → Mock 底盘 → 1009 雷达反馈`

| 脚本 | 内容 |
|------|------|
| `start_mock_stack.sh` | 底盘 + Web |
| `start_nav_sim_stack.sh` | + 激光/超声 + Nav2 + Safety |
| `start_full_sim_stack.sh` | + 视觉 Mock + Mission |
| `start_real_stack.sh` | 真车（需 `REAL_ROBOT_ENABLED=true`） |

## 离线验证

```bash
# 单测（无需 ROS2 节点）
cd ros2_ws
export PYTHONPATH=src/agv_control:src/agv_mission:src/agv_lidar:src/agv_ultrasonic:src/agv_simulation:src/agv_vision_mock:src/agv_tf:$PYTHONPATH
python3 -m pytest src/agv_*/agv_*/tests -v

# cmd_vel → odom
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' -r 10
ros2 topic echo /odom
```

## 环境变量（默认）

```bash
AGV_BACKEND=mock
REAL_ROBOT_ENABLED=false
DELIVERY_ENV=mock
```

## 包清单

| 包 | 作用 |
|----|------|
| `agv_control` | `/cmd_vel` 直控桥 + MockBackend |
| `agv_navigation` | Nav2 bringup + 离线地图 |
| `agv_safety` | 安全监控 + 碰撞桥 |
| `agv_mission` | 任务状态机 |
| `agv_tf` | 静态 TF |
| `agv_lidar` | 双激光仿真 + 融合 |
| `agv_ultrasonic` | 12 超声仿真 |
| `agv_vision_mock` | face/qr/object/image Mock |
| `agv_simulation` | 场景 YAML |
| `agv_bringup` | 统一 launch |
| `delivery_web` | Web Dashboard V0.51.4 |
| `agv_bridge` | 旧桥（DEPRECATED） |

## 文档

- `AGV_CURRENT_STATE_AUDIT.md`
- `AGV_API_MAPPING.md`
- `AGV_REFACTOR_PLAN.md`
- `AGV_OFFLINE_DEPLOYMENT.md`
- `AGV_TF_ARCHITECTURE.md`
- `AGV_SAFETY_ARCHITECTURE.md`
- `AGV_NAVIGATION_ROADMAP.md`
- `AGV_AGENT_CHANGELOG.md`
