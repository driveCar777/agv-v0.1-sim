# AGV Agent Changelog

本文件记录所有由 agent 完成的重构、修改、新增。

---

## 2026-08-16 — ROS2 Humble Docker + 实车同款 TCP 控制桥

### 发现

- 本机已有 `agv_ros2_ws/docker`（Humble + Nav2 + Gazebo），此前**未构建镜像**
- Docker Desktop 已拉起；V0.1 新增轻量 Humble 镜像（国内 apt 镜像）

### 实车可测控制链路

- `TcpApiClient` / `RealBackend`：`/cmd_vel` → **3055/3056** + **4005 lock**（Mock 与真车同一协议）
- `AGV_BACKEND=tcp` 连 Mock；`real` + `REAL_ROBOT_ENABLED=true` 连真车
- 已验证：对着本机 Mock `connect/lock/3055` → odom 反馈正常

### 地图 / Docker

- `scripts/export_smap_to_navmap.py` → `maps/office_nav/{map.yaml,map.pgm,pois.yaml}`
- `docker/` + `scripts/start_ros2_docker.bat`
- 文档：`docs/ROS2_DOCKER_REAL_ROBOT.md`

---

### 选型

- **引擎**：真实 `.smap` 占用栅格世界 + 双雷达 ray-cast + A* + **Three.js** 主视口  
- 原因：贴合 Web 主界面、Windows 可离线开发、控制仍走实车 Robokit API；比 Gazebo/Isaac 更贴当前交付路径

### 新增 / 改造

- `agv_bridge/smap_loader.py` — 加载 `agv_downloaded/maps/*.smap` + 室外公开园区
- `sim_world.py` — 真地图碰撞/规划/稠密点云
- `sim_api_ext.py` — 两段式导航：`plan → pending_confirm → confirm`
- `www/sim_main.html` + `sim3d.js` — 第三人称点云主视口、AMB-150 车模、引导带、滚轮镜头
- 组件：地图缩略图、导航（搜点/规划）、车上「开始」确认弹窗
- 室内规划线 **黄** / 室外 **蓝**

### 启动

```bat
scripts\start_web_sim.bat
```

---

## 2026-08-16 — API 仿真 Web：双雷达 + 设点导航 + 文言提示

### 新增

- `agv_bridge/sim_world.py` — 障碍物地图、双对角雷达 ray-cast、文言编年
- `agv_bridge/sim_api_ext.py` — Mock 增强：3055/3056、物理积分、设点 A* 导航
- `scripts/run_web_sim.py` / `start_web_sim.bat` — **无需 ROS2** 的 Web 仿真栈

### 行为

- 仿真车也走实车同款 API（3055 平动 / 1009 双雷达 / 3051 坐标导航）
- Web 单击地图设点 → 规划绕障 → 到达
- 页面中上方文言体提示条：说明当前动作与原因（见障缓行 / 规划 / 到达等）
- 感知仅双对角雷达（无视觉）

### 启动

```bat
scripts\start_web_sim.bat
```

浏览器：http://127.0.0.1:19999/

---

## 2026-08-16 — V0.1仿真版整理 + 修复 + 全量离线可跑

### 整理

- 从 `AGV_Web_V0.51.4_Sim_20260815` 复制并固化为独立目录 **`V0.1仿真版`**
- `VERSION` → `0.1.0-sim`

### 关键修复

- 全部新包 `setup.py`：移除错误的绝对路径 `package_dir`
- 全部 `scripts/*.sh`：`ROOT` 从错误的 `../..` 改为包根 `..`
- `agv_sim_bringup` / `agv_nav_bringup`：修正 TF / Include 路径；接上激光、超声、Safety
- 新增 `agv_full_sim_bringup.launch.py` + `start_full_sim_stack.sh`
- `agv_tf/static_tf.py`：RPY → 四元数（原实现错误地把 yaw 写进 quat.z）
- Nav2：`use_sim_time=False`（Mock 无 `/clock`）；地图 `maps/empty_room.{pgm,yaml}`
- `agv_real_bringup`：强制检查 `REAL_ROBOT_ENABLED` + `AGV_BACKEND=real`
- 旧桥 `agv_sim_node` / `agv_bridge_node`：DEPRECATED banner + legacy 硬开关
- Mission Manager：补齐离线业务推进 + `auto_start`
- CI：补全 `.github/workflows/ci.yml`

### 默认

- `AGV_BACKEND=mock`
- `REAL_ROBOT_ENABLED=false`
- Web：保留 delivery_web V0.51.4
- Leo / Zack：不碰

---

## 2026-08-15 — V0.51.4_Sim 重构 + 全量新包

### 新增包（11 个）

| 包 | 描述 | 状态 |
|---|---|---|
| `agv_control` | 底盘直控桥 — cmd_vel ⇄ mock/sim/real backend | ✅ |
| `agv_navigation` | Nav2 bringup + 双激光 costmap 配置 | ✅ |
| `agv_safety` | Safety monitor + Collision bridge + Watchdog | ✅ |
| `agv_mission` | Mission Manager 状态机（IDLE → FINISHED） | ✅ |
| `agv_tf` | 静态 TF — base_link → 全部传感器 / 机械臂 | ✅ |
| `agv_lidar` | 双激光仿真 + merger | ✅ |
| `agv_ultrasonic` | 12 超声仿真 + PointCloud 集群 | ✅ |
| `agv_simulation` | 场景 YAML + 加载器 + 5 个预置场景 | ✅ |
| `agv_vision_mock` | face / qr / object / image 离线视觉 | ✅ |
| `agv_bringup` | 统一启动入口（sim / nav / full / real） | ✅ |
| `delivery_interfaces` | 扩展 SafetyState.msg | ✅ |
