# AGV 导航演进路线图

## 当前架构（V0.51.4_Sim_20260815 + 重构）

```
订单 → Mission → Nav2 NavigateToPose → /cmd_vel
   → Collision Bridge → Safety Monitor → Control Bridge → Backend → 底盘
```

## Stage 1 — 离线仿真（已 ✅）

| 项 | 状态 |
|---|---|
| MockBackend 运动学 | ✅ |
| 静态 TF (12 sensors + 2 arms) | ✅ |
| 双激光融合 costmap | ✅ |
| 12 超声 → PointCloud | ✅ |
| Smac Hybrid Planner | ✅ |
| Regulated Pure Pursuit | ✅ |
| Velocity Smoother | ✅ |
| Collision Monitor | ✅ |
| Mission Manager 状态机 | ✅ |
| Safety 3 层防护 | ✅ |
| 单测 46 个用例 | ✅ |

## Stage 2 — Mock + Nav2 集成（✅ V0.51.4 Sim 可直接跑）

```bash
bash scripts/start_nav_sim_stack.sh
ros2 action send_goal /navigate_to_pose nav2_msgs/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}}}}"
```

预期：
- Global Planner 在地图上规划路径
- Local Controller 跟随
- /cmd_vel_safe 限速到 0.4 m/s
- /odom 实时更新
- /scan_front / /scan_rear 持续输出

## Stage 3 — 完整业务流（Mock Vision + Mission + Arm Mock）

```bash
ros2 launch agv_vision_mock agv_mock_face.launch.py
ros2 launch agv_vision_mock agv_mock_qr.launch.py
ros2 launch agv_vision_mock agv_mock_object.launch.py
ros2 launch agv_vision_mock agv_mock_image.launch.py
ros2 launch agv_mission agv_mission_manager.launch.py
ros2 action send_goal /mission/start agv_mission/StartMission \
  "{pickup_xy: [1.5, 0.0], user_xy: [-1.5, 0.0]}"
```

预期：
- IDLE → RECEIVED → GO_TO_PICKUP → ARRIVED_PICKUP
- → IDENTIFY_OBJECT (vision) → PICK (arm mock)
- → VERIFY_PICK (qr mock) → GO_TO_USER
- → IDENTIFY_USER (face mock) → DELIVER (arm mock)
- → VERIFY_DELIVER → FINISHED

## Stage 4 — Gazebo 全量 3D 仿真（⏳ 需 Docker）

依赖：
- Docker image `delivery-ros2:humble-gazebo-v02`（向团队索取）
- delivery_gazebo 包（URDF + world）
- gazebo_ros 启动

不在本仓库范围。Stage 4 是 production-like 的最后验证。

## Stage 5 — 真车接入（⏳ 待 Phase 5）

依赖：
- `读写寄存器[4x].docx` 寄存器地址
- `只读寄存器[3x].docx` 寄存器地址
- 真车 IP（目前文档里写的是 192.168.192.5）
- pymodbus 安装（`pip install pymodbus==3.6.9`）

第一步：补全 `agv_control/backend/real.py` 的 `connect/set_velocity/get_odom` 实现。
第二步：`REAL_ROBOT_ENABLED=true AGV_BACKEND=real bash scripts/start_real_stack.sh`

## Stage 6 — 全场景验证（⏳ 需 Gazebo + 真车）

| 场景 | Stage 2 | Stage 3 | Stage 4 | Stage 5 |
|---|---|---|---|---|
| empty_room | ✅ | ✅ | ✅ | ✅ |
| obstacle_corridor | ✅ | ✅ | ✅ | ✅ |
| dynamic_obstacle | ✅ | ✅ | ✅ | ✅ |
| narrow_passage | ✅ | ✅ | ✅ | ✅ |
| emergency_stop | ✅ | ✅ | ✅ | ✅ |

## Stage 7 — Jetson / Leo / Zack 接入（⏳ 用户的业务决策）

按用户要求，本项目**不**处理 Jetson / Leo / Zack 视觉栈源码。
后续如有需要，新开分支单独处理。

## 不做的事

- ❌ 不重写 Robokit TCP 协议（沿用 V0.51.4_Sim 的客户端/服务端）
- ❌ 不引入新的运动学模型（差速底盘够用）
- ❌ 不引入 SLAM Toolbox（用静态地图 + amcl 即可）
- ❌ 不引入 ros2_control 仿真（MockBackend 已经是完整替代）
- ❌ 不重写 Web Dashboard（V0.51.4 已验证）

## 总结

**新架构 = 旧 Robokit 客户端（保留） + 新增 cmd_vel 直控桥 + Nav2 + Safety**

不动原有投资，叠加新能力。