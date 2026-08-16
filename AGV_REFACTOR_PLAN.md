# AGV 重构路线图

## KEEP（保留）

- `robokit_mock_server.py`
- `robokit_client.py`
- `agv_adapter/{base,factory,mock,real,dev,models,config,laser_utils}.py`
- `arm/*`
- `camera_bridge/*`
- `delivery_web/*`
- `delivery_interfaces/*`（扩展）

## REFACTOR（重构）

- `agv_sim_node.py` — 加 DEPRECATED banner
- `agv_bridge_node.py` — 加 DEPRECATED banner + ENABLE_LEGACY_BRIDGE 开关
- `agv_adapter/base.py` — 加 set_velocity / get_odom 接口

## ADD（新增）

- `agv_control/` — 底盘直控桥
- `agv_navigation/` — Nav2 集成
- `agv_safety/` — 安全层
- `agv_mission/` — Mission Manager
- `agv_tf/` — 静态 TF
- `agv_lidar/` — 双激光融合
- `agv_ultrasonic/` — 12 超声仿真
- `agv_simulation/` — 场景 YAML
- `agv_vision_mock/` — 离线视觉仿真
- `agv_bringup/` — 统一启动

## DELETE（删除）

无（Leo/Zack 不在本包）

## DEPRECATE（标记）

- `agv_sim_node.py`
- `agv_bridge_node.py`（默认禁用）

## 时间线

| 阶段 | 内容 | 状态 |
|---|---|---|
| Phase 0 | 审计 | ✅ Done |
| Phase 1 | agv_control + MockBackend + 单测 | ✅ Done |
| Phase 2 | agv_navigation + agv_safety + Nav2 bringup | ✅ Done |
| Phase 3 | 旧 agv_bridge 标记 DEPRECATED | ✅ Done |
| Phase 4 | agv_mission 状态机 | ✅ Done |
| Phase 5 | agv_tf + agv_lidar + agv_ultrasonic 感知 | ✅ Done |
| Phase 6 | agv_simulation + agv_vision_mock 离线环境 | ✅ Done |
| Phase 7 | CI + 文档 + 部署手册 | ✅ Done |
| Phase 8 | 真车接入（REAL_ROBOT_ENABLED） | ⏳ 待真车 |

## 业务完整链路

```
用户下单
   ↓
调度系统
   ↓
AGV 收到任务
   ↓
Mission Manager: IDLE → RECEIVED
   ↓
GO_TO_PICKUP (Nav2 NavigateToPose)
   ↓
ARRIVED_PICKUP
   ↓
IDENTIFY_OBJECT (vision/object_detector)
   ↓
PICK (xArm7 motion + wrist camera)
   ↓
VERIFY_PICK (vision/qr)
   ↓
GO_TO_USER (Nav2 NavigateToPose)
   ↓
IDENTIFY_USER (vision/face)
   ↓
VERIFY_USER
   ↓
DELIVER (xArm7 motion)
   ↓
VERIFY_DELIVER
   ↓
FINISHED
```

## 离线 / 仿真原则

1. **零硬件依赖**：不连真车、真激光、真相机
2. **零外网依赖**：不下载模型、不访问云
3. **CI 可跑**：所有代码可单测
4. **Mock 默认**：AGV_BACKEND=mock
5. **真车硬开关**：REAL_ROBOT_ENABLED=true 才允许 real backend