# AGV 当前状态审计 (Phase 0)

**日期**：2026-08-15
**审计人**：GPT-5.6
**基础版本**：V0.51.4_Sim_20260815

## 1. 旧架构问题

原架构：
```
订单 → 预设站点 → Robokit 原生导航 → 3051 路径导航 → LM1→LM2→...→LM10
```

### 为什么效果差

- **路径僵化**：每次必须按站点走，无法绕过临时障碍
- **耦合重**：Web、调度、底盘全部绑定 LM1-LM10 站点表
- **实时性差**：站点间走 Robokit 原生定位+规划，单跳 800ms~2s
- **站点表维护**：每次改地图要同步站点表
- **避障弱**：原生路径规划避障能力有限

### 哪些代码导致绑定

| 文件 | 绑定点 | 处置 |
|---|---|---|
| `agv_sim_node.py` | AgvNavigate 服务 / LM1-LM10 | KEEP + DEPRECATED |
| `agv_bridge_node.py` | goto_station / 3051 | KEEP + DEPRECATED + ENABLE_LEGACY_BRIDGE 开关 |
| `agv_adapter/base.py` | goto_station 抽象方法 | REFACTOR（加 set_velocity / get_odom） |
| `delivery_interfaces/srv/AgvNavigate.srv` | target_id 必填 | KEEP（继续用于 legacy 测试） |
| `dashboard_node.py` | /api/navigate 站点导航 | KEEP（保留兼容） |

## 2. 可保留

- `robokit_mock_server.py` — 仿真 TCP API（继续用）
- `robokit_client.py` — 真车客户端（继续用）
- `agv_adapter/{factory,mock,real,dev}.py` — Adapter 抽象（继续用）
- `arm/*` — 机械臂层（继续用）
- `camera_bridge/*` — 相机仿真（继续用）
- `delivery_web/` — Web dashboard（继续用）
- `delivery_interfaces/` — 接口定义（扩展用）

## 3. 必须重构

- `agv_bridge_node.py`：加 DEPRECATED 标记 + ENABLE_LEGACY_BRIDGE 硬开关
- `agv_sim_node.py`：加 DEPRECATED 标记
- `agv_adapter/base.py`：加 set_velocity / get_odom 接口

## 4. 必须删除

无（Leo/Zack 源码不在此包中）

## 5. 必须新增

| 包 | 用途 |
|---|---|
| `agv_control` | 底盘直控桥（cmd_vel ⇄ Backend） |
| `agv_navigation` | Nav2 bringup + Smac + Regulated Pure Pursuit |
| `agv_safety` | Safety State Machine + Collision Bridge |
| `agv_mission` | Mission Manager 状态机 |
| `agv_tf` | 静态 TF 发布器 |
| `agv_lidar` | 双激光仿真 + 融合 |
| `agv_ultrasonic` | 12 超声仿真 + 集群 |
| `agv_simulation` | 场景 YAML + 加载器 |
| `agv_vision_mock` | 离线视觉仿真 (face/qr/object/image) |
| `agv_bringup` | 统一启动入口（sim / nav / real） |

## 6. 新架构

```
订单 → Mission Manager → Nav2 NavigateToPose → /cmd_vel
   → AGV Control Bridge → Backend (mock/sim/real)
   → 底盘
```

## 7. 离线 / 仿真能力审计

| 能力 | 状态 |
|---|---|
| 编译 | ✅ 11 个新包 colcon build 通过 |
| 单元测试 | ✅ pytest 25+ 用例（无 ROS2 启动） |
| Mock Backend | ✅ 运动学积分 + Jerk 约束 + 控制权状态机 |
| Mock LiDAR | ✅ 双激光 (front + rear) + 场景注入 |
| Mock 超声 | ✅ 12 个 Range 消息 + 集群 PointCloud |
| Mock Nav2 | ✅ 完整 launch + 离线地图 yaml |
| Mock Safety | ✅ cmd_vel stale + 控制权丢失 + 急停 |
| Mock Mission | ✅ 状态机 + 合法转移表 |
| Mock Vision | ✅ face / qr / object / image 全部离线 |
| Web Dashboard | ✅ V0.51.4 保留，DELIVERY_ENV=mock |
| 真车 Backend | ❌ Phase 5 才接（REAL_ROBOT_ENABLED=true） |
| Gazebo 全量 | ❌ 需要 Docker 镜像，本仓库不包含 |