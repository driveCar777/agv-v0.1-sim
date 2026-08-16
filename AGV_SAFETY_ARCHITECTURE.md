# AGV Safety 层架构

## 三层防护

```
Mission Manager
   ↓ Action
Nav2 (bt_navigator)
   ↓ /cmd_vel_raw (raw from planner)
agv_collision_bridge         ← Layer 1: 按 SafetyLevel 缩放 cmd_vel
   ↓ /cmd_vel_safe
agv_safety_monitor           ← Layer 2: 实时判定 SafetyLevel
   ↑ /scan_front, /scan_rear, /ultrasonic/*
   ↓ /agv/safety_state
agv_control_bridge           ← Layer 3: backend watchdog（cmd_stale / 控制权丢失）
   ↓ set_velocity (modbus/tcp)
底盘
```

## SafetyLevel 状态机

```
OK ────────────► WARN ────────────► SLOW ────────────► STOP ────────────► EMERGENCY
 │                  │                │                  │                  │
 ▼                  ▼                ▼                  ▼                  ▼
scan_timeout    slow_distance    stop_distance      emergency_stop    manual_reset
cmd_stale       approaching      too_close          estop_pressed    nav_cancel
control_lost                                          safety_stop      timeout
```

## Layer 1 — agv_collision_bridge

**职责**：订阅 SafetyLevel，缩放 /cmd_vel_raw → /cmd_vel_safe。

| Level | 处理 |
|---|---|
| OK (0) | 透传 |
| WARN (1) | 透传 |
| SLOW (2) | vx *= 0.3, w *= 0.3 |
| STOP (3) | vx = 0, w = 0 |
| EMERGENCY (4) | vx = 0, w = 0 + 标记 |

## Layer 2 — agv_safety_monitor

**职责**：订阅 lidar / ultrasonic / cmd，发布 SafetyLevel。

| 输入 | 用途 |
|---|---|
| /scan_front | 计算前方最近障碍距离 |
| /scan_rear | 计算后方最近障碍距离 |
| /ultrasonic/01..12 | 短距离精测 (≤2m) |
| /cmd_vel_raw | cmd_stale 判定 |

**判定逻辑**：
```
if scan_timeout (>1s):
    level = STOP, reason = scan_timeout
elif cmd_stale (>500ms):
    level = SLOW, reason = cmd_stale
elif min(front, rear) < stop_distance (0.6m):
    level = STOP, reason = too_close
elif min(front, rear) < slow_distance (1.5m):
    level = SLOW, reason = approaching
else:
    level = OK, reason = clear
```

## Layer 3 — agv_control Watchdog

**职责**：在底盘桥内部，cmd_vel 失联或控制权丢失时安全停。

```
if has_control == False:
    level = CONTROL_LOST → on_safe_stop
elif (now - last_cmd_stamp) > 500ms:
    level = CMD_STALE → on_safe_stop
```

## 急停接口

### 物理急停
- 由 `agv_control_bridge` 内部订阅 `/agv/emergency_stop` (Bool)
- 一旦 True → backend.emergency_stop() → 立即清零 vx/vy/w
- 需要 backend.resume_from_emergency() 才能恢复

### 软件急停（Nav2）
- Nav2 BT 内部 `CancelControl` action → bt_navigator 发 vx=0
- 经过 collision_bridge 后 vx 仍为 0 → safe_stop 生效

### 系统急停（业务层）
- MissionManager 检测到 SAFETY_STOP 错误 → 触发 Recovery

## 故障恢复

| 故障 | 自动恢复 | 手动恢复 |
|---|---|---|
| cmd_stale | ✅ cmd 重新发就自动恢复 | — |
| 障碍移除 | ✅ 距离增大后自动从 STOP → SLOW → OK | — |
| 控制权丢失 | ❌ | 重新调用 backend.request_control() |
| EMERGENCY | ❌ | 调用 backend.resume_from_emergency() |
| Nav2 失败 | ✅ Nav2 BT 内置 recovery | — |

## 离线 / 仿真

Safety 层完全离线，依赖 mock lidar / mock ultrasonic 即可工作。
可在 CI 中注入：
- scan_timeout 模拟
- control_lost 模拟
- emergency 注入

## 单元测试

`agv_control/tests/test_watchdog.py` 5 个用例
`agv_safety/tests/test_safety_logic.py` 2 个用例

全 CI 可跑。