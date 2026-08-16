# AGV API 映射表

## ROS2 → Backend → AGV API

| ROS2 接口 | Backend 接口 | AGV API | 单位 | 频率 |
|---|---|---|---|---|
| `/cmd_vel` (Twist) | `set_velocity(vx,vy,w)` | Modbus 4x 开环速度寄存器（待核对） | m/s, rad/s | 50Hz |
| `/odom` (Odometry) | `get_odom()` | 1004 + 1005 位置/速度 | m, rad, m/s | 50Hz |
| `/tf` (odom→base_link) | `get_odom()` | 1004 | m, rad | 50Hz |
| `/navigate_to_pose` Action | Nav2 内部 | 3051 路径导航（legacy，已弃用） | — | — |
| `/agv/safety_state` | 内部 | 1006 阻挡状态 + 自定义判定 | — | 20Hz |
| `/agv/battery` | `get_battery()` | 1007 | % | 1Hz |
| `/scan_front` (LaserScan) | 1009 / sim | 1009 激光 | m | 10Hz |
| `/scan_rear` (LaserScan) | 1009 / sim | 1009 激光 | m | 10Hz |
| `/scan` (合并后) | dual_lidar_merger | — | m | 10Hz |
| `/ultrasonic/01..12` (Range) | mock 仿真 | 12 超声 | m | 20Hz |
| `/ultrasonic_cloud` (PointCloud) | cluster 融合 | — | — | 20Hz |
| `/camera/front/image_raw` | mock_image | 相机图像 | rgb8 | 10Hz |
| `/face/detections` | mock_face | 人脸识别 | — | 1Hz |
| `/qr/detections` | mock_qr | 二维码识别 | — | 1Hz |
| `/vision/objects` | mock_object | 物体检测 | — | 2Hz |

## 控制权状态机

```
DISCONNECTED → CONNECTED → REQUEST_CONTROL → CONTROL_GRANTED
              → READY → MOVING → STOPPING → RELEASE
              ↘ CONTROL_LOST → SAFE_STOP
```

## Backend 选择

| AGV_BACKEND | 默认 | 用途 | 触发条件 |
|---|---|---|---|
| mock | ✅ | 单元测试、CI、开发 | 默认 |
| sim |  | 与 Nav2 仿真协同 | AGV_BACKEND=sim |
| real |  | 真车 | REAL_ROBOT_ENABLED=true |

## 寄存器（Modbus 4x，对照原始文档）

| 寄存器 | 含义 | 单位 | 备注 |
|---|---|---|---|
| 4x_开环_vx | 机器人坐标系 vx | m/s | 写 0 = 停 |
| 4x_开环_vy | 机器人坐标系 vy | m/s | 差速车一般 0 |
| 4x_开环_w | 机器人坐标系 w | rad/s | 正=逆时针 |
| 0x_控制权请求 | 抢控制权 | bit | 0/1 |

> ⚠️ **寄存器具体地址必须从项目原始 `读写寄存器[4x].docx` 核对**，本表只占位。

## 状态查询（3x 只读）

| 寄存器 | 含义 | 单位 |
|---|---|---|
| 3x00001 | 机器人 X 坐标 | m |
| 3x00003 | 机器人 Y 坐标 | m |
| 3x00005 | 机器人角度 | rad |
| 3x00007 | 当前导航站点 id | — |
| 3x00013 | 电池电量 | % |
| 3x00015 | 电池温度 | ℃ |
| 3x00017 | 电池电压 | V |
| 3x00019 | 电池电流 | A |
| 3x00043 | 控制权被外部抢占 | 0/1 |