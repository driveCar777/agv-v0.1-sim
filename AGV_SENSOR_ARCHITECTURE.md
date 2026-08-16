# AGV_SENSOR_ARCHITECTURE.md

## V0.1 仿真版传感器

| 传感器 | frame | topic | 来源 | 频率 |
|--------|-------|-------|------|------|
| 前激光 | lidar_front | /scan_front | agv_lidar mock | 10Hz |
| 后激光 | lidar_rear | /scan_rear | agv_lidar mock | 10Hz |
| 融合激光 | base_link | /scan_merged | dual_lidar_merger | 10Hz |
| 超声×12 | ultrasonic_01..12 | /ultrasonic/... | agv_ultrasonic | 20Hz |
| 前/后/腕相机 | camera_* | /camera/*/image_raw | vision_mock / camera_bridge | 5~10Hz |
| odom | odom→base_link | /odom + TF | agv_control_bridge | 50Hz |

## 能力边界（如实）

- 激光：2D 几何 ray-cast，无强度、无严格时间同步
- 相机：合成帧 / Mock 检测结果，**不接 Leo/Zack/Jetson**
- 超声：Range 消息，可进 Collision Monitor（后续）
- 真车传感器：Phase 真车接入后再接 API

## 离线保证

默认 `AGV_BACKEND=mock`，零硬件、零外网。
