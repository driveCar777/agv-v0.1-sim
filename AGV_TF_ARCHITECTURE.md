# AGV TF 树架构

```
world
└── odom          (agv_control_bridge)
    └── base_link (agv_control_bridge / odom_node)
        ├── lidar_front          (agv_tf/static_tf)
        ├── lidar_rear           (agv_tf/static_tf)
        ├── camera_front         (agv_tf/static_tf)
        ├── camera_rear          (agv_tf/static_tf)
        ├── camera_down          (agv_tf/static_tf)
        ├── left_arm_base        (agv_tf/static_tf)
        │   └── left_arm_link1   (xarm_driver / joint_states)
        │       └── ... → left_end_effector
        ├── right_arm_base       (agv_tf/static_tf)
        │   └── right_arm_link1
        │       └── ... → right_end_effector
        ├── wrist_camera         (xarm_driver)
        ├── ultrasonic_01..12    (agv_tf/static_tf)
        └── base_footprint       (Nav2 / AMCL)
```

## 静态 TF 关系（相对 base_link）

| 子 frame | x | y | z | yaw |
|---|---|---|---|---|
| lidar_front | 0.30 | 0.00 | 0.20 | 0.0 |
| lidar_rear | -0.30 | 0.00 | 0.20 | π |
| camera_front | 0.35 | 0.00 | 0.30 | 0.0 |
| camera_rear | -0.35 | 0.00 | 0.30 | π |
| camera_down | 0.00 | 0.00 | 0.05 | π/2 |
| left_arm_base | 0.10 | 0.20 | 0.40 | 0.0 |
| right_arm_base | 0.10 | -0.20 | 0.40 | 0.0 |
| ultrasonic_01..06 | ±0.35 | ±0.10..0.30 | 0.10 | 0 / π |

## 动态 TF

| 来源 | 发布 |
|---|---|
| `agv_control_bridge` | odom → base_link (50Hz) |
| `xArm driver` | arm_base → arm_link1..N (50Hz) |
| `wrist_camera` | arm_end → wrist_camera (TF) |
| `agv_static_tf` | base_link → 全部传感器 (latched) |

## 启动顺序

```bash
ros2 launch agv_tf static_tf.launch.py           # 全部静态 TF
ros2 launch agv_control agv_control_bridge.py    # odom → base_link
# 然后再启动其他依赖 TF 的节点（Nav2 / lidar / arm）
```

## 验证

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link lidar_front
ros2 run tf2_ros tf2_echo odom base_link
```

## 离线模拟

完全静态 TF 节点无需任何硬件，启动后立刻可用。