# AGV Web V0.51.4 — 本地仿真部署手册

**适用对象**：在新电脑上跑 Web 仿真（Mock / Dev Sim），不需要 NUC、不需要真车。  
**系统要求**：Ubuntu 22.04 + ROS2 Humble（推荐裸机或 VM，内存 ≥ 8GB）

---

## 1. 这个包能做什么 / 不能做什么

| 能力 | Mock 模式 | Dev Sim 模式 | 需要团队 Docker 镜像 |
|------|-----------|--------------|---------------------|
| Web 页面打开 | ✅ | ✅ | Gazebo 3D 可视化 |
| 站点导航 LM1→LM6 | ✅（Mock TCP） | ✅（agv_sim_node） | delivery_gazebo |
| 地图/激光显示 | ✅ 模拟数据 | ✅ 模拟数据 | 真 Gazebo 物理 |
| 机械臂卡片 | ✅ 仿真态 | ✅ 仿真态 | 真 xArm |
| Leo 人脸 | ✅ 离线 Mock | ✅ 离线 Mock | 真 Leo Jetson |
| 腕部 Pylon 相机 | ❌ 无硬件 | ❌ 无硬件 | 需相机 |

**结论**：本包可在新电脑**完整编译并跑通 Web + 导航仿真**。完整 Gazebo 3D 需另向团队索取 Docker 镜像 `delivery-ros2:humble-gazebo-v02`。

---

## 2. 包内结构

```
AGV_Web_V0.51.4_Sim_20260815/
├── MANUAL_SIM.md              ← 本手册
├── README.md
├── QUICKSTART.txt
├── VERSION                    ← 0.51.4
├── config/
│   └── devices.sim.yaml       ← 仿真设备配置
├── maps/
│   └── stations_from_smap.json← LM1-LM10 站点坐标
├── scripts/
│   ├── install_deps_ubuntu22.sh
│   ├── build.sh
│   ├── start_mock_stack.sh    ← 推荐：Mock Robokit
│   └── start_dev_sim_stack.sh ← 备选：ROS 仿真节点
└── ros2_ws/src/
    ├── delivery_interfaces/   ← ROS 消息/服务定义（必须）
    ├── delivery_web/          ← Web Dashboard
    ├── agv_bridge/            ← Robokit 客户端 + Mock + agv_sim_node
    ├── camera_bridge/         ← 相机仿真节点
    └── delivery_bringup/      ← Launch 参考
```

**未打包的（故意排除）**：
- NUC 部署脚本、真车 release 配置
- 测试补丁脚本（apply_phase3_patch.py 等）
- freeze 备份、开发调试垃圾文件
- Gazebo 完整栈（delivery_gazebo 包，体积大，在 Docker 镜像里）

---

## 3. 快速开始（5 步）

```bash
# 0. 解压
tar xzf AGV_Web_V0.51.4_Sim_20260815.zip
cd AGV_Web_V0.51.4_Sim_20260815

# 1. 安装依赖（仅首次，需 sudo）
bash scripts/install_deps_ubuntu22.sh

# 2. 编译
source /opt/ros/humble/setup.bash
bash scripts/build.sh

# 3. 启动 Mock 仿真（推荐）
bash scripts/start_mock_stack.sh

# 4. 浏览器
# http://127.0.0.1:19999/  （Ctrl+F5）
```

验证：
```bash
curl http://127.0.0.1:19999/api/version
curl http://127.0.0.1:19999/api/state
```

---

## 4. 两种仿真模式详解

### 4.1 Mock 模式（推荐）

- 启动 `robokit_mock_server`（模拟仙工 TCP 19204–19207）
- Dashboard 设 `DELIVERY_ENV=mock`，连 `127.0.0.1`
- 可测试：抢锁 4005、导航 3051、Web 全部卡片、诊断 API
- **不需要** ROS 导航节点，最轻量

```bash
bash scripts/start_mock_stack.sh
```

Web 里切到 **mock** 模式，或 API：
```bash
curl -X POST http://127.0.0.1:19999/api/env \
  -H 'Content-Type: application/json' \
  -d '{"mode":"mock","agv_host":"127.0.0.1"}'
```

导航测试：
```bash
curl -X POST http://127.0.0.1:19999/api/navigate \
  -H 'Content-Type: application/json' \
  -d '{"target_id":"LM6"}'
```

### 4.2 Dev Sim 模式

- 启动 `agv_sim_node`（纯 Python 离线导航，内置 LM1–LM10）
- 启动 `camera_sim_node`（模拟前向相机帧）
- Dashboard 设 `DELIVERY_ENV=dev`
- 适合测试 ROS topic 订阅、地图上的 AGV 位置动画

```bash
bash scripts/start_dev_sim_stack.sh
```

---

## 5. 编译说明

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select delivery_interfaces delivery_web agv_bridge camera_bridge --symlink-install
source install/setup.bash
```

常见错误：

| 错误 | 解决 |
|------|------|
| `delivery_interfaces not found` | 先编 delivery_interfaces |
| `No module named agv_bridge` | source install/setup.bash |
| `ros2: command not found` | 先 source /opt/ros/humble/setup.bash |
| 端口 19999 占用 | `pkill -f dashboard_node` |

---

## 6. Web 功能清单（仿真可用）

| 功能 | Mock | Dev Sim |
|------|------|---------|
| 地图 + 站点 | ✅ | ✅ |
| AGV 位置刷新 | ✅ | ✅ |
| 导航到站点 | ✅ | ✅ |
| 取消导航 | ✅ | ✅ |
| 抢/释放控制权 | ✅ | ✅ |
| 诊断快照 `/api/diag/snapshot` | ✅ Mock 数据 | 部分 |
| 机械臂状态卡片 | ✅ 仿真 | ✅ 仿真 |
| Leo 人脸（连续识别开/关） | ✅ Mock | ✅ Mock |
| 腕部相机 | ❌ | ❌ |
| Jason 户外相机 | ❌ | ❌ |

---

## 7. 环境变量

| 变量 | Mock 默认 | 说明 |
|------|-----------|------|
| `ROS_DOMAIN_ID` | 30 | 必须 |
| `DELIVERY_ENV` | mock / dev | 模式 |
| `ARM_MODE` | simulation | 机械臂仿真 |
| `ARM_REAL_MOTION` | 0 | 禁止真机运动 |
| `USE_SIM_CAMERAS` | 1 | 相机走 Mock |
| `DELIVERY_DEVICES_YAML` | config/devices.sim.yaml | 设备配置 |

---

## 8. 日志

```bash
# Mock 模式日志
tail -f logs/robokit_mock.log

# Dashboard 输出在启动终端
```

---

## 9. 与真车/NUC 的区别

| 项目 | 本仿真包 | NUC 真车环境 |
|------|----------|-------------|
| AGV IP | 127.0.0.1 (Mock) | 192.168.18.198 |
| 网段 | 本机 | NUC 双网卡 172+192 |
| Docker | 不需要 | delivery_gazebo_soft |
| 配置文件 | devices.sim.yaml | devices.nuc.release.yaml |
| Leo 人脸 | Mock 数据 | 真 ROS service |

---

## 10. 获取完整 Gazebo 仿真（可选）

如需 3D Gazebo 可视化，向团队索取：

1. Docker 镜像：`delivery-ros2:humble-gazebo-v02`（约数 GB）
2. 完整 workspace（含 `delivery_gazebo` 包）
3. `agv_downloaded/` 地图资源

参考团队 VM：`172.31.0.111` 或 `192.168.18.240` 上的 `.run/workspace_v02`。

---

## 11. 版本信息

- **版本**：0.51.4
- **日期**：2026-08-15
- **变更**：Leo SetBool 修复、P0 诊断、腕部相机、机械臂死锁修复

---

## 12. 联系

- Web / AGV 桥接：Pengfei.Hao
- 接口定义问题：查 `delivery_interfaces/` 包内 .msg/.srv 文件
