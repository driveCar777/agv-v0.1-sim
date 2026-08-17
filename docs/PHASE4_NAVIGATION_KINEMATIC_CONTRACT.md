# PHASE 4 — Navigation Kinematic Contracts（P0-0 Freeze）

日期：2026-08-17  
基线：`71c466b` / V0.11  
状态：**FROZEN for P0-A**  
约束：本文件只定义语义；**P0-A 不实现 Global Preview / Kinematic Validator / Approach / Rotation Guard。**

---

## 0. Body-frame convention（统一）

与现有差速积分一致：

```text
Body frame (vehicle-centric):
  +x = FRONT   (vehicle forward)
  -x = REAR
  +y = LEFT
  -y = RIGHT
  yaw = 0 along +world-x when vehicle faces +X
```

World pose `(x, y, yaw)`：车体原点在几何中心（`center_offset_x_m = 0`，除非后续标定）。

UI Local Candidate：**上 = FRONT (+body x)**，**下 = REAR**（已在 3F-CORRECTIVE 约定）。

---

## 1. Global Path

| | |
|--|--|
| **定义** | 从起点到目标的拓扑/栅格参考折线 |
| **当前** | `(x_i, y_i)` polyline reference（A* + path_quality） |
| **未来** | densify → `yaw_i`, `κ_i` → **kinematic validated trajectory** |
| **不是** | cmd_vel；不是 Local Physical Trajectory；不是 Safety Envelope |
| **P0-A** | **不改** Global Planner 行为 |

---

## 2. Global Reference Corridor

| | |
|--|--|
| **定义** | 长距离、供 Policy / Local / 地图参考的 **Global Reference Preview** |
| **目标 preview** | adaptive **2–8 m**，normal ≈ **5 m** |
| **内容** | densified centerline + yaw + first-turn metadata + lightweight display swept（REFERENCE_ONLY） |
| **P0-A** | **仅 contract** |
| **P0-B** | **已实现 REFERENCE ONLY**：`nav_global_preview` / `nav.global_reference`；**不**产 cmd_vel |
| **P0-C** | **已实现 telemetry-only validator**：`nav_kinematic` / `nav.kinematic_validation`；`kinematic_valid` = true/false/null；**不**改 preview_m；**不** SAFE_STOP |
| **不是** | Local Physical Trajectory；不是 Safety；不是认证可执行轨迹 |

---

## 3. Local Physical Trajectory

| | |
|--|--|
| **定义** | 有限时间 horizon 内差速积分得到的局部轨迹 |
| **特点** | time-based；DD：`x+=v cosθ dt`…；短 horizon；动态 |
| **当前 horizon** | Local ≈1.5 s，MPPI ≈1.6 s（P0-A **不改**） |
| **几何 truth（P0-A）** | poses × **VehicleGeometry footprint** → swept volume 近似 |
| **不是** | Global Corridor；禁止再称“长距离蓝带规划器” |

---

## 4. Safety Envelope

| | |
|--|--|
| **定义** | Safety Supervisor 判断 **当前 command 是否立即允许** 的最终边界 |
| **权威** | **HARD OVERRIDE** — planner / corridor / probe 不得 bypass |
| **P0-A** | 行为不改；半径保留为 broad-phase approximation |

---

## 5. Vehicle Footprint

| | |
|--|--|
| **定义** | **唯一**车辆几何事实源：`VehicleGeometry` in `nav_geometry.py` |
| **表示** | 矩形 polygon（FL/FR/RR/RL）+ 采样点 |
| **Truth** | footprint polygon / samples = **narrow-phase truth** |
| **Approximation** | `planner_radius` / `local_radius` / `safety_radius` = **broad-phase only** |

所有模块必须：

```text
geometry = get_vehicle_geometry()  # DEFAULT_GEOM
```

禁止第二套 `AGV_LENGTH` / `BODY_RADIUS` 等散落常量。

**P0-A 适配器（行为冻结）：**

```text
nav_footprint.footprint_points / transform_footprint
  = narrow-phase polygon TRUTH（swept / corridor / audits / future P0-C/E）

local_maneuver._footprint_points
  = Probe / Local rollout 既有 0.45·L/W 采样集
  = 尺寸仍只来自 VehicleGeometry（无第二套常量）
  = 刻意保持 3F Probe decision 语义不变
```

将 Probe 切换到全 polygon narrow-phase = **后续阶段**（不可在本阶段偷偷做）。

---

## 6. Swept Volume

| | |
|--|--|
| **定义** | `⋃_i T_i(Footprint ⊕ margin)` 沿轨迹（含 pose 插值） |
| **允许近似** | spatial/angular 离散采样（非严格布尔运算） |
| **禁止** | 把 `centerline ± half_width` 当作完整 swept volume **最终 truth** |
| **P0-A** | Local PhysicalTrajectory / corridor 显示与 collision API 升级为 footprint swept |

---

## 7. Kinematic Feasible Path

| | |
|--|--|
| **定义** | Global（或候选）路径在车辆 κ / ω / footprint swept 下可执行 |
| **P0-A** | **不实现** validator（P0-C） |
| **P0-C** | **已实现** `KinematicPathValidator`：densify + κ=Δyaw/Δs + circular fillet + v=ω/κ speed profile + P0-A swept；telemetry only |
| **依赖** | P0-A swept / footprint API；控制限幅引用既有 `max_vx` / `wz_max=0.42` / `v_min=0.04`（不发明 MAX_W） |

---

## 8. Maneuver / Recovery / Replan

| Term | 定义 | P0-A |
|------|------|------|
| **Maneuver** | FSM 执行态：FORWARD / LOCAL_* / ALIGN / TURN / REVERSE_ESCAPE … | 行为不变 |
| **Recovery** | 3F ladder：`LOCAL_REVERSE`→`REVERSE_ESCAPE`→负 vx + progress | **必须保持 PASS** |
| **Replan** | 拓扑路径更换 | 不改 |

禁止本阶段把 `REVERSE_ESCAPE` 改成 `REVERSE_ARC`。

---

## 9. Layer ownership（摘要）

```text
Global Path          → topology reference
Global Ref Corridor  → long preview (P0-B)
Local Phys Traj      → short DD + swept (P0-A geometry)
Safety Envelope      → final command gate
VehicleGeometry      → single footprint truth
Recovery             → 3F execution (frozen semantics)
```

---

## 10. Sampling defaults（P0-A 实现值）

写入实现后以报告为准；contract 基线：

```text
SWEPT_SPATIAL_STEP_M   ≤ 0.10 m   (impl default 0.08)
SWEPT_ANGULAR_STEP_RAD ≤ 5°       (impl default 5°)
```
