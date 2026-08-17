# PHASE4 P0-A — Swept Footprint Report

日期：2026-08-17  
基线：`71c466b` / V0.11  
范围：**仅 P0-0 Contract Freeze + P0-A Unified VehicleGeometry + Swept Footprint**  
禁止：P0-B / P0-C / P0-D / P0-E / P1 / P2

```text
P0-A STATUS = PASS
3F REGRESSION = PASS
NEXT ALLOWED STEP = P0-B
```

---

## 1. Files changed

| Path | Change |
|------|--------|
| `docs/PHASE4_NAVIGATION_KINEMATIC_CONTRACT.md` | **NEW** — P0-0 contract freeze |
| `ros2_ws/.../nav_footprint.py` | **NEW** — footprint / interpolate / swept / rotation / collision APIs |
| `ros2_ws/.../nav_geometry.py` | Extend `VehicleGeometry` aliases + optional UNAVAILABLE params; `get_vehicle_geometry()` |
| `ros2_ws/.../nav_trajectory.py` | Local corridor edges from swept footprint (not ribbon-as-truth) |
| `ros2_ws/.../local_maneuver.py` | Keep probe/rollout 0.45 sample set; sizes still from `DEFAULT_GEOM` only |
| `ros2_ws/.../nav_debug.py` | Geometry telemetry: footprint model, broad/narrow phase, sampling steps |
| `ros2_ws/.../nav_phase4_telemetry.py` | Attach `vehicle_geometry` via `geometry_telemetry()` |
| `scripts/_audit_swept_footprint.py` | **NEW** — Tests A–G |
| `docs/PHASE4_P0A_SWEPT_FOOTPRINT_REPORT.md` | **THIS FILE** |

未改行为：`NavigationPolicy` / GlobalPlanner / ManeuverFSM / MPPI horizon / Obstacle Approach / Rotation policy / Replan / Commitment / Side-switch / Recovery FSM。

---

## 2. VehicleGeometry final schema

单一事实源：`nav_geometry.DEFAULT_GEOM` / `get_vehicle_geometry()`（AMB-150 既有尺寸，**未猜新数**）。

```text
length / length_m              = 1.05
width / width_m                = 0.55
bumper_l / front_overhang_m    = 0.55
rear_overhang_m                = 0.55  (symmetric; same bumper_l)
center_offset_x_m              = 0.0
safety_margin_m                = 0.08
planner_radius                 = 0.25   # broad-phase approx
local_radius                   = 0.24   # broad-phase approx
safety_radius                  = 0.28   # broad-phase approx
track_width_m                  = None   # UNAVAILABLE
wheelbase_m                    = None   # UNAVAILABLE
```

Body frame（contract）：`+x` front，`+y` left。

---

## 3. Footprint model

```text
footprint_model = "polygon"
corners = FL → FR → RR → RL
samples  = 4 corners + front/rear/left/right midpoints + center
```

API：`footprint_polygon_body`, `transform_footprint`, `footprint_points`, `footprint_collide`。

---

## 4. Swept sampling algorithm

1. `interpolate_poses` — 按距离与 yaw 差插入中间 pose  
2. 每 pose：`transform_footprint` → polygon + 9 点 samples  
3. `swept_boundary_edges` — 每 pose 取 body-+y 最大 / body-−y 最小顶点 → left/right outer edges（捕获转弯外扩与 overhang）  
4. `sample_rotation_sweep` — 定点 yaw 扫掠（几何 only，不改 FSM）

**实现值：**

```text
SWEPT_SPATIAL_STEP_M     = 0.08 m   (≤ 0.10)
SWEPT_ANGULAR_STEP_RAD   = 5.0°
```

---

## 5. Broad-phase / narrow-phase separation

| Layer | Model | Role |
|-------|--------|------|
| Broad | `bounding_radius` (`planner`/`local`/`safety`) | early reject / soft approx |
| Narrow | `footprint_polygon_samples` | final geometry truth for swept / audits |

Telemetry：`collision_model_broad_phase` / `collision_model_narrow_phase`。

**Probe/Local adapter（行为冻结）：** `local_maneuver._footprint_points` 仍用历史 `0.45·L/W` 采样（尺寸只读 `VehicleGeometry`）。全 polygon 进 Probe = 后续阶段，避免破坏 3F。

---

## 6. Collision examples

- **E Narrow obstacle：** 中心线 clear，侧向障碍撞 footprint → `collision=TRUE`  
- **F Radius FN：** `local_radius` 判 safe，前左角撞障碍 → footprint `COLLISION`  
- Local corridor `geometry_model = "swept_footprint_polygon"`（不再把 ribbon 当 truth）

---

## 7. Test commands

```bash
# from V0.1仿真版
set PYTHONPATH=ros2_ws\src\agv_bridge   # PowerShell: $env:PYTHONPATH=...

python scripts/_audit_swept_footprint.py
python scripts/_audit_phase4_commitment.py      # 3C
python scripts/_audit_phase4_probe.py           # 3D
python scripts/_audit_phase4_side_switch.py     # 3E
python scripts/_audit_phase4_recovery_execution.py  # 3F offline
# web sim must be running on :19999
python scripts/_trace_phase4_recovery_corrective_live.py  # 3F LIVE S1/S2/S6
```

---

## 8. Test results

| Suite | Result |
|-------|--------|
| Swept A–G | **PASS** |
| 3C Commitment | **PASS** |
| 3D Probe | **PASS** |
| 3E Side-switch | **PASS** |
| 3F Recovery offline | **PASS** |

---

## 9. 3F regression results

LIVE (`_trace_phase4_recovery_corrective_live.py`)：

| Scene | Result | Notes |
|-------|--------|-------|
| **S1** | **PASS** | `LOCAL_REVERSE` → `REVERSE_ESCAPE`; `requested/safe/state_vx < 0`; signed back ≥0.12 m; `spin_ticks=0` |
| **S2** | **PASS** | stop/replan; no reverse; no spin |
| **S6** | **PASS** | no illegal reverse while side VALID |

```text
3F REGRESSION = PASS
```

第一次把 Probe 直接接到全 polygon 时曾出现 S1 不触发 `LOCAL_REVERSE`；已回退 Probe 采样 adapter，**未改 Recovery FSM**。

---

## 10. Known limitations

1. Probe/Local rollout 尚未使用全 polygon narrow-phase（有意冻结，留给后续）。  
2. Swept 为离散采样，非精确布尔体积。  
3. `track_width` / `wheelbase` = UNAVAILABLE（不伪造）。  
4. Global Reference Corridor / Preview = **未实现**（P0-B）。  
5. KinematicPathValidator / Rotation Guard / Approach = **未实现**。  
6. Local horizon 仍为既有短时域（未做 P1）。

---

## 11. Why P0-B can safely consume this API

P0-B Global Preview 只需：

```text
poses(along global densify)
  → get_vehicle_geometry()
  → sample_swept_footprint / swept_boundary_edges / trajectory_collision
```

与 Local Physical Trajectory **同一** `VehicleGeometry` + **同一种** swept 语义；不必再发明第二套尺寸或 ribbon 公式。  
本阶段未碰 Global Planner 行为；P0-B 可在此 API 上叠加 2–8 m preview，而不回退 3F。

---

## Completion checklist

```text
[x] VehicleGeometry 唯一 Truth Source
[x] planner/local/safety radius = approximation（职责写明）
[x] footprint polygon 可统一调用（nav_footprint）
[x] swept footprint sampling 可调用
[x] pose interpolation 已实现
[x] local PhysicalTrajectory 不再只是 ribbon truth
[x] rotation sweep 基础 API 存在
[x] 内圈/外圈由 footprint corners 表示（非仅中心线）
[x] 没有新建第二套几何常量
[x] 3C PASS
[x] 3D PASS
[x] 3E PASS
[x] 3F PASS
```

---

## Stop

```text
P0-A STATUS = PASS
3F REGRESSION = PASS
NEXT ALLOWED STEP = P0-B
```

**停止。不进入 P0-B，直至用户明确授权。**
