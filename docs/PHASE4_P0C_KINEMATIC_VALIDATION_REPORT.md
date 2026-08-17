# PHASE4 P0-C — Kinematic Path Validation Report

日期：2026-08-17  
基线：`03f636268cda85a9ef439f479e2febdee4d251f8` / P0-B COMPLETE  
范围：**仅 P0-C KinematicPathValidator**（telemetry / reference only）  
禁止：P0-D / P0-E / P1 / 3G / Local horizon 加长 / cmd_vel / SAFE_STOP on INVALID / 发明 MAX_W 或缩小车体

```text
P0-C STATUS = PASS
P0-A  = PASS
P0-B0 = PASS
P0-B  = PASS
3C    = PASS
3D    = PASS
3E    = PASS
3F    = PASS
```

---

## Geometry resolution（不 BLOCK）

| 量 | 值 | 含义 |
|----|----|------|
| `length` | 1.05 m | 3D / 车体长度 |
| `width` | 0.55 m | 车体宽度 |
| narrow-phase half-length | **0.525 m** | `footprint_polygon_body` clamps `bumper_l` to `0.5*length` |
| `bumper_l` | 0.55 m | **MPPI bumper SAMPLE offset**（比车体前缘超前 2.5 cm），不是多边形半长 |
| `track_width_m` / `wheelbase_m` | **UNAVAILABLE** | 不发明轮半径 |

数值字段 **未改**。

---

## Q1 — Global Path 现在是否经过真实 kinematic validation？

```text
YES — telemetry only
KinematicPathValidator.validate_global_path(raw or processed)
status = VALID | INVALID | DEGRADED | NOT_VALIDATED
controls_vehicle = false
```

## Q2 — Validation 是否包含 x/y/yaw、κ、speed profile、footprint、swept？

```text
YES
densify (P0-B yaw convention) → κ = Δyaw/Δs
circular fillet (not path_quality chamfer)
v_i = min(v_max, w_max/|κ|) + acc_v
P0-A trajectory_collision / sample_rotation_sweep / footprint_points
```

## Q3 — 真实可减速执行的路径是否 PASS？

```text
YES
TEST B R≈2 m → VALID, not limited
TEST C R=0.5 m → VALID + speed_limited (w_req≈0.80, vmin≈0.21 ≥ 0.04)
TEST E 4 m 90° arms → VALID + fillet
```

## Q4 — 不可执行的硬角是否 FAIL？

```text
YES
TEST D 0.20 m 90° arms → INVALID NO_BOUNDED_CURVATURE_TRANSITION
不使用 v=0 原地转证明 VALID
```

## Q5 — 内圈/外圈碰撞是否能被抓住？

```text
YES
TEST F inner → INVALID FOOTPRINT_COLLISION
TEST G outer → INVALID FOOTPRINT_COLLISION
TEST H clearance 0.03 m → DEGRADED (kinematic_valid=null)
```

## Q6 — Validator 是否复用 P0-A VehicleGeometry？

```text
YES — get_vehicle_geometry() / footprint_polygon_body / trajectory_collision
```

## Q7 — Validator 是否改变了 Local Planner？

```text
NO
```

## Q8 — Validator 是否改变了 Safety？

```text
NO — 不 SAFE_STOP on INVALID；不改 cmd_vel
```

## Q9 — 3F 是否保持 PASS？

```text
YES — 3C/3D/3E/3F offline + 3F LIVE S1 仍要求真实 reverse
```

## Q10 — Global Preview 是否仍是 REFERENCE ONLY？

```text
YES
geometry_status = REFERENCE_ONLY
INVALID 不把 preview_m 置 0
P0-B 长预览仍约 5 m
```

---

## Kinematic numbers (audit A–L)

```text
v_max = 0.40
w_max = min(0.42, geom.max_w=0.45) = 0.42
v_min_forward = 0.04
v_ref = 0.40
acc_v = 0.9
κ > 0 = LEFT
min fillet R (|Δψ|≥60° cannot fit) = 0.5 * width = 0.275 m
```

| Test | Result |
|------|--------|
| A straight | VALID, κ=0, R=None |
| B R≈2 m | VALID, κ≈0.50, R≈2.00 |
| C R=0.5 m | VALID, speed_limited |
| D 0.20 m 90° | INVALID `NO_BOUNDED_CURVATURE_TRANSITION` |
| E 4 m 90° | VALID, fillet |
| F inner obstacle | INVALID `FOOTPRINT_COLLISION` |
| G outer obstacle | INVALID `FOOTPRINT_COLLISION` |
| H clearance 0.03 | DEGRADED |
| I ~180° | INVALID `HEADING_REVERSAL`, needs_reverse, no reverse traj |
| J cage + short corner | INVALID + `ROTATION_SWEEP_COLLISION` |
| K tight then straight | speed_limited |
| L revision 21→22 | cache hit then miss, new validation_id |
| 50 m path | 10.04 ms, 501 pts, 9009 swept |

---

## LIVE

Outdoor open straight: `KINEMATIC=VALID`, `κ=0`, `preview_m≈4.91`, `controls_vehicle=false`.  
Indoor A* often `INVALID FOOTPRINT_COLLISION` (planner broad-phase radius 0.25 m vs footprint width 0.55 m) — **expected diagnostic**, not a control change. Preview stayed ~4.9–5.0 m.

P0-B during outdoor nav: `GLOBAL≈4.9–5.0 m` + `LOCAL_MAX≈0.33–0.38 m`.

3F LIVE: S1 `LOCAL_REVERSE`→`REVERSE_ESCAPE`→`vx<0`→`signed_back=0.125 m`; S2/S6 PASS.

---

## Known limitations

```text
P0-D not implemented   (Approach)
P0-E not implemented   (Rotation Guard FSM)
P1 not implemented     (Reverse Arc / local horizon)
track_width unavailable
wheelbase unavailable
```

Corner transition 是 **circular fillet 近似**（tangent-in / arc / tangent-out），不是 Hybrid A*。  
`path_quality.smooth_path_collision_checked` 的线性 chamfer **不是** kinematic truth。  
Cache token = dyn-obstacle **count**（同数量换位可能滞后一次刷新）。  
INVALID 只染色 / 打日志，Local 仍按短 horizon 跑。

---

## Files

| Path | Change |
|------|--------|
| `ros2_ws/.../nav_kinematic.py` | **NEW** validator + fillet + speed profile + swept |
| `ros2_ws/.../nav_observability.py` | category `KINEMATIC`; events; decision/summary fields |
| `ros2_ws/.../sim_api_ext.py` | after-control validate + `get_nav_preview.kinematic_validation` |
| `scripts/run_web_sim.py` | `/api/state` `nav.kinematic_validation` + preview pass-through |
| `www/sim3d.js` | VALID slate / DEGRADED amber / INVALID split+dashed |
| `www/sim_main.html` | status + hdrMeta + legend + cache-bust |
| `scripts/_audit_phase4_kinematic_path.py` | Tests A–L |
| `scripts/_trace_phase4_kinematic_validation_live.py` | LIVE timeline A–F |
| `docs/PHASE4_NAVIGATION_LOG_API.md` | P0-C events / fields |
| `docs/PHASE4_NAVIGATION_KINEMATIC_CONTRACT.md` | P0-C implemented |

---

## STOP

未进入 P0-D / P0-E / P1 / 3G。
