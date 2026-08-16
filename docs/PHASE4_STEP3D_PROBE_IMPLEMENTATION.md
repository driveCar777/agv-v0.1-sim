# PHASE 4 STEP 3D — ProbeEngine Implementation

日期：2026-08-16  
状态：**PASS**  
`BEHAVIOR CHANGED = NO`（相对 3C：Probe 为 evidence-only，不授权换边 / reverse / recovery）  
`3E PREREQUISITES = PASS`

---

## 0. Verdict

| Check | Result |
|-------|--------|
| Forward / Backward / Left / Right / Turn | **IMPLEMENTED** |
| UNKNOWN / STALE | **IMPLEMENTED** |
| ObstacleSnapshot same-tick | **YES** |
| Footprint = `DEFAULT_GEOM` + `_collide_body` | **YES** |
| Auto side switch / reverse / recovery | **NO** |
| 3C soft KEEP / hard REVALIDATION | **PASS** (regression) |
| LIVE 19999 evidence-only | **PASS** (switches=0, authorized=false) |

---

## 1. Scope

建立 **ProbeEngine 未来可行性证据层**：

```text
ObstacleSnapshot → ProbeEngine.evaluate → ProbeBundle
  FORWARD / BACKWARD / LEFT / RIGHT / TURN_IN_PLACE
```

**禁止**：`side_switch_authorized=true`、自动换边、自动 reverse、改 Safety/MPPI 策略。

---

## 2. Architecture

```text
LocalMppiModel.step
  → build_obstacle_snapshot(same tick)
  → ProbeEngine.evaluate(snapshot)   # evidence
  → NavigationPolicy.step(...)       # unchanged by Probe
  → Maneuver / Selector / MPPI / Safety
  → telemetry attaches probe.to_dict()
```

Probe **不**写 `vx/w`，**不**改 FSM mode，**不**改 Selector 结果。

---

## 3. ObstacleSnapshot

字段：`timestamp / pose_ts / obstacle_ts / pose / vx / w / front/rear/left/right / collide / clearance_at / path / goal / corridor / signature / obstacle_key / skew_ms / age_s / validity`。

原则：同一 tick 供给全部方向。`skew > 400ms` 或 `age > 0.80s` → 整包 **STALE**。

---

## 4. ProbeConfig（集中参数）

| 参数 | 值 | 依据 |
|------|-----|------|
| `horizon_short/medium/long_m` | 0.50 / 1.05 / 1.80 | 3A + cruise 0.15–0.22 m/s |
| `horizon_vx_gain` | 2.5 | `horizon = clip(base + k\|vx\|)` |
| `min_clearance_m` | 0.18 | 对齐 `CLR_HARD` |
| `safety/localization/control_margin_m` | 0.08 / 0.05 / 0.05 | stopping_margin |
| `stale_timeout_s` | 0.80 | ~16 ticks @20Hz |
| `cache_ttl_static/dynamic_s` | 0.25 / 0.08 | 动态更短 |
| `reverse_vx` | −0.12 | 差速倒车 |
| `escape_min_m` | 0.35 | 逃生最短位移 |
| `turn_yaw_step/max` | 15° / 90° | footprint yaw sweep |

**未**为测试 PASS 放大阈值。

---

## 5–9. Directions

### Forward
- 复用 `rollout_candidate(FORWARD)` + `stopping_distance = v²/(2a) + reaction·v`
- `stopping_margin < 0` → `INVALID` / `STOPPING_MARGIN_NEGATIVE`
- 速度↑ → horizon↑（clip）

### Backward
- 负 `vx` 差速积分 + footprint swept（**不是** `rear_distance > thr`）
- 直线 + 小舵角 `BACKWARD_LEFT/RIGHT` 评估 `turning_space` / `escape_available`
- 无后方几何 → `UNKNOWN` / `NO_REAR_GEOMETRY`

### Left / Right
- 包装 `rollout_candidate(LEFT/RIGHT)`（与 Selector 同运动学 / footprint）
- 主弧失败时二次 `WZ_MAX` 弧（仍是证据，不是投票）
- **不**使用 Selector 的 `SOFT_FREE_SPACE` 伪可行（Probe 更严）

### Turn-in-place
- 位姿固定，yaw 扫描 `_collide_body`（防贴障纯转扫墙）
- 输出 `max_safe_yaw_delta` / `collision_yaw` / `rotation_valid`

---

## 10–11. Footprint / Collision

复用 `local_maneuver._footprint_points` / `_collide_body` + `DEFAULT_GEOM`（L≈1.05, W≈0.55）。禁止第三套中心点碰撞。

---

## 12–14. Horizon / UNKNOWN / Cache

- SHORT 每 cycle；MEDIUM 每 2；LONG 每 5  
- Status：`VALID | INVALID | UNKNOWN | STALE`；`UNKNOWN ≠ FREE`  
- Cache：`input_signature`（含 `obstacle_key=id(collide)`）+ TTL；障碍变化即失效  

---

## 15. Performance

| Source | P50 | P95 |
|--------|-----|-----|
| Offline open-world microbench (200) | **0.57 ms** | **0.81 ms** |
| LIVE `step3d_idle_open` (bundle_ms) | **~31 ms** | **~45 ms** |

LIVE 更高因真实 `world.collides` / clearance 查询。20Hz 预算内可接受；优先 cache + 分级 horizon（已实现），**未**删 Backward/Turn。

---

## 16–17. Telemetry / UI

- schema：`phase4_step3d_v1`
- `probe.implemented=true`（导航 tick 后）；idle 未 step 时可为 placeholder
- UI **NAV PROBE**：F/B/L/R/TURN status + clearance + stopping_margin / escape / max_yaw
- Events：`PROBE_STATUS_CHANGE` / `PROBE_HARD_FAIL` / `PROBE_RECOVERED` / `PROBE_STALE` / `PROBE_UNKNOWN`

---

## 18–19. Tests

`scripts/_audit_phase4_probe.py` → **PASS**（P1–P32 + I1–I6）

关键集成：

- `COMMITTED_LEFT` + Probe `LEFT INVALID` / `RIGHT VALID` → Selector 仍 `COMMIT_*`，**不**切 RIGHT  
- ProbeResult **无** `command_vx/w`  
- 3C commitment audit **PASS**；T/M regression 27+33 **PASS**

---

## 20. LIVE (19999)

Traces（不覆盖 STEP2/3B/3C）：

| File | Notes |
|------|-------|
| `docs/_phase4_trace/step3d_mid_inject_1786890887.jsonl` | hard mid_inject |
| `docs/_phase4_trace/step3d_mid_inject_soft_1786890791.jsonl` | soft |
| `docs/_phase4_trace/step3d_idle_open_1786890827.jsonl` | observe + latency |

Hard mid_inject（相对 STEP2 LEFT→RIGHT）：

```text
t≈1.7  COMMITTED_LEFT
t≈3.3  COMMIT_REVALIDATION_REQUIRED
Probe: F=INVALID L=INVALID R=INVALID B=VALID (early)
FSM: stay LOCAL_LEFT — NO LOCAL_RIGHT
side_switch.authorized = false
```

说明：Stage2 几何下 **右侧 footprint rollout 仍常 INVALID**（Probe 不做 Selector `SOFT_FREE_SPACE`）。这比 STEP2 假 `RIGHT_ONLY` 更保守，正是 3E 应消费的证据。Soft：`COMMIT_KEEP` 持续，Probe 可报 L/R VALID 变化，**仍不换边**。

---

## 21. Regression

| Suite | Result |
|-------|--------|
| `_audit_phase4_probe.py` | PASS |
| `_audit_phase4_commitment.py` | PASS |
| `_audit_local_maneuver.py` | PASS 27 |
| `_audit_maneuver_cases.py` | PASS 33 |

---

## 22. Known Limitations

1. Idle 未跑 `LocalMppiModel.step` 时 debug 可能仍见 Probe placeholder  
2. Side Probe 比 Selector soft fallback **更严** → LIVE Stage2 未必出现 `RIGHT=VALID`  
3. `confidence=null`（无可靠标定，不伪造）  
4. 动态障碍不做速度外推（占位 `obstacle_type`）  
5. 地图轨迹可视化未做（优先级低于正确性）  
6. LIVE probe_ms 含 world query，高于离线空场  

---

## 23. 3E Inputs（已具备）

```text
current side Probe status
alternative side Probe status
forward / backward escape / turn feasibility
commitment phase (REVALIDATION_REQUIRED)
side_switch_authorized = false  (until 3E)
```

---

## Q1–Q10

| # | Answer |
|---|--------|
| Q1 Forward predicts? | Diff-drive short rollout + stopping_margin |
| Q2 Backward predicts? | Negative-vx footprint trajectory + escape/turn space |
| Q3 Why not rear_distance alone? | Point clearance ≠ swept reverse path |
| Q4 L/R vs Selector? | Same `rollout_candidate` kinematics/footprint；Probe=feasibility，Selector=preference |
| Q5 Turn prevents wall sweep? | Yaw sweep `_collide_body` at fixed x/y |
| Q6 UNKNOWN/STALE? | NO_DATA / NAN / age / skew → never treated as FREE |
| Q7 Probe modifies vx/w? | **NO** |
| Q8 Auto side switch? | **NO** |
| Q9 Auto reverse? | **NO** |
| Q10 3E gets? | F/B/L/R/TURN feasibility evidence + commitment failure class |

---

## Files

### ADDED
- `agv_bridge/nav_probe.py`
- `scripts/_audit_phase4_probe.py`
- `scripts/_trace_phase4_probe_live.py`
- `scripts/_analyze_phase4_step3d.py`
- `docs/PHASE4_STEP3D_PROBE_IMPLEMENTATION.md`

### MODIFIED
- `nav_models.py` — snapshot + evaluate（不改决策）
- `sim_api_ext.py` — attach probe telemetry
- `nav_debug.py` / `nav_phase4_telemetry.py` / `nav_ui.js`

### Code audit
`nav_probe.py` 无 `force_vx` / `force_w` / `LOCAL_RIGHT` / authorize 路径。

---

## Completion Checklist

- [x] ProbeEngine + Snapshot + Config + Bundle  
- [x] F/B/L/R/TURN  
- [x] UNKNOWN/STALE + cache + footprint reuse  
- [x] Telemetry + UI  
- [x] Unit/integration PASS  
- [x] LIVE traces + 3C regression  
- [x] Evidence-only（无换边）  
- [x] Stop before 3E  

## Final

```text
3D STATUS = PASS
BEHAVIOR CHANGED = NO
PROBE LATENCY P50/P95 (offline) = 0.57 / 0.81 ms
PROBE LATENCY P50/P95 (LIVE bundle) ≈ 31 / 45 ms
FORWARD = IMPLEMENTED
BACKWARD = IMPLEMENTED
LEFT = IMPLEMENTED
RIGHT = IMPLEMENTED
TURN = IMPLEMENTED
LIVE = PASS (evidence-only)
REGRESSION = PASS
3E PREREQUISITES = PASS
```

下一阶段：

```text
STEP 3E — Side Switch Authorization
```

（本报告结束后停止，不自动进入 3E。）
