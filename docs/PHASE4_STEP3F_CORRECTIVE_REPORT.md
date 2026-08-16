# PHASE 4 STEP 3F-CORRECTIVE — Recovery Execution + Main Map Corridor + Vehicle-Centric Candidates

日期：2026-08-16  
状态：**PASS**  
`3G PREREQUISITES = PASS`（本报告验收通过后方可进入 3G；**尚未进入 3G**）

---

## 0. Verdict

| Area | Before (claimed 3F) | After corrective |
|------|---------------------|------------------|
| Recovery **decision** | PASS (`action=LOCAL_REVERSE`) | PASS |
| Recovery **execution** | **FAIL** (`vx≈0`, `w≠0`, REPOSITION/TURN) | **PASS** |
| Main map blue corridor | FAIL / incomplete | **PASS** (`nav.physical_trajectory` + `setGuideBand`) |
| Local candidate geometry | Misleading world-frame “lateral” look | **PASS** (vehicle-centric arcs) |

```text
3F-CORRECTIVE STATUS = PASS
3G PREREQUISITES = PASS
```

---

## 1. 原 3F 报告中的问题

### Recovery action ≠ execution

Telemetry 可显示：

```text
recovery.action = LOCAL_REVERSE
```

但 LIVE 实际：

```text
mode = REPOSITION / TURN_IN_PLACE
force_vx ≈ 0   (or ±0.06)
force_w ≠ 0
→ 原地转圈，无法脱困
```

**Decision PASS ≠ AGV escaped PASS。**

### Main Map

PHYSICAL TRAJECTORY 组件有数据 ≠ 主地图蓝带可见。`applySnap` 只读 debug，且 VALID→绿色；`nav` snapshot 无 corridor。

### Local Candidate

`rollout_candidate` 本是差速积分，但 UI 用世界坐标 auto-fit，易被看成横向平移。

---

## 2. COR-A 审计结论（只读）

详见 `docs/PHASE4_STEP3F_CORRECTIVE_AUDIT.md`。

### Q1 — 谁把 LOCAL_REVERSE 转成负 vx？

**应然链路：**

```text
nav_recovery.evaluate_recovery
  → LocalMppiModel.step (policy_ctx.recovery_action)
  → ManeuverFSM.decide → mode=REVERSE_ESCAPE, force_vx<0
  → DiffDriveMppi.step (directed REVERSE_ESCAPE)
  → apply_safety
  → physics / state.vx
```

**原缺陷：** FSM **不消费** `recovery_action`；大航向误差时优先 `TURN_IN_PLACE`/`REPOSITION`；`nav_models` 在 ALIGN/TURN 时清掉 `want_rev`。

### Q2 — 为何 mode=REPOSITION 而非 REVERSE_ESCAPE？

旧 FSM 优先级：`HEADING_ALIGN` / `STUCK→REPOSITION` 压过 dead-end reverse（尤其 `rot_safe=True`）。

### Q3 — 为何 vx≈0, w≠0？

ALIGN/TURN：`force_vx=0`, `force_w≠0` → MPPI 定向执行纯旋转。  
属 **Case B：从未发出 reverse command**，不是 Safety 猜阻。

### 附加 Probe 缺陷（LIVE S1）

三面堵时旧 `_probe_backward` 在侧向转向空间不足时把 **直线后退仍畅通** 判为 `DEAD_END → B=INVALID`，导致梯子直接 `SAFE_STOP`。已改为直线后退足够则 `B=VALID`（`LIMITED_TURN_AFTER_REVERSE`）。

---

## 3. 修复摘要

### COR-B — Recovery 真正执行

| Change | File |
|--------|------|
| `recovery_action∈{LOCAL_REVERSE,HISTORICAL_RETREAT}` → 强制 `REVERSE_ESCAPE`，跳过 TURN/REPOSITION | `maneuver.py` |
| `force_vx=-0.12`, `force_w≈0`（`TEMPORARY_REVERSE_TRACKER`） | `maneuver.py` / `mppi_controller.py` |
| 纵向 `signed_progress_m`（禁止用 yaw 冒充 progress） | `maneuver.py` |
| Stall / `RECOVERY_MOVE_TIMEOUT_S` → `RECOVERY_EXECUTION_STUCK/FAILED` | `maneuver.py` |
| `SAFE_STOP`/`REPLAN` policy → 禁止无限纯转 | `maneuver.py` |
| Telemetry `recovery.execution` / `progress` | `nav_phase4_telemetry.py` / `sim_api_ext.py` |

### COR-C — 主地图蓝带

| Change | File |
|--------|------|
| `nav.physical_trajectory` 写入 `/api/state` | `run_web_sim.py` |
| `applySnap` 优先 nav + debug corridor；recovery→cyan-blue | `sim_main.html` |
| Main map `prefer_blue`（不全改绿） | `sim3d.js` |

### COR-D — Vehicle-centric 局部候选

| Change | File |
|--------|------|
| 局部候选 canvas：上=车头、下=车尾；body 坐标画弧 | `sim_main.html` |
| 优先 Probe F/L/R/B poses（与主地图同源） | `sim_main.html` |

### Audits

- `scripts/_audit_phase4_recovery_execution.py`
- `scripts/_audit_phase4_physical_corridor.py`
- `scripts/_audit_phase4_candidate_geometry.py`
- `scripts/_trace_phase4_recovery_corrective_live.py`

---

## 4. LIVE S1 真实时间线（验收）

```text
T+2.46: INJECT three-side (F/L/R seal, rear open)
T+3.18: RECOVERY=LOCAL_REVERSE
T+3.18: mode=REVERSE_ESCAPE
T+3.18: requested_vx=-0.120
T+3.18: safe_vx=-0.120
T+3.18: state_vx=-0.115
T+5.34: pose moved -0.12m (signed longitudinal back)
endless_spin_ticks=0
→ S1 PASS
```

**不是** “action=LOCAL_REVERSE 就算 PASS”。

### S2 / S6

| Scene | Result |
|-------|--------|
| S2 四面笼 | `stop_or_replan=True`, `spin_ticks=0`, **PASS** |
| S6 侧仍 VALID | `illegal_reverse=0`, **PASS** |

---

## 5. Offline / Regression

| Audit | Result |
|-------|--------|
| Recovery execution | PASS |
| Physical corridor | PASS |
| Candidate geometry | PASS |
| Commitment (3C) | PASS |
| Side-switch auth (3E) | PASS |
| Probe (3D) | PASS |

---

## 6. Completion checklist

### Recovery execution

- [x] LOCAL_REVERSE → 负 `requested_vx`
- [x] Safety 允许时 `safe_vx < 0`
- [x] `state.vx < 0`
- [x] pose 纵向后退 progress
- [x] 无无限纯旋转
- [x] reverse timeout / stuck 可识别
- [x] SAFE_STOP / REPLAN 可识别

### Physical Corridor

- [x] Backend corridor edges
- [x] Main map payload (`nav.physical_trajectory`)
- [x] Local candidate Probe 同源
- [x] Recovery rearward tint

### Geometry

- [x] F/L/R/B 差速弧
- [x] Vehicle-centric UI
- [x] 无 lateral teleport（audit）

---

## 7. 仍属临时 / 后续

- Reverse 跟踪器标记为 `TEMPORARY_REVERSE_TRACKER`（直线 Probe path；非完整 MPPI reverse path track）。
- S1 脱困位移目前达 `min_progress`（~0.12m）；更长走廊脱困与 Historical Retreat 执行可在后续强化，**不阻塞本 corrective PASS**。
- **不要进入 3G，直到用户明确下令。**

---

## 8. Final

```text
原问题：Recovery decision 与 execution 不一致；主地图蓝带缺失；局部候选易误解为横移。
修复：Policy recovery 强制 REVERSE_ESCAPE + 负 vx；Probe 三面堵不再误杀直线后退；主地图接入 active corridor；局部候选 vehicle-centric。
LIVE：S1 真实负 vx + 后退位移；S2 停转；S6 不误 reverse。
STATUS：3F-CORRECTIVE = PASS ；3G PREREQUISITES = PASS
```
