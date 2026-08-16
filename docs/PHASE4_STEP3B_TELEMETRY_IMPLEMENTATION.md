# PHASE 4 STEP 3B — Telemetry Implementation

日期：2026-08-16  
状态：**PASS**  
`BEHAVIOR CHANGED: **NO**`  
`3C prerequisites: **PASS**`

---

## 1. Scope

只增加可观测性：Commitment / Probe / Side-switch **schema placeholders** + 现有 Policy/Selector/FSM/Controller/Safety **事实拆分** + 结构化 observe 事件。

**未修改**任何导航决策条件或 `_select` / `allow_side_compare` / Safety / MPPI。

## 2. Files Changed

### MODIFIED

| File | Why | Decision impact |
|------|-----|-----------------|
| `agv_bridge/nav_debug.py` | 组装 `phase4` 块、事件镜像 | **无** |
| `delivery_web/www/nav_ui.js` | NAV PROBE / AVOIDANCE COMMIT / SIDE SWITCH 组件 | **无** |
| `scripts/_trace_phase4_side_switch.py` | JSONL 透传 phase4 字段 | **无** |

### ADDED

| File | Why |
|------|-----|
| `agv_bridge/nav_phase4_telemetry.py` | 只读 Phase4 schema + `Phase4ObserveTracker` |
| `scripts/_audit_phase4_telemetry.py` | API schema / STEP2 对比审计 |
| `docs/PHASE4_STEP3B_TELEMETRY_IMPLEMENTATION.md` | 本报告 |

### NOT MODIFIED (决策路径)

- `nav_policy.py` 决策条件
- `maneuver.py` / `local_maneuver.py` selection
- `mppi_controller.py`
- `apply_safety` / stuck / recovery / replan

## 3. Fields Added

Top-level `/api/nav/debug`（增量，保留旧 `nav_policy` / `local_maneuver`）：

```text
phase4.*          # 完整块
commitment        # placeholder implemented=false
probe             # NOT_IMPLEMENTED stubs
side_switch       # authorization_status=NOT_IMPLEMENTED
sides             # policy/selector/fsm/controller/actual 拆开
ownership         # CURRENT vs TARGET authority
phase4_events     # observe ring tail
performance.phase4_build_ms
```

## 4. Event Types Added

| Event | Meaning |
|-------|---------|
| `SIDE_SELECTED` | NONE → LEFT/RIGHT 首次 |
| `SIDE_SWITCH_OBSERVED` | 实际换边（**不是** AUTHORIZED） |
| `FSM_TRANSITION` | mode 变化 |
| `SAFETY_OVERRIDE` / `SAFETY_RELEASE` | Safety 限制变化 |

仅状态变化时记录；ring maxlen=200。

## 5. API Changes

- 向后兼容：旧字段保留
- 新字段全部 optional / additive
- Placeholder 诚实：`commitment.implemented=false`，**不**把 `avoid_side` 冒充 commit

## 6. UI Changes

`+组件` 新增（半透明 / 拖动 / resize / 仅 X）：

- **NAV PROBE** → 全部 `NOT_IMPLEMENTED`
- **AVOIDANCE COMMIT** → `NOT IMPLEMENTED` + 下方 CURRENT FACTS（selector/fsm/…）
- **SIDE SWITCH** → `authorization_status=NOT_IMPLEMENTED` + CURRENT authority=`LocalManeuverSelector` + TARGET=`NavigationPolicy` + observe 事件

## 7. Trace Format

`docs/_phase4_trace/step3b_mid_inject_*.jsonl`  
`docs/_phase4_trace/step3b_mid_inject_soft_*.jsonl`  

（STEP2 原文件 **未覆盖**）

每 sample 可含：`phase4` / `commitment` / `probe` / `side_switch` / `sides` / `ownership` / `phase4_events`。

## 8. Current vs Target Fields

| Concept | CURRENT (3B) | TARGET (3C+) |
|---------|--------------|--------------|
| Side selection authority | `LocalManeuverSelector` | Policy `side_switch_authorized` |
| Commitment | `implemented=false` | AvoidanceCommitment |
| Probe | `NOT_IMPLEMENTED` | ProbeEngine F/B/L/R |
| Switch event | `SIDE_SWITCH_OBSERVED` | + `SIDE_SWITCH_AUTHORIZED` |

## 9. Baseline Comparison

| | STEP2 `mid_inject_1786888402` | STEP3B `step3b_mid_inject_1786889366` |
|--|-------------------------------|----------------------------------------|
| LEFT→RIGHT | yes @ 3.331 | yes @ 3.229 |
| reason | RIGHT_ONLY | RIGHT_ONLY |
| mode seq | … LOCAL_LEFT → LOCAL_RIGHT → REPOSITION … | 同构 |
| allow_side_compare | True | True |

时间戳有调度抖动（±0.1s），**决策序列等价** → telemetry 未改变行为。

## 10. STEP2 Replay

在切边帧 `t≈3.229` telemetry 可还原：

```text
POLICY: behavior=AVOID_LEFT, avoid_side=LEFT, allow_side_compare=True
COMMITMENT: implemented=False, side=NONE
SELECTOR: RIGHT, reason=RIGHT_ONLY, L=700, R=75
FSM: LOCAL_RIGHT
CONTROLLER: mppi_w=-0.42
SAFETY: FRONT_OBSTACLE / safe_vx=0
SIDES: policy=LEFT | selector=RIGHT | fsm=RIGHT | controller=RIGHT
EVENTS: SIDE_SWITCH_OBSERVED (LocalManeuverSelector), FSM_TRANSITION LOCAL_LEFT→LOCAL_RIGHT
```

**Authority conflict 一目了然**（Policy 标签 vs Selector/FSM）。

## 11. Performance

| Metric | Value |
|--------|-------|
| `phase4_build_ms` (API) | ~0.03–0.05 ms |
| 决策路径额外 rollout | **无** |

无明显拖慢。

## 12. Thread Safety

- Debug 仍由现有 snapshot 构建路径输出
- Phase4 组装包在 `try/except`：失败不崩导航环
- Event mirror 使用既有 `EventRing` 去重/上限

## 13. Regression

- 19999 重启后 API schema OK
- mid_inject 仍 LEFT→RIGHT
- mid_inject_soft 仍无切边（对照）
- 旧 UI 组件未删除

## 14. Known Limitations

1. `hysteresis_state` / `hold_state` = `NOT_AVAILABLE`（未造假）
2. `progress.local` = `NOT_IMPLEMENTED`
3. `obstacle_snapshot` schema only
4. 细粒度 `policy_ms`/`mppi_ms` 仍 `NOT_AVAILABLE`
5. `actual_motion_side` 在切边瞬间可能滞后一帧（state_w 惯性）— 如实暴露

---

## Completion Checklist

- [x] 决策逻辑未改
- [x] Commitment / Probe / Auth 仅为 placeholder
- [x] Side identity 拆分可见
- [x] Selector / FSM / Controller / Safety 可见
- [x] `SIDE_SWITCH_OBSERVED` + `FSM_TRANSITION`
- [x] Trace 落盘（不覆盖 STEP2）
- [x] API + UI
- [x] STEP2 可完整还原
- [x] behavior unchanged
- [x] 本报告

## Final

```text
3B STATUS = PASS
BEHAVIOR CHANGED = NO
3C PREREQUISITES = PASS
```

下一阶段才是：

```text
STEP 3C — Avoidance Commitment
```

（本报告结束后停止，不提前实现 Commitment 控制效果。）
