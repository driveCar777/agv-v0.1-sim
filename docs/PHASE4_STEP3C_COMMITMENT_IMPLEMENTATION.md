# PHASE 4 STEP 3C — Avoidance Commitment Implementation

日期：2026-08-16  
状态：**PASS**  
`BEHAVIOR CHANGED: **YES** (intended — block unauthorized LEFT↔RIGHT)`  
`3D prerequisites: **PASS** (commitment gate live; Probe still NOT_IMPLEMENTED)`

---

## 0. Verdict

| Check | Result |
|-------|--------|
| Soft mid-inject keeps LEFT | **PASS** (`COMMIT_KEEP`, switches=0) |
| Hard mid-inject blocks RIGHT_ONLY | **PASS** (`COMMIT_REVALIDATION_REQUIRED`, no `LOCAL_RIGHT`) |
| Vs STEP2 mid_inject LEFT→RIGHT @ ~3.3s | **FIXED** (3C: stay `LOCAL_LEFT`) |
| Probe / authorize switch | **Not in 3C** (3D / 3E) |

---

## 1. Scope

实现 **Avoidance Commitment** 行为记忆：

- Policy 拥有 commitment lifecycle + `allow_side_compare` 门控
- Selector 在 commit 下输出 `COMMIT_KEEP` / `COMMIT_REVALIDATION_REQUIRED`
- FSM 用 `eval_sides = allow_side_compare OR commitment_active` 继续评估可行性
- **不**实现 Probe（3D）、**不**授权换边（3E）、不改 Safety/MPPI 业务择边

---

## 2. Files Changed

### ADDED

| File | Why |
|------|-----|
| `agv_bridge/nav_commitment.py` | Commitment 模型 / phases / fail / auth constants |
| `scripts/_audit_phase4_commitment.py` | Offline KEEP / REVALIDATION 审计 |
| `scripts/_analyze_phase4_step3c.py` | Trace 分析 |
| `docs/PHASE4_STEP3C_COMMITMENT_IMPLEMENTATION.md` | 本报告 |

### MODIFIED

| File | Why | Decision impact |
|------|-----|-----------------|
| `nav_policy.py` | create/update/release；LOCAL_* 下 `allow_side_compare=False` when active | **有** |
| `local_maneuver.py` | `_select` commitment gate；raw preference 调试字段 | **有** |
| `maneuver.py` | `eval_sides`；向下传 commitment_* | **有**（评估门，非换边） |
| `nav_models.py` | `policy_ctx` + post-cycle `update_commitment_after_selection` | **有** |
| `nav_phase4_telemetry.py` | `commitment.implemented=true`；schema `phase4_step3c_v1` | 观测 |
| `nav_debug.py` | 透传 commitment 字段 | 观测 |
| `nav_ui.js` | AVOIDANCE COMMIT 卡显示 fail/auth/age | 观测 |

### NOT MODIFIED (本步)

- ProbeEngine / `nav_probe.py`
- Side-switch **authorization**（`side_switch_authorized` 恒 `false`）
- MPPI / Safety 择边逻辑
- Recovery / Replan 大改

---

## 3. Architecture (3C)

```text
Policy.step
  → allow_side_compare := False if commitment.active
  → policy_ctx {commitment_active, committed_side, hard_fail, ...}
ManeuverFSM.decide
  → eval_sides = allow_side_compare OR commitment_active
  → LocalManeuverSelector.compare(... commitment_*)
Selector._select
  → if commit & !authorized:
       feasible → COMMIT_KEEP
       else     → COMMIT_REVALIDATION_REQUIRED (keep side token)
  → never RIGHT_ONLY / LOWER_TOTAL_COST switch while locked
Policy.update_commitment_after_selection (post-cycle)
  → CREATE / SOFT_DEGRADED / HARD_FAILED→REVALIDATING / RELEASE
```

---

## 4. Core Rules Implemented

| Condition | Behavior |
|-----------|----------|
| COMMITTED + side still feasible | `COMMIT_KEEP`；other cheaper = soft `TEMPORARY_COST_WORSE` |
| COMMITTED + side infeasible | keep side；`COMMIT_REVALIDATION_REQUIRED`；**no** direct RIGHT |
| OBSTACLE_PASSED / PATH_RECAPTURE / RETURN_FORWARD | RELEASE |
| `side_switch_authorized` | always false in 3C |

---

## 5. Offline Audit

`python scripts/_audit_phase4_commitment.py` → **PASS**

- soft → `LEFT` / `COMMIT_KEEP`
- hard → `LEFT` / `COMMIT_REVALIDATION_REQUIRED`（baseline without commit still `RIGHT_ONLY`）
- Policy hard_fail + soft LOCKED OK

回归：`_audit_local_maneuver.py` 27 PASS；`_audit_maneuver_cases.py` 33 PASS。

---

## 6. LIVE Comparison

| | STEP2 `mid_inject_1786888402` | STEP3C `step3c_mid_inject_1786890044` |
|--|-------------------------------|----------------------------------------|
| LEFT→RIGHT | **yes** @ ~3.331 `RIGHT_ONLY` | **no** (switches=0) |
| After Stage2 | `LOCAL_RIGHT` → Safety/STOP | stay `LOCAL_LEFT` |
| Selector | `RIGHT_ONLY` | `COMMIT_KEEP` → `COMMIT_REVALIDATION_REQUIRED` |
| allow_side_compare | True during LOCAL_* | False once committed |

| | Soft STEP2/3B | STEP3C `step3c_mid_inject_soft_1786890067` |
|--|---------------|---------------------------------------------|
| Switch | none | none |
| After Stage2 | HYSTERESIS_KEEP | **`COMMIT_KEEP`** + soft fail streak |

Traces（不覆盖 STEP2）：

- `docs/_phase4_trace/step3c_mid_inject_1786890044.jsonl`
- `docs/_phase4_trace/step3c_mid_inject_soft_1786890067.jsonl`

---

## 7. Hard mid_inject timeline (3C)

```text
t≈1.67  LOCAL_LEFT LEFT_ONLY  commit=COMMITTED_LEFT allow=True→False next ticks
t≈2.70  Stage2 hard clog left (RIGHT opens)
t≈2.94  COMMIT_KEEP (locked; no RIGHT)
t≈3.32  COMMIT_REVALIDATION_REQUIRED  (LEFT infeasible; still no LOCAL_RIGHT)
…       stay LOCAL_LEFT；RIGHT samples = 0
```

STEP2 同场景在 ~3.33 切 `LOCAL_RIGHT`。

---

## 8. Telemetry / UI

- `commitment.implemented=true`
- fields: active / side / phase / failure_reason / hard_fail / soft_fail_streak / authorization_status / age_s
- UI **AVOIDANCE COMMIT**：LOCKED/READY + fail/auth
- Probe 仍 `NOT_IMPLEMENTED`
- `side_switch.authorized=false`；status=`LOCKED_BY_COMMITMENT` | `REVALIDATION_REQUIRED`

---

## 9. Q1–Q12

| # | Question | Answer |
|---|----------|--------|
| Q1 | Who owns commitment? | `NavigationPolicy` (`AvoidanceCommitment`) |
| Q2 | Who selects side under commit? | Selector **executes KEEP/REVALIDATE**; cannot authorize switch |
| Q3 | Does MPPI choose side? | No |
| Q4 | Soft cost worse → switch? | **No** → `COMMIT_KEEP` + `TEMPORARY_COST_WORSE` |
| Q5 | Hard infeasible → switch? | **No** → `REVALIDATION_REQUIRED`（3E 才授权） |
| Q6 | Still evaluate L/R while locked? | **Yes** via `eval_sides` |
| Q7 | allow_side_compare when committed? | **False** |
| Q8 | Probe in 3C? | **No** |
| Q9 | LIVE hard mid_inject LEFT→RIGHT? | **Eliminated** |
| Q10 | Safety bypassed? | **No** |
| Q11 | Architecture rewrite? | **No** — Policy sub-state only |
| Q12 | Ready for 3D Probe? | **Yes** |

---

## 10. Known Limitations

1. Hard fail keeps infeasible side until 3E authorize / recovery — may linger in `LOCAL_LEFT` + Safety stop（预期）
2. Cross-scenario sticky commit briefly possible until `RETURN_FORWARD` / mission reset（已加 RETURN_FORWARD release）
3. Full side-switch authorization **not** implemented
4. Probe evidence **not** implemented
5. Soft fail streak 不触发换边（正确）；也不触发 replan（留给后续）

---

## Completion Checklist

- [x] Commitment model + Policy lifecycle
- [x] Selector `COMMIT_KEEP` / `COMMIT_REVALIDATION_REQUIRED`
- [x] `eval_sides` wiring
- [x] Telemetry `implemented=true` + UI
- [x] Offline audit PASS
- [x] LIVE mid_inject / soft → `step3c_*.jsonl`
- [x] Hard mid_inject 无 LEFT→RIGHT
- [x] 本报告
- [x] **Stop before 3D**

## Final

```text
3C STATUS = PASS
BEHAVIOR CHANGED = YES (commitment lock)
UNAUTHORIZED SIDE SWITCH = BLOCKED
PROBE = NOT_IMPLEMENTED
AUTHORIZE = NOT_IMPLEMENTED (3E)
```

下一阶段：

```text
STEP 3D — ProbeEngine (evidence only; still no auto side switch)
```
