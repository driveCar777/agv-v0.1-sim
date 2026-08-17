# PHASE4 P0-B-0 — Navigation Observability Report

日期：2026-08-17  
基线：`71c466b` / V0.11  
范围：**仅诊断基础设施**（Event / Trace / Snapshot / API Access Log）  
禁止：P0-B Global Preview / P0-C / P0-D / P0-E / P1 / P2 / 任何 Planning 行为修改

```text
P0-B0 OBSERVABILITY = PASS
3F REGRESSION = PASS
LOGGING OVERHEAD = 0.11 ms EMA (live); 0.05 ms avg (audit ingest)
API READY FOR FUTURE DEBUG UI = YES
NEXT ALLOWED STEP = P0-B
```

---

## 1. Files changed

| Path | Change |
|------|--------|
| `ros2_ws/.../nav_observability.py` | **NEW** — unified diagnostic bus |
| `ros2_ws/.../nav_log_forensics.py` | **NEW** — pure `likely_owner` rules (auditable) |
| `ros2_ws/.../sim_api_ext.py` | After `_refresh_debug_snapshot`：ingest only；挂载 log APIs |
| `scripts/run_web_sim.py` | `/api/logs*` + `/api/nav/logs*`；HTTP access log 采样 |
| `scripts/_audit_phase4_navigation_stop_forensics.py` | **NEW** Cases A–F + filter tests |
| `scripts/_trace_phase4_navigation_deadlock.py` | **NEW** last-N-seconds dump |
| `docs/PHASE4_NAVIGATION_LOG_API.md` | **NEW** API contract |
| `docs/PHASE4_P0B0_OBSERVABILITY_REPORT.md` | **THIS FILE** |

未改：GlobalPlanner / Local horizon / ManeuverFSM 语义 / MPPI / Safety 阈值 / Recovery 3F / Probe 决策。

---

## 2. Four-layer model

```text
EVENT     NavLogEvent  (event_id / cycle_id / trace_id / level / category / event)
TRACE     TRACE-AVOID|RECOVERY|REPLAN-NNNNNN  → events[] + summary + outcome
SNAPSHOT  SNAP-NNNNNN  large decision (on STOP / interesting)
API LOG   separate ring  (never mixed into GET /api/logs by default)
```

IDs：

```text
event_id  EVT-000001
cycle_id  NAV-000123     # one debug refresh (~20 Hz)
trace_id  TRACE-AVOID-000017
```

---

## 3. Decision trace (per cycle)

`OBS.ingest_debug_snapshot(existing NavDebugHub snapshot)` 组装：

```text
observe → global → probe → policy → recovery
→ candidates (all, not winner-only) → selection → fsm
→ command (requested/safe/state) → safety → execution → diagnostics → performance
```

Global Preview 字段预留且为 `null`（未伪造）：`preview_m` / `max_curvature` / `kinematic_valid`。

Candidate 默认：endpoint / distance / score / valid / reject_reasons / score_breakdown。完整 poses 仅 `NAV_LOG_CANDIDATE_DETAIL=1`。

---

## 4. Detectors (observe-only)

| Event | Trigger (approx) |
|-------|------------------|
| `NAVIGATION_STOP_DIAGNOSTIC` | `\|state.vx\|<0.03` 持续 ≥0.4 s |
| `NAVIGATION_DEADLOCK_SUSPECTED` | 平移进度停滞 ≥3 s，非 goal/intentional |
| `SPIN_LOOP_SUSPECTED` | vx≈0 且 \|w\|>0.12 且平移≈0 ≥1.2 s |
| `PLANNING_STARVATION_SUSPECTED` | max candidate distance 持续明显短于 ~0.25 m 名义值 |
| `PLAN_EXECUTION_MISMATCH` | selected vs FSM / requested vs safe vs state |
| `SAFETY_CLAMP` | requested_vx>ε 且 safe_vx≈0 |
| `NAV_CYCLE_OVERRUN` | cycle_ms > 50 |

`likely_owner` 规则在 `nav_log_forensics.infer_likely_owner`（纯函数，测试锁定）。

---

## 5. API (stable)

见 `docs/PHASE4_NAVIGATION_LOG_API.md`。

```text
GET  /api/logs?level=&category=&event=&trace_id=&cycle_id=&since=&focus=&limit=&cursor=
GET  /api/logs/summary
GET  /api/logs/trace/<trace_id>
GET  /api/logs/cycle/<cycle_id>
GET  /api/logs/diagnostics?window_s=10
GET  /api/logs/api          # HTTP access only
POST /api/logs/config
```

`/api/nav/logs/...` 等价别名。默认 limit≤200。`/api/state` body **不**写入磁盘。

---

## 6. Test commands / results

```text
python scripts/_audit_phase4_navigation_stop_forensics.py   PASS
python scripts/_audit_phase4_commitment.py                  PASS (3C)
python scripts/_audit_phase4_probe.py                       PASS (3D)
python scripts/_audit_phase4_side_switch.py                 PASS (3E)
python scripts/_audit_phase4_recovery_execution.py          PASS (3F offline)
python scripts/_trace_phase4_recovery_corrective_live.py    PASS (3F LIVE)
python scripts/_trace_phase4_navigation_deadlock.py         PASS (API dump)
```

Forensics：

```text
requested=0.2 safe=0 → likely_owner=SAFETY
requested=0.2 safe=0.2 state=0 → EXECUTION
selected=NONE valid=0 → CANDIDATE
candidate_count=0 → LOCAL_PLANNING
vx≈0 w≠0 translation≈0 → SPIN_LOOP_SUSPECTED
```

LIVE 3F S1：

```text
LOCAL_REVERSE → REVERSE_ESCAPE
requested_vx=-0.120  safe_vx=-0.120  state_vx=-0.037
signed_back=0.129 m  spin_ticks=0
```

S2/S6 PASS。

---

## 7. Overhead / no behavior change

| Mode | logging_overhead |
|------|------------------|
| Audit ingest ×30 | **0.050 ms avg** |
| Live EMA after 3F | **0.11 ms** |

目标 &lt; 1 ms：**满足**。未因日志删字段。

Instrumentation 只在 `debug_hub.build` **之后**读取快照；不进入 `policy.step` / `compare` / `mppi.step`。3C–3F 与 P0-A 回归仍 PASS。

---

## 8. Q1–Q10

| Q | Answer |
|---|--------|
| **Q1 谁让它停？** | `summary.likely_owner` + `NAVIGATION_STOP_DIAGNOSTIC.data.likely_owner` |
| **Q2 全部 candidate / 拒绝原因？** | cycle `candidates.items[]`：`valid` / `primary_reason` / `reject_reasons` / `score_breakdown` |
| **Q3 requested→safe→state？** | `decision.command` + SAFETY_CLAMP |
| **Q4 为何 TURN/ALIGN？** | `FSM_TRANSITION` + `fsm.transition_reason` |
| **Q5 为何没有 reverse？** | recovery.action / probe.backward / selection_failure_reason |
| **Q6 Global Path 是否存在？** | `observe.global_path_exists` / `global.path_length_m` |
| **Q7 实际 horizon？** | `candidates.max_distance_m` vs `expected_nominal_distance_m`（≈0.25 m @ 1.5 s） |
| **Q8 候选越来越短？** | `PLANNING_STARVATION_SUSPECTED` + diagnostics window cycles |
| **Q9 spin loop？** | `SPIN_LOOP_SUSPECTED` / `summary.spin_suspected` |
| **Q10 前端只看 STOP/SAFETY/…？** | `GET /api/logs?focus=STOP\|SAFETY\|PLANNING\|RECOVERY` |

---

## 9. Known limitations

1. Probe/Local 仍用 P0-A 冻结的 0.45 采样（本阶段不改）。  
2. Global Preview / kinematic_valid = **null**（P0-B / P0-C）。  
3. `policy_vx` 若上层未单独暴露则为 null。  
4. left/right_near 当前 snapshot 可能为空。  
5. 本阶段 **无 UI**。  
6. DEBUG candidate 全量 path 默认关闭。

---

## 10. Completion checklist

```text
[x] Unified Nav Event Schema
[x] cycle_id / trace_id / event_id
[x] Navigation Decision Trace
[x] Candidate full summary + reject reasons + score breakdown
[x] FSM transition trace
[x] requested → safe → state
[x] Safety reason + execution pose delta
[x] STOP / DEADLOCK / SPIN / STARVATION / MISMATCH detectors
[x] performance + logging_overhead_ms
[x] API access log separated + filtering + since/cursor
[x] trace / summary / time-window diagnostics
[x] memory ring + optional JSONL
[x] API body redaction
[x] no logging-induced 3C–3F regression
[x] 3C/3D/3E/3F PASS
```

---

## Stop

```text
P0-B0 OBSERVABILITY = PASS
3F REGRESSION = PASS
LOGGING OVERHEAD = 0.11 ms
API READY FOR FUTURE DEBUG UI = YES
NEXT ALLOWED STEP = P0-B
```

**停止。不进入 P0-B，直至用户明确授权。**
