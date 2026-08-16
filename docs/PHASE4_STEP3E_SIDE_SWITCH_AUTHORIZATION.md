# PHASE 4 STEP 3E — Side Switch Authorization

日期：2026-08-16  
状态：**PASS**  
`BEHAVIOR CHANGED: **YES** (intended — Policy-owned LEFT↔RIGHT gate)`  
`3F PREREQUISITES: **PASS** (auth live; Recovery/Replan **not** started)`

---

## 0. Verdict

| Check | Result |
|-------|--------|
| Scene A: LEFT fail + RIGHT/TURN VALID → AUTHORIZED → `LOCAL_RIGHT` | **PASS** |
| Scene B: hard mid_inject (RIGHT Probe INVALID) → no RIGHT / auth false | **PASS** |
| Scene C: soft / current still VALID → DENY / no switch | **PASS** |
| Probe INVALID never authorizes that side | **PASS** |
| Current side still VALID → no soft switch | **PASS** |
| Auth latency (offline) | **~0.025 ms** (O(1) gates, no new rollouts) |
| 3C / 3D regression | **PASS** |
| Auto reverse / Recovery (3F) | **NOT STARTED** |

---

## 1. Scope

实现 **Policy-owned `side_switch_authorized` 门控**：

```text
ProbeBundle (3D evidence)
    ↓
NavigationPolicy.authorize_side_switch_gate  → token (TTL)
    ↓
Selector: POLICY_AUTHORIZED_SWITCH | COMMIT_KEEP | COMMIT_REVALIDATION
    ↓
ManeuverFSM: execute LOCAL_* / AUTH_GATE_BLOCK
    ↓
note_side_switch_executed → consume token + retarget commitment
```

**禁止**：为 Scene A “造 PASS” 放宽 Probe；在 authorize 内新增 rollout；自动 reverse / Recovery（3F）。

---

## 2. Files Changed

### ADDED

| File | Why |
|------|-----|
| `agv_bridge/nav_side_switch.py` | Gates / token / `authorize_side_switch` / `validate_token_for_execution` |
| `scripts/_audit_phase4_side_switch.py` | Offline E-series + latency |
| `scripts/_trace_phase4_side_switch_auth_live.py` | LIVE Scene A/B/C |
| `docs/PHASE4_STEP3E_SIDE_SWITCH_AUTHORIZATION.md` | 本报告 |

### MODIFIED

| File | Why | Decision impact |
|------|-----|-----------------|
| `nav_policy.py` | `authorize_side_switch_gate` / `note_side_switch_executed` / events | **有** |
| `nav_models.py` | Probe → Policy.step → **authorize** → decide → consume | **有** |
| `local_maneuver.py` | `POLICY_AUTHORIZED_SWITCH` bypass min-hold | **有** |
| `maneuver.py` | `authorized_side` + `AUTH_GATE_BLOCK_*` | **有** |
| `nav_phase4_telemetry.py` | schema `phase4_step3e_v1`；真实 side_switch | 观测 |
| `nav_ui.js` | SIDE SWITCH 卡：auth / reason / gates / probes | 观测 |

---

## 3. Architecture (3E)

```text
LocalMppiModel.step
  → ProbeEngine.evaluate
  → NavigationPolicy.step          # commitment / allow_side_compare
  → authorize_side_switch_gate     # O(1) over ProbeBundle
  → ManeuverFSM.decide(policy_ctx{authorized_side, side_switch_authorized})
  → if FSM LEFT↔RIGHT & live token: note_side_switch_executed
  → update_commitment_after_selection
```

| Owner | Role |
|-------|------|
| **Policy** | **唯一**换边授权权威 |
| Selector | 候选 / `POLICY_AUTHORIZED_SWITCH` 执行口 |
| FSM | 模式执行 + 无 token 时 `AUTH_GATE_BLOCK` |
| Probe | 证据 only（INVALID ⇒ 不可授权该侧） |
| Safety | 硬覆盖（`safety_zero` / emergency → DENY） |

---

## 4. Gates（全部为真才 AUTHORIZED）

| Gate | DENY reason |
|------|-------------|
| commitment.active + side∈{L,R} | `NO_COMMITMENT` |
| current hard fail (Probe INVALID / hard_fail) | `CURRENT_SIDE_STILL_VALID` / HARD_* |
| alternative Probe **VALID only** | `ALTERNATIVE_PROBE_INVALID` / UNKNOWN / STALE |
| TURN_IN_PLACE VALID | `TURN_NOT_SAFE` |
| probe fresh | `STALE_PROBE` |
| safety / not emergency | `SAFETY_DENIED` / `EMERGENCY` |
| not dynamic_short | `DYNAMIC_WAIT_REQUIRED` |
| path_valid | `PATH_INVALID` |
| cooldown ≥ 2.0 s | `SIDE_SWITCH_COOLDOWN` |
| oscillation < 3 / 8 s | `SIDE_SWITCH_OSCILLATION` |
| scene / signature | `SCENE_CHANGED` / mismatch |

Soft：两侧 VALID 且当前未硬失败 → **一律 DENY**（`CURRENT_SIDE_STILL_VALID`），即使 Selector 成本更低。

Token：`SideSwitchAuthorizationToken`，TTL **0.40 s**，one-shot consume。

---

## 5. Q1–Q20（钉死）

| # | Question | Answer |
|---|----------|--------|
| Q1 | Who authorizes side switch? | **NavigationPolicy** only |
| Q2 | Who may propose candidate? | Selector (raw preference) |
| Q3 | Who executes mode? | ManeuverFSM |
| Q4 | Can Selector switch without auth under commit? | **No** (`COMMIT_*`) |
| Q5 | Probe INVALID on alt? | **DENY** — never authorize that side |
| Q6 | Current still VALID? | **DENY** even if other cheaper |
| Q7 | Hard fail of current? | Necessary but not sufficient |
| Q8 | Turn required? | **Yes** — TURN Probe VALID |
| Q9 | Extra rollouts in authorize? | **No** — consume ProbeBundle |
| Q10 | Token lifetime? | 0.40 s, one-shot |
| Q11 | Cooldown? | 2.0 s after executed switch |
| Q12 | Oscillation? | max 3 switches / 8 s |
| Q13 | Safety role? | Hard DENY / override; not business chooser |
| Q14 | First side select need auth? | **No** — only LEFT↔RIGHT under commitment |
| Q15 | After auth execute? | Consume token + retarget commitment |
| Q16 | Hard mid_inject (STEP2 style)? | Stay LEFT / REVALIDATION；RIGHT Probe INVALID → DENY |
| Q17 | Soft mid_inject? | DENY `CURRENT_SIDE_STILL_VALID` |
| Q18 | Full `/obstacles/clear` mid-avoid? | **Avoid** — can flash FORWARD → release commitment (test uses surgical remove) |
| Q19 | Auto reverse? | **No** (3F) |
| Q20 | 3F ready? | **Yes** — auth gate + Probe evidence online |

---

## 6. LIVE Evidence (19999)

| Scene | Trace | Expect | Result |
|-------|-------|--------|--------|
| **A** | `docs/_phase4_trace/step3e_scene_a_1786892173.jsonl` | LEFT commit → surgical S2 → AUTHORIZED → `LOCAL_RIGHT` | **PASS** |
| **B** | `docs/_phase4_trace/step3e_scene_b_1786892228.jsonl` | hard mid_inject；RIGHT=0；auth=0 | **PASS** (`deny_alt_probe_ticks=110`) |
| **C** | `docs/_phase4_trace/step3e_scene_c_1786892253.jsonl` | soft；no RIGHT；auth=0 | **PASS** (`still_valid_ticks=84`) |

### Scene A causal slice

```text
T+1.32  S1 force_LEFT (front pillar + RIGHT seals)
T+1.71  LOCAL_LEFT + commitment LEFT
T+2.18  S2 surgical: remove seals, pillar nudge=0.75m LEFT, far left clutter
T+2.62  selector=POLICY_AUTHORIZED_SWITCH  Probe L=INVALID R=VALID T=VALID
        → LOCAL_RIGHT ; events SIDE_SWITCH_EXECUTED / COMMITMENT_SWITCHED
```

**几何原则**：不放宽 Probe；调 Stage2 障碍使 RIGHT/TURN 真正 VALID。  
**禁止** `obstacles/clear` 作为 Stage2（会短暂 FORWARD → `RETURN_FORWARD` 释放 commitment，造成无授权换边假象）。

---

## 7. Regression

| Suite | Result |
|-------|--------|
| `_audit_phase4_side_switch.py` | **PASS** (~0.025 ms) |
| `_audit_phase4_commitment.py` | **PASS** |
| `_audit_phase4_probe.py` | **PASS** |

未覆盖写 STEP2/3B/3C/3D 主证据文件。

---

## 8. Telemetry / UI

- `side_switch.implemented=true`
- 字段：`authorized` / `status` / `reason` / `gates` / `*_probe_status` / `token` / `authorization_ms`
- 事件：`SIDE_SWITCH_AUTHORIZED|DENIED|EXECUTED|EXECUTION_FAILED|AUTH_EXPIRED`
- UI：**SIDE SWITCH** 卡显示 auth / reason / gates / probes

注：授权与 FSM 同 tick 消费 token 后，下一采样可能 `authorized=false` 但 `selector_reason=POLICY_AUTHORIZED_SWITCH` / 事件仍可证明授权执行。

---

## 9. 3F Prerequisites

| Prerequisite | Status |
|--------------|--------|
| Commitment lifecycle (3C) | PASS |
| Probe evidence (3D) | PASS |
| Side-switch Policy gate (3E) | **PASS** |
| Unauthorized mid-avoid flip blocked | PASS (Scene B/C) |
| Authorized escape when Probe allows | PASS (Scene A) |
| Recovery / Replan / auto-reverse | **NOT IN 3E** |

---

## 10. Explicit Non-Goals (this step)

- 不启动 **STEP 3F Recovery / Replan**
- 不自动 reverse
- 不改 MPPI / Safety 择边业务
- 不放宽 Probe 阈值以换 Scene A PASS

---

## 11. STATUS

```text
STEP 3E STATUS = PASS
BEHAVIOR CHANGED = YES (Policy side-switch authorization)
3F PREREQUISITES = PASS
NEXT = STEP 3F only when explicitly ordered
```
