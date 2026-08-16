# Local Maneuver Selection 报告

日期：2026-08-16  
范围：Web 仿真导航（`agv_bridge` + `delivery_web`）  
原则：短视界 LEFT/RIGHT 成本比较，扩展现有 ManeuverFSM；不引入第二套 FSM / Hybrid-A* / Nav2；不把 free-space 当作唯一选路依据。

---

## 1. 调用链（修复后）

```
Global A* path
  → ManeuverFSM.decide(...)
       front 近距 / need_side_compare
         → LocalManeuverSelector.compare(...)
              FORWARD / LEFT / RIGHT / ALIGN / REPOSITION / WAIT / REVERSE 候选
              footprint rollout（1.5s）+ hard/soft cost + hysteresis + min-hold
         → decision ∈ {LEFT, RIGHT, …} → mode LOCAL_LEFT / LOCAL_RIGHT / …
  → DiffDriveMppi：LOCAL_* 直接执行有向 (vx≥0, w 符号锁定)
  → Safety / SIL → accel → integrate
```

**STATE vs DECISION**：UI/telemetry 里 `decision=LEFT|RIGHT`，FSM `mode=LOCAL_LEFT|LOCAL_RIGHT`。

---

## 2. 候选与名义动作

| 候选 | vx | w | 说明 |
|------|----|---|------|
| FORWARD | 0.18 | 0 | 直穿可行性 |
| LEFT / RIGHT | 0.22 | ±0.35 | 主采样 |
| LEFT/RIGHT 近距二次 | 0.10 | ±0.42 | `front_near<1.35` 且主采样不可行时 |
| ALIGN / REPOSITION / WAIT / REVERSE | 合成 | — | 两侧硬失败后的回退 |

Horizon：`ROLLOUT_STEPS=15` × `dt=0.1` → **1.5s**。碰撞用车体采样点 + `world.collides`，不是仅中心点。

---

## 3. 成本（对齐 MPPI 量级）

**Hard（≈不可行）**

| 条件 | cost / reason |
|------|----------------|
| 碰撞 | ≈700 / `COLLISION` |
| 净空 < 0.18 m | hard |
| 出界 | `OUT_OF_MAP` |
| 侧向后 capture 距离 > 4.5 m | `NO_PATH_CAPTURE` |

**Soft**：clearance / capture / heading / progress / turn(|w|) / path deviation / switch；尺度贴近 `5·gdev`、`2.4·|w|`。

**Free-space**：只做 soft 偏置（±侧净空差）；**不是**单独选路器。近距两侧主采样都撞时，若一侧净空明显更大（差 >0.45 m 且 >1.0 m），可标 `SOFT_FREE_SPACE`（WARNING）再参与比较——仍须经 hysteresis / Safety。

---

## 4. 滞回与保持

| 参数 | 值 |
|------|-----|
| `HYSTERESIS_RATIO` | 0.12 |
| `HYSTERESIS_ABS` | 6.0 |
| `MIN_HOLD_S` | 0.85 |
| `MAX_SIDE_ATTEMPTS` | 3 |
| `COMPARE_PERIOD_S` | 0.18 |

侧向期间 `pause_stuck=True`；`obstacle_passed` 后优先回 `FORWARD`。

---

## 5. Debug / UI

- `+组件`：`LOCAL MANEUVER` / `MANEUVER COMPARISON` / `MANEUVER COST`（`dbg_local_man` / `dbg_man_cmp` / `dbg_man_cost`）
- Telem：`local_decision`、`fwd_cost` / `left_cost` / `right_cost`、candidate rows、paths、events
- 缓存：`nav_ui.js?v=20260816m2`

---

## 6. 实况时间线（outdoor_campus，spawn ≈(-35,0,0)）

| Case | 布置 | 结果 |
|------|------|------|
| LIVE-A | 前柱 + 右侧墙 | `decision=LEFT`，`mode=LOCAL_LEFT`，max_rev=0 |
| LIVE-B | 前柱 + 左侧墙 | `decision=RIGHT`，`mode=LOCAL_RIGHT`，max_rev=0 |
| LIVE-C | 前向笼障 | 无无限 reverse；见 REPOSITION/LEFT/FORWARD |
| LIVE-TEL | debug keys | PASS |

**先前失败根因**：近距柱（≈0.95 m）+ 膨胀足迹下，主弧线（0.22 m/s × ±0.35 rad/s）在 1.5 s 内仍撞柱 → `left=right=700` → `SIDES_BLOCKED→REPOSITION`。  
**修复**：近距激进二次采样 + 净空 soft fallback。

---

## 7. 验收

| 套件 | 结果 |
|------|------|
| `_audit_local_maneuver.py` T1–T25 + Scenario C | **27/27 PASS** |
| `_audit_maneuver_cases.py` M1–M18 | **33/33 PASS** |
| `_audit_local_maneuver_live.py` A/B/C/TEL | **LIVE RESULT PASS** |

启动：先清 19999 等端口，再 `PYTHONPATH=.../agv_bridge` + `python scripts/run_web_sim.py`（单进程，避免旧代码占端口）。

---

## 8. 未做 / 开放问题

- LIVE-D（动态 WAIT）、LIVE-E（死胡同 reverse 后再评估）未单独脚本化；逻辑由 WAIT/REVERSE 候选与 M 套覆盖一部分。
- Soft free-space 仅在两侧硬失败时启用；极端“两侧都假净空”仍可能选错侧——依赖 Safety 与 `MAX_SIDE_ATTEMPTS`。
- 未改全局 A*、未绕过 Safety、未缩小 footprint、未恢复 stuck→无限 reverse。

---

## 关键文件

| 文件 | 作用 |
|------|------|
| `agv_bridge/local_maneuver.py` | 候选 rollout / 成本 / 滞回 / soft fallback |
| `agv_bridge/maneuver.py` | `LOCAL_LEFT`/`LOCAL_RIGHT` + `local_selector` |
| `agv_bridge/mppi_controller.py` | 侧向有向命令 |
| `agv_bridge/nav_debug.py` / `sim_api_ext.py` | local_maneuver telemetry |
| `delivery_web/www/nav_ui.js` | 三张对比卡 |
| `scripts/_audit_local_maneuver.py` | T1–T25 |
| `scripts/_audit_local_maneuver_live.py` | Live A/B/C |
| `scripts/_audit_maneuver_cases.py` | M1–M18 回归 |
