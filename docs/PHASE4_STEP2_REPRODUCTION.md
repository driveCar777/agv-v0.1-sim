# PHASE 4 STEP 2 — SIDE SWITCH REPRODUCTION

日期：2026-08-16  
约束：**只复现 / 只观测 / 只取证**（未改 Policy / FSM / Selector / MPPI / Safety 决策逻辑）  
方法修正（用户指出）：**先空场导航，再中途投放障碍**；禁止「先放障再规划」导致全局绕开。

主证据文件：

- `docs/_phase4_trace/mid_inject_1786888402.jsonl` — **LIVE REPRODUCTION（硬 Stage2，抓到 LEFT→RIGHT）**
- `docs/_phase4_trace/mid_inject_soft_1786888471.jsonl` — LIVE soft Stage2（**未切边**，对照）
- `docs/_phase4_trace/left_wide_long_1786887908.jsonl` — 旧方法（先放障）对照，无切边
- `scripts/_trace_phase4_side_switch.py` — 只读采集器（`mid_inject` / `mid_inject_soft`）

```text
============================================
PHASE 4 STEP 2 — SIDE SWITCH REPRODUCTION
============================================
```

---

## 1. Environment

| 项 | 值 |
|----|-----|
| Host | Windows |
| URL | `http://127.0.0.1:19999/` |
| Engine | `sim_true_scene` / `smap_occupancy+dual_lidar+threejs` |
| Backend | `robokit_mock_3055` |
| Scene | `outdoor_campus` |
| Tracer | 20Hz poll `/api/nav/debug` + `/api/state` |

## 2. Scenario

### 正确方法（本 STEP 主跑）

```text
clear obstacles
  → plan/confirm 直线目标 (~6.5m ahead)   # 空场导航
  → FORWARD_TRACK / FOLLOW_GLOBAL
  → t≈1.3s INJECT Stage1（前柱偏右 + 右侧三重封死）→ 逼 LOCAL_LEFT
  → LOCAL_LEFT 持续 ≥0.95s（越过 selector MIN_HOLD_S=0.85s）
  → INJECT Stage2（左通道硬堵塞 + 右通道部分打开）
  → LEFT → RIGHT
  → 继续观测 至 REPOSITION / 停转
```

Stage1/2 几何故意 **比用户原场景更刁钻**（多右封 + 左通道多球堵塞）。

### 旧错误方法（对照）

先放柱/右墙再 `plan` → 规划器一开始就绕，常直接 `LOCAL_LEFT→POST_TURN`，**抓不到中途切边**。

## 3. Simulation PID / Port

| 项 | 值 |
|----|-----|
| Port | **19999 LISTEN** |
| PID | **27264** (`python`) |
| StartTime | 2026-08-16 21:16:12 |
| Web | alive（全程 CONNECTED） |

## 4. Reproduction Result

| 类型 | 结果 |
|------|------|
| **LIVE REPRODUCTION** `mid_inject` | **SUCCESS：LEFT → RIGHT** @ t=3.331s，reason=`RIGHT_ONLY` |
| LIVE `mid_inject_soft` | NO SWITCH（两侧后来都可行时被 `HYSTERESIS_KEEP` / `HOLD_SIDE_UNTIL_PASSED` 锁住） |
| LIVE 旧 `left_wide*` | NO SWITCH（先放障） |
| DETERMINISTIC SYNTHETIC | 另有 offline selector 可切边，见附录；**不得与 LIVE 混写** |

**本报告主结论以 LIVE `mid_inject_1786888402.jsonl` 为准。**

## 5. Exact Timeline

相对 `t0`（trace 内 `t`）：

```text
T+0.000     clear + plan/confirm goal=(-28.5,0) from pose=(-35,0)
T+1.312     INJECT Stage1（前柱+右三重封）; mode=FORWARD_TRACK; front≈18.6 → 突变
T+1.531     FRONT_OBSTACLE; front=0.386; FOLLOW_GLOBAL→即将避障
T+1.825     LOCAL_LEFT | LEFT_ONLY | L=5.0 R=700 | Lf=T Rf=F | allow_side_compare=True
            mppi_w=+0.084 (LEFT) | safe_vx=0
T+2.144…2.658  HOLD_MIN_TIME（仍持续 compare）L≈5.2→5.5 R=700 allow=True
T+2.857     INJECT Stage2（清障重建：柱左移 + 左通道 5 球堵塞 + 右残留）
T+3.155     LAST LEFT: HYSTERESIS_KEEP L=5.7 R=700 Lf=T Rf=F | w=+0.084 | allow=True
T+3.331     ★ FIRST RIGHT: LOCAL_RIGHT | RIGHT_ONLY | L=700 R=75 | Lf=F Rf=T
            mppi_w=-0.084 (符号翻转) | policy_behavior 仍短暂 AVOID_LEFT | allow=True
T+3.704     FRONT_OBSTACLE 再现; mppi_w=-0.42; safe_vx=0
T+4.592…13.9 LOCAL_RIGHT + HYSTERESIS_KEEP; state_vx≈0; yaw 由 +0.11 → -0.64（右转）
T+14.169    REPOSITION | SIDES_BLOCKED→REPOSITION
T+15.8…21   REPOSITION 失败循环（HEADING_UNACHIEVABLE / ALIGN_UNSAFE）
```

### 关键时间点

| 标记 | t (s) |
|------|-------|
| LEFT_DECISION_TIME | **1.825** |
| Stage2 inject | **2.857** |
| RIGHT_DECISION_TIME | **3.331** |
| delta_t (LEFT→RIGHT) | **1.506 s** |
| delta_t (Stage2→RIGHT) | **0.474 s** |

## 6. LEFT Decision

```text
t=1.825
class/function: LocalManeuverSelector._select → reason LEFT_ONLY
               ManeuverFSM.decide → mode LOCAL_LEFT
condition: left_ok=True, right_ok=False (R cost=700 / infeasible)
L=5.0  R=700  Lf=True Rf=False
allow_side_compare=True
mppi_w=+0.084  safe_vx=0  front=0.386
```

## 7. RIGHT Decision

```text
t=3.331
class/function: LocalManeuverSelector._select → reason RIGHT_ONLY
               ManeuverFSM.decide → mode LOCAL_RIGHT
condition: left_ok=False, right_ok=True（Stage2 后 LEFT rollout 硬碰撞）
L=700  R=75  Lf=False Rf=True
allow_side_compare=True
mppi_w=-0.084（命令侧已翻到 RIGHT）
policy_behavior 仍 AVOID_LEFT 一帧（Policy 标签滞后于 FSM）
```

## 8. Score Comparison

### 切边前最后一帧 LEFT（t=3.155）

| side | feasible | total_cost | clr | capture |
|------|----------|------------|-----|---------|
| LEFT | True | 5.7 | 3.5 | 0.20 |
| RIGHT | False | 700 | — | — |

### 切边第一帧 RIGHT（t=3.331）

| side | feasible | total_cost | note |
|------|----------|------------|------|
| LEFT | **False** | 700 | Stage2 左堵塞 → 碰撞硬失败 |
| RIGHT | True | 75 | SOFT/仅存可行侧（telemetry 未拆满分量） |

**最小原因集合（本 LIVE 次）：**

```text
CONFIRMED: LEFT 变为 infeasible (collision cost 路径 → 700)
CONFIRMED: RIGHT 成为唯一可行侧 → _select 返回 RIGHT_ONLY
NOT CONFIRMED in this LIVE run: 双方均可行时 LOWER_TOTAL_COST 微差翻边
```

分量级 clearance/capture/progress 在硬失败帧上被碰撞短路，API 行内多为 `None`（见 Remaining Unknowns）。

## 9. Policy State Timeline

```text
FOLLOW_GLOBAL / PATH_RECAPTURE  →  LOCAL_AVOID(AVOID_LEFT)
  →  LOCAL_AVOID(AVOID_RIGHT)   →  REPOSITION ...
```

切边瞬间：`maneuver_mode=LOCAL_RIGHT` 而 `policy_behavior=AVOID_LEFT`（短暂不一致）——Policy **不拥有**边权威。

## 10–12. allow_side_compare / need_side_compare / avoid_hold

| t | avoid_side | allow_side_compare | need_side_compare | selector |
|---|------------|--------------------|-------------------|----------|
| 1.825 | LEFT | **True** | NOT_AVAILABLE（API）/ 由代码：mode∈LOCAL_* ⇒ 会比 | LEFT |
| 2.3 | LEFT | **True** | （同上） | HOLD_MIN_TIME |
| 3.155 | LEFT | **True** | （同上） | HYSTERESIS_KEEP |
| 3.331 | LEFT→ | **True** | （同上） | **RIGHT** |
| 4.7+ | RIGHT | **True** | （同上） | HYSTERESIS_KEEP |

```text
CONFIRMED: LOCAL_LEFT 全程 allow_side_compare=True
CONFIRMED: Policy avoid_hold / avoid_side 未阻止 selector 换边
LIKELY: need_side_compare 在 LOCAL_LEFT 期间持续为 True（代码路径 + 持续出现新 reason）
NOT_AVAILABLE: 显式 avoid_hold_active / need_side_compare 字段（telemetry 缺口）
```

## 13. Obstacle Geometry Timeline

| 阶段 | 几何 | 拓扑含义 |
|------|------|----------|
| pre-inject | 空 | 直线跟随 |
| Stage1 | 前柱偏右 + 右×3 封 | RIGHT 不可行 → LEFT_ONLY |
| Stage2 | 柱左移 + 左×5 堵 + 右残留 | LEFT 不可行 → RIGHT_ONLY |

本 LIVE 切边属于：

```text
正常拓扑切边（LEFT 真失效）+ 异常门控缺失（无 Commitment，仍允许重投票）
```

不是「LEFT 仍可行、RIGHT 只便宜一点」的异常成本翻边（该型在 soft 对照未出现）。

## 14. Safety Timeline

```text
T+1.531  FRONT_OBSTACLE（Stage1 后贴障）
T+1.825…3.3  多数 NONE；safe_vx 常 0（已贴障）
T+3.704  FRONT_OBSTACLE 与 LOCAL_RIGHT 同现（右转命令 mppi_w=-0.42）
之后     FRONT_OBSTACLE 间歇闪烁；safe_vx 持续 ≈0
```

**Safety 不改 mode**；它把 `vx` 置零。切边由 Selector/FSM 完成。

## 15. Actual Motion Timeline

| 阶段 | policy side | fsm | selector | mppi_w | state_w | 一致？ |
|------|-------------|-----|----------|--------|---------|--------|
| LOCAL_LEFT | LEFT | LEFT | LEFT | **+** | **+** | YES |
| 切边帧 | LEFT(滞后) | RIGHT | RIGHT | **−** | +→− | FSM/命令一致，Policy 标签滞后 |
| LOCAL_RIGHT | RIGHT | RIGHT | RIGHT | **−** | **−** | YES |

`state_vx≈0` 全程贴障；**转向仍发生**（yaw +0.11→−0.64），说明切边后命令推动了朝向障碍侧的旋转。

## 16. Stuck / Deadlock Timeline

```text
LOCAL_RIGHT + safe_vx=0 + FRONT_OBSTACLE 闪烁（~10s）
  → REPOSITION
  → HEADING_UNACHIEVABLE / ALIGN_UNSAFE→REPOSITION
  → 未能脱离（观测窗口结束时仍困）
```

未见到明确 `SAFE_STOP` 作为终态标签（过程中有 `FAILED` stop_reason 快照）；实质是 **贴障零速 + 失败重定位循环**。

## 17. Root Cause Evidence

| 证据 | 等级 |
|------|------|
| 19999 真实连接并采到 LEFT→RIGHT | **CONFIRMED** |
| 换边权威 = `LocalManeuverSelector._select` → `ManeuverFSM` | **CONFIRMED** |
| 换边条件本 run = `RIGHT_ONLY`（LEFT infeasible） | **CONFIRMED** |
| `allow_side_compare=True` 贯穿 LOCAL_LEFT | **CONFIRMED** |
| Policy `avoid_hold` / `avoid_side` 不锁 selector | **CONFIRMED** |
| 切边后 `mppi_w` 符号翻转并跟随 FSM | **CONFIRMED** |
| Safety `FRONT_OBSTACLE` 在切边之后加重 / 维持零速 | **CONFIRMED（后果）** |
| 无 Avoidance Commitment / Probe | **CONFIRMED（代码+行为）** |
| 「双方可行 + 微差 LOWER_TOTAL_COST」导致用户原现象 | **NOT CONFIRMED on LIVE**（soft 未翻） |
| 先放障再导航会掩盖中途切边 | **CONFIRMED**（旧 trace 无 switch） |

## 18. Competing Hypotheses

| ID | 假设 | 本 STEP |
|----|------|---------|
| H1 | 无 Commitment → 绕行中持续重投票 → 可切边 | **CONFIRMED 机制**；本 LIVE 触发为硬失效 |
| H2 | 双方可行微差 `LOWER_TOTAL_COST` | **POSSIBLE**（代码路径存在）；LIVE soft **未触发** |
| H3 | Safety 直接把 LEFT 改成 RIGHT | **RULED OUT** |
| H4 | MPPI 业务层自选边 | **RULED OUT** |
| H5 | 全局规划在切边帧改边 | **RULED OUT**（局部 mode 切换） |

## 19. Ruled-Out Hypotheses

- Safety 是 mode 写入者 → **否**
- MPPI 独立 LEFT/RIGHT 业务择边 → **否**
- `avoid_hold≈1s` 等于边锁定 → **否**（只影响 behavior 标签）

## 20. Remaining Unknowns

1. API 未暴露 `need_side_compare` / `avoid_hold_active` 布尔值（只能由 behavior+代码推断）。
2. 硬碰撞帧缺少完整 cost 分量拆解（clearance/capture/… 被 700 短路）。
3. 用户原「轻微更优就翻边」路径：**LIVE soft 未复现**；需 STEP3 设计 Commitment 时仍按 H2 防护。
4. `stuck_s` 在 RIGHT 段多为 `null`，卡死更多表现为零速+REPOSITION 失败而非清晰 stuck counter。

---

## Q1 — 是谁让 LEFT 变成 RIGHT？

```text
LocalManeuverSelector._select
  条件: left_ok=False and right_ok=True
  返回: selected=RIGHT, reason=RIGHT_ONLY
→ ManeuverFSM.decide 写入 mode=LOCAL_RIGHT
→ DiffDriveMppi LOCAL_RIGHT force（w 取负）
```

**CONFIRMED（本 LIVE）。**

## Q2 — 为什么当时允许重新比较？

```text
NavigationPolicy: mm in {LOCAL_LEFT,LOCAL_RIGHT} ⇒ allow_side_compare=True
ManeuverFSM: mode in LOCAL_LEFT/RIGHT ⇒ need_side_compare 条件满足
avoid_hold / avoid_side: 只刷新 behavior 标签，不传入 selector 禁止对侧
⇒ 无 Avoidance Commitment
```

**CONFIRMED。**

## Q3 — RIGHT 为什么赢？

本 LIVE：

```text
LEFT.feasible=False (total_cost=700)
RIGHT.feasible=True  (total_cost=75)
⇒ RIGHT_ONLY
```

不是「总分略低」；是 **LEFT 失效后的唯一可行侧**。

## Q4 — LEFT 当时是真的失效了吗？

```text
YES
```

证据：切边帧 `left_feasible=False`，`left_cost=700`；Stage2 左通道多障碍与柱左移同时存在。  
（对照 soft：LEFT 长期 `feasible=True`，**未切边**。）

## Q5 — Safety 是根因还是后果？

```text
CONSEQUENCE（对 mode）
SECONDARY CAUSE（对卡死：safe_vx=0 维持贴障）
```

切边发生时 `stop_reason=NONE`；`FRONT_OBSTACLE` 在 RIGHT 命令之后更频繁。

## Q6 — 卡死为什么发生？

```text
LEFT → RIGHT（朝更堵/更近几何）
  → mppi 右向 w，front 维持 ~0.35–0.45
  → Safety FRONT_OBSTACLE ⇒ safe_vx=0
  → 位置几乎不动，仅 yaw 右转
  → 两侧持续恶化 / SIDES_BLOCKED → REPOSITION
  → REPOSITION 失败（HEADING_UNACHIEVABLE / ALIGN_UNSAFE）
  → 无法脱出
```

---

## STEP 2 完成清单

- [x] 19999 已真实连接（PID 27264）
- [x] 尝试复现（**中途投放**，非先放障）
- [x] LEFT 选择时间点
- [x] RIGHT 选择时间点
- [x] 完整时间线
- [x] selector score（L/R total + feasible）
- [x] `allow_side_compare`
- [x] `need_side_compare`（代码推断；API=NOT_AVAILABLE）
- [x] `avoid_hold`（无显式字段；行为证明未锁边）
- [x] obstacle geometry 状态
- [x] Safety 状态
- [x] actual motion（w 符号 / yaw）
- [x] stuck/deadlock（REPOSITION 失败环）
- [x] 证据等级
- [x] 新建本报告
- [x] **未修改**导航决策逻辑

---

## 附录 A — 方法教训

用户指出正确：先导航再放障。旧 `left_wide` 系列因规划器预绕，**系统性低估切边概率**。本 STEP 以 `mid_inject` 纠正。

## 附录 B — soft 对照要点

`mid_inject_soft`：Stage2 后后期出现 `Lf=True Rf=True` 且 L≈20 / R≈20–21，但 selector 输出 `HYSTERESIS_KEEP` / `HOLD_SIDE_UNTIL_PASSED`，**无 LEFT→RIGHT**。  
说明：当前 hysteresis 对「微差」有一定抑制；**硬失效 + 无 Commitment** 仍足以切边并卡死。

## 附录 C — 下一步（STEP 3，勿在本 STEP 实施）

根因设计方向（仅记录，**未实现**）：

1. Avoidance Commitment（LOCAL_LEFT 期间禁止对侧除非 LEFT 硬失效+Probe）
2. Probe（F/B/L/R）验证通道
3. side_switch 因果 telemetry（含 need_side_compare / avoid_hold_active / cost 分量）
