# AGV Navigation Phase 4 — STEP 1 只读审计报告

日期：2026-08-16  
范围：`V0.1仿真版` Web 仿真导航  
约束：**本阶段不修改任何代码**  
目标：回答「谁让 LEFT 变成 RIGHT？」并列出所有 side-switch 路径、Safety/Global 影响、可复现性与 telemetry 缺口。

---

## 0. 结论摘要（先读）

| 问题 | 结论 |
|------|------|
| **真正推动 LEFT→RIGHT 的权威** | 主路径是 **`LocalManeuverSelector._select`**，经 **`ManeuverFSM.decide`** 映射为 `LOCAL_RIGHT` |
| Policy 是否锁边？ | **否。** `avoid_side` / `MIN_AVOID_HOLD_S=1.0s` 只影响 Policy 的 `behavior` 标签；**不禁止** selector 换边 |
| 进入 `LOCAL_LEFT` 后是否每帧重投票？ | **实质是。** `mode∈{LOCAL_LEFT,LOCAL_RIGHT}` 时 `need_side_compare=True`，且 Policy 在 ACTIVE_LOCAL_* 时仍设 `allow_side_compare=True` |
| Hold / Hysteresis 够不够？ | **不够。** `MIN_HOLD_S=0.85s` 过后，只要 RIGHT `total_cost` 明显更低（`HYSTERESIS_ABS=6` 或 12%）即可 `LOWER_TOTAL_COST` 换边 |
| MPPI 会不会自己把 LEFT 改成 RIGHT？ | **不会（业务层）。** `LOCAL_LEFT` 走 force 短路，`w=abs(w)` 锁左；但 **mode 一旦被 FSM 改成 LOCAL_RIGHT**，MPPI 会立刻执行右向 force |
| Safety 会不会把 LEFT 改成 RIGHT？ | **不会直接改 mode。** 可能 `STOP_FRONT` 把 `vx→0`，造成卡住/贴障；随后 selector 重评分更容易切 RIGHT |
| Avoidance Commitment？ | **不存在** |
| Probe（F/B/L/R）？ | **不存在**（仅有 selector 1.5s rollout + sector_free 近似） |
| Side switch 因果日志？ | **PARTIAL**：有 `MANEUVER_SWITCH` / `BEHAVIOR_SWITCHED`，**无**完整 `authority` / obstacle_signature / safety_rejected 字段 |

**与用户现象的对齐假设（待 STEP 2 复现验证）：**

```text
LOCAL_LEFT（绕行中）
  → compare 仍每周期运行
  → LEFT rollout 因靠近障碍 / capture / deviation 变差（feasible↓ 或 cost↑）
  → RIGHT 因 SOFT_FREE / 瞬时 free-space / 成本优势变为 best
  → _select: LOWER_TOTAL_COST → RIGHT
  → FSM: LOCAL_RIGHT + force_w < 0
  → 车朝障碍侧转向 → 前向 clearance 崩塌 → Safety STOP_FRONT → 停死
```

---

## 1. 当前 LEFT/RIGHT 所有决策入口

### 1.1 权威链（实际）

```text
LocalMppiModel.step
  → NavigationPolicy.step(...)          # 建议 behavior / allow_side_compare
  → ManeuverFSM.decide(policy_ctx=...)  # 提交 mode
       若 allow_side / need_side_compare:
         → LocalManeuverSelector.compare → selected ∈ {LEFT,RIGHT,...}
         → mode = LOCAL_LEFT | LOCAL_RIGHT
  → DiffDriveMppi.step(maneuver_mode, force_vx/w)
  → apply_safety(phase=local_avoid|...)
  → accel → integrate
```

### 1.2 入口表

| # | 位置 | 作用 | 能否单独改边？ |
|---|------|------|----------------|
| E1 | `nav_policy.py` scene→`AVOID_LEFT/RIGHT` | 根据 `left_free/right_free` 或 `local_decision` 设 behavior | **否**（只写 telemetry/门控） |
| E2 | `nav_policy.py` `mm in LOCAL_LEFT/RIGHT` | 镜像 FSM mode，刷新 `avoid_hold` | **否** |
| E3 | `maneuver.py` `need_side_compare` + `local_selector.compare` | **主入口**：把 `sel` 写成 mode | **是（提交）** |
| E4 | `local_maneuver.py` `_select` | LEFT/RIGHT 成本比较 + hold/hysteresis | **是（提议）** |
| E5 | `local_maneuver.py` `SOFT_FREE_SPACE` | 两侧硬失败时按 free 复活一侧 | **是（可翻转）** |
| E6 | `maneuver.py` force_vx/w for LOCAL_* | 从 LEFT/RIGHT candidate 取 vx/w | 否（跟随 mode） |
| E7 | `mppi_controller.py` LOCAL_* short-circuit | `w` 符号锁定 | 否（跟随 mode） |
| E8 | `apply_safety` | 可能零速，不改 mode | 否 |

### 1.3 Policy 不锁边的关键代码

`nav_policy.py` 在 ACTIVE `LOCAL_LEFT` 时：

```text
allow_compare = True   # ← 仍授权侧向重比较
```

`maneuver.py`：

```text
self.mode in (LOCAL_LEFT, LOCAL_RIGHT)  ∈ need_side_compare 触发条件
```

因此：**绕行中并未停止投票。**

### 1.4 Policy `AVOID_LEFT` 是否约束 Selector？

**否。** `policy_ctx` 传入：

- `allow_side_compare`
- `require_capture_hard`
- `path_follow_weight`
- `max_deviation_m`

**没有** `committed_side` / `forbid_opposite_side`。  
Selector 不知道 Policy 想保持 LEFT。

---

## 2. 当前 LEFT→RIGHT 所有可能路径

### Path A — Selector 成本翻转（最高嫌疑）

```text
current=LEFT, hold_until 已过
LEFT.feasible 仍 True，但 cost 升高
  （贴障 → clearance_cost↑ / capture↑ / deviation↑ / 近距二次采样失败）
RIGHT.feasible True 且 cost 明显更低
→ _select: LOWER_TOTAL_COST → RIGHT
→ FSM LOCAL_RIGHT
authority = LocalManeuverSelector → ManeuverFSM
```

条件（`local_maneuver.py` `_select` / `_better`）：

- `now >= hold_until`（`MIN_HOLD_S=0.85`）
- `new_cost + max(6.0, |cur|*0.12) < cur_cost`

**不需要** LEFT 不可行；只要 RIGHT「好一点超过阈值」即可换边。  
**这正是「无充分理由换边」的主路径。**

### Path B — LEFT 变不可行，RIGHT 仅存

```text
LEFT rollout COLLISION / NO_PATH_CAPTURE / LOCAL_DEVIATION_EXCEEDED
RIGHT still feasible
→ RIGHT_ONLY
authority = LocalManeuverSelector
```

绕行中车体靠近障碍时 LEFT arc 易撞 → 合法换边的一种，但**缺少 Probe 再验证与 commitment 异常流程**。

### Path C — SOFT_FREE_SPACE 翻转

```text
两侧 hard-fail
瞬时 right_free > left_free + 0.45
→ 复活 RIGHT（cost≈85）
→ 选 RIGHT
authority = LocalManeuverSelector (soft fallback)
```

绕行姿态变化时 `sector_free` 噪声可导致左右 free 翻转。

### Path D — HOLD 过期后 FORWARD 插入再重选

```text
FORWARD 变 feasible (GOOD/WARNING)
若 obstacle_passed 或 LEFT 不再 hold
→ FORWARD 或后续再次 compare
→ 重新选边可能 RIGHT
```

### Path E — Policy PATH_RECAPTURE / corridor exceeded 打断

```text
LOCAL_LEFT 中 lateral_error 大 → corridor.exceeded
Policy → PATH_RECAPTURE
FSM 可能 POST_TURN / 退出 LOCAL_*
之后再次 OBSTACLE → 重新 compare → 可能 RIGHT
authority = NavigationPolicy (间接) + LocalManeuverSelector
```

### Path F — REPOSITION 后重入

```text
两侧失败 → REPOSITION
再进入 LOCAL_AVOID → 按瞬时 free 选边
authority = ManeuverFSM + Policy scene + Selector
```

### Path G — side_attempts 耗尽

```text
MAX_SIDE_ATTEMPTS=3 次 LEFT↔RIGHT 后
→ REPOSITION / REPLAN
不是直接 RIGHT，但会打断 LEFT commitment（且 commitment 本就不存在）
```

### Path H — MPPI 偷换边？

**排除为业务决策源。**  
仅当 FSM mode 已是 `LOCAL_RIGHT` 时执行右向。  
FOLLOW 模式下采样可有左右扰动，但 LOCAL_* 不走该路径。

### Path I — Safety 直接写 RIGHT？

**排除。** Safety 只改 `safe_vx/w`。

---

## 3. Safety 对 LEFT/RIGHT 的影响

文件：`sim_api_ext.apply_safety`

| 行为 | 影响 |
|------|------|
| `phase=local_avoid` 计入 turning | 允许 `|vx|<0.04` 时保留 `w` |
| `front_near < front_stop(0.70)` 且 `vx≥0` | **`vx→0`**，`w` 削弱；`STOP_FRONT` |
| collision + turning | 硬停 `(0,0)` |
| 不修改 `maneuver.mode` | mode 仍可能是 LOCAL_LEFT，但车不动 |

**危险耦合：**

```text
LOCAL_LEFT + 仍朝障碍有分量的运动
  → Safety 反复 zero vx
  → progress≈0，LEFT rollout 越来越差
  → Selector 换 RIGHT
  → 右向 force 朝障碍
  → 再撞 / 再 STOP → 停死
```

Safety **未能防止**「错误的 RIGHT 命令」：若 RIGHT 的 `vx` 在 `front_stop` 外短暂可行，会先执行再撞。  
**无**「LEFT 被 Safety 拒绝 N 次 → 反馈 Policy 换边」闭环（仅有粗 deadlock 标志）。

---

## 4. Global Path 对局部绕障的影响

| 机制 | 效果 |
|------|------|
| AVOID `path_follow_weight≈1.25` | 软化 MPPI 轨约束（LOCAL_* 本就不采样） |
| Selector `require_capture` / soft | AVOID 时硬 capture 放宽；仍有 capture_cost |
| `max_deviation_m` + lateral 惩罚 | 过大偏差可判 `LOCAL_DEVIATION_EXCEEDED` |
| `DEV_SOFT * gdev * path_follow_scale` | 绕开时仍惩罚偏离全局折线 |

**风险：** 绕左中 endpoint 离全局折线变远 → LEFT cost↑；RIGHT 若更「贴轨」则 cost↓ → **Path A 换边**，即使 RIGHT 几何更危险（若 clearance 尚未硬失败）。

---

## 5. Stuck / Recovery / Reverse 触发路径（与换边关系）

| 路径 | 与 LEFT→RIGHT |
|------|----------------|
| `pause_stuck=True` on LOCAL_* | 绕行中 stuck 时钟暂停 — 好 |
| Safety zero + stuck → Policy deadlock → REPLAN | 间接打断 LEFT |
| REVERSE 需 `allow_recovery` | 目标在后不默认 reverse（上轮保留） |
| 无 Backward Probe | Recovery 仍可能「觉得该退」但缺后方可行性事实 |

换边主因**不是** reverse；停死更像 **Safety zero + 错误 RIGHT**。

---

## 6. 当前可以复现什么

| 项 | 状态 |
|----|------|
| LIVE-A/B 选 LEFT/RIGHT | 已通过（静态布置） |
| **绕行中途 LEFT→RIGHT→撞** | **未在本 STEP 复现**（只读，未跑新脚本） |
| 确定性复现缺口 | 缺：committed 绕行中人为抬高 RIGHT score / 扰动障碍几何的场景脚本 |
| 日志缺口 | 无结构化 `side_switch_event`（authority/scores/safety） |

**为何可能不稳定复现：**

1. 依赖 rollout 数值噪声与 `sector_free`  
2. Hold 0.85s 后窗口短  
3. 障碍为圆形 dyn add，几何简单但姿态敏感  
4. 无 obstacle_signature，无法确认「同一障碍」

**建议 STEP 2 确定性复现：**

1. outdoor spawn → 前柱 + 右墙 → 确认 `LOCAL_LEFT`  
2. 保持障碍，注入 telemetry 监视 `local_maneuver.decision` / `maneuver.mode`  
3. 可选：临时在右侧减少阻挡或左侧加薄障（**仅测试场景**，非产品改参）  
4. 记录 `MANEUVER_SWITCH` 时间线与 `left_cost/right_cost`

---

## 7. 当前缺什么 telemetry

| 需求 | 现状 |
|------|------|
| `side_switch_event` 全字段 | **缺** |
| `authority` 枚举 | **缺** |
| `avoid_commit_active/side/elapsed` | **缺**（仅有 `avoid_side` + hold_until 弱信号） |
| `obstacle_signature` | **缺** |
| `probe_ctx` F/B/L/R | **缺** |
| `behavior_side` vs `trajectory_side` vs `actual_cmd_side` | **缺** |
| Safety reject 连续计数反馈到 Policy | **PARTIAL**（`_last_safety_blocked` 有，无 per-side） |
| UI：NAV PROBE / COMMIT / SWITCH LOCK | **缺** |
| 地图 corridor / probe 轨迹 | **缺**（上轮已标 NOT IMPLEMENTED） |

已有可用：

- `nav_policy.state/behavior/corridor/profile`
- `local_maneuver.rows/costs/events`（含 `MANEUVER_SWITCH`）
- `maneuver.mode/reason`
- `mppi_vx` / `safe_vx` / `stop_reason`

---

## 8. Hold / Hysteresis /「伪 Commitment」参数（现状）

| 参数 | 值 | 评价 |
|------|-----|------|
| `MIN_HOLD_S` | 0.85 s | 过短，绕障通常 >2–5 s |
| `HYSTERESIS_ABS` | 6.0 | 对 total_cost 量级偏松，易被瞬时 cost 波动打穿 |
| `HYSTERESIS_RATIO` | 0.12 | 同上 |
| `MIN_AVOID_HOLD_S` | 1.0 s | **只锁 Policy behavior，不锁 Selector** |
| `MAX_SIDE_ATTEMPTS` | 3 | 换边计数，非 commitment |
| `COMPARE_PERIOD_S` | 0.18 | 仍高频 |

---

## 9. 根因候选排序（供 STEP 3 验证）

1. **P0 — 无 Avoidance Commitment + 绕行中持续 `compare`**  
   → 允许「RIGHT 成本更好就切」  
2. **P0 — Hysteresis/Hold 过弱**  
   → 0.85s 后即可 `LOWER_TOTAL_COST`  
3. **P1 — Global deviation / capture 成本在 AVOID 下仍偏好「更贴轨」的一侧**  
4. **P1 — SOFT_FREE_SPACE / sector_free 噪声**  
5. **P1 — Safety 零速不反馈 → LEFT 评分恶化 → 切 RIGHT**  
6. **P2 — 无 Forward/Left/Right Probe 再验证即执行**

---

## 10. STEP 1 交付对照（用户清单）

| # | 要求 | 本报告 |
|---|------|--------|
| 1 | LEFT/RIGHT 所有决策入口 | §1 |
| 2 | LEFT→RIGHT 所有可能路径 | §2 |
| 3 | Safety 对 LEFT/RIGHT 影响 | §3 |
| 4 | Global Path 对局部绕障影响 | §4 |
| 5 | stuck/recovery/reverse 路径 | §5 |
| 6 | 当前可复现什么 | §6 |
| 7 | 缺什么 telemetry | §7 |

**代码未修改。**

---

## 11. 建议进入 STEP 2 时的最小动作

1. 单进程重启 `run_web_sim`（19999）  
2. 跑 LIVE-A 并持续 `GET /api/nav/debug` 抓 `mode`/`decision`/`left_cost`/`right_cost`/`stop_reason`  
3. 若未自然出现 LEFT→RIGHT：构造「LEFT committed 后 RIGHT score 人为更优」的确定性用例（先日志、后改逻辑）  
4. 产出时间线后再写 **STEP 3 Root Cause Report**（带 file/function/condition）

---

**下一步（待你确认）：STEP 2 Reproduce — 仍以观测为主；仅当需要确定性复现脚本时才允许新增只读审计脚本，不改导航决策逻辑。**
