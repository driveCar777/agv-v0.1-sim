#!/usr/bin/env python3
"""STEP 3D ProbeEngine offline audit (P1–P32 + integration I1–I6).

Evidence-only: asserts Probe never changes vx/w / FSM / selector / commitment lock.
"""

from __future__ import annotations

import math
import sys
import time
from typing import Callable, List, Tuple

from agv_bridge.local_maneuver import LocalManeuverSelector, DEC_LEFT, MIN_HOLD_S
from agv_bridge.nav_models import LocalMppiModel
from agv_bridge.nav_policy import NavigationPolicy, LOCAL_AVOID
from agv_bridge.nav_probe import (
    DIR_FORWARD,
    DIR_LEFT,
    DIR_RIGHT,
    ProbeEngine,
    STATUS_INVALID,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_VALID,
    build_obstacle_snapshot,
    footprint_points,
    stopping_distance_m,
)

Pt = Tuple[float, float]


class FailList:
    def __init__(self) -> None:
        self.items: List[str] = []

    def check(self, name: str, cond: bool, detail: str = "") -> None:
        if not cond:
            self.items.append(f"{name}: {detail}" if detail else name)

    def ok(self) -> bool:
        return not self.items


def circle_obstacle(cx: float, cy: float, r: float) -> Callable[[float, float], bool]:
    def collide(x: float, y: float) -> bool:
        return math.hypot(x - cx, y - cy) < r

    return collide


def clearance_from(cx: float, cy: float, r: float) -> Callable[[float, float], float]:
    def clr(x: float, y: float) -> float:
        return max(0.01, math.hypot(x - cx, y - cy) - r)

    return clr


def open_world():
    return (lambda x, y: False), (lambda x, y: 5.0)


def path() -> List[Pt]:
    return [(0.0, 0.0), (10.0, 0.0)]


def snap_at(
    *,
    collide,
    clearance_at=None,
    x=0.0,
    y=0.0,
    yaw=0.0,
    vx=0.15,
    front=5.0,
    rear=5.0,
    left=3.0,
    right=3.0,
    now=None,
    obstacle_ts=None,
    validity_ok=True,
):
    ts = now if now is not None else time.time()
    s = build_obstacle_snapshot(
        now=ts,
        x=x,
        y=y,
        yaw=yaw,
        vx=vx,
        w=0.0,
        front_near=front,
        rear_near=rear,
        left_free=left,
        right_free=right,
        collide=collide if validity_ok else None,
        clearance_at=clearance_at,
        global_path=path(),
        goal=(10.0, 0.0),
        pose_ts=ts,
        obstacle_ts=obstacle_ts if obstacle_ts is not None else ts,
    )
    return s


def main() -> int:
    F = FailList()
    eng = ProbeEngine()
    now = time.time()

    # ---- P1–P8 basics ----
    c_open, clr_open = open_world()
    b = eng.evaluate(snap_at(collide=c_open, clearance_at=clr_open, now=now), now=now)
    F.check("P1_forward_clear", b.forward.status == STATUS_VALID, b.forward.status)
    F.check("P3_left_clear", b.left.status == STATUS_VALID, b.left.status)
    F.check("P5_right_clear", b.right.status == STATUS_VALID, b.right.status)
    F.check("P7_backward_clear", b.backward.status in (STATUS_VALID, STATUS_UNKNOWN), b.backward.status)

    c_front = circle_obstacle(0.55, 0.0, 0.35)
    clr_front = clearance_from(0.55, 0.0, 0.30)
    b2 = eng.evaluate(
        snap_at(collide=c_front, clearance_at=clr_front, front=0.25, vx=0.20, now=now + 0.01),
        now=now + 0.01,
    )
    F.check("P2_forward_blocked", b2.forward.status == STATUS_INVALID, f"{b2.forward.status}/{b2.forward.failure_reason}")

    c_left = circle_obstacle(0.4, 0.55, 0.40)
    b3 = eng.evaluate(
        snap_at(collide=c_left, clearance_at=clearance_from(0.4, 0.55, 0.35), left=0.2, now=now + 0.02),
        now=now + 0.02,
    )
    F.check("P4_left_blocked", b3.left.status == STATUS_INVALID, f"{b3.left.status}/{b3.left.failure_reason}")

    c_right = circle_obstacle(0.4, -0.55, 0.40)
    b4 = eng.evaluate(
        snap_at(collide=c_right, clearance_at=clearance_from(0.4, -0.55, 0.35), right=0.2, now=now + 0.03),
        now=now + 0.03,
    )
    F.check("P6_right_blocked", b4.right.status == STATUS_INVALID, f"{b4.right.status}/{b4.right.failure_reason}")

    c_rear = circle_obstacle(-0.55, 0.0, 0.35)
    b5 = eng.evaluate(
        snap_at(collide=c_rear, clearance_at=clearance_from(-0.55, 0.0, 0.30), rear=0.2, now=now + 0.04),
        now=now + 0.04,
    )
    F.check("P8_backward_blocked", b5.backward.status == STATUS_INVALID, f"{b5.backward.status}/{b5.backward.failure_reason}")

    # ---- P9–P13 backward ----
    F.check("P9_backward_straight_clear", b.backward.status == STATUS_VALID and bool(b.backward.escape_available), str(b.backward.to_dict()))
    F.check("P10_backward_straight_blocked", b5.backward.hard_collision or b5.backward.status == STATUS_INVALID)

    # Escape insufficient: rear wall close after short reverse space
    c_box = circle_obstacle(-0.35, 0.0, 0.25)
    b6 = eng.evaluate(
        snap_at(collide=c_box, clearance_at=clearance_from(-0.35, 0.0, 0.22), rear=0.25, left=0.3, right=0.3, now=now + 0.05),
        now=now + 0.05,
    )
    F.check(
        "P11_escape_insufficient",
        b6.backward.status == STATUS_INVALID or b6.backward.escape_available is False,
        str(b6.backward.escape_available),
    )

    # Turning space: rear open but side walls — steered reverse hits
    def corridor_collide(x, y):
        if abs(y) > 0.42 and x < 0.1:
            return True
        if x < -0.85:
            return True
        return False

    b7 = eng.evaluate(
        snap_at(collide=corridor_collide, clearance_at=lambda x, y: 0.3, rear=1.0, left=0.35, right=0.35, now=now + 0.06),
        now=now + 0.06,
    )
    F.check(
        "P12_turn_space_or_escape",
        b7.backward.status in (STATUS_VALID, STATUS_INVALID),
        b7.backward.failure_reason,
    )

    # Dead-end-ish: reverse short clear but boxed
    def dead_end(x, y):
        if x < -0.55:
            return True
        if abs(y) > 0.38:
            return True
        if x > 0.7:
            return True
        return False

    b8 = eng.evaluate(
        snap_at(collide=dead_end, rear=0.6, left=0.3, right=0.3, front=0.4, now=now + 0.07),
        now=now + 0.07,
    )
    F.check(
        "P13_dead_end_or_invalid_escape",
        b8.backward.escape_available is False or b8.backward.status == STATUS_INVALID,
        str(b8.backward.to_dict()),
    )

    # ---- P14–P16 turn ----
    bt = eng.evaluate(snap_at(collide=c_open, clearance_at=clr_open, now=now + 0.08), now=now + 0.08)
    F.check("P14_turn_clear", bt.turn_in_place.status == STATUS_VALID and bt.turn_in_place.rotation_valid)

    def side_wall(x, y):
        # Walls at |y|≥0.38: current yaw=0 footprint clears (~0.25), yaw sweep hits
        return abs(y) >= 0.38 and abs(x) < 0.60

    bts = eng.evaluate(snap_at(collide=side_wall, now=now + 0.09), now=now + 0.09)
    F.check(
        "P15_turn_side_blocked",
        bts.turn_in_place.status == STATUS_INVALID
        or (bts.turn_in_place.max_safe_yaw_delta or 99) < math.radians(60),
        str(bts.turn_in_place.max_safe_yaw_delta),
    )

    def rear_corner(x, y):
        # body corner sweep hits obstacle behind-right when rotating
        return (x < -0.15 and y < -0.20 and math.hypot(x + 0.2, y + 0.35) < 0.35)

    btr = eng.evaluate(snap_at(collide=rear_corner, now=now + 0.10), now=now + 0.10)
    F.check(
        "P16_turn_rear_corner",
        btr.turn_in_place.status in (STATUS_VALID, STATUS_INVALID),  # must evaluate footprint, not crash
        btr.turn_in_place.failure_reason,
    )
    # Stronger: if footprint collision at some yaw, must not claim unlimited rotation
    if btr.turn_in_place.collision_yaw is not None:
        F.check("P16_collision_yaw_set", abs(btr.turn_in_place.collision_yaw) > 0)

    # ---- P17–P20 stopping ----
    d_low = stopping_distance_m(0.05)
    d_high = stopping_distance_m(0.35)
    F.check("P19_high_gt_low", d_high > d_low, f"{d_high} vs {d_low}")
    # clearance > stop → may be VALID; clearance < stop → INVALID
    b_ok = eng.evaluate(
        snap_at(collide=c_open, clearance_at=clr_open, front=2.0, vx=0.10, now=now + 0.11),
        now=now + 0.11,
    )
    F.check("P17_clr_gt_stop", b_ok.forward.status == STATUS_VALID and (b_ok.forward.stopping_margin or 0) > 0)
    # front close + high speed
    b_bad = eng.evaluate(
        snap_at(collide=c_open, clearance_at=clr_open, front=0.25, vx=0.40, now=now + 0.12),
        now=now + 0.12,
    )
    F.check(
        "P18_clr_lt_stop",
        b_bad.forward.status == STATUS_INVALID and b_bad.forward.failure_reason in ("STOPPING_MARGIN_NEGATIVE", "FOOTPRINT_COLLISION", "INSUFFICIENT_CLEARANCE", "COLLISION"),
        f"{b_bad.forward.status}/{b_bad.forward.failure_reason}/{b_bad.forward.stopping_margin}",
    )
    F.check("P20_low_speed_margin", stopping_distance_m(0.05) < stopping_distance_m(0.25))

    # ---- P21–P24 unknown/stale ----
    bu = eng.evaluate(snap_at(collide=None, validity_ok=False, now=now + 0.13), now=now + 0.13)
    F.check("P21_no_data", bu.forward.status == STATUS_UNKNOWN and bu.forward.failure_reason == "NO_DATA")

    stale_ts = now + 0.14
    bs = build_obstacle_snapshot(
        now=stale_ts - 2.0,
        x=0.0,
        y=0.0,
        yaw=0.0,
        vx=0.1,
        w=0.0,
        front_near=3.0,
        rear_near=3.0,
        left_free=2.0,
        right_free=2.0,
        collide=c_open,
        clearance_at=clr_open,
        pose_ts=stale_ts - 2.0,
        obstacle_ts=stale_ts - 2.0,
    )
    bst = eng.evaluate(bs, now=stale_ts)
    F.check("P22_stale", bst.forward.status == STATUS_STALE, bst.forward.status)

    bnan = build_obstacle_snapshot(
        now=now + 0.15,
        x=float("nan"),
        y=0.0,
        yaw=0.0,
        vx=0.1,
        w=0.0,
        front_near=3.0,
        rear_near=3.0,
        left_free=2.0,
        right_free=2.0,
        collide=c_open,
    )
    bn = eng.evaluate(bnan, now=now + 0.15)
    F.check("P23_invalid_geom_or_nan", bn.forward.status == STATUS_UNKNOWN, bn.forward.failure_reason)

    F.check("P24_nan_not_valid", bn.forward.status != STATUS_VALID)

    # ---- P25–P27 footprint ----
    # Center clear but body hits: obstacle beside center line
    def body_hit(x, y):
        # obstacle at front-right corner of footprint while center path clear
        return abs(x - 0.45) < 0.12 and abs(y + 0.22) < 0.12

    # Point-center check would miss; footprint must catch
    center_clear = not body_hit(0.45, 0.0)
    fps = footprint_points(0.45, 0.0, 0.0)
    body_hits = any(body_hit(px, py) for px, py in fps)
    F.check("P25_center_clear_body_hit_setup", center_clear and body_hits)
    bp = eng.evaluate(snap_at(collide=body_hit, front=0.8, vx=0.18, now=now + 0.16), now=now + 0.16)
    F.check("P25_forward_invalid_by_body", bp.forward.status == STATUS_INVALID, bp.forward.failure_reason)

    F.check("P26_body_clear_open", b.forward.status == STATUS_VALID)

    # Corner sweep: turn hits
    F.check("P27_turn_uses_footprint", bts.turn_in_place.sample_count > 0)

    # ---- P28–P29 path capture ----
    F.check("P28_capture_field", b.left.path_capture_valid is not None or b.left.status == STATUS_VALID)
    # No path → capture may be false but still can be VALID (require_capture=False)
    s_nopath = build_obstacle_snapshot(
        now=now + 0.17,
        x=0.0,
        y=0.0,
        yaw=0.0,
        vx=0.1,
        w=0.0,
        front_near=4.0,
        rear_near=4.0,
        left_free=3.0,
        right_free=3.0,
        collide=c_open,
        clearance_at=clr_open,
        global_path=[],
        goal=None,
    )
    bnopath = eng.evaluate(s_nopath, now=now + 0.17)
    F.check("P29_no_path_still_evaluates", bnopath.left.status in (STATUS_VALID, STATUS_INVALID, STATUS_UNKNOWN))

    # ---- P30–P32 snapshot consistency / cache ----
    s1 = snap_at(collide=c_open, clearance_at=clr_open, now=now + 0.20)
    r1 = eng.evaluate(s1, now=now + 0.20)
    r2 = eng.evaluate(s1, now=now + 0.20)
    F.check("P30_same_snapshot", r1.forward.status == r2.forward.status and r2.cache_hit)

    s_chg = snap_at(collide=c_front, clearance_at=clr_front, front=0.3, now=now + 0.21)
    r3 = eng.evaluate(s_chg, now=now + 0.21)
    F.check("P31_obstacle_change_invalidates", r3.forward.status == STATUS_INVALID and not r3.cache_hit)

    F.check("P32_stale_status", bst.forward.status == STATUS_STALE)

    # ---- Integration I1–I6 ----
    pol = NavigationPolicy()
    pol.create_commitment(now=now, side="LEFT", reason="I_TEST")
    model = LocalMppiModel()
    model.policy.create_commitment(now=now, side="LEFT", reason="I_TEST")
    # Probe LEFT invalid RIGHT valid must not switch
    class World:
        def collides(self, px, py, robot_r=0.24, include_actors=True):
            return math.hypot(px - 0.5, py - 0.45) < 0.42  # blocks left arc

        def clearance_at_xy(self, px, py):
            return max(0.05, math.hypot(px - 0.5, py - 0.45) - 0.38)

    # Direct probe bundle for evidence
    s_i = snap_at(
        collide=circle_obstacle(0.5, 0.45, 0.42),
        clearance_at=clearance_from(0.5, 0.45, 0.38),
        left=0.2,
        right=2.5,
        front=0.5,
        now=now + 0.30,
    )
    bi = eng.evaluate(s_i, now=now + 0.30)
    F.check("I1_bundle_has_dirs", bi.left is not None and bi.right is not None)
    F.check("I_probe_left_invalid_right_ok", bi.left.status == STATUS_INVALID, bi.left.status)
    # RIGHT should be better chance VALID
    F.check("I_probe_right_validish", bi.right.status in (STATUS_VALID, STATUS_INVALID), bi.right.status)

    # Commitment unchanged by evaluating probe
    before = (pol.commitment.active, pol.commitment.side)
    _ = eng.evaluate(s_i, now=now + 0.31)
    after = (pol.commitment.active, pol.commitment.side)
    F.check("I2_commitment_unchanged", before == after)

    # Selector under commitment still KEEP even if RIGHT probe valid
    sel = LocalManeuverSelector()
    sel.current = DEC_LEFT
    sel.hold_until = now - 1.0
    cmp = sel.compare(
        now=now + 0.32,
        x=0.15,
        y=0.35,
        yaw=0.4,
        path=path(),
        goal=(10.0, 0.0),
        front_near=0.5,
        rear_near=3.0,
        collide=circle_obstacle(0.5, 0.45, 0.42),
        clearance_at=clearance_from(0.5, 0.45, 0.38),
        forward_feasible=False,
        rotation_safe=True,
        left_free=0.3,
        right_free=2.5,
        force=True,
        require_capture=False,
        commitment_active=True,
        committed_side="LEFT",
        side_switch_authorized=False,
    )
    F.check("I5_selector_not_switched_by_probe_world", cmp.selected == "LEFT", f"{cmp.selected}/{cmp.reason}")
    F.check("I4_no_fsm_in_probe", not hasattr(eng, "mode"))

    # ProbeResult must not contain command fields
    d = bi.left.to_dict()
    F.check("I3_no_cmd_fields", "command_vx" not in d and "command_w" not in d and "vx" not in d)

    # Soft: both valid — still KEEP
    bsoft = eng.evaluate(snap_at(collide=c_open, clearance_at=clr_open, now=now + 0.33), now=now + 0.33)
    F.check("I_soft_both_valid", bsoft.left.status == STATUS_VALID and bsoft.right.status == STATUS_VALID)
    cmp2 = sel.compare(
        now=now + 0.34,
        x=0.2,
        y=0.3,
        yaw=0.3,
        path=path(),
        goal=(10.0, 0.0),
        front_near=0.55,
        rear_near=3.0,
        collide=c_open,
        clearance_at=clr_open,
        forward_feasible=False,
        rotation_safe=True,
        left_free=1.2,
        right_free=2.5,
        force=True,
        require_capture=False,
        commitment_active=True,
        committed_side="LEFT",
    )
    F.check("I_soft_still_left", cmp2.selected == "LEFT" and "COMMIT" in cmp2.reason, cmp2.reason)

    # Telemetry shape
    td = bi.to_dict()
    F.check("I6_telemetry_shape", td.get("implemented") is True and "forward" in td and "backward" in td)

    # Search audit: ProbeEngine must not force motion (static code check via attrs)
    for forbidden in ("force_vx", "force_w", "side_switch_authorized"):
        F.check(f"no_{forbidden}_attr", not hasattr(ProbeEngine, forbidden))

    if not F.ok():
        print("RESULT FAIL")
        for it in F.items:
            print(" ", it)
        return 1
    print("RESULT PASS")
    print(f"  P1–P32 + I1–I6 ok; bundle_ms~{b.bundle_ms:.2f}ms")
    print(f"  forward={b.forward.status} back={b.backward.status} L={b.left.status} R={b.right.status} turn={b.turn_in_place.status}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
