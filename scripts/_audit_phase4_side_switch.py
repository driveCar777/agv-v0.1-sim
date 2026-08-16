#!/usr/bin/env python3
"""STEP 3E Side Switch Authorization offline audit (E1–E20 style)."""

from __future__ import annotations

import time
from typing import List, Optional

from agv_bridge.local_maneuver import LocalManeuverSelector, DEC_LEFT, DEC_RIGHT
from agv_bridge.nav_policy import NavigationPolicy
from agv_bridge.nav_probe import (
    STATUS_INVALID,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_VALID,
    ObstacleSnapshot,
    ProbeBundle,
    ProbeEngine,
    ProbeResult,
    build_obstacle_snapshot,
)
from agv_bridge.nav_side_switch import (
    AUTH_STATUS_AUTHORIZED,
    AUTH_STATUS_DENIED,
    REASON_ALT_PROBE_INVALID,
    REASON_COOLDOWN,
    REASON_CURRENT_SIDE_STILL_VALID,
    REASON_DYNAMIC_WAIT,
    REASON_NO_COMMITMENT,
    REASON_SAFETY_DENIED,
    REASON_STALE_PROBE,
    REASON_TURN_NOT_SAFE,
    REASON_UNKNOWN_PROBE,
    SIDE_SWITCH_AUTH_TTL_S,
    authorize_side_switch,
    validate_token_for_execution,
)


class FailList:
    def __init__(self) -> None:
        self.items: List[str] = []

    def check(self, name: str, cond: bool, detail: str = "") -> None:
        if not cond:
            self.items.append(f"{name}: {detail}" if detail else name)

    def ok(self) -> bool:
        return not self.items


def _pr(direction: str, status: str, **kw) -> ProbeResult:
    r = ProbeResult(direction=direction, status=status, timestamp=time.time())
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def _bundle(
    *,
    left: str,
    right: str,
    forward: str = STATUS_INVALID,
    backward: str = STATUS_VALID,
    turn: str = STATUS_VALID,
    now: Optional[float] = None,
    age_s: float = 0.0,
) -> ProbeBundle:
    ts = now if now is not None else time.time()
    snap = build_obstacle_snapshot(
        now=ts - age_s,
        x=0.0,
        y=0.0,
        yaw=0.0,
        vx=0.1,
        w=0.0,
        front_near=0.5,
        rear_near=2.0,
        left_free=0.3,
        right_free=2.0,
        collide=lambda x, y: False,
        clearance_at=lambda x, y: 2.0,
        pose_ts=ts - age_s,
        obstacle_ts=ts - age_s,
    )
    return ProbeBundle(
        snapshot=snap,
        forward=_pr("FORWARD", forward, hard_collision=(forward == STATUS_INVALID)),
        backward=_pr("BACKWARD", backward),
        left=_pr(
            "LEFT",
            left,
            hard_collision=(left == STATUS_INVALID),
            collision=(left == STATUS_INVALID),
        ),
        right=_pr(
            "RIGHT",
            right,
            hard_collision=(right == STATUS_INVALID),
            collision=(right == STATUS_INVALID),
        ),
        turn_in_place=_pr("TURN_IN_PLACE", turn, rotation_valid=(turn == STATUS_VALID)),
        bundle_ms=0.1,
        input_signature=snap.signature(),
    )


def main() -> int:
    F = FailList()
    now = time.time()

    # E20 no commitment
    d0 = authorize_side_switch(
        now=now,
        commitment=NavigationPolicy().commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
    )
    F.check("E20_no_commitment", d0.primary_reason == REASON_NO_COMMITMENT, d0.primary_reason)

    pol = NavigationPolicy()
    pol.create_commitment(now=now, side="LEFT", reason="TEST", signature="f0.5_L0.3_R2.0_T")

    # E3 soft — both valid → DENY CURRENT_SIDE_STILL_VALID
    d3 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_VALID, right=STATUS_VALID, forward=STATUS_VALID, now=now),
        raw_candidate_side="RIGHT",
    )
    F.check("E3_soft_still_valid", not d3.authorized and d3.primary_reason == REASON_CURRENT_SIDE_STILL_VALID)

    # E6 alternative invalid
    d6 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_INVALID, now=now),
        raw_candidate_side="RIGHT",
    )
    F.check("E6_alt_invalid", not d6.authorized and d6.primary_reason == REASON_ALT_PROBE_INVALID)

    # E7 unknown alt
    d7 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_UNKNOWN, now=now),
    )
    F.check("E7_unknown", not d7.authorized and d7.primary_reason == REASON_UNKNOWN_PROBE)

    # E8 stale alt
    d8 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_STALE, now=now),
    )
    F.check("E8_stale_alt", not d8.authorized and d8.primary_reason == REASON_STALE_PROBE)

    # E9 turn invalid
    d9 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, turn=STATUS_INVALID, now=now),
    )
    F.check("E9_turn", not d9.authorized and d9.primary_reason == REASON_TURN_NOT_SAFE)

    # E10 safety
    d10 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
        safety_zero=True,
    )
    F.check("E10_safety", not d10.authorized and d10.primary_reason == REASON_SAFETY_DENIED)

    # E19 dynamic
    d19 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
        dynamic_short=True,
    )
    F.check("E19_dynamic", not d19.authorized and d19.primary_reason == REASON_DYNAMIC_WAIT)

    # E5 / E18 authorize success
    d5 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
        raw_candidate_side="RIGHT",
    )
    F.check("E5_authorized", d5.authorized and d5.status == AUTH_STATUS_AUTHORIZED, d5.primary_reason)
    F.check("E5_token", d5.token is not None and d5.token.to_side == "RIGHT")

    # E11 cooldown
    pol.commitment.last_switch_at = now - 0.1
    d11 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
    )
    F.check("E11_cooldown", not d11.authorized and d11.primary_reason == REASON_COOLDOWN)
    pol.commitment.last_switch_at = None

    # E12 oscillation
    times = [now - 1.0, now - 2.0, now - 3.0]
    d12 = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
        switch_times=times,
    )
    F.check("E12_oscillation", not d12.authorized and "OSCILLATION" in d12.primary_reason)

    # E14 token expiry
    d_ok = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
    )
    tok = d_ok.token
    assert tok is not None
    ok, why = validate_token_for_execution(
        tok, now=now + SIDE_SWITCH_AUTH_TTL_S + 1.0, current_side="LEFT", commitment_side="LEFT"
    )
    F.check("E14_expiry", not ok and why == "AUTHORIZATION_EXPIRED", why)

    # E15 wrong from_side
    ok2, why2 = validate_token_for_execution(
        tok, now=now, current_side="RIGHT", commitment_side="LEFT"
    )
    F.check("E15_mismatch", not ok2 and "MISMATCH" in why2, why2)

    # E16 one-shot consume via Policy
    pol2 = NavigationPolicy()
    pol2.create_commitment(now=now, side="LEFT", reason="T", signature="f0.5_L0.3_R2.0_T")
    sw = pol2.authorize_side_switch_gate(
        now=now,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
        raw_candidate_side="RIGHT",
    )
    F.check("E16_gate_auth", sw.authorized)
    ok_exec = pol2.note_side_switch_executed(now=now, from_side="LEFT", to_side="RIGHT", maneuver_mode="LOCAL_RIGHT")
    F.check("E16_executed", ok_exec and pol2.commitment.side == "RIGHT", pol2.commitment.side)
    F.check("E16_consumed", pol2.switch_token is None)
    # second execute fails
    F.check(
        "E16_oneshot",
        not pol2.note_side_switch_executed(now=now + 0.01, from_side="LEFT", to_side="RIGHT"),
    )

    # Selector: unauthorized RIGHT_ONLY blocked; authorized forces switch
    sel = LocalManeuverSelector()
    sel.current = DEC_LEFT
    sel.hold_until = now - 1.0
    r_keep = sel._select(
        now,
        {
            DEC_LEFT: type("C", (), {"feasible": False, "total_cost": 700, "type": DEC_LEFT})(),
            DEC_RIGHT: type("C", (), {"feasible": True, "total_cost": 40, "type": DEC_RIGHT})(),
            "FORWARD": type("C", (), {"feasible": False, "total_cost": 900, "type": "FORWARD", "route_quality": "BLOCKED"})(),
            "WAIT": type("C", (), {"feasible": False, "total_cost": 900, "type": "WAIT"})(),
        },
        forward_feasible=False,
        dynamic_short=False,
        commitment_active=True,
        committed_side="LEFT",
        side_switch_authorized=False,
    )
    # Need proper candidate objects - use simpler compare path via authorize only
    # Direct _select with minimal stubs may fail on FORWARD quality check
    # Use POLICY_AUTHORIZED path:
    r_auth = sel._select(
        now + 0.1,
        {
            DEC_LEFT: type("C", (), {"feasible": False, "total_cost": 700, "type": DEC_LEFT})(),
            DEC_RIGHT: type("C", (), {"feasible": True, "total_cost": 40, "type": DEC_RIGHT})(),
            "FORWARD": type(
                "C", (), {"feasible": False, "total_cost": 900, "type": "FORWARD", "route_quality": "BLOCKED"}
            )(),
            "WAIT": type("C", (), {"feasible": False, "total_cost": 900, "type": "WAIT"})(),
        },
        forward_feasible=False,
        dynamic_short=False,
        commitment_active=True,
        committed_side="LEFT",
        side_switch_authorized=True,
        authorized_side="RIGHT",
    )
    F.check("E18_policy_auth_select", r_auth[0] == "RIGHT" and r_auth[1] == "POLICY_AUTHORIZED_SWITCH", str(r_auth))

    r_block = sel._select(
        now + 0.2,
        {
            DEC_LEFT: type("C", (), {"feasible": False, "total_cost": 700, "type": DEC_LEFT})(),
            DEC_RIGHT: type("C", (), {"feasible": True, "total_cost": 40, "type": DEC_RIGHT})(),
            "FORWARD": type(
                "C", (), {"feasible": False, "total_cost": 900, "type": "FORWARD", "route_quality": "BLOCKED"}
            )(),
            "WAIT": type("C", (), {"feasible": False, "total_cost": 900, "type": "WAIT"})(),
        },
        forward_feasible=False,
        dynamic_short=False,
        commitment_active=True,
        committed_side="LEFT",
        side_switch_authorized=False,
    )
    F.check(
        "E4_revalidation_keep",
        r_block[0] == "LEFT" and "COMMIT" in r_block[1],
        str(r_block),
    )

    # Selector says RIGHT, Probe INVALID → DENY (evidence reconciliation)
    d_recon = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_INVALID, now=now),
        raw_candidate_side="RIGHT",
    )
    F.check(
        "recon_selector_right_probe_invalid",
        not d_recon.authorized and d_recon.primary_reason == REASON_ALT_PROBE_INVALID,
    )
    F.check("recon_inconsistent_flag", d_recon.evidence_consistent is False)

    # Stale snapshot age
    d_age = authorize_side_switch(
        now=now,
        commitment=pol.commitment,
        probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now, age_s=2.0),
    )
    F.check("E8b_probe_age_stale", not d_age.authorized and d_age.primary_reason == REASON_STALE_PROBE)

    # Auth latency budget
    t0 = time.perf_counter()
    for _ in range(200):
        authorize_side_switch(
            now=now,
            commitment=pol.commitment,
            probe_bundle=_bundle(left=STATUS_INVALID, right=STATUS_VALID, now=now),
        )
    avg_ms = (time.perf_counter() - t0) * 1000.0 / 200.0
    F.check("auth_latency_lt_1ms", avg_ms < 1.0, f"avg={avg_ms:.3f}ms")

    if not F.ok():
        print("RESULT FAIL")
        for it in F.items:
            print(" ", it)
        return 1
    print("RESULT PASS")
    print(f"  E3 soft DENY / E5 AUTHORIZED / E6 alt INVALID / gates OK")
    print(f"  auth avg {avg_ms:.3f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
