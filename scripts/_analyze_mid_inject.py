#!/usr/bin/env python3
"""Compress mid_inject / side-switch JSONL into STEP2 timeline facts."""
from __future__ import annotations

import json
import sys
from pathlib import Path


def main() -> int:
    path = Path(sys.argv[1])
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    samples = [r for r in rows if r.get("type") == "sample"]
    injections = [r for r in rows if r.get("type") == "scene_injection"]
    switches = [r for r in rows if r.get("type") == "side_switch"]
    summary = next((r for r in rows if r.get("type") == "summary"), {})

    print("FILE", path)
    print("SUMMARY", json.dumps(summary, ensure_ascii=False))
    print("INJECTIONS", [(i.get("t"), i.get("stage")) for i in injections])
    print("SWITCHES", [(s.get("t"), s.get("from"), s.get("to")) for s in switches])

    def key_events(s):
        return (
            s.get("maneuver_mode"),
            s.get("policy_state"),
            s.get("selector_selected"),
            s.get("selector_reason"),
            s.get("allow_side_compare"),
            s.get("stop_reason"),
            round(float(s.get("left_cost") or -1), 1) if s.get("left_cost") is not None else None,
            round(float(s.get("right_cost") or -1), 1) if s.get("right_cost") is not None else None,
            s.get("left_feasible"),
            s.get("right_feasible"),
        )

    print("--- compressed ---")
    prev = None
    for s in samples:
        k = key_events(s)
        if k != prev:
            print(
                f"t={s.get('t'):.3f} mode={s.get('maneuver_mode')} pol={s.get('policy_state')} "
                f"sel={s.get('selector_selected')} reason={s.get('selector_reason')} "
                f"allow={s.get('allow_side_compare')} L={s.get('left_cost')} R={s.get('right_cost')} "
                f"Lf={s.get('left_feasible')} Rf={s.get('right_feasible')} "
                f"stop={s.get('stop_reason')} front={s.get('front_near')} "
                f"mppi_vx={s.get('mppi_vx')} mppi_w={s.get('mppi_w')} "
                f"safe_vx={s.get('safe_vx')} safe_w={s.get('safe_w')} "
                f"state_vx={s.get('state_vx')} state_w={s.get('state_w')} "
                f"stuck={s.get('stuck_s')} avoid={s.get('avoid_side')}"
            )
            prev = k

    # LEFT / RIGHT decision windows
    left_rows = [s for s in samples if (s.get("maneuver_mode") or "") == "LOCAL_LEFT"]
    right_rows = [s for s in samples if (s.get("maneuver_mode") or "") == "LOCAL_RIGHT"]
    if left_rows:
        L0 = left_rows[0]
        print("\nLEFT_DECISION_TIME", L0.get("t"))
        print("LEFT_DETAIL", {k: L0.get(k) for k in (
            "left_cost","right_cost","left_feasible","right_feasible","selector_reason",
            "allow_side_compare","mppi_w","safe_vx","safe_w","stop_reason","front_near",
            "avoid_side","left_clr","right_clr","left_capture","right_capture","maneuver_reason",
            "stuck_s","obstacles",
        )})
    if right_rows:
        R0 = right_rows[0]
        print("\nRIGHT_DECISION_TIME", R0.get("t"))
        print("RIGHT_DETAIL", {k: R0.get(k) for k in (
            "left_cost","right_cost","left_feasible","right_feasible","selector_reason",
            "allow_side_compare","mppi_w","safe_vx","safe_w","stop_reason","front_near",
            "avoid_side","left_clr","right_clr","left_capture","right_capture","maneuver_reason",
            "stuck_s","obstacles",
        )})
        if left_rows:
            print("DELTA_T", round(float(R0["t"]) - float(L0["t"]), 4))

    # post-switch 5s window
    if switches:
        sw_t = float(switches[0]["t"])
        print("\n--- post-switch window ---")
        for s in samples:
            if float(s.get("t") or 0) < sw_t - 0.2:
                continue
            if float(s.get("t") or 0) > sw_t + 8.0:
                break
            if abs(float(s.get("t") or 0) - sw_t) < 0.02 or key_events(s) != prev:
                print(
                    f"t={s.get('t'):.3f} mode={s.get('maneuver_mode')} stop={s.get('stop_reason')} "
                    f"front={s.get('front_near')} mppi_w={s.get('mppi_w')} safe_vx={s.get('safe_vx')} "
                    f"state_vx={s.get('state_vx')} stuck={s.get('stuck_s')} "
                    f"L={s.get('left_cost')} R={s.get('right_cost')} reason={s.get('selector_reason')}"
                )

    # allow_side_compare table during avoid
    print("\n--- allow/hold proxy during LOCAL_* ---")
    for s in samples:
        mode = s.get("maneuver_mode") or ""
        if "LOCAL_" not in mode and mode not in ("REPOSITION", "SAFE_STOP"):
            continue
        if float(s.get("t") or 0) % 0.25 > 0.08 and mode == getattr(main, "_pm", None):
            continue
        print(
            f"t={s.get('t'):.3f} mode={mode} allow={s.get('allow_side_compare')} "
            f"avoid={s.get('avoid_side')} sel={s.get('selector_selected')} "
            f"reason={s.get('selector_reason')} stop={s.get('stop_reason')}"
        )
        main._pm = mode  # type: ignore
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
