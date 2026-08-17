#!/usr/bin/env python3
"""P0-B — Global Reference Preview audits (Tests A–J). Reference-only; no control claims."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_global_preview import (  # noqa: E402
    GLOBAL_PREVIEW_MAX_M,
    GLOBAL_PREVIEW_MIN_M,
    GLOBAL_PREVIEW_NORMAL_M,
    adaptive_preview_m,
    build_global_reference,
    densify_polyline,
    expected_local_distance_m,
    first_meaningful_turn,
    preview_enabled,
)


def _line(n: int = 40, step: float = 0.5) -> List[Dict[str, float]]:
    return [{"x": i * step, "y": 0.0} for i in range(n + 1)]


def _l_turn(straight_m: float = 4.0, after_m: float = 3.0, step: float = 0.25) -> List[Dict[str, float]]:
    pts: List[Dict[str, float]] = []
    x = 0.0
    while x <= straight_m + 1e-9:
        pts.append({"x": round(x, 3), "y": 0.0})
        x += step
    y = step
    while y <= after_m + 1e-9:
        pts.append({"x": round(straight_m, 3), "y": round(y, 3)})
        y += step
    return pts


def _ok(cond: bool, msg: str, fails: List[str]) -> None:
    if not cond:
        fails.append(msg)


def main() -> int:
    fails: List[str] = []
    print("=== P0-B Global Preview Audit (A–J) ===")
    print(f"NORMAL={GLOBAL_PREVIEW_NORMAL_M} MIN={GLOBAL_PREVIEW_MIN_M} MAX={GLOBAL_PREVIEW_MAX_M}")

    # A — Open scene: long path + far goal → ~5m (4–8)
    path_long = _line(40, 0.5)  # 20m
    g_a = build_global_reference(
        path=path_long, x=0.0, y=0.0, yaw=0.0, path_progress_s=0.0, goal_distance_m=18.0, path_revision=1
    )
    _ok(4.0 <= float(g_a["preview_m"]) <= 8.0, f"A: preview_m={g_a['preview_m']} not in [4,8]", fails)
    _ok(abs(float(g_a["preview_m"]) - GLOBAL_PREVIEW_NORMAL_M) < 0.6, f"A: not ~5m got {g_a['preview_m']}", fails)
    _ok(g_a.get("kinematic_valid") is None, "A: kinematic_valid must be null", fails)
    _ok(g_a.get("controls_vehicle") is False, "A: must not control vehicle", fails)
    print(f"A open-scene preview_m={g_a['preview_m']} reason={g_a.get('preview_reason')} ->", "PASS" if not any(f.startswith("A:") for f in fails) else "FAIL")

    # B — Goal limited
    g_b = build_global_reference(
        path=path_long, x=0.0, y=0.0, yaw=0.0, path_progress_s=0.0, goal_distance_m=3.0, path_revision=1
    )
    _ok(float(g_b["preview_m"]) <= 3.0 + 0.15, f"B: preview={g_b['preview_m']} > goal 3m", fails)
    _ok(g_b.get("preview_reason") == "GOAL_LIMITED", f"B: reason={g_b.get('preview_reason')}", fails)
    print(f"B goal-limited preview_m={g_b['preview_m']} ->", "PASS" if not any(f.startswith("B:") for f in fails) else "FAIL")

    # C — Path limited
    path_short = _line(int(2.4 / 0.2), 0.2)  # ~2.4m
    g_c = build_global_reference(
        path=path_short, x=0.0, y=0.0, yaw=0.0, path_progress_s=0.0, goal_distance_m=20.0, path_revision=2
    )
    rem = float(g_c.get("remaining_m") or 0.0)
    _ok(float(g_c["preview_m"]) <= rem + 0.15, f"C: preview={g_c['preview_m']} > remaining={rem}", fails)
    _ok(g_c.get("preview_reason") in ("PATH_LIMITED", "GOAL_LIMITED"), f"C: reason={g_c.get('preview_reason')}", fails)
    print(f"C path-limited preview_m={g_c['preview_m']} rem={rem} ->", "PASS" if not any(f.startswith("C:") for f in fails) else "FAIL")

    # D — Preview starts near vehicle
    g_d = build_global_reference(
        path=path_long, x=2.0, y=0.05, yaw=0.0, path_progress_s=2.0, goal_distance_m=15.0, path_revision=1
    )
    poses = g_d.get("poses") or []
    _ok(len(poses) >= 2, "D: no poses", fails)
    if poses:
        d0 = math.hypot(float(poses[0]["x"]) - 2.0, float(poses[0]["y"]) - 0.05)
        _ok(d0 < 0.35, f"D: start offset {d0:.3f}m too large", fails)
        _ok(float(g_d.get("start_offset_m") or d0) < 0.35, "D: start_offset_m too large", fails)
    print(f"D start_near vehicle offset={g_d.get('start_offset_m')} ->", "PASS" if not any(f.startswith("D:") for f in fails) else "FAIL")

    # E — Progress stability: advance 0.2m → still ~5m ahead (not collapse)
    g_e0 = build_global_reference(
        path=path_long, x=1.0, y=0.0, yaw=0.0, path_progress_s=1.0, goal_distance_m=16.0, path_revision=1
    )
    g_e1 = build_global_reference(
        path=path_long, x=1.2, y=0.0, yaw=0.0, path_progress_s=1.2, goal_distance_m=15.8, path_revision=1
    )
    _ok(abs(float(g_e1["preview_m"]) - float(g_e0["preview_m"])) < 0.8, f"E: preview jump {g_e0['preview_m']}->{g_e1['preview_m']}", fails)
    _ok(float(g_e1["preview_m"]) >= 4.0, f"E: collapsed to {g_e1['preview_m']}", fails)
    print(f"E stable {g_e0['preview_m']}->{g_e1['preview_m']} ->", "PASS" if not any(f.startswith("E:") for f in fails) else "FAIL")

    # F — First turn metadata on L-path
    lpath = _l_turn(4.0, 3.0, 0.2)
    dens = densify_polyline(lpath)
    turn_d, turn_deg = first_meaningful_turn(dens)
    g_f = build_global_reference(
        path=lpath, x=0.0, y=0.0, yaw=0.0, path_progress_s=0.0, goal_distance_m=10.0, path_revision=3
    )
    _ok(g_f.get("first_turn_distance_m") is not None, "F: missing first_turn_distance_m", fails)
    ftd = float(g_f.get("first_turn_distance_m") or -1)
    _ok(3.2 <= ftd <= 4.6, f"F: first_turn={ftd} not near ~4m corner", fails)
    _ok(float(g_f.get("heading_change_deg") or 0) >= 45.0, f"F: heading_change too small {g_f.get('heading_change_deg')}", fails)
    # tiny sawtooth should not count
    zig = [{"x": i * 0.15, "y": (0.02 if i % 2 else -0.02)} for i in range(40)]
    zd, zdeg = first_meaningful_turn(densify_polyline(zig))
    _ok(zd is None, f"F: sawtooth falsely detected turn d={zd} deg={zdeg}", fails)
    print(f"F first_turn={g_f.get('first_turn_distance_m')} deg={g_f.get('heading_change_deg')} ->", "PASS" if not any(f.startswith("F:") for f in fails) else "FAIL")

    # G — Global/Local separation (expected local short)
    exp = expected_local_distance_m(state_vx=0.18, requested_vx=0.18, horizon_s=1.5)
    _ok(0.15 <= exp <= 0.45, f"G: expected local {exp} not in 0.2–0.4 band", fails)
    _ok(float(g_a["preview_m"]) >= 4.0 and exp < 1.0, "G: global/local not separated", fails)
    print(f"G global={g_a['preview_m']} expected_local={exp:.3f} ->", "PASS" if not any(f.startswith("G:") for f in fails) else "FAIL")

    # H — Preview ON/OFF does not change adaptive math when disabled returns empty; behavior gate is env
    os.environ["NAV_GLOBAL_PREVIEW"] = "0"
    _ok(preview_enabled() is False, "H: NAV_GLOBAL_PREVIEW=0 should disable", fails)
    os.environ["NAV_GLOBAL_PREVIEW"] = "1"
    _ok(preview_enabled() is True, "H: NAV_GLOBAL_PREVIEW=1 should enable", fails)
    # densify / adaptive pure functions identical regardless of env
    p1, r1 = adaptive_preview_m(remaining_path_m=20.0, remaining_goal_m=20.0)
    p2, r2 = adaptive_preview_m(remaining_path_m=20.0, remaining_goal_m=20.0)
    _ok(p1 == p2 and r1 == r2, "H: adaptive not deterministic", fails)
    print("H env toggle / deterministic ->", "PASS" if not any(f.startswith("H:") for f in fails) else "FAIL")

    # I — path revision stamped
    g_i1 = build_global_reference(path=path_long, x=0, y=0, yaw=0, goal_distance_m=12, path_revision=42)
    g_i2 = build_global_reference(path=path_long, x=0, y=0, yaw=0, goal_distance_m=12, path_revision=43)
    _ok(int(g_i1["path_revision"]) == 42 and int(g_i2["path_revision"]) == 43, "I: revision not stamped", fails)
    print(f"I revision {g_i1['path_revision']}->{g_i2['path_revision']} ->", "PASS" if not any(f.startswith("I:") for f in fails) else "FAIL")

    # J — Goal reached
    g_j = build_global_reference(
        path=path_long, x=0, y=0, yaw=0, goal_distance_m=0.1, path_revision=5, goal_reached=True
    )
    _ok(g_j.get("status") == "GOAL_REACHED", f"J: status={g_j.get('status')}", fails)
    _ok(not (g_j.get("poses") or []), "J: poses should be empty at goal", fails)
    _ok(g_j.get("kinematic_valid") is None, "J: kinematic_valid must stay null", fails)
    print(f"J goal_reached status={g_j.get('status')} ->", "PASS" if not any(f.startswith("J:") for f in fails) else "FAIL")

    # densify quality
    dens2 = densify_polyline(_line(10, 1.0), spacing_m=0.10)
    gaps = []
    for i in range(1, len(dens2)):
        gaps.append(math.hypot(dens2[i]["x"] - dens2[i - 1]["x"], dens2[i]["y"] - dens2[i - 1]["y"]))
    _ok(gaps and max(gaps) <= 0.11, f"densify gap max={max(gaps) if gaps else None}", fails)
    _ok(all("yaw" in p for p in dens2), "densify missing yaw", fails)

    if fails:
        print("FAILS:")
        for f in fails:
            print(" -", f)
        print("P0-B AUDIT = FAIL")
        return 1
    print("P0-B AUDIT = PASS (A–J)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
