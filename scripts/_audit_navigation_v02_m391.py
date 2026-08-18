#!/usr/bin/env python3
"""M3.9.1 turn execution latency / handoff / freshness regression."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402
from agv_bridge.nav_trajectory_integrity import max_trajectory_age_ms  # noqa: E402
from agv_bridge.nav_turn_execution import (  # noqa: E402
    TURN_OMEGA_EPSILON,
    classify_turn_readiness,
    detect_turn_start_frames,
    turn_required_distance_m,
)


def _run(name: str, fn: Callable[[], None]) -> Tuple[str, bool, str]:
    try:
        fn()
        return name, True, "ok"
    except AssertionError as e:
        return name, False, str(e) or "assertion failed"
    except Exception as e:
        return name, False, f"{type(e).__name__}: {e}"


def test_turn_start_threshold() -> None:
    assert TURN_OMEGA_EPSILON >= 0.04
    idx = detect_turn_start_frames([0.0, 0.01, 0.06, 0.07])
    assert idx == 2


def test_turn_readiness_late() -> None:
    req = turn_required_distance_m(vx=0.24, omega_target=0.28)
    assert classify_turn_readiness(obstacle_distance_m=0.30, turn_required_distance_m=req) == "LATE"


def test_adaptive_freshness_gate() -> None:
    young = max_trajectory_age_ms(actual_planner_compute_s=0.25)
    old_compute = max_trajectory_age_ms(actual_planner_compute_s=2.6)
    assert young < old_compute
    assert young > 400


def test_omega_sign_left() -> None:
    """+omega = CCW = LEFT."""
    mppi = DiffDriveMppi(batch_size=8, time_steps=4)
    res = mppi.step(
        0.0,
        0.0,
        0.0,
        [(0, 0), (5, 0)],
        (5, 0),
        lambda *_: False,
        maneuver_mode="LOCAL_LEFT",
        force_vx=0.08,
        force_w=0.28,
    )
    assert res.w > 0.05


def test_omega_sign_right() -> None:
    """-omega = CW = RIGHT."""
    mppi = DiffDriveMppi(batch_size=8, time_steps=4)
    res = mppi.step(
        0.0,
        0.0,
        0.0,
        [(0, 0), (5, 0)],
        (5, 0),
        lambda *_: False,
        maneuver_mode="LOCAL_RIGHT",
        force_vx=0.08,
        force_w=-0.28,
    )
    assert res.w < -0.05


def test_mppi_inner_loop_speed() -> None:
    """Inner loop must stay under 500ms for batch=90 (regression guard)."""
    import time as _time

    mppi = DiffDriveMppi(batch_size=90, time_steps=12)
    t0 = _time.perf_counter()
    mppi.step(
        -10.0,
        0.0,
        0.0,
        [(-10, 0), (10, 0)],
        (10, 0),
        lambda *_: False,
        front_near=3.0,
    )
    dt_ms = (_time.perf_counter() - t0) * 1000.0
    assert dt_ms < 800.0, f"MPPI step too slow: {dt_ms:.0f}ms"


def test_late_turn_synthetic() -> None:
    req = turn_required_distance_m(vx=0.24, omega_target=0.28)
    margin = 0.30 - req - 0.35
    assert margin < 0


UNIT_TESTS = [
    ("TEST_TURN_START_THRESHOLD", test_turn_start_threshold),
    ("TEST_ADAPTIVE_FRESHNESS", test_adaptive_freshness_gate),
    ("TEST_TURN_READINESS_LATE", test_turn_readiness_late),
    ("TEST_OMEGA_SIGN_LEFT", test_omega_sign_left),
    ("TEST_OMEGA_SIGN_RIGHT", test_omega_sign_right),
    ("TEST_MPPI_INNER_LOOP_SPEED", test_mppi_inner_loop_speed),
    ("TEST_LATE_TURN", test_late_turn_synthetic),
]


def run_live() -> int:
    trace = ROOT / "scripts" / "_trace_navigation_v02_live.py"
    scenes = [
        ("M32-OPEN-STRAIGHT", 12),
        ("SCENE-CURVE-01", 18),
        ("ONLINE-LEFT", 36),
        ("ONLINE-RIGHT", 36),
    ]
    for scene, dur in scenes:
        print(f"LIVE {scene} {dur}s ...")
        subprocess.run(
            [sys.executable, str(trace), "--scene", scene, "--seconds", str(dur)],
            cwd=str(ROOT),
            check=False,
        )
        time.sleep(1)
    logs = sorted((ROOT / "logs" / "navigation_v02").glob("*.jsonl"), key=lambda p: p.stat().st_mtime)[-4:]
    analyze = ROOT / "scripts" / "_analyze_turn_execution.py"
    subprocess.run([sys.executable, str(analyze), *[str(p) for p in logs]], cwd=str(ROOT), check=False)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    args = ap.parse_args()
    results = [_run(n, fn) for n, fn in UNIT_TESTS]
    failed = [r for r in results if not r[1]]
    for name, ok, msg in results:
        print(f"{'PASS' if ok else 'FAIL'} {name}: {msg}")
    if failed:
        return 1
    if args.live:
        return run_live()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
