#!/usr/bin/env python3
"""M3 audit — trace schema + analyzer smoke tests (no LIVE required)."""

from __future__ import annotations

import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

REQUIRED_TRACE_FIELDS = (
    "ts", "seq", "schema", "vehicle_pose", "vehicle_vx", "vehicle_omega",
    "nav_state", "avoidance_phase", "committed_side", "execution_corridor",
    "footprint_clearance_m", "predicted_min_clearance_m",
    "candidate_count", "valid_candidate_count", "collision_rejected_count",
    "mppi_vx", "requested_vx", "approved_vx", "safe_vx_reason",
    "planner_state", "recovery_state", "recovery_attempt", "nav_ui_severity",
)


def test_trace_script_fields() -> None:
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "trace", ROOT / "scripts" / "_trace_navigation_v02_live.py"
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    src = inspect.getsource(mod._sample_v02)
    for f in REQUIRED_TRACE_FIELDS:
        assert f'"{f}"' in src, f"missing field in trace sampler: {f}"


def test_analyzer_imports() -> None:
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "analyze", ROOT / "scripts" / "_analyze_navigation_v02_trace.py"
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    assert hasattr(mod, "analyze")
    assert hasattr(mod, "MILESTONES")


def test_braking_recorder_exists() -> None:
    p = ROOT / "scripts" / "_record_braking_calibration.py"
    assert p.is_file()


def main() -> int:
    tests = [
        ("TEST_TRACE_SCHEMA_FIELDS", test_trace_script_fields),
        ("TEST_ANALYZER", test_analyzer_imports),
        ("TEST_BRAKING_RECORDER", test_braking_recorder_exists),
    ]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS {name}")
        except Exception as exc:
            failed += 1
            print(f"  FAIL {name}: {exc}")
    print(f"\nRESULT: {'PASS' if failed == 0 else 'FAIL'} ({failed} failed)")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
