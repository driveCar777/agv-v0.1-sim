#!/usr/bin/env python3
"""M3.8.2 trajectory semantics / freshness / scene-reset regression."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_models import LocalMppiModel  # noqa: E402
from agv_bridge.nav_recovery import ACT_HISTORICAL_RETREAT, ACT_LOCAL_REVERSE  # noqa: E402
from agv_bridge.nav_trajectory_integrity import (  # noqa: E402
    TRAJ_KIND_BACKWARD_FUTURE,
    TRAJ_KIND_FORWARD_FUTURE,
    TRAJ_KIND_HISTORICAL_RETREAT,
    TRAJ_KIND_NONE,
    TRAJ_KIND_STALE,
    classify_display_source,
    classify_source_kind,
    control_eligible_for_kind,
    enrich_trajectory_metadata,
    max_reanchor_shift_m,
)

Vehicle = Dict[str, Any]


def _run(name: str, fn: Callable[[], None]) -> Tuple[str, bool, str]:
    try:
        fn()
        return name, True, "ok"
    except AssertionError as e:
        return name, False, str(e) or "assertion failed"
    except Exception as e:
        return name, False, f"{type(e).__name__}: {e}"


def _pr(source: str, poses: List[Dict[str, Any]]) -> SimpleNamespace:
    return SimpleNamespace(
        corridor={"source": source, "poses": poses, "status": "VALID", "valid": True},
        status="VALID",
        collision=False,
        min_clearance=1.0,
        poses=poses,
    )


def _forward_poses() -> List[Dict[str, Any]]:
    return [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.2, "y": 0.0, "yaw": 0.0}]


def _behind_poses() -> List[Dict[str, Any]]:
    return [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": -1.2, "y": 0.0, "yaw": 0.0}]


def _model_with_probe() -> LocalMppiModel:
    m = LocalMppiModel()
    m.scene_id = 7
    m.last_probe = SimpleNamespace(
        forward=_pr("FORWARD", _forward_poses()),
        left=_pr("LEFT", [{"x": 0.0, "y": 0.4, "yaw": 0.0}, {"x": 1.0, "y": 0.6, "yaw": 0.2}]),
        right=_pr("RIGHT", [{"x": 0.0, "y": -0.4, "yaw": 0.0}, {"x": 1.0, "y": -0.6, "yaw": -0.2}]),
        backward=_pr("BACKWARD", _behind_poses()),
    )
    return m


def test_trajectory_source_semantics() -> None:
    assert classify_source_kind("FORWARD") == TRAJ_KIND_FORWARD_FUTURE
    assert classify_source_kind("LEFT") == TRAJ_KIND_FORWARD_FUTURE
    assert classify_source_kind("RIGHT") == TRAJ_KIND_FORWARD_FUTURE
    assert classify_source_kind("BACKWARD") == TRAJ_KIND_BACKWARD_FUTURE
    assert classify_source_kind("HISTORICAL_RETREAT") == TRAJ_KIND_HISTORICAL_RETREAT
    assert control_eligible_for_kind(TRAJ_KIND_FORWARD_FUTURE) is True
    assert control_eligible_for_kind(TRAJ_KIND_BACKWARD_FUTURE) is False
    assert control_eligible_for_kind(TRAJ_KIND_HISTORICAL_RETREAT) is False

    m = _model_with_probe()
    m._publish_active_corridors(live_auth=False, auth_side=None, recovery=None)
    m.planner_cycle_id = 3
    m.planner_input_timestamp = 10.0
    m.planner_start_timestamp = 10.01
    m.planner_finish_timestamp = 10.05
    m._stamp_published_trajectories()
    pt = m.last_physical_trajectory or {}
    assert pt.get("trajectory_kind") == TRAJ_KIND_FORWARD_FUTURE, pt.get("trajectory_kind")
    assert pt.get("control_eligible") is True
    assert pt.get("visualization_only") is False
    assert m.last_retreat_trajectory is None

    rec = SimpleNamespace(
        action=ACT_HISTORICAL_RETREAT,
        retreat=SimpleNamespace(corridor={"source": "HISTORICAL_RETREAT", "poses": _behind_poses()}),
    )
    m._publish_active_corridors(live_auth=False, auth_side=None, recovery=rec)
    m._stamp_published_trajectories()
    assert m.last_physical_trajectory is None, "HISTORICAL_RETREAT must not enter physical_trajectory"
    rt = m.last_retreat_trajectory or {}
    assert rt.get("trajectory_kind") == TRAJ_KIND_HISTORICAL_RETREAT
    assert rt.get("control_eligible") is False
    assert rt.get("visualization_only") is True

    rec_b = SimpleNamespace(action=ACT_LOCAL_REVERSE, retreat=None)
    m._publish_active_corridors(live_auth=False, auth_side=None, recovery=rec_b)
    m._stamp_published_trajectories()
    assert m.last_physical_trajectory is None, "BACKWARD must not enter physical_trajectory"
    bt = m.last_retreat_trajectory or {}
    assert bt.get("trajectory_kind") == TRAJ_KIND_BACKWARD_FUTURE
    assert bt.get("control_eligible") is False


def test_stale_trajectory_reject() -> None:
    now = time.time()
    veh = {"x": 2.0, "y": 0.0, "angle": 0.0, "vx": 0.20}
    traj = {
        "source": "FORWARD",
        "trajectory_kind": TRAJ_KIND_FORWARD_FUTURE,
        "poses": [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 0.8, "y": 0.0, "yaw": 0.0}],
        "trajectory_generated_at": now,
        "planner_finish_timestamp": now,
        "planner_cycle_id": 1,
        "scene_id": 1,
    }
    out = enrich_trajectory_metadata(traj, vehicle=veh, now=now, current_scene_id=1, current_cycle_id=1)
    assert out.get("integrity_reject") == "STALE_TRAJECTORY", out.get("integrity_reject")
    assert out.get("control_eligible") is False
    assert out.get("trajectory_kind") == TRAJ_KIND_STALE
    assert not out.get("small_reanchor_applied")
    assert out.get("stale_reanchor_applied") in (False, None)
    raw = (out.get("integrity") or {}).get("raw_anchor_error_m")
    assert raw is not None and raw >= 1.9, raw


def test_reanchor_limit() -> None:
    cap = max_reanchor_shift_m(vehicle_speed_mps=0.20)
    assert cap <= 0.40, cap
    assert cap < 2.0, "2m-class shift must not be an allowed reanchor"
    now = time.time()
    veh = {"x": 0.12, "y": 0.0, "angle": 0.0, "vx": 0.50}
    traj = {
        "source": "FORWARD",
        "poses": [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": 0.0}],
        "trajectory_generated_at": now,
        "planner_cycle_id": 2,
        "scene_id": 1,
    }
    allowed = max_reanchor_shift_m(vehicle_speed_mps=0.50)
    out = enrich_trajectory_metadata(traj, vehicle=veh, now=now, current_scene_id=1, current_cycle_id=2)
    if 0.12 <= allowed:
        assert out.get("small_reanchor_applied") is True
        assert out.get("control_eligible") is True
        assert (out.get("integrity") or {}).get("anchor_error_m") is not None
        assert (out.get("integrity") or {}).get("anchor_error_m") < 0.05
    else:
        assert out.get("integrity_reject") == "STALE_TRAJECTORY"


def test_planner_timestamp() -> None:
    now = time.time()
    generated = now - 0.80
    veh = {"x": 0.0, "y": 0.0, "angle": 0.0, "vx": 0.10}
    traj = {
        "source": "FORWARD",
        "poses": _forward_poses(),
        "trajectory_generated_at": generated,
        "planner_finish_timestamp": generated,
        "sample_timestamp": now,
        "planner_cycle_id": 4,
        "scene_id": 1,
    }
    out = enrich_trajectory_metadata(traj, vehicle=veh, now=now, current_scene_id=1, current_cycle_id=4)
    age = (out.get("integrity") or {}).get("trajectory_age_ms")
    assert age is not None, "age missing"
    assert abs(age - 800.0) < 80.0, f"age must use planner completion, got {age} ms"
    missing = {
        "source": "FORWARD",
        "poses": _forward_poses(),
        "planner_cycle_id": 4,
        "scene_id": 1,
    }
    cal = enrich_trajectory_metadata(missing, vehicle=veh, now=now, current_scene_id=1, current_cycle_id=4)
    assert cal.get("timestamp_status") == "CALIBRATION_REQUIRED"
    assert cal.get("trajectory_timestamp") is None


def test_planner_cycle_id() -> None:
    m = _model_with_probe()
    m.scene_id = 3
    m.planner_cycle_id = 11
    m.planner_input_timestamp = 1.0
    m.planner_start_timestamp = 1.01
    m.planner_finish_timestamp = 1.08
    m._publish_active_corridors(live_auth=False, auth_side=None, recovery=None)
    m._stamp_published_trajectories()
    pt = m.last_physical_trajectory or {}
    assert pt.get("planner_cycle_id") == 11
    assert pt.get("trajectory_cycle_id") == 11
    now = time.time()
    bad = dict(pt)
    bad["trajectory_generated_at"] = now
    out = enrich_trajectory_metadata(bad, vehicle={"x": 0.0, "y": 0.0, "angle": 0.0, "vx": 0.1}, now=now, current_scene_id=3, current_cycle_id=99)
    assert out.get("integrity_reject") == "TRAJECTORY_CYCLE_MISMATCH"
    assert out.get("control_eligible") is False


def test_scene_reset_clear_corridor() -> None:
    m = _model_with_probe()
    m.last_physical_trajectory = {"source": "RIGHT", "poses": _forward_poses()}
    m.last_retreat_trajectory = {"source": "HISTORICAL_RETREAT", "poses": _behind_poses()}
    m.last_execution_corridor = SimpleNamespace()
    m.last_selected_trajectory = m.last_physical_trajectory
    m.planner_failure_reason = "MPPI_NO_FEASIBLE_TRAJECTORY"
    m.reset()
    assert m.last_physical_trajectory is None
    assert m.last_retreat_trajectory is None
    assert m.last_execution_corridor is None
    assert m.last_selected_trajectory is None
    assert m.planner_failure_reason == ""
    assert m.planner_cycle_id == 0


def test_scene_reset_clear_trajectory() -> None:
    m = _model_with_probe()
    rec = SimpleNamespace(
        action=ACT_HISTORICAL_RETREAT,
        retreat=SimpleNamespace(corridor={"source": "HISTORICAL_RETREAT", "poses": _behind_poses()}),
    )
    m._publish_active_corridors(live_auth=True, auth_side="RIGHT", recovery=rec)
    assert m.last_retreat_trajectory is not None
    m.reset()
    assert m.last_physical_trajectory is None
    assert m.last_retreat_trajectory is None
    m.scene_id = 9
    m.last_probe = SimpleNamespace(
        forward=_pr("FORWARD", _forward_poses()),
        left=None,
        right=None,
        backward=None,
    )
    m._publish_active_corridors(live_auth=False, auth_side=None, recovery=None)
    m.planner_finish_timestamp = time.time()
    m._stamp_published_trajectories()
    pt = m.last_physical_trajectory or {}
    assert pt.get("scene_id") == 9
    assert pt.get("source") in ("FORWARD", "LEFT", "RIGHT")


def test_scene_id_mismatch_reject() -> None:
    now = time.time()
    traj = {
        "source": "RIGHT",
        "poses": _forward_poses(),
        "trajectory_generated_at": now,
        "planner_cycle_id": 1,
        "scene_id": 4,
    }
    out = enrich_trajectory_metadata(
        traj,
        vehicle={"x": 0.0, "y": 0.0, "angle": 0.0, "vx": 0.1},
        now=now,
        current_scene_id=5,
        current_cycle_id=1,
    )
    assert out.get("integrity_reject") == "TRAJECTORY_STALE_SCENE"
    assert out.get("control_eligible") is False


def test_historical_not_future_display() -> None:
    snap = {
        "nav": {
            "physical_trajectory": None,
            "retreat_trajectory": {
                "trajectory_kind": TRAJ_KIND_HISTORICAL_RETREAT,
                "source": "HISTORICAL_RETREAT",
                "poses": _behind_poses(),
                "control_eligible": False,
                "visualization_only": True,
            },
        }
    }
    assert classify_display_source(snap) == TRAJ_KIND_HISTORICAL_RETREAT
    stuffed = {
        "nav": {
            "physical_trajectory": {
                "trajectory_kind": TRAJ_KIND_HISTORICAL_RETREAT,
                "poses": _behind_poses(),
                "control_eligible": False,
            }
        }
    }
    assert classify_display_source(stuffed) != TRAJ_KIND_FORWARD_FUTURE
    future = {
        "nav": {
            "physical_trajectory": {
                "trajectory_kind": TRAJ_KIND_FORWARD_FUTURE,
                "poses": _forward_poses(),
                "control_eligible": True,
            }
        }
    }
    assert classify_display_source(future) == TRAJ_KIND_FORWARD_FUTURE
    empty = {"nav": {"physical_trajectory": None, "retreat_trajectory": None, "local_path": []}}
    assert classify_display_source(empty) == TRAJ_KIND_NONE


def _jsonl_rows(path: Path) -> List[dict]:
    return [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip() and l.strip()[0] == "{"]


def _forensic_row(r: dict) -> dict:
    pt = r.get("physical_trajectory") if isinstance(r.get("physical_trajectory"), dict) else {}
    integ = pt.get("integrity") if isinstance(pt.get("integrity"), dict) else {}
    return {
        "seq": r.get("seq"),
        "scene": r.get("scene"),
        "source": r.get("trajectory_source") or pt.get("trajectory_source") or pt.get("source"),
        "kind": r.get("trajectory_kind") or pt.get("trajectory_kind"),
        "control_eligible": r.get("trajectory_control_eligible") if r.get("trajectory_control_eligible") is not None else pt.get("control_eligible"),
        "age_ms": r.get("trajectory_age_ms") if r.get("trajectory_age_ms") is not None else integ.get("trajectory_age_ms"),
        "shift_m": r.get("reanchor_shift_m") if r.get("reanchor_shift_m") is not None else integ.get("raw_anchor_error_m") or integ.get("anchor_error_m"),
        "scene_id": r.get("scene_id") if r.get("scene_id") is not None else pt.get("scene_id") or r.get("nav_scene_id"),
        "cycle_id": r.get("planner_cycle_id") if r.get("planner_cycle_id") is not None else pt.get("planner_cycle_id"),
        "direction_dot": r.get("direction_dot") if r.get("direction_dot") is not None else pt.get("direction_dot") or integ.get("direction_dot"),
        "direction_angle": r.get("direction_angle") if r.get("direction_angle") is not None else pt.get("direction_angle") or integ.get("direction_angle_deg"),
        "integrity_reject": r.get("integrity_reject") or pt.get("integrity_reject") or integ.get("integrity_reject"),
        "stale_reanchor_applied": r.get("trajectory_reanchored") or integ.get("stale_reanchor_applied"),
    }


def analyze_jsonl(path: Path) -> Dict[str, Any]:
    rows = [r for r in _jsonl_rows(path) if not r.get("event") or r.get("vehicle_pose")]
    forensic = [_forensic_row(r) for r in rows]
    kinds = {}
    sources = {}
    hist_as_physical = 0
    backward_as_physical = 0
    stale_reanchor = 0
    for f in forensic:
        k = str(f.get("kind") or "NONE")
        s = str(f.get("source") or "NONE")
        kinds[k] = kinds.get(k, 0) + 1
        sources[s] = sources.get(s, 0) + 1
        if k == TRAJ_KIND_HISTORICAL_RETREAT and f.get("control_eligible"):
            hist_as_physical += 1
        if s.upper() in ("BACKWARD", "HISTORICAL_RETREAT") and f.get("control_eligible"):
            backward_as_physical += 1
        if f.get("stale_reanchor_applied"):
            stale_reanchor += 1
    dots = [f.get("direction_dot") for f in forensic if f.get("direction_dot") is not None]
    ages = [f.get("age_ms") for f in forensic if f.get("age_ms") is not None]
    shifts = [f.get("shift_m") for f in forensic if f.get("shift_m") is not None]
    return {
        "file": path.name,
        "samples": len(rows),
        "kinds": kinds,
        "sources": sources,
        "hist_control_eligible_count": hist_as_physical,
        "backward_control_eligible_count": backward_as_physical,
        "stale_reanchor_count": stale_reanchor,
        "direction_dot_min": min(dots) if dots else None,
        "direction_dot_max": max(dots) if dots else None,
        "age_ms_max": max(ages) if ages else None,
        "shift_m_max": max(shifts) if shifts else None,
        "forensic_head": forensic[:8],
        "forensic_mid": forensic[len(forensic) // 2 : len(forensic) // 2 + 3] if forensic else [],
    }


def _ping(base: str) -> bool:
    import urllib.request

    try:
        with urllib.request.urlopen(f"{base}/api/state?lite=1", timeout=3) as resp:
            return resp.status == 200
    except Exception:
        return False


def _live_optional(base: str, seconds: float, out_dir: Path) -> Dict[str, Any]:
    if not _ping(base):
        return {"status": "NOT_RUN", "reason": "SIM_NOT_RUNNING", "base": base}
    sys.path.insert(0, str(ROOT / "scripts"))
    from _trace_navigation_v02_live import run_scene  # type: ignore
    from agv_bridge.nav_live_client import NavLiveClient
    from agv_bridge.sim_world import SimWorld

    client = NavLiveClient(base=base)
    world = SimWorld()
    out_dir.mkdir(parents=True, exist_ok=True)
    report: Dict[str, Any] = {"status": "RAN", "files": []}
    seq = 0
    for scene, dur in (("M32-OPEN-STRAIGHT", seconds), ("ONLINE-RIGHT", min(12.0, seconds)), ("ONLINE-BOTH-BLOCKED", 8.0)):
        rows, seq, setup, fname = run_scene(client, world, scene, dur, 5.0, seq)
        if not fname:
            report.setdefault("setup_fail", []).append({"scene": scene, "setup": setup})
            continue
        path = out_dir / fname
        with path.open("w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False, default=str) + "\n")
        summary = analyze_jsonl(path)
        summary["scene"] = scene
        report["files"].append(summary)
        if scene == "ONLINE-BOTH-BLOCKED":
            head = [r for r in rows if r.get("vehicle_pose") and not r.get("event")][:6]
            leaks = []
            for r in head:
                src = str(r.get("trajectory_source") or "").upper()
                kind = str(r.get("trajectory_kind") or "").upper()
                if src in ("RIGHT", "BACKWARD", "HISTORICAL_RETREAT") or "RETREAT" in kind or "BACKWARD" in kind:
                    leaks.append({"seq": r.get("seq"), "source": src, "kind": kind})
            report["both_head_leak"] = leaks
            report["both_head"] = [_forensic_row(r) for r in head]
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.8.2 trajectory semantics / freshness / scene reset")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--base", default="http://127.0.0.1:19999")
    ap.add_argument("--live-seconds", type=float, default=20.0)
    ap.add_argument("--jsonl", default="")
    args = ap.parse_args()

    tests = [
        ("TEST_TRAJECTORY_SOURCE_SEMANTICS", test_trajectory_source_semantics),
        ("TEST_STALE_TRAJECTORY_REJECT", test_stale_trajectory_reject),
        ("TEST_REANCHOR_LIMIT", test_reanchor_limit),
        ("TEST_PLANNER_TIMESTAMP", test_planner_timestamp),
        ("TEST_PLANNER_CYCLE_ID", test_planner_cycle_id),
        ("TEST_SCENE_RESET_CLEAR_CORRIDOR", test_scene_reset_clear_corridor),
        ("TEST_SCENE_RESET_CLEAR_TRAJECTORY", test_scene_reset_clear_trajectory),
        ("TEST_SCENE_ID_MISMATCH_REJECT", test_scene_id_mismatch_reject),
        ("TEST_HISTORICAL_NOT_FUTURE_DISPLAY", test_historical_not_future_display),
    ]
    results = [_run(n, fn) for n, fn in tests]
    failed = [r for r in results if not r[1]]

    print("=" * 40)
    print("AGV NAVIGATION V0.2 M3.8.2")
    print("=" * 40)
    print("\nREGRESSION")
    for name, ok, msg in results:
        print(f"  {name}: {'PASS' if ok else 'FAIL'}  {'' if ok else msg}")

    live_rep: Dict[str, Any] = {"status": "NOT_RUN"}
    if args.live:
        out_dir = ROOT / "logs" / "navigation_v02"
        live_rep = _live_optional(args.base, args.live_seconds, out_dir)
        print("\nLIVE")
        print(f"  status: {live_rep.get('status')}")
        if live_rep.get("reason"):
            print(f"  reason: {live_rep.get('reason')}")
        for f in live_rep.get("files") or []:
            print(f"  {f.get('scene')}: samples={f.get('samples')} kinds={f.get('kinds')} "
                  f"dot=[{f.get('direction_dot_min')},{f.get('direction_dot_max')}] "
                  f"age_max={f.get('age_ms_max')} shift_max={f.get('shift_m_max')} "
                  f"stale_reanchor={f.get('stale_reanchor_count')}")
        if live_rep.get("both_head_leak") is not None:
            print(f"  ONLINE-BOTH head leak: {live_rep.get('both_head_leak') or 'NONE'}")

    if args.jsonl:
        p = Path(args.jsonl)
        print("\nJSONL", analyze_jsonl(p))

    out_dir = ROOT / "logs" / "navigation_v02"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    report = {
        "phase": "M3.8.2",
        "regression": [{"name": n, "pass": ok, "detail": msg} for n, ok, msg in results],
        "live": live_rep,
    }
    report_path = out_dir / f"M3.8.2-TRAJECTORY-FORENSICS-{stamp}.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(f"\nFORENSICS: {report_path}")
    print("RESULT:", "FAIL" if failed else "PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
