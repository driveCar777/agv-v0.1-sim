#!/usr/bin/env python3
"""M3.8.3 command ownership / omega sign / trajectory-command consistency."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_command_ownership import (  # noqa: E402
    SRC_GLOBAL_PATH_TRACKER,
    SRC_LOCAL_AVOIDANCE,
    SRC_LOCAL_MPPI,
    classify_command_source,
    command_actual_consistency,
    omega_side,
    safety_direction_override,
    trajectory_command_consistency,
)


def _run(name: str, fn: Callable[[], None]) -> Tuple[str, bool, str]:
    try:
        fn()
        return name, True, "ok"
    except AssertionError as e:
        return name, False, str(e) or "assertion failed"
    except Exception as e:
        return name, False, f"{type(e).__name__}: {e}"


def _integrate(x: float, y: float, yaw: float, vx: float, w: float, dt: float, steps: int) -> Tuple[float, float, float]:
    """Same kinematics as sim_api_ext physics: x+=vx*cos(yaw)*dt, angle+=w*dt."""
    for _ in range(steps):
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw = (yaw + w * dt + math.pi) % (2 * math.pi) - math.pi
    return x, y, yaw


def test_omega_sign_left() -> None:
    x, y, yaw = _integrate(0.0, 0.0, 0.0, vx=0.20, w=+0.20, dt=0.05, steps=20)
    dyaw = yaw  # started at 0
    assert dyaw > 0.05, f"LEFT +omega must increase yaw, got {dyaw}"
    assert y > 0.0, f"LEFT +omega with +vx at yaw=0 must move +y, got y={y}"


def test_omega_sign_right() -> None:
    x, y, yaw = _integrate(0.0, 0.0, 0.0, vx=0.20, w=-0.20, dt=0.05, steps=20)
    assert yaw < -0.05, f"RIGHT -omega must decrease yaw, got {yaw}"
    assert y < 0.0, f"RIGHT -omega with +vx at yaw=0 must move -y, got y={y}"


def test_command_owner() -> None:
    left = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="LOCAL_LEFT",
        follow_path_source="MANEUVER",
        requested_vx=0.14,
        requested_omega=0.28,
        approved_vx=0.14,
        approved_omega=0.28,
    )
    assert left["command_source"] == SRC_LOCAL_AVOIDANCE, left
    hold = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="LOCAL_LEFT",
        follow_path_source="MANEUVER",
        requested_vx=0.14,
        requested_omega=0.35,
        approved_vx=0.14,
        approved_omega=0.35,
        physical_control_eligible=False,
    )
    assert hold["command_source"] == SRC_LOCAL_AVOIDANCE, hold
    recap = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="POST_TURN",
        follow_path_source="GLOBAL_PATH",
        tracking_local_plan=False,
        requested_vx=0.12,
        requested_omega=-0.10,
        approved_vx=0.12,
        approved_omega=-0.10,
    )
    assert recap["command_source"] == SRC_GLOBAL_PATH_TRACKER, recap
    glob = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="FORWARD_TRACK",
        follow_path_source="GLOBAL_PATH",
        tracking_local_plan=False,
        requested_vx=0.20,
        requested_omega=0.08,
        approved_vx=0.20,
        approved_omega=0.08,
        physical_control_eligible=False,
        local_plan_kinematic_valid=False,
    )
    assert glob["command_source"] == SRC_GLOBAL_PATH_TRACKER, glob
    assert glob["fallback"] == "STALE_LOCAL_TO_GLOBAL_TRACK", glob
    loc = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="FORWARD_TRACK",
        follow_path_source="LOCAL_PLAN",
        tracking_local_plan=True,
        requested_vx=0.20,
        requested_omega=-0.10,
        approved_vx=0.20,
        approved_omega=-0.10,
    )
    assert loc["command_source"] == SRC_LOCAL_MPPI, loc
    assert loc["command_source_module"]
    assert loc["command_write_trace"]
    stale_track = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="FORWARD_TRACK",
        follow_path_source="LOCAL_PLAN",
        tracking_local_plan=True,
        physical_control_eligible=False,
        requested_vx=0.17,
        requested_omega=0.08,
        approved_vx=0.17,
        approved_omega=0.08,
    )
    assert stale_track["command_source"] == SRC_LOCAL_MPPI, stale_track
    assert stale_track["fallback"] == "STALE_TRAJECTORY_STILL_TRACKED", stale_track


def test_requested_approved_command() -> None:
    assert safety_direction_override(-0.20, +0.08) is True
    assert safety_direction_override(-0.20, -0.08) is False
    assert safety_direction_override(-0.20, 0.0) is False
    mag = classify_command_source(
        nav_mode="tracking",
        requested_vx=0.2,
        requested_omega=-0.20,
        approved_vx=0.2,
        approved_omega=-0.08,
        safe_vx_reason="FRONT",
    )
    assert mag["safety_direction_override"] is False
    flip = classify_command_source(
        nav_mode="tracking",
        requested_vx=0.2,
        requested_omega=-0.20,
        approved_vx=0.2,
        approved_omega=+0.08,
    )
    assert flip["safety_direction_override"] is True
    assert flip["command_source"] == "SAFETY_LIMIT"


def test_trajectory_command_consistency() -> None:
    bad = trajectory_command_consistency(vx=0.2, omega=0.1, direction_dot=-1.0, trajectory_control_eligible=True)
    assert bad["command_trajectory_inconsistency"] is True
    ok = trajectory_command_consistency(vx=0.2, omega=0.1, direction_dot=1.0, trajectory_control_eligible=True)
    assert ok["command_trajectory_inconsistency"] is False
    stale = trajectory_command_consistency(vx=0.2, omega=0.1, direction_dot=-1.0, trajectory_control_eligible=False)
    assert "TRAJECTORY_NOT_CONTROL_ELIGIBLE" in stale["issues"]
    assert stale["command_trajectory_inconsistency"] is False  # cannot blame command vs ineligible viz


def test_stale_local_fallback() -> None:
    own = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="FORWARD_TRACK",
        follow_path_source="GLOBAL_PATH",
        tracking_local_plan=False,
        physical_control_eligible=False,
        local_plan_kinematic_valid=False,
        requested_vx=0.17,
        requested_omega=0.08,
        approved_vx=0.17,
        approved_omega=0.08,
    )
    assert own["command_source"] == SRC_GLOBAL_PATH_TRACKER
    assert own["fallback"] == "STALE_LOCAL_TO_GLOBAL_TRACK"
    assert omega_side(0.08) == "LEFT"


def test_global_tracker_fallback() -> None:
    own = classify_command_source(
        nav_mode="tracking",
        maneuver_mode="FORWARD_TRACK",
        follow_path_source="GLOBAL_PATH",
        tracking_local_plan=False,
        requested_omega=0.05,
        approved_omega=0.05,
        requested_vx=0.2,
        approved_vx=0.2,
    )
    assert own["command_source"] == SRC_GLOBAL_PATH_TRACKER
    cons = command_actual_consistency(approved_omega=0.08, actual_omega=0.07)
    assert cons["actuator_direction_mismatch"] is False
    cons_bad = command_actual_consistency(approved_omega=-0.08, actual_omega=0.08)
    assert cons_bad["actuator_direction_mismatch"] is True


def _ping(base: str) -> bool:
    import urllib.request

    try:
        with urllib.request.urlopen(f"{base}/api/state?lite=1", timeout=3) as resp:
            return resp.status == 200
    except Exception:
        return False


def _live(base: str, seconds: float, out_dir: Path) -> Dict[str, Any]:
    if not _ping(base):
        return {"status": "NOT_RUN", "reason": "SIM_NOT_RUNNING"}
    sys.path.insert(0, str(ROOT / "scripts"))
    from _trace_navigation_v02_live import run_scene  # type: ignore
    from agv_bridge.nav_live_client import NavLiveClient
    from agv_bridge.sim_world import SimWorld

    client = NavLiveClient(base=base)
    world = SimWorld()
    out_dir.mkdir(parents=True, exist_ok=True)
    report: Dict[str, Any] = {"status": "RAN", "files": [], "right_traj_left_vehicle": []}
    seq = 0
    online_s = max(42.0, float(seconds))
    open_s = max(12.0, min(float(seconds), 20.0))
    for scene, dur in (("M32-OPEN-STRAIGHT", open_s), ("ONLINE-LEFT", online_s), ("ONLINE-RIGHT", online_s)):
        rows, seq, setup, fname = run_scene(client, world, scene, dur, 5.0, seq)
        if not fname:
            report.setdefault("setup_fail", []).append({"scene": scene, "setup": setup})
            continue
        path = out_dir / fname
        with path.open("w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False, default=str) + "\n")
        samples = [r for r in rows if r.get("vehicle_pose") and not r.get("event")]
        owners: Dict[str, int] = {}
        mismatches = []
        sign_flip = 0
        for r in samples:
            src = str(r.get("command_source") or "NONE")
            owners[src] = owners.get(src, 0) + 1
            req = r.get("requested_omega")
            appr = r.get("approved_omega")
            act = r.get("actual_omega")
            if req is not None and appr is not None and safety_direction_override(float(req), float(appr)):
                sign_flip += 1
            traj_side = str(r.get("trajectory_side") or "")
            elig = r.get("trajectory_control_eligible")
            cmd_side = str(r.get("command_side") or omega_side(float(appr) if appr is not None else 0.0))
            if traj_side == "RIGHT" and cmd_side == "LEFT":
                mismatches.append(
                    {
                        "seq": r.get("seq"),
                        "scene": scene,
                        "trajectory_source": r.get("trajectory_source"),
                        "trajectory_side": traj_side,
                        "command_side": cmd_side,
                        "trajectory_kind": r.get("trajectory_kind"),
                        "control_eligible": elig,
                        "age_ms": r.get("trajectory_age_ms"),
                        "command_source": src,
                        "requested_omega": req,
                        "approved_omega": appr,
                        "actual_omega": act,
                        "fallback": r.get("command_fallback"),
                    }
                )
        last = samples[-1] if samples else {}
        omegas = [abs(float(r.get("approved_omega") or 0)) for r in samples]
        yaws = []
        for r in samples:
            pose = r.get("vehicle_pose") or {}
            if pose.get("yaw") is not None:
                yaws.append(float(pose["yaw"]))
        injects = [r for r in rows if r.get("event") == "ONLINE_OBSTACLE_INJECT"]
        fallbacks: Dict[str, int] = {}
        for r in samples:
            fb = str(r.get("command_fallback") or "NONE")
            fallbacks[fb] = fallbacks.get(fb, 0) + 1
        report["files"].append(
            {
                "scene": scene,
                "file": fname,
                "samples": len(samples),
                "command_sources": owners,
                "safety_direction_override_count": sign_flip,
                "max_abs_approved_omega": max(omegas) if omegas else 0.0,
                "yaw_min": min(yaws) if yaws else None,
                "yaw_max": max(yaws) if yaws else None,
                "inject_count": len(injects),
                "fallbacks": fallbacks,
                "right_traj_left_cmd": mismatches[:8],
                "last": {
                    "command_source": last.get("command_source"),
                    "follow_path_source": last.get("follow_path_source"),
                    "requested_omega": last.get("requested_omega"),
                    "approved_omega": last.get("approved_omega"),
                    "actual_omega": last.get("actual_omega"),
                    "trajectory_kind": last.get("trajectory_kind"),
                    "control_eligible": last.get("trajectory_control_eligible"),
                    "local_plan_status": last.get("local_plan_status"),
                    "viz_mismatch": last.get("visualization_control_mismatch"),
                },
            }
        )
        report["right_traj_left_vehicle"].extend(mismatches)
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.8.3 command ownership forensics")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--base", default="http://127.0.0.1:19999")
    ap.add_argument("--live-seconds", type=float, default=16.0)
    args = ap.parse_args()

    tests = [
        ("TEST_OMEGA_SIGN_LEFT", test_omega_sign_left),
        ("TEST_OMEGA_SIGN_RIGHT", test_omega_sign_right),
        ("TEST_COMMAND_OWNER", test_command_owner),
        ("TEST_REQUESTED_APPROVED_COMMAND", test_requested_approved_command),
        ("TEST_TRAJECTORY_COMMAND_CONSISTENCY", test_trajectory_command_consistency),
        ("TEST_STALE_LOCAL_FALLBACK", test_stale_local_fallback),
        ("TEST_GLOBAL_TRACKER_FALLBACK", test_global_tracker_fallback),
    ]
    results = [_run(n, fn) for n, fn in tests]
    failed = [r for r in results if not r[1]]
    print("=" * 41)
    print("M3.8.3 COMMAND OWNERSHIP FORENSICS")
    print("=" * 41)
    print("\nREGRESSION")
    for name, ok, msg in results:
        print(f"  {name}: {'PASS' if ok else 'FAIL'}  {'' if ok else msg}")

    live_rep: Dict[str, Any] = {"status": "NOT_RUN"}
    if args.live:
        out_dir = ROOT / "logs" / "navigation_v02"
        try:
            live_rep = _live(args.base, args.live_seconds, out_dir)
        except Exception as e:
            live_rep = {"status": "ERROR", "error": f"{type(e).__name__}: {e}"}
        print("\nLIVE", live_rep.get("status"), live_rep.get("error") or "")
        for f in live_rep.get("files") or []:
            print(
                f"  {f.get('scene')}: sources={f.get('command_sources')} fallbacks={f.get('fallbacks')} "
                f"max|ω|={f.get('max_abs_approved_omega')} yaw=[{f.get('yaw_min')},{f.get('yaw_max')}] "
                f"inject={f.get('inject_count')} last={f.get('last')} "
                f"right→left={len(f.get('right_traj_left_cmd') or [])} safety_flip={f.get('safety_direction_override_count')}"
            )

    out_dir = ROOT / "logs" / "navigation_v02"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = out_dir / f"M3.8.3-COMMAND-FORENSICS-{stamp}.json"
    path.write_text(
        json.dumps({"regression": results, "live": live_rep}, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )
    print("FORENSICS", path)
    print("RESULT:", "FAIL" if failed else "PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
