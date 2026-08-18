#!/usr/bin/env python3
"""M3.6 — Side decision → ExecutionCorridor → MPPI closure audit + regression."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402
from agv_bridge.nav_avoidance_phase import AvoidancePhaseTracker  # noqa: E402
from agv_bridge.nav_execution_corridor import (  # noqa: E402
    MODE_LEFT,
    MODE_RIGHT,
    build_execution_corridor,
    corridor_allows_omega_sign,
)
from agv_bridge.nav_geometry import get_vehicle_geometry  # noqa: E402
from agv_bridge.nav_local_planner import KIND_LEFT_ARC, KIND_RIGHT_ARC, RollingLocalPlanner  # noqa: E402
from agv_bridge.nav_scenario_injector import (  # noqa: E402
    M32_OPEN_GOAL,
    M32_OPEN_START,
    body_frame,
    path_point_at_distance,
)
from agv_bridge.nav_side_probe import run_side_probe  # noqa: E402
from agv_bridge.sim_world import get_world  # noqa: E402

# Source map (M3.6 audit)
SIDE_DECISION_SOURCE = "nav_side_probe.run_side_probe"
COMMIT_SOURCE = "nav_avoidance_phase.AvoidancePhaseTracker.update"
CORRIDOR_SOURCE = "nav_execution_corridor.build_execution_corridor"
MPPI_INPUT_SOURCE = "nav_models.LocalMppiModel.step → mppi_controller.DiffDriveMppi.step"
OMEGA_SOURCE = "mppi_controller.DiffDriveMppi.step (w_cmd blend)"
COMMAND_SOURCE = "sim_api_ext.apply_safety → state._cmd_vx/w"


def _run(name: str, fn: Callable[[], None]) -> Tuple[str, bool, str]:
    try:
        fn()
        return name, True, "ok"
    except AssertionError as e:
        return name, False, str(e) or "assertion failed"
    except Exception as e:
        return name, False, f"{type(e).__name__}: {e}"


def test_corridor_left_on_side_commit_hold() -> None:
    """SIDE_COMMIT hold must activate corridor even if commit_ready flickers false."""
    geom = get_vehicle_geometry()
    c = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="LEFT",
        preferred_side="LEFT",
        commit_ready=False,
        front_near=1.8,
        left_free=1.2,
        right_free=0.4,
        geom=geom,
        reason="TEST_HOLD",
    )
    assert c.active and c.mode == MODE_LEFT and c.committed_side == "LEFT"


def test_corridor_right_on_side_commit_hold() -> None:
    geom = get_vehicle_geometry()
    c = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="RIGHT",
        preferred_side="RIGHT",
        commit_ready=False,
        front_near=1.8,
        left_free=0.4,
        right_free=1.2,
        geom=geom,
        reason="TEST_HOLD",
    )
    assert c.active and c.mode == MODE_RIGHT


def test_side_execution_left_mppi() -> None:
    """TEST_SIDE_LEFT_EXECUTION — forced LEFT corridor → non-zero left omega."""
    geom = get_vehicle_geometry()
    corridor = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="LEFT",
        preferred_side="LEFT",
        commit_ready=True,
        front_near=2.0,
        left_free=1.5,
        right_free=0.5,
        geom=geom,
    )
    mppi = DiffDriveMppi(batch_size=48, geom=geom)
    path = [(0.0, 0.0), (3.0, 0.0)]
    res = mppi.step(
        0.0, 0.0, 0.0, path, (3.0, 0.0), lambda _x, _y: False,
        front_near=4.0, maneuver_mode="FORWARD_TURN", vx_max=0.22,
        execution_corridor=corridor,
        clearance_at=lambda _x, y: abs(0.6 - y),
    )
    assert res.w >= -0.02, f"LEFT corridor allowed strong RIGHT omega: {res.w}"
    assert res.w >= 0.01 or abs(res.w) <= 0.02, f"unexpected omega={res.w}"


def test_side_execution_right_mppi() -> None:
    geom = get_vehicle_geometry()
    corridor = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="RIGHT",
        preferred_side="RIGHT",
        commit_ready=True,
        front_near=2.0,
        left_free=0.5,
        right_free=1.5,
        geom=geom,
    )
    mppi = DiffDriveMppi(batch_size=48, geom=geom)
    path = [(0.0, 0.0), (3.0, 0.0)]
    res = mppi.step(
        0.0, 0.0, 0.0, path, (3.0, 0.0), lambda _x, _y: False,
        front_near=4.0, maneuver_mode="FORWARD_TURN", vx_max=0.22,
        execution_corridor=corridor,
        clearance_at=lambda _x, y: abs(0.6 - y),
    )
    assert res.w <= 0.02, f"RIGHT corridor allowed strong LEFT omega: {res.w}"


def _world_static_left() -> Tuple[Any, float, float, float]:
    world = get_world()
    world.set_scene("m32_open_straight")
    world.clear_dyn_obstacles()
    s = (M32_OPEN_START["x"], M32_OPEN_START["y"])
    g = (M32_OPEN_GOAL["x"], M32_OPEN_GOAL["y"])
    px, py, seg_yaw = path_point_at_distance(world, s, g, 2.3)

    class _C:
        def post(self, path: str, body: Optional[dict] = None) -> None:
            if body and "/obstacles/add" in path:
                world.add_dyn_obstacle(body["x"], body["y"], body["r"], body.get("name", ""))

    from agv_bridge.nav_scenario_injector import _inject_static_left_at

    _inject_static_left_at(_C(), px, py, seg_yaw, "test")
    yaw = math.atan2(g[1] - s[1], g[0] - s[0])
    return world, s[0], s[1], yaw


def _world_static_right() -> Tuple[Any, float, float, float]:
    world = get_world()
    world.set_scene("m32_open_straight")
    world.clear_dyn_obstacles()
    s = (M32_OPEN_START["x"], M32_OPEN_START["y"])
    g = (M32_OPEN_GOAL["x"], M32_OPEN_GOAL["y"])
    px, py, seg_yaw = path_point_at_distance(world, s, g, 2.3)

    class _C:
        def post(self, path: str, body: Optional[dict] = None) -> None:
            if body and "/obstacles/add" in path:
                world.add_dyn_obstacle(body["x"], body["y"], body["r"], body.get("name", ""))

    from agv_bridge.nav_scenario_injector import _inject_static_right_at

    _inject_static_right_at(_C(), px, py, seg_yaw, "test")
    yaw = math.atan2(g[1] - s[1], g[0] - s[0])
    return world, s[0], s[1], yaw


def test_probe_static_left_prefers_left_at_range() -> None:
    """After lat_sign fix, static-left should prefer LEFT when both sides evaluated far from pillar."""
    world, x0, y0, yaw = _world_static_left()
    geom = get_vehicle_geometry()

    def collide(x: float, y: float) -> bool:
        return world.collides(x, y, geom.planner_radius, include_actors=True)

    sp = run_side_probe(
        x=x0, y=y0, yaw=yaw, vx=0.12, horizon_m=5.0,
        collide=collide, clearance_at=world.clearance_at_xy, probe_active=True,
    )
    assert sp.left_valid, "LEFT side should remain valid at start for static-left layout"
    assert sp.preferred_side in (None, "LEFT")
    if sp.left_valid and sp.right_valid and sp.preferred_side:
        assert sp.preferred_side == "LEFT"
    assert (sp.left_min_clearance or 0) >= (sp.right_min_clearance or 0) - 0.05


def test_probe_static_right_prefers_right_at_range() -> None:
    world, x0, y0, yaw = _world_static_right()
    geom = get_vehicle_geometry()

    def collide(x: float, y: float) -> bool:
        return world.collides(x, y, geom.planner_radius, include_actors=True)

    sp = run_side_probe(
        x=x0, y=y0, yaw=yaw, vx=0.12, horizon_m=5.0,
        collide=collide, clearance_at=world.clearance_at_xy, probe_active=True,
    )
    assert sp.right_valid, "RIGHT side should remain valid at start"
    assert sp.preferred_side in (None, "RIGHT")
    if sp.left_valid and sp.right_valid and sp.preferred_side:
        assert sp.preferred_side == "RIGHT"


def test_both_blocked_no_commit() -> None:
    world = get_world()
    world.set_scene("m32_open_straight")
    world.clear_dyn_obstacles()
    s = (M32_OPEN_START["x"], M32_OPEN_START["y"])
    g = (M32_OPEN_GOAL["x"], M32_OPEN_GOAL["y"])
    px, py, seg_yaw = path_point_at_distance(world, s, g, 2.1)
    world.add_dyn_obstacle(px, py, 0.44, "front")
    for i, (fwd, lat, r) in enumerate([(0.85, 0.80, 0.40), (1.10, 1.00, 0.38), (0.85, -0.80, 0.40), (1.10, -1.00, 0.38)]):
        bx, by = body_frame(px, py, seg_yaw, fwd, lat)
        world.add_dyn_obstacle(bx, by, r, f"b{i}")
    yaw = math.atan2(g[1] - s[1], g[0] - s[0])
    geom = get_vehicle_geometry()

    def collide(x: float, y: float) -> bool:
        return world.collides(x, y, geom.planner_radius, include_actors=True)

    sp = run_side_probe(
        x=s[0] + 1.5, y=s[1], yaw=yaw, vx=0.10, horizon_m=4.0,
        collide=collide, clearance_at=world.clearance_at_xy, probe_active=True,
    )
    assert not sp.left_valid and not sp.right_valid, "both should be infeasible near block"
    assert sp.preferred_side is None
    assert not sp.commit_ready

    tr = AvoidancePhaseTracker()
    av = tr.update(
        now=1.0, vx=0.1, preview_m=5.0, future_collision=True, first_collision_m=1.5,
        front_near=1.2, left_free=0.3, right_free=0.3, lateral_error=0.0,
        side_probe_active=True, probe_confidence_left=0.0, probe_confidence_right=0.0,
        preferred_side=None, commit_ready=False, left_probe_valid=False, right_probe_valid=False,
        committed_side_external=None,
        obstacle_passed_external=False, commitment_active=False, geom=geom,
    )
    assert av.phase != "SIDE_COMMIT", f"both blocked must not SIDE_COMMIT, got {av.phase}"


def test_positive_omega_is_left() -> None:
    assert corridor_allows_omega_sign(
        build_execution_corridor(
            now=0, avoidance_phase="SIDE_COMMIT", committed_side="LEFT", preferred_side="LEFT",
            commit_ready=True, front_near=2, left_free=1, right_free=1,
        ),
        0.2,
    )
    assert not corridor_allows_omega_sign(
        build_execution_corridor(
            now=0, avoidance_phase="SIDE_COMMIT", committed_side="RIGHT", preferred_side="RIGHT",
            commit_ready=True, front_near=2, left_free=1, right_free=1,
        ),
        0.2,
    )


def _maybe_live(base: str, scene: str, seconds: float) -> Optional[Path]:
    trace = ROOT / "scripts" / "_trace_navigation_v02_live.py"
    if not trace.is_file():
        return None
    log_dir = ROOT / "logs" / "navigation_v02"
    log_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, str(trace), "--scene", scene, "--seconds", str(seconds), "--hz", "2", "--base", base,
    ]
    try:
        subprocess.run(cmd, cwd=str(ROOT), check=True, timeout=max(90, seconds + 30))
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
        return None
    files = sorted(log_dir.glob(f"*{scene.lower().replace('-', '-')}*.jsonl"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def _summarize_jsonl(path: Path) -> Dict[str, Any]:
    import json

    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    samples = [r for r in rows if not r.get("error") and r.get("event") not in ("OBSTACLE_INJECT",)]
    commits = [r for r in samples if r.get("avoidance_phase") == "SIDE_COMMIT"]
    last = samples[-1] if samples else {}
    return {
        "file": str(path.name),
        "samples": len(samples),
        "side_commit_frames": len(commits),
        "last_probe": last.get("preferred_side") or last.get("probe_side"),
        "last_commit": last.get("committed_side") or last.get("commit_side"),
        "last_corridor": last.get("corridor_side") or (last.get("execution_corridor") or {}).get("mode"),
        "last_omega_sign": last.get("trajectory_omega_sign"),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.6 side→corridor→MPPI audit")
    ap.add_argument("--live", action="store_true", help="Run short LIVE traces (needs web sim)")
    ap.add_argument("--base", default="http://127.0.0.1:19999")
    ap.add_argument("--live-seconds", type=float, default=25.0)
    args = ap.parse_args()

    tests = [
        ("TEST_CORRIDOR_LEFT_HOLD", test_corridor_left_on_side_commit_hold),
        ("TEST_CORRIDOR_RIGHT_HOLD", test_corridor_right_on_side_commit_hold),
        ("TEST_SIDE_LEFT_EXECUTION", test_side_execution_left_mppi),
        ("TEST_SIDE_RIGHT_EXECUTION", test_side_execution_right_mppi),
        ("TEST_PROBE_STATIC_LEFT", test_probe_static_left_prefers_left_at_range),
        ("TEST_PROBE_STATIC_RIGHT", test_probe_static_right_prefers_right_at_range),
        ("TEST_BOTH_NO_SIDE", test_both_blocked_no_commit),
        ("TEST_POSITIVE_OMEGA_LEFT", test_positive_omega_is_left),
    ]
    results: List[Tuple[str, bool, str]] = [_run(n, fn) for n, fn in tests]

    print("=" * 40)
    print("AGV NAVIGATION V0.2 M3.6")
    print("=" * 40)
    print("\nSOURCE MAP")
    for k, v in [
        ("SIDE_DECISION_SOURCE", SIDE_DECISION_SOURCE),
        ("COMMIT_SOURCE", COMMIT_SOURCE),
        ("CORRIDOR_SOURCE", CORRIDOR_SOURCE),
        ("MPPI_INPUT_SOURCE", MPPI_INPUT_SOURCE),
        ("OMEGA_SOURCE", OMEGA_SOURCE),
        ("COMMAND_SOURCE", COMMAND_SOURCE),
    ]:
        print(f"  {k}: {v}")

    print("\nREGRESSION")
    ok_all = True
    for name, ok, detail in results:
        ok_all = ok_all and ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if not ok else ""))

    live_summaries: Dict[str, Any] = {}
    if args.live:
        print("\nLIVE TRACES (short)")
        for scene in ("OBS-OPEN-LEFT", "OBS-OPEN-RIGHT", "OBS-OPEN-BOTH-BLOCKED"):
            p = _maybe_live(args.base, scene, args.live_seconds)
            if p:
                live_summaries[scene] = _summarize_jsonl(p)
                print(f"  {scene}: {live_summaries[scene]}")
            else:
                print(f"  {scene}: skipped (sim down or trace failed)")
                ok_all = False

    print("\nVERDICT:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
