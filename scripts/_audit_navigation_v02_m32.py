#!/usr/bin/env python3
"""M3.2 Baseline Validity Isolation — static MPPI/geometry audits + optional LIVE.

Answers: can V0.2 produce valid footprint-safe trajectories in open space with
no recovery, before any obstacle validation?
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
import urllib.error
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws" / "src" / "agv_bridge"
sys.path.insert(0, str(BRIDGE))

from agv_bridge.mppi_controller import DiffDriveMppi  # noqa: E402
from agv_bridge.nav_footprint import (  # noqa: E402
    current_footprint_clearance,
    footprint_polygon_body,
    trajectory_collision,
    transform_footprint,
)
from agv_bridge.nav_geometry import DEFAULT_GEOM, get_vehicle_model  # noqa: E402
from agv_bridge.nav_scenario_injector import M32_OPEN_GOAL, M32_OPEN_START  # noqa: E402
from agv_bridge.nav_trajectory import TrajectorySample  # noqa: E402
from agv_bridge.nav_trajectory_validator import validate_trajectory  # noqa: E402
from agv_bridge.sim_world import SimWorld  # noqa: E402

Pt = Tuple[float, float]
LOG_DIR = ROOT / "logs" / "navigation_v02"


@dataclass
class LayerResult:
    name: str
    passed: bool
    detail: str = ""
    data: Dict[str, Any] = field(default_factory=dict)


def _dd_poses(vx: float, w: float, n: int = 20, dt: float = 0.1, x0: float = 0.0, y0: float = 0.0, yaw0: float = 0.0) -> List[TrajectorySample]:
    x, y, yaw = x0, y0, yaw0
    out = [TrajectorySample(0.0, x, y, yaw, vx, w)]
    for i in range(n):
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += w * dt
        out.append(TrajectorySample((i + 1) * dt, x, y, yaw, vx, w))
    return out


def _world_fns(world: SimWorld, geom=DEFAULT_GEOM):
    def collide(x: float, y: float) -> bool:
        return world.collides(x, y, robot_r=geom.safety_radius, include_actors=False)

    def clearance_at(x: float, y: float) -> float:
        return world.clearance_at_xy(x, y, search_m=4.0)

    return collide, clearance_at


def _path_min_clearance(world: SimWorld, path: List[Pt]) -> float:
    if not path:
        return 0.0
    return min(world.clearance_at_xy(p[0], p[1], search_m=5.0) for p in path)


def _future_collision_m(world: SimWorld, x: float, y: float, yaw: float, path: List[Pt], geom=DEFAULT_GEOM) -> Optional[float]:
    """First distance along global path where center collides (inflated map)."""
    if len(path) < 2:
        return None
    acc = 0.0
    for i in range(len(path)):
        if world.collides(path[i][0], path[i][1], robot_r=geom.planner_radius, include_actors=False):
            return acc
        if i > 0:
            acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    return None


def audit_vehicle_model_straight() -> LayerResult:
    geom = DEFAULT_GEOM
    vm = get_vehicle_model()
    poly = footprint_polygon_body(geom)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    ok = len(poly) == 4 and max(xs) - min(xs) >= 0.9 and max(ys) - min(ys) >= 0.5
    return LayerResult(
        "VehicleModel straight",
        ok,
        f"body frame +x front; polygon {len(poly)} corners; center_offset=({geom.center_offset_x_m},{geom.center_offset_y_m})",
        {
            "length_m": geom.length,
            "width_m": geom.width,
            "bumper_l_m": geom.bumper_l,
            "base_link_offset": (geom.base_link_offset_x_m, geom.base_link_offset_y_m),
            "model_source": vm.to_dict() if hasattr(vm, "to_dict") else {},
        },
    )


def audit_footprint_straight() -> LayerResult:
    geom = DEFAULT_GEOM
    poses = _dd_poses(0.15, 0.0, n=15)
    res = trajectory_collision(poses, lambda _x, _y: False, geom, margin_m=0.0)
    end = poses[-1]
    progressed = end.x > 0.15
    ok = not res.collision and progressed
    return LayerResult(
        "Footprint straight",
        ok,
        f"straight rollout x_end={end.x:.2f} collision={res.collision}",
        {"first_collision_pose": res.first_pose, "progress_m": end.x},
    )


def audit_kinematic_turn() -> LayerResult:
    geom = DEFAULT_GEOM
    vx, w = 0.15, 0.12
    poses = _dd_poses(vx, w, n=18)
    res = trajectory_collision(poses, lambda _x, _y: False, geom, margin_m=0.0)
    r_expected = abs(vx / w) if abs(w) > 1e-6 else float("inf")
    end = poses[-1]
    ok = not res.collision and end.x > 0.15 and abs(end.y) > 0.02
    return LayerResult(
        "Kinematic turn",
        ok,
        f"R_expected={r_expected:.2f}m end=({end.x:.2f},{end.y:.2f})",
        {"collision": res.collision, "omega": w, "vx": vx},
    )


def audit_trajectory_validator_open(world: SimWorld) -> LayerResult:
    geom = DEFAULT_GEOM
    collide, clearance_at = _world_fns(world, geom)
    sx, sy = M32_OPEN_START["x"], M32_OPEN_START["y"]
    poses = _dd_poses(0.12, 0.0, n=12, x0=sx, y0=sy, yaw0=0.0)
    vr = validate_trajectory(
        poses,
        collide=collide,
        clearance_at=clearance_at,
        geom=geom,
        margin_m=geom.safety_margin_m,
        dt=0.1,
        execution_corridor=None,
    )
    ok = vr.valid
    return LayerResult(
        "TrajectoryValidator",
        ok,
        f"reason={vr.reason} min_clr={vr.minimum_clearance.minimum_clearance_m}",
        vr.to_dict(),
    )


def _mppi_rejection_breakdown(meta: dict, cand: int) -> Dict[str, Any]:
    cc = int(meta.get("collision_rejected_count") or 0)
    cl = int(meta.get("clearance_rejected_count") or 0)
    cn = int(meta.get("constraint_rejected_count") or 0)
    vc = int(meta.get("valid_candidate_count") or 0)
    total_rej = cc + cl + cn
    if cand <= 0:
        dominant = "NO_CANDIDATES_GENERATED"
    elif vc > 0:
        dominant = "NONE"
    elif total_rej == 0:
        dominant = "UNKNOWN"
    else:
        parts = {"COLLISION": cc, "CLEARANCE": cl, "CONSTRAINT": cn}
        mx = max(parts.values())
        top = [k for k, v in parts.items() if v == mx]
        dominant = top[0] if len(top) == 1 else "MIXED"
    return {
        "ALL_REJECTED_BECAUSE": dominant,
        "collision_rejected_count": cc,
        "clearance_rejected_count": cl,
        "constraint_rejected_count": cn,
        "valid_candidate_count": vc,
        "candidate_count": cand,
    }


def audit_mppi_static(world: SimWorld, label: str, start: Pt, goal: Pt, yaw: float) -> LayerResult:
    geom = DEFAULT_GEOM
    collide, clearance_at = _world_fns(world, geom)
    path = world.plan_path(start, goal, robot_r=geom.planner_radius)
    if not path:
        return LayerResult(f"MPPI static ({label})", False, "no global path")
    random.seed(42)
    mppi = DiffDriveMppi(batch_size=64, geom=geom)
    dual = world.dual_lidar(start[0], start[1], yaw, step_deg=4.0)
    front_near = float(dual.get("front_near", 30.0))
    fp_clr = current_footprint_clearance({"x": start[0], "y": start[1], "yaw": yaw}, clearance_at, geom).minimum_clearance_m
    res = mppi.step(
        start[0],
        start[1],
        yaw,
        path,
        goal,
        collide,
        front_near=front_near,
        target_vx=0.20,
        maneuver_mode="FORWARD_TRACK",
        clearance_at=clearance_at,
        execution_corridor=None,
    )
    meta = getattr(mppi, "_last_meta", None) or {}
    breakdown = _mppi_rejection_breakdown(meta, int(meta.get("candidate_count") or 0))
    ok = breakdown["valid_candidate_count"] > 0 and abs(res.vx) > 0.01
    return LayerResult(
        f"MPPI static ({label})",
        ok,
        f"vx={res.vx:.3f} w={res.w:.3f} {breakdown['ALL_REJECTED_BECAUSE']}",
        {
            **breakdown,
            "selected_vx": res.vx,
            "selected_omega": res.w,
            "front_near_m": front_near,
            "footprint_clearance_m": fp_clr,
            "clearance_reference": "map_occupied_inflated nearest cell to footprint sample points",
            "path_waypoints": len(path),
            "path_min_clearance_m": round(_path_min_clearance(world, path), 3),
            "future_collision_along_path_m": _future_collision_m(world, start[0], start[1], yaw, path),
            "failure_reason": meta.get("failure_reason"),
            "predicted_min_clearance_m": meta.get("predicted_min_clearance_m"),
        },
    )


def audit_geometry_compare() -> LayerResult:
    """Indoor spawn vs M32 open — explains M3.1 baseline invalidity."""
    from agv_bridge.nav_scenario_injector import BASELINE_GOAL, BASELINE_START

    indoor = SimWorld()
    open_w = SimWorld()
    open_w.load_m32_open_straight()
    geom = DEFAULT_GEOM
    ix, iy = BASELINE_START["x"], BASELINE_START["y"]
    gx, gy = BASELINE_GOAL["x"], BASELINE_GOAL["y"]
    iyaw = math.atan2(gy - iy, gx - ix)
    ipath = indoor.plan_path((ix, iy), (gx, gy), robot_r=geom.planner_radius) or []
    ox, oy = M32_OPEN_START["x"], M32_OPEN_START["y"]
    mx, my = M32_OPEN_GOAL["x"], M32_OPEN_GOAL["y"]
    opath = open_w.plan_path((ox, oy), (mx, my), robot_r=geom.planner_radius) or []
    idual = indoor.dual_lidar(ix, iy, iyaw, step_deg=4.0)
    odual = open_w.dual_lidar(ox, oy, 0.0, step_deg=4.0)
    data = {
        "indoor": {
            "front_near_m": round(float(idual["front_near"]), 3),
            "footprint_clearance_m": round(indoor.clearance_at_xy(ix, iy), 3),
            "future_collision_m": _future_collision_m(indoor, ix, iy, iyaw, ipath),
            "path_min_clearance_m": round(_path_min_clearance(indoor, ipath), 3),
        },
        "m32_open": {
            "front_near_m": round(float(odual["front_near"]), 3),
            "footprint_clearance_m": round(open_w.clearance_at_xy(ox, oy), 3),
            "future_collision_m": _future_collision_m(open_w, ox, oy, 0.0, opath),
            "path_min_clearance_m": round(_path_min_clearance(open_w, opath), 3),
        },
    }
    return LayerResult(
        "Geometry compare indoor vs open",
        data["m32_open"]["path_min_clearance_m"] > 2.0,
        json.dumps(data, ensure_ascii=False),
        data,
    )


def audit_execution_corridor_baseline() -> LayerResult:
    """Baseline must not activate LEFT/RIGHT detour corridor."""
    from agv_bridge.nav_execution_corridor import build_execution_corridor

    geom = DEFAULT_GEOM
    ec = build_execution_corridor(
        now=1.0,
        avoidance_phase="FORWARD",
        committed_side=None,
        preferred_side=None,
        commit_ready=False,
        front_near=30.0,
        left_free=5.0,
        right_free=5.0,
        geom=geom,
        reason="M32_BASELINE",
    )
    ok = not ec.active or ec.mode in ("FORWARD", "NONE", "")
    return LayerResult(
        "Execution corridor baseline",
        ok,
        f"active={ec.active} mode={ec.mode}",
        ec.to_dict() if hasattr(ec, "to_dict") else {"mode": ec.mode, "active": ec.active},
    )


def run_live_m32(base: str, seconds: float, hz: float) -> LayerResult:
    try:
        import urllib.request

        url = base.rstrip("/") + "/api/state"
        with urllib.request.urlopen(url, timeout=5) as resp:
            resp.read()
    except Exception as exc:
        return LayerResult("Open-space LIVE", False, f"SIM_NOT_RUNNING: {exc}", {"run": False})

    trace = ROOT / "scripts" / "_trace_navigation_v02_live.py"
    import subprocess

    rc = subprocess.call(
        [sys.executable, str(trace), "--scene", "M32-OPEN-STRAIGHT", "--seconds", str(seconds), "--hz", str(hz), "--base", base],
    )
    if rc != 0:
        return LayerResult("Open-space LIVE", False, f"trace rc={rc}")

    files = sorted(LOG_DIR.glob("M32-OPEN-STRAIGHT-*.jsonl"), key=os.path.getmtime)
    if not files:
        return LayerResult("Open-space LIVE", False, "no JSONL written")
    path = files[-1]
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    samples = [r for r in rows if r.get("vehicle_vx") is not None and not r.get("error")]
    if not samples:
        return LayerResult("Open-space LIVE", False, "empty trace")

    first = samples[0]
    last = samples[-1]
    max_vx = max(abs(float(r.get("vehicle_vx") or 0)) for r in samples)
    pose0 = first.get("vehicle_pose") or {}
    pose1 = last.get("vehicle_pose") or {}
    dist0 = math.hypot(float(M32_OPEN_GOAL["x"]) - float(pose0.get("x") or 0), float(M32_OPEN_GOAL["y"]) - float(pose0.get("y") or 0))
    dist1 = math.hypot(float(M32_OPEN_GOAL["x"]) - float(pose1.get("x") or 0), float(M32_OPEN_GOAL["y"]) - float(pose1.get("y") or 0))
    progress = dist0 - dist1
    recovery_at_t0 = str(first.get("planner_state") or "") == "LOCAL_RECOVERY" or str(first.get("stop_reason") or "") == "RECOVERY"
    ok = (
        not recovery_at_t0
        and max_vx > 0.04
        and progress > 0.5
        and str(last.get("planner_state") or "") not in ("NAVIGATION_FAILED",)
    )
    return LayerResult(
        "Open-space LIVE",
        ok,
        f"max_vx={max_vx:.3f} progress={progress:.2f}m last_stop={last.get('stop_reason')}",
        {
            "jsonl": str(path),
            "T0_planner_state": first.get("planner_state"),
            "T0_stop_reason": first.get("stop_reason"),
            "requested_vx": last.get("requested_vx"),
            "mppi_vx": last.get("mppi_vx"),
            "approved_vx": last.get("approved_vx"),
            "safe_vx_reason": last.get("safe_vx_reason"),
            "initial_goal_dist_m": round(dist0, 3),
            "final_goal_dist_m": round(dist1, 3),
            "BASELINE_NAVIGATION_INVALID": recovery_at_t0,
        },
    )


def _print_matrix(results: List[LayerResult]) -> None:
    print("\nTEST                          RESULT")
    print("-" * 44)
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"{r.name:<28} {status}")


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.2 baseline validity isolation")
    ap.add_argument("--live", action="store_true", help="Run M32-OPEN-STRAIGHT LIVE trace (requires sim)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    open_world = SimWorld()
    open_world.load_m32_open_straight()
    start = (M32_OPEN_START["x"], M32_OPEN_START["y"])
    goal = (M32_OPEN_GOAL["x"], M32_OPEN_GOAL["y"])

    results: List[LayerResult] = [
        audit_vehicle_model_straight(),
        audit_footprint_straight(),
        audit_kinematic_turn(),
        audit_trajectory_validator_open(open_world),
        audit_execution_corridor_baseline(),
        audit_geometry_compare(),
        audit_mppi_static(open_world, "open", start, goal, 0.0),
    ]

    # Indoor comparison MPPI (documents M3.1 failure, not pass criterion)
    indoor = SimWorld()
    from agv_bridge.nav_scenario_injector import BASELINE_START, BASELINE_GOAL

    isx, isy = BASELINE_START["x"], BASELINE_START["y"]
    igx, igy = BASELINE_GOAL["x"], BASELINE_GOAL["y"]
    iyaw = math.atan2(igy - isy, igx - isx)
    indoor_mppi = audit_mppi_static(indoor, "indoor_LM1_LM5", (isx, isy), (igx, igy), iyaw)
    indoor_mppi.name = "MPPI static (indoor control)"
    results.append(indoor_mppi)

    static_ok = all(r.passed for r in results if "MPPI static (open" in r.name or r.name.startswith("Footprint") or r.name.startswith("Kinematic") or r.name == "TrajectoryValidator")

    live_result: Optional[LayerResult] = None
    if args.live and static_ok:
        live_result = run_live_m32(args.base, args.seconds, args.hz)
        results.append(live_result)
    elif args.live and not static_ok:
        live_result = LayerResult("Open-space LIVE", False, "NOT RUN — static MPPI feasibility failed")
        results.append(live_result)

    _print_matrix(results)

    open_mppi = next(r for r in results if r.name.startswith("MPPI static (open"))
    first_failure = next((r.name for r in results if not r.passed and "indoor control" not in r.name), "NONE")

    print("\n===========================================")
    print("AGV NAVIGATION V0.2 M3.2")
    print("BASELINE VALIDITY ISOLATION")
    print("===========================================\n")
    print(f"VehicleModel:        {'PASS' if results[0].passed else 'FAIL'}")
    print(f"Footprint:           {'PASS' if results[1].passed else 'FAIL'}")
    print(f"MPPI (open static):  {'PASS' if open_mppi.passed else 'FAIL'} — {open_mppi.data.get('ALL_REJECTED_BECAUSE')}")
    print(f"ExecutionCorridor:   {'PASS' if results[4].passed else 'FAIL'}")
    print(f"Validator:           {'PASS' if results[3].passed else 'FAIL'}")
    print(f"Safety:              (LIVE only)")
    print(f"Controller:          (LIVE only)")
    print(f"\nStatic Straight:     {'PASS' if results[1].passed and open_mppi.passed else 'FAIL'}")
    print(f"Kinematic Turn:      {'PASS' if results[2].passed else 'FAIL'}")
    if live_result:
        print(f"Open-Space LIVE:     {'PASS' if live_result.passed else 'FAIL'}")
        print(f"Progress:            {live_result.data.get('initial_goal_dist_m')} → {live_result.data.get('final_goal_dist_m')} m")
        print(f"Requested VX:        {live_result.data.get('requested_vx')}")
        print(f"MPPI VX:             {live_result.data.get('mppi_vx')}")
        print(f"Approved VX:         {live_result.data.get('approved_vx')}")
        print(f"Safe VX Reason:      {live_result.data.get('safe_vx_reason')}")
    else:
        print("Open-Space LIVE:     NOT RUN")
    print(f"\nFirst Failure:       {first_failure}")
    if not open_mppi.passed:
        print(f"Root Cause:          MPPI fail because {open_mppi.data.get('ALL_REJECTED_BECAUSE')} — {open_mppi.detail}")
    elif live_result and not live_result.passed:
        print(f"Root Cause:          LIVE — {live_result.detail}")
    else:
        print("Root Cause:          NONE (static open-space feasible)")
    obstacle_ready = live_result.passed if live_result else False
    print(f"\nObstacle Validation: {'READY' if obstacle_ready else 'BLOCKED'}")

    if args.json:
        print(json.dumps([{"name": r.name, "passed": r.passed, "detail": r.detail, "data": r.data} for r in results], indent=2, ensure_ascii=False))

    critical = [r for r in results if r.name.startswith("MPPI static (open") or r.name == "Footprint straight" or r.name == "Kinematic turn"]
    return 0 if all(r.passed for r in critical) else 1


if __name__ == "__main__":
    raise SystemExit(main())
