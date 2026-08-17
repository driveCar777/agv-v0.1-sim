#!/usr/bin/env python3
"""M3.3 Global path physical feasibility + open-runway obstacle validation.

1. PathPhysicalValidator audit (LM1→LM5, M32, synthetic corridors)
2. Distinguish PATH CLASS A / B / C
3. Optional OBS-OPEN-* LIVE traces on M32 runway
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import urllib.error
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws" / "src" / "agv_bridge"
sys.path.insert(0, str(BRIDGE))

from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.nav_path_physical_validator import (  # noqa: E402
    PATH_CLASS_A,
    PATH_CLASS_B,
    PATH_CLASS_C,
    classify_path_case,
    validate_global_path,
)
from agv_bridge.nav_scenario_injector import (  # noqa: E402
    BASELINE_GOAL,
    BASELINE_START,
    M32_OPEN_GOAL,
    M32_OPEN_START,
    M33_SCENES,
    OBS_OPEN_SCENES,
)
from agv_bridge.smap_loader import build_m33_narrow_corridor, build_m33_tight_corner  # noqa: E402
from agv_bridge.sim_world import SimWorld  # noqa: E402

Pt = Tuple[float, float]
LOG_DIR = ROOT / "logs" / "navigation_v02"


@dataclass
class AuditCase:
    name: str
    reachable: bool
    path_class: str
    physically_feasible: bool
    first_invalid_index: int
    first_collision_reason: str
    first_invalid_pose: Optional[dict]
    path_min_clearance_m: Optional[float]
    segment_count: int
    detail: str = ""
    data: Dict[str, Any] = field(default_factory=dict)


def _world_fns(world: SimWorld, geom=DEFAULT_GEOM):
    def collide(x: float, y: float) -> bool:
        return world.collides(x, y, robot_r=geom.safety_radius, include_actors=False)

    def clearance_at(x: float, y: float) -> float:
        return world.clearance_at_xy(x, y, search_m=4.0)

    return collide, clearance_at


def _load_map(world: SimWorld, scene_key: str) -> None:
    if scene_key == "indoor":
        world.load_indoor()
    elif scene_key == "m32_open":
        world.load_m32_open_straight()
    elif scene_key == "m33_narrow":
        loaded = build_m33_narrow_corridor()
        world._clear_poi_cells(loaded, clear_r=1.0)
        world.map = loaded
        world.scene_id = "m33_narrow_corridor"
    elif scene_key == "m33_tight":
        loaded = build_m33_tight_corner()
        world._clear_poi_cells(loaded, clear_r=1.0)
        world.map = loaded
        world.scene_id = "m33_tight_corner"
    else:
        raise ValueError(scene_key)


def _audit_route(
    name: str,
    world: SimWorld,
    start: Pt,
    goal: Pt,
    *,
    map_inflate_m: float = 0.28,
) -> AuditCase:
    geom = DEFAULT_GEOM
    collide, clearance_at = _world_fns(world, geom)
    path = world.plan_path(start, goal, robot_r=geom.planner_radius)
    reachable = bool(path and len(path) >= 2)
    if not reachable:
        return AuditCase(
            name=name,
            reachable=False,
            path_class=PATH_CLASS_A,
            physically_feasible=False,
            first_invalid_index=-1,
            first_collision_reason="NO_PATH",
            first_invalid_pose=None,
            path_min_clearance_m=None,
            segment_count=0,
            detail="A* returned no path",
        )

    vr = validate_global_path(
        path,
        collide,
        clearance_at,
        geom=geom,
        map_inflate_m=map_inflate_m,
    )
    pc = classify_path_case(reachable, vr)
    first_seg = None
    if vr.first_invalid_index >= 0 and vr.segment_audits:
        first_seg = vr.segment_audits[vr.first_invalid_index].to_dict()

    return AuditCase(
        name=name,
        reachable=True,
        path_class=pc,
        physically_feasible=vr.physically_feasible,
        first_invalid_index=vr.first_invalid_index,
        first_collision_reason=vr.first_collision_reason,
        first_invalid_pose=vr.first_invalid_pose,
        path_min_clearance_m=vr.path_min_clearance_m,
        segment_count=len(vr.segment_audits),
        detail=f"class={pc} reason={vr.first_collision_reason} seg={vr.first_invalid_index}",
        data={
            "waypoints": len(path),
            "minimum_corridor_width_m": vr.minimum_corridor_width_m,
            "required_corridor_width_m": vr.required_corridor_width_m,
            "turning_radius_feasible": vr.turning_radius_feasible,
            "braking_feasible": vr.braking_feasible,
            "first_segment_audit": first_seg,
            "inflation_contract": vr.inflation_contract,
            "segment_audits": [s.to_dict() for s in vr.segment_audits[:8]],
        },
    )


def _print_lm15_report(case: AuditCase) -> None:
    print("\n--- LM1→LM5 segment audit ---")
    print(f"GLOBAL_PATH_REACHABLE:       {case.reachable}")
    print(f"GLOBAL_PATH_PHYSICALLY_FEASIBLE: {case.physically_feasible}")
    print(f"PATH CLASS:                  {case.path_class}")
    print(f"path_min_clearance_m:        {case.path_min_clearance_m}")
    print(f"first_invalid_index:         {case.first_invalid_index}")
    print(f"first_collision_reason:      {case.first_collision_reason}")
    print(f"first_invalid_pose:          {json.dumps(case.first_invalid_pose, ensure_ascii=False)}")
    if case.data.get("first_segment_audit"):
        seg = case.data["first_segment_audit"]
        print(f"first invalid segment:       #{seg.get('index')} from=({seg['from']['x']:.2f},{seg['from']['y']:.2f}) "
              f"to=({seg['to']['x']:.2f},{seg['to']['y']:.2f}) clr={seg.get('min_clearance_m')}")


def run_path_audits() -> List[AuditCase]:
    geom = DEFAULT_GEOM
    cases: List[AuditCase] = []

    indoor = SimWorld()
    indoor.load_indoor()
    isx, isy = BASELINE_START["x"], BASELINE_START["y"]
    igx, igy = BASELINE_GOAL["x"], BASELINE_GOAL["y"]
    lm15 = _audit_route("LM1→LM5 (indoor)", indoor, (isx, isy), (igx, igy))
    cases.append(lm15)

    open_w = SimWorld()
    open_w.load_m32_open_straight()
    ox, oy = M32_OPEN_START["x"], M32_OPEN_START["y"]
    mx, my = M32_OPEN_GOAL["x"], M32_OPEN_GOAL["y"]
    cases.append(_audit_route("M32 OPEN straight", open_w, (ox, oy), (mx, my)))

    narrow_w = SimWorld()
    _load_map(narrow_w, "m33_narrow")
    cases.append(_audit_route("M33 NARROW corridor", narrow_w, (ox, oy), (mx, my)))

    tight_w = SimWorld()
    _load_map(tight_w, "m33_tight")
    cases.append(_audit_route("M33 TIGHT-CORNER", tight_w, (2.0, 0.0), (22.0, 12.0)))

    return cases


def run_live_obs_open(base: str, seconds: float, hz: float, scenes: List[str]) -> List[dict]:
    try:
        import urllib.request

        with urllib.request.urlopen(base.rstrip("/") + "/api/state", timeout=5) as resp:
            resp.read()
    except Exception as exc:
        return [{"scene": "ALL", "passed": False, "detail": f"SIM_NOT_RUNNING: {exc}"}]

    trace = ROOT / "scripts" / "_trace_navigation_v02_live.py"
    analyze = ROOT / "scripts" / "_analyze_navigation_v02_trace.py"
    results: List[dict] = []

    for scene in scenes:
        rc = subprocess.call(
            [sys.executable, str(trace), "--scene", scene, "--seconds", str(seconds), "--hz", str(hz), "--base", base],
        )
        if rc != 0:
            results.append({"scene": scene, "passed": False, "detail": f"trace rc={rc}"})
            continue

        tag_prefix = scene.replace("OBS-OPEN-", "OBS-OPEN-").split("-")[0:3]
        files = sorted(LOG_DIR.glob(f"{scene}*.jsonl"), key=os.path.getmtime)
        if not files:
            results.append({"scene": scene, "passed": False, "detail": "no JSONL"})
            continue
        path = files[-1]
        out = subprocess.check_output([sys.executable, str(analyze), str(path), "--json"], text=True)
        report = json.loads(out)[0]
        stages = report.get("stages") or {}
        overall = stages.get("Overall", "FAIL")
        results.append(
            {
                "scene": scene,
                "passed": overall in ("PASS", "PARTIAL"),
                "overall": overall,
                "failure_class": report.get("failure_class"),
                "stages": stages,
                "jsonl": str(path),
                "timeline": {k: (v.get("seq") if v else None) for k, v in (report.get("timeline") or {}).items()},
            }
        )
    return results


def _print_matrix(cases: List[AuditCase], live: List[dict]) -> None:
    print("\n=== M3.3 Path Physical Audit ===")
    print(f"{'Route':<22} {'Reach':>5} {'Class':>5} {'PhysOK':>6} {'MinClr':>7} {'1stInv':>6}")
    print("-" * 58)
    for c in cases:
        clr = f"{c.path_min_clearance_m:.3f}" if c.path_min_clearance_m is not None else "N/A"
        print(
            f"{c.name:<22} {str(c.reachable):>5} {c.path_class:>5} {str(c.physically_feasible):>6} {clr:>7} {c.first_invalid_index:>6}"
        )

    if live:
        print("\n=== OBS-OPEN LIVE Evidence ===")
        print(f"{'Scene':<26} {'Overall':>8} {'Detect':>8} {'Probe':>8} {'Commit':>8}")
        print("-" * 62)
        for r in live:
            st = r.get("stages") or {}
            print(
                f"{r.get('scene',''):<26} {str(r.get('overall','')):>8} "
                f"{str(st.get('Detection','')):>8} {str(st.get('Probe','')):>8} {str(st.get('Commit','')):>8}"
            )


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.3 global path physical + open obstacle validation")
    ap.add_argument("--live", action="store_true", help="Run OBS-OPEN-* LIVE traces")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    cases = run_path_audits()
    lm15 = cases[0]

    print("\n===========================================")
    print("AGV NAVIGATION V0.2 M3.3")
    print("GLOBAL PATH PHYSICAL FEASIBILITY")
    print("===========================================")
    _print_lm15_report(lm15)

    print("\nInflation ownership:")
    contract = lm15.data.get("inflation_contract") or {}
    for k in ("map_inflation_m", "map_inflation_owner", "global_planner_robot_r", "validator_margin_m", "validator_margin_owner"):
        print(f"  {k}: {contract.get(k)}")

    live_results: List[dict] = []
    if args.live:
        print("\nRunning OBS-OPEN LIVE traces …")
        live_results = run_live_obs_open(args.base, args.seconds, args.hz, OBS_OPEN_SCENES)

    _print_matrix(cases, live_results)

    print("\n--- Classification summary ---")
    for label, expect in [
        ("OPEN (M32)", PATH_CLASS_C),
        ("WIDE (=OPEN)", PATH_CLASS_C),
        ("NARROW", "B or C"),
        ("TIGHT-CORNER", "B or C"),
        ("LM1→LM5", PATH_CLASS_B),
    ]:
        if label.startswith("LM1"):
            c = lm15
            print(f"  {label}: CLASS {c.path_class} (expect {expect}) — {'MATCH' if c.path_class == PATH_CLASS_B else 'CHECK'}")
        elif label.startswith("OPEN") or label.startswith("WIDE"):
            c = next(x for x in cases if "M32" in x.name)
            print(f"  {label}: CLASS {c.path_class} (expect {expect})")
        elif label.startswith("NARROW"):
            c = next(x for x in cases if "NARROW" in x.name)
            print(f"  {label}: CLASS {c.path_class}")
        elif label.startswith("TIGHT"):
            c = next(x for x in cases if "TIGHT" in x.name)
            print(f"  {label}: CLASS {c.path_class}")

    obstacle_ready = bool(live_results) and any(r.get("passed") for r in live_results)
    m32_open = next(c for c in cases if "M32" in c.name)
    print(f"\nOpen runway baseline (path):  {'PASS' if m32_open.path_class == PATH_CLASS_C else 'FAIL'}")
    print(f"Indoor LM1→LM5 (path):        CLASS {lm15.path_class} — local MPPI failure expected if CLASS B")
    print(f"Obstacle chain (open LIVE):   {'READY' if obstacle_ready else 'NOT RUN' if not live_results else 'PARTIAL/FAIL'}")

    if args.json:
        print(json.dumps({"path_audits": [c.__dict__ for c in cases], "live": live_results}, indent=2, ensure_ascii=False, default=str))

    path_ok = lm15.path_class in (PATH_CLASS_B, PATH_CLASS_C) and m32_open.path_class == PATH_CLASS_C
    return 0 if path_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
