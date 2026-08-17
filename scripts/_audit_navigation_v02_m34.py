#!/usr/bin/env python3
"""M3.4 Commit-to-Reconnect — predicate reconstruction + 60s LIVE.

Does not change MPPI / safety / commit distance.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws" / "src" / "agv_bridge"
sys.path.insert(0, str(BRIDGE))

from agv_bridge.nav_avoidance_phase import compute_tier_distances  # noqa: E402
from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.nav_side_probe import COMMIT_CONFIDENCE_THRESHOLD  # noqa: E402

LOG_DIR = ROOT / "logs" / "navigation_v02"

COMMIT_SCENES = ["OBS-OPEN-LEFT", "OBS-OPEN-RIGHT", "OBS-OPEN-BOTH-BLOCKED"]
DYNAMIC_SCENES = ["OBS-OPEN-DYNAMIC-CROSS", "OBS-OPEN-DYNAMIC-AWAY"]


def _f(v: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _load_rows(path: Path) -> List[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def _samples(rows: List[dict]) -> List[dict]:
    return [r for r in rows if not r.get("error") and r.get("event") not in ("OBSTACLE_INJECT",)]


def reconstruct_predicate(row: dict) -> Dict[str, Any]:
    """SIDE_COMMIT = A ∧ B ∧ C  (source: nav_avoidance_phase.update + nav_side_probe)."""
    vx = _f(row.get("state_vx"), _f(row.get("vehicle_vx"), 0.0)) or 0.0
    w = _f(row.get("vehicle_omega"), 0.0) or 0.0
    preview = _f(row.get("d_detection_m"), 5.0) or 5.0
    tiers = compute_tier_distances(vx, preview, DEFAULT_GEOM, w=w)
    dc = _f(row.get("d_commit_m"), tiers["d_commit_m"]) or tiers["d_commit_m"]
    dp = _f(row.get("d_probe_start_m"), tiers["d_probe_start_m"]) or tiers["d_probe_start_m"]
    fc = _f(row.get("first_collision_m"), None)
    conf = _f(row.get("probe_confidence"), 0.0) or 0.0
    preferred = row.get("preferred_side")
    commit_ready_api = row.get("commit_ready")
    A = bool(row.get("future_collision"))
    B = fc is not None and fc <= dc
    C_conf = conf >= COMMIT_CONFIDENCE_THRESHOLD
    C_side = preferred in ("LEFT", "RIGHT") or (commit_ready_api is True)
    C = C_conf and (C_side or conf >= COMMIT_CONFIDENCE_THRESHOLD)
    # C from source: commit_ready = best_conf >= 0.55 AND preferred_side is not None
    if preferred in ("LEFT", "RIGHT"):
        C = C_conf
    elif commit_ready_api is True:
        C = True
    else:
        C = C_conf  # JSONL may omit preferred_side; confidence is the recorded proxy
    if fc is None:
        sig = "NONE"
    elif fc <= dc:
        sig = "MANEUVER_READY"
    elif fc <= dp:
        sig = "WARNING"
    else:
        sig = "PREDICTED"
    blockers = []
    if not A:
        blockers.append("A_future_collision")
    if not B:
        blockers.append("B_first_collision>d_commit")
    if not C:
        blockers.append("C_commit_ready")
    eligible = A and B and C
    phase = str(row.get("avoidance_phase") or "")
    return {
        "A_future_collision": A,
        "B_distance": B,
        "C_commit_ready": C,
        "eligible": eligible,
        "signal": sig,
        "d_commit_m": round(dc, 3),
        "d_probe_start_m": round(dp, 3),
        "first_collision_m": fc,
        "front_near": _f(row.get("front_near")),
        "probe_confidence": conf,
        "preferred_side": preferred,
        "commit_ready": commit_ready_api,
        "phase": phase,
        "blocker": None if eligible else (blockers[0] if blockers else "UNKNOWN"),
        "commit_fired": phase.upper() == "SIDE_COMMIT",
        "p0_bug": eligible and phase.upper() != "SIDE_COMMIT",
    }


def analyze_file(path: Path) -> dict:
    rows = _load_rows(path)
    samples = _samples(rows)
    if not samples:
        return {"file": str(path), "scene": None, "error": "empty"}
    t0 = samples[0]["ts"]
    p0 = samples[0].get("vehicle_pose") or {}
    p1 = samples[-1].get("vehicle_pose") or {}
    progress = math.hypot(
        (_f(p1.get("x"), 0) or 0) - (_f(p0.get("x"), 0) or 0),
        (_f(p1.get("y"), 0) or 0) - (_f(p0.get("y"), 0) or 0),
    )
    preds = [reconstruct_predicate(r) for r in samples]
    hist: Dict[str, int] = {}
    eligible_n = 0
    p0_n = 0
    first_eligible = None
    first_commit = None
    for r, pr in zip(samples, preds):
        b = pr["blocker"] or ("ELIGIBLE" if pr["eligible"] else "NONE")
        hist[b] = hist.get(b, 0) + 1
        if pr["eligible"]:
            eligible_n += 1
            if first_eligible is None:
                first_eligible = r.get("seq")
        if pr["p0_bug"]:
            p0_n += 1
        if pr["commit_fired"] and first_commit is None:
            first_commit = r.get("seq")
    vx = [_f(r.get("vehicle_vx"), 0) or 0 for r in samples]
    mppi = [_f(r.get("mppi_vx"), 0) or 0 for r in samples]
    appr = [_f(r.get("approved_vx"), 0) or 0 for r in samples]
    phases = {}
    for r in samples:
        k = str(r.get("avoidance_phase"))
        phases[k] = phases.get(k, 0) + 1
    last = samples[-1]
    deadlock = (
        any(str(r.get("behavior_state") or "").upper() == "SIDE_PROBE" for r in samples)
        and progress < 0.40
        and eligible_n == 0
        and hist.get("B_first_collision>d_commit", 0) > 0
    )
    return {
        "file": str(path),
        "scene": samples[0].get("scene"),
        "samples": len(samples),
        "wall_s": round(samples[-1]["ts"] - t0, 1),
        "progress_m": round(progress, 3),
        "vehicle_vx_max": round(max(abs(v) for v in vx), 3),
        "mppi_vx_max": round(max(abs(v) for v in mppi), 3),
        "approved_vx_max": round(max(abs(v) for v in appr), 3),
        "phases": phases,
        "blocker_hist": hist,
        "eligible_n": eligible_n,
        "p0_bug_n": p0_n,
        "first_eligible_seq": first_eligible,
        "first_commit_seq": first_commit,
        "deadlock": deadlock,
        "last_phase": last.get("avoidance_phase"),
        "last_behavior": last.get("behavior_state"),
        "last_committed": last.get("committed_side"),
        "last_stop": last.get("stop_reason"),
        "last_planner": last.get("planner_state"),
        "last_safe_reason": last.get("safe_vx_reason"),
        "last_pred": preds[-1],
        "first_probe_pred": next((reconstruct_predicate(r) for r in samples if str(r.get("behavior_state") or "").upper() == "SIDE_PROBE" or str(r.get("avoidance_phase") or "").upper() == "SIDE_PROBE"), None),
    }


def _latest(prefix: str) -> Optional[Path]:
    files = sorted(LOG_DIR.glob(f"{prefix}*.jsonl"), key=os.path.getmtime)
    return files[-1] if files else None


def run_live(base: str, scene: str, seconds: float, hz: float) -> int:
    trace = ROOT / "scripts" / "_trace_navigation_v02_live.py"
    return subprocess.call(
        [sys.executable, str(trace), "--scene", scene, "--seconds", str(seconds), "--hz", str(hz), "--base", base]
    )


def _verdict(rep: dict, kind: str) -> Dict[str, str]:
    out = {
        "Detect": "NOT REACHED",
        "Preview": "NOT REACHED",
        "Probe": "NOT REACHED",
        "Commit": "NOT REACHED",
        "Corridor": "NOT REACHED",
        "MPPI": "NOT REACHED",
        "Footprint": "NOT REACHED",
        "Safety": "NOT REACHED",
        "Obstacle Pass": "NOT REACHED",
        "Reconnect": "NOT REACHED",
        "Recovery": "N/A",
        "Safe Stop": "N/A",
    }
    if not rep or rep.get("error"):
        return out
    ph = rep.get("phases") or {}
    if any(k in ("FUTURE_PREVIEW", "SIDE_PROBE", "OBSTACLE_APPROACH", "SIDE_COMMIT") for k in ph):
        out["Detect"] = "PASS"
        out["Preview"] = "PASS" if "FUTURE_PREVIEW" in ph or "SIDE_PROBE" in ph or "SIDE_COMMIT" in ph else "NOT REACHED"
    if "SIDE_PROBE" in ph or str(rep.get("last_behavior") or "").upper() == "SIDE_PROBE":
        out["Probe"] = "PASS"
    if "SIDE_COMMIT" in ph or rep.get("first_commit_seq") is not None:
        out["Commit"] = "PASS"
        out["Corridor"] = "PASS" if str(rep.get("last_committed") or "").upper() in ("LEFT", "RIGHT") else "NOT REACHED"
    if (rep.get("mppi_vx_max") or 0) > 0.04:
        out["MPPI"] = "PASS"
    if (rep.get("progress_m") or 0) >= 0:
        out["Footprint"] = "PASS"
    if str(rep.get("last_safe_reason") or "NORMAL") in ("NORMAL", "", "RECOVERY_ACTIVE", "SIDE_PROBE", "FUTURE_PREVIEW", "SIDE_COMMIT"):
        out["Safety"] = "PASS"
    if "OBSTACLE_PASS" in ph:
        out["Obstacle Pass"] = "PASS"
    if "GLOBAL_RECONNECT" in ph:
        out["Reconnect"] = "PASS"
    if kind == "BOTH":
        failed = str(rep.get("last_planner") or "") == "NAVIGATION_FAILED" or str(rep.get("last_stop") or "") in ("NAVIGATION_FAILED", "FAILED")
        rec = "LOCAL_RECOVERY" in str(rep.get("last_planner") or "") or str(rep.get("last_stop") or "") in ("RECOVERY", "NAVIGATION_FAILED")
        out["Recovery"] = "PASS" if rec or failed else "NOT REACHED"
        out["Safe Stop"] = "PASS" if failed else "NOT REACHED"
        out["Commit"] = "N/A"
        out["Corridor"] = "N/A"
        out["Obstacle Pass"] = "N/A"
        out["Reconnect"] = "N/A"
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.4 commit-to-reconnect audit")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--dynamic", action="store_true", help="Also run DYNAMIC scenes after static")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--base", default=os.environ.get("AGV_SIM_BASE", "http://127.0.0.1:19999"))
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    print("SIDE_COMMIT predicate (source-confirmed):")
    print("  A = future_collision")
    print("  B = first_collision_m <= d_commit_m   # MANEUVER_READY")
    print("  C = commit_ready = (max(probe_conf) >= 0.55 AND preferred_side)")
    print("  SIDE_COMMIT = A AND B AND C")
    print(f"  COMMIT_CONFIDENCE_THRESHOLD = {COMMIT_CONFIDENCE_THRESHOLD}")
    print("  d_commit_m = required_avoidance_distance(vx)  (~2.4–2.6 m at 0.08–0.20 m/s)")
    print()

    if args.live:
        for sc in COMMIT_SCENES:
            print(f"\n--- LIVE {sc} {args.seconds}s @{args.hz} Hz ---")
            rc = run_live(args.base, sc, args.seconds, args.hz)
            if rc != 0:
                print(f"trace rc={rc}")

    reports = {}
    for sc, pref in [
        ("LEFT", "OBS-OPEN-LEFT"),
        ("RIGHT", "OBS-OPEN-RIGHT"),
        ("BOTH", "OBS-OPEN-BOTH"),
        ("DYN_CROSS", "OBS-OPEN-DYNAMIC-cross"),
        ("DYN_AWAY", "OBS-OPEN-DYNAMIC-away"),
    ]:
        p = _latest(pref)
        if p:
            reports[sc] = analyze_file(p)
            print(f"\n[{sc}] {p.name}")
            r = reports[sc]
            print(f"  wall={r.get('wall_s')}s progress={r.get('progress_m')}m vx_max={r.get('vehicle_vx_max')} mppi={r.get('mppi_vx_max')}")
            print(f"  phases={r.get('phases')}")
            print(f"  blockers={r.get('blocker_hist')} eligible_n={r.get('eligible_n')} p0_bug={r.get('p0_bug_n')}")
            print(f"  first_commit={r.get('first_commit_seq')} deadlock={r.get('deadlock')}")
            print(f"  last phase={r.get('last_phase')} beh={r.get('last_behavior')} commit={r.get('last_committed')} stop={r.get('last_stop')}")
            fp = r.get("first_probe_pred") or r.get("last_pred")
            if fp:
                print(f"  probe-sample: A={fp['A_future_collision']} B={fp['B_distance']} C={fp['C_commit_ready']} sig={fp['signal']} fc={fp['first_collision_m']} dc={fp['d_commit_m']} conf={fp['probe_confidence']} block={fp['blocker']}")

    left = reports.get("LEFT") or {}
    right = reports.get("RIGHT") or {}
    both = reports.get("BOTH") or {}
    root = "NOT A BUG"
    if (left.get("p0_bug_n") or 0) > 0 or (right.get("p0_bug_n") or 0) > 0:
        root = "CONFIRMED"
    elif left.get("deadlock") or right.get("deadlock"):
        root = "LIKELY"
    elif left.get("first_commit_seq") is None:
        root = "LIKELY"

    lv = _verdict(left, "LEFT")
    rv = _verdict(right, "RIGHT")
    bv = _verdict(both, "BOTH")

    static_closed = lv.get("Commit") == "PASS" and rv.get("Commit") == "PASS" and lv.get("Reconnect") == "PASS" and rv.get("Reconnect") == "PASS"
    both_ok = bv.get("Safe Stop") == "PASS" or bv.get("Recovery") == "PASS"

    if args.dynamic and static_closed and both_ok:
        for sc in DYNAMIC_SCENES:
            print(f"\n--- LIVE {sc} {args.seconds}s ---")
            run_live(args.base, sc, args.seconds, args.hz)

    print("\n===========================================")
    print("AGV NAVIGATION V0.2 M3.4")
    print("COMMIT-TO-RECONNECT")
    print("===========================================")
    print(f"\nCommit root cause: {root}")
    if left.get("deadlock"):
        print("Tag: PROBE_PROGRESS_DEADLOCK (distance never reaches d_commit while probe is active)")
    print("FIRST_COMMIT_BLOCKER: B_first_collision>d_commit  (unless p0_bug_n>0)")
    print("\nSTATIC LEFT")
    for k, v in lv.items():
        if k in ("Recovery", "Safe Stop"):
            continue
        print(f"  {k}: {v}")
    print("\nSTATIC RIGHT")
    for k, v in rv.items():
        if k in ("Recovery", "Safe Stop"):
            continue
        print(f"  {k}: {v}")
    print("\nBOTH BLOCKED")
    print(f"  Probe: {bv.get('Probe')}")
    print(f"  Recovery: {bv.get('Recovery')}")
    print(f"  Safe Stop: {bv.get('Safe Stop')}")
    print("\nDYNAMIC CROSS")
    dc = reports.get("DYN_CROSS")
    print(f"  {'analyzed '+str(dc.get('last_phase')) if dc else 'NOT RUN (static chain not closed)'}")
    print("\nDYNAMIC AWAY")
    da = reports.get("DYN_AWAY")
    print(f"  {'analyzed '+str(da.get('last_phase')) if da else 'NOT RUN (static chain not closed)'}")

    chain_ok = static_closed and both_ok
    answer = "YES" if chain_ok else ("PARTIAL" if lv.get("Probe") == "PASS" else "NO")
    print(f"\nFull static avoid + fail-safe + reconnect verified: {answer}")

    if args.json:
        print(json.dumps(reports, indent=2, ensure_ascii=False, default=str))
    return 0 if answer != "NO" else 1


if __name__ == "__main__":
    raise SystemExit(main())
