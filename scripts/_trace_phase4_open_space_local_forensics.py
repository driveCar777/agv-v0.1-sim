#!/usr/bin/env python3
"""P0-C.1 OPEN-SPACE local planning forensics (READ/TRACE only).

Requires web sim at BASE (default 127.0.0.1:19999).
Does NOT change planning / Safety / FSM / MPPI / P0-C validator.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 10.0) -> Dict[str, Any]:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _f(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def pose() -> Tuple[float, float, float]:
    a = (api("GET", "/api/state").get("agv") or {})
    return float(a.get("x") or 0), float(a.get("y") or 0), float(a.get("angle") or 0)


def bf(x: float, y: float, yaw: float, fwd: float, lat: float) -> Tuple[float, float]:
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def cancel_clear() -> None:
    for p in ("/api/cancel", "/api/nav/stop"):
        try:
            api("POST", p, {})
        except Exception:
            pass
    try:
        api("POST", "/api/obstacles/clear", {})
    except Exception:
        pass
    time.sleep(0.3)


def setup_open_space(goal_m: float = 10.0) -> Dict[str, Any]:
    cancel_clear()
    try:
        api("POST", "/api/scene", {"id": "outdoor_campus"})
        time.sleep(0.7)
    except Exception:
        pass
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.35)
    x, y, yaw = pose()
    gx, gy = bf(x, y, yaw, goal_m, 0.0)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.9)
    return {"start": (x, y, yaw), "goal": (gx, gy), "goal_m": goal_m}


def sample_row(t: float) -> Dict[str, Any]:
    st = api("GET", "/api/state")
    try:
        fos_wrap = api("GET", "/api/nav/forensics/open-space")
    except Exception:
        fos_wrap = {}
    prev = api("GET", "/api/nav/preview")
    nav = st.get("nav") or {}
    fos = (
        fos_wrap.get("open_space_forensics")
        or nav.get("open_space_forensics")
        or prev.get("open_space_forensics")
        or {}
    )
    loc = fos.get("local_selector") or {}
    mppi = fos.get("mppi") or {}
    cmd = fos.get("command") or {}
    pol = fos.get("policy") or {}
    diag = fos.get("diagnostics") or {}
    p0c = fos.get("p0c") or {}
    render = fos.get("render") or {}
    gref = prev.get("global_reference") or nav.get("global_reference") or {}
    return {
        "t": round(t, 3),
        "scene": fos.get("scene") or pol.get("scene"),
        "policy_state": pol.get("state"),
        "policy_behavior": pol.get("behavior"),
        "vx_scale": pol.get("vx_scale") or mppi.get("vx_scale"),
        "profile": pol.get("profile_name"),
        "global_preview_m": fos.get("global_preview_m") or gref.get("preview_m"),
        "local_horizon_s": loc.get("horizon_s"),
        "local_max_distance_m": loc.get("max_distance_m"),
        "local_nominal_distance_m": loc.get("nominal_distance_m"),
        "compare_invoked": loc.get("compare_invoked"),
        "compare_called": loc.get("compare_called"),
        "compare_reason": loc.get("compare_reason"),
        "fsm_skip_reason": loc.get("fsm_skip_reason"),
        "candidate_count": loc.get("candidate_count"),
        "valid_count": loc.get("valid_count"),
        "selected_candidate": loc.get("selected_candidate"),
        "mppi_horizon_s": mppi.get("horizon_s"),
        "mppi_time_steps": mppi.get("time_steps"),
        "mean_vx_before": mppi.get("mean_vx_before"),
        "mean_vx_after": mppi.get("mean_vx_after"),
        "vx_raw": mppi.get("vx_raw"),
        "vx_cmd": mppi.get("vx_cmd"),
        "path_follow_weight": mppi.get("path_follow_weight"),
        "mppi_best_path_m": mppi.get("best_path_distance_m"),
        "control_mode": mppi.get("control_mode"),
        "requested_vx": cmd.get("requested_vx"),
        "safe_vx": cmd.get("safe_vx"),
        "state_vx": cmd.get("state_vx"),
        "safety_clamp": cmd.get("safety_clamp"),
        "front_near": cmd.get("front_near"),
        "stop_reason": cmd.get("stop_reason") or nav.get("stop_reason"),
        "coverage_ratio": diag.get("coverage_ratio"),
        "short_horizon_reason": diag.get("short_horizon_reason"),
        "expected_at_nominal": diag.get("expected_distance_at_nominal_vx"),
        "expected_at_state": diag.get("expected_distance_at_state_vx"),
        "planned_distance_m": diag.get("planned_distance_m"),
        "survived_distance_m": diag.get("actual_survived_distance_m"),
        "speed_ratio": diag.get("speed_ratio"),
        "kinematic_valid": p0c.get("kinematic_valid"),
        "path_valid": p0c.get("path_valid"),
        "p0c_contradiction": p0c.get("open_vs_invalid_contradiction"),
        "render_local_src": render.get("local_candidates_source"),
        "render_physical_src": render.get("active_physical_source"),
        "candidates": loc.get("candidates") or [],
        "raw_forensics": fos,
    }


def pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    ys = sorted(xs)
    i = min(len(ys) - 1, max(0, int(round((p / 100.0) * (len(ys) - 1)))))
    return ys[i]


def summarize(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    state_vx = [v for v in (_f(r.get("state_vx")) for r in rows) if v is not None]
    req_vx = [v for v in (_f(r.get("requested_vx")) for r in rows) if v is not None]
    safe_vx = [v for v in (_f(r.get("safe_vx")) for r in rows) if v is not None]
    mean_vx = [v for v in (_f(r.get("mean_vx_after")) for r in rows) if v is not None]
    local_d = [v for v in (_f(r.get("local_max_distance_m")) for r in rows) if v is not None]
    cov = [v for v in (_f(r.get("coverage_ratio")) for r in rows) if v is not None]
    gprev = [v for v in (_f(r.get("global_preview_m")) for r in rows) if v is not None]
    compare_false = sum(1 for r in rows if r.get("compare_called") is False or r.get("compare_invoked") is False)
    reasons = {}
    for r in rows:
        k = str(r.get("short_horizon_reason") or "UNKNOWN")
        reasons[k] = reasons.get(k, 0) + 1
    scenes = {}
    for r in rows:
        k = str(r.get("scene") or r.get("policy_state") or "?")
        scenes[k] = scenes.get(k, 0) + 1
    safety_clamps = sum(1 for r in rows if r.get("safety_clamp"))
    max_vx = 0.40
    mean_state = statistics.mean(state_vx) if state_vx else None
    return {
        "n_samples": len(rows),
        "mean_state_vx": None if mean_state is None else round(mean_state, 4),
        "p50_state_vx": None if not state_vx else round(pct(state_vx, 50) or 0.0, 4),
        "p90_state_vx": None if not state_vx else round(pct(state_vx, 90) or 0.0, 4),
        "max_state_vx": None if not state_vx else round(max(state_vx), 4),
        "mean_requested_vx": None if not req_vx else round(statistics.mean(req_vx), 4),
        "mean_safe_vx": None if not safe_vx else round(statistics.mean(safe_vx), 4),
        "mean_mppi_mean_vx": None if not mean_vx else round(statistics.mean(mean_vx), 4),
        "max_speed_ratio": None if mean_state is None else round(mean_state / max_vx, 4),
        "mean_local_max_distance_m": None if not local_d else round(statistics.mean(local_d), 4),
        "mean_global_preview_m": None if not gprev else round(statistics.mean(gprev), 4),
        "mean_coverage_ratio": None if not cov else round(statistics.mean(cov), 4),
        "compare_not_called_count": compare_false,
        "compare_not_called_ratio": round(compare_false / max(1, len(rows)), 3),
        "short_horizon_reason_hist": reasons,
        "scene_hist": scenes,
        "safety_clamp_count": safety_clamps,
        "expected_nominal_1_5s_x_0_18": round(0.18 * 1.5, 3),
    }


def classify_root_causes(summary: Dict[str, Any], rows: List[Dict[str, Any]]) -> Dict[str, str]:
    """Evidence tags only: CONFIRMED / LIKELY / POSSIBLE / NOT EVIDENCED."""
    out: Dict[str, str] = {}
    reasons = summary.get("short_horizon_reason_hist") or {}
    mean_local = summary.get("mean_local_max_distance_m")
    expected = summary.get("expected_nominal_1_5s_x_0_18")
    if mean_local is not None and expected is not None and abs(mean_local - expected) < 0.12:
        out["LOCAL_HORIZON_FIXED_1_5S"] = "CONFIRMED"
    elif reasons.get("FIXED_1_5S", 0) >= max(1, len(rows) // 3):
        out["LOCAL_HORIZON_FIXED_1_5S"] = "CONFIRMED"
    else:
        out["LOCAL_HORIZON_FIXED_1_5S"] = "LIKELY"

    mean_state = summary.get("mean_state_vx")
    if mean_state is not None and mean_state < 0.22:
        out["LOW_SPEED_VS_SIDE_NOM_0_22"] = "CONFIRMED"
    else:
        out["LOW_SPEED_VS_SIDE_NOM_0_22"] = "NOT EVIDENCED"

    mean_mppi = summary.get("mean_mppi_mean_vx")
    if mean_mppi is not None and abs(mean_mppi - 0.16) < 0.06:
        out["MPPI_MEAN_VX_NEAR_0_16"] = "CONFIRMED"
    elif mean_mppi is not None and mean_mppi < 0.22:
        out["MPPI_MEAN_VX_NEAR_0_16"] = "LIKELY"
    else:
        out["MPPI_MEAN_VX_NEAR_0_16"] = "POSSIBLE"

    if summary.get("compare_not_called_ratio", 0) >= 0.5:
        out["OPEN_NO_SUSTAINED_COMPARE"] = "CONFIRMED"
    elif summary.get("compare_not_called_count", 0) > 0:
        out["OPEN_NO_SUSTAINED_COMPARE"] = "LIKELY"
    else:
        out["OPEN_NO_SUSTAINED_COMPARE"] = "NOT EVIDENCED"

    if summary.get("safety_clamp_count", 0) == 0:
        out["SAFETY_CLAMP"] = "NOT EVIDENCED"
    else:
        out["SAFETY_CLAMP"] = "CONFIRMED"

    capture_hits = 0
    early_invalid = 0
    for r in rows:
        for c in r.get("candidates") or []:
            if str(c.get("reason") or "") == "NO_PATH_CAPTURE":
                capture_hits += 1
            plan_d = _f(c.get("planned_distance_m"))
            surv = _f(c.get("actual_survived_distance_m") or c.get("distance_m"))
            if plan_d and surv is not None and surv + 0.04 < 0.7 * plan_d:
                early_invalid += 1
    out["PATH_CAPTURE_FAILURE"] = "CONFIRMED" if capture_hits else "NOT EVIDENCED"
    out["EARLY_INVALIDATION"] = "CONFIRMED" if early_invalid else "NOT EVIDENCED"

    p0c_bad = any(r.get("p0c_contradiction") for r in rows)
    kin_false = any(r.get("kinematic_valid") is False for r in rows)
    open_ok = any(str(r.get("scene") or "").upper() in ("OPEN", "OPEN_SPACE") for r in rows)
    if p0c_bad:
        out["P0C_FORCES_PATH_INVALID"] = "CONFIRMED"
    elif kin_false and open_ok:
        out["P0C_FORCES_PATH_INVALID"] = "NOT EVIDENCED"  # kin false but scene still OPEN
    else:
        out["P0C_FORCES_PATH_INVALID"] = "NOT EVIDENCED"

    return out


def main() -> int:
    global BASE
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--seconds", type=float, default=4.5)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--goal-m", type=float, default=10.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    BASE = args.base.rstrip("/")
    out_dir = args.out or os.path.join("docs", "_phase4_trace")
    os.makedirs(out_dir, exist_ok=True)
    stamp = int(time.time())
    jsonl_path = os.path.join(out_dir, f"open_space_forensics_{stamp}.jsonl")
    csv_path = os.path.join(out_dir, f"open_space_forensics_{stamp}.csv")
    summary_path = os.path.join(out_dir, f"open_space_forensics_{stamp}_summary.json")

    print("=== P0-C.1 Open-Space Local Planning Forensics ===")
    print(f"BASE={BASE} window={args.seconds}s hz={args.hz}")
    try:
        api("GET", "/api/state")
    except Exception as e:
        print(f"SIM_UNREACHABLE {e}")
        return 2

    try:
        meta = setup_open_space(goal_m=float(args.goal_m))
    except Exception as e:
        print(f"SETUP_FAIL {e}")
        return 3
    print(f"SETUP ok goal_m={meta['goal_m']} start={meta['start']} goal={meta['goal']}")

    dt = 1.0 / max(1.0, float(args.hz))
    t0 = time.time()
    rows: List[Dict[str, Any]] = []
    print("TIMELINE")
    with open(jsonl_path, "w", encoding="utf-8") as jf:
        jf.write(json.dumps({"type": "meta", "base": BASE, "goal_m": args.goal_m, "setup": meta}, ensure_ascii=False) + "\n")
        while time.time() - t0 <= float(args.seconds):
            t = time.time() - t0
            try:
                row = sample_row(t)
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
                print(f"T+{t:5.2f} FETCH_FAIL {e}")
                time.sleep(dt)
                continue
            rows.append(row)
            slim = {k: row[k] for k in row if k not in ("candidates", "raw_forensics")}
            jf.write(json.dumps({"type": "sample", **slim}, ensure_ascii=False) + "\n")
            print(
                f"T+{t:5.2f} scene={row.get('scene')} pol={row.get('policy_state')} "
                f"scale={row.get('vx_scale')} compare={row.get('compare_called')}/{row.get('compare_reason')} "
                f"G={row.get('global_preview_m')} Lh={row.get('local_horizon_s')} Ld={row.get('local_max_distance_m')} "
                f"Mh={row.get('mppi_horizon_s')} mean={row.get('mean_vx_after')} raw={row.get('vx_raw')} "
                f"req={row.get('requested_vx')} safe={row.get('safe_vx')} state={row.get('state_vx')} "
                f"cov={row.get('coverage_ratio')} reason={row.get('short_horizon_reason')} "
                f"kin={row.get('kinematic_valid')} path_valid={row.get('path_valid')}"
            )
            time.sleep(dt)

    summary = summarize(rows)
    root = classify_root_causes(summary, rows)
    payload = {
        "type": "summary",
        "summary": summary,
        "root_cause_tags": root,
        "jsonl": jsonl_path,
        "csv": csv_path,
    }
    with open(summary_path, "w", encoding="utf-8") as sf:
        json.dump(payload, sf, ensure_ascii=False, indent=2)
    with open(jsonl_path, "a", encoding="utf-8") as jf:
        jf.write(json.dumps(payload, ensure_ascii=False) + "\n")

    csv_fields = [
        "t",
        "global_preview_m",
        "local_horizon_s",
        "local_max_distance_m",
        "vx_scale",
        "mean_vx_after",
        "vx_raw",
        "requested_vx",
        "safe_vx",
        "state_vx",
        "coverage_ratio",
        "compare_called",
        "compare_reason",
        "short_horizon_reason",
        "scene",
        "policy_state",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as cf:
        w = csv.DictWriter(cf, fieldnames=csv_fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    cancel_clear()
    print("SUMMARY", json.dumps(summary, ensure_ascii=False))
    print("ROOT_CAUSE_TAGS", json.dumps(root, ensure_ascii=False))
    print(f"WROTE {jsonl_path}")
    print(f"WROTE {csv_path}")
    print(f"WROTE {summary_path}")
    print("P0-C.1 OPEN-SPACE FORENSICS DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
