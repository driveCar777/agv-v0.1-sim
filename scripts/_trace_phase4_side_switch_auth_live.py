#!/usr/bin/env python3
"""STEP 3E LIVE scenes on http://127.0.0.1:19999.

Scene A: LEFT blocked, RIGHT clear → expect AUTHORIZED → LOCAL_RIGHT
Scene B: mid_inject hard (Probe often R INVALID) → expect DENIED, no RIGHT
Scene C: soft both-valid style → expect DENIED CURRENT_SIDE_STILL_VALID

Does not loosen Probe. Does not clear STEP2/3C/3D traces.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from typing import Any, Dict, List, Optional

BASE = "http://127.0.0.1:19999"


def api(method: str, path: str, body=None, timeout: float = 10.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def debug() -> Dict[str, Any]:
    wrap = api("GET", "/api/nav/debug")
    return wrap.get("debug") if isinstance(wrap.get("debug"), dict) else wrap


def sample(t0: float) -> Dict[str, Any]:
    d = debug()
    pol = d.get("nav_policy") or {}
    man = d.get("maneuver") or {}
    lm = d.get("local_maneuver") or {}
    sw = d.get("side_switch") or (d.get("phase4") or {}).get("side_switch") or pol.get("side_switch") or {}
    pr = d.get("probe") or (d.get("phase4") or {}).get("probe") or pol.get("probe") or {}
    c = d.get("commitment") or (d.get("phase4") or {}).get("commitment") or pol.get("commitment") or {}
    events = pol.get("events") or d.get("events") or (d.get("phase4") or {}).get("events") or []
    return {
        "type": "sample",
        "t": round(time.time() - t0, 4),
        "policy_state": pol.get("state"),
        "maneuver_mode": man.get("mode"),
        "selector": lm.get("decision"),
        "selector_reason": lm.get("reason") or man.get("reason"),
        "commitment": c,
        "side_switch": sw,
        "probe": {
            "forward": (pr.get("forward") or {}).get("status"),
            "backward": (pr.get("backward") or {}).get("status"),
            "left": (pr.get("left") or {}).get("status"),
            "right": (pr.get("right") or {}).get("status"),
            "turn": (pr.get("turn_in_place") or {}).get("status"),
            "bundle_ms": pr.get("bundle_ms"),
        },
        "auth_ms": sw.get("authorization_ms"),
        "stop_reason": (d.get("diagnostics") or {}).get("stop_reason"),
        "recent_events": [
            e.get("type")
            for e in (events[-8:] if isinstance(events, list) else [])
            if isinstance(e, dict)
        ],
    }


def body_frame(x, y, yaw, fwd, lat):
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def setup_nav(gx: float = -20.0, gy: float = 0.0) -> None:
    try:
        api("POST", "/api/cancel", {})
    except Exception:
        pass
    try:
        api("POST", "/api/nav/stop", {})
    except Exception:
        pass
    api("POST", "/api/obstacles/clear", {})
    time.sleep(0.35)
    wrap = api("POST", "/api/nav/plan", {"x": gx, "y": gy})
    plan = wrap.get("api") or wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        raise RuntimeError(f"plan failed: {wrap}")
    api("POST", "/api/nav/confirm", {})
    time.sleep(0.8)


def get_pose():
    st = api("GET", "/api/state")
    agv = st.get("agv") or {}
    return float(agv.get("x") or 0), float(agv.get("y") or 0), float(agv.get("angle") or 0)


def scene_a_stage1(x: float, y: float, yaw: float) -> Dict[str, Any]:
    """Front block + RIGHT seal → force LOCAL_LEFT commitment (no clear)."""
    px, py = body_frame(x, y, yaw, 1.15, -0.12)
    seals = []
    api("POST", "/api/obstacles/add", {"x": px, "y": py, "r": 0.42, "name": "e_a_pillar"})
    for name, fwd, lat, r in (
        ("e_a_ra", 1.05, -0.95, 0.55),
        ("e_a_rb", 1.45, -1.15, 0.48),
        ("e_a_rc", 0.75, -0.85, 0.40),
    ):
        bx, by = body_frame(x, y, yaw, fwd, lat)
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": name})
        seals.append(name)
    return {"pillar": {"x": px, "y": py, "r": 0.42}, "right_seals": seals}


def scene_a_stage2(x: float, y: float, yaw: float, stage1: Dict[str, Any]) -> Dict[str, Any]:
    """Surgical: remove RIGHT seals, nudge pillar LEFT — keep commitment alive.

    Never call /obstacles/clear (full clear can flash FORWARD and release commitment).
    Tuned LIVE: nudge≈0.75m into LEFT → Probe L=INVALID R=VALID T=VALID.
    """
    for name in stage1.get("right_seals") or ["e_a_ra", "e_a_rb", "e_a_rc"]:
        try:
            api("POST", "/api/obstacles/remove", {"name": name})
        except Exception:
            pass
    pillar = (stage1 or {}).get("pillar") or {}
    nudge = 0.75
    if pillar:
        ox = float(pillar["x"]) + nudge * math.cos(yaw + math.pi / 2)
        oy = float(pillar["y"]) + nudge * math.sin(yaw + math.pi / 2)
        try:
            api("POST", "/api/obstacles/remove", {"name": "e_a_pillar"})
        except Exception:
            pass
        api("POST", "/api/obstacles/add", {"x": ox, "y": oy, "r": 0.46, "name": "e_a_pillar"})
        pillar = {"x": ox, "y": oy, "r": 0.46, "nudge_m": nudge}
    # Far LEFT clutter only — must not reseal RIGHT/TURN
    left_blocks = []
    for i, (fwd, lat, r) in enumerate([(1.2, 1.30, 0.36), (1.5, 1.50, 0.36)]):
        bx, by = body_frame(x, y, yaw, fwd, lat)
        name = f"e_a_left_{i}"
        api("POST", "/api/obstacles/add", {"x": bx, "y": by, "r": r, "name": name})
        left_blocks.append({"x": bx, "y": by, "r": r, "name": name})
    return {
        "pillar": pillar,
        "left_blocks": left_blocks,
        "removed_seals": stage1.get("right_seals"),
        "nudge_m": nudge,
    }


def run_scene_a(out_dir: str, duration: float = 22.0) -> Dict[str, Any]:
    t0 = time.time()
    path = os.path.join(out_dir, f"step3e_scene_a_{int(t0)}.jsonl")
    setup_nav()
    samples: List[Dict[str, Any]] = []
    stage1_done = False
    stage2_done = False
    stage1_info: Dict[str, Any] = {}
    left_since: Optional[float] = None
    authorized = False
    executed_right = False
    auth_then_right = False
    saw_left = False
    with open(path, "w", encoding="utf-8") as f:
        f.write(
            json.dumps(
                {
                    "type": "meta",
                    "scene": "A",
                    "goal": "LEFT_commit_then_LEFT_fail_RIGHT_valid_AUTHORIZED_switch",
                    "method": "SURGICAL_STAGE2_NO_CLEAR",
                },
                ensure_ascii=False,
            )
            + "\n"
        )
        end = time.time() + duration
        while time.time() < end:
            s = sample(t0)
            x, y, yaw = get_pose()
            s["pose"] = {"x": x, "y": y, "yaw": yaw}
            samples.append(s)
            f.write(json.dumps(s, ensure_ascii=False) + "\n")
            mode = str(s.get("maneuver_mode") or "")
            t = float(s["t"])

            if (not stage1_done) and t >= 1.2:
                stage1_info = scene_a_stage1(x, y, yaw)
                stage1_done = True
                f.write(
                    json.dumps(
                        {"type": "inject", "stage": 1, "t": t, "info": stage1_info, "pose": s["pose"]},
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                print(f"[A] S1 force_LEFT at t={t:.2f} pose=({x:.2f},{y:.2f})")

            if "LEFT" in mode:
                saw_left = True
                if left_since is None:
                    left_since = t
                    print(f"[A] LOCAL_LEFT at t={t:.2f}")

            if (
                stage1_done
                and (not stage2_done)
                and left_since is not None
                and (t - left_since) >= 0.28
            ):
                info2 = scene_a_stage2(x, y, yaw, stage1_info)
                stage2_done = True
                f.write(
                    json.dumps(
                        {"type": "inject", "stage": 2, "t": t, "info": info2, "pose": s["pose"]},
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                print(f"[A] S2 open_RIGHT/clog_LEFT at t={t:.2f} held={t - left_since:.2f}s")

            sw = s.get("side_switch") or {}
            ev = s.get("recent_events") or []
            sel_r = str(s.get("selector_reason") or "")
            if (
                sw.get("authorized")
                or sw.get("status") == "AUTHORIZED"
                or "SIDE_SWITCH_AUTHORIZED" in ev
                or "SIDE_SWITCH_EXECUTED" in ev
                or "POLICY_AUTHORIZED_SWITCH" in sel_r
            ):
                authorized = True
            c = s.get("commitment") or {}
            if int(c.get("switch_count") or 0) >= 1:
                authorized = True
            if "RIGHT" in mode:
                executed_right = True
                if authorized or "POLICY_AUTHORIZED_SWITCH" in sel_r:
                    auth_then_right = True
            time.sleep(0.05)
        summary = {
            "type": "summary",
            "scene": "A",
            "samples": len(samples),
            "stage1": stage1_done,
            "stage2": stage2_done,
            "saw_left": saw_left,
            "authorized_seen": authorized,
            "executed_local_right": executed_right,
            "auth_then_right": auth_then_right,
            "pass": bool(
                stage2_done and saw_left and authorized and executed_right and auth_then_right
            ),
        }
        f.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_A", summary)
    print("WROTE", path)
    return summary


def run_scene_b(out_dir: str) -> Dict[str, Any]:
    """Reuse mid_inject hard — expect DENY / no LOCAL_RIGHT."""
    import subprocess

    script = os.path.join(os.path.dirname(__file__), "_trace_phase4_side_switch.py")
    proc = subprocess.run(
        [
            sys.executable,
            script,
            "--scenario",
            "mid_inject",
            "--duration",
            "16",
            "--out",
            out_dir,
        ],
        cwd=os.path.dirname(os.path.dirname(__file__)),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    print((proc.stdout or "")[-800:])
    if proc.returncode not in (0, None):
        print("mid_inject rc", proc.returncode)
    files = sorted(
        [p for p in os.listdir(out_dir) if p.startswith("mid_inject_") and p.endswith(".jsonl") and "soft" not in p]
    )
    if not files:
        return {"pass": False, "reason": "no mid_inject file"}
    src = os.path.join(out_dir, files[-1])
    dst = os.path.join(out_dir, f"step3e_scene_b_{int(time.time())}.jsonl")
    rights = 0
    auth_true = 0
    deny_alt = 0
    with open(src, encoding="utf-8") as fin, open(dst, "w", encoding="utf-8") as fout:
        fout.write(json.dumps({"type": "meta", "scene": "B", "source": files[-1]}, ensure_ascii=False) + "\n")
        for line in fin:
            fout.write(line)
            try:
                o = json.loads(line)
            except Exception:
                continue
            if o.get("type") != "sample":
                continue
            if "RIGHT" in str(o.get("maneuver_mode") or "") or o.get("selector_selected") == "RIGHT":
                rights += 1
            sw = o.get("side_switch") or {}
            if sw.get("authorized") is True:
                auth_true += 1
            reason = sw.get("reason") or sw.get("primary_reason") or ""
            if reason == "ALTERNATIVE_PROBE_INVALID":
                deny_alt += 1
        summary = {
            "type": "summary",
            "scene": "B",
            "RIGHT_samples": rights,
            "authorized_true": auth_true,
            "deny_alt_probe_ticks": deny_alt,
            "pass": rights == 0 and auth_true == 0,
        }
        fout.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_B", summary)
    print("WROTE", dst)
    return summary


def run_scene_c(out_dir: str) -> Dict[str, Any]:
    import subprocess

    script = os.path.join(os.path.dirname(__file__), "_trace_phase4_side_switch.py")
    proc = subprocess.run(
        [
            sys.executable,
            script,
            "--scenario",
            "mid_inject_soft",
            "--duration",
            "14",
            "--out",
            out_dir,
        ],
        cwd=os.path.dirname(os.path.dirname(__file__)),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    print((proc.stdout or "")[-800:])
    files = sorted(
        [p for p in os.listdir(out_dir) if p.startswith("mid_inject_soft_") and p.endswith(".jsonl")]
    )
    src = os.path.join(out_dir, files[-1])
    dst = os.path.join(out_dir, f"step3e_scene_c_{int(time.time())}.jsonl")
    rights = 0
    still_valid = 0
    auth_true = 0
    with open(src, encoding="utf-8") as fin, open(dst, "w", encoding="utf-8") as fout:
        fout.write(json.dumps({"type": "meta", "scene": "C", "source": files[-1]}, ensure_ascii=False) + "\n")
        for line in fin:
            fout.write(line)
            try:
                o = json.loads(line)
            except Exception:
                continue
            if o.get("type") != "sample":
                continue
            if "RIGHT" in str(o.get("maneuver_mode") or ""):
                rights += 1
            sw = o.get("side_switch") or {}
            if sw.get("authorized") is True:
                auth_true += 1
            if (sw.get("reason") or sw.get("primary_reason") or "") == "CURRENT_SIDE_STILL_VALID":
                still_valid += 1
        summary = {
            "type": "summary",
            "scene": "C",
            "RIGHT_samples": rights,
            "authorized_true": auth_true,
            "still_valid_ticks": still_valid,
            "pass": rights == 0 and auth_true == 0,
        }
        fout.write(json.dumps(summary, ensure_ascii=False) + "\n")
    print("SUMMARY_C", summary)
    print("WROTE", dst)
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", choices=["A", "B", "C", "all"], default="all")
    ap.add_argument("--out", default=r"d:\Cursor\AGV项目\V0.1仿真版\docs\_phase4_trace")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    results = []
    if args.scene in ("A", "all"):
        results.append(run_scene_a(args.out))
    if args.scene in ("B", "all"):
        results.append(run_scene_b(args.out))
    if args.scene in ("C", "all"):
        results.append(run_scene_c(args.out))
    ok = all(r.get("pass") for r in results if isinstance(r, dict))
    print("OVERALL", "PASS" if ok else "FAIL", results)
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
