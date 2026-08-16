"""Real-run incident replay: forward → stall/reverse → capture evidence (no algo changes)."""
from __future__ import annotations

import json
import time
import urllib.request

BASE = "http://127.0.0.1:19999"


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=8) as r:
        return json.loads(r.read().decode())


def post(path: str, obj=None):
    data = json.dumps({} if obj is None else obj).encode()
    req = urllib.request.Request(
        BASE + path, data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=45) as r:
        return json.loads(r.read().decode())


def reset():
    post("/api/cancel")
    time.sleep(0.15)
    post("/api/obstacles/clear", {})
    post("/api/scene", {"id": "indoor_office"})
    time.sleep(0.5)
    post("/api/nav/debug/level", {"level": "FULL"})
    return get("/api/state")


def main():
    s = reset()
    x0, y0 = float(s["agv"]["x"]), float(s["agv"]["y"])
    # Place a wall-like barrier ahead but allow some forward motion first
    post("/api/obstacles/add", {"x": x0 + 2.2, "y": y0, "r": 0.55, "name": "wall_a"})
    post("/api/obstacles/add", {"x": x0 + 2.2, "y": y0 + 0.45, "r": 0.45, "name": "wall_b"})
    post("/api/obstacles/add", {"x": x0 + 2.2, "y": y0 - 0.45, "r": 0.45, "name": "wall_c"})

    post("/api/nav/plan", {"x": x0 + 4.0, "y": y0})
    post("/api/nav/confirm")

    timeline = []
    saw_forward = False
    t_end = time.time() + 14.0
    while time.time() < t_end:
        d = get("/api/nav/debug")["debug"]
        st = d.get("status") or {}
        evs = [e.get("event") for e in (d.get("events") or [])]
        row = {
            "t": time.time(),
            "phase": st.get("phase"),
            "vx": st.get("vx"),
            "motion": st.get("motion_state"),
            "progress": st.get("path_progress_s"),
            "stop": st.get("stop_reason"),
            "why": (d.get("diagnostics") or {}).get("primary_reason"),
            "front": (d.get("radar") or {}).get("front_near"),
            "actual_clr": (d.get("wall_approach") or {}).get("actual_clearance"),
            "path_clr": (d.get("path_vs_actual_clearance") or {}).get("planned_path_clearance"),
            "lat": (d.get("controller") or {}).get("lateral_err"),
            "fwd": (d.get("local_planner") or {}).get("forward_trajectory"),
            "anomaly_rev": (d.get("anomalies") or {}).get("reverse_with_forward_available"),
            "last_events": evs[-6:],
        }
        timeline.append(row)
        if (st.get("vx") or 0) > 0.05 and st.get("phase") == "forward":
            saw_forward = True
        if "REVERSE_START" in evs or st.get("phase") == "reverse_escape" or "PATH_PROGRESS_STALLED" in evs:
            # wait a bit more for post window
            time.sleep(2.0)
            break
        time.sleep(0.35)

    time.sleep(1.0)
    d = get("/api/nav/debug")["debug"]
    cap = post("/api/nav/debug/capture", {}).get("capture") or {}
    inc = d.get("incident") or cap.get("incident")
    evs = [e.get("event") for e in (d.get("events") or [])]

    # Find first anomaly time relative to session
    first_anom = None
    for name in (
        "DIVERGENCE_START",
        "PATH_PROGRESS_STALLED",
        "APPROACHING_OBSTACLE_WHILE_TRACKING",
        "SAFETY_BLOCK",
        "REVERSE_START",
        "RECOVERY_START",
        "REVERSE_WITH_FORWARD_AVAILABLE",
    ):
        for e in d.get("events") or []:
            if e.get("event") == name:
                first_anom = e
                break
        if first_anom:
            break

    report = {
        "saw_forward": saw_forward,
        "final_phase": (d.get("status") or {}).get("phase"),
        "final_vx": (d.get("status") or {}).get("vx"),
        "final_why": (d.get("diagnostics") or {}).get("primary_reason"),
        "forward_trajectory": (d.get("local_planner") or {}).get("forward_trajectory"),
        "reverse_with_forward_available": (d.get("anomalies") or {}).get("reverse_with_forward_available"),
        "first_anomaly": first_anom,
        "first_divergence": (d.get("tracking_divergence") or {}).get("first_divergence"),
        "incident_trigger": (inc or {}).get("trigger") if isinstance(inc, dict) else None,
        "incident_summary": (inc or {}).get("summary") if isinstance(inc, dict) else None,
        "incident_keyframes_n": len((inc or {}).get("keyframes") or []) if isinstance(inc, dict) else 0,
        "recovery_attempts": d.get("recovery_attempts") or [],
        "reverse_decisions": d.get("reverse_decisions") or [],
        "candidate_switches": len(d.get("candidate_switch_log") or []),
        "pose_trace_n": len(d.get("pose_trace") or []),
        "events_tail": evs[-20:],
        "timeline_compact": [
            {
                "phase": r["phase"],
                "vx": r["vx"],
                "progress": r["progress"],
                "front": r["front"],
                "why": r["why"],
                "events": r["last_events"],
            }
            for r in timeline[::3]
        ],
        "checks": {
            "actual_trace_ok": len(d.get("pose_trace") or []) >= 10,
            "has_incident_or_events": bool(inc) or "PATH_PROGRESS_STALLED" in evs or "REVERSE_START" in evs,
            "path_vs_actual_fields": "path_vs_actual_clearance" in d,
            "steering_fields": "steering" in d,
        },
    }
    report["OVERALL"] = (
        "PASS"
        if report["saw_forward"]
        and report["checks"]["actual_trace_ok"]
        and report["checks"]["has_incident_or_events"]
        else "FAIL"
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["OVERALL"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
