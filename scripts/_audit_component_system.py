"""Component system static + API regression (no browser driver required)."""
from __future__ import annotations

import json
import re
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:19999"
WWW = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "delivery_web" / "www"


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=8) as r:
        return json.loads(r.read().decode())


def post(path: str, obj=None):
    data = json.dumps({} if obj is None else obj).encode()
    req = urllib.request.Request(
        BASE + path, data=data, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode())


def main():
    results = {}
    html = (WWW / "sim_main.html").read_text(encoding="utf-8")
    cs = (WWW / "component_system.js").read_text(encoding="utf-8")
    nu = (WWW / "nav_ui.js").read_text(encoding="utf-8")

    results["C1_scripts"] = {
        "ok": "component_system.js" in html and "nav_ui.js" in html and "AgvComponentSystem" in html,
    }
    results["C2_no_nav_debug_btn"] = {
        "ok": "btnDebug" not in html and "Nav Debug" not in html,
    }
    results["C3_picker_groups"] = {
        "ok": "pickerList" in html and "GROUP_ORDER" in html,
    }
    results["C4_no_minimize"] = {
        "ok": "collapse" not in cs and ".collapsed" not in html and "MINIMIZED" not in cs,
        "note": "legacy minimized field stripped on persist",
    }
    results["C5_layout_key"] = {"ok": "agv.component.layout" in cs}
    results["C6_resize_handles"] = {
        "ok": "resize-e" in cs and "resize-s" in cs and "resize-se" in cs,
    }
    results["C7_only_close_x"] = {"ok": 'title="关闭"' in cs and "collapse" not in cs}
    results["C8_timeline_scroll"] = {
        "ok": "EXECUTION TIMELINE" in nu and "scrollBody" in nu and "bb-timeline" in nu,
    }
    results["C9_curve_cards"] = {
        "ok": all(
            k in nu
            for k in (
                "LONGITUDINAL VELOCITY",
                "LONGITUDINAL ACCEL",
                "ANGULAR VELOCITY",
                "ANGULAR ACCEL",
                "TRACKING ERROR",
                "CLEARANCE",
                "PATH PROGRESS",
            )
        ),
    }
    results["C10_single_instance"] = {"ok": "cards.has(id)" in cs}
    results["C11_glass"] = {
        "ok": "backdrop-filter" in cs and "rgba(15, 23, 42" in cs,
    }
    results["C12_registry_ids"] = {
        "ok": all(
            x in nu
            for x in (
                "dbg_dock",
                "dbg_motion",
                "dbg_timeline",
                "dbg_incident",
                "dbg_lon_v",
                "nav_mission",
                "dbg_maneuver",
                "dbg_fwd_rev",
                "dbg_capture",
                "dbg_maneuver_tl",
            )
        ),
    }
    results["C13_drive_replay"] = {"ok": "DRIVE REPLAY" in nu and "incident" in nu}
    results["C14_z_index"] = {"ok": "bringToFront" in cs and "zCounter" in cs}

    # live API: physics unaffected by debug spam
    try:
        post("/api/nav/debug/level", {"level": "FULL"})
        a = get("/api/state")
        for _ in range(10):
            get("/api/nav/debug")
        time.sleep(0.3)
        b = get("/api/state")
        results["C15_physics_ok"] = {"ok": "agv" in a and "agv" in b}
        d = get("/api/nav/debug")["debug"]
        results["C16_debug_fields"] = {
            "ok": all(
                k in d
                for k in (
                    "telemetry",
                    "events",
                    "paths",
                    "wall_approach",
                    "steering",
                    "tracking_divergence",
                )
            ),
            "keys_sample": sorted(d.keys())[:20],
        }
        results["C17_actual_trace"] = {
            "ok": bool((d.get("paths") or {}).get("actual_trace") is not None or d.get("pose_trace") is not None),
        }
        results["C18_incident_slot"] = {"ok": "incident" in d or "incidents" in d}
    except urllib.error.URLError as e:
        results["C15_physics_ok"] = {"ok": False, "error": str(e)}
        results["C16_debug_fields"] = {"ok": False}
        results["C17_actual_trace"] = {"ok": False}
        results["C18_incident_slot"] = {"ok": False}

    # open/close semantics in JS source
    results["C19_open_closed_only"] = {
        "ok": "display:none" in cs.replace(" ", "") or 'display = "none"' in cs or "display: none" in cs,
    }
    results["C20_clamp"] = {"ok": "clampLayout" in cs and "minWidth" in cs or "minW" in cs}

    ok = all(v.get("ok") for v in results.values())
    print(json.dumps({"OVERALL": "PASS" if ok else "FAIL", "results": results}, indent=2, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
