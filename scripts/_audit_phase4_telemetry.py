#!/usr/bin/env python3
"""STEP 3B telemetry audit — read-only API checks + mid_inject sequence compare."""
from __future__ import annotations

import json
import sys
import time
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:19999"
ROOT = Path(__file__).resolve().parents[1]


def api(method: str, path: str, body=None):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=8) as r:
        return json.loads(r.read().decode("utf-8"))


def check_schema() -> dict:
    wrap = api("GET", "/api/nav/debug")
    d = wrap.get("debug") or wrap
    p4 = d.get("phase4") or {}
    need = [
        "commitment",
        "probe",
        "side_switch",
        "sides",
        "ownership",
        "schema_version",
    ]
    missing = [k for k in need if k not in p4 and k not in d]
    commit = d.get("commitment") or p4.get("commitment") or {}
    probe = d.get("probe") or p4.get("probe") or {}
    sw = d.get("side_switch") or p4.get("side_switch") or {}
    own = d.get("ownership") or p4.get("ownership") or {}
    return {
        "ok": not missing and commit.get("implemented") is False and probe.get("implemented") is False,
        "missing": missing,
        "commitment_implemented": commit.get("implemented"),
        "probe_implemented": probe.get("implemented"),
        "side_switch_auth_status": sw.get("authorization_status") or sw.get("status"),
        "authority_current": own.get("side_switch_authority_current"),
        "authority_target": own.get("side_switch_authority_target"),
        "phase4_build_ms": (p4.get("performance") or {}).get("phase4_build_ms"),
        "has_nav_policy": "nav_policy" in d,
        "has_local_maneuver": "local_maneuver" in d,
    }


def extract_seq(jsonl_path: Path):
    modes, sels, reasons, allows, wsigns = [], [], [], [], []
    switches = []
    for line in jsonl_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        if r.get("type") == "side_switch":
            switches.append((r.get("t"), r.get("from"), r.get("to")))
            continue
        if r.get("type") != "sample":
            continue
        modes.append(r.get("maneuver_mode"))
        sels.append(r.get("selector_selected"))
        reasons.append(r.get("selector_reason"))
        allows.append(r.get("allow_side_compare"))
        w = r.get("mppi_w")
        try:
            wsigns.append(0 if w is None or abs(float(w)) < 0.05 else (1 if float(w) > 0 else -1))
        except Exception:
            wsigns.append(0)
    # compress mode sequence
    mode_seq = []
    for m in modes:
        if not mode_seq or mode_seq[-1] != m:
            mode_seq.append(m)
    return {"mode_seq": mode_seq, "switches": switches, "n": len(modes)}


def main() -> int:
    print("SCHEMA", json.dumps(check_schema(), ensure_ascii=False))
    # expect caller already wrote step3b traces; compare to STEP2 if present
    step2 = ROOT / "docs" / "_phase4_trace" / "mid_inject_1786888402.jsonl"
    step3b = sorted((ROOT / "docs" / "_phase4_trace").glob("step3b_mid_inject_*.jsonl"))
    if step2.exists() and step3b:
        a = extract_seq(step2)
        b = extract_seq(step3b[-1])
        print("STEP2", a)
        print("STEP3B", b)
        # behavior equivalence: both should have LEFT->RIGHT switch; mode seq contains LOCAL_LEFT then LOCAL_RIGHT
        ok = bool(a["switches"]) and bool(b["switches"])
        ok = ok and a["switches"][0][1:] == b["switches"][0][1:]
        print("BEHAVIOR_EQUIVALENCE_SWITCH", ok, "step2", a["switches"][:1], "step3b", b["switches"][:1])
        return 0 if ok else 2
    print("NO_COMPARE_YET")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
