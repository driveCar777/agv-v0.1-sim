#!/usr/bin/env python3
"""M3.5 Web Simulator smoke audit — static assets, JS parse, API shape.

FIRST BAD COMMIT (blank page): dca5e55 — sim3d.js line 757 used Python `if` syntax.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
WWW = ROOT / "ros2_ws" / "src" / "delivery_web" / "www"
DEFAULT_BASE = "http://127.0.0.1:19999"

JS_FILES = ("component_system.js", "nav_ui.js", "sim3d.js")
REQUIRED_STATE_KEYS = ("agv", "nav", "scene", "map")
FIRST_BAD_COMMIT = "dca5e55a97a83df80653331ebad3932f6928f7e5"


def _get(url: str, timeout: float = 5.0) -> Tuple[int, bytes, str]:
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, resp.read(), resp.headers.get("Content-Type", "")
    except urllib.error.HTTPError as e:
        return e.code, e.read(), e.headers.get("Content-Type", "")


def _node_check(path: Path) -> Tuple[bool, str]:
    try:
        proc = subprocess.run(
            ["node", "--check", str(path)],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
    except FileNotFoundError:
        return False, "node not found — install Node.js for JS syntax audit"
    if proc.returncode == 0:
        return True, "ok"
    err = (proc.stderr or proc.stdout or "").strip()
    return False, err.splitlines()[-1] if err else "syntax error"


def _html_script_refs(html: str) -> List[str]:
    return re.findall(r'<script[^>]+src="([^"]+)"', html)


def audit(base: str) -> Dict[str, Any]:
    report: Dict[str, Any] = {
        "base": base,
        "first_bad_commit": FIRST_BAD_COMMIT,
        "checks": [],
        "pass": True,
    }
    checks: List[Dict[str, Any]] = report["checks"]

    def record(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"name": name, "ok": ok, "detail": detail})
        if not ok:
            report["pass"] = False

    # Root redirect → sim_main.html
    code, body, ctype = _get(base + "/")
    record("GET /", code == 200, f"status={code} content-type={ctype}")
    if code != 200:
        return report

    html = body.decode("utf-8", errors="replace")
    record("sim_main served", "Sim3DView" in html or "view3d" in html, "canvas entry present")

    # Local Three.js (offline)
    code3, _, _ = _get(base + "/vendor/three.min.js")
    record("vendor/three.min.js", code3 == 200, f"status={code3}")

    for src in _html_script_refs(html):
        path = src.split("?")[0]
        if path.startswith("http"):
            continue
        c, _, _ = _get(base + "/" + path.lstrip("/"))
        record(f"static {path}", c == 200, f"status={c}")

    # JS syntax (local files — authoritative even if server stale)
    for name in JS_FILES:
        ok, msg = _node_check(WWW / name)
        record(f"node --check {name}", ok, msg)

    # API
    code, raw, _ = _get(base + "/api/state")
    record("GET /api/state", code == 200, f"status={code} bytes={len(raw)}")
    if code == 200:
        try:
            snap = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as e:
            record("api/state JSON", False, str(e))
            snap = {}
        else:
            missing = [k for k in REQUIRED_STATE_KEYS if k not in snap]
            record("api/state keys", not missing, f"missing={missing}" if missing else "ok")
            agv = snap.get("agv") or {}
            record("agv pose", "x" in agv and "y" in agv, f"x={agv.get('x')} y={agv.get('y')}")

    return report


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.5 web sim smoke audit")
    ap.add_argument("--base", default=DEFAULT_BASE, help="Web sim base URL")
    ap.add_argument("--json", action="store_true", help="Print JSON report")
    args = ap.parse_args()

    report = audit(args.base.rstrip("/"))
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        print(f"M3.5 Web Sim Audit — {args.base}")
        print(f"FIRST BAD COMMIT: {FIRST_BAD_COMMIT} (sim3d.js Python-style if)")
        for c in report["checks"]:
            mark = "PASS" if c["ok"] else "FAIL"
            detail = f" — {c['detail']}" if c.get("detail") else ""
            print(f"  [{mark}] {c['name']}{detail}")
        print("VERDICT:", "PASS" if report["pass"] else "FAIL")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
