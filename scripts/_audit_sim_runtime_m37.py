#!/usr/bin/env python3
"""M3.7 simulator runtime smoke — single instance, build commit, physics health."""

from __future__ import annotations

import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = "http://127.0.0.1:19999"


def _git_head() -> str:
    try:
        r = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            timeout=3,
        )
        if r.returncode == 0:
            return r.stdout.strip()
    except Exception:
        pass
    return "unknown"


def _port_pids(port: int) -> list[int]:
    pids: list[int] = []
    try:
        out = subprocess.check_output(["netstat", "-ano"], text=True, errors="replace")
        token = f":{port}"
        for line in out.splitlines():
            if token in line and "LISTENING" in line.upper():
                parts = line.split()
                if parts:
                    try:
                        pids.append(int(parts[-1]))
                    except ValueError:
                        continue
    except Exception:
        pass
    return list(dict.fromkeys(pids))


def _get(path: str) -> dict:
    with urllib.request.urlopen(f"{BASE}{path}", timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    head = _git_head()
    pids = _port_pids(19999)
    print("=" * 60)
    print(" M3.7 SIM RUNTIME AUDIT")
    print("=" * 60)
    print(f" git HEAD: {head}")
    print(f" port 19999 LISTENING PID(s): {pids or 'NONE'}")

    if len(pids) != 1:
        print(f" FAIL: expected exactly 1 sim on 19999, found {len(pids)}")
        return 1

    try:
        st = _get("/api/state?lite=1")
    except Exception as e:
        print(f" FAIL: /api/state?lite=1 unreachable: {e}")
        return 1

    build = st.get("build_commit")
    rt = st.get("sim_runtime") or {}
    hz = rt.get("physics_hz")
    dt_ms = rt.get("physics_dt_ms")
    pid = rt.get("process_pid")

    print(f" build_commit API: {build}")
    print(f" process_pid API:  {pid}")
    print(f" physics_hz:       {hz}")
    print(f" physics_dt_ms:    {dt_ms}")

    ok = True
    if build != head:
        print(f" FAIL: build_commit mismatch API={build} git={head}")
        ok = False
    if pid != pids[0]:
        print(f" WARN: API pid {pid} != netstat pid {pids[0]}")
    if hz is None or float(hz) < 5.0:
        print(f" FAIL: physics_hz too low ({hz})")
        ok = False
    if dt_ms is None or float(dt_ms) > 200.0:
        print(f" FAIL: physics_dt_ms too high ({dt_ms})")
        ok = False

    t0_x = (st.get("agv") or {}).get("x")
    time.sleep(2.0)
    st2 = _get("/api/state?lite=1")
    t2_x = (st2.get("agv") or {}).get("x")
    nav = st2.get("nav") or {}
    progress = float(nav.get("path_progress_s") or 0.0)
    print(f" agv.x delta (2s idle): {None if t0_x is None or t2_x is None else round(float(t2_x) - float(t0_x), 4)}")
    print(f" path_progress_s: {progress}")

    runtime_ep = _get("/api/nav/runtime")
    if not runtime_ep.get("sim_runtime"):
        print(" FAIL: /api/nav/runtime missing sim_runtime")
        ok = False

    if ok:
        print(" PASS: M3.7 runtime smoke")
        return 0
    print(" FAIL: M3.7 runtime smoke")
    return 1


if __name__ == "__main__":
    sys.exit(main())
