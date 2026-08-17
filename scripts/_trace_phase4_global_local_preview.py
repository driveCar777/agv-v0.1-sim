#!/usr/bin/env python3
"""P0-B — Global vs Local diagnostic timeline (read-only HTTP).

Prints GLOBAL / LOCAL_MAX / SELECTED / STOP owner over time.
Does not change control. Requires web sim at BASE (default 127.0.0.1:19999).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Optional


def _get(url: str, timeout: float = 3.0) -> Dict[str, Any]:
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def _f(v: Any) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:19999")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--hz", type=float, default=2.0)
    args = ap.parse_args()
    base = args.base.rstrip("/")
    dt = 1.0 / max(0.5, float(args.hz))
    t0 = time.time()
    print("=== P0-B Global vs Local Timeline ===")
    print(f"BASE={base} window={args.seconds}s")
    print("TIMELINE")
    rows = 0
    try:
        while time.time() - t0 <= float(args.seconds):
            t = time.time() - t0
            try:
                st = _get(f"{base}/api/state")
                prev = _get(f"{base}/api/nav/preview")
                summ = _get(f"{base}/api/logs/summary")
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
                print(f"T+{t:5.2f}  FETCH_FAIL {e}")
                time.sleep(dt)
                continue
            nav = st.get("nav") or {}
            gref = (prev.get("global_reference") or nav.get("global_reference") or {})
            loc = (prev.get("local_candidates") or nav.get("local_candidates") or {})
            gvl = prev.get("global_vs_local") or {}
            agv = st.get("agv") or {}
            g_m = _f(gref.get("preview_m") if gref.get("preview_m") is not None else gvl.get("global_preview_m"))
            l_max = _f(loc.get("max_distance_m") if loc.get("max_distance_m") is not None else gvl.get("local_max_distance_m"))
            l_valid = loc.get("valid_count")
            l_count = loc.get("count")
            sel = (
                (nav.get("selected_local") or {}).get("candidate_id")
                or gvl.get("selected_candidate")
                or summ.get("current_candidate")
                or "—"
            )
            owner = summ.get("immediate_owner") or summ.get("likely_owner") or "—"
            vx = _f(agv.get("vx") if "vx" in agv else (st.get("nav") or {}).get("state_vx"))
            reason = gref.get("preview_reason") or gvl.get("global_preview_reason") or "—"
            print(
                f"T+{t:5.2f}  GLOBAL={g_m if g_m is not None else '—':>6}m  "
                f"LOCAL_MAX={l_max if l_max is not None else '—':>5}m  "
                f"LOCAL_VALID={l_valid}/{l_count}  SELECTED={sel}  "
                f"vx={vx if vx is not None else '—'}  "
                f"OWNER={owner}  reason={reason}"
            )
            rows += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    if rows == 0:
        print("NO_SAMPLES — is the web sim running?")
        return 2
    print(f"samples={rows}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
