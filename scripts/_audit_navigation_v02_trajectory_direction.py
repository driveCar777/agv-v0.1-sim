#!/usr/bin/env python3
"""M3.8.1 FAILURE-FIRST trajectory direction forensics.

Reads existing JSONL. Does not modify planner / safety / thresholds.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws" / "src" / "agv_bridge"
sys.path.insert(0, str(BRIDGE))

from agv_bridge.nav_trajectory_direction import summarize_rows  # noqa: E402


def _load(path: Path) -> List[dict]:
    return [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]


def _print_file(path: Path, rep: Dict[str, Any]) -> None:
    print("-" * 60)
    print(f"FILE: {path.name}")
    print(f"  samples_with_pose={rep['samples_with_pose']}  with_poses={rep['samples_with_trajectory_poses']}")
    print(f"  classification={rep['classification_counts']}")
    print(f"  source={rep['source_counts']}")
    print(f"  history_source_frames={rep['history_source_frames']}")
    print(f"  reanchored_frames={rep['reanchored_frames']}  max_shift_m={rep['max_reanchor_shift_m']}")
    print(f"  direction_dot min/mean={rep['direction_dot_min']}/{rep['direction_dot_mean']}")
    print(f"  angle≈180° count={rep['direction_angle_near_180_count']}")
    print(f"  vx>0 AND backward={rep['vx_positive_backward_count']}")
    print(f"  age_ms_max={rep['trajectory_age_ms_max']}  anchor_error_m_max={rep['anchor_error_m_max']}")
    print(f"  collisions={rep['collisions']}")
    print(f"  FIRST_TRAJECTORY_LAG={rep['first_trajectory_lag']}")
    print(f"  FIRST_DIRECTION_FAILURE seq={rep['first_direction_failure_seq']}")
    print(f"  FIRST_COMMAND_TRAJECTORY_INCONSISTENCY seq={rep['first_command_inconsistency_seq']}")
    # sample a few backward / retreat frames
    shown = 0
    for fr in rep["frames"]:
        if fr["classification"] in ("TRAJECTORY_BACKWARD", "TRAJECTORY_STALE") or fr.get("history_source") or fr.get("command_trajectory_inconsistency"):
            print(
                f"    seq={fr['seq']} cls={fr['classification']} src={fr['source']} "
                f"dot={fr.get('direction_dot')} ang={fr.get('direction_angle_deg')} "
                f"fwd={fr.get('forward_fraction')} back={fr.get('backward_fraction')} "
                f"vx={fr['vehicle'].get('vx')} reanchor={fr.get('reanchored')} shift={fr.get('reanchor_shift_m')} "
                f"probe={fr.get('probe_side')} commit={fr.get('commit_side')} exec={fr.get('execution_side')}"
            )
            shown += 1
            if shown >= 8:
                break


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", nargs="+", default=[])
    ap.add_argument("--latest-online", action="store_true")
    args = ap.parse_args()
    logdir = ROOT / "logs" / "navigation_v02"
    paths: List[Path] = [Path(p) for p in args.jsonl]
    if args.latest_online or not paths:
        for tag in ("ONLINE-LEFT-static-left", "ONLINE-RIGHT-static-right", "ONLINE-BOTH-blocked"):
            cands = sorted(logdir.glob(f"{tag}-*.jsonl"))
            if cands:
                paths.append(cands[-1])
        open_cands = sorted(logdir.glob("M32-OPEN-STRAIGHT-open-*.jsonl"))
        if open_cands:
            paths.append(open_cands[-1])

    print("=" * 60)
    print(" M3.8.1 TRAJECTORY DIRECTION FORENSICS")
    print("=" * 60)

    all_rep: List[Dict[str, Any]] = []
    for p in paths:
        if not p.exists():
            print(f"MISSING {p}")
            continue
        rows = _load(p)
        rep = summarize_rows(rows)
        all_rep.append(rep)
        _print_file(p, rep)

    # Aggregate failure-first
    crit = []
    safety = []
    incon = []
    unver = []
    for p, rep in zip(paths, all_rep):
        if rep["collisions"]:
            crit.append(f"{p.name}: collision frames={rep['collisions']}")
        if rep["vx_positive_backward_count"]:
            crit.append(
                f"{p.name}: COMMAND_TRAJECTORY_INCONSISTENCY vx>0 AND backward n={rep['vx_positive_backward_count']}"
            )
        if rep["history_source_frames"]:
            incon.append(
                f"{p.name}: HISTORY source drawn as FUTURE_LOCAL_PHYSICAL n={rep['history_source_frames']} src={rep['source_counts']}"
            )
        if rep["reanchored_frames"] and rep["max_reanchor_shift_m"] > 0.25:
            incon.append(
                f"{p.name}: reanchor shift max={rep['max_reanchor_shift_m']}m (may mask stale planner)"
            )
        if rep["samples_with_trajectory_poses"] == 0:
            unver.append(f"{p.name}: no trajectory poses in JSONL")
        if rep["classification_counts"].get("TRAJECTORY_UNKNOWN"):
            unver.append(f"{p.name}: TRAJECTORY_UNKNOWN n={rep['classification_counts']['TRAJECTORY_UNKNOWN']}")

    print("=" * 60)
    print(" AGGREGATE")
    print(" CRITICAL:", crit or "none in these JSONL")
    print(" SAFETY:", safety or "none in these JSONL")
    print(" INCONSISTENCY:", incon or "none")
    print(" UNVERIFIED:", unver or "none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
