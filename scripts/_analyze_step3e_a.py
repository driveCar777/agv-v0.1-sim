# -*- coding: utf-8 -*-
import json
import sys
from collections import Counter

p = sys.argv[1]
print("file", p)
auth_ticks = []
right_ticks = []
reasons = Counter()
probes = Counter()
events = Counter()
after2 = False
for line in open(p, encoding="utf-8"):
    o = json.loads(line)
    if o.get("type") == "inject" and o.get("stage") == 2:
        after2 = True
        print("S2@", o.get("t"), "nudge", (o.get("info") or {}).get("nudge_m"))
        continue
    if o.get("type") == "summary":
        print("SUMMARY", o)
        continue
    if "maneuver_mode" not in o:
        continue
    sw = o.get("side_switch") or {}
    pr = o.get("probe") or {}
    reasons[sw.get("reason") or sw.get("status")] += 1
    probes[(pr.get("left"), pr.get("right"), pr.get("turn"))] += 1
    for e in o.get("recent_events") or []:
        events[e] += 1
    if sw.get("authorized") or sw.get("status") == "AUTHORIZED":
        auth_ticks.append(o.get("t"))
    if "RIGHT" in str(o.get("maneuver_mode") or ""):
        right_ticks.append(
            (
                o.get("t"),
                o.get("selector_reason"),
                sw.get("authorized"),
                sw.get("reason"),
                (pr.get("left"), pr.get("right"), pr.get("turn")),
            )
        )
print("auth_ticks", auth_ticks[:8], "n=", len(auth_ticks))
print("first_right", right_ticks[:3])
print("reasons", reasons.most_common(8))
if after2:
    print("probes_all", probes.most_common(6))
print("events", events.most_common(10))
