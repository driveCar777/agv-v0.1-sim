#!/usr/bin/env python3
"""P0-C — Kinematic path validator audits (Tests A–L). Telemetry-only; no control claims."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_footprint import footprint_polygon_body  # noqa: E402
from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.nav_kinematic import (  # noqa: E402
    KinematicPathValidator,
    W_MAX_CONTROL,
    V_MIN_FORWARD,
    control_limits,
    discrete_curvature,
    polygon_extents,
    validator_enabled,
)


def _ok(cond: bool, msg: str, fails: list) -> None:
    if not cond:
        fails.append(msg)


def _line(n: int, step: float = 0.5) -> list:
    return [(i * step, 0.0) for i in range(n + 1)]


def _arc(r: float, sweep_deg: float = 90.0, ds: float = 0.15) -> list:
    sweep = math.radians(sweep_deg)
    n = max(8, int(abs(r) * abs(sweep) / ds))
    pts = []
    for i in range(n + 1):
        a = sweep * i / n
        pts.append((r * math.sin(a), r * (1.0 - math.cos(a))))
    return pts


def _l_corner(arm: float = 4.0, step: float = 0.25) -> list:
    pts = []
    x = 0.0
    while x <= arm + 1e-9:
        pts.append((round(x, 3), 0.0))
        x += step
    y = step
    while y <= arm + 1e-9:
        pts.append((round(arm, 3), round(y, 3)))
        y += step
    return pts


def main() -> int:
    fails: list = []
    print("=== P0-C Kinematic Path Audit (A–L) ===")
    lim = control_limits()
    ext = polygon_extents()
    print(f"  v_max={lim['v_max']} w_max={lim['w_max']} v_min={lim['v_min_forward']} v_ref={lim['v_ref']}")
    print(f"  polygon front={ext['front_extent_m']:.3f} rear={ext['rear_extent_m']:.3f} bumper_sample={ext['bumper_sample_m']}")
    _ok(abs(ext["front_extent_m"] - 0.5 * DEFAULT_GEOM.length) < 1e-6, "geom: polygon not length-clamped", fails)
    _ok(abs(ext["bumper_sample_m"] - 0.55) < 1e-9, "geom: bumper_l drifted", fails)
    _ok(abs(DEFAULT_GEOM.length - 1.05) < 1e-9, "geom: length drifted", fails)

    v = KinematicPathValidator()

    # A straight
    a = v.validate(_line(20, 0.5), path_revision=1)
    _ok(a.status == "VALID", f"A status={a.status} {a.reason}", fails)
    _ok((a.max_abs_curvature or 0) < 0.05, f"A kappa={a.max_abs_curvature}", fails)
    _ok(a.min_turn_radius_m is None, f"A R should be inf/None got {a.min_turn_radius_m}", fails)
    _ok(a.kinematic_valid is True, "A kinematic_valid", fails)
    _ok(a.controls_vehicle is False, "A must not control", fails)
    print(f"A straight kappa={a.max_abs_curvature} R={a.min_turn_radius_m} ->", "PASS" if not any(x.startswith("A") for x in fails) else "FAIL")

    # B gentle R=2
    b = v.validate(_arc(2.0, 90.0), path_revision=2)
    _ok(b.status == "VALID", f"B status={b.status} {b.reason}", fails)
    _ok(b.kinematic_valid is True, "B valid", fails)
    r_b = b.min_turn_radius_m
    _ok(r_b is not None and 1.4 < r_b < 2.8, f"B radius={r_b}", fails)
    print(f"B gentle R~2 kappa={b.max_abs_curvature} R={r_b} limited={b.speed_limited} ->", "PASS" if not any(x.startswith("B") for x in fails) else "FAIL")

    # C tight but slowable R=0.5 → κ=2, v_ref*κ=0.8 > w_max=0.42, v_allowed=0.21>0.04
    c = v.validate(_arc(0.50, 90.0, ds=0.08), path_revision=3)
    _ok(c.status == "VALID", f"C status={c.status} {c.reason} viol={c.violations}", fails)
    _ok(c.speed_limited is True, f"C should be speed_limited kappa={c.max_abs_curvature} w_req={c.max_required_w_at_reference_speed}", fails)
    _ok((c.feasible_speed_min_mps or 0) >= V_MIN_FORWARD - 1e-6, f"C vmin={c.feasible_speed_min_mps}", fails)
    print(f"C tight-slowable kappa={c.max_abs_curvature} w_req={c.max_required_w_at_reference_speed} vmin={c.feasible_speed_min_mps} ->", "PASS" if not any(x.startswith("C") for x in fails) else "FAIL")

    # D impossible short 90° arms
    d = v.validate(_l_corner(arm=0.20, step=0.05), path_revision=4)
    _ok(d.status == "INVALID", f"D status={d.status} reason={d.reason}", fails)
    _ok(d.reason in ("NO_BOUNDED_CURVATURE_TRANSITION", "HARD_CORNER", "CURVATURE_UNBOUNDED", "HEADING_REVERSAL"), f"D reason={d.reason}", fails)
    _ok(d.kinematic_valid is False, "D kinematic_valid false", fails)
    _ok(d.executable is False, "D executable", fails)
    print(f"D short-corner INVALID reason={d.reason} ->", "PASS" if not any(x.startswith("D") for x in fails) else "FAIL")

    # E 90° with long arms → fillet VALID
    e = v.validate(_l_corner(arm=4.0, step=0.25), path_revision=5)
    _ok(e.status == "VALID", f"E status={e.status} {e.reason} hard={e.hard_corner_count}", fails)
    _ok(e.transition_applied or (e.max_abs_curvature or 0) < 50, f"E transition={e.transition_applied} k={e.max_abs_curvature}", fails)
    _ok(e.hard_corner_count >= 1, f"E hard_corner_count={e.hard_corner_count}", fails)
    print(f"E 90deg long-arm status={e.status} trans={e.transition_applied} R={e.min_turn_radius_m} ->", "PASS" if not any(x.startswith("E") for x in fails) else "FAIL")

    # F inner obstacle: left turn, obstacle inside
    # Arc R=2 along +y; inner is toward center (0,2)
    f_path = _arc(2.0, 90.0)
    def inner_hit(x, y):
        return math.hypot(x - 0.0, y - 2.0) < 1.88  # centerline R=2; inner gap 0.12 < half-width 0.275
    f = v.validate(f_path, path_revision=6, collide=inner_hit)
    _ok(f.status == "INVALID", f"F status={f.status} {f.reason}", fails)
    _ok(f.reason == "FOOTPRINT_COLLISION" or f.swept_collision, f"F reason={f.reason} col={f.swept_collision}", fails)
    print(f"F inner FOOTPRINT status={f.status} reason={f.reason} ->", "PASS" if not any(x.startswith("F") for x in fails) else "FAIL")

    # G outer obstacle: same left arc, obstacle outside (right of start / outer front)
    def outer_hit(x, y):
        # outer of left turn is away from center (0,2): e.g. (2.0, -0.4) near start
        return math.hypot(x - 1.2, y + 0.45) < 0.55
    gres = v.validate(f_path, path_revision=7, collide=outer_hit)
    _ok(gres.status == "INVALID", f"G status={gres.status} {gres.reason}", fails)
    _ok(gres.reason == "FOOTPRINT_COLLISION" or gres.swept_collision, f"G reason={gres.reason}", fails)
    print(f"G outer FOOTPRINT status={gres.status} reason={gres.reason} ->", "PASS" if not any(x.startswith("G") for x in fails) else "FAIL")

    # H clearance < margin
    def clr_low(_x, _y):
        return 0.03
    h = v.validate(_line(16, 0.5), path_revision=8, clearance_at=clr_low)
    _ok(h.status == "DEGRADED", f"H status={h.status} {h.reason}", fails)
    _ok(h.kinematic_valid is None, f"H kinematic_valid should be null got {h.kinematic_valid}", fails)
    _ok("CLEARANCE_TOO_LOW" in h.violations or h.reason == "CLEARANCE_TOO_LOW", f"H viol={h.violations}", fails)
    print(f"H clearance DEGRADED min_cl={h.min_clearance_m} ->", "PASS" if not any(x.startswith("H") for x in fails) else "FAIL")

    # I no reverse generated
    _ok(a.needs_reverse_maneuver is False and e.needs_reverse_maneuver is False, "I reverse on forward path", fails)
    rev = v.validate([(0, 0), (2, 0), (0, 0.01)], path_revision=9)
    _ok(rev.needs_reverse_maneuver or rev.status == "INVALID", f"I 180-ish status={rev.status} rev={rev.needs_reverse_maneuver}", fails)
    _ok(rev.status != "VALID" or rev.needs_reverse_maneuver, "I must not VALID-forward a reversal", fails)
    print(f"I reverse-independent reversal status={rev.status} needs_rev={rev.needs_reverse_maneuver} ->", "PASS" if not any(x.startswith("I") for x in fails) else "FAIL")

    # J rotation cannot save short corner in a cage
    cage = _l_corner(arm=0.20, step=0.05)
    def cage_hit(x, y):
        return abs(x) < 0.8 and abs(y) < 0.8 and not (x > 0.05 and y > 0.05 and x < 0.25 and y < 0.25)
    j = v.validate(cage, path_revision=10, collide=cage_hit)
    _ok(j.status == "INVALID", f"J status={j.status}", fails)
    _ok(j.kinematic_valid is False, "J valid", fails)
    print(f"J rotation-not-escape status={j.status} rot_ok={j.rotation_sweep_valid} viol={j.violations} ->", "PASS" if not any(x.startswith("J") for x in fails) else "FAIL")

    # K speed profile drops into tight then recovers (tangent-out, no extra 90° polyline)
    k_path = _line(10, 0.4)
    sweep = math.pi / 3.0
    k_arc = [(4.0 + 0.5 * math.sin(a), 0.5 * (1.0 - math.cos(a))) for a in [sweep * i / 12.0 for i in range(1, 13)]]
    end = k_arc[-1]
    hx, hy = math.cos(sweep), math.sin(sweep)
    k_tail = [(end[0] + i * 0.35 * hx, end[1] + i * 0.35 * hy) for i in range(1, 12)]
    k = v.validate(k_path + k_arc + k_tail, path_revision=11)
    if k.speed_profile and k.speed_profile.v_max_mps:
        vmin = min(k.speed_profile.v_max_mps)
        vmax = max(k.speed_profile.v_max_mps)
        _ok(vmin < vmax - 0.02, f"K profile flat vmin={vmin} vmax={vmax}", fails)
    else:
        fails.append("K missing speed profile")
    print(f"K profile limited={k.speed_limited} from={k.speed_limited_from_m} vmin={k.feasible_speed_min_mps} ->", "PASS" if not any(x.startswith("K") for x in fails) else "FAIL")

    # L revision cache
    l1 = v.validate(_line(12, 0.5), path_revision=21)
    l1b = v.validate(_line(12, 0.5), path_revision=21)
    l2 = v.validate(_line(12, 0.5), path_revision=22)
    _ok(l1b.cache_hit is True, "L cache miss on same revision", fails)
    _ok(l2.cache_hit is False, "L cache hit on new revision", fails)
    _ok(l1.validation_id != l2.validation_id, "L validation_id not refreshed", fails)
    print(f"L cache hit={l1b.cache_hit} rev21→22 id {l1.validation_id}→{l2.validation_id} ->", "PASS" if not any(x.startswith("L") for x in fails) else "FAIL")

    # numeric stability
    z = v.validate([(0, 0)], path_revision=30)
    _ok(z.status in ("VALID", "INVALID"), f"one-point crash {z.status}", fails)
    z2 = v.validate([(0, 0), (0, 0), (1, 0)], path_revision=31)
    _ok(math.isfinite(z2.max_abs_curvature or 0), "dup NaN kappa", fails)
    _ok(discrete_curvature((0, 0), (1, 0), (2, 0)) == 0.0, "colinear kappa not 0", fails)
    left_k = discrete_curvature((0, 0), (1, 0), (1, 1))
    _ok(left_k > 0, f"LEFT kappa sign {left_k}", fails)

    # 50m path budget
    longp = _line(100, 0.5)
    tlong = v.validate(longp, path_revision=40)
    _ok(tlong.compute_ms < 500, f"50m too slow {tlong.compute_ms}ms", fails)
    print(f"  50m points={tlong.path_points} ms={tlong.compute_ms:.2f} swept={tlong.swept_samples}")

    os.environ["NAV_KINEMATIC_VALIDATOR"] = "0"
    _ok(validator_enabled() is False, "env off", fails)
    os.environ["NAV_KINEMATIC_VALIDATOR"] = "1"
    _ok(validator_enabled() is True, "env on", fails)

    if fails:
        print("FAILS:")
        for f in fails:
            print(" -", f)
        print("P0-C AUDIT = FAIL")
        return 1
    print("P0-C AUDIT = PASS (A–L)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
