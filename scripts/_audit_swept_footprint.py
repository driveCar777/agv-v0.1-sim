#!/usr/bin/env python3
"""P0-A — Swept footprint geometry audits (no policy/FSM behavior changes)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_footprint import (  # noqa: E402
    SWEPT_ANGULAR_STEP_RAD,
    SWEPT_SPATIAL_STEP_M,
    footprint_polygon_body,
    interpolate_poses,
    sample_rotation_sweep,
    sample_swept_footprint,
    trajectory_collision,
    transform_footprint,
)
from agv_bridge.nav_geometry import DEFAULT_GEOM  # noqa: E402
from agv_bridge.nav_trajectory import (  # noqa: E402
    TrajectorySample,
    build_corridor_from_poses,
)


def _dd(vx, w, n=20, dt=0.1, x0=0.0, y0=0.0, yaw0=0.0):
    x, y, yaw = x0, y0, yaw0
    poses = [TrajectorySample(0.0, x, y, yaw, vx, w)]
    for i in range(n):
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw += w * dt
        poses.append(TrajectorySample((i + 1) * dt, x, y, yaw, vx, w))
    return poses


def main() -> int:
    fails = []
    geom = DEFAULT_GEOM

    # A — Straight rectangle footprint
    poly = footprint_polygon_body(geom)
    if len(poly) != 4:
        fails.append(f"A: polygon corners={len(poly)}")
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if max(xs) - min(xs) < 0.9 or max(ys) - min(ys) < 0.5:
        fails.append(f"A: rectangle extent unexpected xs={xs} ys={ys}")

    # B — Left arc outer expansion (left edge further out than straight ribbon half-width alone)
    left_arc = _dd(0.15, 0.35, n=18)
    corr_l = build_corridor_from_poses(left_arc, source="LEFT", status="VALID")
    # At end pose, max body-left among left_edge should exceed half-width of body alone
    end = left_arc[-1]
    c, s = math.cos(end.yaw), math.sin(end.yaw)
    max_by = -1e9
    for px, py in corr_l.left_edge[-5:]:
        by = -s * (px - end.x) + c * (py - end.y)
        max_by = max(max_by, by)
    if max_by < 0.5 * geom.width - 0.02:
        fails.append(f"B: left arc outer left_edge body_y={max_by:.3f} too small")

    # C — Right arc
    right_arc = _dd(0.15, -0.35, n=18)
    corr_r = build_corridor_from_poses(right_arc, source="RIGHT", status="VALID")
    end = right_arc[-1]
    c, s = math.cos(end.yaw), math.sin(end.yaw)
    min_by = 1e9
    for px, py in corr_r.right_edge[-5:]:
        by = -s * (px - end.x) + c * (py - end.y)
        min_by = min(min_by, by)
    if min_by > -(0.5 * geom.width - 0.02):
        fails.append(f"C: right arc outer right_edge body_y={min_by:.3f} too large")

    # D — Interpolation
    sparse = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    dense = interpolate_poses(sparse, spatial_step_m=SWEPT_SPATIAL_STEP_M)
    if len(dense) < 10:
        fails.append(f"D: interpolate 1m → {len(dense)} samples (expect >=10)")

    # E — Narrow obstacle: centerline clear, footprint hits
    # Obstacle at body-left of a straight path (y=+0.30) — center at y=0 misses, left side hits
    obs = {(round(0.5, 2), round(0.30, 2))}

    def collide(px, py):
        return (round(px, 2), round(py, 2)) in obs or math.hypot(px - 0.5, py - 0.30) < 0.06

    straight = _dd(0.20, 0.0, n=8)
    # Center points along path should not hit, but footprint should
    center_hit = any(collide(p.x, p.y) for p in straight)
    res = trajectory_collision(straight, collide, geom, margin_m=0.0)
    if center_hit:
        fails.append("E: setup invalid — centerline already hits")
    if not res.collision:
        fails.append("E: expected footprint collision on lateral obstacle")

    # F — Radius false negative: circle around center clear of point at front-left corner zone
    # Place obstacle near front-left of pose (0,0,0): (0.50, 0.25)
    def collide_fl(px, py):
        return math.hypot(px - 0.50, py - 0.26) < 0.05

    poses = [TrajectorySample(0.0, 0.0, 0.0, 0.0)]
    # Broad-phase local_radius around origin: obstacle at ~0.56m > 0.24 → radius says safe
    r = geom.local_radius
    if math.hypot(0.50, 0.26) <= r:
        fails.append("F: setup — obstacle inside local_radius")
    res_f = trajectory_collision(poses, collide_fl, geom)
    if not res_f.collision:
        fails.append("F: footprint must catch front-left hit that radius misses")

    # G — Rotation sweep area
    rot = sample_rotation_sweep(0.0, 0.0, 0.0, math.pi / 2, geom)
    if len(rot) < 5:
        fails.append(f"G: rotation samples={len(rot)}")
    # Corners must move (not only center)
    p0 = rot[0].polygon[0]
    p1 = rot[-1].polygon[0]
    if math.hypot(p0[0] - p1[0], p0[1] - p1[1]) < 0.3:
        fails.append("G: front-left corner barely moved during 90° sweep")

    # Corridor geometry model flag
    if corr_l.geometry_model != "swept_footprint_polygon":
        fails.append(f"corridor model={corr_l.geometry_model}")

    # Single geometry truth sizes
    if abs(geom.length - 1.05) > 1e-6 or abs(geom.width - 0.55) > 1e-6:
        fails.append("geometry size drift from AMB-150 baseline")

    print("=== P0-A Swept Footprint Audit ===")
    print(f"  spatial_step={SWEPT_SPATIAL_STEP_M}m angular_step={math.degrees(SWEPT_ANGULAR_STEP_RAD):.1f}deg")
    print(f"  geom L={geom.length} W={geom.width} bumper_l={geom.bumper_l}")
    if fails:
        for f in fails:
            print("FAIL:", f)
        return 1
    print("PASS: A–G")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
