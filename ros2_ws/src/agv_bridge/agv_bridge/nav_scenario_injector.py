"""M3.1 deterministic LIVE scenario definitions for Navigation V0.2 validation.

Obstacles inject through SimWorld dyn_obstacles / scenario_movers → LiDAR / WorldModel.
Does not bypass navigation perception.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Protocol, Tuple

from agv_bridge.nav_geometry import DEFAULT_GEOM
from agv_bridge.sim_world import SimWorld

Pt = Tuple[float, float]

# M3.2 open straight baseline — 50m runway, no temp/dynamic obstacles.
M32_OPEN_START = {"poi": "M32_A", "x": -25.0, "y": 0.0}
M32_OPEN_GOAL = {"poi": "M32_B", "x": 25.0, "y": 0.0}

# Shared indoor baseline corridor — LM1→LM5 (M3.1; not valid for tracking proof).
BASELINE_START = {"poi": "LM1", "x": 6.26, "y": -0.659}
BASELINE_GOAL = {"poi": "LM5", "x": 4.506, "y": 0.265}


class LiveClient(Protocol):
    def post(self, path: str, body: Optional[dict] = None) -> dict: ...
    def get(self, path: str) -> dict: ...
    def mock_control(self, payload: dict) -> dict: ...


def body_frame(x: float, y: float, yaw: float, fwd: float, lat: float) -> Pt:
    return (
        x + fwd * math.cos(yaw) - lat * math.sin(yaw),
        y + fwd * math.sin(yaw) + lat * math.cos(yaw),
    )


def start_yaw_toward(start: Pt, goal: Pt) -> float:
    return math.atan2(goal[1] - start[1], goal[0] - start[0])


def path_point_at_distance(world: SimWorld, start: Pt, goal: Pt, dist_m: float) -> Tuple[float, float, float]:
    """Return (x, y, segment_yaw) dist_m along planned global path."""
    path = world.plan_path(start, goal, robot_r=DEFAULT_GEOM.planner_radius)
    if not path or len(path) < 2:
        mx = (start[0] + goal[0]) * 0.5
        my = (start[1] + goal[1]) * 0.5
        return mx, my, start_yaw_toward(start, goal)
    acc = 0.0
    for i in range(1, len(path)):
        seg = math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        acc += seg
        if acc >= dist_m or i >= len(path) - 1:
            nxt = path[min(i + 1, len(path) - 1)]
            yaw = math.atan2(nxt[1] - path[i][1], nxt[0] - path[i][0])
            return float(path[i][0]), float(path[i][1]), float(yaw)
    lx, ly = path[-1]
    return float(lx), float(ly), start_yaw_toward(path[-2], path[-1])


@dataclass
class ScenarioSpec:
    scene_id: str
    label: str
    description: str
    start: Dict[str, float]
    goal: Dict[str, float]
    disable_actors: bool = True
    map_scene: str = "indoor_office"
    pre_obstacles: List[Dict[str, Any]] = field(default_factory=list)
    post_setup: Optional[Callable[[LiveClient, SimWorld, Pt, float], None]] = None
    inject_delay_s: float = 0.0
    inject_fn: Optional[Callable[[LiveClient, SimWorld, Pt, float], None]] = None
    expect_navigation_failed: bool = False


def _add_obs(client: LiveClient, x: float, y: float, r: float, name: str) -> None:
    client.post("/api/obstacles/add", {"x": x, "y": y, "r": r, "name": name})


def _pillar_and_side_seal(
    client: LiveClient,
    px: float,
    py: float,
    yaw: float,
    *,
    block_side: str,
    prefix: str,
) -> None:
    """Place pillar on path; seal one lateral side (LEFT or RIGHT)."""
    _add_obs(client, px, py, 0.42, f"{prefix}_pillar")
    lat_sign = 1.0 if block_side.upper() == "RIGHT" else -1.0
    for i, (fwd, lat_off, r) in enumerate([(0.85, 0.85, 0.40), (1.15, 1.05, 0.38), (0.65, 0.95, 0.36)]):
        bx, by = body_frame(px, py, yaw, fwd, lat_sign * lat_off)
        _add_obs(client, bx, by, r, f"{prefix}_seal_{i}")


def _inject_static_left(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    px, py, seg_yaw = path_point_at_distance(
        world, (BASELINE_START["x"], BASELINE_START["y"]), (BASELINE_GOAL["x"], BASELINE_GOAL["y"]), 2.4
    )
    _pillar_and_side_seal(client, px, py, seg_yaw, block_side="RIGHT", prefix="live01")


def _inject_static_right(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    px, py, seg_yaw = path_point_at_distance(
        world, (BASELINE_START["x"], BASELINE_START["y"]), (BASELINE_GOAL["x"], BASELINE_GOAL["y"]), 2.4
    )
    _pillar_and_side_seal(client, px, py, seg_yaw, block_side="LEFT", prefix="live02")


def _inject_both_blocked(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    px, py, seg_yaw = path_point_at_distance(
        world, (BASELINE_START["x"], BASELINE_START["y"]), (BASELINE_GOAL["x"], BASELINE_GOAL["y"]), 2.2
    )
    _add_obs(client, px, py, 0.44, "live03_front")
    for i, (fwd, lat, r) in enumerate(
        [(0.85, 0.80, 0.40), (1.10, 1.00, 0.38), (0.85, -0.80, 0.40), (1.10, -1.00, 0.38), (1.25, 0.0, 0.42)]
    ):
        bx, by = body_frame(px, py, seg_yaw, fwd, lat)
        _add_obs(client, bx, by, r, f"live03_box_{i}")


def _inject_dynamic_cross(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    px, py, seg_yaw = path_point_at_distance(
        world, (BASELINE_START["x"], BASELINE_START["y"]), (BASELINE_GOAL["x"], BASELINE_GOAL["y"]), 2.8
    )
    # Cross path: velocity perpendicular to segment (lat + direction)
    cross_v = 0.55
    vx = -math.sin(seg_yaw) * cross_v
    vy = math.cos(seg_yaw) * cross_v
    lat_start = -1.2
    sx, sy = body_frame(px, py, seg_yaw, 0.0, lat_start)
    client.post(
        "/api/scenario/mover/add",
        {"name": "live04_cross", "x": sx, "y": sy, "r": 0.32, "vx": vx, "vy": vy, "kind": "dynamic_cross"},
    )


def _inject_dynamic_away(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    px, py, seg_yaw = path_point_at_distance(
        world, (BASELINE_START["x"], BASELINE_START["y"]), (BASELINE_GOAL["x"], BASELINE_GOAL["y"]), 2.0
    )
    # Ahead on path, moving away along +path direction
    spd = 0.45
    vx = math.cos(seg_yaw) * spd
    vy = math.sin(seg_yaw) * spd
    client.post(
        "/api/scenario/mover/add",
        {"name": "live05_away", "x": px, "y": py, "r": 0.30, "vx": vx, "vy": vy, "kind": "dynamic_away"},
    )


def _inject_field_p0d1(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    """Approximate field failure: close obstacle + right seal → heading turn + clearance drop."""
    px, py, seg_yaw = path_point_at_distance(
        world, (BASELINE_START["x"], BASELINE_START["y"]), (BASELINE_GOAL["x"], BASELINE_GOAL["y"]), 1.6
    )
    _add_obs(client, px, py, 0.38, "live06_pillar")
    for i, (fwd, lat, r) in enumerate([(0.75, -0.72, 0.36), (1.05, -0.88, 0.34)]):
        bx, by = body_frame(px, py, seg_yaw, fwd, lat)
        _add_obs(client, bx, by, r, f"live06_r{i}")
    # Narrow left gap only
    bx, by = body_frame(px, py, seg_yaw, 1.0, 1.05)
    _add_obs(client, bx, by, 0.34, "live06_left_far")


SCENARIOS: Dict[str, ScenarioSpec] = {
    "LIVE-00": ScenarioSpec(
        scene_id="LIVE-00",
        label="BASELINE",
        description="Reachable LM1→LM5, no obstacles, stable tracking",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
    ),
    "LIVE-01": ScenarioSpec(
        scene_id="LIVE-01",
        label="STATIC-LEFT",
        description="Pillar on path; RIGHT sealed; LEFT detour feasible",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
        inject_delay_s=1.5,
        inject_fn=_inject_static_left,
    ),
    "LIVE-02": ScenarioSpec(
        scene_id="LIVE-02",
        label="STATIC-RIGHT",
        description="Pillar on path; LEFT sealed; RIGHT detour feasible",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
        inject_delay_s=1.5,
        inject_fn=_inject_static_right,
    ),
    "LIVE-03": ScenarioSpec(
        scene_id="LIVE-03",
        label="BOTH-BLOCKED",
        description="Forward + both sides blocked → recovery exhausted",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
        inject_delay_s=1.5,
        inject_fn=_inject_both_blocked,
        expect_navigation_failed=True,
    ),
    "LIVE-04": ScenarioSpec(
        scene_id="LIVE-04",
        label="DYNAMIC-CROSS",
        description="Scripted mover crosses global path (LiDAR-visible)",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
        inject_delay_s=1.2,
        inject_fn=_inject_dynamic_cross,
    ),
    "LIVE-05": ScenarioSpec(
        scene_id="LIVE-05",
        label="DYNAMIC-AWAY",
        description="Obstacle ahead moving away — prefer wait/resume",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
        inject_delay_s=1.2,
        inject_fn=_inject_dynamic_away,
    ),
    "LIVE-06": ScenarioSpec(
        scene_id="LIVE-06",
        label="FIELD-P0D1",
        description="Field-like: heading vs footprint clearance divergence",
        start=dict(BASELINE_START),
        goal=dict(BASELINE_GOAL),
        inject_delay_s=1.0,
        inject_fn=_inject_field_p0d1,
    ),
    "M32-OPEN-STRAIGHT": ScenarioSpec(
        scene_id="M32-OPEN-STRAIGHT",
        label="OPEN-STRAIGHT",
        description="M3.2 open runway M32_A→M32_B, no obstacles, global tracking only",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
    ),
}

ALL_SCENES = ["LIVE-00", "LIVE-01", "LIVE-02", "LIVE-03", "LIVE-04", "LIVE-05", "LIVE-06"]
M32_SCENES = ["M32-OPEN-STRAIGHT"]


def reset_scenario(client: LiveClient, world: SimWorld) -> dict:
    for p in ("/api/cancel", "/api/nav/stop"):
        try:
            client.post(p, {})
        except Exception:
            pass
    client.post("/api/scenario/clear", {})
    client.post("/api/obstacles/clear", {})
    client.mock_control(
        {
            "vx": 0.0,
            "w": 0.0,
            "r_vx": 0.0,
            "r_w": 0.0,
            "is_stop": False,
            "emergency": False,
            "soft_emc": False,
            "block_reason": 0,
        }
    )
    return {"success": True}


def apply_scenario(client: LiveClient, world: SimWorld, scene_id: str) -> dict:
    spec = SCENARIOS.get(scene_id)
    if spec is None:
        return {"success": False, "message": f"unknown scene {scene_id}"}

    reset_scenario(client, world)
    client.post("/api/scene", {"id": spec.map_scene})
    time.sleep(0.45)
    if spec.disable_actors:
        client.post("/api/scenario/actors", {"enabled": False})
    else:
        client.post("/api/scenario/actors", {"enabled": True})

    sx, sy = float(spec.start["x"]), float(spec.start["y"])
    gx, gy = float(spec.goal["x"]), float(spec.goal["y"])
    syaw = start_yaw_toward((sx, sy), (gx, gy))

    # Teleport only after scene reset; always clear cmd/stop latches.
    client.mock_control(
        {
            "set_pose": {"x": sx, "y": sy, "angle": syaw},
            "is_stop": False,
            "emergency": False,
            "soft_emc": False,
            "block_reason": 0,
            "vx": 0.0,
            "w": 0.0,
        }
    )
    time.sleep(0.35)

    for obs in spec.pre_obstacles:
        client.post("/api/obstacles/add", obs)

    plan_wrap = client.post("/api/nav/plan", {"x": gx, "y": gy, "yaw": syaw})
    plan = plan_wrap.get("api") if isinstance(plan_wrap.get("api"), dict) else plan_wrap
    rc = plan.get("ret_code")
    if rc is None or int(rc) != 0:
        return {
            "success": False,
            "scene": scene_id,
            "failure_class": "BASELINE_PATH_FAILURE",
            "plan": plan_wrap,
        }

    client.post("/api/nav/confirm", {})
    time.sleep(0.6)

    return {
        "success": True,
        "scene": scene_id,
        "label": spec.label,
        "start": {"x": sx, "y": sy, "yaw": syaw, "poi": spec.start.get("poi")},
        "goal": {"x": gx, "y": gy, "poi": spec.goal.get("poi")},
        "waypoints": plan.get("waypoints"),
        "inject_delay_s": spec.inject_delay_s,
        "expect_navigation_failed": spec.expect_navigation_failed,
    }
