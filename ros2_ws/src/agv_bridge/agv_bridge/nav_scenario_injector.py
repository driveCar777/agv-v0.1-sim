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
    inject_min_progress_m: float = 0.0
    inject_min_vx: float = 0.0
    test_class: str = "BASELINE"
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
    pillar_lat_m: float = 0.0,
) -> None:
    """Place pillar on path (optional lateral offset); seal one lateral side (LEFT or RIGHT)."""
    if abs(pillar_lat_m) > 1e-6:
        px, py = body_frame(px, py, yaw, 0.0, pillar_lat_m)
    _add_obs(client, px, py, 0.42, f"{prefix}_pillar")
    # Vehicle body frame: +lat = left (+y when yaw=0). block_side RIGHT seals vehicle right (-lat).
    lat_sign = -1.0 if block_side.upper() == "RIGHT" else 1.0
    for i, (fwd, lat_off, r) in enumerate([(0.85, 0.85, 0.40), (1.15, 1.05, 0.38), (0.65, 0.95, 0.36)]):
        bx, by = body_frame(px, py, yaw, fwd, lat_sign * lat_off)
        _add_obs(client, bx, by, r, f"{prefix}_seal_{i}")


def _inject_at(
    client: LiveClient,
    world: SimWorld,
    start: Pt,
    goal: Pt,
    along_m: float,
    fn: Callable[[LiveClient, float, float, float, str], None],
    prefix: str,
) -> None:
    px, py, seg_yaw = path_point_at_distance(world, start, goal, along_m)
    fn(client, px, py, seg_yaw, prefix)


def _inject_static_left_at(client: LiveClient, px: float, py: float, seg_yaw: float, prefix: str) -> None:
    # Pillar biased to vehicle RIGHT so LEFT (+y) detour remains open.
    _pillar_and_side_seal(client, px, py, seg_yaw, block_side="RIGHT", prefix=prefix, pillar_lat_m=-0.55)


def _inject_static_right_at(client: LiveClient, px: float, py: float, seg_yaw: float, prefix: str) -> None:
    _pillar_and_side_seal(client, px, py, seg_yaw, block_side="LEFT", prefix=prefix, pillar_lat_m=0.55)


def _inject_both_at(client: LiveClient, px: float, py: float, seg_yaw: float, prefix: str) -> None:
    _add_obs(client, px, py, 0.44, f"{prefix}_front")
    for i, (fwd, lat, r) in enumerate(
        [(0.85, 0.80, 0.40), (1.10, 1.00, 0.38), (0.85, -0.80, 0.40), (1.10, -1.00, 0.38), (1.25, 0.0, 0.42)]
    ):
        bx, by = body_frame(px, py, seg_yaw, fwd, lat)
        _add_obs(client, bx, by, r, f"{prefix}_box_{i}")


def _inject_dynamic_cross_at(client: LiveClient, px: float, py: float, seg_yaw: float, prefix: str) -> None:
    cross_v = 0.55
    vx = -math.sin(seg_yaw) * cross_v
    vy = math.cos(seg_yaw) * cross_v
    sx, sy = body_frame(px, py, seg_yaw, 0.0, -1.2)
    client.post(
        "/api/scenario/mover/add",
        {"name": f"{prefix}_cross", "x": sx, "y": sy, "r": 0.32, "vx": vx, "vy": vy, "kind": "dynamic_cross"},
    )


def _inject_dynamic_away_at(client: LiveClient, px: float, py: float, seg_yaw: float, prefix: str) -> None:
    spd = 0.45
    vx = math.cos(seg_yaw) * spd
    vy = math.sin(seg_yaw) * spd
    client.post(
        "/api/scenario/mover/add",
        {"name": f"{prefix}_away", "x": px, "y": py, "r": 0.30, "vx": vx, "vy": vy, "kind": "dynamic_away"},
    )


def _inject_field_at(client: LiveClient, px: float, py: float, seg_yaw: float, prefix: str) -> None:
    _add_obs(client, px, py, 0.38, f"{prefix}_pillar")
    for i, (fwd, lat, r) in enumerate([(0.75, -0.72, 0.36), (1.05, -0.88, 0.34)]):
        bx, by = body_frame(px, py, seg_yaw, fwd, lat)
        _add_obs(client, bx, by, r, f"{prefix}_r{i}")
    bx, by = body_frame(px, py, seg_yaw, 1.0, 1.05)
    _add_obs(client, bx, by, 0.34, f"{prefix}_left_far")


def _inject_ahead_of_pose(
    client: LiveClient,
    world: SimWorld,
    start: dict,
    goal: dict,
    pose: Pt,
    ahead_m: float,
    kind: str,
    prefix: str,
) -> None:
    """Place obstacle ahead_m along global path from current pose (open-runway friendly)."""
    s = (float(start["x"]), float(start["y"]))
    g = (float(goal["x"]), float(goal["y"]))
    path = world.plan_path(s, g, robot_r=DEFAULT_GEOM.planner_radius)
    if not path or len(path) < 2:
        px, py, seg_yaw = path_point_at_distance(world, s, g, ahead_m)
        _inject_at(client, world, s, g, ahead_m, _KIND_MAP[kind], prefix)
        return
    best_i = 0
    best_d = 1e9
    for i, p in enumerate(path):
        d = math.hypot(p[0] - pose[0], p[1] - pose[1])
        if d < best_d:
            best_d = d
            best_i = i
    acc = 0.0
    target = max(0.0, ahead_m)
    inject_i = best_i
    for i in range(best_i, len(path) - 1):
        seg = math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if acc + seg >= target:
            inject_i = i + 1
            break
        acc += seg
        inject_i = i + 1
    px, py = float(path[inject_i][0]), float(path[inject_i][1])
    nxt = path[min(inject_i + 1, len(path) - 1)]
    seg_yaw = math.atan2(nxt[1] - path[inject_i][1], nxt[0] - path[inject_i][0])
    _KIND_MAP[kind](client, px, py, seg_yaw, prefix)


_KIND_MAP = {
    "static_left": _inject_static_left_at,
    "static_right": _inject_static_right_at,
    "both": _inject_both_at,
    "dynamic_cross": _inject_dynamic_cross_at,
    "dynamic_away": _inject_dynamic_away_at,
    "field": _inject_field_at,
}


def _route_inject(start: dict, goal: dict, along_m: float, kind: str, prefix: str):
    s = (float(start["x"]), float(start["y"]))
    g = (float(goal["x"]), float(goal["y"]))

    def fn(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
        _inject_at(client, world, s, g, along_m, _KIND_MAP[kind], prefix)

    return fn


def _route_inject_ahead(start: dict, goal: dict, ahead_m: float, kind: str, prefix: str):
    def fn(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
        _inject_ahead_of_pose(client, world, start, goal, pose, ahead_m, kind, prefix)

    return fn


def _inject_static_left(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    _route_inject(BASELINE_START, BASELINE_GOAL, 2.4, "static_left", "live01")(client, world, pose, yaw)


def _inject_static_right(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    _route_inject(BASELINE_START, BASELINE_GOAL, 2.4, "static_right", "live02")(client, world, pose, yaw)


def _inject_both_blocked(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    _route_inject(BASELINE_START, BASELINE_GOAL, 2.2, "both", "live03")(client, world, pose, yaw)


def _inject_dynamic_cross(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    _route_inject(BASELINE_START, BASELINE_GOAL, 2.8, "dynamic_cross", "live04")(client, world, pose, yaw)


def _inject_dynamic_away(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    _route_inject(BASELINE_START, BASELINE_GOAL, 2.0, "dynamic_away", "live05")(client, world, pose, yaw)


def _inject_field_p0d1(client: LiveClient, world: SimWorld, pose: Pt, yaw: float) -> None:
    _route_inject(BASELINE_START, BASELINE_GOAL, 1.6, "field", "live06")(client, world, pose, yaw)


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
    "OBS-OPEN-LEFT": ScenarioSpec(
        scene_id="OBS-OPEN-LEFT",
        label="OPEN-STATIC-LEFT",
        description="M32 runway + static obstacle; LEFT detour feasible",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=2.0,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 2.3, "static_left", "obs_open_l"),
    ),
    "OBS-OPEN-RIGHT": ScenarioSpec(
        scene_id="OBS-OPEN-RIGHT",
        label="OPEN-STATIC-RIGHT",
        description="M32 runway + static obstacle; RIGHT detour feasible",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=2.0,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 2.3, "static_right", "obs_open_r"),
    ),
    "OBS-OPEN-BOTH-BLOCKED": ScenarioSpec(
        scene_id="OBS-OPEN-BOTH-BLOCKED",
        label="OPEN-BOTH-BLOCKED",
        description="M32 runway + three-side block → recovery exhausted",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=2.0,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 2.1, "both", "obs_open_bb"),
        expect_navigation_failed=True,
    ),
    "OBS-OPEN-DYNAMIC-CROSS": ScenarioSpec(
        scene_id="OBS-OPEN-DYNAMIC-CROSS",
        label="OPEN-DYNAMIC-CROSS",
        description="M32 runway + mover crosses path",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=1.5,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 5.0, "dynamic_cross", "obs_open_cross"),
    ),
    "OBS-OPEN-DYNAMIC-AWAY": ScenarioSpec(
        scene_id="OBS-OPEN-DYNAMIC-AWAY",
        label="OPEN-DYNAMIC-AWAY",
        description="M32 runway + mover leaving path ahead",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=1.5,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 4.0, "dynamic_away", "obs_open_away"),
    ),
    "OBS-OPEN-FIELD-P0D1": ScenarioSpec(
        scene_id="OBS-OPEN-FIELD-P0D1",
        label="OPEN-FIELD-P0D1",
        description="M32 runway field-like heading vs footprint divergence",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=1.5,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 3.5, "field", "obs_open_p0d1"),
        test_class="GLOBAL_PLANNING_AT_START",
    ),
    # M3.8 — ONLINE injection: vehicle must move before obstacle appears (local avoidance test).
    "SCENE-CURVE-01": ScenarioSpec(
        scene_id="SCENE-CURVE-01",
        label="CURVE-APPROACH",
        description="M3.9 open runway to offset goal — curvature preview then turn",
        start=dict(M32_OPEN_START),
        goal={"x": 0.0, "y": 8.0, "poi": "CURVE_01"},
        map_scene="m32_open_straight",
        test_class="MOTION_CURVE",
    ),
    "ONLINE-LEFT": ScenarioSpec(
        scene_id="ONLINE-LEFT",
        label="ONLINE-STATIC-LEFT",
        description="M3.8 baseline cruise then inject LEFT-feasible obstacle on unchanged global path",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=9999.0,
        inject_min_progress_m=1.2,
        inject_min_vx=0.05,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 2.5, "static_left", "online_l"),
        test_class="LOCAL_AVOIDANCE_ONLINE",
    ),
    "ONLINE-RIGHT": ScenarioSpec(
        scene_id="ONLINE-RIGHT",
        label="ONLINE-STATIC-RIGHT",
        description="M3.8 baseline cruise then inject RIGHT-feasible obstacle on unchanged global path",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=9999.0,
        inject_min_progress_m=1.2,
        inject_min_vx=0.05,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 2.5, "static_right", "online_r"),
        test_class="LOCAL_AVOIDANCE_ONLINE",
    ),
    "ONLINE-BOTH-BLOCKED": ScenarioSpec(
        scene_id="ONLINE-BOTH-BLOCKED",
        label="ONLINE-BOTH-BLOCKED",
        description="M3.8 baseline cruise then inject both-blocked obstacle — expect safe stop",
        start=dict(M32_OPEN_START),
        goal=dict(M32_OPEN_GOAL),
        map_scene="m32_open_straight",
        inject_delay_s=9999.0,
        inject_min_progress_m=1.2,
        inject_min_vx=0.05,
        inject_fn=_route_inject_ahead(M32_OPEN_START, M32_OPEN_GOAL, 2.3, "both", "online_bb"),
        test_class="LOCAL_AVOIDANCE_ONLINE",
        expect_navigation_failed=True,
    ),
}

ALL_SCENES = ["LIVE-00", "LIVE-01", "LIVE-02", "LIVE-03", "LIVE-04", "LIVE-05", "LIVE-06"]
M32_SCENES = ["M32-OPEN-STRAIGHT"]
OBS_OPEN_SCENES = [
    "OBS-OPEN-LEFT",
    "OBS-OPEN-RIGHT",
    "OBS-OPEN-BOTH-BLOCKED",
    "OBS-OPEN-DYNAMIC-CROSS",
    "OBS-OPEN-DYNAMIC-AWAY",
    "OBS-OPEN-FIELD-P0D1",
]
ONLINE_SCENES = ["ONLINE-LEFT", "ONLINE-RIGHT", "ONLINE-BOTH-BLOCKED"]
MOTION_SCENES = ["SCENE-CURVE-01"]
M33_SCENES = M32_SCENES + OBS_OPEN_SCENES + ONLINE_SCENES


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
        "inject_min_progress_m": spec.inject_min_progress_m,
        "inject_min_vx": spec.inject_min_vx,
        "test_class": spec.test_class,
        "expect_navigation_failed": spec.expect_navigation_failed,
    }
