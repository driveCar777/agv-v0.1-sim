"""仿真场景加载器 — 纯 Python，可单测。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class Obstacle:
    x: float
    y: float
    r: float = 0.05
    kind: str = "static"


@dataclass
class DynamicObstacle:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    r: float = 0.2
    path: List[tuple] = field(default_factory=list)


@dataclass
class Scenario:
    name: str
    map_size: tuple = (4.0, 4.0)
    start_pose: tuple = (0.0, 0.0, 0.0)
    pickup_pose: tuple = (1.5, 0.0, 0.0)
    user_pose: tuple = (-1.5, 0.0, 3.14)
    obstacles: List[Obstacle] = field(default_factory=list)
    dynamic_obstacles: List[DynamicObstacle] = field(default_factory=list)
    expected_path: Optional[List[tuple]] = None
    max_duration_s: float = 60.0
    fail_conditions: List[str] = field(default_factory=list)
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "map_size": list(self.map_size),
            "start_pose": list(self.start_pose),
            "pickup_pose": list(self.pickup_pose),
            "user_pose": list(self.user_pose),
            "obstacles": [
                {"x": o.x, "y": o.y, "r": o.r, "kind": o.kind}
                for o in self.obstacles
            ],
            "dynamic_obstacles": [
                {"x": d.x, "y": d.y, "vx": d.vx, "vy": d.vy, "r": d.r}
                for d in self.dynamic_obstacles
            ],
            "max_duration_s": self.max_duration_s,
            "fail_conditions": list(self.fail_conditions),
            "description": self.description,
        }


def load_scenario(path: str) -> Scenario:
    """从 YAML 加载场景。"""
    import yaml
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"invalid scenario file: {path}")
    obstacles = [Obstacle(**o) for o in data.get("obstacles", [])]
    dyn = [DynamicObstacle(**d) for d in data.get("dynamic_obstacles", [])]
    return Scenario(
        name=str(data.get("name", Path(path).stem)),
        map_size=tuple(data.get("map_size", [4.0, 4.0])),
        start_pose=tuple(data.get("start_pose", [0.0, 0.0, 0.0])),
        pickup_pose=tuple(data.get("pickup_pose", [1.5, 0.0, 0.0])),
        user_pose=tuple(data.get("user_pose", [-1.5, 0.0, 3.14])),
        obstacles=obstacles,
        dynamic_obstacles=dyn,
        max_duration_s=float(data.get("max_duration_s", 60.0)),
        fail_conditions=list(data.get("fail_conditions", [])),
        description=str(data.get("description", "")),
    )


def scenario_to_obstacle_list(s: Scenario) -> List[tuple]:
    """转为 (x, y, r) 元组列表，给 lidar / ultrasonic 用。"""
    return [(o.x, o.y, o.r) for o in s.obstacles]