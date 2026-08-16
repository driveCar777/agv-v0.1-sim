"""agv_simulation 单测 — 完全离线。"""

import os

from agv_simulation.scenario_loader import (
    load_scenario,
    scenario_to_obstacle_list,
)


def test_load_empty_room():
    here = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    path = os.path.join(here, "agv_simulation", "scenarios", "empty_room.yaml")
    s = load_scenario(path)
    assert s.name == "empty_room"
    assert len(s.obstacles) >= 4
    assert s.start_pose == (0.0, 0.0, 0.0)


def test_load_narrow_passage():
    here = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    path = os.path.join(here, "agv_simulation", "scenarios", "narrow_passage.yaml")
    s = load_scenario(path)
    assert s.name == "narrow_passage"


def test_load_dynamic_obstacle():
    here = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    path = os.path.join(here, "agv_simulation", "scenarios", "dynamic_obstacle.yaml")
    s = load_scenario(path)
    assert s.name == "dynamic_obstacle"
    assert len(s.dynamic_obstacles) >= 1


def test_scenario_to_obstacle_list():
    here = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    path = os.path.join(here, "agv_simulation", "scenarios", "empty_room.yaml")
    s = load_scenario(path)
    obs = scenario_to_obstacle_list(s)
    assert len(obs) == len(s.obstacles)
    assert all(len(o) == 3 for o in obs)


def test_to_dict_roundtrip():
    here = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    path = os.path.join(here, "agv_simulation", "scenarios", "empty_room.yaml")
    s = load_scenario(path)
    d = s.to_dict()
    assert d["name"] == "empty_room"
    assert "map_size" in d
    assert "obstacles" in d