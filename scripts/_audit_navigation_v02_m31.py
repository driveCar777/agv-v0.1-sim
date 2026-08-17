#!/usr/bin/env python3
"""M3.1 smoke audit — scenario injector + trace/analyzer schema."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BRIDGE = os.path.join(ROOT, "ros2_ws", "src", "agv_bridge")
if BRIDGE not in sys.path:
    sys.path.insert(0, BRIDGE)


def _load(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


class TestM31ScenarioInjector(unittest.TestCase):
    def test_all_scenes_defined(self) -> None:
        from agv_bridge.nav_scenario_injector import ALL_SCENES, SCENARIOS

        self.assertEqual(len(ALL_SCENES), 7)
        for sid in ALL_SCENES:
            self.assertIn(sid, SCENARIOS)
            self.assertIn("x", SCENARIOS[sid].start)
            self.assertIn("x", SCENARIOS[sid].goal)

    def test_baseline_path_reachable_offline(self) -> None:
        from agv_bridge.nav_geometry import DEFAULT_GEOM
        from agv_bridge.nav_scenario_injector import BASELINE_GOAL, BASELINE_START
        from agv_bridge.sim_world import SimWorld

        w = SimWorld()
        path = w.plan_path(
            (BASELINE_START["x"], BASELINE_START["y"]),
            (BASELINE_GOAL["x"], BASELINE_GOAL["y"]),
            robot_r=DEFAULT_GEOM.planner_radius,
        )
        self.assertTrue(path and len(path) >= 3)

    def test_path_point_at_distance(self) -> None:
        from agv_bridge.nav_scenario_injector import BASELINE_GOAL, BASELINE_START, path_point_at_distance
        from agv_bridge.sim_world import SimWorld

        w = SimWorld()
        x, y, yaw = path_point_at_distance(
            w,
            (BASELINE_START["x"], BASELINE_START["y"]),
            (BASELINE_GOAL["x"], BASELINE_GOAL["y"]),
            2.0,
        )
        self.assertIsInstance(x, float)
        self.assertIsInstance(yaw, float)

    def test_sim_world_scenario_mover(self) -> None:
        from agv_bridge.sim_world import SimWorld

        w = SimWorld()
        r = w.add_scenario_mover("t1", 1.0, 2.0, 0.3, 0.5, 0.0)
        self.assertTrue(r.get("success"))
        self.assertEqual(len(w.scenario_mover_list()), 1)
        w.step_scenario_movers(0.1)
        self.assertNotEqual(w.scenario_mover_list()[0]["x"], 1.0)
        clr = w.clear_scenario()
        self.assertEqual(clr.get("movers_removed"), 1)


class TestM31Analyzer(unittest.TestCase):
    def test_analyzer_timeline_keys(self) -> None:
        analyze_mod = _load("analyze_v02", os.path.join(ROOT, "scripts", "_analyze_navigation_v02_trace.py"))
        row = {
            "ts": 1.0,
            "seq": 0,
            "scene": "LIVE-00",
            "nav_state": "tracking",
            "stop_reason": "NONE",
            "planner_state": "NORMAL",
            "vehicle_vx": 0.1,
            "front_near": 10.0,
        }
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False, encoding="utf-8") as fh:
            fh.write(json.dumps(row) + "\n")
            path = fh.name
        try:
            report = analyze_mod.analyze(__import__("pathlib").Path(path))
            self.assertIn("timeline", report)
            self.assertIn("T0", report["timeline"])
            self.assertIn("stages", report)
        finally:
            os.unlink(path)


def main() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    ok = result.wasSuccessful()
    print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
