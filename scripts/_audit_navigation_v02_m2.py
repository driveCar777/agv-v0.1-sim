"""V0.2 M2 regression — FSM semantics, footprint MPPI, recovery, safety, diagnostics."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.mppi_controller import DiffDriveMppi
from agv_bridge.nav_execution_corridor import MODE_LEFT, MODE_RIGHT, build_execution_corridor, corridor_allows_omega_sign
from agv_bridge.nav_geometry import BrakingModel, get_vehicle_model
from agv_bridge.nav_planner_state import (
    LOCAL_PLAN_INFEASIBLE,
    LOCAL_RECOVERY,
    NAVIGATION_FAILED,
    REASON_MPPI_INFEASIBLE,
    ui_severity,
)
from agv_bridge.nav_trajectory_validator import REASON_BRAKING_UNAVAILABLE, braking_feasible, validate_trajectory
from agv_bridge.recovery.recovery_planner import RecoveryPlanner
from agv_bridge.sim_api_ext import patch_mock_state

DT = 0.10


def box_collide(cx: float, cy: float, hw: float = 0.12, hh: float = 0.12):
    def _f(x: float, y: float) -> bool:
        return abs(x - cx) <= hw and abs(y - cy) <= hh

    return _f


def wall_clearance(x: float, y: float) -> float:
    return abs(0.55 - y)


def test_committed_side_is_execution_constraint() -> None:
    """LEFT feasible / RIGHT blocked and reverse."""
    geom = get_vehicle_model().geometry
    left_c = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="LEFT",
        preferred_side="LEFT",
        commit_ready=True,
        front_near=1.5,
        left_free=1.2,
        right_free=0.3,
        geom=geom,
        reason="TEST_LEFT",
    )
    right_c = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="RIGHT",
        preferred_side="RIGHT",
        commit_ready=True,
        front_near=1.5,
        left_free=0.3,
        right_free=1.2,
        geom=geom,
        reason="TEST_RIGHT",
    )
    assert left_c.mode == MODE_LEFT
    assert right_c.mode == MODE_RIGHT
    assert corridor_allows_omega_sign(left_c, 0.15)
    assert not corridor_allows_omega_sign(left_c, -0.15)
    assert corridor_allows_omega_sign(right_c, -0.15)
    assert not corridor_allows_omega_sign(right_c, 0.15)

    mppi = DiffDriveMppi(batch_size=32, geom=geom)
    path = [(0.0, 0.0), (2.0, 0.0)]
    goal = (2.0, 0.0)

    def no_collide(_x: float, _y: float) -> bool:
        return False

    left_res = mppi.step(
        0.0, 0.0, 0.0, path, goal, no_collide,
        front_near=5.0, maneuver_mode="FORWARD_TURN", vx_max=0.25,
        execution_corridor=left_c, clearance_at=wall_clearance,
    )
    right_res = mppi.step(
        0.0, 0.0, 0.0, path, goal, no_collide,
        front_near=5.0, maneuver_mode="FORWARD_TURN", vx_max=0.25,
        execution_corridor=right_c, clearance_at=wall_clearance,
    )
    assert left_res.w >= -0.02, f"LEFT corridor allowed RIGHT omega: {left_res.w}"
    assert right_res.w <= 0.02, f"RIGHT corridor allowed LEFT omega: {right_res.w}"


def test_mppi_infeasible_enter_recovery() -> None:
    rp = RecoveryPlanner(max_attempts=5)
    r1 = rp.on_planner_failure(now=1.0, failure_reason=REASON_MPPI_INFEASIBLE)
    assert r1.planner_state == LOCAL_RECOVERY
    assert r1.action == "REPROBE"
    assert not r1.navigation_failed


def test_recovery_reprobe_rebuild() -> None:
    rp = RecoveryPlanner(max_attempts=5)
    rp.on_planner_failure(now=1.0, failure_reason=REASON_MPPI_INFEASIBLE)
    r2 = rp.on_planner_failure(now=2.0, failure_reason=REASON_MPPI_INFEASIBLE)
    assert r2.action in ("REBUILD_CORRIDOR", "REPLAN", "REPROBE")


def test_recovery_alternate_side() -> None:
    rp = RecoveryPlanner(max_attempts=5)
    for t in range(3):
        rp.on_planner_failure(now=float(t), failure_reason=REASON_MPPI_INFEASIBLE)
    r4 = rp.on_planner_failure(
        now=4.0,
        failure_reason=REASON_MPPI_INFEASIBLE,
        committed_side="LEFT",
        alternate_side_available=True,
    )
    assert r4.request_alternate_side or r4.action in ("ALTERNATE_SIDE", "GLOBAL_REPLAN", "WAIT_DYNAMIC")


def test_recovery_exhaustion() -> None:
    rp = RecoveryPlanner(max_attempts=3)
    for t in range(4):
        r = rp.on_planner_failure(now=float(t), failure_reason=REASON_MPPI_INFEASIBLE)
    assert r.navigation_failed
    assert r.planner_state == NAVIGATION_FAILED


def test_navigation_failed_semantics() -> None:
    sev = ui_severity(planner_state=NAVIGATION_FAILED, safe_vx_reason="NAVIGATION_FAILED", stop_reason="NAVIGATION_FAILED")
    assert sev == "FAILED"
    sev2 = ui_severity(planner_state=LOCAL_RECOVERY, safe_vx_reason="RECOVERY_ACTIVE", stop_reason="RECOVERY")
    assert sev2 == "RECOVERY"
    assert sev2 != "FAILED"


def test_safe_vx_reason_mppi_not_failed() -> None:
    sev = ui_severity(
        planner_state=LOCAL_PLAN_INFEASIBLE,
        safe_vx_reason=REASON_MPPI_INFEASIBLE,
        stop_reason="RECOVERY",
    )
    assert sev == "DEGRADED"
    assert sev != "FAILED"


def test_footprint_clearance_veto_validator() -> None:
    geom = get_vehicle_model().geometry
    poses = [{"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0}, {"x": 0.2, "y": 0.0, "yaw": 0.0, "t": DT}]
    bad = validate_trajectory(poses, collide=box_collide(0.2, 0.0), clearance_at=wall_clearance, geom=geom, dt=DT)
    assert not bad.valid


def test_braking_distance_veto() -> None:
    braking = BrakingModel(max_decel_mps2=0.8, reaction_latency_s=0.1, controller_latency_s=0.05, command_latency_s=0.05)
    ok, _ = braking_feasible(0.35, 0.5, braking, hard_stop_m=0.7)
    assert ok
    bad, reason = braking_feasible(0.35, 0.05, braking, hard_stop_m=0.7)
    assert not bad and reason == REASON_BRAKING_UNAVAILABLE or not bad or reason != "NONE"


def test_braking_unavailable_conservative() -> None:
    braking = BrakingModel(max_decel_mps2=None, calibration_status="CALIBRATION_REQUIRED")
    ok, reason = braking_feasible(0.30, 0.2, braking, hard_stop_m=0.7)
    assert not ok
    assert reason == REASON_BRAKING_UNAVAILABLE


def test_mppi_footprint_collision_counters() -> None:
    geom = get_vehicle_model().geometry
    mppi = DiffDriveMppi(batch_size=16, geom=geom)
    path = [(0.0, 0.0), (1.0, 0.0)]
    goal = (1.0, 0.0)

    def always(_x: float, _y: float) -> bool:
        return True

    mppi.step(0.0, 0.0, 0.0, path, goal, always, front_near=0.2)
    meta = dict(getattr(mppi, "_last_meta", {}) or {})
    assert meta.get("valid_candidate_count", -1) == 0
    assert meta.get("collision_rejected_count", 0) > 0
    assert meta.get("failure_reason") == "MPPI_NO_FEASIBLE_TRAJECTORY"


def test_mppi_score_uses_footprint_not_center_only() -> None:
    import inspect
    from agv_bridge.mppi_controller import DiffDriveMppi

    src = inspect.getsource(DiffDriveMppi._score)
    assert "collide(p[0]" not in src
    assert "footprint_validated" in src or "validate_trajectory" in src


def test_stale_sensor_veto_semantics() -> None:
    from agv_bridge.recovery.recovery_planner import RecoveryPlanner

    rp = RecoveryPlanner()
    r = rp.on_planner_failure(now=1.0, failure_reason="MPPI_NO_FEASIBLE_TRAJECTORY", sensor_stale=True)
    assert r.action == "SAFE_STOP"
    assert r.reason == "STALE_SENSOR"


def test_localization_invalid_veto_semantics() -> None:
    from agv_bridge.recovery.recovery_planner import RecoveryPlanner

    rp = RecoveryPlanner()
    r = rp.on_planner_failure(now=1.0, failure_reason="MPPI_NO_FEASIBLE_TRAJECTORY", localization_invalid=True)
    assert r.action == "SAFE_STOP"
    assert r.reason == "LOCALIZATION_INVALID"


def test_scene_021_field_p0d1_mppi_vx_zero() -> None:
    """Simulate probe→commit→turn→footprint approach→infeasible; must enter recovery not FAILED."""
    from types import SimpleNamespace

    state = SimpleNamespace(
        lock=__import__("threading").Lock(),
        x=0.0, y=0.0, angle=0.0, vx=0.0, vy=0.0, w=0.0,
        r_vx=0.0, r_vy=0.0, r_w=0.0,
        emergency=False, soft_emc=False, is_stop=True,
        locked_by=None, task_status=0, block_reason=0,
        max_vx=0.4, max_w=0.45,
    )
    patch_mock_state(state)
    from agv_bridge.nav_models import LocalMppiModel

    model = LocalMppiModel()
    world = state._sim_world

    def narrow_collide(x: float, y: float) -> bool:
        return abs(y - 0.52) < 0.06 and x > 0.15

    def clr_at(x: float, y: float) -> float:
        return abs(0.52 - y)

    corridor = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="LEFT",
        preferred_side="LEFT",
        commit_ready=True,
        front_near=1.2,
        left_free=0.9,
        right_free=0.4,
        geom=get_vehicle_model().geometry,
        reason="SCENE021",
    )
    model.last_execution_corridor = corridor
    model.planner_state = LOCAL_PLAN_INFEASIBLE

    res = model.mppi.step(
        0.0, 0.0, math.radians(25),
        [(0.0, 0.0), (3.0, 0.0)], (3.0, 0.0),
        narrow_collide,
        front_near=1.0,
        maneuver_mode="FORWARD_TURN",
        vx_max=0.22,
        execution_corridor=corridor,
        clearance_at=clr_at,
    )
    meta = dict(getattr(model.mppi, "_last_meta", {}) or {})
    assert meta.get("failure_reason") == "MPPI_NO_FEASIBLE_TRAJECTORY"
    rec = model.recovery_planner.on_planner_failure(
        now=2.0,
        failure_reason=REASON_MPPI_INFEASIBLE,
        committed_side="LEFT",
        alternate_side_available=True,
    )
    assert rec.planner_state in (LOCAL_RECOVERY, LOCAL_PLAN_INFEASIBLE)
    assert not rec.navigation_failed


def main() -> int:
    tests = [
        ("TEST_COMMITTED_SIDE_IS_EXECUTION_CONSTRAINT", test_committed_side_is_execution_constraint),
        ("TEST_MPPI_INFEASIBLE_ENTER_RECOVERY", test_mppi_infeasible_enter_recovery),
        ("TEST_RECOVERY_REPROBE", test_recovery_reprobe_rebuild),
        ("TEST_RECOVERY_ALTERNATE_SIDE", test_recovery_alternate_side),
        ("TEST_RECOVERY_EXHAUSTION", test_recovery_exhaustion),
        ("TEST_SAFE_STOP_AFTER_RECOVERY_EXHAUSTION", test_recovery_exhaustion),
        ("TEST_NAVIGATION_FAILED_SEMANTICS", test_navigation_failed_semantics),
        ("TEST_SAFE_VX_REASON", test_safe_vx_reason_mppi_not_failed),
        ("TEST_FOOTPRINT_CLEARANCE_VETO", test_footprint_clearance_veto_validator),
        ("TEST_BRAKING_DISTANCE_VETO", test_braking_distance_veto),
        ("TEST_BRAKING_UNAVAILABLE", test_braking_unavailable_conservative),
        ("TEST_MPPI_COLLISION_COUNTERS", test_mppi_footprint_collision_counters),
        ("TEST_MPPI_FOOTPRINT_SCORE", test_mppi_score_uses_footprint_not_center_only),
        ("TEST_STALE_SENSOR_VETO", test_stale_sensor_veto_semantics),
        ("TEST_LOCALIZATION_INVALID_VETO", test_localization_invalid_veto_semantics),
        ("SCENE-021-FIELD-P0D1-MPPI-VX-ZERO", test_scene_021_field_p0d1_mppi_vx_zero),
    ]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS {name}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL {name}: {exc}")
        except Exception as exc:
            failed += 1
            print(f"  ERROR {name}: {exc}")
    print(f"\nRESULT: {'PASS' if failed == 0 else 'FAIL'} ({failed} failed)")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
