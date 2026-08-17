"""V0.2 M1 focused regression — geometry, corridor, validator, MPPI constraint."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.mppi_controller import DiffDriveMppi
from agv_bridge.nav_execution_corridor import MODE_LEFT, MODE_RIGHT, build_execution_corridor
from agv_bridge.nav_footprint import get_corners, get_inflated_footprint, trajectory_min_clearance
from agv_bridge.nav_geometry import get_vehicle_model
from agv_bridge.nav_trajectory_validator import REASON_COLLISION, REASON_CORRIDOR, validate_trajectory

DT = 0.10
VX = 0.20


def _timed_straight(n: int = 4) -> list[dict]:
    poses = []
    x = 0.0
    for i in range(n):
        poses.append({"x": x, "y": 0.0, "yaw": 0.0, "t": i * DT})
        x += VX * DT
    return poses


def box_collide(cx: float, cy: float, half_w: float = 0.08, half_h: float = 0.08):
    def _f(x: float, y: float) -> bool:
        return abs(x - cx) <= half_w and abs(y - cy) <= half_h

    return _f


def wall_clearance(x: float, y: float) -> float:
    return abs(0.55 - y)


def test_footprint() -> None:
    vehicle = get_vehicle_model()
    geom = vehicle.geometry
    corners = get_corners({"x": 0.0, "y": 0.0, "yaw": 0.0}, geom)
    assert "front_left" in corners and "rear_right" in corners
    inflated = get_inflated_footprint({"x": 0.0, "y": 0.0, "yaw": 0.0}, 0.10, geom)
    assert len(inflated) == 4


def test_corridor() -> None:
    geom = get_vehicle_model().geometry
    left = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="LEFT",
        preferred_side="LEFT",
        commit_ready=True,
        front_near=1.8,
        left_free=1.2,
        right_free=0.4,
        probe_confidence_left=0.8,
        probe_confidence_right=0.2,
        geom=geom,
        reason="TEST",
    )
    right = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="RIGHT",
        preferred_side="RIGHT",
        commit_ready=True,
        front_near=1.8,
        left_free=0.4,
        right_free=1.2,
        probe_confidence_left=0.2,
        probe_confidence_right=0.8,
        geom=geom,
        reason="TEST",
    )
    assert left.mode == MODE_LEFT and left.constrains_side()
    assert right.mode == MODE_RIGHT and right.constrains_side()


def test_validator() -> None:
    geom = get_vehicle_model().geometry
    straight = _timed_straight()
    valid = validate_trajectory(straight, collide=box_collide(2.0, 2.0), clearance_at=wall_clearance, geom=geom, dt=DT)
    assert valid.valid, f"straight invalid: {valid.reason}"

    colliding = _timed_straight(6)
    # place obstacle at final front bumper area
    last = colliding[-1]
    invalid = validate_trajectory(
        colliding,
        collide=box_collide(float(last["x"]) + 0.05, 0.0, 0.15, 0.15),
        clearance_at=wall_clearance,
        geom=geom,
        dt=DT,
    )
    assert not invalid.valid and invalid.reason == REASON_COLLISION

    turn = [
        {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0},
        {"x": 0.02, "y": 0.0, "yaw": math.radians(10), "t": DT},
        {"x": 0.04, "y": 0.004, "yaw": math.radians(22), "t": 2 * DT},
        {"x": 0.06, "y": 0.010, "yaw": math.radians(30), "t": 3 * DT},
    ]
    clearance = trajectory_min_clearance(turn, wall_clearance, geom)
    assert clearance.minimum_clearance_m is not None


def test_mppi_left_corridor() -> None:
    """SCENE-021 subset: committed LEFT must reject RIGHT-sign omega samples."""
    geom = get_vehicle_model().geometry
    mppi = DiffDriveMppi(batch_size=24, geom=geom)
    corridor = build_execution_corridor(
        now=1.0,
        avoidance_phase="SIDE_COMMIT",
        committed_side="LEFT",
        preferred_side="LEFT",
        commit_ready=True,
        front_near=1.5,
        left_free=1.0,
        right_free=0.5,
        geom=geom,
        reason="SCENE021",
    )
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    goal = (2.0, 0.0)

    def no_collide(_x: float, _y: float) -> bool:
        return False

    res = mppi.step(
        0.0,
        0.0,
        0.0,
        path,
        goal,
        no_collide,
        front_near=5.0,
        maneuver_mode="FORWARD_TURN",
        vx_min=0.04,
        vx_max=0.25,
        execution_corridor=corridor,
        clearance_at=wall_clearance,
    )
    meta = dict(getattr(mppi, "_last_meta", {}) or {})
    assert meta.get("constraint_rejected_count", 0) >= 0
    assert meta.get("execution_corridor", {}).get("mode") == "LEFT"
    # With LEFT corridor, selected omega should not be strongly RIGHT
    assert res.w >= -0.02, f"MPPI selected negative omega under LEFT corridor: {res.w}"


def test_mppi_infeasible_meta() -> None:
    geom = get_vehicle_model().geometry
    mppi = DiffDriveMppi(batch_size=12, geom=geom)
    path = [(0.0, 0.0), (0.5, 0.0)]
    goal = (0.5, 0.0)

    def always_collide(_x: float, _y: float) -> bool:
        return True

    res = mppi.step(0.0, 0.0, 0.0, path, goal, always_collide, front_near=0.2)
    meta = dict(getattr(mppi, "_last_meta", {}) or {})
    assert res.vx == 0.0 and res.w == 0.0
    assert meta.get("failure_reason") == "MPPI_NO_FEASIBLE_TRAJECTORY"
    assert meta.get("valid_candidate_count", -1) == 0


def main() -> int:
    failed = 0
    tests = [
        ("TEST_FOOTPRINT", test_footprint),
        ("TEST_LEFT_CORRIDOR", test_corridor),
        ("TEST_VALIDATOR", test_validator),
        ("SCENE-021_MPPI_CORRIDOR", test_mppi_left_corridor),
        ("TEST_MPPI_INFEASIBLE", test_mppi_infeasible_meta),
    ]
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
