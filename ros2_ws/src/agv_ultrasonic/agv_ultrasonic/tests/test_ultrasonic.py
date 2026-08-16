"""agv_ultrasonic 单测 — 完全离线。"""

import math

from agv_ultrasonic.layout import SENSOR_LAYOUT, closest_distance


def test_sensor_layout_count():
    assert len(SENSOR_LAYOUT) == 12


def test_sensor_layout_covers_front_and_rear():
    front = [s for s in SENSOR_LAYOUT if s[0] > 0]
    rear = [s for s in SENSOR_LAYOUT if s[0] < 0]
    assert len(front) == 6
    assert len(rear) == 6


def test_closest_distance_no_obstacle():
    d = closest_distance(0.0, 0.0, 0.0, [], 4.0)
    assert d == 4.0


def test_closest_distance_point_obstacle():
    # 正前方 1m 处点状障碍
    d = closest_distance(0.0, 0.0, 0.0, [(1.0, 0.0, 0.0)], 4.0)
    assert abs(d - 1.0) < 1e-6


def test_closest_distance_behind_sensor():
    # 障碍在背后（投影为负），应忽略
    d = closest_distance(0.0, 0.0, 0.0, [(-1.0, 0.0, 0.0)], 4.0)
    assert d == 4.0


def test_closest_distance_circular_obstacle():
    # 圆心 1m 半径 0.3，最近点 = 0.7
    d = closest_distance(0.0, 0.0, 0.0, [(1.0, 0.0, 0.3)], 4.0)
    assert abs(d - 0.7) < 1e-6


def test_closest_distance_multiple_picks_closest():
    scene = [(2.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.5, 0.0, 0.0)]
    d = closest_distance(0.0, 0.0, 0.0, scene, 4.0)
    assert abs(d - 0.5) < 1e-6


def test_closest_distance_perpendicular_obstacle():
    # 障碍在垂直方向，不在射线内
    d = closest_distance(0.0, 0.0, 0.0, [(0.0, 1.0, 0.0)], 4.0)
    assert d == 4.0