"""agv_lidar 单测 — 完全离线。"""

import math

from agv_lidar.scan_sim import make_scan


def test_make_scan_empty_scene():
    msg = make_scan(
        stamp=None,
        frame_id="lidar_front",
        obstacle_world_xy=[],
        lidar_world_xy=(0.0, 0.0),
        lidar_yaw=0.0,
    )
    assert msg.header.frame_id == "lidar_front"
    assert len(msg.ranges) > 0
    # 空场景：所有距离都是 range_max
    assert all(r >= msg.range_max - 1e-3 for r in msg.ranges)


def test_make_scan_detects_obstacle():
    # 在正前方 1m 处放一个点状障碍
    msg = make_scan(
        stamp=None,
        frame_id="lidar_front",
        obstacle_world_xy=[(1.0, 0.0, 0.0)],
        lidar_world_xy=(0.0, 0.0),
        lidar_yaw=0.0,
    )
    # 第 (pi/angle_increment) 个元素附近应该是 1.0
    mid = len(msg.ranges) // 2
    assert abs(msg.ranges[mid] - 1.0) < 0.05


def test_make_scan_rotated():
    # 障碍在 (0, 1)，LiDAR 旋转 90°
    msg = make_scan(
        stamp=None,
        frame_id="lidar_front",
        obstacle_world_xy=[(0.0, 1.0, 0.0)],
        lidar_world_xy=(0.0, 0.0),
        lidar_yaw=math.pi / 2,
    )
    # 旋转后，正前方就是 +Y，1m
    mid = len(msg.ranges) // 2
    assert abs(msg.ranges[mid] - 1.0) < 0.05


def test_make_scan_circular_obstacle():
    msg = make_scan(
        stamp=None,
        frame_id="lidar_front",
        obstacle_world_xy=[(1.0, 0.0, 0.5)],
        lidar_world_xy=(0.0, 0.0),
        lidar_yaw=0.0,
    )
    mid = len(msg.ranges) // 2
    # 圆心在 1m，半径 0.5，最近点 = 0.5
    assert abs(msg.ranges[mid] - 0.5) < 0.05