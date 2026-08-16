"""Pose helpers — quaternion / yaw 转换。"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def yaw_to_quat(yaw: float) -> tuple:
    """Z 轴 yaw → 四元数 (x,y,z,w)。"""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw(z: float, w: float) -> float:
    """从四元数提取 yaw。"""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def angle_normalize(a: float) -> float:
    """wrap 到 [-π, π]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi