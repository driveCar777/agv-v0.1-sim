"""agv_tf 静态 TF 四元数单测 — 完全离线。"""

import math

from agv_tf.geometry import quat_from_rpy


def test_quat_identity():
    x, y, z, w = quat_from_rpy(0.0, 0.0, 0.0)
    assert abs(w - 1.0) < 1e-6
    assert abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6


def test_quat_yaw_180():
    x, y, z, w = quat_from_rpy(0.0, 0.0, math.pi)
    assert abs(w) < 1e-6
    assert abs(z - 1.0) < 1e-6 or abs(z + 1.0) < 1e-6


def test_quat_yaw_90():
    x, y, z, w = quat_from_rpy(0.0, 0.0, math.pi / 2)
    assert abs(w - math.cos(math.pi / 4)) < 1e-6
    assert abs(z - math.sin(math.pi / 4)) < 1e-6
