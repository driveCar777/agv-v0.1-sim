"""Arm adapter factory."""

from __future__ import annotations

from typing import Any, Optional

from agv_bridge.arm.base import ArmAdapter
from agv_bridge.arm.mock import MockArmAdapter
from agv_bridge.arm.models import ArmMode
from agv_bridge.arm.real import RealArmAdapter


def create_arm_adapter(
    mode: str,
    *,
    node: Optional[Any] = None,
    robot_ip: str = "172.31.0.123",
    real_motion_enabled: bool = False,
) -> ArmAdapter:
    m = (mode or "simulation").strip().lower()
    if m in ("real", "hardware", "live"):
        if node is None:
            raise ValueError("ROS node required for real arm adapter")
        return RealArmAdapter(node, robot_ip=robot_ip, real_motion_enabled=real_motion_enabled)
    return MockArmAdapter(robot_ip=robot_ip)
