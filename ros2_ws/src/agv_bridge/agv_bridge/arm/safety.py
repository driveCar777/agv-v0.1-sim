"""Arm motion safety gates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from agv_bridge.arm.models import ArmCommand, ArmMode


@dataclass
class ArmSafetyLimits:
    joint_min_deg: Tuple[float, ...] = (-180.0,) * 7
    joint_max_deg: Tuple[float, ...] = (180.0,) * 7
    tcp_x_mm: Tuple[float, float] = (-800.0, 800.0)
    tcp_y_mm: Tuple[float, float] = (-800.0, 800.0)
    tcp_z_mm: Tuple[float, float] = (-200.0, 1000.0)
    max_joint_delta_deg: float = 15.0
    max_tcp_delta_mm: float = 50.0


def check_motion_allowed(mode: ArmMode, real_motion_enabled: bool) -> Tuple[bool, str]:
    if mode == ArmMode.SIMULATION:
        return True, "simulation"
    if mode == ArmMode.REAL and real_motion_enabled:
        return True, "real authorized"
    return False, "REAL motion blocked — awaiting explicit authorization"


def clamp_joint_delta(cmd: ArmCommand, limits: ArmSafetyLimits) -> float:
    d = float(cmd.delta_deg)
    if abs(d) > limits.max_joint_delta_deg:
        return limits.max_joint_delta_deg if d > 0 else -limits.max_joint_delta_deg
    return d
