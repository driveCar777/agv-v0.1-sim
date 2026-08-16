"""Velocity command helpers — limits + smoothing."""

from __future__ import annotations

from dataclasses import dataclass

from agv_control.backend.base import Twist2D


@dataclass
class VelocityLimits:
    max_vx: float = 0.5
    max_w: float = 1.0
    max_acc: float = 0.5
    max_jerk: float = 1.0


def clamp(twist: Twist2D, limits: VelocityLimits) -> Twist2D:
    return twist.clamp_abs(limits.max_vx, limits.max_w)


def rate_limit(prev: Twist2D, target: Twist2D, limits: VelocityLimits, dt: float) -> Twist2D:
    """线性 + 角速度 限速。"""
    max_dvx = limits.max_acc * dt
    max_dw = limits.max_acc * dt
    dvx = max(-max_dvx, min(max_dvx, target.vx - prev.vx))
    dw = max(-max_dw, min(max_dw, target.w - prev.w))
    return Twist2D(vx=prev.vx + dvx, vy=prev.vy, w=prev.w + dw)