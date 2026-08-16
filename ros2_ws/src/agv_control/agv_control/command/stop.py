"""Stop command — soft stop vs emergency stop."""

from __future__ import annotations

from enum import Enum


class StopKind(str, Enum):
    SOFT = "soft"
    EMERGENCY = "emergency"


def make_zero_twist() -> tuple:
    return (0.0, 0.0, 0.0)