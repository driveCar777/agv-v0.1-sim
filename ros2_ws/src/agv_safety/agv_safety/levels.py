"""安全等级枚举 — 无 ROS 依赖。"""

from __future__ import annotations

import enum


class SafetyLevel(int, enum.Enum):
    OK = 0
    WARN = 1
    SLOW = 2
    STOP = 3
    EMERGENCY = 4
