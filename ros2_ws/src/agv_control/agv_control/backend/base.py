"""IAGVControlBackend — 统一底盘后端接口。

默认 Backend = Mock。完全离线，不连真车。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional


@dataclass
class Twist2D:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0

    def clamp_abs(self, max_vx: float, max_w: float) -> "Twist2D":
        return Twist2D(
            vx=max(-max_vx, min(max_vx, self.vx)),
            vy=self.vy,
            w=max(-max_w, min(max_w, self.w)),
        )


@dataclass
class Odom2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    is_stop: bool = True
    confidence: float = 0.95


@dataclass
class BatteryState:
    level: float = 1.0     # 0~1
    charging: bool = False


class IAGVControlBackend(ABC):
    """所有底盘后端必须实现这套接口。

    控制权状态机：
        DISCONNECTED → CONNECTED → REQUEST_CONTROL → CONTROL_GRANTED
                      → READY → MOVING → STOPPING → RELEASE
    """

    @property
    @abstractmethod
    def mode(self) -> str:
        """mock | sim | real"""

    # ---- lifecycle ----
    @abstractmethod
    def connect(self) -> bool: ...
    @abstractmethod
    def disconnect(self) -> None: ...
    @property
    @abstractmethod
    def connected(self) -> bool: ...

    # ---- control authority ----
    @abstractmethod
    def request_control(self, nick: str = "ros2_agv") -> bool: ...
    @abstractmethod
    def release_control(self) -> bool: ...
    @property
    @abstractmethod
    def has_control(self) -> bool: ...

    # ---- motion ----
    @abstractmethod
    def set_velocity(self, twist: Twist2D) -> None: ...
    @abstractmethod
    def stop(self) -> None: ...
    @abstractmethod
    def emergency_stop(self) -> None: ...
    @abstractmethod
    def resume_from_emergency(self) -> None: ...

    # ---- state feedback ----
    @abstractmethod
    def get_odom(self) -> Odom2D: ...
    @abstractmethod
    def get_battery(self) -> BatteryState: ...
    @abstractmethod
    def get_emergency_state(self) -> bool: ...

    # ---- simulation driver (no-op for real) ----
    def step(self) -> None:
        """供仿真后端推进仿真。真实后端无操作。"""