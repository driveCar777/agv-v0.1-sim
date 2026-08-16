"""MockBackend — 纯 Python 运动学仿真。

完全离线。无任何外部依赖。带运动学积分 + 噪声。
用于 CI、单元测试、本地开发。
"""

from __future__ import annotations

import math
import random
import threading
import time

from agv_control.backend.base import (
    BatteryState,
    IAGVControlBackend,
    Odom2D,
    Twist2D,
)


class MockBackend(IAGVControlBackend):
    """差速底盘的运动学积分仿真。"""

    def __init__(
        self,
        max_vx: float = 0.5,
        max_w: float = 1.0,
        max_acc: float = 0.5,
        max_jerk: float = 1.0,
        dt: float = 0.02,                # 50 Hz 控制
        wheel_base: float = 0.45,
        start_pose: tuple = (0.0, 0.0, 0.0),
        add_noise: bool = False,
        sim_speed: float = 1.0,
        initial_battery: float = 0.95,
    ) -> None:
        self.max_vx = max_vx
        self.max_w = max_w
        self.max_acc = max_acc
        self.max_jerk = max_jerk
        self.dt = dt
        self.wheel_base = wheel_base
        self._add_noise = add_noise
        self._sim_speed = sim_speed

        self._x, self._y, self._yaw = start_pose
        self._vx = 0.0
        self._w = 0.0
        self._target_vx = 0.0
        self._target_w = 0.0
        self._is_stop = True
        self._battery = BatteryState(level=initial_battery, charging=False)
        self._emergency = False
        self._connected = False
        self._has_control = False

        self._lock = threading.RLock()
        self._last_tick = time.time()

    @property
    def mode(self) -> str:
        return "mock"

    # ---- lifecycle ----
    def connect(self) -> bool:
        with self._lock:
            self._connected = True
            self._last_tick = time.time()
        return True

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False
            self._has_control = False

    @property
    def connected(self) -> bool:
        return self._connected

    # ---- control authority ----
    def request_control(self, nick: str = "ros2_agv") -> bool:
        with self._lock:
            if not self._connected:
                return False
            self._has_control = True
        return True

    def release_control(self) -> bool:
        with self._lock:
            self._has_control = False
            self._vx = 0.0
            self._w = 0.0
            self._is_stop = True
        return True

    @property
    def has_control(self) -> bool:
        return self._has_control

    # ---- motion ----
    def set_velocity(self, twist: Twist2D) -> None:
        with self._lock:
            if self._emergency or not self._has_control:
                return
            target_vx = max(-self.max_vx, min(self.max_vx, twist.vx))
            target_w = max(-self.max_w, min(self.max_w, twist.w))
            self._target_vx = target_vx
            self._target_w = target_w
            dvx = max(
                -self.max_acc * self.dt,
                min(self.max_acc * self.dt, target_vx - self._vx),
            )
            dw = max(
                -self.max_acc * self.dt,
                min(self.max_acc * self.dt, target_w - self._w),
            )
            self._vx += dvx
            self._w += dw
            self._is_stop = abs(self._vx) < 1e-3 and abs(self._w) < 1e-3

    def step(self, dt: float | None = None) -> None:
        """推进仿真一步。

        默认使用固定 self.dt（可复现、适合单测/CI）。
        传入 dt 时可覆盖；若要用墙钟，调用方自行计算。
        """
        with self._lock:
            use_dt = float(self.dt if dt is None else dt)
            use_dt = max(1e-4, min(use_dt, 0.1)) * self._sim_speed
            # 每步继续朝目标速度爬升（与 set_velocity 解耦后仍可闭环）
            dvx = max(
                -self.max_acc * use_dt,
                min(self.max_acc * use_dt, self._target_vx - self._vx),
            )
            dw = max(
                -self.max_acc * use_dt,
                min(self.max_acc * use_dt, self._target_w - self._w),
            )
            self._vx += dvx
            self._w += dw
            self._is_stop = abs(self._vx) < 1e-3 and abs(self._w) < 1e-3
            self._x += self._vx * math.cos(self._yaw) * use_dt
            self._y += self._vx * math.sin(self._yaw) * use_dt
            self._yaw = (self._yaw + self._w * use_dt + math.pi) % (2 * math.pi) - math.pi
            if self._add_noise:
                self._x += random.gauss(0, 0.005)
                self._y += random.gauss(0, 0.005)
                self._yaw += random.gauss(0, 0.001)
            self._battery.level -= abs(self._vx) * use_dt * 1e-5
            self._last_tick = time.time()

    def stop(self) -> None:
        with self._lock:
            self._vx = 0.0
            self._w = 0.0
            self._target_vx = 0.0
            self._target_w = 0.0
            self._is_stop = True

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency = True
            self._vx = 0.0
            self._w = 0.0
            self._target_vx = 0.0
            self._target_w = 0.0
            self._is_stop = True

    def resume_from_emergency(self) -> None:
        with self._lock:
            self._emergency = False

    # ---- state feedback ----
    def get_odom(self) -> Odom2D:
        with self._lock:
            return Odom2D(
                x=self._x, y=self._y, yaw=self._yaw,
                vx=self._vx, vy=0.0, w=self._w,
                is_stop=self._is_stop, confidence=0.95,
            )

    def get_battery(self) -> BatteryState:
        with self._lock:
            return BatteryState(level=self._battery.level, charging=self._battery.charging)

    def get_emergency_state(self) -> bool:
        with self._lock:
            return self._emergency

    # ---- helpers for tests ----
    def teleport(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._x, self._y, self._yaw = x, y, yaw