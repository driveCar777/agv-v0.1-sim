"""RealBackend / TcpBackend — cmd_vel → Robokit 3055/3056。

仿真 Mock 与真车使用同一套 TCP 协议；仅 host 不同：
  - 仿真：127.0.0.1（或 Docker 里 host.docker.internal）
  - 真车：192.168.192.5（或 AGV_HOST）

启用真车：
  AGV_BACKEND=real REAL_ROBOT_ENABLED=true AGV_HOST=<ip>
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from typing import Optional

from agv_control.backend.base import (
    BatteryState,
    IAGVControlBackend,
    Odom2D,
    Twist2D,
)
from agv_control.transport.tcp_client import TcpApiClient


LOG = logging.getLogger(__name__)


class RealBackend(IAGVControlBackend):
    """经实车同款 API 控车；对 Mock 与真机透明。"""

    def __init__(
        self,
        agv_host: str | None = None,
        max_vx: float = 0.5,
        max_w: float = 1.0,
        timeout: float = 1.0,
        nick: str = "ros2_agv",
        **kwargs,
    ) -> None:
        self._host = agv_host or os.environ.get("AGV_HOST", "192.168.192.5")
        self.max_vx = float(max_vx)
        self.max_w = float(max_w)
        self._nick = nick
        self._client = TcpApiClient(self._host, timeout=timeout)
        self._connected = False
        self._has_control = False
        self._emergency = False
        self._odom = Odom2D()
        self._battery = BatteryState(level=1.0, charging=False)
        self._lock = threading.RLock()
        self._last_cmd = Twist2D()
        self._last_cmd_ts = 0.0

    @property
    def mode(self) -> str:
        return "real"

    def connect(self) -> bool:
        ok = self._client.ping()
        with self._lock:
            self._connected = bool(ok)
        if ok:
            self._refresh_state()
            LOG.info("RealBackend connected to %s", self._host)
        else:
            LOG.error("RealBackend cannot reach %s:19204", self._host)
        return ok

    def disconnect(self) -> None:
        try:
            if self._has_control:
                self.release_control()
        finally:
            with self._lock:
                self._connected = False
                self._has_control = False

    @property
    def connected(self) -> bool:
        return self._connected

    def request_control(self, nick: str = "ros2_agv") -> bool:
        if not self._connected and not self.connect():
            return False
        ok = self._client.lock(nick or self._nick)
        with self._lock:
            self._has_control = ok
        return ok

    def release_control(self) -> bool:
        self.stop()
        ok = self._client.unlock()
        with self._lock:
            self._has_control = False if ok else self._has_control
        return ok

    @property
    def has_control(self) -> bool:
        return self._has_control

    def set_velocity(self, twist: Twist2D) -> None:
        if not self._has_control or self._emergency:
            return
        cmd = twist.clamp_abs(self.max_vx, self.max_w)
        with self._lock:
            self._last_cmd = cmd
            self._last_cmd_ts = time.time()
        # 实车 / Mock：统一走 3055
        self._client.translate(cmd.vx, cmd.vy, cmd.w)

    def stop(self) -> None:
        self._client.stop()
        with self._lock:
            self._last_cmd = Twist2D()

    def emergency_stop(self) -> None:
        with self._lock:
            self._emergency = True
        self.stop()

    def resume_from_emergency(self) -> None:
        with self._lock:
            self._emergency = False

    def get_odom(self) -> Odom2D:
        self._refresh_state()
        with self._lock:
            return Odom2D(
                x=self._odom.x,
                y=self._odom.y,
                yaw=self._odom.yaw,
                vx=self._odom.vx,
                vy=self._odom.vy,
                w=self._odom.w,
                is_stop=self._odom.is_stop,
                confidence=self._odom.confidence,
            )

    def get_battery(self) -> BatteryState:
        bat = self._client.get_battery()
        if bat:
            level = float(bat.get("battery_level", self._battery.level))
            if level > 1.0:
                level = level / 100.0
            self._battery = BatteryState(
                level=level,
                charging=bool(bat.get("charging", False)),
            )
        return self._battery

    def get_emergency_state(self) -> bool:
        em = self._client.get_emergency()
        if em:
            self._emergency = bool(em.get("emergency") or em.get("soft_emc"))
        return self._emergency

    def step(self) -> None:
        # 真机/API 后端：姿态由车端反馈，本地不积分
        return

    def _refresh_state(self) -> None:
        loc = self._client.get_pose()
        spd = self._client.get_speed()
        with self._lock:
            if loc:
                # Robokit angle 通常为度；Mock 当前用弧度。兼容两种。
                ang = float(loc.get("angle", self._odom.yaw))
                if abs(ang) > math.pi * 2.5:
                    ang = math.radians(ang)
                self._odom.x = float(loc.get("x", self._odom.x))
                self._odom.y = float(loc.get("y", self._odom.y))
                self._odom.yaw = ang
                self._odom.confidence = float(loc.get("confidence", 0.95))
            if spd:
                self._odom.vx = float(spd.get("vx", 0.0))
                self._odom.vy = float(spd.get("vy", 0.0))
                self._odom.w = float(spd.get("w", 0.0))
                self._odom.is_stop = bool(spd.get("is_stop", abs(self._odom.vx) < 1e-3))


# 别名：语义上强调「走 TCP API」
TcpBackend = RealBackend
