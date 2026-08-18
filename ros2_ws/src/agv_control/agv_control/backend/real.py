"""RealBackend — cmd_vel → Robokit API (M3.9.3 corrected contract).

=== M3.9.3 API CONTRACT CORRECTION ===
BREAKING CHANGES:
  - set_velocity() now routes through API 2010 (continuous velocity), NOT 3055.
  - 3055 / 3056 are motion TASKS (translate_distance / rotate_angle), not velocity.
  - stop() now correctly handles STOP_FAILED without 3055 fallback.
  - AGV_HOST default IP (192.168.192.5) is now FORBIDDEN in real mode.
  - Emergency API corrected from 1011 → 1012.

NAVIGATION_CONTROL_MODE = VELOCITY_2010
  Navigation vx/omega → set_velocity(Twist2D) → 2010
  Status: API 2010 UNVERIFIED on real vehicle.

Physical test gate:
  PHYSICAL_TEST_ALLOWED = NO (2010 unverified, emergency API unverified on real hw)
  RC1_READY = NO
  REAL_BACKEND_READY = NO (pending 2010 real-vehicle verification)

Enabling real mode:
  AGV_BACKEND=real REAL_ROBOT_ENABLED=true AGV_HOST=<ip>
  All three are required. No default IP in real mode.
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

# Safety gate: 2010 is the correct velocity API but unverified on real vehicle.
# Once confirmed, update this flag and re-run full regression.
_API_2010_REAL_VEHICLE_VERIFIED = False  # UNVERIFIED as of M3.9.3


class RealBackend(IAGVControlBackend):
    """Real vehicle backend using correct Robokit API contract.

    Motion path: Navigation vx/omega → set_velocity(Twist2D) → API 2010
    Stop path: stop() → API 2000 → STOP_FAILED if 2000 fails (no 3055 fallback)
    Emergency: emergency_stop() → stop() (emergency API 1012 is read-only status query)

    Control authority:
        connect() → lock (API 4005) → set_velocity / stop → unlock (API 4006) → disconnect()
    """

    def __init__(
        self,
        agv_host: str | None = None,
        max_vx: float = 0.5,
        max_w: float = 1.0,
        timeout: float = 1.0,
        nick: str = "ros2_agv",
        require_host: bool = True,
        **kwargs,
    ) -> None:
        resolved_host = agv_host or os.environ.get("AGV_HOST", "")
        if require_host and not resolved_host:
            raise ValueError(
                "RealBackend: AGV_HOST is required in real mode. "
                "Set AGV_HOST environment variable or pass agv_host= explicitly. "
                "No default IP is allowed (M3.9.3 safety rule)."
            )
        # Only allow fallback to 127.0.0.1 in tcp/api (non-real) mode explicitly.
        self._host = resolved_host or "127.0.0.1"
        self.max_vx = float(max_vx)
        self.max_w = float(max_w)
        self._nick = nick
        self._client = TcpApiClient(self._host, timeout=timeout)
        self._connected = False
        self._has_control = False
        self._emergency = False
        self._stop_failed = False
        self._odom = Odom2D()
        self._battery = BatteryState(level=1.0, charging=False)
        self._lock = threading.RLock()
        self._last_cmd = Twist2D()
        self._last_cmd_ts = 0.0
        self._cmd_sequence = 0

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
        """Send continuous velocity command via API 2010.

        M3.9.3: Uses API 2010 (VELOCITY_2010 mode), NOT 3055.
        3055 is a distance-based motion task and must NOT be used for velocity streaming.

        API 2010 status: UNVERIFIED on real vehicle.
        """
        if not self._has_control or self._emergency:
            return
        if self._stop_failed:
            LOG.warning(
                "set_velocity() blocked — previous stop() returned STOP_FAILED. "
                "Motion is disabled until stop state is cleared."
            )
            return
        cmd = twist.clamp_abs(self.max_vx, self.max_w)
        with self._lock:
            self._last_cmd = cmd
            self._last_cmd_ts = time.time()
            self._cmd_sequence += 1
            seq = self._cmd_sequence

        ok, tel = self._client.set_velocity_cmd(
            vx=cmd.vx,
            vy=cmd.vy,
            w=cmd.w,
            duration=0,
            source="nav",
            command_age_ms=round((time.time() - self._last_cmd_ts) * 1000, 1),
            sequence=seq,
        )
        if not ok:
            LOG.warning(
                "set_velocity_cmd (API 2010) failed: latency=%.1fms error=%s",
                tel.get("latency_ms", 0),
                tel.get("error"),
            )

    def stop(self) -> None:
        """Stop vehicle via API 2000. If 2000 fails, sets STOP_FAILED — no 3055 fallback."""
        ok, status = self._client.stop()
        with self._lock:
            self._last_cmd = Twist2D()
            if not ok:
                self._stop_failed = True
                LOG.error(
                    "stop() STOP_FAILED on %s — motion authority suspended. "
                    "Do not send further motion commands.",
                    self._host,
                )
            else:
                self._stop_failed = False

    def clear_stop_failed(self) -> None:
        """Manually clear STOP_FAILED state after operator verification.

        Only call this if the vehicle has been independently confirmed stopped
        (e.g., physical observation or E-stop button press).
        """
        with self._lock:
            self._stop_failed = False
        LOG.info("STOP_FAILED state cleared by operator.")

    @property
    def stop_failed(self) -> bool:
        """True if the last stop() attempt failed — motion is suspended."""
        return self._stop_failed

    def emergency_stop(self) -> None:
        """Emergency stop: lock out further commands and stop vehicle."""
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
        """Query emergency state via API 1012 (corrected from wrong 1011)."""
        em = self._client.get_emergency()
        if em:
            self._emergency = bool(em.get("emergency") or em.get("soft_emc"))
        return self._emergency

    def step(self) -> None:
        # Real backend: pose from vehicle feedback, no local integration.
        return

    def _refresh_state(self) -> None:
        loc = self._client.get_pose()
        spd = self._client.get_speed()
        with self._lock:
            if loc:
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


# Alias: semantic emphasis on TCP API path
TcpBackend = RealBackend
