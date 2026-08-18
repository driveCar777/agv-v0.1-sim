"""Robokit TCP API 客户端 — 仿真 Mock 与真车共用同一协议。

端口约定（与实车一致，来源：robokit_client.py 已验证接口）：
  19204 status  — 1004 loc / 1005 speed / 1007 battery / 1012 emergency
  19205 control — 2010 velocity (continuous) / 2000 soft-stop (UNVERIFIED)
  19207 config  — 4005 lock / 4006 unlock

=== M3.9.3 API CONTRACT CORRECTION ===
BREAKING CHANGES from pre-M3.9.3:
  - Emergency API corrected: 1011 → 1012 (confirmed in robokit_client.py)
  - translate(vx,vy,w) → set_velocity_cmd(vx,vy,w,duration) via 2010
    3055 is a distance-based motion TASK, NOT a velocity streaming API.
    Use translate_distance(dist,vx,mode) for 3055.
  - rotate(w) → rotate_angle(angle,vw,mode) via 3056
    3056 requires angle+vw+mode fields.
  - stop() no longer falls back to translate(0,0,0)/3055.
    If 2000 fails → returns STOP_FAILED, caller must handle.

API 2010 status: UNVERIFIED on real vehicle. Contract from project spec.
API 2000 status: UNVERIFIED. Used as primary stop — real behavior unknown.
"""

from __future__ import annotations

import json
import logging
import socket
import struct
import time
from typing import Any, Dict, Optional, Tuple


LOG = logging.getLogger(__name__)

HEADER_FMT = ">BBHIH6s"
HEADER_SIZE = 16

PORT_STATUS = 19204
PORT_CONTROL = 19205
PORT_CONFIG = 19207

# ---- STATUS APIs (port 19204, read-only) ----
API_LOC = 1004
API_SPEED = 1005
API_BATTERY = 1007
# CORRECTED: emergency is 1012, NOT 1011
# 1011 was a P0 contract bug in pre-M3.9.3 code.
API_EMERGENCY = 1012          # confirmed via robokit_client.py
_WRONG_EMERGENCY_1011 = 1011  # kept as documentation, must NOT be used

# ---- CONTROL APIs (port 19205, write) ----
# 2000: SOFT_STOP — UNVERIFIED on real vehicle
API_SOFT_STOP = 2000          # status: UNVERIFIED
# 2010: CONTINUOUS VELOCITY COMMAND — UNVERIFIED on real vehicle
API_VELOCITY = 2010           # status: UNVERIFIED; project spec: {vx, vy, w, duration}
# 3055: DISTANCE TRANSLATE TASK — NOT a velocity streaming API
# payload: {dist: float, vx: float, mode: int} — NOT {vx, vy, w}
API_TRANSLATE_TASK = 3055
# 3056: ANGLE ROTATE TASK — NOT an angular velocity streaming API
# payload: {angle: float, vw: float, mode: int} — NOT {w: float}
API_ROTATE_TASK = 3056

# ---- CONFIG APIs (port 19207) ----
API_LOCK = 4005
API_UNLOCK = 4006


class TcpApiClient:
    """短连接请求：每次 call 建连，避免跨线程复用 socket 踩坑。

    M3.9.3: 所有 motion API call 记录遥测。
    """

    def __init__(self, host: str, timeout: float = 1.0) -> None:
        self.host = host
        self.timeout = timeout
        self._seq = 0

    def call(
        self,
        port: int,
        api_type: int,
        payload: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
        """Execute one API call. Returns (response, telemetry).

        response: None on failure.
        telemetry: always populated with timing and error info.
        """
        body = b""
        if payload:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._seq = (self._seq + 1) % 65536
        header = struct.pack(
            HEADER_FMT, 0x5A, 1, self._seq, len(body), api_type, b"\x00" * 6
        )
        t_request = time.time()
        tel: Dict[str, Any] = {
            "api_id": api_type,
            "tcp_port": port,
            "request": payload,
            "request_ts": t_request,
            "response_ts": None,
            "latency_ms": None,
            "result": "pending",
            "error": None,
        }
        try:
            with socket.create_connection((self.host, port), timeout=self.timeout) as sock:
                sock.settimeout(self.timeout)
                sock.sendall(header + body)
                hdr = self._recv_exact(sock, HEADER_SIZE)
                _sync, _ver, _num, length, _rtype, _res = struct.unpack(HEADER_FMT, hdr)
                data = self._recv_exact(sock, length) if length else b""
                t_response = time.time()
                tel["response_ts"] = t_response
                tel["latency_ms"] = round((t_response - t_request) * 1000, 2)
                if not data:
                    result = {"ret_code": 0}
                else:
                    result = json.loads(data.decode("utf-8"))
                tel["result"] = "ok" if int(result.get("ret_code", 0)) == 0 else "error"
                return result, tel
        except (OSError, struct.error, json.JSONDecodeError) as exc:
            t_response = time.time()
            tel["response_ts"] = t_response
            tel["latency_ms"] = round((t_response - t_request) * 1000, 2)
            tel["result"] = "exception"
            tel["error"] = str(exc)
            LOG.warning("tcp fail %s:%d api=%d: %s", self.host, port, api_type, exc)
            return None, tel

    def _call(
        self,
        port: int,
        api_type: int,
        payload: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Legacy call interface (no telemetry). Internal use only."""
        result, _ = self.call(port, api_type, payload)
        return result

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise OSError("closed")
            buf.extend(chunk)
        return bytes(buf)

    # ---- high level: status / read ----

    def ping(self) -> bool:
        return self._call(PORT_STATUS, API_LOC, {}) is not None

    def lock(self, nick_name: str = "ros2_agv") -> bool:
        r = self._call(PORT_CONFIG, API_LOCK, {"nick_name": nick_name})
        return bool(r) and int(r.get("ret_code", 1)) == 0

    def unlock(self) -> bool:
        r = self._call(PORT_CONFIG, API_UNLOCK, {})
        return bool(r) and int(r.get("ret_code", 1)) == 0

    def get_pose(self) -> Optional[Dict[str, Any]]:
        return self._call(PORT_STATUS, API_LOC, {})

    def get_speed(self) -> Optional[Dict[str, Any]]:
        return self._call(PORT_STATUS, API_SPEED, {})

    def get_battery(self) -> Optional[Dict[str, Any]]:
        return self._call(PORT_STATUS, API_BATTERY, {})

    def get_emergency(self) -> Optional[Dict[str, Any]]:
        """Query emergency state via API 1012 (corrected from wrong 1011)."""
        return self._call(PORT_STATUS, API_EMERGENCY, {})

    # ---- high level: motion (M3.9.3 corrected) ----

    def set_velocity_cmd(
        self,
        vx: float,
        vy: float = 0.0,
        w: float = 0.0,
        duration: int = 0,
        source: str = "nav",
        command_age_ms: Optional[float] = None,
        sequence: Optional[int] = None,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Continuous velocity command via API 2010.

        Semantic: continuous velocity streaming (vx, vy, w).
        duration=0: execute until next command (per project spec — UNVERIFIED on real vehicle).

        API 2010 status: UNVERIFIED on real vehicle.
        DO NOT use 3055 (translate_distance) for velocity streaming.

        Returns (success: bool, telemetry: dict).
        """
        payload: Dict[str, Any] = {
            "vx": float(vx),
            "vy": float(vy),
            "w": float(w),
            "duration": int(duration),
        }
        result, tel = self.call(PORT_CONTROL, API_VELOCITY, payload)
        tel["command_source"] = source
        if command_age_ms is not None:
            tel["command_age_ms"] = command_age_ms
        if sequence is not None:
            tel["sequence"] = sequence
        ok = result is not None and int(result.get("ret_code", 1)) == 0
        return ok, tel

    def translate_distance(
        self,
        dist: float,
        vx: float,
        mode: int = 0,
        source: str = "nav",
    ) -> Tuple[bool, Dict[str, Any]]:
        """Distance translate task via API 3055.

        Semantic: move forward/backward by `dist` meters at `vx` m/s.
        NOT a velocity streaming API.
        payload: {dist, vx, mode}

        DO NOT pass omega/w to this function — 3055 is not a velocity command.
        """
        payload: Dict[str, Any] = {
            "dist": float(dist),
            "vx": float(vx),
            "mode": int(mode),
        }
        result, tel = self.call(PORT_CONTROL, API_TRANSLATE_TASK, payload)
        tel["task_type"] = "TRANSLATE"
        tel["command_source"] = source
        ok = result is not None and int(result.get("ret_code", 1)) == 0
        return ok, tel

    def rotate_angle(
        self,
        angle: float,
        vw: float,
        mode: int = 0,
        source: str = "nav",
    ) -> Tuple[bool, Dict[str, Any]]:
        """Angle rotate task via API 3056.

        Semantic: rotate by `angle` radians at `vw` rad/s.
        NOT an angular velocity streaming API.
        payload: {angle, vw, mode}

        DO NOT pass only `w` — 3056 requires angle + vw + mode.
        """
        payload: Dict[str, Any] = {
            "angle": float(angle),
            "vw": float(vw),
            "mode": int(mode),
        }
        result, tel = self.call(PORT_CONTROL, API_ROTATE_TASK, payload)
        tel["task_type"] = "ROTATE"
        tel["command_source"] = source
        ok = result is not None and int(result.get("ret_code", 1)) == 0
        return ok, tel

    def stop(self) -> Tuple[bool, str]:
        """Soft stop via API 2000 (UNVERIFIED on real vehicle).

        Returns (success: bool, status: str).

        CRITICAL: If stop fails, returns STOP_FAILED.
        There is NO fallback to translate(0,0,0) or any 3055 zero-velocity trick.
        Caller must handle STOP_FAILED by entering safe state.
        """
        r = self._call(PORT_CONTROL, API_SOFT_STOP, {})
        if r is not None and int(r.get("ret_code", 1)) == 0:
            return True, "STOP_OK"
        # DO NOT fallback to translate(0,0,0)/3055.
        # The previous fallback was a P0 contract violation:
        #   translate(0,0,0) sends {vx:0, vy:0, w:0} to API 3055 (distance task).
        #   Sending zero-distance to a motion task API has undefined behavior.
        LOG.error(
            "stop() via API 2000 FAILED on %s — returning STOP_FAILED. "
            "Caller must enter safe state. NO fallback to 3055.",
            self.host,
        )
        return False, "STOP_FAILED"
