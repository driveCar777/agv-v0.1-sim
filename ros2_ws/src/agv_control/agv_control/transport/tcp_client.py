"""Robokit TCP API 客户端 — 仿真 Mock 与真车共用同一协议。

端口约定（与实车一致）：
  19204 status  — 1004 loc / 1005 speed / 1007 battery / 1011 emergency
  19205 control — 3055 translate / 3056 rotate / 2004? stop soft
  19207 config  — 4005 lock / 4006 unlock
"""

from __future__ import annotations

import json
import logging
import socket
import struct
from typing import Any, Dict, Optional


LOG = logging.getLogger(__name__)

HEADER_FMT = ">BBHIH6s"
HEADER_SIZE = 16

PORT_STATUS = 19204
PORT_CONTROL = 19205
PORT_CONFIG = 19207

API_LOC = 1004
API_SPEED = 1005
API_BATTERY = 1007
API_EMERGENCY = 1011
API_SOFT_STOP = 2000  # 部分机型；无则回落 3055 零速
API_TRANSLATE = 3055
API_ROTATE = 3056
API_LOCK = 4005
API_UNLOCK = 4006


class TcpApiClient:
    """短连接请求：每次 call 建连，避免跨线程复用 socket 踩坑。"""

    def __init__(self, host: str, timeout: float = 1.0) -> None:
        self.host = host
        self.timeout = timeout
        self._seq = 0

    def call(
        self,
        port: int,
        api_type: int,
        payload: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        body = b""
        if payload:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._seq = (self._seq + 1) % 65536
        header = struct.pack(
            HEADER_FMT, 0x5A, 1, self._seq, len(body), api_type, b"\x00" * 6
        )
        try:
            with socket.create_connection((self.host, port), timeout=self.timeout) as sock:
                sock.settimeout(self.timeout)
                sock.sendall(header + body)
                hdr = self._recv_exact(sock, HEADER_SIZE)
                _sync, _ver, _num, length, _rtype, _res = struct.unpack(HEADER_FMT, hdr)
                data = self._recv_exact(sock, length) if length else b""
                if not data:
                    return {"ret_code": 0}
                return json.loads(data.decode("utf-8"))
        except (OSError, struct.error, json.JSONDecodeError) as exc:
            LOG.warning("tcp fail %s:%d api=%d: %s", self.host, port, api_type, exc)
            return None

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise OSError("closed")
            buf.extend(chunk)
        return bytes(buf)

    # ---- high level ----
    def ping(self) -> bool:
        return self.call(PORT_STATUS, API_LOC, {}) is not None

    def lock(self, nick_name: str = "ros2_agv") -> bool:
        r = self.call(PORT_CONFIG, API_LOCK, {"nick_name": nick_name})
        return bool(r) and int(r.get("ret_code", 1)) == 0

    def unlock(self) -> bool:
        r = self.call(PORT_CONFIG, API_UNLOCK, {})
        return bool(r) and int(r.get("ret_code", 1)) == 0

    def translate(self, vx: float, vy: float = 0.0, w: float = 0.0) -> bool:
        r = self.call(PORT_CONTROL, API_TRANSLATE, {"vx": float(vx), "vy": float(vy), "w": float(w)})
        return bool(r) and int(r.get("ret_code", 1)) == 0

    def rotate(self, w: float) -> bool:
        r = self.call(PORT_CONTROL, API_ROTATE, {"w": float(w)})
        return bool(r) and int(r.get("ret_code", 1)) == 0

    def stop(self) -> bool:
        # 优先软停；失败则发零速 3055
        r = self.call(PORT_CONTROL, API_SOFT_STOP, {})
        if r is not None and int(r.get("ret_code", 1)) == 0:
            return True
        return self.translate(0.0, 0.0, 0.0)

    def get_pose(self) -> Optional[Dict[str, Any]]:
        return self.call(PORT_STATUS, API_LOC, {})

    def get_speed(self) -> Optional[Dict[str, Any]]:
        return self.call(PORT_STATUS, API_SPEED, {})

    def get_battery(self) -> Optional[Dict[str, Any]]:
        return self.call(PORT_STATUS, API_BATTERY, {})

    def get_emergency(self) -> Optional[Dict[str, Any]]:
        return self.call(PORT_STATUS, API_EMERGENCY, {})
