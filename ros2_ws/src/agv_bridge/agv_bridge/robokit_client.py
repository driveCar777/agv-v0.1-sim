"""Robokit SEER TCP/IP binary protocol client (16-byte header + JSON body)."""

from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time
from typing import Any, Dict, List, Optional, Tuple


LOG = logging.getLogger(__name__)


# Port map
PORT_STATUS = 19204
PORT_CONTROL = 19205
PORT_NAV = 19206
PORT_CONFIG = 19207
PORT_OTHER = 19210
PORT_PUSH = 19301

# API type IDs (request)
API_LOC = 1004
API_SPEED = 1005
API_BLOCK = 1006
API_BATTERY = 1007
API_LASER = 1009
API_EMERGENCY = 1012
API_TASK_STATUS = 1020
API_RELOC_STATUS = 1021
API_MAP_LOAD_STATUS = 1022  # query map load: 0=fail 1=ok 2=loading
API_CONTROL_OWNER = 1060  # robot_status_current_lock_req
API_BATCH_STATUS = 1100  # robot_status_all1_req
API_PATH_QUERY = 1303  # query path between two stations
API_RELOC = 2002
API_CONFIRM_LOC = 2003
API_CANCEL_RELOC = 2004
API_CANCEL_NAV = 3003
API_PAUSE_NAV = 3001
API_RESUME_NAV = 3002
API_GOTO_TARGET = 3051
API_LOCK = 4005
API_UNLOCK = 4006
API_MAP = 1300
API_STATION = 1301
API_SWITCH_MAP = 2022
API_UPLOAD_SWITCH = 2025
API_UPLOAD_MAP = 4010
API_DOWNLOAD_MAP = 4011
API_PUSH_CONFIG = 9300

# Reloc status values (1021)
RELOC_INIT = 0
RELOC_SUCCESS = 1
RELOC_RELOCING = 2
RELOC_COMPLETED_NEED_CONFIRM = 3  # old fw (< 3.4.6.1800): completed but not yet confirmed (2003)

# Acceptable confirm-loc "not needed / benign" error codes when new firmware
# drops the need for manual 2003; unknown api type (60001) / ret_code != 0
# with err_msg mentioning version/unknown/already should not be fatal.
_CONFIRM_LOC_BENIGN_RET_CODES = (60001, 60003)
_CONFIRM_LOC_BENIGN_SUBSTRINGS = ("unknown api", "version", "already", "无需", "not supported")

# Test injection: robokit_mock_server API_MOCK_CTRL = 50000 (NOT a real API).
# Body fields: set_confidence:float, old_fw_need_confirm:bool, set_reloc_status:int,
# set_pose:{x,y,angle,current_station?}, inject_reloc_stages:[int,...]
# Response type (req+10000 = 60000) < 65535 so uint16 header fits.
# Requests to real vehicle on 50000 will get "unknown api type 60001" (RobokitError).
API_MOCK_CTRL = 50000

HEADER_FMT = ">BBHIH6s"  # sync, ver, number, length, type, reserved (big-endian per Robokit protocol spec)
HEADER_SIZE = 16


class RobokitError(RuntimeError):
    def __init__(self, message: str, ret_code: int = -1, raw: Optional[dict] = None):
        super().__init__(message)
        self.ret_code = ret_code
        self.raw = raw or {}


class RobokitClient:
    """Thread-safe multi-port TCP client for Robokit API."""

    def __init__(
        self,
        host: str,
        timeout: float = 3.0,
        protocol_version: int = 1,
        connect_on_init: bool = False,
    ):
        self.host = host
        self.timeout = timeout
        self.protocol_version = protocol_version
        self._seq = 0
        self._lock = threading.Lock()
        self._port_locks: Dict[int, threading.Lock] = {}
        self._socks: Dict[int, socket.socket] = {}
        if connect_on_init:
            self.connect_all()

    def connect_all(self) -> None:
        for port in (PORT_STATUS, PORT_CONTROL, PORT_NAV, PORT_CONFIG):
            self._ensure_sock(port)

    def close(self) -> None:
        with self._lock:
            for s in self._socks.values():
                try:
                    s.close()
                except OSError:
                    pass
            self._socks.clear()

    def _next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) % 65536
            return self._seq

    def _lock_for_port(self, port: int) -> threading.Lock:
        with self._lock:
            lk = self._port_locks.get(port)
            if lk is None:
                lk = threading.Lock()
                self._port_locks[port] = lk
            return lk

    def _ensure_sock(self, port: int) -> socket.socket:
        with self._lock:
            s = self._socks.get(port)
            if s is not None:
                return s
            s = socket.create_connection((self.host, port), timeout=self.timeout)
            s.settimeout(self.timeout)
            self._socks[port] = s
            return s

    def _drop_sock(self, port: int) -> None:
        with self._lock:
            s = self._socks.pop(port, None)
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass

    def request(
        self,
        port: int,
        api_type: int,
        payload: Optional[Dict[str, Any]] = None,
        retries: int = 1,
    ) -> Dict[str, Any]:
        body = b""
        if payload is not None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")

        last_err: Optional[Exception] = None
        for _ in range(retries + 1):
            try:
                return self._request_once(port, api_type, body)
            except (OSError, RobokitError, struct.error, json.JSONDecodeError) as exc:
                last_err = exc
                self._drop_sock(port)
                time.sleep(0.05)
        raise RobokitError(f"request failed: {last_err}") from last_err

    def _request_once(self, port: int, api_type: int, body: bytes) -> Dict[str, Any]:
        # One in-flight request per TCP port — concurrent 1004/1009/1021 on 19204
        # used to interleave frames and break 3051 navigation.
        with self._lock_for_port(port):
            seq = self._next_seq()
            header = struct.pack(
                HEADER_FMT,
                0x5A,
                self.protocol_version,
                seq,
                len(body),
                api_type,
                b"\x00" * 6,
            )
            sock = self._ensure_sock(port)
            sock.sendall(header + body)

            hdr = self._recv_exact(sock, HEADER_SIZE)
            sync, ver, number, length, rtype, _reserved = struct.unpack(HEADER_FMT, hdr)
            if sync != 0x5A:
                raise RobokitError(f"bad sync byte: {sync:#x}")
            data = self._recv_exact(sock, length) if length else b""
            if data:
                obj = json.loads(data.decode("utf-8"))
            else:
                obj = {}
            if not isinstance(obj, dict):
                raise RobokitError("response JSON is not an object", raw={"data": obj})
            ret = obj.get("ret_code", 0)
            if ret not in (0, None):
                raise RobokitError(
                    obj.get("err_msg", f"ret_code={ret}"),
                    ret_code=int(ret),
                    raw=obj,
                )
            obj["_req_type"] = api_type
            obj["_res_type"] = rtype
            obj["_seq"] = number
            obj["_proto_ver"] = ver
            return obj

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise OSError("socket closed while receiving")
            buf.extend(chunk)
        return bytes(buf)

    # ---- convenience ----

    def get_pose(self) -> Dict[str, Any]:
        return self.request(PORT_STATUS, API_LOC)

    def get_speed(self) -> Dict[str, Any]:
        """Robokit 1005 — robot_status_speed_req (port 19204).

        Documented fields: vx/vy/w (actual), r_vx/r_vy/r_w (nav commanded),
        is_stop, plus steer/spin variants. ``r_vx`` is the P0 split:
        0 → upper layer not commanding motion; >0 with vx=0 → chassis not executing.
        """
        return self.request(PORT_STATUS, API_SPEED)

    def get_block_status(self) -> Dict[str, Any]:
        """Robokit 1006 — robot_status_block_req (port 19204)."""
        return self.request(PORT_STATUS, API_BLOCK)

    def get_batch_status(self) -> Dict[str, Any]:
        """Robokit 1100 — robot_status_all1_req (port 19204, READ-ONLY)."""
        return self.request(PORT_STATUS, API_BATCH_STATUS)

    def get_map_load_status(self) -> Dict[str, Any]:
        """Robokit 1022 — map load status (port 19204, READ-ONLY).

        ``loadmap_status``: 0=fail, 1=success, 2=loading.
        """
        return self.request(PORT_STATUS, API_MAP_LOAD_STATUS)

    def get_control_owner(self) -> Dict[str, Any]:
        """Robokit 1060 — current lock owner (port 19204, READ-ONLY).

        Fields: locked, ip, port, type, nick_name, time_t, desc.
        Nested ``current_lock`` exists on 1100, not on this API.
        """
        return self.request(PORT_STATUS, API_CONTROL_OWNER)

    def get_path_info(self, source_id: str, target_id: str) -> Dict[str, Any]:
        """Robokit 1303 — path between two stations (port 19204, READ-ONLY)."""
        return self.request(
            PORT_STATUS,
            API_PATH_QUERY,
            {"source_id": source_id, "target_id": target_id, "id": target_id},
        )

    def get_battery(self) -> Dict[str, Any]:
        """Robokit 1007 — robot_status_battery_req (port 19204)."""
        return self.request(PORT_STATUS, API_BATTERY)

    def get_emergency(self) -> Dict[str, Any]:
        """Robokit 1012 — robot_status_emergency_req (port 19204)."""
        return self.request(PORT_STATUS, API_EMERGENCY)

    def get_laser(self, return_beams3d: bool = False) -> Dict[str, Any]:
        """Robokit 1009 — robot_status_laser_req (port 19204)."""
        payload = {"return_beams3D": bool(return_beams3d)}
        return self.request(PORT_STATUS, API_LASER, payload)

    def get_task_status(self, simple: bool = False) -> Dict[str, Any]:
        payload = {"simple": True} if simple else None
        return self.request(PORT_STATUS, API_TASK_STATUS, payload)

    def lock(self, nick_name: str) -> Dict[str, Any]:
        return self.request(PORT_CONFIG, API_LOCK, {"nick_name": nick_name})

    def unlock(self) -> Dict[str, Any]:
        return self.request(PORT_CONFIG, API_UNLOCK)

    def cancel_nav(self) -> Dict[str, Any]:
        return self.request(PORT_NAV, API_CANCEL_NAV)

    def pause_nav(self) -> Dict[str, Any]:
        """Robokit 3001 — pause current navigation (port 19206)."""
        return self.request(PORT_NAV, API_PAUSE_NAV)

    def resume_nav(self) -> Dict[str, Any]:
        """Robokit 3002 — resume paused navigation (port 19206)."""
        return self.request(PORT_NAV, API_RESUME_NAV)

    def goto_station(
        self,
        target_id: str,
        source_id: Optional[str] = None,
        task_id: Optional[str] = None,
        angle: Optional[float] = None,
        method: Optional[str] = None,
        max_speed: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Robokit 3051 — path navigation to target station (port 19206).

        Per official spec (路径导航.docx): `source_id` is documented as
        required, taking either a station id or `"SELF_POSITION"`. **However,
        real-vehicle testing on 192.168.18.198 (2026-07-28 by AGV colleague)
        proved that sending `source_id` (any value, including
        `SELF_POSITION`) makes the vehicle return an error — only
        `{"id":"<target>"}` works.** Therefore the default here is
        `source_id=None` (field omitted from payload). Pass a non-empty
        string only if a future firmware revision restores the documented
        behavior.
        """
        payload: Dict[str, Any] = {"id": target_id}
        if source_id:  # only include when explicitly set non-empty
            payload["source_id"] = source_id
        if task_id:
            payload["task_id"] = task_id
        if angle is not None:
            payload["angle"] = float(angle)
        if method:
            payload["method"] = method
        if max_speed is not None and max_speed > 0:
            payload["max_speed"] = float(max_speed)
        return self.request(PORT_NAV, API_GOTO_TARGET, payload)

    def wait_nav_done(
        self,
        timeout_sec: float = 120.0,
        poll_sec: float = 0.3,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Poll 1020 until COMPLETED/FAILED/CANCELED/NONE after leave RUNNING."""
        deadline = time.time() + timeout_sec
        saw_running = False
        last: Dict[str, Any] = {}
        while time.time() < deadline:
            last = self.get_task_status(simple=False)
            status = int(last.get("task_status", 0))
            if status == 2:  # RUNNING
                saw_running = True
            if status == 4:  # COMPLETED
                return True, last
            if status in (5, 6):  # FAILED / CANCELED
                return False, last
            if saw_running and status == 0:
                return True, last
            time.sleep(poll_sec)
        return False, last

    # ---- relocation (2002/2003/2004/1021) ----

    def get_reloc_status(self) -> Dict[str, Any]:
        """Robokit 1021 — query current relocation status (port 19204, no body).

        Returns dict with key `reloc_status` (0=INIT / 1=SUCCESS / 2=RELOCING /
        3=COMPLETED + needs 2003 confirm on old firmware).
        """
        LOG.info(
            "[RELOC] 1021 get_reloc_status() — requesting relocation status"
            " (host=%s, port=%d)",
            self.host,
            PORT_STATUS,
        )
        res = self.request(PORT_STATUS, API_RELOC_STATUS)
        LOG.info(
            "[RELOC] 1021 done — reloc_status=%s raw=%s",
            res.get("reloc_status"),
            res,
        )
        return res

    def relocate(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        angle: Optional[float] = None,
        length: Optional[float] = None,
        is_auto: Optional[bool] = None,
        home: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """Robokit 2002 — trigger relocation (port 19205 / control).

        Per spec: at least one meaningful field must be provided. If `is_auto`
        is True all other fields are ignored by vehicle. If `home` is True the
        vehicle uses Roboshop-configured RobotHome1-5 (x/y/angle ignored).
        """
        payload: Dict[str, Any] = {}
        if is_auto is not None:
            payload["isAuto"] = bool(is_auto)
        if home is not None:
            payload["home"] = bool(home)
        if x is not None:
            payload["x"] = float(x)
        if y is not None:
            payload["y"] = float(y)
        if angle is not None:
            payload["angle"] = float(angle)
        if length is not None:
            payload["length"] = float(length)

        if not payload:
            raise ValueError(
                "relocate(): payload empty — at least x+y (or is_auto/home) must be set"
            )

        LOG.info(
            "[RELOC] 2002 relocate START — host=%s port=%d payload=%s",
            self.host,
            PORT_CONTROL,
            payload,
        )
        t0 = time.time()
        try:
            res = self.request(PORT_CONTROL, API_RELOC, payload)
            LOG.info(
                "[RELOC] 2002 relocate ACK — elapsed=%.2fs ret_code=%s raw=%s",
                time.time() - t0,
                res.get("ret_code"),
                res,
            )
            return res
        except RobokitError as e:
            LOG.warning(
                "[RELOC] 2002 relocate FAIL — elapsed=%.2fs ret_code=%s msg=%s raw=%s",
                time.time() - t0,
                getattr(e, "ret_code", -1),
                e,
                getattr(e, "raw", {}),
            )
            raise
        except Exception as e:
            LOG.warning(
                "[RELOC] 2002 relocate EXCEPTION — elapsed=%.2fs type=%s msg=%s",
                time.time() - t0,
                type(e).__name__,
                e,
            )
            raise

    def confirm_loc(self, allow_benign_error: bool = True) -> Dict[str, Any]:
        """Robokit 2003 — confirm relocation correct (port 19205 / control, no body).

        Robokit 3.4.6.18+ no longer requires this call. If
        `allow_benign_error` is True and the vehicle rejects with one of
        `_CONFIRM_LOC_BENIGN_RET_CODES` (e.g. unknown api type 60001) or
        err_msg contains "not supported"/"无需" the response is still returned
        without re-raising; callers can check `_was_benign=True` in result.
        """
        LOG.info(
            "[RELOC] 2003 confirm_loc START — host=%s port=%d allow_benign=%s",
            self.host,
            PORT_CONTROL,
            allow_benign_error,
        )
        t0 = time.time()
        try:
            res = self.request(PORT_CONTROL, API_CONFIRM_LOC)
            res["_was_benign"] = False
            LOG.info(
                "[RELOC] 2003 confirm_loc OK — elapsed=%.2fs ret_code=%s raw=%s",
                time.time() - t0,
                res.get("ret_code"),
                res,
            )
            return res
        except RobokitError as e:
            raw = getattr(e, "raw", {}) or {}
            ret = getattr(e, "ret_code", -1)
            msg = (str(e) + " " + str(raw.get("err_msg", ""))).lower()
            benign = allow_benign_error and (
                ret in _CONFIRM_LOC_BENIGN_RET_CODES
                or any(sub.lower() in msg for sub in _CONFIRM_LOC_BENIGN_SUBSTRINGS)
            )
            if benign:
                LOG.info(
                    "[RELOC] 2003 confirm_loc BENIGN (new fw, no confirm needed) —"
                    " elapsed=%.2fs ret_code=%s msg=%s",
                    time.time() - t0,
                    ret,
                    e,
                )
                raw["ret_code"] = 0
                raw["_was_benign"] = True
                raw["_benign_ret_code"] = ret
                raw["err_msg"] = "confirm_loc not needed (benign): " + str(e)
                return raw
            LOG.warning(
                "[RELOC] 2003 confirm_loc FAIL — elapsed=%.2fs ret_code=%s msg=%s raw=%s",
                time.time() - t0,
                ret,
                e,
                raw,
            )
            raise
        except Exception as e:
            LOG.warning(
                "[RELOC] 2003 confirm_loc EXCEPTION — elapsed=%.2fs type=%s msg=%s",
                time.time() - t0,
                type(e).__name__,
                e,
            )
            raise

    def cancel_reloc(self) -> Dict[str, Any]:
        """Robokit 2004 — cancel running relocation (port 19205 / control, no body).

        No-op if reloc_status != 2 (RELOCING) per spec.
        """
        LOG.info(
            "[RELOC] 2004 cancel_reloc START — host=%s port=%d",
            self.host,
            PORT_CONTROL,
        )
        t0 = time.time()
        try:
            res = self.request(PORT_CONTROL, API_CANCEL_RELOC)
            LOG.info(
                "[RELOC] 2004 cancel_reloc ACK — elapsed=%.2fs ret_code=%s raw=%s",
                time.time() - t0,
                res.get("ret_code"),
                res,
            )
            return res
        except RobokitError as e:
            LOG.warning(
                "[RELOC] 2004 cancel_reloc FAIL — elapsed=%.2fs ret_code=%s msg=%s raw=%s",
                time.time() - t0,
                getattr(e, "ret_code", -1),
                e,
                getattr(e, "raw", {}),
            )
            raise

    def wait_reloc_done(
        self,
        timeout_sec: float = 30.0,
        poll_sec: float = 0.4,
        auto_confirm_if_needed: bool = True,
    ) -> Tuple[int, Dict[str, Any]]:
        """Poll 1021 until relocation finishes (status != INIT/RELOCING).

        Returns (final_reloc_status, last_1021_response).
        Final status in { RELOC_SUCCESS(1), RELOC_COMPLETED_NEED_CONFIRM(3) } on
        success path. If `auto_confirm_if_needed` and final == 3 then 2003 is
        called (benign errors tolerated) and the final returned status is
        forced to RELOC_SUCCESS when confirm succeeds (or was benign).
        """
        LOG.info(
            "[RELOC] wait_reloc_done START — timeout=%.1fs poll=%.2fs auto_confirm=%s",
            timeout_sec,
            poll_sec,
            auto_confirm_if_needed,
        )
        deadline = time.time() + timeout_sec
        last: Dict[str, Any] = {}
        last_status = RELOC_INIT
        while time.time() < deadline:
            last = self.get_reloc_status()
            last_status = int(last.get("reloc_status", RELOC_INIT))
            if last_status in (RELOC_SUCCESS, RELOC_COMPLETED_NEED_CONFIRM):
                break
            if last_status == RELOC_RELOCING:
                LOG.info("[RELOC] wait_reloc_done — still RELOCING (status=2)")
            else:
                LOG.info(
                    "[RELOC] wait_reloc_done — status=%s (still waiting)", last_status
                )
            time.sleep(poll_sec)
        else:
            LOG.warning(
                "[RELOC] wait_reloc_done TIMEOUT — elapsed=%.1fs last_status=%s",
                timeout_sec,
                last_status,
            )
            return last_status, last

        if last_status == RELOC_COMPLETED_NEED_CONFIRM and auto_confirm_if_needed:
            LOG.info(
                "[RELOC] wait_reloc_done — status=3 (old fw, needs confirm),"
                " auto-invoking 2003 confirm_loc"
            )
            self.confirm_loc(allow_benign_error=True)
            last_status = RELOC_SUCCESS
            LOG.info(
                "[RELOC] wait_reloc_done — 2003 confirm done, promoting status to 1 (SUCCESS)"
            )

        LOG.info(
            "[RELOC] wait_reloc_done DONE — final_reloc_status=%s raw=%s",
            last_status,
            last,
        )
        return last_status, last

    # ---- map / station (1300 / 1301 / 2022 / 4010 / 4011 / 2025) ----

    def get_map_info(self) -> Dict[str, Any]:
        """Robokit 1300 — current_map + maps list (port 19204)."""
        return self.request(PORT_STATUS, API_MAP)

    def get_station_list(self) -> Dict[str, Any]:
        """Robokit 1301 — stations in current map (port 19204)."""
        return self.request(PORT_STATUS, API_STATION)

    def switch_map(self, map_name: str) -> Dict[str, Any]:
        """Robokit 2022 — switch current map by ASCII name (port 19205)."""
        return self.request(PORT_CONTROL, API_SWITCH_MAP, {"map_name": map_name})

    def download_map(self, map_name: str) -> Dict[str, Any]:
        """Robokit 4011 — download smap JSON for map_name (port 19207)."""
        return self.request(PORT_CONFIG, API_DOWNLOAD_MAP, {"map_name": map_name})

    def upload_map(self, smap_json: Dict[str, Any]) -> Dict[str, Any]:
        """Robokit 4010 — upload smap JSON to storage (port 19207)."""
        body = json.dumps(smap_json, separators=(",", ":")).encode("utf-8")
        return self._request_with_body(PORT_CONFIG, API_UPLOAD_MAP, body)

    def upload_and_switch_map(self, smap_json: Dict[str, Any]) -> Dict[str, Any]:
        """Robokit 2025 — upload + switch current map (port 19205)."""
        body = json.dumps(smap_json, separators=(",", ":")).encode("utf-8")
        return self._request_with_body(PORT_CONTROL, API_UPLOAD_SWITCH, body)

    def configure_push(
        self,
        interval_ms: int = 200,
        included_fields: Optional[List[str]] = None,
        excluded_fields: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Robokit 9300 — configure 19301 push channel."""
        payload: Dict[str, Any] = {"interval": int(interval_ms)}
        if included_fields:
            payload["included_fields"] = included_fields
        if excluded_fields:
            payload["excluded_fields"] = excluded_fields
        return self.request(PORT_PUSH, API_PUSH_CONFIG, payload)

    def _request_with_body(
        self, port: int, api_type: int, body: bytes, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        seq = self._next_seq()
        header = struct.pack(
            HEADER_FMT,
            0x5A,
            self.protocol_version,
            seq,
            len(body),
            api_type,
            b"\x00" * 6,
        )
        sock = self._ensure_sock(port)
        old_to = sock.gettimeout()
        try:
            sock.settimeout(float(timeout) if timeout else max(self.timeout, 90.0 if len(body) > 50_000 else self.timeout))
            sock.sendall(header + body)
            hdr = self._recv_exact(sock, HEADER_SIZE)
            sync, ver, number, length, rtype, _reserved = struct.unpack(HEADER_FMT, hdr)
            if sync != 0x5A:
                raise RobokitError(f"bad sync byte: {sync:#x}")
            data = self._recv_exact(sock, length) if length else b""
        finally:
            try:
                sock.settimeout(old_to)
            except OSError:
                pass
        if not data:
            return {"ret_code": 0}
        obj = json.loads(data.decode("utf-8"))
        if not isinstance(obj, dict):
            raise RobokitError("response JSON is not an object", raw={"data": obj})
        ret = obj.get("ret_code", 0)
        if ret not in (0, None):
            raise RobokitError(obj.get("err_msg", f"ret_code={ret}"), ret_code=int(ret), raw=obj)
        return obj


def parse_host_port(host_port: str, default_port: int = PORT_STATUS) -> Tuple[str, int]:
    if ":" in host_port:
        h, p = host_port.rsplit(":", 1)
        return h, int(p)
    return host_port, default_port
