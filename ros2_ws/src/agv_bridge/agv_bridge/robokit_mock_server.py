"""Offline Robokit TCP mock (SEER binary header + JSON).

Implements the APIs our Demo path needs so联调 can be proven without a real AGV:
  19204: 1004 pose, 1007 battery, 1009 laser, 1020 task status
  19206: 3003 cancel, 3051 goto
  19207: 4005 lock, 4006 unlock

Usage:
  python3 -m agv_bridge.robokit_mock_server --host 127.0.0.1
  # then: POST /api/env {"mode":"demo","agv_host":"127.0.0.1"}
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

HEADER_FMT = ">BBHIH6s"  # big-endian per Robokit protocol spec (match real robot)
HEADER_SIZE = 16

PORT_STATUS = 19204
PORT_CONTROL = 19205
PORT_NAV = 19206
PORT_CONFIG = 19207
PORT_PUSH = 19301

# request -> response type = req + 10000
API_LOC = 1004
API_BATTERY = 1007
API_LASER = 1009
API_SPEED = 1005
API_BLOCK = 1006
API_EMERGENCY = 1012
API_TASK = 1020
API_RELOC_STATUS = 1021
API_MAP_LOAD = 1022
API_CONTROL_OWNER = 1060
API_BATCH = 1100
API_RELOC = 2002
API_CONFIRM_LOC = 2003
API_CANCEL_RELOC = 2004
API_CANCEL = 3003
API_GOTO = 3051
API_GOTO_LIST = 3066
API_TRANSLATE = 3055  # 平动速度控制（仿真与实车同款 API）
API_ROTATE = 3056     # 转动
API_LOCK = 4005
API_UNLOCK = 4006
API_MAP = 1300
API_STATION = 1301
API_PATH_QUERY = 1303
API_SWITCH_MAP = 2022
API_UPLOAD_SWITCH = 2025
API_UPLOAD_MAP = 4010
API_DOWNLOAD_MAP = 4011
API_ADD_OBS = 4350
API_ADD_GOBS = 4351
API_REM_OBS = 4352
# Internal test injection hook (not a real Robokit API). Body JSON fields:
#   set_confidence: float       → override pose confidence (next 1004 returns this)
#   old_fw_need_confirm: bool   → relocate goes to status=3 COMPLETED need confirm instead of 1 SUCCESS
#   set_reloc_status: 0|1|2|3   → directly write reloc status
#   set_pose: {x,y,angle,current_station?}  → overwrite pose state
#   inject_reloc_stages: [status,...] → preset ring of statuses returned by 1021 in order
# req_type=50000 → res_type=60000 (fits uint16 header field)
API_MOCK_CTRL = 50000


class RobokitMockState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.confidence = 0.95
        self.current_station = "LM1"
        self.last_station = ""
        self.vehicle_id = "MOCK-AGV"
        self.battery_level = 0.87
        self.charging = False
        self.task_status = 0  # NONE
        self.task_type = 0
        self.target_id = ""
        self.finished_path: List[str] = []
        self.unfinished_path: List[str] = []
        self.locked_by = ""
        self._nav_thread: Optional[threading.Thread] = None
        # station poses for simulated motion (id -> x,y,angle)
        self.stations: Dict[str, Tuple[float, float, float]] = {
            "LM1": (0.0, 0.0, 0.0),
            "LM2": (2.0, 0.0, 0.0),
            "LM3": (2.0, 2.0, 1.57),
            "LM4": (0.0, 2.0, 3.14),
        }
        # Relocation (2002/2003/2004/1021)
        self.reloc_status = 1  # starts SUCCESS (vehicle started + localization OK)
        self.reloc_old_fw_need_confirm = False
        self._reloc_thread: Optional[threading.Thread] = None
        self._reloc_stage_ring: Optional[List[int]] = None
        self._reloc_stage_idx = 0
        self.current_map = "mock_map_001"
        self.maps: Dict[str, Dict[str, Any]] = {
            "mock_map_001": self._default_smap("mock_map_001"),
        }
        self.obstacles: List[Dict[str, Any]] = []
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.r_vx = 0.0
        self.r_vy = 0.0
        self.r_w = 0.0
        self.is_stop = True
        self.dispatch_mode = 0
        self.connect_fleet = False
        self.block_reason = 0
        self.brake = False
        self.emergency = False
        self.soft_emc = False
        self.driver_emc = False
        self.manual_charge = False
        self.loadmap_status = 1
        self.fail_1100 = False
        self.fail_1005 = False
        self.batch_override: Dict[str, Any] = {}
        self.locked_ip = "127.0.0.1"
        self.locked_port = 52681
        self.locked_type = 0
        self.stations["LM6"] = (6.0, 0.0, 0.0)
        self.stations["LM7"] = (7.0, 0.0, 0.0)

    def _default_smap(self, name: str) -> Dict[str, Any]:
        return {
            "header": {"mapName": name, "mapType": "2D", "minPos": {"x": -1, "y": -1}, "maxPos": {"x": 5, "y": 5}},
            "normalPosList": [{"x": float(i), "y": float(j)} for i in range(0, 5) for j in range(0, 5)],
            "advancedPointList": [
                {
                    "className": "LocationMark",
                    "instanceName": sid,
                    "pos": {"x": x, "y": y},
                    "dir": a,
                }
                for sid, x, y, a in (
                    ("LM1", 0.0, 0.0, 0.0),
                    ("LM2", 2.0, 0.0, 0.0),
                    ("LM3", 2.0, 2.0, 1.57),
                    ("LM4", 0.0, 2.0, 3.14),
                )
            ],
            "advancedCurveList": [],
        }

    def map_info(self) -> Dict[str, Any]:
        return {
            "ret_code": 0,
            "current_map": self.current_map,
            "maps": list(self.maps.keys()),
        }

    def station_list(self) -> Dict[str, Any]:
        smap = self.maps.get(self.current_map) or self._default_smap(self.current_map)
        stations = []
        for pt in smap.get("advancedPointList") or []:
            pos = pt.get("pos") or {}
            stations.append(
                {
                    "id": str(pt.get("instanceName") or ""),
                    "type": str(pt.get("className") or "LocationMark"),
                    "x": float(pos.get("x", 0.0)),
                    "y": float(pos.get("y", 0.0)),
                    "r": float(pt.get("dir", 0.0)),
                    "desc": str(pt.get("instanceName") or ""),
                }
            )
        return {"ret_code": 0, "stations": stations}

    def switch_map(self, map_name: str) -> Dict[str, Any]:
        if map_name not in self.maps:
            self.maps[map_name] = self._default_smap(map_name)
        self.current_map = map_name
        return {"ret_code": 0}

    def download_map(self, map_name: str) -> Dict[str, Any]:
        smap = self.maps.get(map_name)
        if smap is None:
            return {"ret_code": 40004, "err_msg": f"map {map_name} not found"}
        return smap

    def upload_map(self, smap: Dict[str, Any]) -> Dict[str, Any]:
        name = str((smap.get("header") or {}).get("mapName") or f"map_{int(time.time())}")
        self.maps[name] = smap
        return {"ret_code": 0, "map_name": name}

    def upload_and_switch(self, smap: Dict[str, Any]) -> Dict[str, Any]:
        out = self.upload_map(smap)
        if out.get("ret_code", 0) == 0:
            self.current_map = str(out.get("map_name") or self.current_map)
        return out

    def start_nav_list(self, tasks: List[Dict[str, Any]]) -> Dict[str, Any]:
        if not tasks:
            return {"ret_code": 1, "err_msg": "move_task_list empty"}
        last = tasks[-1]
        tid = str(last.get("id") or last.get("target_id") or "")
        return self.start_nav(tid, str(last.get("source_id") or "SELF_POSITION"))

    def add_obstacle(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        name = str(payload.get("name") or f"obs_{len(self.obstacles)}")
        obs = {"name": name, **payload}
        with self.lock:
            self.obstacles = [o for o in self.obstacles if o.get("name") != name]
            self.obstacles.append(obs)
        return {"ret_code": 0}

    def add_global_obstacle(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.add_obstacle(payload)

    def remove_obstacle(self, name: str) -> Dict[str, Any]:
        with self.lock:
            self.obstacles = [o for o in self.obstacles if str(o.get("name")) != name]
        return {"ret_code": 0}

    def loc(self) -> Dict[str, Any]:
        return {
            "ret_code": 0,
            "err_msg": "",
            "x": self.x,
            "y": self.y,
            "angle": self.angle,
            "confidence": self.confidence,
            "current_station": self.current_station,
            "last_station": self.last_station,
            "vehicle_id": self.vehicle_id,
        }

    def battery(self) -> Dict[str, Any]:
        return {
            "ret_code": 0,
            "battery_level": self.battery_level,
            "charging": self.charging,
        }

    def task(self, simple: bool = False) -> Dict[str, Any]:
        if simple:
            return {"ret_code": 0, "task_status": self.task_status}
        return {
            "ret_code": 0,
            "task_status": self.task_status,
            "task_type": self.task_type,
            "target_id": self.target_id,
            "finished_path": list(self.finished_path),
            "unfinished_path": list(self.unfinished_path),
        }

    def laser(self) -> Dict[str, Any]:
        # Official examples use beam angle in **degrees** (-90..90)
        beams = []
        for i in range(181):
            deg = -90.0 + float(i)
            rad = math.radians(deg)
            # simple room walls ~3–5 m
            dist = 3.5 + 0.4 * math.sin(rad * 3.0 + time.time() * 0.2)
            beams.append({"angle": deg, "dist": dist, "valid": True})
        return {
            "ret_code": 0,
            "lasers": [
                {
                    "device_name": "laser_mock",
                    "min_angle": -90,
                    "max_angle": 90,
                    "max_range": 30,
                    "install_info": {"x": 0.3, "y": 0.0, "yaw": 0.0},
                    "beams": beams,
                }
            ],
        }

    def start_nav(self, target_id: str, source_id: str = "SELF_POSITION") -> Dict[str, Any]:
        with self.lock:
            if target_id not in self.stations:
                # still accept unknown ids: invent pose near current
                self.stations[target_id] = (self.x + 1.0, self.y + 0.5, self.angle)
            goal = self.stations[target_id]
            self.task_status = 2  # RUNNING
            self.task_type = 3
            self.target_id = target_id
            src = self.current_station or source_id
            self.finished_path = [src] if src and src != "SELF_POSITION" else []
            self.unfinished_path = [target_id]
            if self._nav_thread and self._nav_thread.is_alive():
                # cancel previous by marking; new thread will overwrite
                self.task_status = 6
                time.sleep(0.05)
            self.task_status = 2
            self._nav_thread = threading.Thread(
                target=self._run_nav, args=(target_id, goal), daemon=True
            )
            self._nav_thread.start()
        return {"ret_code": 0, "create_on_tasklist_id": 1}

    def _run_nav(self, target_id: str, goal: Tuple[float, float, float]) -> None:
        gx, gy, ga = goal
        steps = 20
        with self.lock:
            sx, sy, sa = self.x, self.y, self.angle
        for i in range(1, steps + 1):
            with self.lock:
                if self.task_status != 2 or self.target_id != target_id:
                    return
                t = i / float(steps)
                self.x = sx + (gx - sx) * t
                self.y = sy + (gy - sy) * t
                self.angle = sa + (ga - sa) * t
                self.current_station = ""
            time.sleep(0.05)
        with self.lock:
            if self.task_status != 2 or self.target_id != target_id:
                return
            self.x, self.y, self.angle = gx, gy, ga
            self.last_station = self.finished_path[-1] if self.finished_path else self.last_station
            self.current_station = target_id
            self.finished_path = list(self.finished_path) + [target_id]
            self.unfinished_path = []
            self.task_status = 4  # COMPLETED
            self.task_type = 3

    def cancel(self) -> Dict[str, Any]:
        with self.lock:
            if self.task_status == 2:
                self.task_status = 6
            return {"ret_code": 0}

    def do_lock(self, nick: str) -> Dict[str, Any]:
        with self.lock:
            self.locked_by = nick or "anonymous"
            return {"ret_code": 0}

    def do_unlock(self) -> Dict[str, Any]:
        with self.lock:
            self.locked_by = ""
            return {"ret_code": 0}

    # ---- relocation (2002 / 2003 / 2004 / 1021) ----

    def reloc_status_q(self) -> Dict[str, Any]:
        with self.lock:
            if self._reloc_stage_ring:
                # walk injected stages one per query; when exhausted pin to last value
                ring = self._reloc_stage_ring
                if self._reloc_stage_idx < len(ring):
                    st = ring[self._reloc_stage_idx]
                    self._reloc_stage_idx += 1
                    self.reloc_status = st
            return {"ret_code": 0, "reloc_status": self.reloc_status}

    def do_relocate(
        self,
        x: Optional[float],
        y: Optional[float],
        angle: Optional[float],
        is_auto: Optional[bool],
        home: Optional[bool],
    ) -> Dict[str, Any]:
        if x is None and y is None and not is_auto and not home:
            return {"ret_code": 1, "err_msg": "relocate payload must not be empty"}
        with self.lock:
            self.reloc_status = 2  # RELOCING
            if self._reloc_thread and self._reloc_thread.is_alive():
                # cancel previous reloc thread gracefully: no-op, new thread below will overwrite
                pass
            target_reloc_final = 3 if self.reloc_old_fw_need_confirm else 1
            # Use x/y/angle if provided; else fall back to current pose
            fx = float(x) if x is not None else self.x
            fy = float(y) if y is not None else self.y
            fa = float(angle) if angle is not None else self.angle
            self._reloc_thread = threading.Thread(
                target=self._run_reloc,
                args=(fx, fy, fa, target_reloc_final),
                daemon=True,
            )
            self._reloc_thread.start()
        return {"ret_code": 0}

    def _run_reloc(self, x: float, y: float, a: float, final_status: int) -> None:
        # Simulate: 200ms RELOCING, then jump to final status, boost confidence to 0.95
        time.sleep(0.2)
        with self.lock:
            if self.reloc_status != 2:
                return  # cancelled or already moved on
            self.x = x
            self.y = y
            self.angle = a
            self.confidence = 0.95
            self.reloc_status = final_status

    def do_confirm_loc(self) -> Dict[str, Any]:
        with self.lock:
            if self.reloc_status == 3:
                self.reloc_status = 1
            return {"ret_code": 0}

    def do_cancel_reloc(self) -> Dict[str, Any]:
        with self.lock:
            if self.reloc_status == 2:
                self.reloc_status = 3
            return {"ret_code": 0}

    # ---- internal mock control hook (API_MOCK_CTRL=99999) ----

    def mock_control(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        report: Dict[str, Any] = {"ret_code": 0}
        with self.lock:
            if "set_confidence" in payload:
                self.confidence = float(payload["set_confidence"])
                report["confidence"] = self.confidence
            if "old_fw_need_confirm" in payload:
                self.reloc_old_fw_need_confirm = bool(payload["old_fw_need_confirm"])
                report["old_fw_need_confirm"] = self.reloc_old_fw_need_confirm
            if "set_reloc_status" in payload:
                self.reloc_status = int(payload["set_reloc_status"])
                report["reloc_status"] = self.reloc_status
            pose = payload.get("set_pose")
            if isinstance(pose, dict):
                for k in ("x", "y", "angle"):
                    if k in pose:
                        setattr(self, k, float(pose[k]))
                if "current_station" in pose:
                    self.current_station = str(pose["current_station"])
                report["pose"] = {
                    "x": self.x,
                    "y": self.y,
                    "angle": self.angle,
                    "current_station": self.current_station,
                }
            stages = payload.get("inject_reloc_stages")
            if isinstance(stages, list):
                self._reloc_stage_ring = [int(s) for s in stages]
                self._reloc_stage_idx = 0
                report["inject_reloc_stages"] = self._reloc_stage_ring
            if "fail_1100" in payload:
                self.fail_1100 = bool(payload["fail_1100"])
                report["fail_1100"] = self.fail_1100
            if "fail_1005" in payload:
                self.fail_1005 = bool(payload["fail_1005"])
                report["fail_1005"] = self.fail_1005
            nav_xy = payload.get("navigate_xy")
            ov = payload.get("batch_override")
            if isinstance(ov, dict):
                self.batch_override.update(ov)
                report["batch_override"] = dict(self.batch_override)
            for key in (
                "dispatch_mode",
                "connect_fleet",
                "block_reason",
                "brake",
                "emergency",
                "soft_emc",
                "driver_emc",
                "manual_charge",
                "r_vx",
                "vx",
                "is_stop",
                "loadmap_status",
            ):
                if key in payload:
                    setattr(self, key, payload[key])
                    report[key] = payload[key]
        if isinstance(nav_xy, dict) and hasattr(self, "start_nav_xy"):
            report["navigate_xy"] = self.start_nav_xy(
                float(nav_xy.get("x", 0.0)),
                float(nav_xy.get("y", 0.0)),
                float(nav_xy.get("theta", 0.0)),
            )
        return report

    def current_lock_obj(self) -> Dict[str, Any]:
        locked = bool(self.locked_by)
        return {
            "locked": locked,
            "ip": self.locked_ip if locked else "",
            "port": self.locked_port if locked else 0,
            "type": self.locked_type if locked else 0,
            "nick_name": self.locked_by,
            "time_t": int(time.time()) if locked else 0,
            "desc": "",
        }

    def batch_status(self) -> Dict[str, Any]:
        if self.fail_1100:
            return {"ret_code": 40004, "err_msg": "mock 1100 failed"}
        res = {
            "ret_code": 0,
            "dispatch_mode": int(self.dispatch_mode),
            "connectFleet": bool(self.connect_fleet),
            "current_lock": self.current_lock_obj(),
            "vx": float(self.vx),
            "vy": float(self.vy),
            "w": float(self.w),
            "r_vx": float(self.r_vx),
            "r_vy": float(self.r_vy),
            "r_w": float(self.r_w),
            "is_stop": bool(self.is_stop),
            "blocked": bool(self.block_reason),
            "block_reason": int(self.block_reason),
            "brake": bool(self.brake),
            "emergency": bool(self.emergency),
            "soft_emc": bool(self.soft_emc),
            "driver_emc": bool(self.driver_emc),
            "manual_charge": bool(self.manual_charge),
            "task_status": self.task_status,
            "task_type": self.task_type,
            "target_id": self.target_id,
            "finished_path": list(self.finished_path),
            "unfinished_path": list(self.unfinished_path),
            "reloc_status": self.reloc_status,
            "loadmap_status": int(self.loadmap_status),
            "current_map": self.current_map,
            "vehicle_id": self.vehicle_id,
            "move_status_info": "",
            "confidence": self.confidence,
            "current_station": self.current_station,
            "motor_info": [],
            "errors": [],
            "fatals": [],
            "warnings": [],
        }
        res.update(self.batch_override)
        return res

    def speed(self) -> Dict[str, Any]:
        if self.fail_1005:
            return {"ret_code": 40004, "err_msg": "mock 1005 failed"}
        return {
            "ret_code": 0,
            "vx": float(self.vx),
            "vy": float(self.vy),
            "w": float(self.w),
            "r_vx": float(self.r_vx),
            "r_vy": float(self.r_vy),
            "r_w": float(self.r_w),
            "is_stop": bool(self.is_stop),
        }

    def block(self) -> Dict[str, Any]:
        return {
            "ret_code": 0,
            "blocked": bool(self.block_reason),
            "block_reason": int(self.block_reason),
        }

    def emergency_q(self) -> Dict[str, Any]:
        return {
            "ret_code": 0,
            "emergency": bool(self.emergency),
            "soft_emc": bool(self.soft_emc),
            "driver_emc": bool(self.driver_emc),
        }

    def control_owner(self) -> Dict[str, Any]:
        out = {"ret_code": 0}
        out.update(self.current_lock_obj())
        return out

    def path_info(self, source_id: str, target_id: str) -> Dict[str, Any]:
        smap = self.maps.get(self.current_map) or {}
        ids = {str(pt.get("instanceName") or "") for pt in (smap.get("advancedPointList") or [])}
        ids.update(self.stations.keys())
        if source_id and target_id and source_id in ids and target_id in ids:
            return {"ret_code": 0, "path": [source_id, target_id]}
        return {"ret_code": 0, "path": []}


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise OSError("closed")
        buf.extend(chunk)
    return bytes(buf)


def _handle_client(conn: socket.socket, state: RobokitMockState) -> None:
    conn.settimeout(60.0)
    try:
        while True:
            hdr = _recv_exact(conn, HEADER_SIZE)
            sync, ver, number, length, api_type, _res = struct.unpack(HEADER_FMT, hdr)
            if sync != 0x5A:
                break
            body = _recv_exact(conn, length) if length else b""
            payload: Dict[str, Any] = {}
            if body:
                try:
                    obj = json.loads(body.decode("utf-8"))
                    if isinstance(obj, dict):
                        payload = obj
                except json.JSONDecodeError:
                    payload = {}

            with state.lock:
                if api_type == API_LOC:
                    resp = state.loc()
                elif api_type == API_SPEED:
                    resp = state.speed()
                elif api_type == API_BLOCK:
                    resp = state.block()
                elif api_type == API_EMERGENCY:
                    resp = state.emergency_q()
                elif api_type == API_BATTERY:
                    resp = state.battery()
                elif api_type == API_LASER:
                    resp = state.laser()
                elif api_type == API_TASK:
                    resp = state.task(simple=bool(payload.get("simple")))
                elif api_type == API_RELOC_STATUS:
                    resp = state.reloc_status_q()
                elif api_type == API_MAP_LOAD:
                    resp = {"ret_code": 0, "loadmap_status": int(state.loadmap_status)}
                elif api_type == API_CONTROL_OWNER:
                    resp = state.control_owner()
                elif api_type == API_BATCH:
                    resp = state.batch_status()
                elif api_type == API_PATH_QUERY:
                    resp = state.path_info(
                        str(payload.get("source_id") or ""),
                        str(payload.get("target_id") or payload.get("id") or ""),
                    )
                elif api_type == API_RELOC:
                    resp = state.do_relocate(
                        x=payload.get("x"),
                        y=payload.get("y"),
                        angle=payload.get("angle"),
                        is_auto=payload.get("isAuto") if "isAuto" in payload else None,
                        home=payload.get("home") if "home" in payload else None,
                    )
                elif api_type == API_CONFIRM_LOC:
                    resp = state.do_confirm_loc()
                elif api_type == API_CANCEL_RELOC:
                    resp = state.do_cancel_reloc()
                elif api_type == API_GOTO:
                    tid = str(payload.get("id") or payload.get("target_id") or "")
                    src = str(payload.get("source_id") or "SELF_POSITION")
                    # 支持直接坐标导航：{"x":..,"y":..} 或 id 为 XY
                    if "x" in payload and "y" in payload and hasattr(state, "start_nav_xy"):
                        resp = state.start_nav_xy(float(payload["x"]), float(payload["y"]), float(payload.get("theta", 0.0)))
                    else:
                        resp = state.start_nav(tid, src)
                elif api_type == API_GOTO_LIST:
                    tasks = payload.get("move_task_list") or []
                    resp = state.start_nav_list(tasks if isinstance(tasks, list) else [])
                elif api_type == API_TRANSLATE:
                    resp = state.set_translate(payload) if hasattr(state, "set_translate") else {"ret_code": 40004, "err_msg": "no sim ext"}
                elif api_type == API_ROTATE:
                    resp = state.set_rotate(payload) if hasattr(state, "set_rotate") else {"ret_code": 40004, "err_msg": "no sim ext"}
                elif api_type == API_CANCEL:
                    if hasattr(state, "soft_stop"):
                        state.soft_stop()
                    resp = state.cancel()
                elif api_type == API_LOCK:
                    resp = state.do_lock(str(payload.get("nick_name") or ""))
                elif api_type == API_UNLOCK:
                    resp = state.do_unlock()
                elif api_type == API_MOCK_CTRL:
                    resp = state.mock_control(payload)
                elif api_type == API_MAP:
                    resp = state.map_info()
                elif api_type == API_STATION:
                    resp = state.station_list()
                elif api_type == API_SWITCH_MAP:
                    resp = state.switch_map(str(payload.get("map_name") or ""))
                elif api_type == API_DOWNLOAD_MAP:
                    resp = state.download_map(str(payload.get("map_name") or state.current_map))
                elif api_type == API_UPLOAD_MAP:
                    resp = state.upload_map(payload if payload else {})
                elif api_type == API_UPLOAD_SWITCH:
                    resp = state.upload_and_switch(payload if payload else {})
                elif api_type == API_ADD_OBS:
                    resp = state.add_obstacle(payload)
                elif api_type == API_ADD_GOBS:
                    resp = state.add_global_obstacle(payload)
                elif api_type == API_REM_OBS:
                    resp = state.remove_obstacle(str(payload.get("name") or ""))
                else:
                    resp = {"ret_code": 40004, "err_msg": f"mock unsupported api {api_type}"}

            raw = json.dumps(resp, separators=(",", ":")).encode("utf-8")
            out = struct.pack(
                HEADER_FMT,
                0x5A,
                ver,
                number,
                len(raw),
                api_type + 10000,
                b"\x00" * 6,
            ) + raw
            conn.sendall(out)
    except (OSError, struct.error):
        pass
    finally:
        try:
            conn.close()
        except OSError:
            pass


def _serve_push(host: str, port: int, state: RobokitMockState) -> None:
    """19301 — newline-delimited JSON pose push for dashboard testing."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(8)
    clients: list[socket.socket] = []
    cl_lock = threading.Lock()
    print(f"[robokit_mock] push listen {host}:{port}", flush=True)

    def accept_loop() -> None:
        while True:
            conn, _addr = srv.accept()
            conn.setblocking(False)
            with cl_lock:
                clients.append(conn)

    threading.Thread(target=accept_loop, daemon=True).start()
    while True:
        with state.lock:
            payload = {
                "x": state.x,
                "y": state.y,
                "angle": state.angle,
                "confidence": state.confidence,
                "current_station": state.current_station,
                "vehicle_id": state.vehicle_id,
            }
        line = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        dead: list[socket.socket] = []
        with cl_lock:
            for c in clients:
                try:
                    c.sendall(line)
                except OSError:
                    dead.append(c)
            for c in dead:
                try:
                    c.close()
                except OSError:
                    pass
                clients.remove(c)
        time.sleep(0.2)


def _serve_port(host: str, port: int, state: RobokitMockState) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(16)
    print(f"[robokit_mock] listen {host}:{port}", flush=True)
    while True:
        conn, _addr = srv.accept()
        threading.Thread(target=_handle_client, args=(conn, state), daemon=True).start()


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description="Robokit TCP mock for offline Demo dry-run")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument(
        "--ports",
        default=f"{PORT_STATUS},{PORT_CONTROL},{PORT_NAV},{PORT_CONFIG}",
        help="comma-separated ports",
    )
    args = ap.parse_args(argv)
    state = RobokitMockState()
    try:
        from agv_bridge.sim_api_ext import patch_mock_state
        patch_mock_state(state)
        print("[robokit_mock] sim_api_ext ON — dual lidar + 3055/3056 + xy nav", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[robokit_mock] sim_api_ext skipped: {exc}", flush=True)
    ports = [int(p.strip()) for p in args.ports.split(",") if p.strip()]
    for port in ports:
        threading.Thread(target=_serve_port, args=(args.host, port, state), daemon=True).start()
    threading.Thread(target=_serve_push, args=(args.host, PORT_PUSH, state), daemon=True).start()
    print(
        "[robokit_mock] ready — 3055/3056 velocity + dual lidar 1009 + map obstacles\n"
        f'  curl -X POST http://127.0.0.1:19999/api/env -H "Content-Type: application/json" '
        f'-d \'{{"mode":"demo","agv_host":"{args.host}"}}\'',
        flush=True,
    )
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
