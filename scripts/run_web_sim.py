#!/usr/bin/env python3
"""V0.1 真场景仿真 Web 栈（Three.js 第三人称 + smap 办公室 / 室外园区）。

无需 ROS2，Windows 可跑。控制仍走 Robokit 实车同款 API。
"""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ros2_ws" / "src"
sys.path.insert(0, str(SRC / "agv_bridge"))

from agv_bridge.robokit_mock_server import (  # noqa: E402
    API_MOCK_CTRL,
    HEADER_FMT,
    HEADER_SIZE,
    PORT_CONFIG,
    PORT_CONTROL,
    PORT_NAV,
    PORT_PUSH,
    PORT_STATUS,
    RobokitMockState,
    _serve_port,
    _serve_push,
)
from agv_bridge.sim_api_ext import patch_mock_state  # noqa: E402
from agv_bridge.sim_world import get_world  # noqa: E402
from agv_bridge.nav_trajectory_integrity import (  # noqa: E402
    TRAJ_KIND_BACKWARD_FUTURE,
    TRAJ_KIND_HISTORICAL_RETREAT,
    TRAJ_KIND_NONE,
    audit_snapshot_trajectory,
    enrich_trajectory_metadata,
    global_path_fingerprint,
)

WWW = ROOT / "ros2_ws" / "src" / "delivery_web" / "www"
WEB_SIM_PORT = 19999


def _read_build_commit() -> str:
    try:
        r = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            timeout=3,
        )
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip()
    except Exception:
        pass
    return "unknown"


def _port_listening_pids(port: int) -> list[int]:
    pids: list[int] = []
    try:
        out = subprocess.check_output(["netstat", "-ano"], text=True, errors="replace")
        token = f":{port}"
        for line in out.splitlines():
            upper = line.upper()
            if token in line and "LISTENING" in upper:
                parts = line.split()
                if parts:
                    try:
                        pids.append(int(parts[-1]))
                    except ValueError:
                        continue
    except Exception:
        pass
    return list(dict.fromkeys(pids))


def _ensure_single_sim_instance(port: int) -> None:
    pids = _port_listening_pids(port)
    if pids:
        print(f"[web_sim] ERROR: port {port} already LISTENING — PID(s): {', '.join(map(str, pids))}", flush=True)
        print("[web_sim] Stop the existing simulator before starting another instance.", flush=True)
        raise SystemExit(2)
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(("0.0.0.0", port))
    except OSError as e:
        print(f"[web_sim] ERROR: cannot bind port {port}: {e}", flush=True)
        raise SystemExit(2) from e
    finally:
        probe.close()


class RobokitTcp:
    def __init__(self, host: str = "127.0.0.1") -> None:
        self.host = host
        self._seq = 0

    def call(self, port: int, api: int, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        body = b""
        if payload:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._seq = (self._seq + 1) % 65536
        hdr = struct.pack(HEADER_FMT, 0x5A, 1, self._seq, len(body), api, b"\x00" * 6)
        with socket.create_connection((self.host, port), timeout=2.0) as sock:
            sock.sendall(hdr + body)
            rh = self._recv(sock, HEADER_SIZE)
            _s, _v, _n, length, _t, _r = struct.unpack(HEADER_FMT, rh)
            data = self._recv(sock, length) if length else b""
            if not data:
                return {}
            return json.loads(data.decode("utf-8"))

    @staticmethod
    def _recv(sock: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise OSError("closed")
            buf.extend(chunk)
        return bytes(buf)


class SimApp:
    def __init__(self) -> None:
        self.host = "127.0.0.1"
        self.build_commit = _read_build_commit()
        self.started_at = time.time()
        self.process_pid = os.getpid()
        self.state = RobokitMockState()
        patch_mock_state(self.state)
        self.state.build_commit = self.build_commit
        self.state.process_pid = self.process_pid
        self.state.sim_started_at = self.started_at
        self._snap_lock = threading.Lock()
        self._snap_full: Optional[Dict[str, Any]] = None
        self._snap_lite: Optional[Dict[str, Any]] = None
        self._snap_full_ts = 0.0
        self._snap_lite_ts = 0.0
        self._world_meta_ts = 0.0
        self._world_meta_cache: Dict[str, Any] = {}
        self._debug_snap_ts = 0.0
        self._snapshot_stop = threading.Event()
        self.state._snapshot_cache_refresh = self.refresh_snapshot_cache
        self.world = get_world()
        self.tcp = RobokitTcp(self.host)
        self.goal: Optional[Tuple[float, float]] = None
        self._start_mock()
        threading.Thread(target=self._snapshot_loop, daemon=True, name="sim_snapshot").start()

    def _refresh_world_meta_cache(self, now: float) -> None:
        if now - self._world_meta_ts < 1.0 and self._world_meta_cache:
            return
        self._world_meta_cache = {
            "banner": self.world.current_banner(),
            "scene": self.world.scene_info(),
            "obstacles": self.world.obstacle_list(),
            "scenario_movers": self.world.scenario_mover_list(),
            "actors": self.world.actor_list(),
            "chronicle": self.world.recent_chronicle(15),
            "pois": self.world.pois(),
            "stations": {s["id"]: s for s in self._stations()},
        }
        self._world_meta_ts = now

    def _snapshot_loop(self) -> None:
        """Background snapshot builder — must never run on physics or HTTP threads."""
        while not self._snapshot_stop.is_set():
            try:
                now = time.time()
                refresh_dbg = getattr(self.state, "_refresh_debug_snapshot", None)
                if callable(refresh_dbg) and now - self._debug_snap_ts >= 0.50:
                    refresh_dbg(now)
                    self._debug_snap_ts = now
                self.refresh_snapshot_cache()
            except Exception:
                pass
            self._snapshot_stop.wait(0.05)

    def refresh_snapshot_cache(self) -> None:
        """Build HTTP snapshots off the physics thread."""
        now = time.time()
        self._refresh_world_meta_cache(now)
        with self._snap_lock:
            if now - self._snap_lite_ts >= 0.20:
                self._snap_lite = self._build_snapshot(lite=True)
                self._snap_lite_ts = now
            if now - self._snap_full_ts >= 1.0:
                self._snap_full = self._build_snapshot(lite=False)
                self._snap_full_ts = now

    def get_snapshot(self, *, lite: bool = False) -> Dict[str, Any]:
        with self._snap_lock:
            cached = self._snap_lite if lite else (self._snap_full or self._snap_lite)
        if cached is None:
            snap = self._build_snapshot(lite=lite)
        else:
            snap = dict(cached)
            snap["updated_at"] = time.time()
        snap["sim_runtime"] = self._sim_runtime_payload()
        snap["build_commit"] = self.build_commit
        return snap

    def snapshot(self, *, lite: bool = False) -> Dict[str, Any]:
        return self.get_snapshot(lite=lite)

    def _sim_runtime_payload(self) -> Dict[str, Any]:
        rt = dict(getattr(self.state, "_sim_runtime", {}) or {})
        rt.setdefault("build_commit", self.build_commit)
        rt.setdefault("process_pid", self.process_pid)
        rt.setdefault("started_at", self.started_at)
        return rt

    def _start_mock(self) -> None:
        print("[web_sim] starting mock ports...", flush=True)
        for port in (PORT_STATUS, PORT_CONTROL, PORT_NAV, PORT_CONFIG):
            threading.Thread(
                target=_serve_port, args=(self.host, port, self.state), daemon=True
            ).start()
        threading.Thread(
            target=_serve_push, args=(self.host, PORT_PUSH, self.state), daemon=True
        ).start()
        time.sleep(0.5)
        self.state.do_lock("web_sim")
        self.world.emit("control_lock", "Web 仿真端已抢控制权")
        print("[web_sim] mock ready · scene=", self.world.scene_id, flush=True)

    def _build_snapshot(self, *, lite: bool = False) -> Dict[str, Any]:
        if lite:
            with self.state.lock:
                x = float(self.state.x)
                y = float(self.state.y)
                yaw = float(self.state.angle)
                spd_vx = float(self.state.vx)
                spd_w = float(self.state.w)
                bat_level = float(getattr(self.state, "battery_level", 0.87) or 0.87)
            loc = {"x": x, "y": y, "angle": yaw, "confidence": 0.95, "current_station": ""}
            spd = {"vx": spd_vx, "w": spd_w, "is_stop": abs(spd_vx) < 1e-3 and abs(spd_w) < 1e-3}
            bat = {"battery_level": bat_level, "charging": False}
            task = {"task_status": int(getattr(self.state, "task_status", 0) or 0), "target_id": ""}
            live = list(getattr(self.state, "_cached_laser_points", []) or [])
            meta = self._world_meta_cache or {}
            banner = meta.get("banner") or self.world.current_banner()
            scene = meta.get("scene") or self.world.scene_info()
        else:
            loc = self.tcp.call(PORT_STATUS, 1004, {})
            spd = self.tcp.call(PORT_STATUS, 1005, {})
            bat = self.tcp.call(PORT_STATUS, 1007, {})
            task = self.tcp.call(PORT_STATUS, 1020, {})
            x = float(loc.get("x", self.state.x))
            y = float(loc.get("y", self.state.y))
            yaw = float(loc.get("angle", self.state.angle))
            live = self.world.lidar_world_points(x, y, yaw, stride=1, step_deg=1.0)
            self.state._cached_laser_points = live
            banner = self.world.current_banner()
            scene = self.world.scene_info()
            meta = {}
        with self.state.lock:
            local_raw = list(getattr(self.state, "_path", []) or [])
            planned_raw = list(getattr(self.state, "_planned_path", []) or [])
            global_raw = list(getattr(self.state, "_global_path", []) or [])
            nav_mode = getattr(self.state, "_nav_mode", "idle")
            pending = bool(getattr(self.state, "_pending_confirm", False))
            goal_xy = getattr(self.state, "_goal_xy", None)
            local_n = int(getattr(self.state, "_local_replan_count", 0) or 0)
            global_n = int(getattr(self.state, "_global_replan_count", 0) or 0)
            candidates = list(getattr(self.state, "_path_candidates", []) or [])
            best_conf = float(getattr(self.state, "_best_confidence", 0.0) or 0.0)
            stuck_s = float(getattr(self.state, "_stuck_s", 0.0) or 0.0)
            track_mode = str(getattr(self.state, "_track_mode", "") or "")
            nav_phase = str(getattr(self.state, "_nav_phase", "forward") or "forward")
            ctrl_note = str(getattr(self.state, "_ctrl_note", "") or "")
            stop_reason = str(getattr(self.state, "_stop_reason", "NONE") or "NONE")
            mppi_vx = float(getattr(self.state, "_mppi_vx", 0.0) or 0.0)
            mppi_w = float(getattr(self.state, "_mppi_w", 0.0) or 0.0)
            cmd_vx_bs = float(getattr(self.state, "_cmd_vx_before_safety", 0.0) or 0.0)
            cmd_w_bs = float(getattr(self.state, "_cmd_w_before_safety", 0.0) or 0.0)
            cmd_vx_as = float(getattr(self.state, "_cmd_vx_after_safety", 0.0) or 0.0)
            cmd_w_as = float(getattr(self.state, "_cmd_w_after_safety", 0.0) or 0.0)
            front_near = float(getattr(self.state, "_front_near", 30.0) or 30.0)
            rear_near = float(getattr(self.state, "_rear_near", 30.0) or 30.0)
            collision = bool(getattr(self.state, "_collision", False))
            path_progress_s = float(getattr(self.state, "_path_progress_s", 0.0) or 0.0)
            goal_distance = float(getattr(self.state, "_goal_distance", 0.0) or 0.0)
            recovery_attempts = int(getattr(self.state, "_recovery_attempts", 0) or 0)
            safe_vx_reason = str(getattr(self.state, "_safe_vx_reason", "NORMAL") or "NORMAL")
            planner_state = str(getattr(self.state, "_planner_state", "NORMAL") or "NORMAL")
            planner_failure_reason = str(getattr(self.state, "_planner_failure_reason", "") or "")
            recovery_state = str(getattr(self.state, "_recovery_state", "NONE") or "NONE")
            recovery_attempt = int(getattr(self.state, "_recovery_attempt", 0) or 0)
            footprint_clearance_m = getattr(self.state, "_footprint_clearance_m", None)
            predicted_min_clearance_m = getattr(self.state, "_predicted_min_clearance_m", None)
            nav_ui_severity = str(getattr(self.state, "_nav_ui_severity", "NORMAL") or "NORMAL")
            control_mode = str(getattr(self.state, "_control_mode", "mppi") or "mppi")
            emergency = bool(getattr(self.state, "emergency", False) or getattr(self.state, "soft_emc", False))
            raw_global = list(getattr(self.state, "_raw_global_path", []) or [])
            planning_metrics = dict(getattr(self.state, "_planning_metrics", {}) or {})
            debug_blob = dict(getattr(self.state, "_debug_snapshot", {}) or {})
            path_lateral = float(getattr(self.state, "_path_lateral_m", 0.0) or 0.0)
            path_heading = float(getattr(self.state, "_path_heading_err", 0.0) or 0.0)
            physical_corridor = dict(getattr(self.state, "_physical_corridor", {}) or {})
            retreat_corridor = dict(getattr(self.state, "_retreat_trajectory", {}) or {})
            nav_scene_id = getattr(self.state, "_nav_scene_id", None)
            planner_cycle_id = int(getattr(self.state, "_planner_cycle_id", 0) or 0)
            planner_input_ts = float(getattr(self.state, "_planner_input_timestamp", 0) or 0)
            planner_start_ts = float(getattr(self.state, "_planner_start_timestamp", 0) or 0)
            planner_finish_ts = float(getattr(self.state, "_planner_finish_timestamp", 0) or 0)
            scene_revision = int(getattr(self.state, "_nav_scene_revision", 0) or 0)
            scene_warmup = int(getattr(self.state, "_scene_warmup_remaining", 0) or 0)
            global_reference = dict(getattr(self.state, "_global_reference", {}) or {})
            local_cands_layer = dict(getattr(self.state, "_local_candidates_layer", {}) or {})
            selected_local = dict(getattr(self.state, "_selected_local", {}) or {})
            kinematic_validation = dict(getattr(self.state, "_kinematic_validation", {}) or {})
            open_space_forensics = dict(getattr(self.state, "_open_space_forensics", {}) or {})
            local_plan = dict(getattr(self.state, "_local_plan", {}) or {})
            path_rev = int(getattr(self.state, "_global_path_revision", 0) or 0)
            obstacle_preview = dict(getattr(self.state, "_obstacle_preview", {}) or {})
            command_own = dict(getattr(self.state, "_command_ownership", {}) or {})
            motion = dict(getattr(self.state, "_motion", {}) or {})
        navigating = nav_mode in ("tracking", "avoid", "planned", "planner_debug")
        if lite:
            surround = list(getattr(self.state, "_cached_surround_cloud", []) or live)
            map_preview = list(getattr(self.state, "_cached_map_cloud", []) or [])
        elif navigating and nav_mode != "planner_debug":
            surround = live
            map_preview = self.world.map_cloud(max_n=5000)
            self.state._cached_surround_cloud = surround
            self.state._cached_map_cloud = map_preview
        else:
            surround = self.world.local_cloud(x, y, radius=35.0, max_n=7000 if not lite else 2000)
            map_preview = surround
            if not lite:
                self.state._cached_surround_cloud = surround
                self.state._cached_map_cloud = map_preview
        # 小地图用全局；主视图蓝带默认 executed band（_path）
        global_pts = [{"x": p[0], "y": p[1]} for p in (global_raw or local_raw)]
        raw_pts = [{"x": p[0], "y": p[1]} for p in raw_global] if raw_global else []
        local_pts = [{"x": p[0], "y": p[1]} for p in local_raw] if local_raw else []
        planned_pts = [{"x": p[0], "y": p[1]} for p in planned_raw] if planned_raw else []
        agv_dict = {
            "x": x,
            "y": y,
            "angle": yaw,
            "yaw_deg": yaw * 180.0 / math.pi,
            "battery": int(float(bat.get("battery_level", 0.87)) * 100)
            if float(bat.get("battery_level", 0.87)) <= 1.0
            else int(bat.get("battery_level", 87)),
            "battery_level": float(bat.get("battery_level", 0.87)),
            "charging": bool(bat.get("charging", False)),
            "vx": float(spd.get("vx", 0.0)),
            "w": float(spd.get("w", 0.0)),
            "speed": abs(float(spd.get("vx", 0.0))),
            "is_stop": bool(spd.get("is_stop", True)),
            "task_status": int(task.get("task_status", 0)),
            "target_id": task.get("target_id") or "",
            "blocked": bool(getattr(self.state, "block_reason", 0)),
            "emergency": False,
            "soft_emc": False,
            "current_station": loc.get("current_station") or "",
            "confidence": float(loc.get("confidence", 0.95)),
            "model": "AMB-150",
        }
        tnow = time.time()
        pt_enriched = (
            enrich_trajectory_metadata(
                dict(physical_corridor),
                vehicle=agv_dict,
                now=tnow,
                current_scene_id=nav_scene_id,
                current_cycle_id=planner_cycle_id or None,
            )
            if physical_corridor
            else {}
        )
        retreat_enriched = (
            enrich_trajectory_metadata(
                dict(retreat_corridor),
                vehicle=agv_dict,
                now=tnow,
                current_scene_id=nav_scene_id,
                current_cycle_id=planner_cycle_id or None,
            )
            if retreat_corridor
            else {}
        )
        # Scene mismatch / wrong family cannot be Future Local Plan.
        # STALE forward stays visible in API with control_eligible=false (Web will not draw it as Future).
        reject = str((pt_enriched or {}).get("integrity_reject") or "")
        kind = str((pt_enriched or {}).get("trajectory_kind") or "")
        scene_mismatch = reject == "TRAJECTORY_STALE_SCENE" or (
            bool(pt_enriched)
            and scene_warmup > 0
            and str(pt_enriched.get("scene_id")) != str(nav_scene_id)
        )
        if (
            not pt_enriched
            or scene_mismatch
            or reject == "TRAJECTORY_CYCLE_MISMATCH"
            or kind in (TRAJ_KIND_HISTORICAL_RETREAT, TRAJ_KIND_BACKWARD_FUTURE, TRAJ_KIND_NONE)
        ):
            physical_for_nav = None
        else:
            physical_for_nav = pt_enriched
        retreat_reject = str((retreat_enriched or {}).get("integrity_reject") or "")
        if retreat_reject == "TRAJECTORY_STALE_SCENE" or (
            scene_warmup > 0 and retreat_enriched and str(retreat_enriched.get("scene_id")) != str(nav_scene_id)
        ):
            retreat_for_nav = None
        else:
            retreat_for_nav = retreat_enriched or None
        tracking_local = bool(command_own.get("tracking_local_plan"))
        follow_src = str(command_own.get("follow_path_source") or "")
        local_plan_status = str((local_plan or {}).get("status") or "NONE")
        phys_ineligible = bool(physical_for_nav) and physical_for_nav.get("control_eligible") is False
        if not local_plan:
            local_plan_status = "NONE"
        elif tracking_local and phys_ineligible:
            local_plan_status = "TRACKING_STALE_POSES"
        elif not tracking_local:
            local_plan_status = "STALE_FALLBACK" if phys_ineligible else "NOT_TRACKING"
        viz_mismatch = bool(
            (local_plan and local_plan.get("poses") and (not tracking_local or phys_ineligible))
            or (local_cands_layer and (local_cands_layer.get("items") or []) and not tracking_local)
        )
        cmd_side = str(command_own.get("command_side") or "STRAIGHT")
        try:
            from agv_bridge.nav_command_ownership import command_actual_consistency, path_heading_rad

            global_path_heading = path_heading_rad(global_pts, float(x), float(y))
            cmd_act = command_actual_consistency(
                approved_omega=float(cmd_w_as),
                actual_omega=float(spd.get("w", 0.0) or 0.0),
            )
        except Exception:
            global_path_heading = None
            cmd_act = {}
        snap = {
            "updated_at": time.time(),
            "env": {
                "mode": "sim_true_scene",
                "adapter_connected": True,
                "agv_host": self.host,
                "control_locked": True,
                "control_wanted": True,
                "backend": "robokit_mock_3055",
                "sim_engine": "smap_occupancy+dual_lidar+threejs",
            },
            "agv_link": {"status": "ONLINE"},
            "agv": agv_dict,
            "meta": {
                "map_name": scene["name"],
                "vehicle_model": "AMB-150",
                "scene": scene,
            },
            "scene": scene,
            "laser": {
                "live_lidar": True,
                "source": "robokit_1009_dual",
                "label": f"DUAL LIDAR {len(live)} pts",
                "points": live,
                "beam_count": len(live),
                "message": "front+rear diagonal",
                "point_count": len(live),
            },
            "perception": {
                "surround_cloud": surround,
                "mode": "live_lidar" if navigating else "panorama_map",
            },
            "obstacles": meta.get("obstacles") if lite and meta else self.world.obstacle_list(),
            "scenario_movers": meta.get("scenario_movers") if lite and meta else self.world.scenario_mover_list(),
            "actors": meta.get("actors") if lite and meta else self.world.actor_list(),
            "chronicle": meta.get("chronicle") if lite and meta else self.world.recent_chronicle(15),
            "banner": banner,
            "nav": {
                "mode": nav_mode,
                "pending_confirm": pending,
                "goal": {"x": goal_xy[0], "y": goal_xy[1]} if goal_xy else None,
                "path": global_pts,
                "raw_global_path": raw_pts,
                "processed_global_path": global_pts,
                "local_path": local_pts,
                "planned_path": planned_pts,
                "executed_path": local_pts,
                "candidates": candidates,
                "confidence": best_conf,
                "stuck_s": round(stuck_s, 1),
                "track_mode": track_mode,
                "phase": nav_phase,
                "ctrl_note": ctrl_note,
                "control_mode": control_mode,
                "global_replan_count": global_n,
                "local_replan_count": local_n,
                "planner": "global_astar_quality+local_mppi",
                "path_color": scene["path_color"],
                "scene_kind": scene["kind"],
                "mppi_vx": round(mppi_vx, 4),
                "mppi_w": round(mppi_w, 4),
                "cmd_vx_before_safety": round(cmd_vx_bs, 4),
                "cmd_w_before_safety": round(cmd_w_bs, 4),
                "cmd_vx_after_safety": round(cmd_vx_as, 4),
                "cmd_w_after_safety": round(cmd_w_as, 4),
                "state_vx": round(float(spd.get("vx", 0.0)), 4),
                "state_w": round(float(spd.get("w", 0.0)), 4),
                "path_progress_s": round(path_progress_s, 3),
                "goal_distance": round(goal_distance, 3),
                "recovery_attempts": recovery_attempts,
                "stop_reason": stop_reason,
                "safe_vx_reason": safe_vx_reason,
                "planner_state": planner_state,
                "planner_failure_reason": planner_failure_reason,
                "recovery_state": recovery_state,
                "recovery_attempt": recovery_attempt,
                "footprint_clearance_m": footprint_clearance_m,
                "predicted_min_clearance_m": predicted_min_clearance_m,
                "nav_ui_severity": nav_ui_severity,
                "requested_vx": round(mppi_vx, 4),
                "approved_vx": round(cmd_vx_as, 4),
                "requested_omega": round(mppi_w, 4),
                "approved_omega": round(cmd_w_as, 4),
                "candidate_count": len(candidates),
                "planning": planning_metrics,
                "path_lateral_error": round(path_lateral, 4),
                "path_heading_error": round(path_heading, 4),
                "blue_band_means": "FORWARD_FUTURE control-eligible only; historical/backward are retreat_trajectory",
                "physical_trajectory": physical_for_nav,
                "retreat_trajectory": retreat_for_nav,
                "historical_retreat": retreat_for_nav if (retreat_for_nav or {}).get("trajectory_kind") == "HISTORICAL_RETREAT" else None,
                "nav_scene_id": nav_scene_id,
                "scene_id": nav_scene_id,
                "scene_revision": scene_revision,
                "planner_cycle_id": planner_cycle_id,
                "planner_input_timestamp": planner_input_ts or None,
                "planner_start_timestamp": planner_start_ts or None,
                "planner_finish_timestamp": planner_finish_ts or None,
                "command_source": command_own.get("command_source") or "OTHER",
                "command_source_module": command_own.get("command_source_module"),
                "command_source_reason": command_own.get("command_source_reason") or command_own.get("command_reason"),
                "command_reason": command_own.get("command_reason"),
                "last_command_writer": command_own.get("last_command_writer"),
                "command_write_trace": command_own.get("command_write_trace") or [],
                "command_side": cmd_side,
                "command_fallback": command_own.get("fallback") or "NONE",
                "safety_direction_override": bool(command_own.get("safety_direction_override")),
                "follow_path_source": follow_src or None,
                "visualization_control_mismatch": viz_mismatch,
                "local_plan_status": local_plan_status,
                "pp_w": command_own.get("pp_w"),
                "motion": motion or {},
                "motion_health": (motion or {}).get("health") or "NORMAL",
                "limited_vx": (motion or {}).get("limited_vx"),
                "limited_omega": (motion or {}).get("limited_omega"),
                "speed_limit_reason": (motion or {}).get("speed_limit_reason"),
                "curve_vmax": (motion or {}).get("curve_vmax"),
                "future_max_abs_kappa": (motion or {}).get("future_max_abs_kappa"),
                "turn": (motion or {}).get("turn") or dict(getattr(self.state, "_turn_exec", {}) or {}),
                "turn_readiness": ((motion or {}).get("turn") or {}).get("turn_readiness"),
                "turn_feasibility": ((motion or {}).get("turn") or {}).get("turn_feasibility"),
                "command_age_ms": ((motion or {}).get("turn") or {}).get("command_age_ms"),
                "global_path_heading": None if global_path_heading is None else round(float(global_path_heading), 4),
                "command_actual_sign": cmd_act.get("command_actual_sign"),
                "actuator_direction_mismatch": bool(cmd_act.get("actuator_direction_mismatch")),
                "command_actual_magnitude_error": cmd_act.get("command_actual_magnitude_error"),
                "global_reference": self._preview_summary(global_reference),
                "local_plan": {
                    "plan_id": local_plan.get("plan_id"),
                    "revision": local_plan.get("revision"),
                    "horizon_s": local_plan.get("horizon_s"),
                    "horizon_m": local_plan.get("horizon_m"),
                    "selected_candidate": local_plan.get("selected_candidate"),
                    "status": local_plan.get("status"),
                    "active": local_plan.get("active"),
                    "speed_target": local_plan.get("speed_target"),
                    "kinematic_valid": local_plan.get("kinematic_valid"),
                    "min_clearance": local_plan.get("min_clearance"),
                    "source": "ROLLING_LOCAL_PLANNER",
                    "tracking_for_control": tracking_local,
                    "display_as_execution": tracking_local,
                    "poses": (local_plan.get("poses") or [])[:64],
                }
                if local_plan
                else {"active": False, "source": "ROLLING_LOCAL_PLANNER", "tracking_for_control": False, "display_as_execution": False},
                "local_candidates": {
                    "count": local_cands_layer.get("count"),
                    "valid_count": local_cands_layer.get("valid_count"),
                    "max_distance_m": local_cands_layer.get("max_distance_m"),
                    "mean_distance_m": local_cands_layer.get("mean_distance_m"),
                    "source": local_cands_layer.get("source") or "DEBUG_CANDIDATES",
                    "role": "DEBUG_NOT_COMMAND",
                    "highlight_selected": bool(tracking_local and not phys_ineligible),
                    "items": (local_cands_layer.get("items") or [])[:12],
                }
                if local_cands_layer
                else {"count": 0, "items": []},
                "selected_local": selected_local or None,
                "kinematic_validation": self._kinematic_summary(kinematic_validation),
                "open_space_forensics": self._open_space_summary(open_space_forensics),
                "global_path_revision": path_rev,
                "global_path_hash": global_path_fingerprint(global_pts),
                "safety_envelope": {
                    "front_near": round(front_near, 3),
                    "rear_near": round(rear_near, 3),
                    "stop_reason": stop_reason,
                },
            },
            "debug": debug_blob,
            "safety": {
                "front_near": round(front_near, 3),
                "rear_near": round(rear_near, 3),
                "collision": collision,
                "emergency": emergency,
                "obstacle_blocked": stop_reason in ("FRONT_OBSTACLE", "REAR_OBSTACLE", "COLLISION_GUARD"),
                "stop_reason": stop_reason,
                "safe_vx_reason": safe_vx_reason,
                "planner_state": planner_state,
                "recovery_state": recovery_state,
                "nav_ui_severity": nav_ui_severity,
                "footprint_clearance_m": footprint_clearance_m,
                "predicted_min_clearance_m": predicted_min_clearance_m,
            },
            "stations": meta.get("stations") if lite and meta.get("stations") else {s["id"]: s for s in self._stations()},
            "pois": meta.get("pois") if lite and meta.get("pois") is not None else self.world.pois(),
            "map": {
                "cloud": map_preview,
                "stations": self._stations(),
                "available": self.world.list_scenes(),
                "smap_file": scene["name"],
                "bounds": scene["bounds"],
                "path_color": scene["path_color"],
                "kind": scene["kind"],
            },
            "vision": {},
            "devices": {},
            "route_task": {},
            "obstacle_preview": obstacle_preview,
        }
        snap["trajectory_integrity"] = audit_snapshot_trajectory(snap)
        return snap

    def _stations(self) -> list:
        return self.world.pois()

    @staticmethod
    def _preview_summary(gref: Dict[str, Any]) -> Dict[str, Any]:
        """Compact /api/state field. Full poses via GET /api/nav/preview."""
        if not gref:
            return {
                "status": "NO_GLOBAL_PATH",
                "preview_m": 0.0,
                "kinematic_valid": None,
                "kinematic_status": "NOT_VALIDATED",
            }
        poses = gref.get("poses") or gref.get("centerline") or []
        # Keep a thinned centerline in /api/state so the main map can draw Global Reference
        # without requiring a second fetch. Cap to ~80 points.
        thin = poses
        if len(poses) > 80:
            step = max(1, len(poses) // 80)
            thin = poses[::step]
            if thin[-1] is not poses[-1]:
                thin = list(thin) + [poses[-1]]
        return {
            "status": gref.get("status"),
            "geometry_status": gref.get("geometry_status") or "REFERENCE_ONLY",
            "preview_m": gref.get("preview_m"),
            "remaining_m": gref.get("remaining_m"),
            "preview_reason": gref.get("preview_reason"),
            "preview_point_count": gref.get("preview_point_count"),
            "first_turn_distance_m": gref.get("first_turn_distance_m"),
            "heading_change_deg": gref.get("heading_change_deg"),
            "kinematic_valid": gref.get("kinematic_valid"),
            "kinematic_status": gref.get("kinematic_status") or "NOT_VALIDATED",
            "first_invalid_distance_m": gref.get("first_invalid_distance_m"),
            "speed_limited": gref.get("speed_limited"),
            "max_curvature": gref.get("max_curvature"),
            "min_turn_radius_m": gref.get("min_turn_radius_m"),
            "path_revision": gref.get("path_revision"),
            "path_exists": gref.get("path_exists"),
            "path_length_m": gref.get("path_length_m"),
            "poses": thin,
            "left_edge": (gref.get("left_edge") or [])[:80],
            "right_edge": (gref.get("right_edge") or [])[:80],
            "note": gref.get("note") or "REFERENCE_ONLY",
            "controls_vehicle": False,
        }

    @staticmethod
    def _kinematic_summary(kv: Dict[str, Any]) -> Dict[str, Any]:
        if not kv:
            return {
                "status": "NOT_VALIDATED",
                "kinematic_valid": None,
                "kinematic_status": "NOT_VALIDATED",
                "controls_vehicle": False,
            }
        keys = (
            "status",
            "valid",
            "kinematic_valid",
            "kinematic_status",
            "reason",
            "primary_reason",
            "raw_path_length_m",
            "validated_path_length_m",
            "max_curvature",
            "min_turn_radius_m",
            "reference_speed_mps",
            "max_required_w_rad_s",
            "max_feasible_speed_mps",
            "min_clearance_m",
            "swept_collision",
            "first_invalid_index",
            "first_invalid_distance_m",
            "speed_limited",
            "speed_limited_from_m",
            "path_revision",
            "validation_revision",
            "validation_id",
            "cache_hit",
            "compute_ms",
            "needs_reverse_maneuver",
            "controls_vehicle",
        )
        out = {k: kv.get(k) for k in keys if k in kv}
        out.setdefault("status", kv.get("kinematic_status") or "NOT_VALIDATED")
        out.setdefault("kinematic_valid", kv.get("kinematic_valid"))
        out.setdefault("controls_vehicle", False)
        return out

    @staticmethod
    def _open_space_summary(fos: Dict[str, Any]) -> Dict[str, Any]:
        """Compact /api/state open-space forensics (full blob via GET /api/nav/forensics/open-space)."""
        if not fos:
            return {}
        loc = fos.get("local_selector") if isinstance(fos.get("local_selector"), dict) else {}
        mppi = fos.get("mppi") if isinstance(fos.get("mppi"), dict) else {}
        cmd = fos.get("command") if isinstance(fos.get("command"), dict) else {}
        pol = fos.get("policy") if isinstance(fos.get("policy"), dict) else {}
        diag = fos.get("diagnostics") if isinstance(fos.get("diagnostics"), dict) else {}
        return {
            "scene": fos.get("scene"),
            "policy_scene": fos.get("policy_scene") or pol.get("scene"),
            "global_preview_m": fos.get("global_preview_m"),
            "local_horizon_s": loc.get("horizon_s"),
            "local_max_distance_m": loc.get("max_distance_m"),
            "compare_called": loc.get("compare_called"),
            "compare_reason": loc.get("compare_reason"),
            "mppi_horizon_s": mppi.get("horizon_s"),
            "mean_vx": mppi.get("mean_vx_after") or mppi.get("mean_vx_before"),
            "vx_raw": mppi.get("vx_raw"),
            "vx_cmd": mppi.get("vx_cmd"),
            "vx_scale": mppi.get("vx_scale") or pol.get("vx_scale"),
            "requested_vx": cmd.get("requested_vx"),
            "safe_vx": cmd.get("safe_vx"),
            "state_vx": cmd.get("state_vx"),
            "coverage_ratio": diag.get("coverage_ratio"),
            "short_horizon_reason": diag.get("short_horizon_reason"),
            "safety_clamp": diag.get("safety_clamp"),
            "controls_vehicle": False,
        }

    @staticmethod
    def _local_path(path: list, x: float, y: float, horizon_m: float = 5.0) -> list:
        """主视图只用车前方一段局部引导带。"""
        if not path:
            return []
        # 找最近点
        best_i = 0
        best_d = 1e18
        for i, p in enumerate(path):
            d = math.hypot(float(p["x"]) - x, float(p["y"]) - y)
            if d < best_d:
                best_d = d
                best_i = i
        out = [{"x": x, "y": y}]
        acc = 0.0
        prev = (x, y)
        for p in path[best_i:]:
            px, py = float(p["x"]), float(p["y"])
            acc += math.hypot(px - prev[0], py - prev[1])
            out.append({"x": px, "y": py})
            prev = (px, py)
            if acc >= horizon_m:
                break
        return out

    def plan_goal(self, x: float, y: float, target_id: str = "") -> Dict[str, Any]:
        self.goal = (x, y)
        if hasattr(self.state, "plan_nav_xy"):
            resp = self.state.plan_nav_xy(x, y, target_id=target_id)
        else:
            resp = self.tcp.call(PORT_NAV, 3051, {"x": x, "y": y, "theta": 0.0})
        ok = resp.get("ret_code", 1) == 0
        return {
            "success": ok,
            "api": resp,
            "goal": {"x": x, "y": y},
            "path": resp.get("path") or [],
            "pending_confirm": bool(resp.get("pending_confirm")),
            "waypoints": resp.get("waypoints", 0),
        }

    def confirm(self) -> Dict[str, Any]:
        if hasattr(self.state, "confirm_nav"):
            resp = self.state.confirm_nav()
        else:
            resp = {"ret_code": 1}
        return {"success": resp.get("ret_code", 1) == 0, "api": resp}

    def set_goal(self, x: float, y: float, auto_start: bool = False) -> Dict[str, Any]:
        if auto_start:
            self.goal = (x, y)
            resp = self.state.start_nav_xy(x, y, auto_start=True)
            return {"success": resp.get("ret_code", 1) == 0, "api": resp, "goal": {"x": x, "y": y}}
        return self.plan_goal(x, y)

    def navigate_poi(self, poi_id: str) -> Dict[str, Any]:
        poi = self.world.find_poi(poi_id)
        if not poi:
            return {"success": False, "message": f"点位不存在: {poi_id}"}
        return self.plan_goal(poi.x, poi.y, target_id=poi.id)

    def set_scene(self, scene_id: str) -> Dict[str, Any]:
        if hasattr(self.state, "apply_scene"):
            r = self.state.apply_scene(scene_id)
        else:
            r = self.world.set_scene(scene_id)
        return r

    def cmd_vel(self, vx: float, w: float) -> Dict[str, Any]:
        resp = self.tcp.call(PORT_CONTROL, 3055, {"vx": vx, "vy": 0.0, "w": w})
        return {"success": resp.get("ret_code", 1) == 0, "api": resp}

    def cancel(self) -> Dict[str, Any]:
        if hasattr(self.state, "soft_stop"):
            resp = self.state.soft_stop()
        else:
            resp = self.tcp.call(PORT_NAV, 3003, {})
            with self.state.lock:
                self.state._pending_confirm = False
                self.state._nav_mode = "idle"
                self.state._path = []
                self.state._goal_xy = None
        self.world.emit("cancel", "MANUAL_STOP", level="warn")
        return {"success": True, "api": resp}


APP: Optional[SimApp] = None


def make_handler(www: Path):
    class Handler(SimpleHTTPRequestHandler):
        extensions_map = {
            **getattr(SimpleHTTPRequestHandler, "extensions_map", {}),
            ".html": "text/html; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".json": "application/json; charset=utf-8",
        }

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(www), **kwargs)

        def log_message(self, fmt: str, *args) -> None:
            if "/api/" in (args[0] if args else ""):
                return
            super().log_message(fmt, *args)

        def _json(self, code: int, obj: Any) -> None:
            raw = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(raw)
            # P0-B-0: sampled API access log (no /api/state body dump)
            try:
                from agv_bridge.nav_observability import OBS

                t0 = getattr(self, "_req_t0", None)
                dur = (time.time() - t0) * 1000.0 if t0 else 0.0
                OBS.log_api_access(
                    method=str(getattr(self, "_req_method", "GET")),
                    path=str(getattr(self, "_req_path", "") or ""),
                    status=int(code),
                    duration_ms=dur,
                    client=str(getattr(self, "client_address", ("",))[0] or ""),
                    response_size=len(raw),
                    error=None if code < 400 else str((obj or {}).get("message") or code),
                )
            except Exception:
                pass

        def _begin_req(self, method: str) -> None:
            self._req_t0 = time.time()
            self._req_method = method
            self._req_path = urlparse(self.path).path

        def _read_json(self) -> Dict[str, Any]:
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8"))
            except json.JSONDecodeError:
                return {}

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            self._begin_req("GET")
            path = urlparse(self.path).path
            qs = parse_qs(urlparse(self.path).query)
            assert APP is not None
            if path in ("/", "/index.html"):
                # 新主界面
                self.path = "/sim_main.html"
                return super().do_GET()
            if path in ("/api/state", "/api/heartbeat"):
                lite = qs.get("lite", ["0"])[0].lower() in ("1", "true", "yes")
                snap = APP.get_snapshot(lite=lite)
                if path == "/api/heartbeat":
                    self._json(
                        200,
                        {
                            "ok": True,
                            "agv": snap["agv"],
                            "banner": snap["banner"],
                            "laser": {"label": snap["laser"]["label"], "live_lidar": True},
                            "env": snap["env"],
                            "nav": snap["nav"],
                            "scene": snap["scene"],
                        },
                    )
                else:
                    self._json(200, snap)
                return
            if path == "/api/version":
                self._json(
                    200,
                    {
                        "version": "0.1.0-sim-true",
                        "mode": "true_scene_sim",
                        "engine": "smap+threejs",
                    },
                )
                return
            if path == "/api/scenes":
                self._json(200, {"scenes": APP.world.list_scenes(), "current": APP.world.scene_info()})
                return
            if path == "/api/pois":
                q = qs.get("q", [""])[0]
                pois = APP.world.pois()
                if q:
                    ql = q.lower()
                    pois = [p for p in pois if ql in p["id"].lower() or ql in str(p.get("kind", "")).lower()]
                self._json(200, {"pois": pois})
                return
            if path == "/api/chronicle":
                self._json(
                    200,
                    {"banner": APP.world.current_banner(), "events": APP.world.recent_chronicle()},
                )
                return
            if path in ("/api/nav/debug", "/api/debug/nav"):
                if hasattr(APP.state, "get_nav_debug"):
                    self._json(200, APP.state.get_nav_debug())
                else:
                    self._json(200, {"success": False, "message": "debug unsupported"})
                return
            if path in ("/api/nav/preview", "/api/preview"):
                if hasattr(APP.state, "get_nav_preview"):
                    self._json(200, APP.state.get_nav_preview())
                else:
                    self._json(404, {"success": False, "error": "preview unavailable"})
                return
            if path in ("/api/nav/local-plan", "/api/nav/local_plan"):
                if hasattr(APP.state, "get_nav_local_plan"):
                    self._json(200, APP.state.get_nav_local_plan())
                else:
                    self._json(404, {"success": False, "error": "local-plan unavailable"})
                return
            if path in ("/api/nav/forensics/open-space", "/api/nav/forensics/open_space"):
                if hasattr(APP.state, "get_open_space_forensics"):
                    self._json(200, APP.state.get_open_space_forensics())
                else:
                    self._json(404, {"success": False, "error": "forensics unavailable"})
                return
            if path in ("/api/nav/forensics/lookahead", "/api/nav/forensics/look_ahead"):
                if hasattr(APP.state, "get_lookahead_forensics"):
                    self._json(200, APP.state.get_lookahead_forensics())
                else:
                    self._json(404, {"success": False, "error": "lookahead forensics unavailable"})
                return
            if path in ("/api/nav/obstacle-preview", "/api/nav/obstacle_preview"):
                if hasattr(APP.state, "get_obstacle_preview"):
                    self._json(200, APP.state.get_obstacle_preview())
                else:
                    self._json(404, {"success": False, "error": "obstacle preview unavailable"})
                return
            if path in ("/api/nav/runtime", "/api/sim/runtime"):
                snap = APP.get_snapshot(lite=True)
                self._json(
                    200,
                    {
                        "success": True,
                        "sim_runtime": snap.get("sim_runtime") or {},
                        "build_commit": snap.get("build_commit"),
                        "agv": {"x": (snap.get("agv") or {}).get("x"), "y": (snap.get("agv") or {}).get("y")},
                        "nav_mode": (snap.get("nav") or {}).get("mode"),
                        "generated_at": time.time(),
                    },
                )
                return
            if path in ("/api/nav/safety",):
                try:
                    snap = APP.snapshot()
                    nav = snap.get("nav") or {}
                    safety = snap.get("safety") or {}
                    dbg = snap.get("debug") or {}
                    vc = dbg.get("velocity_chain") or {}
                    self._json(
                        200,
                        {
                            "success": True,
                            "safe_vx": vc.get("safe_vx") or nav.get("cmd_vx_after_safety"),
                            "safe_vx_reason": nav.get("safe_vx_reason") or safety.get("safe_vx_reason"),
                            "requested_vx": nav.get("requested_vx") or vc.get("requested_vx"),
                            "approved_vx": nav.get("approved_vx") or vc.get("approved_vx"),
                            "requested_omega": nav.get("requested_omega") or vc.get("requested_omega"),
                            "approved_omega": nav.get("approved_omega") or vc.get("approved_omega"),
                            "footprint_clearance_m": nav.get("footprint_clearance_m") or safety.get("footprint_clearance_m"),
                            "predicted_min_clearance_m": nav.get("predicted_min_clearance_m") or safety.get("predicted_min_clearance_m"),
                            "front_near": safety.get("front_near"),
                            "rear_near": safety.get("rear_near"),
                            "stop_reason": nav.get("stop_reason") or safety.get("stop_reason"),
                            "nav_ui_severity": nav.get("nav_ui_severity") or safety.get("nav_ui_severity"),
                            "braking_calibration_status": "CALIBRATION_REQUIRED",
                        },
                    )
                except Exception as exc:
                    self._json(500, {"success": False, "error": str(exc)})
                return
            if path in ("/api/nav/recovery",):
                try:
                    snap = APP.snapshot()
                    nav = snap.get("nav") or {}
                    dbg = snap.get("debug") or {}
                    rec = dbg.get("execution_recovery") or dbg.get("recovery") or {}
                    self._json(
                        200,
                        {
                            "success": True,
                            "planner_state": nav.get("planner_state") or rec.get("planner_state"),
                            "planner_failure_reason": nav.get("planner_failure_reason") or rec.get("planner_failure_reason"),
                            "recovery_state": nav.get("recovery_state") or rec.get("recovery_state"),
                            "recovery_attempt": nav.get("recovery_attempt") or rec.get("recovery_attempt"),
                            "recovery_attempts": nav.get("recovery_attempts") or rec.get("recovery_attempts"),
                            "max_attempts": rec.get("max_attempts"),
                            "last_recovery_action": rec.get("last_recovery_action"),
                            "phase": rec.get("phase") or nav.get("phase"),
                        },
                    )
                except Exception as exc:
                    self._json(500, {"success": False, "error": str(exc)})
                return
            if path in ("/api/nav/diagnostics", "/api/nav/state"):
                try:
                    snap = APP.snapshot()
                    nav = snap.get("nav") or {}
                    dbg = snap.get("debug") or {}
                    lp = dbg.get("local_planner") or {}
                    self._json(
                        200,
                        {
                            "success": True,
                            "nav": nav,
                            "safety": snap.get("safety"),
                            "recovery": dbg.get("execution_recovery") or dbg.get("recovery"),
                            "diagnostics": dbg.get("diagnostics"),
                            "status": dbg.get("status"),
                            "velocity_chain": dbg.get("velocity_chain"),
                            "mppi": {
                                "candidate_count": lp.get("candidate_count"),
                                "valid_candidate_count": lp.get("valid_candidate_count"),
                                "collision_rejected_count": lp.get("collision_rejected_count"),
                                "constraint_rejected_count": lp.get("constraint_rejected_count"),
                            },
                            "execution_corridor": (dbg.get("nav_policy") or {}).get("execution_corridor"),
                        },
                    )
                except Exception as exc:
                    self._json(500, {"success": False, "error": str(exc)})
                return
            # P0-B-0 observability APIs
            if path in ("/api/logs", "/api/nav/logs", "/api/logs/events", "/api/nav/logs/events"):
                if hasattr(APP.state, "get_nav_logs"):
                    query = {k: (v[0] if isinstance(v, list) and v else v) for k, v in qs.items()}
                    self._json(200, APP.state.get_nav_logs(query))
                else:
                    self._json(200, {"success": False, "message": "logs unsupported"})
                return
            if path in ("/api/logs/summary", "/api/nav/logs/summary"):
                self._json(200, APP.state.get_nav_logs_summary() if hasattr(APP.state, "get_nav_logs_summary") else {"success": False})
                return
            if path in ("/api/logs/diagnostics", "/api/nav/logs/diagnostics"):
                win = float(qs.get("window_s", ["10"])[0] or 10)
                self._json(
                    200,
                    APP.state.get_nav_logs_diagnostics(win)
                    if hasattr(APP.state, "get_nav_logs_diagnostics")
                    else {"success": False},
                )
                return
            if path in ("/api/logs/api", "/api/nav/logs/api"):
                lim = int(qs.get("limit", ["100"])[0] or 100)
                self._json(
                    200,
                    APP.state.get_nav_api_logs(lim) if hasattr(APP.state, "get_nav_api_logs") else {"success": False},
                )
                return
            if path.startswith("/api/logs/trace/") or path.startswith("/api/nav/logs/trace/"):
                tid = path.rsplit("/", 1)[-1]
                self._json(
                    200,
                    APP.state.get_nav_logs_trace(tid) if hasattr(APP.state, "get_nav_logs_trace") else {"success": False},
                )
                return
            if path.startswith("/api/logs/cycle/") or path.startswith("/api/nav/logs/cycle/"):
                cid = path.rsplit("/", 1)[-1]
                self._json(
                    200,
                    APP.state.get_nav_logs_cycle(cid) if hasattr(APP.state, "get_nav_logs_cycle") else {"success": False},
                )
                return
            return super().do_GET()

        def do_POST(self) -> None:  # noqa: N802
            self._begin_req("POST")
            path = urlparse(self.path).path
            body = self._read_json()
            assert APP is not None
            if path in ("/api/logs/config", "/api/nav/logs/config"):
                self._json(
                    200,
                    APP.state.configure_nav_logs(body) if hasattr(APP.state, "configure_nav_logs") else {"success": False},
                )
                return
            if path in ("/api/goal", "/api/navigate_xy", "/api/nav/plan"):
                if body.get("poi_id") or body.get("target_id"):
                    self._json(200, APP.navigate_poi(str(body.get("poi_id") or body.get("target_id"))))
                    return
                x = float(body.get("x", 0.0))
                y = float(body.get("y", 0.0))
                auto = bool(body.get("auto_start", False))
                self._json(200, APP.set_goal(x, y, auto_start=auto))
                return
            if path == "/api/nav/confirm" or path == "/api/confirm":
                self._json(200, APP.confirm())
                return
            if path in ("/api/nav/control_mode", "/api/control_mode"):
                mode = str(body.get("mode") or body.get("control_mode") or "mppi")
                if hasattr(APP.state, "set_control_mode"):
                    self._json(200, {"success": True, **APP.state.set_control_mode(mode)})
                else:
                    self._json(200, {"success": False, "message": "control_mode unsupported"})
                return
            if path in ("/api/nav/plan_quality", "/api/nav/planner_debug"):
                x = float(body.get("x", 0.0))
                y = float(body.get("y", 0.0))
                compare = bool(body.get("compare", True))
                if hasattr(APP.state, "plan_quality_debug"):
                    self._json(200, APP.state.plan_quality_debug(x, y, compare=compare))
                else:
                    self._json(200, {"success": False, "message": "plan_quality unsupported"})
                return
            if path in ("/api/nav/debug", "/api/debug/nav"):
                if hasattr(APP.state, "get_nav_debug"):
                    self._json(200, APP.state.get_nav_debug())
                else:
                    self._json(200, {"success": False, "message": "debug unsupported"})
                return
            if path in ("/api/nav/debug/level", "/api/debug/level"):
                level = str(body.get("level") or "BASIC")
                if hasattr(APP.state, "set_debug_level"):
                    self._json(200, APP.state.set_debug_level(level))
                else:
                    self._json(200, {"success": False})
                return
            if path in ("/api/nav/debug/freeze", "/api/debug/freeze"):
                fr = bool(body.get("freeze", body.get("paused", True)))
                if hasattr(APP.state, "set_debug_freeze"):
                    self._json(200, APP.state.set_debug_freeze(fr))
                else:
                    self._json(200, {"success": False})
                return
            if path in ("/api/nav/debug/capture", "/api/debug/capture"):
                if hasattr(APP.state, "capture_debug"):
                    self._json(200, APP.state.capture_debug())
                else:
                    self._json(200, {"success": False})
                return
            if path == "/api/navigate":
                if "x" in body and "y" in body:
                    self._json(200, APP.plan_goal(float(body["x"]), float(body["y"])))
                else:
                    tid = str(body.get("target_id") or body.get("id") or body.get("poi_id") or "")
                    self._json(200, APP.navigate_poi(tid) if tid else {"success": False, "message": "缺少点位"})
                return
            if path == "/api/scene" or path == "/api/scenes/set":
                self._json(200, APP.set_scene(str(body.get("id") or body.get("scene") or "indoor_office")))
                return
            if path == "/api/cmd_vel":
                self._json(200, APP.cmd_vel(float(body.get("vx", 0.0)), float(body.get("w", 0.0))))
                return
            if path == "/api/cancel":
                self._json(200, APP.cancel())
                return
            if path in ("/api/obstacles/add", "/api/obstacle/add"):
                r = APP.world.add_dyn_obstacle(
                    float(body.get("x", 0.0)),
                    float(body.get("y", 0.0)),
                    float(body.get("r", 0.4)),
                    str(body.get("name") or ""),
                    str(body.get("kind") or "box"),
                )
                self._json(200, r)
                return
            if path in ("/api/obstacles/remove", "/api/obstacle/remove"):
                self._json(200, APP.world.remove_dyn_obstacle(str(body.get("name") or "")))
                return
            if path == "/api/obstacles/clear":
                self._json(200, APP.world.clear_dyn_obstacles())
                return
            if path == "/api/scenario/clear":
                self._json(200, APP.world.clear_scenario())
                return
            if path == "/api/scenario/actors":
                enabled = bool(body.get("enabled", True))
                self._json(200, APP.world.set_actors_enabled(enabled))
                return
            if path == "/api/scenario/mover/add":
                self._json(
                    200,
                    APP.world.add_scenario_mover(
                        str(body.get("name") or ""),
                        float(body.get("x", 0.0)),
                        float(body.get("y", 0.0)),
                        float(body.get("r", 0.35)),
                        float(body.get("vx", 0.0)),
                        float(body.get("vy", 0.0)),
                        str(body.get("kind") or "dynamic"),
                    ),
                )
                return
            if path == "/api/scenario/setup":
                from agv_bridge.nav_scenario_injector import apply_scenario
                from agv_bridge.nav_live_client import NavLiveClient

                client = NavLiveClient(host=APP.host)
                self._json(200, apply_scenario(client, APP.world, str(body.get("scene") or body.get("id") or "")))
                return
            if path in ("/api/scenario/inject/ahead", "/api/scenario/inject-ahead"):
                from agv_bridge.nav_scenario_injector import M32_OPEN_GOAL, M32_OPEN_START, _inject_ahead_of_pose

                class _InjectClient:
                    def post(self, p: str, b: Optional[dict] = None) -> dict:
                        b = b or {}
                        if p == "/api/obstacles/add":
                            return APP.world.add_dyn_obstacle(b["x"], b["y"], b["r"], b["name"], b.get("kind", "box"))
                        if p == "/api/scenario/mover/add":
                            return APP.world.add_scenario_mover(
                                b["name"], b["x"], b["y"], b["r"], b["vx"], b["vy"], b.get("kind", "dynamic")
                            )
                        return {"success": False}

                with APP.state.lock:
                    px, py = float(APP.state.x), float(APP.state.y)
                    rev = int(getattr(APP.state, "_global_path_revision", 0) or 0)
                    gpath = list(getattr(APP.state, "_global_path", []) or [])
                ahead_m = float(body.get("ahead_m") or body.get("forward_m") or 2.5)
                kind = str(body.get("kind") or "static_left")
                prefix = str(body.get("prefix") or body.get("name") or "online_api")
                start = body.get("start") or M32_OPEN_START
                goal = body.get("goal") or M32_OPEN_GOAL
                ghash = global_path_fingerprint([{"x": p[0], "y": p[1]} for p in gpath] if gpath else [])
                _inject_ahead_of_pose(_InjectClient(), APP.world, start, goal, (px, py), ahead_m, kind, prefix)
                self._json(
                    200,
                    {
                        "success": True,
                        "event": "ONLINE_OBSTACLE_INJECT",
                        "ahead_m": ahead_m,
                        "kind": kind,
                        "prefix": prefix,
                        "vehicle_pose": {"x": px, "y": py},
                        "global_path_revision_before": rev,
                        "global_path_hash_before": ghash,
                    },
                )
                return
            if path == "/api/mock/control":
                self._json(200, APP.tcp.call(PORT_CONFIG, API_MOCK_CTRL, body))
                return
            if path in ("/api/pois/add", "/api/poi/add"):
                self._json(
                    200,
                    APP.world.add_poi(
                        str(body.get("id") or body.get("name") or ""),
                        float(body.get("x", 0.0)),
                        float(body.get("y", 0.0)),
                        str(body.get("kind") or "UserMark"),
                    ),
                )
                return
            if path in ("/api/pois/remove", "/api/poi/remove"):
                self._json(200, APP.world.remove_poi(str(body.get("id") or body.get("name") or "")))
                return
            if path == "/api/env":
                self._json(200, {"success": True, "env": APP.snapshot()["env"]})
                return
            if path == "/api/maps/robot":
                self._json(200, {"success": True, "message": "sim map ready"})
                return
            if path == "/api/route/status":
                snap = APP.snapshot()
                self._json(200, {"route_task": {}, "route": {"stations": []}, "nav": snap.get("nav")})
                return
            if path == "/api/agv/lock":
                self._json(200, {"success": True})
                return
            self._json(404, {"success": False, "message": f"unknown {path}"})

    return Handler


def main() -> int:
    global APP
    if not WWW.exists():
        print(f"WWW missing: {WWW}")
        return 1
    _ensure_single_sim_instance(WEB_SIM_PORT)
    APP = SimApp()
    handler = make_handler(WWW)
    srv = ThreadingHTTPServer(("0.0.0.0", WEB_SIM_PORT), handler)
    sc = APP.world.scene_info()
    print("=" * 60, flush=True)
    print(" AGV V0.1 TRUE SCENE Simulation READY", flush=True)
    print("  Web:     http://127.0.0.1:19999/", flush=True)
    print(f"  Build:   {APP.build_commit}  pid={APP.process_pid}", flush=True)
    print("  Engine:  smap occupancy + dual lidar + Three.js", flush=True)
    print(f"  Scene:   {sc['kind']} · {sc['name']} · cloud={sc['cloud_count']}", flush=True)
    print("  Control: API 3055/3056 · lidar 1009", flush=True)
    print("  Nav:     plan → onboard confirm → go", flush=True)
    print("=" * 60, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
