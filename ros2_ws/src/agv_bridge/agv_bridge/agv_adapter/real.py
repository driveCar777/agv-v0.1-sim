"""Real vehicle adapter — wraps RobokitClient TCP API."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from agv_bridge.agv_adapter.base import AgvApiAdapter
from agv_bridge.agv_adapter.config import AgvAdapterConfig
from agv_bridge.agv_adapter.laser_utils import laser_scan_from_robokit
from agv_bridge.agv_adapter.models import LaserScan, MapInfo, NavigationStatus, RobotPose, RobotState, Station

try:
    from agv_bridge.robokit_client import RobokitClient
except Exception:  # noqa: BLE001
    RobokitClient = None  # type: ignore


class RealAdapter(AgvApiAdapter):
    """Demo mode: live Robokit TCP to real AGV."""

    def __init__(self, config: AgvAdapterConfig):
        super().__init__(config)
        self._config = config
        self._client: Optional[Any] = None
        self._push_cache: Optional[Any] = None
        self._optional_fail: set = set()

    def set_push_cache(self, cache: Any) -> None:
        self._push_cache = cache

    def configure_push(self, interval_ms: int = 200, **kwargs: Any) -> Dict[str, Any]:
        return self._require_client().configure_push(interval_ms=interval_ms, **kwargs)

    @property
    def mode(self) -> str:
        return "demo"

    @property
    def client(self) -> Optional[Any]:
        return self._client

    def connect(self, host: Optional[str] = None) -> bool:
        if RobokitClient is None:
            self._connected = False
            self._client = None
            return False
        target = (host or self._config.agv_host or "").strip()
        if not target:
            return False
        try:
            if self._client is not None and getattr(self._client, "host", None) == target:
                self._host = target
                self._connected = True
                return True
            self.disconnect()
            self._client = RobokitClient(
                host=target,
                timeout=self._config.timeout,
                protocol_version=self._config.protocol_version,
            )
            self._host = target
            try:
                self._client.get_pose()
            except Exception:  # noqa: BLE001
                self.disconnect()
                return False
            self._connected = True
            return True
        except Exception:  # noqa: BLE001
            self._client = None
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass
        self._client = None
        self._connected = False

    def _require_client(self) -> Any:
        if self._client is None:
            raise RuntimeError("Robokit client not connected")
        return self._client

    def get_pose(self) -> RobotPose:
        if self._push_cache is not None:
            cached = self._push_cache.get_pose()
            st = self._push_cache.status()
            age = st.get("last_push_age_sec")
            if cached is not None and age is not None and float(age) < 3.0:
                return cached
        raw = self._require_client().get_pose()
        pose = RobotPose.from_robokit(raw)
        if self._push_cache is not None:
            self._push_cache.update_from_poll(pose)
        return pose

    def get_laser(self) -> LaserScan:
        client = self._require_client()
        pose = self.get_pose()
        raw = client.get_laser(return_beams3d=False)
        return laser_scan_from_robokit(pose, raw, max_points=self._config.laser_max_points)

    def get_task_status(self) -> NavigationStatus:
        try:
            raw = self._require_client().get_task_status(simple=False)
            return NavigationStatus.from_robokit(raw)
        except Exception:  # noqa: BLE001
            return NavigationStatus()

    def get_battery(self) -> Dict[str, Any]:
        try:
            return self._require_client().get_battery()
        except Exception:  # noqa: BLE001
            return {}

    def get_emergency(self) -> Dict[str, Any]:
        if 1012 in self._optional_fail:
            return {}
        try:
            return self._require_client().get_emergency()
        except Exception:  # noqa: BLE001
            self._optional_fail.add(1012)
            return {}

    def get_block_status(self) -> Dict[str, Any]:
        if 1006 in self._optional_fail:
            return {}
        try:
            return self._require_client().get_block_status()
        except Exception:  # noqa: BLE001
            self._optional_fail.add(1006)
            return {}

    def get_speed(self) -> Dict[str, Any]:
        if 1005 in self._optional_fail:
            return {}
        try:
            return self._require_client().get_speed()
        except Exception:  # noqa: BLE001
            self._optional_fail.add(1005)
            return {}

    def poll_status(self) -> RobotState:
        """Pose + nav + battery. Optional 1005/1006/1012 at most every 3s."""
        pose = self.get_pose()
        nav = self.get_task_status()
        bat = self.get_battery()
        now = __import__("time").time()
        last_opt = float(getattr(self, "_last_opt_poll", 0.0) or 0.0)
        emc: Dict[str, Any] = {}
        blk: Dict[str, Any] = {}
        spd: Dict[str, Any] = {}
        if now - last_opt >= 3.0:
            self._last_opt_poll = now
            emc = self.get_emergency()
            blk = self.get_block_status()
            spd = self.get_speed()
            self._opt_cache = {"emc": emc, "blk": blk, "spd": spd}
        else:
            cache = getattr(self, "_opt_cache", None) or {}
            emc = cache.get("emc") or {}
            blk = cache.get("blk") or {}
            spd = cache.get("spd") or {}
        return RobotState(
            pose=pose,
            navigation=nav,
            laser=None,
            battery_level=float(bat.get("battery_level", 0.0) or 0.0),
            charging=bool(bat.get("charging", False) or bat.get("charging_status", False)),
            emergency=bool(emc.get("emergency", False) or emc.get("emc", False)),
            soft_emc=bool(emc.get("soft_emc", False) or emc.get("softEmc", False)),
            blocked=bool(blk.get("blocked", False) or blk.get("blocked_status", False)),
            vx=float(spd.get("vx", 0.0) or 0.0),
            vy=float(spd.get("vy", 0.0) or 0.0),
            w=float(spd.get("w", 0.0) or spd.get("omega", 0.0) or 0.0),
            connected=self.connected,
            host=self.host,
            mode=self.mode,
            r_vx=_opt_float(spd.get("r_vx")) if spd else None,
            r_vy=_opt_float(spd.get("r_vy")) if spd else None,
            r_w=_opt_float(spd.get("r_w")) if spd else None,
            is_stop=_opt_bool(spd.get("is_stop")) if spd else None,
        )

    def poll_robot_state(self) -> RobotState:
        rs = self.poll_status()
        try:
            rs.laser = self.get_laser()
        except Exception:  # noqa: BLE001
            rs.laser = None
        return rs

    def goto_station(self, target_id: str, **kwargs: Any) -> Dict[str, Any]:
        return self._require_client().goto_station(target_id=target_id, **kwargs)

    def cancel_navigation(self) -> Dict[str, Any]:
        return self._require_client().cancel_nav()

    def pause_navigation(self) -> Dict[str, Any]:
        return self._require_client().pause_nav()

    def resume_navigation(self) -> Dict[str, Any]:
        return self._require_client().resume_nav()

    def get_reloc_status(self) -> Dict[str, Any]:
        return self._require_client().get_reloc_status()

    def wait_navigation_done(
        self,
        timeout_sec: float = 120.0,
        poll_sec: float = 0.3,
    ) -> Tuple[bool, NavigationStatus]:
        ok, raw = self._require_client().wait_nav_done(timeout_sec=timeout_sec, poll_sec=poll_sec)
        return ok, NavigationStatus.from_robokit(raw)

    def lock(self, nick_name: Optional[str] = None) -> Dict[str, Any]:
        return self._require_client().lock(nick_name or self._config.lock_nickname)

    def unlock(self) -> Dict[str, Any]:
        return self._require_client().unlock()

    def relocate(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        angle: Optional[float] = None,
        length: Optional[float] = None,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        return self._require_client().relocate(x=x, y=y, angle=angle, length=length, **kwargs)

    def wait_reloc_done(
        self,
        timeout_sec: float = 25.0,
        poll_sec: float = 0.4,
        auto_confirm_if_needed: bool = True,
    ) -> Tuple[int, Dict[str, Any]]:
        return self._require_client().wait_reloc_done(
            timeout_sec=timeout_sec,
            poll_sec=poll_sec,
            auto_confirm_if_needed=auto_confirm_if_needed,
        )

    def get_maps(self) -> List[MapInfo]:
        raw = self._require_client().get_map_info()
        current = str(raw.get("current_map") or "")
        maps = raw.get("maps") or raw.get("map_files_info") or []
        out: List[MapInfo] = []
        if isinstance(maps, list):
            for m in maps:
                if isinstance(m, str):
                    out.append(MapInfo(name=m, is_current=(m == current)))
                elif isinstance(m, dict):
                    name = str(m.get("name") or m.get("map_name") or "")
                    if name:
                        out.append(MapInfo(name=name, is_current=(name == current)))
        if current and not any(x.name == current for x in out):
            out.insert(0, MapInfo(name=current, is_current=True))
        return out

    def get_current_map(self) -> Optional[MapInfo]:
        maps = self.get_maps()
        for m in maps:
            if m.is_current:
                return m
        return maps[0] if maps else None

    def switch_map(self, map_name: str) -> Dict[str, Any]:
        return self._require_client().switch_map(map_name)

    def download_map_json(self, map_name: str) -> Dict[str, Any]:
        return self._require_client().download_map(map_name)

    def upload_map_json(self, smap: Dict[str, Any]) -> Dict[str, Any]:
        return self._require_client().upload_map(smap)

    def upload_and_switch_map_json(self, smap: Dict[str, Any]) -> Dict[str, Any]:
        return self._require_client().upload_and_switch_map(smap)

    def get_stations(self) -> List[Station]:
        raw = self._require_client().get_station_list()
        out: List[Station] = []
        for s in raw.get("stations") or []:
            if not isinstance(s, dict):
                continue
            out.append(
                Station(
                    id=str(s.get("id") or ""),
                    x=float(s.get("x", 0.0) or 0.0),
                    y=float(s.get("y", 0.0) or 0.0),
                    name=str(s.get("desc") or s.get("id") or ""),
                    type=str(s.get("type") or ""),
                )
            )
        return out

    def get_batch_status(self) -> Dict[str, Any]:
        """1100 — batch status. Does not use _optional_fail."""
        return self._require_client().get_batch_status()

    def get_control_owner(self) -> Dict[str, Any]:
        """1060 — lock owner as a full dict."""
        return self._require_client().get_control_owner()

    def get_map_load_status(self) -> Dict[str, Any]:
        """1022 — map load status."""
        return self._require_client().get_map_load_status()

    def get_path_info(self, source_id: str, target_id: str) -> Dict[str, Any]:
        """1303 — topology query (read-only)."""
        return self._require_client().get_path_info(source_id, target_id)

    def diagnose(self) -> RobotState:
        """On-demand diagnostic snapshot. Not used by 1Hz poll_status()."""
        batch: Dict[str, Any] = {}
        diag_source = "none"
        try:
            raw = self._require_client().get_batch_status()
            if raw and int(raw.get("ret_code", -1) or -1) == 0:
                batch = dict(raw)
                diag_source = "1100"
        except Exception:  # noqa: BLE001
            batch = {}

        if diag_source != "1100":
            diag_source = "fallback"
            try:
                spd = self.get_speed()
                emg = self.get_emergency()
                blk = self.get_block_status()
                if isinstance(spd, dict):
                    batch.update(spd)
                if isinstance(emg, dict):
                    batch.update(emg)
                if isinstance(blk, dict):
                    batch.update(blk)
            except Exception:  # noqa: BLE001
                pass

        current_lock = _lock_dict(batch.get("current_lock"))
        if current_lock is None:
            try:
                current_lock = _lock_dict(self._require_client().get_control_owner())
            except Exception:  # noqa: BLE001
                current_lock = None

        base = self.poll_status()
        if "vx" in batch:
            base.vx = float(batch.get("vx") or 0.0)
        if "vy" in batch:
            base.vy = float(batch.get("vy") or 0.0)
        if "w" in batch or "omega" in batch:
            base.w = float(batch.get("w") or batch.get("omega") or 0.0)
        if "r_vx" in batch:
            base.r_vx = _opt_float(batch.get("r_vx"))
        if "r_vy" in batch:
            base.r_vy = _opt_float(batch.get("r_vy"))
        if "r_w" in batch:
            base.r_w = _opt_float(batch.get("r_w"))
        if "is_stop" in batch:
            base.is_stop = _opt_bool(batch.get("is_stop"))
        base.dispatch_mode = _opt_int(batch.get("dispatch_mode"))
        fleet_key = "connectFleet" if "connectFleet" in batch else "connect_fleet"
        base.connect_fleet = _opt_bool(batch.get(fleet_key)) if fleet_key in batch else None
        base.current_lock = current_lock
        base.block_reason = _opt_int(batch.get("block_reason"))
        base.brake = _opt_bool(batch.get("brake")) if "brake" in batch else None
        base.driver_emc = _opt_bool(batch.get("driver_emc")) if "driver_emc" in batch else None
        base.manual_charge = _opt_bool(batch.get("manual_charge")) if "manual_charge" in batch else None
        motor = batch.get("motor_info")
        base.motor_info = motor if isinstance(motor, list) else None
        base.errors = batch.get("errors") if isinstance(batch.get("errors"), list) else None
        base.fatals = batch.get("fatals") if isinstance(batch.get("fatals"), list) else None
        base.warnings = batch.get("warnings") if isinstance(batch.get("warnings"), list) else None
        cmap = batch.get("current_map")
        base.current_map = str(cmap) if cmap else None
        vid = batch.get("vehicle_id")
        base.vehicle_id = str(vid) if vid else (base.pose.vehicle_id if base.pose else None)
        msi = batch.get("move_status_info")
        base.move_status_info = str(msi) if msi else None
        base.diag_source = diag_source
        base.reloc_status = _opt_int(batch.get("reloc_status"))
        base.loadmap_status = _opt_int(batch.get("loadmap_status"))
        if "emergency" in batch or "emc" in batch:
            base.emergency = bool(batch.get("emergency") or batch.get("emc"))
        if "soft_emc" in batch or "softEmc" in batch:
            base.soft_emc = bool(batch.get("soft_emc") or batch.get("softEmc"))
        return base


def _opt_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _opt_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _opt_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "1", "yes"):
            return True
        if s in ("false", "0", "no", ""):
            return False
    return bool(v)


def _lock_dict(raw: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(raw, dict):
        return None
    nested = raw.get("current_lock")
    if isinstance(nested, dict):
        return {k: v for k, v in nested.items() if not str(k).startswith("_")}
    keys = ("locked", "ip", "port", "type", "nick_name", "time_t", "desc")
    if any(k in raw for k in keys):
        return {
            k: v
            for k, v in raw.items()
            if not str(k).startswith("_") and k not in ("ret_code", "err_msg")
        }
    return None


