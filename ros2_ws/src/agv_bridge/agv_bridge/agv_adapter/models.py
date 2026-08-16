"""AGV API data transfer objects — shared by Real / Dev / Mock adapters."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    angle: float = 0.0
    confidence: float = 0.0
    current_station: str = ""
    last_station: str = ""
    vehicle_id: str = ""

    @classmethod
    def from_robokit(cls, raw: Dict[str, Any]) -> "RobotPose":
        return cls(
            x=float(raw.get("x", 0.0) or 0.0),
            y=float(raw.get("y", 0.0) or 0.0),
            angle=float(raw.get("angle", 0.0) or 0.0),
            confidence=float(raw.get("confidence", 0.0) or 0.0),
            current_station=str(raw.get("current_station", "") or ""),
            last_station=str(raw.get("last_station", "") or ""),
            vehicle_id=str(raw.get("vehicle_id", "") or ""),
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class LaserPoint:
    x: float
    y: float

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y}


@dataclass
class LaserScan:
    ok: bool = False
    points: List[LaserPoint] = field(default_factory=list)
    beam_count: int = 0
    source: str = "none"
    message: str = ""

    def points_as_dicts(self) -> List[Dict[str, float]]:
        return [p.to_dict() for p in self.points]


@dataclass
class NavigationStatus:
    task_status: int = 0
    task_type: int = 0
    target_id: str = ""
    finished_path: List[str] = field(default_factory=list)
    unfinished_path: List[str] = field(default_factory=list)

    @classmethod
    def from_robokit(cls, raw: Dict[str, Any]) -> "NavigationStatus":
        return cls(
            task_status=int(raw.get("task_status", 0) or 0),
            task_type=int(raw.get("task_type", 0) or 0),
            target_id=str(raw.get("target_id", "") or ""),
            finished_path=list(raw.get("finished_path") or []),
            unfinished_path=list(raw.get("unfinished_path") or []),
        )


@dataclass
class MapInfo:
    name: str
    path: str = ""
    is_current: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class Station:
    id: str
    x: float
    y: float
    name: str = ""
    type: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class RobotState:
    """Aggregated robot snapshot for dashboard polling."""

    pose: Optional[RobotPose] = None
    navigation: Optional[NavigationStatus] = None
    laser: Optional[LaserScan] = None
    battery_level: float = 0.0
    charging: bool = False
    emergency: bool = False
    soft_emc: bool = False
    blocked: bool = False
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    connected: bool = False
    host: str = ""
    mode: str = ""
    # Diagnostic fields: None = unknown (never coerce API failure to 0/False).
    r_vx: Optional[float] = None
    r_vy: Optional[float] = None
    r_w: Optional[float] = None
    is_stop: Optional[bool] = None
    dispatch_mode: Optional[int] = None
    connect_fleet: Optional[bool] = None
    current_lock: Optional[Dict[str, Any]] = None
    block_reason: Optional[int] = None
    brake: Optional[bool] = None
    driver_emc: Optional[bool] = None
    manual_charge: Optional[bool] = None
    motor_info: Optional[List[Dict[str, Any]]] = None
    errors: Optional[List[Any]] = None
    fatals: Optional[List[Any]] = None
    warnings: Optional[List[Any]] = None
    current_map: Optional[str] = None
    vehicle_id: Optional[str] = None
    move_status_info: Optional[str] = None
    diag_source: Optional[str] = None
    reloc_status: Optional[int] = None
    loadmap_status: Optional[int] = None

    def agv_state_dict(self, prev: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Merge into dashboard ``agv`` dict shape."""
        prev = prev or {}
        pose = self.pose or RobotPose()
        nav = self.navigation or NavigationStatus()
        return {
            **prev,
            "x": pose.x,
            "y": pose.y,
            "angle": pose.angle,
            "confidence": pose.confidence,
            "current_station": pose.current_station,
            "last_station": pose.last_station or str(prev.get("last_station", "") or ""),
            "mode": "real" if self.mode in ("demo", "mock") else "sim",
            "task_status": nav.task_status if self.navigation else int(prev.get("task_status", 0) or 0),
            "task_type": nav.task_type if self.navigation else int(prev.get("task_type", 0) or 0),
            "target_id": nav.target_id if self.navigation else str(prev.get("target_id", "") or ""),
            "finished_path": nav.finished_path if self.navigation else list(prev.get("finished_path") or []),
            "unfinished_path": nav.unfinished_path if self.navigation else list(prev.get("unfinished_path") or []),
            "battery_level": float(self.battery_level) if self.connected else float(prev.get("battery_level", 0.0) or 0.0),
            "battery": int(round((float(self.battery_level) if self.connected else float(prev.get("battery_level", 0.0) or 0.0)) * 100.0)),
            "charging": bool(self.charging) if self.connected else bool(prev.get("charging", False)),
            "emergency": bool(self.emergency) if self.connected else bool(prev.get("emergency", False)),
            "soft_emc": bool(self.soft_emc) if self.connected else bool(prev.get("soft_emc", False)),
            "blocked": bool(self.blocked) if self.connected else bool(prev.get("blocked", False)),
            "vx": float(self.vx),
            "vy": float(self.vy),
            "w": float(self.w),
            "speed": abs(float(self.vx)),
            "yaw_deg": float(pose.angle) * 180.0 / 3.141592653589793,
            "r_vx": self.r_vx,
            "r_vy": self.r_vy,
            "r_w": self.r_w,
            "is_stop": self.is_stop,
            "dispatch_mode": self.dispatch_mode,
            "connect_fleet": self.connect_fleet,
            "current_lock": self.current_lock,
            "block_reason": self.block_reason,
            "brake": self.brake,
            "driver_emc": self.driver_emc,
            "manual_charge": self.manual_charge,
            "motor_info": self.motor_info,
            "errors": self.errors,
            "fatals": self.fatals,
            "warnings": self.warnings,
            "current_map": self.current_map,
            "vehicle_id": pose.vehicle_id or self.vehicle_id or str(prev.get("vehicle_id") or "REAL-AGV"),
            "move_status_info": self.move_status_info,
            "diag_source": self.diag_source,
            "reloc_status": self.reloc_status,
            "loadmap_status": self.loadmap_status,
        }

    def laser_state_dict(self) -> Dict[str, Any]:
        laser = self.laser or LaserScan(message="no data")
        return {
            "ok": laser.ok,
            "source": laser.source,
            "live_lidar": str(laser.source or "").startswith("robokit"),
            "points": laser.points_as_dicts(),
            "beam_count": laser.beam_count,
            "message": laser.message,
            "updated_at": 0.0,
            "agv_host": self.host,
        }
