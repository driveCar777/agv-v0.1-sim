"""Floor QR → localization verification."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional

from delivery_web.device_config import DeviceConfig, LocalizationThresholds, load_floor_qr_mapping


@dataclass
class LocalizationResult:
    qr_id: str = ""
    timestamp: float = 0.0
    scanner_status: str = "OFFLINE"
    robot_pose: Optional[Dict[str, float]] = None
    qr_pose: Optional[Dict[str, float]] = None
    station_id: str = ""
    position_error: Optional[float] = None
    angle_error: Optional[float] = None
    localization_ok: Optional[bool] = None
    verdict: str = "UNKNOWN"  # MATCH | MISMATCH | UNKNOWN
    level: str = "UNKNOWN"  # legacy: OK | WARNING | ERROR | BLOCKED | UNKNOWN
    message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _resolve_qr_pose(
    qr_id: str,
    mapping: Dict[str, Dict[str, Any]],
    stations: Optional[Dict[str, Dict[str, float]]],
) -> Optional[tuple[float, float, float, str]]:
    """Resolve expected pose: (x, y, angle_rad, station_id)."""
    qr_id = (qr_id or "").strip()
    if not qr_id:
        return None

    if stations and qr_id in stations:
        s = stations[qr_id]
        return (
            float(s.get("x", 0.0)),
            float(s.get("y", 0.0)),
            float(s.get("yaw", s.get("r", s.get("angle", 0.0)))),
            qr_id,
        )

    entry = mapping.get(qr_id)
    if not entry:
        for _key, val in mapping.items():
            if isinstance(val, dict) and str(val.get("station_id") or "") == qr_id:
                entry = val
                break
    if not entry:
        return None

    station_id = str(entry.get("station_id") or qr_id)
    if stations and station_id in stations:
        s = stations[station_id]
        return (
            float(s.get("x", 0.0)),
            float(s.get("y", 0.0)),
            float(s.get("yaw", s.get("r", s.get("angle", 0.0)))),
            station_id,
        )
    return (
        float(entry.get("x", 0.0)),
        float(entry.get("y", 0.0)),
        float(entry.get("angle", entry.get("r", 0.0))),
        station_id,
    )


def verify_localization(
    qr_id: str,
    robot_x: float,
    robot_y: float,
    robot_angle: float,
    *,
    scanner_status: str = "ONLINE",
    thresholds: Optional[LocalizationThresholds] = None,
    mapping: Optional[Dict[str, Dict[str, Any]]] = None,
    stations: Optional[Dict[str, Dict[str, float]]] = None,
) -> LocalizationResult:
    now = time.time()
    qr_id = (qr_id or "").strip()
    robot_pose = {"x": robot_x, "y": robot_y, "angle": robot_angle}
    if not qr_id:
        return LocalizationResult(
            timestamp=now,
            scanner_status=scanner_status,
            robot_pose=robot_pose,
            verdict="UNKNOWN",
            level="UNKNOWN",
            message="waiting for QR",
        )

    mapping = mapping if mapping is not None else load_floor_qr_mapping()
    resolved = _resolve_qr_pose(qr_id, mapping, stations)
    if not resolved:
        return LocalizationResult(
            qr_id=qr_id,
            timestamp=now,
            scanner_status=scanner_status,
            robot_pose=robot_pose,
            verdict="UNKNOWN",
            level="BLOCKED",
            message="QR not mapped to station pose",
        )

    qx, qy, qa, station_id = resolved
    th = thresholds or DeviceConfig.load().localization
    pos_err = math.hypot(robot_x - qx, robot_y - qy)
    ang_err = abs(math.degrees(_norm_angle(robot_angle - qa)))

    if pos_err >= th.position_error_error_m or ang_err >= th.angle_error_error_deg:
        level = "ERROR"
        ok = False
        verdict = "MISMATCH"
    elif pos_err >= th.position_error_warn_m or ang_err >= th.angle_error_warn_deg:
        level = "WARNING"
        ok = False
        verdict = "MISMATCH"
    else:
        level = "OK"
        ok = True
        verdict = "MATCH"

    return LocalizationResult(
        qr_id=qr_id,
        timestamp=now,
        scanner_status=scanner_status,
        robot_pose=robot_pose,
        qr_pose={"x": qx, "y": qy, "angle": qa},
        station_id=station_id,
        position_error=round(pos_err, 4),
        angle_error=round(ang_err, 3),
        localization_ok=ok,
        verdict=verdict,
        level=level,
        message=f"localization {verdict.lower()}",
    )
