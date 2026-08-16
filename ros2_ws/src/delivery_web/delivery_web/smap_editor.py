"""Edit Roboshop .smap JSON — add/rename LocationMark stations."""

from __future__ import annotations

import copy
import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_STATION_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")
_CURVE_PROPS = [
    {"key": "direction", "type": "int", "value": "MA==", "int32Value": 0},
    {"key": "movestyle", "type": "int", "value": "MA==", "int32Value": 0},
]


def validate_station_name(name: str) -> str:
    name = (name or "").strip()
    if not name:
        raise ValueError("station name required")
    if not _STATION_NAME_RE.match(name):
        raise ValueError("station name: letters, digits, underscore, hyphen only")
    return name


def _find_station_index(points: List[Dict[str, Any]], station_id: str) -> Optional[int]:
    for i, pt in enumerate(points):
        if str(pt.get("instanceName") or "") == station_id:
            return i
    return None


def _pos_of(points: List[Dict[str, Any]], station_id: str) -> Optional[Dict[str, float]]:
    idx = _find_station_index(points, station_id)
    if idx is None:
        return None
    pos = (points[idx].get("pos") or {}) if isinstance(points[idx], dict) else {}
    return {"x": float(pos.get("x", 0.0) or 0.0), "y": float(pos.get("y", 0.0) or 0.0)}


def _bezier_pair(a: str, ax: float, ay: float, b: str, bx: float, by: float) -> List[Dict[str, Any]]:
    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
    c1 = {"x": (ax + mx) / 2.0, "y": (ay + my) / 2.0}
    c2 = {"x": (bx + mx) / 2.0, "y": (by + my) / 2.0}
    return [
        {
            "className": "DegenerateBezier",
            "instanceName": f"{a}-{b}",
            "startPos": {"instanceName": a, "pos": {"x": ax, "y": ay}},
            "endPos": {"instanceName": b, "pos": {"x": bx, "y": by}},
            "controlPos1": c1,
            "controlPos2": c2,
            "property": copy.deepcopy(_CURVE_PROPS),
        },
        {
            "className": "DegenerateBezier",
            "instanceName": f"{b}-{a}",
            "startPos": {"instanceName": b, "pos": {"x": bx, "y": by}},
            "endPos": {"instanceName": a, "pos": {"x": ax, "y": ay}},
            "controlPos1": c2,
            "controlPos2": c1,
            "property": copy.deepcopy(_CURVE_PROPS),
        },
    ]


def add_station(
    smap: Dict[str, Any],
    station_id: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    connect_from: str = "",
) -> Dict[str, Any]:
    station_id = validate_station_name(station_id)
    data = dict(smap)
    points = list(data.get("advancedPointList") or [])
    if _find_station_index(points, station_id) is not None:
        raise ValueError(f"station {station_id!r} already exists")
    points.append(
        {
            "className": "LocationMark",
            "instanceName": station_id,
            "pos": {"x": float(x), "y": float(y)},
            "dir": float(yaw),
            "ignoreDir": False,
            "needToPointRunning": False,
            "needToPointRotating": False,
            "property": [],
        }
    )
    data["advancedPointList"] = points
    src = (connect_from or "").strip()
    if src and src != station_id:
        spos = _pos_of(points, src)
        if spos is not None:
            curves = list(data.get("advancedCurveList") or [])
            curves.extend(_bezier_pair(src, spos["x"], spos["y"], station_id, float(x), float(y)))
            data["advancedCurveList"] = curves
    return data


def stations_already_linked(smap: Dict[str, Any], a: str, b: str) -> bool:
    pair = {str(a), str(b)}
    for curve in smap.get("advancedCurveList") or []:
        sp = str((curve.get("startPos") or {}).get("instanceName") or "")
        ep = str((curve.get("endPos") or {}).get("instanceName") or "")
        if {sp, ep} == pair:
            return True
    return False


def connect_stations(smap: Dict[str, Any], from_id: str, to_id: str) -> Dict[str, Any]:
    from_id = validate_station_name(from_id)
    to_id = validate_station_name(to_id)
    if from_id == to_id:
        raise ValueError("cannot connect a station to itself")
    data = dict(smap)
    points = list(data.get("advancedPointList") or [])
    pa = _pos_of(points, from_id)
    pb = _pos_of(points, to_id)
    if pa is None:
        raise ValueError(f"station {from_id!r} not found")
    if pb is None:
        raise ValueError(f"station {to_id!r} not found")
    if stations_already_linked(data, from_id, to_id):
        return data
    curves = list(data.get("advancedCurveList") or [])
    curves.extend(_bezier_pair(from_id, pa["x"], pa["y"], to_id, pb["x"], pb["y"]))
    data["advancedCurveList"] = curves
    return data


def rename_station(smap: Dict[str, Any], old_id: str, new_id: str) -> Dict[str, Any]:
    old_id = validate_station_name(old_id)
    new_id = validate_station_name(new_id)
    data = dict(smap)
    points = list(data.get("advancedPointList") or [])
    idx = _find_station_index(points, old_id)
    if idx is None:
        raise ValueError(f"station {old_id!r} not found")
    if _find_station_index(points, new_id) is not None:
        raise ValueError(f"station {new_id!r} already exists")
    points[idx]["instanceName"] = new_id
    for curve in data.get("advancedCurveList") or []:
        sp = curve.get("startPos") or {}
        ep = curve.get("endPos") or {}
        if str(sp.get("instanceName") or "") == old_id:
            sp["instanceName"] = new_id
        if str(ep.get("instanceName") or "") == old_id:
            ep["instanceName"] = new_id
    data["advancedPointList"] = points
    return data


def delete_station(smap: Dict[str, Any], station_id: str) -> Dict[str, Any]:
    station_id = validate_station_name(station_id)
    data = dict(smap)
    points = list(data.get("advancedPointList") or [])
    idx = _find_station_index(points, station_id)
    if idx is None:
        raise ValueError(f"station {station_id!r} not found")
    points.pop(idx)
    data["advancedPointList"] = points
    curves = []
    for curve in data.get("advancedCurveList") or []:
        sp = str((curve.get("startPos") or {}).get("instanceName") or "")
        ep = str((curve.get("endPos") or {}).get("instanceName") or "")
        if sp == station_id or ep == station_id:
            continue
        curves.append(curve)
    data["advancedCurveList"] = curves
    return data


def load_smap_file(path: Path | str) -> Dict[str, Any]:
    p = Path(path)
    return json.loads(p.read_text(encoding="utf-8-sig"))


def save_smap_file(path: Path | str, smap: Dict[str, Any]) -> None:
    p = Path(path)
    p.write_text(
        json.dumps(smap, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )


def stations_from_smap(smap: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
    out: Dict[str, Dict[str, float]] = {}
    for pt in smap.get("advancedPointList") or []:
        if not isinstance(pt, dict):
            continue
        name = str(pt.get("instanceName") or "").strip()
        if not name:
            continue
        pos = pt.get("pos") or {}
        out[name] = {
            "x": float(pos.get("x", 0.0) or 0.0),
            "y": float(pos.get("y", 0.0) or 0.0),
            "yaw": float(pt.get("dir", 0.0) or 0.0),
            "type": str(pt.get("className") or "LocationMark"),
        }
    return out
