"""Vision device DTOs."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class DetectionResult:
    detected_type: str = ""
    detected_id: str = ""
    confidence: float = 0.0
    timestamp: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class FloorQrScannerStatus:
    device: str = "floor_qr_scanner"
    host: str = ""
    protocol: str = "keyence_tcp"
    status: str = "OFFLINE"
    last_qr_id: str = ""
    last_scan_at: float = 0.0
    ros_topic: str = "/scanner/barcode"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class WristCameraStatus:
    device: str = "wrist_camera"
    host: str = ""
    status: str = "OFFLINE"
    network_online: bool = False
    ros_node_online: bool = False
    image_available: bool = False
    image_timestamp: float = 0.0
    detection_available: bool = False
    has_frame: bool = False
    fps: float = 0.0
    width: int = 0
    height: int = 0
    encoding: str = ""
    last_frame_ms_ago: Optional[int] = None
    qr_recognition: str = "OFF"
    last_qr: str = ""
    last_apriltag: str = ""
    last_detections: List[DetectionResult] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["last_detections"] = [x.to_dict() for x in self.last_detections]
        return d


@dataclass
class LeoFaceStatus:
    device: str = "leo_face"
    jetson_host: str = ""
    camera_host: str = ""
    status: str = "DEVELOPING"
    message: str = "Face Recognition — Offline / Developing"
    connected: bool = False
    person_id: str = ""
    person_name: str = ""
    confidence: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
