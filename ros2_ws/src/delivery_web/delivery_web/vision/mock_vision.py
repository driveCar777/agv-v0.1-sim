"""Mock vision devices for dev/mock mode without hardware."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from delivery_web.device_config import DeviceConfig
from delivery_web.localization import verify_localization
from delivery_web.vision.jpeg_util import mock_wrist_jpeg
from delivery_web.vision.models import DetectionResult, FloorQrScannerStatus, LeoFaceStatus, WristCameraStatus


class MockVisionState:
    def __init__(self) -> None:
        self.floor_qr_online = True
        self.floor_qr_id = "LM3"
        self.wrist_online = True
        self.wrist_detections: list[DetectionResult] = [
            DetectionResult("AprilTag", "TAG-12", 0.92, time.time()),
        ]
        self.leo_online = False
        self.leo_person_id = ""
        self.leo_person_name = ""
        self.leo_confidence = 0.0
        self._jpeg_cache = mock_wrist_jpeg("MOCK WRIST")

    def set_floor_qr(self, online: bool, qr_id: str = "floor_qr_001") -> None:
        self.floor_qr_online = online
        self.floor_qr_id = qr_id

    def set_wrist(self, online: bool, detections: Optional[list] = None) -> None:
        self.wrist_online = online
        if detections is not None:
            self.wrist_detections = detections

    def set_leo(
        self,
        online: bool,
        person_id: str = "MOCK-PERSON",
        person_name: str = "Mock User",
        confidence: float = 0.88,
    ) -> None:
        self.leo_online = online
        self.leo_person_id = person_id
        self.leo_person_name = person_name
        self.leo_confidence = confidence


_MOCK = MockVisionState()


def get_mock_vision_state() -> MockVisionState:
    return _MOCK


class MockVisionBridge:
    def __init__(self, cfg: Optional[DeviceConfig] = None) -> None:
        self._cfg = cfg or DeviceConfig.load()
        self._state = _MOCK

    def floor_qr_status(self) -> Dict[str, Any]:
        st = "ONLINE" if self._state.floor_qr_online else "OFFLINE"
        return FloorQrScannerStatus(
            host=self._cfg.floor_qr_scanner_host,
            protocol="keyence_tcp_mock",
            status=st,
            last_qr_id=self._state.floor_qr_id if st == "ONLINE" else "",
            last_scan_at=time.time() if st == "ONLINE" else 0.0,
        ).to_dict()

    def floor_qr_localization(
        self,
        robot_pose: Dict[str, float],
        *,
        stations: Optional[Dict[str, Dict[str, float]]] = None,
    ) -> Dict[str, Any]:
        st = "ONLINE" if self._state.floor_qr_online else "OFFLINE"
        qr = self._state.floor_qr_id if st == "ONLINE" else ""
        return verify_localization(
            qr,
            float(robot_pose.get("x", 0.0)),
            float(robot_pose.get("y", 0.0)),
            float(robot_pose.get("angle", 0.0)),
            scanner_status=st,
            thresholds=self._cfg.localization,
            stations=stations,
        ).to_dict()

    def wrist_status(self) -> Dict[str, Any]:
        st = "ONLINE" if self._state.wrist_online else "OFFLINE"
        dets = self._state.wrist_detections if st == "ONLINE" else []
        has_jpeg = st == "ONLINE" and bool(self._state._jpeg_cache)
        last_qr = self._last_det(dets, "QR")
        last_tag = self._last_det(dets, "AprilTag")
        return WristCameraStatus(
            host=self._cfg.wrist_camera_host,
            status=st if has_jpeg else ("IMAGE UNAVAILABLE" if st == "ONLINE" else "OFFLINE"),
            network_online=st == "ONLINE",
            ros_node_online=st == "ONLINE",
            image_available=has_jpeg,
            image_timestamp=time.time() if has_jpeg else 0.0,
            detection_available=bool(last_qr or last_tag),
            has_frame=has_jpeg,
            fps=15.0 if has_jpeg else 0.0,
            width=320 if has_jpeg else 0,
            height=200 if has_jpeg else 0,
            qr_recognition="RUNNING" if dets else "OFF",
            last_qr=last_qr,
            last_apriltag=last_tag,
            last_detections=dets,
        ).to_dict()

    @staticmethod
    def _last_det(dets, kind: str) -> str:
        for d in dets:
            if d.detected_type.upper() == kind.upper():
                return d.detected_id
        return ""

    def wrist_snapshot(self) -> Optional[bytes]:
        if not self._state.wrist_online:
            return None
        return self._state._jpeg_cache or mock_wrist_jpeg()

    def leo_status(self) -> Dict[str, Any]:
        online = self._state.leo_online
        return LeoFaceStatus(
            jetson_host=self._cfg.leo_jetson_host,
            camera_host=self._cfg.leo_camera_host,
            status="ONLINE" if online else "DEVELOPING",
            message="Mock face online" if online else "Face Recognition — Offline / Developing",
            connected=online,
            person_id=self._state.leo_person_id,
            person_name=self._state.leo_person_name,
            confidence=self._state.leo_confidence,
        ).to_dict()

    def apply_control(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        dev = str(payload.get("device") or "")
        online = bool(payload.get("online", True))
        if dev == "floor_qr_scanner":
            self._state.set_floor_qr(online, str(payload.get("qr_id") or "floor_qr_001"))
        elif dev == "wrist_camera":
            dets = []
            if payload.get("detected_id"):
                dets.append(
                    DetectionResult(
                        str(payload.get("detected_type") or "QR"),
                        str(payload.get("detected_id")),
                        float(payload.get("confidence") or 1.0),
                        time.time(),
                    )
                )
            self._state.set_wrist(online, dets or None)
        elif dev == "leo_face":
            self._state.set_leo(
                online,
                str(payload.get("person_id") or "MOCK-PERSON"),
                str(payload.get("person_name") or "Mock User"),
                float(payload.get("confidence") or 0.88),
            )
        else:
            return {"success": False, "message": f"unknown device {dev}"}
        return {"success": True, "device": dev, "online": online}
