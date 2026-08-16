"""Unified vision manager — real hardware only (no mock/sim fallback in demo)."""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

from rclpy.node import Node

from delivery_web.device_config import DeviceConfig
from delivery_web.vision.floor_qr_scanner import FloorQrScannerBridge
from delivery_web.vision.leo_face import LeoFaceBridge
from delivery_web.vision.mock_vision import MockVisionBridge
from delivery_web.vision.wrist_camera import WristCameraBridge


class VisionManager:
    def __init__(
        self,
        node: Node,
        lock: threading.Lock,
        encode_jpeg: Callable[..., Any],
        logger: Callable[..., None],
        get_mode: Callable[[], str],
        get_robot_pose: Callable[[], Optional[Dict[str, float]]],
        get_face_state: Callable[[], Dict[str, Any]],
        get_stations: Optional[Callable[[], Dict[str, Dict[str, float]]]] = None,
        get_use_sim_cameras: Optional[Callable[[], bool]] = None,
        get_gazebo_jpeg: Optional[Callable[[], Optional[bytes]]] = None,
        cfg: Optional[DeviceConfig] = None,
    ) -> None:
        self._cfg = cfg or DeviceConfig.load()
        self._get_robot_pose = get_robot_pose
        self._get_face_state = get_face_state
        self._get_stations = get_stations
        self._mock = MockVisionBridge(self._cfg)
        self._node = node
        self._lock = lock
        self._encode_jpeg = encode_jpeg
        self._log = logger
        self._floor: Optional[FloorQrScannerBridge] = None
        self._wrist: Optional[WristCameraBridge] = None
        self._leo = LeoFaceBridge(self._node, self._lock, self._cfg)

    def _ensure_real(self) -> None:
        if self._floor is None:
            self._floor = FloorQrScannerBridge(
                self._node,
                self._lock,
                self._cfg,
                get_robot_pose=self._get_robot_pose,
                get_stations=self._get_stations,
            )
        if self._wrist is None:
            self._wrist = WristCameraBridge(
                self._node,
                self._lock,
                self._encode_jpeg,
                self._log,
                self._cfg,
            )

    def floor_qr_status(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._floor is not None
        return self._floor.status()

    def floor_qr_localization(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._floor is not None
        return self._floor.localization()

    def floor_qr_trigger(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._floor is not None
        return self._floor.trigger_scan()

    def wrist_status(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._wrist is not None
        return self._wrist.status()

    def wrist_snapshot(self) -> Optional[bytes]:
        self._ensure_real()
        assert self._wrist is not None
        return self._wrist.latest_jpeg()

    def reset_for_mode_change(self) -> None:
        pass

    def warmup(self) -> None:
        self._ensure_real()

    def wrist_start_detection(self, mode: str = "qr") -> Dict[str, Any]:
        self._ensure_real()
        assert self._wrist is not None
        return self._wrist.start_detection(mode)

    def wrist_stop_detection(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._wrist is not None
        return self._wrist.stop_detection()

    def wrist_trigger_qr(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._wrist is not None
        return self._wrist.trigger_qr_burst(timeout_sec=5.0)

    def wrist_trigger_apriltag(self) -> Dict[str, Any]:
        self._ensure_real()
        assert self._wrist is not None
        return self._wrist.trigger_apriltag_burst(timeout_sec=5.0)

    def leo_trigger_face(self) -> Dict[str, Any]:
        return self._leo.trigger_face()

    def leo_set_continuous(self, enabled: bool) -> Dict[str, Any]:
        return self._leo.set_continuous(enabled)

    def leo_ensure_continuous(self) -> Dict[str, Any]:
        return self._leo.ensure_continuous()

    def leo_status(self) -> Dict[str, Any]:
        return self._leo.status(self._get_face_state())

    def mock_control(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._mock.apply_control(payload)

    def jason_status(self) -> Dict[str, Any]:
        s = self.wrist_status()
        s["camera_id"] = "wrist_camera"
        s["source"] = "REAL"
        s["model"] = "acA2500-14gc"
        s["device_user_id"] = "106611-18"
        s["topic"] = self._cfg.wrist_camera_image_topic
        dets = s.get("last_detections") or []
        last = dets[0]["detected_id"] if dets else "--"
        s["last_qr_result"] = last if s.get("qr_recognition") == "RUNNING" else "--"
        s["qr_result_updated_at"] = dets[0].get("timestamp", 0.0) if dets else 0.0
        return s

    def jason_snapshot(self) -> Optional[bytes]:
        return self.wrist_snapshot()

    def jason_start_qr(self) -> Dict[str, Any]:
        return self.wrist_start_detection("qr")

    def jason_stop_qr(self) -> Dict[str, Any]:
        return self.wrist_stop_detection()
