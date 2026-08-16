"""Load Phase C device configuration from YAML."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore


def _config_paths() -> list[Path]:
    here = Path(__file__).resolve()
    candidates = [
        Path(os.environ.get("DELIVERY_DEVICES_YAML", "")),
        here.parents[1] / "config" / "devices.yaml",
        Path("/opt/delivery_ws/src/delivery_web/config/devices.yaml"),
    ]
    return [p for p in candidates if p and str(p) and p.is_file()]


def _load_yaml(path: Path) -> Dict[str, Any]:
    if yaml is None:
        return {}
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data if isinstance(data, dict) else {}


@dataclass
class LocalizationThresholds:
    position_error_warn_m: float = 0.15
    position_error_error_m: float = 0.35
    angle_error_warn_deg: float = 5.0
    angle_error_error_deg: float = 12.0


@dataclass
class DeviceConfig:
    raw: Dict[str, Any] = field(default_factory=dict)
    floor_qr_scanner_host: str = "172.31.0.91"
    floor_qr_scanner_port: int = 9004
    floor_qr_ros_topic: str = "/scanner/barcode"
    floor_qr_trigger_service: str = "/scanner/trigger"
    wrist_camera_host: str = "172.31.0.88"
    wrist_camera_image_topic: str = "/my_camera/pylon_ros2_camera_node/image_raw"
    wrist_camera_image_topic_candidates: list[str] = field(default_factory=list)
    wrist_camera_compressed_topic: str = ""
    wrist_camera_qr_topic: str = "/wechat_qr_node/decoded_info"
    wrist_camera_apriltag_topic: str = "/detections"
    leo_jetson_host: str = "172.31.0.85"
    leo_camera_host: str = "172.31.0.87"
    leo_face_camera_name: str = "cam_front"
    leo_face_result_topic: str = "/face/cam_front/recognition_result"
    leo_face_continuous_service: str = "/face/cam_front/continuous_recognition"
    leo_face_capture_service: str = "/face/cam_front/capture_and_recognize"
    ros_domain_id: int = 30
    agv_robokit_host: str = "192.168.18.198"
    xarm_robot_ip: str = "172.31.0.123"
    xarm_model: str = "UFACTORY-xArm7"
    xarm_default_mode: str = "simulation"
    localization: LocalizationThresholds = field(default_factory=LocalizationThresholds)

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "DeviceConfig":
        data: Dict[str, Any] = {}
        if path and path.is_file():
            data = _load_yaml(path)
        else:
            for p in _config_paths():
                data = _load_yaml(p)
                if data:
                    break
        fq = data.get("floor_qr_scanner") or {}
        wc = data.get("wrist_camera") or {}
        leo = data.get("leo_face") or {}
        agv = data.get("agv") or {}
        xarm = data.get("xarm7") or {}
        loc = data.get("localization") or {}
        ros_cfg = data.get("ros") or {}
        domain_id = int(ros_cfg.get("domain_id", leo.get("ros_domain_id", 30)))
        return cls(
            raw=data,
            floor_qr_scanner_host=str(fq.get("host", "172.31.0.91")),
            floor_qr_scanner_port=int(fq.get("port", 9004)),
            floor_qr_ros_topic=str(fq.get("ros_topic", "/scanner/barcode")),
            floor_qr_trigger_service=str(fq.get("ros_trigger_service", "/scanner/trigger")),
            wrist_camera_host=str(wc.get("host", "172.31.0.88")),
            wrist_camera_image_topic=str(wc.get("ros_image_topic", "/my_camera/pylon_ros2_camera_node/image_raw")),
            wrist_camera_image_topic_candidates=[
                str(x) for x in (wc.get("ros_image_topic_candidates") or []) if str(x).strip()
            ],
            wrist_camera_compressed_topic=str(wc.get("ros_compressed_topic", "") or ""),
            wrist_camera_qr_topic=str(wc.get("ros_qr_topic", "/wechat_qr_node/decoded_info")),
            wrist_camera_apriltag_topic=str(wc.get("ros_apriltag_topic", "/detections")),
            leo_jetson_host=str(leo.get("jetson_host", "172.31.0.85")),
            leo_camera_host=str(leo.get("camera_host", "172.31.0.87")),
            leo_face_camera_name=str(leo.get("camera_name", "cam_front")),
            leo_face_result_topic=str(
                leo.get("ros_result_topic", "/face/cam_front/recognition_result")
            ),
            leo_face_continuous_service=str(
                leo.get("ros_continuous_service", "/face/cam_front/continuous_recognition")
            ),
            leo_face_capture_service=str(
                leo.get("ros_capture_service", "/face/cam_front/capture_and_recognize")
            ),
            ros_domain_id=domain_id,
            agv_robokit_host=str(agv.get("robokit_host", "192.168.18.198")),
            xarm_robot_ip=str(xarm.get("host", "172.31.0.123")),
            xarm_model=str(xarm.get("model", "UFACTORY-xArm7")),
            xarm_default_mode=str(xarm.get("default_arm_mode", "simulation")),
            localization=LocalizationThresholds(
                position_error_warn_m=float(loc.get("position_error_warn_m", 0.15)),
                position_error_error_m=float(loc.get("position_error_error_m", 0.35)),
                angle_error_warn_deg=float(loc.get("angle_error_warn_deg", 5.0)),
                angle_error_error_deg=float(loc.get("angle_error_error_deg", 12.0)),
            ),
        )

    def public_dict(self) -> Dict[str, Any]:
        """Safe device metadata for Web (no secrets)."""
        return {
            "floor_qr_scanner": {
                "host": self.floor_qr_scanner_host,
                "port": self.floor_qr_scanner_port,
                "protocol": "Keyence TCP",
                "ros_topic": self.floor_qr_ros_topic,
            },
            "wrist_camera": {
                "host": self.wrist_camera_host,
                "protocol": "Pylon ROS2",
                "ros_image_topic": self.wrist_camera_image_topic,
            },
            "leo_face": {
                "jetson_host": self.leo_jetson_host,
                "camera_host": self.leo_camera_host,
                "camera_name": self.leo_face_camera_name,
                "protocol": "face_executor",
                "ros_result_topic": self.leo_face_result_topic,
            },
            "localization": {
                "position_error_warn_m": self.localization.position_error_warn_m,
                "position_error_error_m": self.localization.position_error_error_m,
                "angle_error_warn_deg": self.localization.angle_error_warn_deg,
                "angle_error_error_deg": self.localization.angle_error_error_deg,
            },
            "ros_domain_id": self.ros_domain_id,
            "agv": {"robokit_host": self.agv_robokit_host},
            "xarm7": {
                "host": self.xarm_robot_ip,
                "model": self.xarm_model,
                "default_arm_mode": self.xarm_default_mode,
            },
        }


def load_floor_qr_mapping(path: Optional[Path] = None) -> Dict[str, Dict[str, Any]]:
    if path is None:
        path = Path(__file__).resolve().parents[1] / "config" / "floor_qr_mapping.yaml"
    if not path.is_file() or yaml is None:
        return {}
    data = _load_yaml(path)
    mappings = data.get("mappings") or {}
    return mappings if isinstance(mappings, dict) else {}
