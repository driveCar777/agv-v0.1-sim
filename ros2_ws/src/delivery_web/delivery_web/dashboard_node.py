"""Dual HTTP dashboards for delivery v0.2.0

- Demo Console  :19999  (operator / acceptance)
- Debug Console :1999   (dji_gui-inspired: param tree, Hz, logs, watchlist)
Both bind 0.0.0.0 for LAN (Windows needs portproxy — see admin ps1).
"""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from delivery_web.stack_supervisor import StackSupervisor
from delivery_interfaces.msg import (
    AgvStatus,
    ArmStatus,
    CameraInfoLite,
    FaceDetectionArray,
    QrDetectionArray,
)
from delivery_interfaces.srv import (
    AgvCancel,
    AgvNavigate,
    ArmExecuteSequence,
    DetectFace,
    DetectQr,
    IdentifyFace,
)

from delivery_web.system_logger import SystemJsonlLogger, load_env_file, save_env_file
from delivery_web.verbose_logger import VerboseLogger
from delivery_web.device_config import DeviceConfig
from delivery_web.smap_editor import (
    add_station,
    connect_stations,
    delete_station,
    load_smap_file,
    rename_station,
    save_smap_file,
    stations_from_smap,
)
from delivery_web.smap_sim import (
    DEFAULT_MAPS_DIR,
    DEFAULT_SMAP,
    list_smaps,
    load_smap_layers,
    new_obstacle,
    segment_hits_obstacle,
    sim_laser_from_map,
    stations_json_from_layers,
)

try:
    from agv_bridge.agv_adapter import AgvAdapterConfig, create_agv_adapter
    from agv_bridge.robokit_client import RobokitError
except Exception:  # noqa: BLE001
    create_agv_adapter = None  # type: ignore
    AgvAdapterConfig = None  # type: ignore
    RobokitError = Exception  # type: ignore


VERSION = "0.51.4"
WEB_FACE_API = 1
DEFAULT_FACE_PEER = os.environ.get("FACE_JETSON_IP", "192.168.0.225")
DEFAULT_DEMO_AGV = os.environ.get("DEMO_AGV_HOST", "192.168.18.198")
CAMERAS_META = {
    "cam_left": {
        "label": "相机1",
        "logical_name": "cam_left",
        "role": "qr_scan",
        "capabilities": ["qr"],
    },
    "cam_front": {
        "label": "相机2",
        "logical_name": "cam_front",
        "role": "face_qr_object",
        "capabilities": ["face", "qr", "object"],
    },
    "cam_right": {
        "label": "相机3",
        "logical_name": "cam_right",
        "role": "third_view_cargo",
        "capabilities": ["view", "cargo"],
    },
}


def _agv(msg: AgvStatus) -> Dict[str, Any]:
    return {
        "task_status": int(msg.task_status),
        "task_type": int(msg.task_type),
        "target_id": msg.target_id,
        "finished_path": list(msg.finished_path),
        "unfinished_path": list(msg.unfinished_path),
        "blocked": bool(msg.blocked),
        "emergency": bool(msg.emergency),
        "soft_emc": bool(msg.soft_emc),
        "battery_level": float(msg.battery_level),
        "charging": bool(msg.charging),
        "current_map": msg.current_map,
        "vehicle_id": msg.vehicle_id,
        "x": float(msg.x),
        "y": float(msg.y),
        "angle": float(msg.angle),
        "confidence": float(msg.confidence),
        "current_station": msg.current_station,
        "mode": msg.mode,
        "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
    }


def _arm(msg: ArmStatus) -> Dict[str, Any]:
    return {
        "arm_name": msg.arm_name,
        "connected": bool(msg.connected),
        "is_moving": bool(msg.is_moving),
        "error": bool(msg.error),
        "mode_name": msg.mode_name,
        "joint_positions": [float(x) for x in msg.joint_positions],
        "current_sequence": msg.current_sequence,
        "error_msg": msg.error_msg,
    }


def _qr(msg: QrDetectionArray) -> Dict[str, Any]:
    return {
        "camera_name": msg.camera_name,
        "detections": [
            {
                "code_type": d.code_type,
                "payload": d.payload,
                "x": float(d.x),
                "y": float(d.y),
                "z": float(d.z),
                "yaw": float(d.yaw),
                "confidence": float(d.confidence),
            }
            for d in msg.detections
        ],
    }


def _msg_to_rgb(msg: Image) -> Optional[tuple]:
    """Return (w, h, rgb_bytes) or None. Never PPM — browsers cannot show PPM in <img>."""
    w, h = int(msg.width), int(msg.height)
    if w <= 0 or h <= 0:
        return None
    data = bytes(msg.data)
    enc = (msg.encoding or "").lower()
    need = w * h * (1 if enc in ("mono8", "8uc1") else 3)
    if len(data) < need:
        return None
    if enc in ("rgb8", "rgb8;"):
        return w, h, data[: w * h * 3]
    if enc in ("bgr8", "bgr8;"):
        arr = bytearray(data[: w * h * 3])
        for i in range(0, len(arr), 3):
            arr[i], arr[i + 2] = arr[i + 2], arr[i]
        return w, h, bytes(arr)
    if enc in ("mono8", "8uc1"):
        # expand gray -> RGB
        g = data[: w * h]
        rgb = bytearray(w * h * 3)
        for i, v in enumerate(g):
            rgb[i * 3 : i * 3 + 3] = bytes((v, v, v))
        return w, h, bytes(rgb)
    if "bayer" in enc or enc in ("yuv422", "uyvy", "yuyv"):
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore

            arr = np.frombuffer(data, dtype=np.uint8)
            if enc.startswith("bayer"):
                img = arr.reshape(h, w)
                code = getattr(cv2, "COLOR_BayerBG2BGR", cv2.COLOR_BayerRG2BGR)
                if "rggb" in enc:
                    code = cv2.COLOR_BayerRG2BGR
                elif "grbg" in enc:
                    code = cv2.COLOR_BayerGR2BGR
                elif "gbrg" in enc:
                    code = cv2.COLOR_BayerGB2BGR
                bgr = cv2.cvtColor(img, code)
            else:
                img = arr.reshape(h, w, -1)
                bgr = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
            rgb = bgr[:, :, ::-1].tobytes()
            return w, h, rgb
        except Exception:  # noqa: BLE001
            pass
    return None


def _rgb_to_png(w: int, h: int, rgb: bytes) -> bytes:
    import struct
    import zlib

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    raw = b"".join(b"\x00" + rgb[y * w * 3 : (y + 1) * w * 3] for y in range(h))
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


def _image_ctype(blob: bytes) -> str:
    if len(blob) >= 2 and blob[:2] == b"\xff\xd8":
        return "image/jpeg"
    if len(blob) >= 8 and blob[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png"
    return "application/octet-stream"


def _rgb8_to_jpeg(msg: Image) -> Optional[bytes]:
    """Encode camera frame for browser <img>/MJPEG. Prefer JPEG; PNG fallback; never PPM."""
    parsed = _msg_to_rgb(msg)
    if not parsed:
        return None
    w, h, rgb = parsed
    max_w = 800
    # 1) Pillow JPEG
    try:
        from io import BytesIO

        from PIL import Image as PILImage  # type: ignore

        im = PILImage.frombytes("RGB", (w, h), rgb)
        if im.width > max_w:
            nh = max(1, int(im.height * max_w / im.width))
            im = im.resize((max_w, nh))
        buf = BytesIO()
        im.save(buf, format="JPEG", quality=50)
        return buf.getvalue()
    except Exception:  # noqa: BLE001
        pass
    # 2) OpenCV JPEG
    try:
        import numpy as np  # type: ignore
        import cv2  # type: ignore

        arr = np.frombuffer(rgb, dtype=np.uint8).reshape(h, w, 3)
        bgr = arr[:, :, ::-1].copy()
        ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
        if ok:
            return bytes(enc)
    except Exception:  # noqa: BLE001
        pass
    # 3) PNG (browser-safe; better than PPM blank)
    try:
        return _rgb_to_png(w, h, rgb)
    except Exception:  # noqa: BLE001
        return None


def _flatten(obj: Any, prefix: str = "") -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            p = f"{prefix}.{k}" if prefix else str(k)
            rows.extend(_flatten(v, p))
    elif isinstance(obj, list):
        if not obj or all(not isinstance(x, (dict, list)) for x in obj):
            rows.append({"path": prefix, "value": obj, "type": "list"})
        else:
            for i, v in enumerate(obj):
                rows.extend(_flatten(v, f"{prefix}[{i}]"))
    else:
        rows.append({"path": prefix, "value": obj, "type": type(obj).__name__})
    return rows


class DashboardNode(Node):
    # Default subscribe table (inspired by dji_gui default_subscribe_list)
    SUBSCRIBE_SPEC = [
        {"name": "agv/status", "topic": "agv/status", "type": "delivery_interfaces/AgvStatus", "auto": True},
        {"name": "agv/pose", "topic": "agv/pose", "type": "delivery_interfaces/AgvPose", "auto": True},
        {"name": "arm/left/status", "topic": "arm/left/status", "type": "delivery_interfaces/ArmStatus", "auto": True},
        {"name": "arm/right/status", "topic": "arm/right/status", "type": "delivery_interfaces/ArmStatus", "auto": True},
        {"name": "camera/cam_front/qr", "topic": "camera/cam_front/qr", "type": "QrDetectionArray", "auto": True},
        {"name": "camera/cam_left/qr", "topic": "camera/cam_left/qr", "type": "QrDetectionArray", "auto": True},
        {"name": "camera/cam_right/qr", "topic": "camera/cam_right/qr", "type": "QrDetectionArray", "auto": True},
        {"name": "camera/cam_front/image_raw", "topic": "/camera/cam_front/image_raw", "type": "sensor_msgs/Image", "auto": True},
        {"name": "camera/cam_left/image_raw", "topic": "/camera/cam_left/image_raw", "type": "sensor_msgs/Image", "auto": True},
        {"name": "camera/cam_right/image_raw", "topic": "/camera/cam_right/image_raw", "type": "sensor_msgs/Image", "auto": True},
        {"name": "face/status", "topic": "face/status", "type": "std_msgs/String", "auto": True},
        {
            "name": "face/cam_front/detections",
            "topic": "face/cam_front/detections",
            "type": "delivery_interfaces/FaceDetectionArray",
            "auto": True,
        },
        {"name": "agv/odom", "topic": "/agv/odom", "type": "nav_msgs/Odometry", "auto": False},
        {"name": "agv/cmd_vel", "topic": "/agv/cmd_vel", "type": "geometry_msgs/Twist", "auto": False},
        {"name": "joint_states", "topic": "/joint_states", "type": "sensor_msgs/JointState", "auto": False},
    ]

    def __init__(self) -> None:
        super().__init__("dashboard_node")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("demo_port", 19999)
        self.declare_parameter("debug_port", 1999)
        # legacy alias
        self.declare_parameter("http_port", 19999)
        self.declare_parameter(
            "stations_file",
            "/data/agv_downloaded/maps/stations_from_smap.json",
        )
        self.declare_parameter("maps_dir", str(DEFAULT_MAPS_DIR))
        self.declare_parameter("smap_file", os.environ.get("SMAP_FILE", DEFAULT_SMAP))
        self.declare_parameter("face_peer", DEFAULT_FACE_PEER)
        self.declare_parameter("face_log_dir", os.environ.get("FACE_LOG_DIR", "/var/log/delivery/face"))
        self.declare_parameter("system_log_dir", os.environ.get("SYSTEM_LOG_DIR", "/var/log/delivery/system"))
        self.declare_parameter("demo_agv_host", DEFAULT_DEMO_AGV)
        self.declare_parameter("laser_hz", 2.0)
        self.declare_parameter("laser_max_points", 720)

        self._cg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._slog = SystemJsonlLogger(
            str(self.get_parameter("system_log_dir").value)
        )
        verbose_dir = os.environ.get("VERBOSE_LOG_DIR", "/tmp/agv_verbose_logs")
        self._vlog = VerboseLogger(verbose_dir)
        self._env = load_env_file(
            default_mode="demo",
            default_agv=str(self.get_parameter("demo_agv_host").value),
        )
        if os.environ.get("USE_SIM_CAMERAS", "0").strip().lower() in ("0", "false", "no", "off"):
            self._env["use_sim_cameras"] = False
        self._env["mode"] = "demo"
        self._device_cfg = DeviceConfig.load()
        self._agv_adapter: Any = None
        self._control_locked = False
        self._control_wanted = True
        self._lock_nick = "ros2_delivery"
        self._events: deque = deque(maxlen=2000)
        self._pose_hist: deque = deque(maxlen=400)
        self._telem: deque = deque(maxlen=240)
        self._jpeg: Dict[str, bytes] = {}
        self._cam_info: Dict[str, Dict[str, Any]] = {}
        self._msg_counts: Dict[str, int] = defaultdict(int)
        self._hz: Dict[str, float] = {}
        self._last_hz_t = time.time()
        self._last_counts: Dict[str, int] = defaultdict(int)
        self._face_status_wall = 0.0
        self._face_det_wall = 0.0
        self._watch: List[str] = [
            "agv.x",
            "agv.y",
            "agv.angle",
            "agv.task_status",
            "agv.current_station",
            "agv.target_id",
            "agv.battery_level",
            "arms.left.is_moving",
            "arms.right.is_moving",
            "nodes_health",
            "face.ok",
            "face.backend",
            "face.person_id",
            "face.confidence",
        ]
        peer = self.get_parameter("face_peer").get_parameter_value().string_value
        self._state: Dict[str, Any] = {
            "version": VERSION,
            "agv": None,
            "arms": {"left": None, "right": None},
            "cameras": {
                "cam_front": {"detections": []},
                "cam_left": {"detections": []},
                "cam_right": {"detections": []},
            },
            "cameras_meta": json.loads(json.dumps(CAMERAS_META)),
            "face": {
                "ok": False,
                "connected": False,
                "backend": "unknown",
                "peer": peer or DEFAULT_FACE_PEER,
                "domain_id": int(os.environ.get("ROS_DOMAIN_ID", "30")),
                "camera": "cam_front",
                "last_event": "",
                "last_error": "waiting_face_bridge",
                "detections_hz": 0.0,
                "person_id": "",
                "person_name": "",
                "confidence": 0.0,
                "bbox_xywh": [],
                "track_id": "",
                "last_identify": None,
                "updated_at": 0.0,
            },
            "cargo": None,
            "laser": {
                "ok": False,
                "source": "none",
                "points": [],
                "beam_count": 0,
                "message": "waiting",
                "updated_at": 0.0,
            },
            "env": self._public_env(),
            "stations": {},
            "meta": {},
            "map": {
                "cloud": [],
                "curves": [],
                "cloud_shown": 0,
                "cloud_total": 0,
                "curve_count": 0,
                "smap_file": "",
                "available": [],
            },
            "obstacles": [],
            "route": {"stations": [], "active": False, "index": 0},
            "updated_at": time.time(),
            "uptime_sec": 0.0,
            "nodes_health": {},
            "ports": {"demo": 19999, "debug": 1999},
            "web_face_api": WEB_FACE_API,
        }
        self._t0 = time.time()
        self._agv_last_seen = 0.0
        self._route_task = {"running": False, "completed": [], "current": "", "next": "", "queue": []}
        self._jason_web = None
        self._vision_manager = None
        self._arm_manager = None
        self._vision_cache: Dict[str, Any] = {}
        self._arm_cache: Dict[str, Any] = {}
        self._wrist_snap_lock = threading.Lock()
        self._wrist_snap_cached: tuple = (0.0, b"")
        self._vision_warmup_done = False
        self._stack_supervisor = StackSupervisor(
            self._device_cfg,
            logger=lambda msg: self.get_logger().info(str(msg)),
        )
        self._current_robot_map = ""
        self._push_cache = None
        self._push_client = None
        self._control_locked = False
        self._lock_nick = "ros2_delivery"
        self._maps_dir = Path(str(self.get_parameter("maps_dir").value) or str(DEFAULT_MAPS_DIR))
        self._map_cloud: List[Dict[str, float]] = []
        self._smap_raw: Dict[str, Any] = {}
        self._smap_path: Optional[Path] = None
        self._load_smap(str(self.get_parameter("smap_file").value) or DEFAULT_SMAP)
        self._push_event("system", f"dashboard v{VERSION} demo:19999 debug:1999 (dji_gui-inspired)")
        self._slog.log(
            "boot",
            kind="system",
            msg=f"dashboard boot env={self._env.get('mode')} agv={self._env.get('agv_host')}",
            mode=self._env.get("mode"),
            agv_host=self._env.get("agv_host"),
        )
        self._init_adapter()
        if self._env.get("mode") in ("demo", "mock"):
            self._ensure_adapter(str(self._env.get("agv_host") or DEFAULT_DEMO_AGV))
            threading.Thread(target=self._boot_map_sync, daemon=True).start()

        self.create_subscription(AgvStatus, "agv/status", self._on_agv, 10)
        self.create_subscription(ArmStatus, "arm/left/status", self._on_arm_left, 10)
        self.create_subscription(ArmStatus, "arm/right/status", self._on_arm_right, 10)
        for name in ("cam_front", "cam_left", "cam_right"):
            self.create_subscription(
                QrDetectionArray,
                f"camera/{name}/qr",
                lambda m, n=name: self._on_qr(n, m),
                10,
            )
            self.create_subscription(
                Image,
                f"/camera/{name}/image_raw",
                lambda m, n=name: self._on_img(n, m),
                5,
            )
            self.create_subscription(
                CameraInfoLite,
                f"/camera/{name}/info",
                lambda m, n=name: self._on_cam_info(n, m),
                10,
            )

        self.create_subscription(String, "face/status", self._on_face_status, 10)
        self.create_subscription(
            FaceDetectionArray,
            "face/cam_front/detections",
            lambda m: self._on_face_dets("cam_front", m),
            10,
        )

        self._cli_nav = self.create_client(AgvNavigate, "agv/navigate", callback_group=self._cg)
        self._cli_cancel = self.create_client(AgvCancel, "agv/cancel", callback_group=self._cg)
        self._cli_arm = self.create_client(
            ArmExecuteSequence, "arm/execute_sequence", callback_group=self._cg
        )
        self._cli_qr = self.create_client(DetectQr, "camera/detect_qr", callback_group=self._cg)
        self._cli_face_id = self.create_client(
            IdentifyFace, "face/identify_face", callback_group=self._cg
        )
        self._cli_face_det = self.create_client(
            DetectFace, "face/detect_face", callback_group=self._cg
        )
        self.create_timer(1.0, self._health_tick)
        self.create_timer(1.0, self._hz_tick)
        laser_hz = float(self.get_parameter("laser_hz").value)
        self.create_timer(1.0 / max(laser_hz, 0.5), self._laser_tick)
        self.create_timer(1.0, self._agv_status_tick)
        self.create_timer(2.0, self._camera_diag_tick)
        self.create_timer(1.0, self._vision_warmup_tick, callback_group=self._cg)
        self.create_timer(30.0, self._stack_watchdog_tick, callback_group=self._cg)
        self.create_timer(2.0, self._vision_status_tick, callback_group=self._cg)

        share = Path(get_package_share_directory("delivery_web"))
        www = share / "www"
        if not (www / "index.html").is_file():
            www = Path(__file__).resolve().parents[1] / "www"
        self._www_demo = www
        self._www_debug = www / "debug"
        if not (self._www_debug / "index.html").is_file():
            alt = Path(__file__).resolve().parents[1] / "www" / "debug"
            if (alt / "index.html").is_file():
                self._www_debug = alt

        host = self.get_parameter("http_host").get_parameter_value().string_value
        demo_port = int(self.get_parameter("demo_port").value)
        debug_port = int(self.get_parameter("debug_port").value)
        self._state["ports"] = {"demo": demo_port, "debug": debug_port}

        self._httpd_demo = ThreadingHTTPServer(
            (host, demo_port), self._make_handler(mode="demo")
        )
        self._httpd_demo.allow_reuse_address = True
        self._httpd_debug = ThreadingHTTPServer(
            (host, debug_port), self._make_handler(mode="debug")
        )
        self._httpd_debug.allow_reuse_address = True
        try:
            threading.Thread(target=self._httpd_demo.serve_forever, daemon=True).start()
            threading.Thread(target=self._httpd_debug.serve_forever, daemon=True).start()
        except OSError as exc:
            self.get_logger().error(f"HTTP server failed to bind: {exc}")
            raise
        self.get_logger().info(
            f"Web DEMO  http://0.0.0.0:{demo_port}/  |  DEBUG http://0.0.0.0:{debug_port}/"
        )

    def _vision_warmup_tick(self) -> None:
        if self._vision_warmup_done:
            return
        self._vision_warmup_done = True
        try:
            stack = self._stack_supervisor.ensure_all()
            self.get_logger().info(f"stack supervisor: {stack}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"stack supervisor failed: {exc}")
        try:
            self._get_vision().warmup()
            self.get_logger().info("vision bridges warmed up")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"vision warmup failed: {exc}")
        threading.Thread(target=self._leo_autostart, daemon=True).start()

    def _leo_autostart(self) -> None:
        # Same command as Leo: SetBool {data: true} on /face/cam_front/continuous_recognition.
        time.sleep(2.0)
        try:
            out = self._get_vision().leo_ensure_continuous()
            self.get_logger().info(f"leo SetBool(true): {out.get('status') or out.get('message')}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"leo SetBool(true) failed: {exc}")

    def _stack_watchdog_tick(self) -> None:
        try:
            self._stack_supervisor.ensure_all()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"stack watchdog failed: {exc}")

    def _vision_status_tick(self) -> None:
        try:
            v = self._get_vision()
            self._vision_cache = {
                "floor_qr_scanner": v.floor_qr_status(),
                "floor_qr_localization": v.floor_qr_localization(),
                "wrist_camera": v.wrist_status(),
                "leo_face": v.leo_status(),
            }
            try:
                self._arm_cache = self._get_arm().status()
            except Exception:  # noqa: BLE001
                pass
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"vision status tick failed: {exc}")

    def _vision_snapshot(self) -> Dict[str, Any]:
        cache = getattr(self, "_vision_cache", None) or {}
        if cache:
            return dict(cache)
        try:
            v = self._get_vision()
            return {
                "floor_qr_scanner": v.floor_qr_status(),
                "floor_qr_localization": v.floor_qr_localization(),
                "wrist_camera": v.wrist_status(),
                "leo_face": v.leo_status(),
            }
        except Exception:  # noqa: BLE001
            return {}

    def _get_stations_dict(self) -> Dict[str, Dict[str, float]]:
        with self._lock:
            return dict(self._state.get("stations") or {})

    def _is_robot_busy(self) -> bool:
        agv = self._state.get("agv") or {}
        ts = int(agv.get("task_status", 0) or 0)
        spd = float(agv.get("vx", 0) or agv.get("speed", 0) or 0)
        if ts == 2 and abs(spd) > 0.05:
            return True
        return ts in (3,)

    def _boot_map_sync(self) -> None:
        time.sleep(0.8)
        try:
            out = self.sync_map_from_robot()
            if out.get("success"):
                self.get_logger().info(f"boot map sync: {out.get('map_name') or out.get('message')}")
            else:
                self.get_logger().warn(f"boot map sync failed: {out.get('message')}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"boot map sync error: {exc}")

    def _robot_map_options(self) -> List[Dict[str, Any]]:
        if self._agv_adapter is None or not self._agv_adapter.connected:
            return list_smaps(self._maps_dir)
        try:
            maps = self._agv_adapter.get_maps()
            out: List[Dict[str, Any]] = []
            for m in maps:
                name = str(m.name or "").replace(".smap", "")
                if not name:
                    continue
                out.append(
                    {
                        "file": f"{name}.smap",
                        "name": name,
                        "current": bool(m.is_current),
                    }
                )
            return out or list_smaps(self._maps_dir)
        except Exception:  # noqa: BLE001
            return list_smaps(self._maps_dir)

    def _get_robot_pose_dict(self) -> Optional[Dict[str, float]]:
        agv = self._state.get("agv") or {}
        if not agv:
            return None
        return {
            "x": float(agv.get("x", 0.0) or 0.0),
            "y": float(agv.get("y", 0.0) or 0.0),
            "angle": float(agv.get("angle", 0.0) or 0.0),
        }

    def _gazebo_preview_jpeg(self) -> Optional[bytes]:
        with self._lock:
            for key in ("cam_left", "cam_right", "cam_front"):
                data = self._jpeg.get(key)
                if data and len(data) > 100:
                    return data
        return None

    def _get_vision(self):
        if self._vision_manager is None:
            from delivery_web.vision.manager import VisionManager

            self._vision_manager = VisionManager(
                self,
                self._lock,
                _rgb8_to_jpeg,
                self.get_logger().info,
                get_mode=lambda: str(self._env.get("mode", "dev")),
                get_robot_pose=self._get_robot_pose_dict,
                get_face_state=lambda: dict(self._state.get("face") or {}),
                get_stations=self._get_stations_dict,
                get_use_sim_cameras=lambda: False,
                get_gazebo_jpeg=self._gazebo_preview_jpeg,
                cfg=self._device_cfg,
            )
        return self._vision_manager

    def _agv_nav_busy(self) -> bool:
        if self._route_task.get("running"):
            return True
        if self._is_robot_busy():
            return True
        agv = self._state.get("agv") or {}
        spd = float(agv.get("vx", 0) or agv.get("speed", 0) or 0)
        if abs(spd) > 0.05:
            return True
        return False

    def _get_arm(self):
        if self._arm_manager is None:
            from delivery_web.arm.manager import ArmManager

            robot_ip = str(getattr(self._device_cfg, "xarm_robot_ip", "172.31.0.123"))
            self._arm_manager = ArmManager(
                self,
                get_env_mode=lambda: str(self._env.get("mode", "dev")),
                robot_ip=robot_ip,
                get_agv_busy=self._agv_nav_busy,
            )
        return self._arm_manager

    def _get_jason_web(self):
        """Backward-compatible alias → wrist camera / vision manager."""
        return self._get_vision()

    def _tick_count(self, key: str) -> None:
        with self._lock:
            self._msg_counts[key] += 1

    def _hz_tick(self) -> None:
        now = time.time()
        dt = max(now - self._last_hz_t, 1e-3)
        with self._lock:
            for k, c in self._msg_counts.items():
                self._hz[k] = (c - self._last_counts[k]) / dt
                self._last_counts[k] = c
        self._last_hz_t = now

    def _health_tick(self) -> None:
        with self._lock:
            agv = self._state["agv"]
            face_age = time.time() - self._face_status_wall if self._face_status_wall else 1e9
            face_connected = face_age < 5.0
            face = self._state["face"]
            face["connected"] = face_connected
            if not face_connected and face.get("backend") != "unknown":
                face["ok"] = False
                if not face.get("last_error"):
                    face["last_error"] = "face_status_stale"
            self._state["nodes_health"] = {
                "agv": agv is not None,
                "arm_left": self._state["arms"]["left"] is not None,
                "arm_right": self._state["arms"]["right"] is not None,
                "cam_front": "cam_front" in self._jpeg
                or bool(self._state["cameras"]["cam_front"].get("detections")),
                "face": face_connected,
                "web": True,
                "mode": (agv or {}).get("mode", "unknown"),
            }
            self._state["uptime_sec"] = time.time() - self._t0

    def _push_event(
        self,
        kind: str,
        message: str,
        *,
        level: str = "INFO",
        event: str = "",
        trace_id: str = "",
    ) -> None:
        with self._lock:
            self._events.appendleft(
                {
                    "t": time.time(),
                    "kind": kind,
                    "message": message,
                    "level": level,
                    "event": event or kind,
                    "trace_id": trace_id,
                }
            )

    def _load_stations(self) -> None:
        """Fallback: stations JSON only (no cloud/curves). Prefer _load_smap."""
        path = Path(self.get_parameter("stations_file").get_parameter_value().string_value)
        if not path.is_file():
            return
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        with self._lock:
            self._state["meta"] = {
                "map_name": data.get("map_name", ""),
                "vehicle_model": data.get("vehicle_model", ""),
                "map_file": data.get("map_file", ""),
            }
            self._state["stations"] = {
                k: {"x": float(v["x"]), "y": float(v["y"]), "yaw": float(v.get("yaw", 0))}
                for k, v in data.get("stations", {}).items()
            }

    def _writable_maps_dir(self) -> Path:
        for cand in (
            Path(os.environ.get("MAPS_RW_DIR", "/opt/delivery_ws/maps_rw")),
            Path("/tmp/agv_maps_rw"),
        ):
            try:
                cand.mkdir(parents=True, exist_ok=True)
                probe = cand / ".w"
                probe.write_text("1", encoding="utf-8")
                probe.unlink()
                return cand
            except OSError:
                continue
        return Path("/tmp/agv_maps_rw")

    def _load_smap(self, smap_name: str) -> Dict[str, Any]:
        name = Path(str(smap_name)).name
        path = None
        for d in (self._writable_maps_dir(), self._maps_dir):
            cand = d / name
            if cand.is_file():
                path = cand
                break
        if path is None:
            self._load_stations()
            return {"success": False, "message": f"smap not found: {name}"}
        self._smap_raw = load_smap_file(path)
        self._smap_path = path
        layers = load_smap_layers(path, max_cloud=8000)
        stations_path = Path(self.get_parameter("stations_file").value)
        try:
            if stations_path.parent.is_dir() and os.access(stations_path.parent, os.W_OK):
                stations_path.write_text(
                    json.dumps(stations_json_from_layers(layers), ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
        except Exception:  # noqa: BLE001
            pass
        cloud = list(layers.get("cloud") or [])
        # UI gets a lighter sample; laser uses full in-memory cloud
        step = max(1, len(cloud) // 1800) if cloud else 1
        cloud_ui = cloud[::step]
        with self._lock:
            self._map_cloud = cloud
            self._state["meta"] = {
                "map_name": layers.get("map_name", ""),
                "vehicle_model": layers.get("vehicle_model", "AMB-150"),
                "map_file": layers.get("map_file", name),
            }
            self._state["stations"] = layers.get("stations") or {}
            self._state["map"] = {
                "cloud": cloud_ui,
                "curves": layers.get("curves") or [],
                "cloud_shown": len(cloud_ui),
                "cloud_total": int(layers.get("cloud_total") or 0),
                "cloud_internal": len(cloud),
                "curve_count": int(layers.get("curve_count") or 0),
                "smap_file": name,
                "header": layers.get("header") or {},
                "available": self._robot_map_options()
                if str(self._env.get("mode", "dev")) in ("demo", "mock")
                else list_smaps(self._maps_dir),
            }
            self._state["updated_at"] = time.time()
        self._push_event(
            "map",
            f"loaded {name} cloud={layers.get('cloud_shown')}/{layers.get('cloud_total')} "
            f"curves={layers.get('curve_count')} stations={len(layers.get('stations') or {})}",
        )
        self._slog.log(
            "smap_loaded",
            kind="map",
            msg=f"smap {name}",
            cloud_shown=layers.get("cloud_shown"),
            curves=layers.get("curve_count"),
        )
        return {"success": True, "message": "ok", "map": self.snapshot().get("map"), "meta": self.snapshot().get("meta")}

    def list_maps(self) -> Dict[str, Any]:
        mode = str(self._env.get("mode", "dev"))
        available = self._robot_map_options() if mode in ("demo", "mock") else list_smaps(self._maps_dir)
        return {
            "success": True,
            "maps_dir": str(self._maps_dir),
            "available": available,
            "current": (self.snapshot().get("map") or {}).get("smap_file"),
        }

    def set_map(self, smap_name: str) -> Dict[str, Any]:
        return self._load_smap(smap_name)

    def sync_map_from_robot(self) -> Dict[str, Any]:
        mode = str(self._env.get("mode", "dev"))
        self._vlog.log("map", "Sync map from robot", {"mode": mode})
        if mode == "dev":
            return {"success": True, "message": "dev uses local smap", "source": "local"}
        if self._agv_adapter is None or not self._agv_adapter.connected:
            if not self._ensure_adapter():
                return {"success": False, "message": "adapter not connected"}
        try:
            assert self._agv_adapter is not None
            maps = self._agv_adapter.get_maps()
            current = self._agv_adapter.get_current_map()
            map_name = current.name if current else (maps[0].name if maps else "")
            if not map_name:
                return {"success": False, "message": "1300 returned no current_map"}
            if map_name == self._current_robot_map and (self._state.get("map") or {}).get("smap_file"):
                return {"success": True, "message": "unchanged", "map_name": map_name}
            raw = self._agv_adapter.download_map_json(map_name)
            if raw.get("ret_code", 0) not in (0, None) and "header" not in raw:
                return {"success": False, "message": raw.get("err_msg", "4011 failed"), "raw": raw}
            fname = f"{map_name}.smap" if not str(map_name).endswith(".smap") else str(map_name)
            path = self._writable_maps_dir() / Path(fname).name
            path.write_text(json.dumps(raw, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
            self._current_robot_map = map_name
            out = self._load_smap(path.name)
            out["source"] = "robokit_1300_4011"
            out["map_name"] = map_name
            self._vlog.log("map", "Map downloaded", {"map_name": map_name, "file": str(path)})
            with self._lock:
                self._state["map"]["available"] = self._robot_map_options()
                self._state["map"]["current_map"] = map_name
            return out
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "message": str(exc)}

    def switch_robot_map(self, map_name: str) -> Dict[str, Any]:
        self._vlog.log("map", "Switch robot map", {"map_name": map_name})
        mode = str(self._env.get("mode", "dev"))
        if mode == "dev":
            return self.set_map(map_name if map_name.endswith(".smap") else f"{map_name}.smap")
        if not self._ensure_adapter():
            return {"success": False, "message": "adapter not connected"}
        try:
            assert self._agv_adapter is not None
            sw = self._agv_adapter.switch_map(map_name.replace(".smap", ""))
            if sw.get("ret_code", 0) not in (0, None):
                return {"success": False, "message": sw.get("err_msg", "2022 failed"), "raw": sw}
            self._current_robot_map = map_name.replace(".smap", "")
            return self.sync_map_from_robot()
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "message": str(exc)}

    def _commit_smap_edit(self, updated: Dict[str, Any], path: Path, expect_id: str = "") -> Dict[str, Any]:
        """Persist station edits to a writable overlay, then Robokit 2025 (needs 4005)."""
        self._smap_raw = updated
        write_path = self._writable_maps_dir() / path.name
        local_ok = False
        local_err = ""
        try:
            save_smap_file(write_path, updated)
            self._smap_path = write_path
            local_ok = True
        except OSError as exc:
            local_err = str(exc)
        stations = stations_from_smap(updated)
        with self._lock:
            self._state["stations"] = stations
            meta = dict(self._state.get("map") or {})
            meta["smap_file"] = path.name
            self._state["map"] = meta
            self._state["updated_at"] = time.time()
        robot_ok = False
        robot_msg = ""
        adapter = self._agv_adapter
        if adapter is not None and bool(getattr(adapter, "connected", False)):
            try:
                self._ensure_control_lock()
                up = adapter.upload_and_switch_map_json(updated)
                robot_ok = up.get("ret_code", 0) in (0, None)
                robot_msg = str(up.get("err_msg") or ("2025 ok" if robot_ok else "2025 failed"))
                if robot_ok and expect_id and hasattr(adapter, "get_stations"):
                    ids = [str(getattr(s, "id", "") or "") for s in (adapter.get_stations() or [])]
                    if expect_id not in ids:
                        robot_ok = False
                        robot_msg = f"2025 已回 0 但 1301 还没有站点 {expect_id}"
            except Exception as exc:  # noqa: BLE001
                robot_ok = False
                robot_msg = f"2025 上传失败: {exc}"
                self.get_logger().warn(robot_msg)
        else:
            robot_msg = "adapter 未连接，未上传到车"
        if robot_ok:
            msg = f"站点已写入车载地图（2025）{'' if local_ok else '；本地只读目录已改写到可写副本'}"
        elif local_ok:
            msg = f"仅写入本地可写副本，未上到车: {robot_msg}"
        else:
            msg = f"保存失败 local={local_err or 'no'} robot={robot_msg}"
        return {
            "success": bool(robot_ok or local_ok),
            "persisted_local": local_ok,
            "persisted_robot": robot_ok,
            "upload_started": False,
            "smap_file": path.name,
            "write_path": str(write_path),
            "stations": stations,
            "message": msg,
        }

    def _upload_smap_bg(self, updated: Dict[str, Any], expect_id: str = "") -> None:
        adapter = self._agv_adapter
        if adapter is None:
            return
        try:
            up = adapter.upload_and_switch_map_json(updated)
            ok = up.get("ret_code", 0) in (0, None)
            self.get_logger().info(
                f"smap upload {'ok' if ok else 'failed'} expect={expect_id} {up.get('err_msg') or ''}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"smap background upload failed: {exc}")

    def _resolve_smap_path(self) -> Optional[Path]:
        smap_file = (self._state.get("map") or {}).get("smap_file") or DEFAULT_SMAP
        name = Path(str(smap_file)).name
        if self._smap_path is not None and self._smap_path.is_file():
            return self._smap_path
        for d in (self._writable_maps_dir(), self._maps_dir):
            path = d / name
            if path.is_file():
                return path
        return None

    def heartbeat(self) -> Dict[str, Any]:
        with self._lock:
            agv = dict(self._state.get("agv") or {})
            laser = dict(self._state.get("laser") or {})
        laser.pop("points", None)
        return {
            "ok": True,
            "agv_link": self.agv_link_status(),
            "env": self._public_env(),
            "agv": agv,
            "laser": laser,
            "arm": {
                "online": bool((getattr(self, "_arm_cache", None) or {}).get("online")),
                "motion": (getattr(self, "_arm_cache", None) or {}).get("motion"),
            },
        }

    def set_control_lock(self, wanted: bool) -> Dict[str, Any]:
        self._control_wanted = bool(wanted)
        if not wanted:
            try:
                if self._agv_adapter is not None:
                    self._agv_adapter.unlock()
            except Exception as exc:  # noqa: BLE001
                self._control_locked = False
                return {"success": False, "message": f"释放控制权失败: {exc}", "control_locked": False}
            self._control_locked = False
            self._slog.log("agv_unlock_ok", kind="agv", msg="4006 unlock")
            return {"success": True, "message": "已释放控制权", "control_locked": False, "control_wanted": False}
        out = self._ensure_control_lock()
        try:
            if out.get("success") and self._agv_adapter is not None:
                nav = self._agv_adapter.get_task_status()
                if int(getattr(nav, "task_status", 0) or 0) == 2:
                    self._agv_adapter.cancel_navigation()
                    out["cancelled_stale_task"] = True
        except Exception:  # noqa: BLE001
            pass
        return out

    def add_station_to_map(
        self, station_id: str, x: float, y: float, yaw: float = 0.0, connect_from: str = ""
    ) -> Dict[str, Any]:
        self._vlog.log("station", "Add station", {"station_id": station_id, "x": x, "y": y, "yaw": yaw})
        path = self._resolve_smap_path()
        if path is None and not self._smap_raw:
            return {"success": False, "message": f"smap not found in {self._maps_dir}"}
        smap = dict(self._smap_raw) if self._smap_raw else load_smap_file(path)
        link = str(connect_from or (self._state.get("agv") or {}).get("current_station") or "").strip()
        try:
            updated = add_station(smap, station_id, x, y, yaw, connect_from=link)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        out = self._commit_smap_edit(updated, path or Path(str(DEFAULT_SMAP)), expect_id=station_id)
        out["station_id"] = station_id
        out["connected_from"] = link
        return out

    def connect_map_stations(self, from_id: str, to_id: str) -> Dict[str, Any]:
        path = self._resolve_smap_path()
        if path is None and not self._smap_raw:
            return {"success": False, "message": "smap not found"}
        smap = dict(self._smap_raw) if self._smap_raw else load_smap_file(path)
        try:
            updated = connect_stations(smap, from_id, to_id)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        out = self._commit_smap_edit(updated, path or Path(str(DEFAULT_SMAP)), expect_id=to_id)
        out["from_id"] = from_id
        out["to_id"] = to_id
        return out

    def rename_map_station(self, old_id: str, new_id: str) -> Dict[str, Any]:
        self._vlog.log("station", "Rename station", {"old_id": old_id, "new_id": new_id})
        path = self._resolve_smap_path()
        if path is None and not self._smap_raw:
            return {"success": False, "message": "smap not found"}
        smap = dict(self._smap_raw) if self._smap_raw else load_smap_file(path)
        try:
            updated = rename_station(smap, old_id, new_id)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        out = self._commit_smap_edit(updated, path or Path(str(DEFAULT_SMAP)), expect_id=new_id)
        out["old_id"] = old_id
        out["new_id"] = new_id
        return out

    def delete_map_station(self, station_id: str) -> Dict[str, Any]:
        self._vlog.log("station", "Delete station", {"station_id": station_id})
        path = self._resolve_smap_path()
        if path is None and not self._smap_raw:
            return {"success": False, "message": "smap not found"}
        smap = dict(self._smap_raw) if self._smap_raw else load_smap_file(path)
        try:
            updated = delete_station(smap, station_id)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        out = self._commit_smap_edit(updated, path or Path(str(DEFAULT_SMAP)))
        out["station_id"] = station_id
        return out

    def mock_vision_control(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._get_vision().mock_control(payload)

    def list_obstacles(self) -> Dict[str, Any]:
        with self._lock:
            return {"success": True, "obstacles": list(self._state.get("obstacles") or [])}

    def add_obstacle(self, x: float, y: float, w: float = 0.6, h: float = 0.6) -> Dict[str, Any]:
        obs = new_obstacle(x, y, w, h)
        with self._lock:
            self._state.setdefault("obstacles", []).append(obs)
            self._state["updated_at"] = time.time()
        self._push_event("sim", f"obstacle + {obs['id']} @({x:.2f},{y:.2f})")
        return {"success": True, "obstacle": obs, "obstacles": self.list_obstacles()["obstacles"]}

    def clear_obstacles(self) -> Dict[str, Any]:
        with self._lock:
            self._state["obstacles"] = []
            self._state["updated_at"] = time.time()
        self._push_event("sim", "obstacles cleared")
        return {"success": True, "obstacles": []}

    def remove_obstacle(self, oid: str) -> Dict[str, Any]:
        with self._lock:
            before = list(self._state.get("obstacles") or [])
            self._state["obstacles"] = [o for o in before if str(o.get("id")) != str(oid)]
        return {"success": True, "obstacles": self.list_obstacles()["obstacles"]}

    def set_route(self, stations: List[str]) -> Dict[str, Any]:
        names = [str(s).strip() for s in stations if str(s).strip()]
        with self._lock:
            known = self._state.get("stations") or {}
            bad = [n for n in names if n not in known]
            if bad:
                return {"success": False, "message": f"unknown stations: {bad}"}
            self._state["route"] = {"stations": names, "active": False, "index": 0}
        self._push_event("sim", f"route set {' -> '.join(names)}")
        return {"success": True, "route": self.snapshot().get("route")}

    def run_route(self, wait: bool = True) -> Dict[str, Any]:
        with self._lock:
            route = dict(self._state.get("route") or {})
            names = list(route.get("stations") or [])
            if not names:
                return {"success": False, "message": "route empty; POST /api/sim/route first"}
            self._state["route"] = {**route, "active": True, "index": 0}
        results = []
        for i, st in enumerate(names):
            with self._lock:
                self._state["route"]["index"] = i
            out = self.navigate(st, wait=wait)
            results.append({"station": st, **out})
            if not out.get("success"):
                with self._lock:
                    self._state["route"]["active"] = False
                return {"success": False, "message": f"stopped at {st}", "results": results}
        with self._lock:
            self._state["route"]["active"] = False
            self._state["route"]["index"] = len(names)
        return {"success": True, "message": "route done", "results": results}

    def _mode_label(self, mode: str) -> str:
        return "实车模式"

    def _public_env(self) -> Dict[str, Any]:
        adapter = self._agv_adapter
        connected = bool(adapter and getattr(adapter, "connected", False))
        return {
            "mode": "demo",
            "agv_host": self._env.get("agv_host", DEFAULT_DEMO_AGV),
            "use_sim_cameras": False,
            "label": "实车模式",
            "adapter_connected": connected,
            "adapter_mode": getattr(adapter, "mode", "demo") if adapter else "demo",
            "robokit_host": self._device_cfg.agv_robokit_host,
            "ros_domain_id": self._device_cfg.ros_domain_id,
            "control_locked": bool(self._control_locked),
            "control_wanted": bool(self._control_wanted),
            "lock_nick": self._lock_nick,
        }

    def agv_link_status(self):
        now = time.time()
        adapter = self._agv_adapter
        adapter_ok = bool(adapter and getattr(adapter, "connected", False))
        last = self._agv_last_seen
        if adapter_ok:
            if last <= 0:
                return {"status": "ONLINE", "last_seen_ms_ago": None, "source": "robokit"}
            age = now - last
            st = "ONLINE" if age < 15 else ("WARNING" if age < 30 else "OFFLINE")
            return {"status": st, "last_seen_ms_ago": int(age * 1000), "source": "robokit"}
        if last <= 0:
            return {"status": "OFFLINE", "last_seen_ms_ago": None, "source": "ros"}
        age = now - last
        st = "ONLINE" if age < 2 else ("WARNING" if age < 5 else "OFFLINE")
        return {"status": st, "last_seen_ms_ago": int(age * 1000), "source": "ros"}

    def list_stations_api(self):
        with self._lock:
            return {"success": True, "stations": dict(self._state.get("stations") or {})}

    def add_station(self, name, x, y, yaw=0.0):
        name = str(name).strip()
        path = Path(self.get_parameter("stations_file").value)
        with self._lock:
            stations = dict(self._state.get("stations") or {})
            stations[name] = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._state["stations"] = stations
            meta = dict(self._state.get("meta") or {})
        data = {"map_file": meta.get("map_file",""), "map_name": meta.get("map_name",""),
                "vehicle_model": meta.get("vehicle_model","AMB-150"), "stations": stations}
        path.write_text(__import__('json').dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        return {"success": True, "stations": stations}

    def remove_station(self, name):
        name = str(name).strip()
        path = Path(self.get_parameter("stations_file").value)
        with self._lock:
            stations = dict(self._state.get("stations") or {})
            if name not in stations:
                return {"success": False, "message": f"station {name} not found"}
            stations.pop(name)
            self._state["stations"] = stations
            meta = dict(self._state.get("meta") or {})
        data = {"map_file": meta.get("map_file",""), "map_name": meta.get("map_name",""),
                "vehicle_model": meta.get("vehicle_model","AMB-150"), "stations": stations}
        path.write_text(__import__('json').dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        return {"success": True, "stations": stations}

    def get_route_status(self):
        with self._lock:
            return {"success": True, "route": dict(self._state.get("route") or {}), "route_task": dict(self._route_task)}

    def start_route_async(self, wait=True):
        with self._lock:
            names = list((self._state.get("route") or {}).get("stations") or [])
            if not names: return {"success": False, "message": "route empty"}
            tid = uuid.uuid4().hex[:12]
            self._route_task.update({"running": True, "task_id": tid, "current": names[0], "stations": names, "completed": []})
        def worker():
            for i, st in enumerate(names):
                with self._lock:
                    self._route_task["current"] = st
                    self._route_task["next"] = names[i+1] if i+1 < len(names) else ""
                out = self.navigate(st, wait=wait)
                if not out.get("success"):
                    with self._lock:
                        self._route_task.update({"running": False, "success": False, "message": f"stopped at {st}"})
                    return
                with self._lock:
                    self._route_task["completed"] = list(self._route_task.get("completed", [])) + [st]
            with self._lock:
                self._route_task.update({"running": False, "success": True, "message": "route complete", "current": ""})
        __import__('threading').Thread(target=worker, daemon=True).start()
        return {"success": True, "message": "route started", "task_id": tid}

    def _on_agv(self, msg: AgvStatus) -> None:
        if self._env.get("mode") in ("demo", "mock"):
            return
        self._agv_last_seen = time.time()
        self._tick_count("agv/status")
        d = _agv(msg)
        with self._lock:
            prev = self._state["agv"]
            self._state["agv"] = d
            self._state["updated_at"] = time.time()
            self._pose_hist.append({"t": d["stamp"], "x": d["x"], "y": d["y"], "angle": d["angle"]})
            self._telem.append(
                {
                    "t": time.time(),
                    "x": d["x"],
                    "y": d["y"],
                    "yaw": d["angle"],
                    "battery": d["battery_level"],
                    "task_status": d["task_status"],
                }
            )
            if prev and prev.get("task_status") != d["task_status"]:
                self._events.appendleft(
                    {
                        "t": time.time(),
                        "kind": "agv",
                        "message": f"task_status {prev.get('task_status')} -> {d['task_status']} target={d['target_id']}",
                    }
                )

    def _on_arm_left(self, msg: ArmStatus) -> None:
        self._tick_count("arm/left/status")
        with self._lock:
            self._state["arms"]["left"] = _arm(msg)
            self._state["updated_at"] = time.time()

    def _on_arm_right(self, msg: ArmStatus) -> None:
        self._tick_count("arm/right/status")
        with self._lock:
            self._state["arms"]["right"] = _arm(msg)
            self._state["updated_at"] = time.time()

    def _on_qr(self, name: str, msg: QrDetectionArray) -> None:
        self._tick_count(f"camera/{name}/qr")
        with self._lock:
            self._state["cameras"][name] = _qr(msg)
            self._state["updated_at"] = time.time()

    def _on_img(self, name: str, msg: Image) -> None:
        self._tick_count(f"/camera/{name}/image_raw")
        jpg = _rgb8_to_jpeg(msg)
        if jpg:
            with self._lock:
                prev = name not in self._jpeg
                self._jpeg[name] = jpg
            if prev:
                self._slog.log(
                    "camera_frame_first",
                    kind="camera",
                    msg=f"{name} first frame bytes={len(jpg)} ctype={_image_ctype(jpg)}",
                    camera=name,
                    bytes=len(jpg),
                    ctype=_image_ctype(jpg),
                )
        else:
            self._slog.log(
                "camera_encode_fail",
                kind="camera",
                level="WARN",
                msg=f"{name} encode failed enc={msg.encoding} {msg.width}x{msg.height}",
                camera=name,
                encoding=msg.encoding,
                width=int(msg.width),
                height=int(msg.height),
            )

    def _on_cam_info(self, name: str, msg: CameraInfoLite) -> None:
        self._tick_count(f"/camera/{name}/info")
        with self._lock:
            self._cam_info[name] = {
                "connected": bool(msg.connected),
                "width": int(msg.width),
                "height": int(msg.height),
                "fps": float(msg.fps),
                "transport": msg.transport,
            }

    def _on_face_status(self, msg: String) -> None:
        self._tick_count("face/status")
        self._face_status_wall = time.time()
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            data = {}
        with self._lock:
            face = self._state["face"]
            face["ok"] = bool(data.get("ok", True))
            face["connected"] = True
            face["backend"] = str(data.get("backend", face.get("backend") or "unknown"))
            face["peer"] = str(data.get("peer") or face.get("peer") or DEFAULT_FACE_PEER)
            face["domain_id"] = int(data.get("domain_id", face.get("domain_id") or 30))
            face["camera"] = str(data.get("camera") or face.get("camera") or "cam_front")
            face["last_event"] = str(data.get("last_event") or face.get("last_event") or "")
            face["last_error"] = str(data.get("last_error") or "")
            face["detections_hz"] = float(data.get("detections_hz") or 0.0)
            face["updated_at"] = time.time()
            self._state["updated_at"] = time.time()

    def _on_face_dets(self, camera: str, msg: FaceDetectionArray) -> None:
        self._tick_count(f"face/{camera}/detections")
        self._face_det_wall = time.time()
        best = None
        if msg.detections:
            best = max(msg.detections, key=lambda d: float(d.confidence))
        with self._lock:
            face = self._state["face"]
            face["camera"] = camera
            face["connected"] = True
            face["updated_at"] = time.time()
            if best is not None:
                face["person_id"] = best.person_id
                face["person_name"] = best.person_name
                face["confidence"] = float(best.confidence)
                face["bbox_xywh"] = [float(x) for x in best.bbox_xywh]
                face["track_id"] = best.track_id
                if best.person_id:
                    face["last_event"] = "det_with_id"
                else:
                    face["last_event"] = "det_box"
            else:
                face["person_id"] = ""
                face["person_name"] = ""
                face["confidence"] = 0.0
                face["bbox_xywh"] = []
                face["track_id"] = ""
            self._state["updated_at"] = time.time()

    def snapshot(self) -> Dict[str, Any]:
        vision = self._vision_snapshot()
        with self._lock:
            out = dict(self._state)
            laser = dict(out.get("laser") or {})
            pts = list(laser.get("points") or [])
            src = str(laser.get("source") or "")
            live = bool(laser.get("live_lidar")) or src.startswith("robokit")
            if live and len(pts) > 900:
                step = max(1, len(pts) // 900)
                laser["points"] = pts[::step]
            elif (not live) and len(pts) > 360:
                step = max(1, len(pts) // 360)
                laser["points"] = pts[::step]
            laser["live_lidar"] = live
            laser["label"] = (
                f"LIVE LIDAR {len(laser.get('points') or [])} pts"
                if live
                else ("SIMULATED LASER" if src in ("smap_cloud+obstacles", "sim_stub") else "NO LIVE LIDAR")
            )
            out["laser"] = laser
            out["events"] = list(self._events)[:40]
            out["pose_history"] = list(self._pose_hist)[-40:]
            out["telemetry"] = list(self._telem)[-40:]
            out["camera_info"] = dict(self._cam_info)
            out["camera_streams"] = {k: True for k in self._jpeg}
            out["uptime_sec"] = time.time() - self._t0
            out["hz"] = dict(self._hz)
            out["env"] = self._public_env()
            out["cargo"] = None
            out["web_face_api"] = WEB_FACE_API
            out["agv_link"] = self.agv_link_status()
            out["route_task"] = dict(self._route_task)
            out["vision"] = vision
            arm_cached = getattr(self, "_arm_cache", None) or {}
            out["arm"] = arm_cached if arm_cached else {}
            out["devices"] = self._device_cfg.public_dict()
            return out

    def verbose_log_status(self) -> Dict[str, bool]:
        return self._vlog.get_status()

    def verbose_log_toggle(self, module: str, enable: bool) -> Dict[str, Any]:
        if module not in self._vlog.get_status():
            return {"success": False, "message": f"Unknown module: {module}"}
        if enable:
            self._vlog.enable(module)
        else:
            self._vlog.disable(module)
        self._vlog.log("system", f"verbose log {module} -> {'on' if enable else 'off'}")
        return {
            "success": True,
            "module": module,
            "enabled": self._vlog.is_enabled(module),
            "message": f"全量日志 [{module}] 已{'开启' if enable else '关闭'}",
        }

    def verbose_log_toggle_all(self, enable: bool) -> Dict[str, Any]:
        for module in self._vlog.get_status():
            if enable:
                self._vlog.enable(module)
            else:
                self._vlog.disable(module)
        self._vlog.log("system", f"verbose log all -> {'on' if enable else 'off'}")
        return {
            "success": True,
            "enabled": enable,
            "message": f"全量日志全部{'开启' if enable else '关闭'}",
        }

    def verbose_log_recent(self, module: str, lines: int = 50) -> Dict[str, Any]:
        return {
            "module": module,
            "lines": self._vlog.get_recent_logs(module, lines),
        }

    def debug_snapshot(self) -> Dict[str, Any]:
        st = self.snapshot()
        flat = _flatten(
            {
                "agv": st.get("agv"),
                "arms": st.get("arms"),
                "cameras": st.get("cameras"),
                "cameras_meta": st.get("cameras_meta"),
                "face": st.get("face"),
                "cargo": st.get("cargo"),
                "camera_info": st.get("camera_info"),
                "meta": st.get("meta"),
                "nodes_health": st.get("nodes_health"),
                "ports": st.get("ports"),
                "version": st.get("version"),
                "uptime_sec": st.get("uptime_sec"),
            }
        )
        watch_vals = []
        fmap = {r["path"]: r["value"] for r in flat}
        for w in self._watch:
            watch_vals.append({"path": w, "value": fmap.get(w)})
        subs = []
        for s in self.SUBSCRIBE_SPEC:
            key = s["topic"]
            # hz keys match tick names
            hz_key = key
            subs.append(
                {
                    **s,
                    "hz": float(st.get("hz", {}).get(hz_key, 0.0)),
                    "count": int(self._msg_counts.get(hz_key, 0)),
                }
            )
        return {
            "version": VERSION,
            "inspired_by": "dji_gui (subscribe list / rate_stat / state tree / scroll log)",
            "state": st,
            "param_tree": flat,
            "watch": watch_vals,
            "subscriptions": subs,
            "logs": list(st.get("events") or [])[:400],
            "state_tree": {
                "title": "delivery_root",
                "type_name": "Root",
                "childs": [
                    {"title": "agv", "type_name": "AgvStatus", "childs": [], "value": st.get("agv")},
                    {"title": "arms", "type_name": "Arms", "childs": [
                        {"title": "left", "type_name": "ArmStatus", "value": (st.get("arms") or {}).get("left")},
                        {"title": "right", "type_name": "ArmStatus", "value": (st.get("arms") or {}).get("right")},
                    ]},
                    {"title": "cameras", "type_name": "Cameras", "childs": [
                        {"title": n, "type_name": "Camera", "value": (st.get("cameras") or {}).get(n)}
                        for n in ("cam_front", "cam_left", "cam_right")
                    ]},
                    {"title": "cameras_meta", "type_name": "CamerasMeta", "value": st.get("cameras_meta")},
                    {"title": "face", "type_name": "Face", "value": st.get("face")},
                    {"title": "cargo", "type_name": "Cargo", "value": st.get("cargo")},
                    {"title": "stations", "type_name": "Stations", "value": st.get("stations")},
                ],
            },
        }

    def latest_jpeg(self, name: str) -> Optional[bytes]:
        with self._lock:
            return self._jpeg.get(name)

    def _wait_future(self, future, timeout: float):
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return future.done()

    def _adapter_config(self) -> Any:
        return AgvAdapterConfig(
            agv_host=str(self._env.get("agv_host") or DEFAULT_DEMO_AGV),
            mock_host=os.environ.get("MOCK_AGV_HOST", "127.0.0.1"),
            laser_max_points=int(self.get_parameter("laser_max_points").value),
            maps_dir=self._maps_dir,
        )

    def _init_adapter(self) -> None:
        if create_agv_adapter is None:
            self._slog.log(
                "adapter_import_fail",
                kind="agv",
                level="ERROR",
                msg="agv_bridge.agv_adapter unavailable",
            )
            return
        mode = str(self._env.get("mode", "dev"))
        self._agv_adapter = create_agv_adapter(mode, self._adapter_config())

    def _ensure_adapter(self, host: Optional[str] = None) -> bool:
        if create_agv_adapter is None:
            self._slog.log(
                "adapter_import_fail",
                kind="agv",
                level="ERROR",
                msg="agv_bridge.agv_adapter unavailable",
            )
            return False
        mode = str(self._env.get("mode", "dev"))
        if mode == "dev":
            if self._agv_adapter is None or getattr(self._agv_adapter, "mode", "") != "dev":
                self._init_adapter()
            return True
        cfg = self._adapter_config()
        target = host or (cfg.mock_host if mode == "mock" else cfg.agv_host)
        if self._agv_adapter is None or getattr(self._agv_adapter, "mode", "") != mode:
            self._init_adapter()
        try:
            if self._agv_adapter.connected and self._agv_adapter.host == target:
                if not self._control_locked:
                    self._ensure_control_lock()
                return True
            ok = bool(self._agv_adapter.connect(target))
            if ok:
                self._slog.log(
                    "adapter_ready",
                    kind="agv",
                    msg=f"AgvApiAdapter({mode}) -> {target}",
                    agv_host=target,
                    mode=mode,
                )
                self._start_push_client(target)
                self._ensure_control_lock()
            else:
                self._slog.log(
                    "adapter_fail",
                    kind="agv",
                    level="ERROR",
                    msg=f"connect failed {target}",
                    agv_host=target,
                    mode=mode,
                )
            return ok
        except Exception as exc:  # noqa: BLE001
            self._slog.log(
                "adapter_fail",
                kind="agv",
                level="ERROR",
                msg=str(exc),
                error=str(exc),
                agv_host=target,
                mode=mode,
            )
            return False

    def _ensure_robokit(self, host: str) -> bool:
        """Backward-compatible alias for demo/mock TCP connect."""
        return self._ensure_adapter(host)

    def _ensure_control_lock(self) -> Dict[str, Any]:
        """Robokit 4005 抢占控制权 (Roboshop 手册：被抢占时无法手动/导航控制)."""
        if self._agv_adapter is None:
            self._control_locked = False
            return {"success": False, "message": "adapter missing"}
        nick = self._lock_nick
        try:
            ack = self._agv_adapter.lock(nick)
            self._control_locked = True
            self._slog.log("agv_lock_ok", kind="agv", msg=f"4005 lock nick={nick}")
            return {"success": True, "message": f"已抢占控制权 ({nick})", "ack": ack}
        except Exception as first:  # noqa: BLE001
            try:
                self._agv_adapter.unlock()
            except Exception:  # noqa: BLE001
                pass
            try:
                ack = self._agv_adapter.lock(nick)
                self._control_locked = True
                self._slog.log("agv_lock_ok", kind="agv", msg=f"4005 steal-lock nick={nick}")
                return {"success": True, "message": f"已抢占控制权 ({nick})", "ack": ack, "stole": True}
            except Exception as exc:  # noqa: BLE001
                self._control_locked = False
                self._slog.log("agv_lock_fail", kind="agv", level="WARN", msg=str(exc), error=str(exc))
                return {
                    "success": False,
                    "message": f"抢控制权失败: {exc}（请在 Roboshop 点「释放控制权」后再试）",
                    "error": str(first),
                }

    def _start_push_client(self, host: str) -> None:
        try:
            from agv_bridge.agv_adapter.push_client import PushPoseCache, RobokitPushClient
        except Exception:  # noqa: BLE001
            return
        if self._push_client is not None:
            try:
                self._push_client.stop()
            except Exception:  # noqa: BLE001
                pass
        self._push_cache = PushPoseCache()
        self._push_client = RobokitPushClient(host, self._push_cache, self.get_logger().info)
        self._push_client.start()
        if self._agv_adapter is not None and hasattr(self._agv_adapter, "set_push_cache"):
            self._agv_adapter.set_push_cache(self._push_cache)
        try:
            if self._agv_adapter is not None and hasattr(self._agv_adapter, "configure_push"):
                self._agv_adapter.configure_push(interval_ms=200)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"9300 push config failed: {exc}")

    def set_env(self, mode: str, agv_host: Optional[str] = None) -> Dict[str, Any]:
        mode = "demo"
        host = (agv_host or self._env.get("agv_host") or DEFAULT_DEMO_AGV).strip()
        self._env = {
            "mode": "demo",
            "agv_host": host,
            "use_sim_cameras": False,
            "updated_at": time.time(),
        }
        save_env_file(self._env)
        old_mode = str(self._state.get("env", {}).get("mode", ""))
        self._slog.log("env_switch", kind="system", msg=f"switch to {mode} agv={host}", mode=mode, agv_host=host)
        self._vlog.log("system", "Mode switch", {"from": old_mode, "to": mode, "agv_host": host})
        self._push_event("system", f"env -> {mode} agv={host}", event="env_switch")
        try:
            self._get_vision().reset_for_mode_change()
            self._vision_cache = {}
        except Exception:  # noqa: BLE001
            pass
        if mode in ("demo",):
            ok = self._ensure_adapter(host if mode == "demo" else None)
            if ok and self._agv_adapter is not None:
                try:
                    self._agv_adapter.lock()
                    self._slog.log("agv_lock_ok", kind="agv", msg=f"locked {self._agv_adapter.host}")
                except Exception as exc:  # noqa: BLE001
                    self._slog.log("agv_lock_fail", kind="agv", level="WARN", msg=str(exc), error=str(exc))
            pub = self._public_env()
            with self._lock:
                self._state["env"] = pub
            return {
                "success": True,
                "message": f"switched to {mode}; adapter={'ok' if ok else 'fail'}",
                "env": pub,
                "robokit_ok": ok,
            }
        if self._agv_adapter is not None:
            try:
                if getattr(self._agv_adapter, "mode", "") in ("demo", "mock"):
                    self._agv_adapter.unlock()
            except Exception:  # noqa: BLE001
                pass
            try:
                self._agv_adapter.disconnect()
            except Exception:  # noqa: BLE001
                pass
        self._init_adapter()
        pub = self._public_env()
        with self._lock:
            self._state["env"] = pub
        return {"success": True, "message": "switched to dev (sim)", "env": pub}

    def _laser_sim_stub(self) -> List[Dict[str, float]]:
        with self._lock:
            agv = self._state.get("agv") or {}
            cloud = list(self._map_cloud or (self._state.get("map") or {}).get("cloud") or [])
            obstacles = list(self._state.get("obstacles") or [])
        ax = float(agv.get("x", 0.0) or 0.0)
        ay = float(agv.get("y", 0.0) or 0.0)
        ayaw = float(agv.get("angle", 0.0) or 0.0)
        if cloud or obstacles:
            return sim_laser_from_map(ax, ay, ayaw, cloud, obstacles, beams=100, max_d=8.0)
        # fallback ring if no smap cloud yet
        import math

        pts = []
        for i in range(72):
            a = ayaw + (i / 72.0) * 2 * math.pi
            d = 1.2 + 0.3 * math.sin(i * 0.4 + time.time())
            pts.append({"x": ax + d * math.cos(a), "y": ay + d * math.sin(a)})
        return pts

    def _agv_status_tick(self) -> None:
        mode = str(self._env.get("mode", "dev"))
        if mode not in ("demo", "mock"):
            return
        if self._agv_adapter is None or not self._agv_adapter.connected:
            if not self._ensure_adapter(str(self._env.get("agv_host") or DEFAULT_DEMO_AGV)):
                return
        try:
            poll = getattr(self._agv_adapter, "poll_status", None)
            rs = poll() if callable(poll) else self._agv_adapter.poll_robot_state()
            with self._lock:
                prev = self._state.get("agv") or {}
                self._state["agv"] = rs.agv_state_dict(prev)
                self._state["updated_at"] = time.time()
            self._agv_last_seen = time.time()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"agv status poll failed: {exc}")

    def _laser_tick(self) -> None:
        if getattr(self, "_laser_busy", False):
            return
        self._laser_busy = True
        try:
            self._laser_tick_inner()
        finally:
            self._laser_busy = False

    def _laser_tick_inner(self) -> None:
        mode = self._env.get("mode", "dev")
        if mode in ("demo", "mock") and self._agv_adapter is not None and self._agv_adapter.connected:
            try:
                pose = self._agv_adapter.get_pose()
                laser = self._agv_adapter.get_laser()
                with self._lock:
                    prev = self._state.get("agv") or {}
                    self._state["agv"] = {
                        **prev,
                        "x": float(pose.x),
                        "y": float(pose.y),
                        "angle": float(pose.angle),
                        "confidence": float(pose.confidence),
                        "current_station": str(pose.current_station or prev.get("current_station") or ""),
                    }
                    ld = {
                        "ok": bool(getattr(laser, "ok", False)),
                        "source": getattr(laser, "source", "robokit_1009") or "robokit_1009",
                        "live_lidar": True,
                        "points": laser.points_as_dicts() if hasattr(laser, "points_as_dicts") else [],
                        "beam_count": int(getattr(laser, "beam_count", 0) or 0),
                        "message": getattr(laser, "message", "") or "ok",
                        "updated_at": time.time(),
                        "agv_host": self._agv_adapter.host,
                        "label": "LIVE LIDAR",
                    }
                    self._state["laser"] = ld
                    self._state["updated_at"] = time.time()
                self._agv_last_seen = time.time()
                return
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._state["laser"] = {
                        "ok": False,
                        "source": "robokit_1009",
                        "points": [],
                        "beam_count": 0,
                        "message": str(exc),
                        "updated_at": time.time(),
                    }
                self._slog.log(
                    "laser_fail",
                    kind="lidar",
                    level="WARN",
                    msg=str(exc),
                    error=str(exc),
                    agv_host=self._env.get("agv_host"),
                )
                return
        # dev: lidar from smap point cloud + sim obstacles (agv_downloaded)
        pts = self._laser_sim_stub()
        with self._lock:
            src = "smap_cloud+obstacles" if (self._state.get("map") or {}).get("cloud") else "sim_stub"
            agv = self._state.get("agv") or {}
            self._state["laser"] = {
                "ok": True,
                "source": src,
                "points": pts,
                "beam_count": len(pts),
                "message": "sim from agv_downloaded smap",
                "updated_at": time.time(),
            }
        if self._vlog.is_enabled("laser"):
            self._vlog.log("laser", "Laser scan received", {
                "point_count": len(pts),
                "robot_pose": {
                    "x": agv.get("x"),
                    "y": agv.get("y"),
                    "angle": agv.get("angle") or agv.get("yaw"),
                },
                "first_5_points": pts[:5],
                "last_5_points": pts[-5:] if pts else [],
            })

    def _camera_diag_tick(self) -> None:
        with self._lock:
            streams = {k: True for k in self._jpeg}
            info = dict(self._cam_info)
            hz = {k: float(self._hz.get(f"/camera/{k}/image_raw", 0.0)) for k in ("cam_front", "cam_left", "cam_right")}
        missing = [n for n in ("cam_front", "cam_left", "cam_right") if n not in streams]
        if missing:
            self._slog.log(
                "camera_missing_frames",
                kind="camera",
                level="WARN",
                msg=f"no jpeg/png for {missing}; need image_raw from gz_physics_proxy (dev) or camera colleague (real)",
                missing=missing,
                camera_info=info,
                hz=hz,
            )

    def diag_snapshot(self) -> Dict[str, Any]:
        """GET /api/diag/snapshot — read-only Robokit diagnostic. Does not navigate."""
        ts = time.time()
        env = self._public_env() if hasattr(self, "_public_env") else (self._state.get("env") or {})
        host = str(env.get("agv_host") or env.get("robokit_host") or "")
        adapter = self._agv_adapter
        if adapter is None or not getattr(adapter, "connected", False):
            return {
                "success": False,
                "message": "adapter not connected",
                "timestamp": ts,
                "snapshot": {
                    "timestamp": ts,
                    "robot_ip": host,
                    "diag_source": None,
                },
            }
        try:
            if not hasattr(adapter, "diagnose"):
                return {"success": False, "message": "adapter has no diagnose()", "timestamp": ts}
            state = adapter.diagnose()
            diag = state.agv_state_dict()
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "message": f"diagnose failed: {exc}", "timestamp": ts}

        reloc = diag.get("reloc_status")
        if reloc is None and hasattr(adapter, "get_reloc_status"):
            try:
                reloc = (adapter.get_reloc_status() or {}).get("reloc_status")
            except Exception:  # noqa: BLE001
                reloc = None
        loadmap = diag.get("loadmap_status")
        if loadmap is None and hasattr(adapter, "get_map_load_status"):
            try:
                loadmap = (adapter.get_map_load_status() or {}).get("loadmap_status")
            except Exception:  # noqa: BLE001
                loadmap = None

        snapshot = {
            "timestamp": ts,
            "robot_ip": host or str(getattr(adapter, "host", "") or ""),
            "diag_source": diag.get("diag_source"),
            "navigation": {
                "task_status": diag.get("task_status"),
                "task_type": diag.get("task_type"),
                "target_id": diag.get("target_id"),
                "finished_path": diag.get("finished_path"),
                "unfinished_path": diag.get("unfinished_path"),
                "move_status_info": diag.get("move_status_info"),
            },
            "motion": {
                "vx": diag.get("vx"),
                "vy": diag.get("vy"),
                "w": diag.get("w"),
                "r_vx": diag.get("r_vx"),
                "r_vy": diag.get("r_vy"),
                "r_w": diag.get("r_w"),
                "is_stop": diag.get("is_stop"),
            },
            "control": {"current_lock": diag.get("current_lock")},
            "dispatch": {
                "dispatch_mode": diag.get("dispatch_mode"),
                "connect_fleet": diag.get("connect_fleet"),
            },
            "safety": {
                "block_reason": diag.get("block_reason"),
                "brake": diag.get("brake"),
                "emergency": diag.get("emergency"),
                "soft_emc": diag.get("soft_emc"),
                "driver_emc": diag.get("driver_emc"),
                "manual_charge": diag.get("manual_charge"),
            },
            "motor": diag.get("motor_info"),
            "alarms": {
                "errors": diag.get("errors"),
                "fatals": diag.get("fatals"),
                "warnings": diag.get("warnings"),
            },
            "meta": {
                "current_map": diag.get("current_map"),
                "vehicle_id": diag.get("vehicle_id"),
                "reloc_status": reloc,
                "loadmap_status": loadmap,
                "confidence": diag.get("confidence"),
                "current_station": diag.get("current_station"),
            },
        }
        return {"success": True, "snapshot": snapshot}

    def camera_diag(self) -> Dict[str, Any]:
        with self._lock:
            streams = {k: (_image_ctype(v), len(v)) for k, v in self._jpeg.items()}
            info = dict(self._cam_info)
            hz = dict(self._hz)
            mode = (self._state.get("env") or {}).get("mode", "dev")
        tips = []
        if mode == "dev":
            tips.append(
                "开发环境：相机2 正式通路为 ROS2（docs/25）：Xavier cam_front_ros_node → NUC camera_ingress。"
            )
            tips.append(
                "订 /camera/cam_front/image_raw/compressed + /heartbeat；HTTP :8080 仅为 legacy。"
            )
            tips.append("cam_left/cam_right 仍可由 gz_physics_proxy 合成。")
        else:
            tips.append("Demo 实车：AGV 走 Robokit；工业相机仍依赖同事按 docs/25 发 ROS2。")
            tips.append("相机同事需对 cam_left/right 发布同名 compressed/heartbeat（或 image_raw）。")
        cf = info.get("cam_front") or {}
        if not streams.get("cam_front") or not cf.get("connected"):
            tips.append(
                "cam_front 无帧：检查 ROS_DOMAIN_ID=30；ros2 topic echo /camera/cam_front/heartbeat；"
                "Xavier eth1 Link+Basler；同事跑 cam_front_ros_node。"
            )
        if not streams:
            tips.append("当前无帧：检查 ros2 topic hz /camera/cam_front/image_raw；浏览器请用 /api/diag/cameras。")
        return {
            "env_mode": mode,
            "frames": {k: {"ctype": c, "bytes": n} for k, (c, n) in streams.items()},
            "camera_info": info,
            "topic_hz": {
                t: hz.get(t, 0.0)
                for t in (
                    "/camera/cam_front/image_raw",
                    "/camera/cam_front/image_raw/compressed",
                    "/camera/cam_front/heartbeat",
                    "/camera/cam_left/image_raw",
                    "/camera/cam_right/image_raw",
                    "/camera/cam_front/info",
                )
            },
            "required_nodes_dev": [
                "gz_physics_proxy",
                "camera_gazebo_qr",
                "dashboard_node",
                "camera_ingress_node",
            ],
            "required_topics": [
                "/camera/cam_front/image_raw/compressed",
                "/camera/cam_front/heartbeat",
                "/camera/cam_front/image_raw",
                "/camera/cam_left/image_raw",
                "/camera/cam_right/image_raw",
                "/camera/cam_front/info",
            ],
            "xavier_cam_front": {
                "host": "192.168.0.225",
                "domain_id": 30,
                "transport_expected": "xavier_ros2",
                "contract": "docs/25",
                "legacy_http": "http://192.168.0.225:8080/stream",
            },
            "colleague_todo": {
                "cam_front": "cam_front_ros_node: compressed+heartbeat+status; Humble Docker; docs/25",
                "cam_left_right": "same ROS2 contract or gazebo until wired",
                "names": ["cam_front", "cam_left", "cam_right"],
            },
            "tips": tips,
            "system_log": str(self._slog.path),
        }

    def navigate(
        self,
        target: str,
        wait: bool = True,
        reloc_threshold: float = 0.35,
        force_reloc: bool = False,
        reloc_length: float = 0.5,
        reloc_timeout: float = 25.0,
        skip_reloc: bool = True,
    ) -> Dict[str, Any]:
        self._push_event("cmd", f"navigate -> {target}")
        self._slog.log("navigate_req", kind="agv", msg=f"navigate {target}", target=target, mode=self._env.get("mode"))
        self._vlog.log("nav", "Navigate request", {"target": target, "kwargs": {
            "wait": wait, "reloc_threshold": reloc_threshold, "force_reloc": force_reloc,
            "skip_reloc": skip_reloc,
        }})
        if self._env.get("mode") != "demo":
            with self._lock:
                agv = self._state.get("agv") or {}
                stations = self._state.get("stations") or {}
                obstacles = list(self._state.get("obstacles") or [])
                # if pose not yet published, use LM1 / first station as start
                if not agv:
                    st0 = stations.get("LM1") or (next(iter(stations.values()), None) if stations else None)
                    agv = {"x": (st0 or {}).get("x", 0.0), "y": (st0 or {}).get("y", 0.0)}
            goal = stations.get(target)
            if goal and obstacles:
                hit = segment_hits_obstacle(
                    float(agv.get("x", 0.0) or 0.0),
                    float(agv.get("y", 0.0) or 0.0),
                    float(goal["x"]),
                    float(goal["y"]),
                    obstacles,
                )
                if hit:
                    msg = f"blocked by obstacle {hit} (sim); clear or move obstacle"
                    self._slog.log("navigate_blocked", kind="agv", level="WARN", msg=msg, obstacle=hit)
                    return {"success": False, "message": msg, "final_status": 5, "mode": "dev"}
        if self._env.get("mode") in ("demo", "mock"):
            nav_mode = str(self._env.get("mode", "demo"))
            reloc_log: Dict[str, Any] = {}
            if not self._ensure_adapter(str(self._env.get("agv_host") or DEFAULT_DEMO_AGV)):
                return {"success": False, "message": "demo/mock adapter unavailable"}
            try:
                assert self._agv_adapter is not None
                agv_now = self._state.get("agv") or {}
                if agv_now.get("emergency") or agv_now.get("soft_emc"):
                    return {
                        "success": False,
                        "message": "AGV 急停中，无法导航（请先复位急停）",
                        "mode": nav_mode,
                        "emergency": True,
                    }
                lock_out = {"success": True, "skipped": True, "message": "control lock switch off"}
                if self._control_wanted:
                    lock_out = self._ensure_control_lock()
                    if not lock_out.get("success"):
                        return {
                            "success": False,
                            "message": lock_out.get("message") or "未拿到控制权",
                            "mode": nav_mode,
                            "lock": lock_out,
                        }
                try:
                    if hasattr(self._agv_adapter, "get_reloc_status"):
                        rs = self._agv_adapter.get_reloc_status()
                        rsv = int((rs or {}).get("reloc_status", 1) or 1)
                        reloc_log["reloc_status_1021"] = rsv
                        if rsv == 3:
                            self.get_logger().info("[navigate:demo] 1021 status=3 → 2003 confirm_loc")
                            cli = getattr(self._agv_adapter, "client", None)
                            if cli is not None and hasattr(cli, "confirm_loc"):
                                cli.confirm_loc(allow_benign_error=True)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"1021/2003 precheck skipped: {exc}")
                try:
                    nav_now = self._agv_adapter.get_task_status()
                    running = int(getattr(nav_now, "task_status", 0) or 0) == 2
                    same = str(getattr(nav_now, "target_id", "") or "") == str(target)
                    spd = self._agv_adapter.get_speed() if hasattr(self._agv_adapter, "get_speed") else {}
                    stopped = bool((spd or {}).get("is_stop", False)) or abs(float((spd or {}).get("vx", 0.0) or 0.0)) < 0.02
                    if running and same and stopped:
                        self.get_logger().info("[navigate:demo] 3002 resume then 3003 if still stopped")
                        if hasattr(self._agv_adapter, "resume_navigation"):
                            self._agv_adapter.resume_navigation()
                        time.sleep(0.5)
                        spd2 = self._agv_adapter.get_speed() if hasattr(self._agv_adapter, "get_speed") else {}
                        vx2 = abs(float((spd2 or {}).get("vx", 0.0) or 0.0))
                        still = bool((spd2 or {}).get("is_stop", False)) or vx2 < 0.02
                        if not still:
                            return {
                                "success": True,
                                "final_status": 2,
                                "message": f"已 3002 继续去 {target}，vx≈{vx2:.3f}",
                                "mode": nav_mode,
                                "resumed": True,
                            }
                        self.get_logger().info("[navigate:demo] still stopped after 3002, 3003 cancel then re-3051")
                        self._agv_adapter.cancel_navigation()
                        time.sleep(0.5)
                    elif running:
                        self.get_logger().info("[navigate:demo] cancel running task before 3051")
                        self._agv_adapter.cancel_navigation()
                        time.sleep(0.4)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"pre-nav cancel skipped: {exc}")
                if skip_reloc:
                    self.get_logger().info(
                        f"[navigate:{nav_mode}] skip_reloc=True — bypassing relocation step"
                        f" (target={target})"
                    )
                    reloc_log = {"skipped": True, "reason": "skip_reloc flag"}
                else:
                    self.get_logger().info(
                        f"[navigate:{nav_mode}] PRE-CHECK relocation decision START"
                        f" — target={target} reloc_threshold={reloc_threshold}"
                        f" force_reloc={force_reloc}"
                    )
                    pose_pre = self._agv_adapter.get_pose()
                    conf_pre = float(pose_pre.confidence or 0.0)
                    station_pre = str(pose_pre.current_station or "")
                    x_pre = float(pose_pre.x or 0.0)
                    y_pre = float(pose_pre.y or 0.0)
                    angle_pre = float(pose_pre.angle or 0.0)
                    self.get_logger().info(
                        f"[navigate:demo] Step1/pose-read OK"
                        f" — confidence={conf_pre:.4f} (threshold={reloc_threshold:.4f})"
                        f" current_station={station_pre!r}"
                        f" pose=({x_pre:.3f},{y_pre:.3f},{angle_pre:.3f}rad)"
                        f" force_reloc={force_reloc}"
                    )
                    reloc_log.update({
                        "confidence_pre": conf_pre,
                        "current_station_pre": station_pre,
                        "threshold": reloc_threshold,
                        "force_reloc": force_reloc,
                        "pose_pre": {"x": x_pre, "y": y_pre, "angle": angle_pre},
                    })
                    # --- Step 2: relocate decision
                    low_conf = conf_pre < reloc_threshold
                    should_reloc = force_reloc or low_conf
                    self.get_logger().info(
                        f"[navigate:demo] Step2/decision"
                        f" — low_confidence={conf_pre:.4f}<{reloc_threshold:.4f}={low_conf}"
                        f" current_station={station_pre!r}"
                        f" force_reloc={force_reloc}"
                        f" => SHOULD_RELOCATE={should_reloc}"
                    )
                    reloc_log["low_confidence"] = low_conf
                    reloc_log["no_current_station"] = not station_pre
                    reloc_log["decision"] = "relocate" if should_reloc else "skip relocate"

                    if should_reloc:
                        # --- Step 3: relocate (use current pose as seed; conservative)
                        # If we're very close to a station (current_station non-empty),
                        # adding a small delta in angle can help match; otherwise we
                        # stick with current pose. We intentionally do NOT assume any
                        # station coordinates (Roboshop is authority).
                        self.get_logger().info(
                            f"[navigate:demo] Step3/relocate TRIGGERED"
                            f" — 2002 relocate(x={x_pre:.3f}, y={y_pre:.3f},"
                            f" angle={angle_pre:.3f}, length={reloc_length})"
                            f" (timeout={reloc_timeout}s, then wait_reloc_done auto 2003 confirm if old fw)"
                        )
                        reloc_start = time.time()
                        self._agv_adapter.relocate(
                            x=x_pre,
                            y=y_pre,
                            angle=angle_pre,
                            length=float(reloc_length),
                        )
                        self.get_logger().info(
                            "[navigate:demo] Step3a/relocate 2002 sent, ACK received —"
                            " now wait_reloc_done polling 1021..."
                        )
                        reloc_final_status, reloc_last_1021 = self._agv_adapter.wait_reloc_done(
                            timeout_sec=float(reloc_timeout),
                            poll_sec=0.4,
                            auto_confirm_if_needed=True,
                        )
                        reloc_elapsed = time.time() - reloc_start
                        pose_post = self._agv_adapter.get_pose()
                        conf_post = float(pose_post.confidence or 0.0)
                        station_post = str(pose_post.current_station or "")
                        x_post = float(pose_post.x or 0.0)
                        y_post = float(pose_post.y or 0.0)
                        angle_post = float(pose_post.angle or 0.0)
                        self.get_logger().info(
                            f"[navigate:demo] Step3b/relocate DONE (elapsed={reloc_elapsed:.2f}s)"
                            f" — 1021 final_reloc_status={reloc_final_status}"
                            f"  (1=SUCCESS, 3=COMPLETED_need_confirm)"
                            f" | post: confidence={conf_post:.4f} (+{conf_post - conf_pre:+.4f})"
                            f" current_station={station_post!r}"
                            f" pose=({x_post:.3f},{y_post:.3f},{angle_post:.3f})"
                        )
                        reloc_log.update({
                            "reloc_triggered": True,
                            "reloc_elapsed_s": round(reloc_elapsed, 3),
                            "reloc_final_status": reloc_final_status,
                            "reloc_last_1021": reloc_last_1021,
                            "confidence_post": conf_post,
                            "current_station_post": station_post,
                            "pose_post": {"x": x_post, "y": y_post, "angle": angle_post},
                            "confidence_delta": round(conf_post - conf_pre, 6),
                        })
                        # 3c: warn if post-reloc confidence still low
                        if conf_post < reloc_threshold:
                            self.get_logger().warning(
                                f"[navigate:demo] Step3c POST-RELOC LOW CONFIDENCE"
                                f" — post={conf_post:.4f} < threshold={reloc_threshold:.4f}"
                                f" even after relocate! Proceeding to 3051 anyway"
                                f" (caller can cancel at any time via /api/cancel)."
                            )
                            reloc_log["post_reloc_still_low"] = True
                    else:
                        self.get_logger().info(
                            "[navigate:demo] Step3/relocate SKIPPED"
                            f" — pre-conf {conf_pre:.4f} >= threshold {reloc_threshold:.4f}"
                            f" AND station={station_pre!r}; going straight to 3051 goto_station."
                        )
                        reloc_log["reloc_triggered"] = False

                # --- Step 4: goto_station (3051)
                self.get_logger().info(
                    f"[navigate:demo] Step4/3051 goto_station(target_id={target!r},"
                    f" source_id=<omitted>) — wait={wait}"
                    f" (real-vehicle tested 2026-07-28: source_id field rejected,"
                    f" only {{\"id\":<target>}} works)"
                )
                # NOTE: source_id intentionally NOT sent.
                # Real-vehicle test on 192.168.18.198 (AGV colleague, 2026-07-28)
                # proved 3051 rejects any source_id value (incl. "SELF_POSITION").
                # Only {"id":"<target>"} succeeds; vehicle auto-plans via
                # intermediate stations per the colleague's note:
                # "选择经过地图中存在的路线和中间站点, 抵达LM3".
                ack = self._agv_adapter.goto_station(target_id=target, max_speed=0.6)
                try:
                    nav_after = self._agv_adapter.get_task_status()
                    after = {
                        "task_status": int(getattr(nav_after, "task_status", 0) or 0),
                        "target_id": str(getattr(nav_after, "target_id", "") or ""),
                        "unfinished_path": list(getattr(nav_after, "unfinished_path", None) or []),
                    }
                except Exception:  # noqa: BLE001
                    after = {}
                self.get_logger().info(
                    "[navigate:demo] Step4a/3051 ACK received."
                    + (" Now polling 1020 until nav done (120s timeout)." if wait else " Fire-and-forget, returning immediately.")
                    + f" ack={ack} after={after}"
                )
                if wait:
                    ok, last = self._agv_adapter.wait_navigation_done(timeout_sec=120.0)
                    final_status = int(last.task_status if ok else (last.task_status or 5))
                    out = {
                        "success": ok,
                        "final_status": final_status,
                        "message": "arrived" if ok else "nav failed",
                        "task_id": "",
                        "mode": nav_mode,
                        "reloc": reloc_log,
                        "ack": ack,
                        "task_after": after,
                        "lock": lock_out,
                    }
                else:
                    time.sleep(0.45)
                    try:
                        nav2 = self._agv_adapter.get_task_status()
                        after = {
                            "task_status": int(getattr(nav2, "task_status", 0) or 0),
                            "target_id": str(getattr(nav2, "target_id", "") or ""),
                            "unfinished_path": list(getattr(nav2, "unfinished_path", None) or []),
                        }
                        spd = self._agv_adapter.get_speed() if hasattr(self._agv_adapter, "get_speed") else {}
                        vx = abs(float((spd or {}).get("vx", 0.0) or 0.0))
                    except Exception:  # noqa: BLE001
                        vx = 0.0
                    ts = int(after.get("task_status") or 2)
                    if ts == 2 and vx < 0.02 and hasattr(self._agv_adapter, "resume_navigation"):
                        try:
                            self._agv_adapter.resume_navigation()
                            time.sleep(0.4)
                            spd = self._agv_adapter.get_speed() if hasattr(self._agv_adapter, "get_speed") else {}
                            vx = abs(float((spd or {}).get("vx", 0.0) or 0.0))
                        except Exception as exc:  # noqa: BLE001
                            self.get_logger().warn(f"3002 resume after 3051: {exc}")
                    if ts == 5:
                        msg = (
                            f"3051→{target} 任务状态=5 失败。"
                            "常见原因：站点没在车载地图(1301)、或没有连到现有 LM 的路径。"
                            "新增点请选「连接到」并保存成功后再去。"
                        )
                    elif ts == 2 and vx < 0.02:
                        msg = (
                            f"3051已受理→{target} 状态=2，已发 3002 继续，车速仍≈0。"
                            f"路径={after.get('unfinished_path')}。"
                            "规划成功但底盘未动：看急停/Roboshop暂停/是否被调度拦住。"
                        )
                    else:
                        msg = f"已受理 3051→{target} 任务状态={ts}"
                    out = {
                        "success": ts in (2, 4),
                        "final_status": ts,
                        "message": msg,
                        "task_id": "",
                        "mode": nav_mode,
                        "reloc": reloc_log,
                        "ack": ack,
                        "task_after": after,
                        "lock": lock_out,
                    }
                self.get_logger().info(
                    f"[navigate:demo] DONE target={target!r}"
                    f" success={out['success']} final_status={out['final_status']}"
                    f" message={out['message']!r}"
                )
                self._slog.log("navigate_res", kind="agv", msg=str(out), **{k: out[k] for k in ("success", "message")})
                return out
            except Exception as exc:  # noqa: BLE001
                self._slog.log("navigate_fail", kind="agv", level="ERROR", msg=str(exc), error=str(exc))
                return {"success": False, "message": str(exc), "mode": nav_mode, "reloc": reloc_log}
        if not self._cli_nav.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "agv/navigate unavailable"}
        req = AgvNavigate.Request()
        req.target_id = target
        req.source_id = "SELF_POSITION"
        req.wait_until_done = wait
        req.timeout_sec = 120.0
        future = self._cli_nav.call_async(req)
        if not self._wait_future(future, 130):
            return {"success": False, "message": "timeout"}
        res = future.result()
        out = {
            "success": bool(res.success),
            "final_status": int(res.final_status),
            "message": res.message,
            "task_id": res.task_id,
            "mode": "dev",
        }
        self._push_event("cmd", f"navigate result: {out}")
        return out

    def cancel(self) -> Dict[str, Any]:
        self._push_event("cmd", "cancel navigation")
        if self._env.get("mode") in ("demo", "mock") and self._agv_adapter is not None:
            try:
                self._agv_adapter.cancel_navigation()
                return {"success": True, "message": "canceled", "mode": self._env.get("mode", "demo")}
            except Exception as exc:  # noqa: BLE001
                return {"success": False, "message": str(exc), "mode": self._env.get("mode", "demo")}
        if not self._cli_cancel.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "agv/cancel unavailable"}
        future = self._cli_cancel.call_async(AgvCancel.Request())
        if not self._wait_future(future, 10):
            return {"success": False, "message": "timeout"}
        res = future.result()
        return {"success": bool(res.success), "message": res.message}

    def arm_sequence(self, arm: str, sequence: str) -> Dict[str, Any]:
        self._push_event("cmd", f"arm {arm} sequence {sequence}")
        if not self._cli_arm.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "arm service unavailable"}
        req = ArmExecuteSequence.Request()
        req.arm_name = arm
        req.sequence_name = sequence
        req.wait = True
        future = self._cli_arm.call_async(req)
        if not self._wait_future(future, 60):
            return {"success": False, "message": "timeout"}
        res = future.result()
        return {
            "success": bool(res.success),
            "message": res.message,
            "steps_done": int(res.steps_done),
        }

    def detect_qr(self, camera: str = "cam_front") -> Dict[str, Any]:
        self._push_event("cmd", f"detect_qr {camera}")
        if not self._cli_qr.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "detect_qr unavailable"}
        req = DetectQr.Request()
        req.camera_name = camera
        req.code_type = "AprilTag"
        future = self._cli_qr.call_async(req)
        if not self._wait_future(future, 10):
            return {"success": False, "message": "timeout"}
        res = future.result()
        dets = [
            {
                "code_type": d.code_type,
                "payload": d.payload,
                "x": float(d.x),
                "y": float(d.y),
                "z": float(d.z),
                "yaw": float(d.yaw),
                "confidence": float(d.confidence),
            }
            for d in res.detections
        ]
        return {"success": bool(res.success), "message": res.message, "detections": dets}

    def identify_face(self, camera: str = "cam_front", min_confidence: float = 0.5) -> Dict[str, Any]:
        self._push_event("face", f"identify_face {camera}", event="identify_req")
        if camera not in ("cam_front", "cam_left", "cam_right"):
            return {"success": False, "message": "bad camera_name", "camera_name": camera}
        if not self._cli_face_id.wait_for_service(timeout_sec=2.0):
            out = {"success": False, "message": "face/identify_face unavailable", "camera_name": camera}
            self._push_event("face", out["message"], level="WARN", event="identify_fail")
            return out
        req = IdentifyFace.Request()
        req.camera_name = camera
        req.min_confidence = float(min_confidence)
        future = self._cli_face_id.call_async(req)
        if not self._wait_future(future, 15):
            out = {"success": False, "message": "timeout", "camera_name": camera}
            self._push_event("face", "identify timeout", level="WARN", event="identify_fail")
            return out
        res = future.result()
        out = {
            "success": bool(res.success),
            "message": res.message,
            "person_id": res.person_id,
            "person_name": res.person_name,
            "confidence": float(res.confidence),
            "camera_name": camera,
            "detection": {
                "person_id": res.detection.person_id,
                "person_name": res.detection.person_name,
                "confidence": float(res.detection.confidence),
                "bbox_xywh": [float(x) for x in res.detection.bbox_xywh],
                "track_id": res.detection.track_id,
                "camera_name": res.detection.camera_name or camera,
            }
            if res.success
            else None,
        }
        with self._lock:
            face = self._state["face"]
            face["last_identify"] = dict(out)
            face["updated_at"] = time.time()
            if out["success"]:
                face["ok"] = True
                face["person_id"] = out["person_id"]
                face["person_name"] = out["person_name"]
                face["confidence"] = out["confidence"]
                face["last_event"] = "identify_ok"
                face["last_error"] = ""
                if out.get("detection"):
                    face["bbox_xywh"] = out["detection"]["bbox_xywh"]
                    face["track_id"] = out["detection"]["track_id"]
            else:
                face["last_event"] = "identify_fail"
                face["last_error"] = out["message"]
        self._push_event(
            "face",
            f"identify {out['message']} id={out.get('person_id','')}",
            level="INFO" if out["success"] else "WARN",
            event="identify_ok" if out["success"] else "identify_fail",
        )
        return out

    def face_logs(self, limit: int = 20, level: str = "", event: str = "") -> Dict[str, Any]:
        limit = max(1, min(int(limit or 20), 200))
        rows: List[Dict[str, Any]] = []
        log_dir = Path(self.get_parameter("face_log_dir").get_parameter_value().string_value)
        day = time.strftime("%Y%m%d")
        path = log_dir / f"face_bridge_{day}.jsonl"
        if path.is_file():
            try:
                lines = path.read_text(encoding="utf-8").splitlines()
                for line in reversed(lines):
                    try:
                        rec = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    rec["kind"] = "face"
                    if level and str(rec.get("level", "")).upper() != level.upper():
                        continue
                    if event and str(rec.get("event", "")) != event:
                        continue
                    rows.append(rec)
                    if len(rows) >= limit:
                        break
            except Exception as exc:  # noqa: BLE001
                return {"logs": [], "error": str(exc), "path": str(path)}
        # also merge in-memory face events
        with self._lock:
            for e in self._events:
                if e.get("kind") != "face":
                    continue
                if level and str(e.get("level", "")).upper() != level.upper():
                    continue
                if event and str(e.get("event", "")) != event:
                    continue
                rows.append(
                    {
                        "ts": e.get("t"),
                        "level": e.get("level", "INFO"),
                        "event": e.get("event", "face"),
                        "trace_id": e.get("trace_id", ""),
                        "msg": e.get("message", ""),
                        "kind": "face",
                        "source": "dashboard_events",
                    }
                )
        # de-dup rough by keeping first `limit`
        return {"logs": rows[:limit], "path": str(path)}

    def destroy_node(self) -> bool:
        for httpd in (getattr(self, "_httpd_demo", None), getattr(self, "_httpd_debug", None)):
            try:
                if httpd:
                    httpd.shutdown()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()

    def _make_handler(self, mode: str = "demo"):
        node = self
        www = self._www_demo if mode == "demo" else self._www_debug

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args) -> None:  # noqa: A003
                return

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _json(self, code: int, obj: Any) -> None:
                self._send(
                    code,
                    json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                    "application/json; charset=utf-8",
                )

            def do_OPTIONS(self) -> None:  # noqa: N802
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                q = parse_qs(parsed.query)
                if path == "/api/jason/camera/status":
                    try:
                        st = node._vision_cache.get("wrist_camera")
                        if st:
                            out = dict(st)
                            out["camera_id"] = "wrist_camera"
                            out["source"] = "REAL"
                            out["model"] = "acA2500-14gc"
                            out["device_user_id"] = "106611-18"
                            out["topic"] = out.get("ros_image_topic") or "/my_camera/pylon_ros2_camera_node/image_raw"
                            dets = out.get("last_detections") or []
                            out["last_qr_result"] = dets[0]["detected_id"] if dets and out.get("qr_recognition") == "RUNNING" else "--"
                            self._json(200, out)
                        else:
                            self._json(200, node._get_vision().jason_status())
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/jason/camera/snapshot":
                    t0 = time.time()
                    jpg = node._get_vision().jason_snapshot()
                    ms = int((time.time() - t0) * 1000)
                    if not jpg:
                        node._vlog.log_request("camera", "GET", "/api/jason/camera/snapshot", 404, ms)
                        self._json(404, {"error": "no live frame", "stale": True})
                        return
                    node._vlog.log_request(
                        "camera", "GET", "/api/jason/camera/snapshot", 200, ms,
                        response_body=f"JPEG {len(jpg)} bytes",
                    )
                    self._send(200, jpg, _image_ctype(jpg))
                    return
                if path == "/api/vision/floor_qr/status":
                    try:
                        cached = node._vision_cache.get("floor_qr_scanner")
                        self._json(200, cached if cached else node._get_vision().floor_qr_status())
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/vision/floor_qr/localization":
                    try:
                        cached = node._vision_cache.get("floor_qr_localization")
                        self._json(200, cached if cached else node._get_vision().floor_qr_localization())
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/vision/wrist_camera/status":
                    try:
                        cached = node._vision_cache.get("wrist_camera")
                        self._json(200, cached if cached else node._get_vision().wrist_status())
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/vision/wrist_camera/snapshot":
                    t0 = time.time()
                    now = time.time()
                    with node._wrist_snap_lock:
                        ts, cached = node._wrist_snap_cached
                        if cached and (now - ts) < 0.15:
                            jpg = cached
                        else:
                            jpg = node._get_vision().wrist_snapshot() or b""
                            node._wrist_snap_cached = (now, jpg) if jpg else (0.0, b"")
                    ms = int((time.time() - t0) * 1000)
                    if not jpg:
                        node._vlog.log_request("camera", "GET", "/api/vision/wrist_camera/snapshot", 404, ms)
                        self._json(404, {"error": "no live frame", "stale": True})
                        return
                    node._vlog.log_request(
                        "camera", "GET", "/api/vision/wrist_camera/snapshot", 200, ms,
                        response_body=f"JPEG {len(jpg)} bytes",
                    )
                    self._send(200, jpg, _image_ctype(jpg))
                    return
                if path == "/api/vision/leo_face/status":
                    self._json(200, node._get_vision().leo_status())
                    return
                if path == "/api/stack/status":
                    self._json(200, node._stack_supervisor.status())
                    return
                if path == "/api/arm/status":
                    try:
                        cached = node._arm_cache
                        st = cached if cached else node._get_arm().status()
                        st = dict(st)
                        st.setdefault("uptime", int(time.time() - node._t0))
                        self._json(200, st)
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/arm/pose":
                    try:
                        self._json(200, node._get_arm().pose())
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/arm/gripper":
                    try:
                        self._json(200, node._get_arm().gripper_get())
                    except Exception as exc:  # noqa: BLE001
                        self._json(500, {"error": str(exc)})
                    return
                if path == "/api/maps/robot":
                    self._json(200, node.sync_map_from_robot())
                    return
                if path == "/api/config/devices":
                    self._json(200, {"success": True, "devices": node._device_cfg.public_dict()})
                    return
                if path == "/api/jason/camera/stream":
                    self._jason_mjpeg()
                    return
                if path == "/api/status":
                    st = node.snapshot()
                    self._json(200, {"success": True, "agv_link": st.get("agv_link"), "route_task": st.get("route_task")})
                    return
                if path == "/api/stations":
                    self._json(200, node.list_stations_api())
                    return
                if path == "/api/route/status":
                    self._json(200, node.get_route_status())
                    return
                if path in ("/", "/index.html"):
                    idx = www / "index.html"
                    if not idx.is_file() and mode == "debug":
                        idx = node._www_demo / "index.html"
                    self._send(200, idx.read_bytes(), "text/html; charset=utf-8")
                    return
                if path in ("/camera", "/camera.html"):
                    cam = www / "camera.html"
                    if cam.is_file():
                        self._send(200, cam.read_bytes(), "text/html; charset=utf-8")
                        return
                if path == "/api/heartbeat":
                    self._json(200, node.heartbeat())
                    return
                if path == "/api/state":
                    t0 = time.time()
                    data = node.snapshot()
                    ms = int((time.time() - t0) * 1000)
                    node._vlog.log_request("agv", "GET", "/api/state", 200, ms)
                    self._json(200, data)
                    return
                if path == "/api/debug":
                    self._json(200, node.debug_snapshot())
                    return
                if path == "/api/face/status":
                    st = node.snapshot()
                    self._json(200, st.get("face") or {})
                    return
                if path == "/api/face/logs":
                    limit = int((q.get("limit") or ["20"])[0])
                    level = (q.get("level") or [""])[0]
                    event = (q.get("event") or [""])[0]
                    self._json(200, node.face_logs(limit=limit, level=level, event=event))
                    return
                if path in ("/api/system/logs", "/api/logs"):
                    limit = int((q.get("limit") or ["100"])[0])
                    kind = (q.get("kind") or [""])[0]
                    level = (q.get("level") or [""])[0]
                    event = (q.get("event") or [""])[0]
                    contains = (q.get("q") or [""])[0]
                    sys_logs = node._slog.query(
                        limit=limit, kind=kind, level=level, event=event, contains=contains
                    )
                    st = node.snapshot()
                    ev = list(st.get("events") or [])
                    if kind:
                        ev = [e for e in ev if e.get("kind") == kind]
                    if level:
                        ev = [e for e in ev if str(e.get("level", "")).upper() == level.upper()]
                    if event:
                        ev = [e for e in ev if e.get("event") == event]
                    self._json(
                        200,
                        {
                            "logs": sys_logs,
                            "events": ev[:limit],
                            "path": str(node._slog.path),
                        },
                    )
                    return
                if path == "/api/log/status":
                    self._json(200, node.verbose_log_status())
                    return
                if path == "/api/log/recent":
                    module = (q.get("module") or ["info"])[0]
                    lines = int((q.get("lines") or ["50"])[0])
                    self._json(200, node.verbose_log_recent(module, lines))
                    return
                if path == "/api/diag/cameras":
                    self._json(200, node.camera_diag())
                    return
                if path == "/api/diag/snapshot":
                    self._json(200, node.diag_snapshot())
                    return
                if path == "/api/sim/maps":
                    self._json(200, node.list_maps())
                    return
                if path == "/api/sim/obstacles":
                    self._json(200, node.list_obstacles())
                    return
                if path in ("/api/route", "/api/sim/route"):
                    self._json(200, {"success": True, "route": (node.snapshot().get("route") or {})})
                    return
                if path == "/api/env":
                    self._json(200, {"env": (node.snapshot().get("env") or {}), "success": True})
                    return
                if path == "/api/version":
                    st = node.snapshot()
                    self._json(
                        200,
                        {
                            "version": VERSION,
                            "project": "delivery-ros2",
                            "stack": "gazebo-v02",
                            "freeze": "v0.50-frozen",
                            "ports": node._state.get("ports"),
                            "ui": mode,
                            "web_face_api": WEB_FACE_API,
                            "env": st.get("env"),
                            "stack": node._stack_supervisor.status(),
                        },
                    )
                    return
                if path == "/api/health":
                    st = node.snapshot()
                    self._json(
                        200,
                        {
                            "ok": True,
                            "version": VERSION,
                            "listen_demo": "0.0.0.0:19999",
                            "listen_debug": "0.0.0.0:1999",
                            "agv_online": st.get("agv") is not None,
                            "nodes_health": st.get("nodes_health"),
                            "uptime_sec": st.get("uptime_sec"),
                            "ui": mode,
                            "web_face_api": WEB_FACE_API,
                            "env": st.get("env"),
                            "laser_ok": (st.get("laser") or {}).get("ok"),
                            "face": {
                                "ok": (st.get("face") or {}).get("ok"),
                                "connected": (st.get("face") or {}).get("connected"),
                                "backend": (st.get("face") or {}).get("backend"),
                            },
                        },
                    )
                    return
                if path.startswith("/api/camera/") and path.endswith("/stream"):
                    name = path.split("/")[3]
                    node._slog.log("camera_stream_open", kind="camera", level="DEBUG", msg=f"stream {name}", camera=name)
                    self._mjpeg(name)
                    return
                if path.startswith("/api/camera/") and path.endswith("/snapshot"):
                    name = path.split("/")[3]
                    jpg = node.latest_jpeg(name)
                    if not jpg:
                        node._slog.log(
                            "camera_snapshot_empty",
                            kind="camera",
                            level="WARN",
                            msg=f"snapshot 404 {name}",
                            camera=name,
                        )
                        self._json(404, {"error": "no frame", "camera": name, "hint": "see /api/diag/cameras"})
                        return
                    self._send(200, jpg, _image_ctype(jpg))
                    return
                rel = path.lstrip("/")
                if ".." in rel.split("/"):
                    self._json(404, {"error": "not found"})
                    return
                f = www / rel
                if f.is_file():
                    ctype = "text/plain"
                    if f.suffix == ".js":
                        ctype = "application/javascript"
                    elif f.suffix == ".css":
                        ctype = "text/css"
                    self._send(200, f.read_bytes(), ctype)
                    return
                self._json(404, {"error": "not found"})

            def _mjpeg(self, name: str) -> None:
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        jpg = node.latest_jpeg(name)
                        if jpg:
                            ctype = _image_ctype(jpg)
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(f"Content-Type: {ctype}\r\n".encode())
                            self.wfile.write(
                                f"Content-Length: {len(jpg)}\r\n\r\n".encode()
                            )
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.2)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return


            def _jason_mjpeg(self) -> None:
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                try:
                    while True:
                        jpg = node._get_vision().wrist_snapshot()
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                        time.sleep(0.12)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return

            def do_POST(self) -> None:  # noqa: N802
                from urllib.parse import urlparse

                path = urlparse(self.path).path
                length = int(self.headers.get("Content-Length", "0") or 0)
                raw = self.rfile.read(length) if length else b"{}"
                try:
                    payload = json.loads(raw.decode("utf-8") or "{}")
                except json.JSONDecodeError:
                    self._json(400, {"success": False, "message": "bad json"})
                    return
                if path == "/api/agv/lock":
                    wanted = payload.get("locked", payload.get("wanted", True))
                    self._json(200, node.set_control_lock(bool(wanted)))
                    return
                if path == "/api/navigate":
                    target = str(payload.get("target_id", "")).strip()
                    if not target:
                        self._json(400, {"success": False, "message": "target_id required"})
                        return
                    # Relocation knobs: thresholds & forces
                    kwargs = {
                        "wait": bool(payload.get("wait", False)),
                        "skip_reloc": bool(payload.get("skip_reloc", True)),
                    }
                    if "reloc_threshold" in payload:
                        kwargs["reloc_threshold"] = float(payload["reloc_threshold"])
                    if "force_reloc" in payload:
                        kwargs["force_reloc"] = bool(payload["force_reloc"])
                    if "reloc_length" in payload:
                        kwargs["reloc_length"] = float(payload["reloc_length"])
                    if "reloc_timeout" in payload:
                        kwargs["reloc_timeout"] = float(payload["reloc_timeout"])
                    if "skip_reloc" in payload:
                        kwargs["skip_reloc"] = bool(payload["skip_reloc"])
                    self._json(200, node.navigate(target, **kwargs))
                    return
                if path == "/api/cancel":
                    self._json(200, node.cancel())
                    return
                if path == "/api/arm_sequence":
                    self._json(
                        200,
                        node.arm_sequence(
                            str(payload.get("arm_name", "left")),
                            str(payload.get("sequence_name", "pick")),
                        ),
                    )
                    return
                if path == "/api/jason/qr/start":
                    self._json(200, node._get_vision().jason_start_qr())
                    return
                if path == "/api/jason/qr/stop":
                    self._json(200, node._get_vision().jason_stop_qr())
                    return
                if path == "/api/vision/wrist_camera/detect/start":
                    self._json(200, node._get_vision().wrist_start_detection(str(payload.get("mode", "qr"))))
                    return
                if path == "/api/vision/wrist_camera/detect/stop":
                    self._json(200, node._get_vision().wrist_stop_detection())
                    return
                if path == "/api/vision/floor_qr/trigger":
                    out = node._get_vision().floor_qr_trigger()
                    node._vlog.log("camera", "POST /api/vision/floor_qr/trigger", out)
                    self._json(200, out)
                    return
                if path == "/api/vision/wrist_camera/trigger/qr":
                    out = node._get_vision().wrist_trigger_qr()
                    node._vlog.log("camera", "POST /api/vision/wrist_camera/trigger/qr", out)
                    self._json(200, out)
                    return
                if path == "/api/vision/wrist_camera/trigger/apriltag":
                    out = node._get_vision().wrist_trigger_apriltag()
                    node._vlog.log("camera", "POST /api/vision/wrist_camera/trigger/apriltag", out)
                    self._json(200, out)
                    return
                if path == "/api/vision/leo_face/trigger":
                    self._json(200, node._get_vision().leo_trigger_face())
                    return
                if path == "/api/vision/leo_face/continuous/start":
                    self._json(200, node._get_vision().leo_set_continuous(True))
                    return
                if path == "/api/vision/leo_face/continuous/stop":
                    self._json(200, node._get_vision().leo_set_continuous(False))
                    return
                if path == "/api/stack/status":
                    self._json(200, node._stack_supervisor.status())
                    return
                if path == "/api/stack/ensure":
                    self._json(200, node._stack_supervisor.ensure_all(force=True))
                    return
                if path == "/api/arm/command":
                    self._json(200, node._get_arm().command(payload))
                    return
                if path == "/api/arm/mode":
                    mode = str(payload.get("mode", "simulation"))
                    self._json(200, node._get_arm().set_arm_mode(mode))
                    return
                if path == "/api/arm/unlock":
                    node._vlog.log("arm", "POST /api/arm/unlock", payload)
                    out = node._get_arm().unlock()
                    node._vlog.log("arm", "unlock result", out)
                    self._json(200, out)
                    return
                if path == "/api/arm/pick_place":
                    cycles = int(payload.get("cycles", 1) or 1)
                    node._vlog.log("arm", "POST /api/arm/pick_place", payload)
                    out = node._get_arm().pick_place(cycles=cycles)
                    node._vlog.log("arm", "pick_place result", out)
                    self._json(200, out)
                    return
                if path == "/api/arm/gripper":
                    open_cmd = bool(payload.get("open", True))
                    node._vlog.log("arm", "POST /api/arm/gripper", payload)
                    out = node._get_arm().gripper_set(open_cmd)
                    node._vlog.log("arm", "gripper result", out)
                    self._json(200, out)
                    return
                if path == "/api/arm/cycles":
                    cycles = int(payload.get("cycles", 1) or 1)
                    node._vlog.log("arm", "POST /api/arm/cycles", payload)
                    self._json(200, node._get_arm().set_cycles(cycles))
                    return
                if path == "/api/arm/stop":
                    node._vlog.log("arm", "POST /api/arm/stop", payload)
                    self._json(200, node._get_arm().stop())
                    return
                if path == "/api/mock/vision":
                    self._json(200, node.mock_vision_control(payload))
                    return
                if path == "/api/maps/switch":
                    name = str(payload.get("map_name") or payload.get("smap") or "").strip()
                    if not name:
                        self._json(400, {"success": False, "message": "map_name required"})
                        return
                    self._json(200, node.switch_robot_map(name))
                    return
                if path == "/api/maps/refresh":
                    self._json(200, node.sync_map_from_robot())
                    return
                if path == "/api/stations/add":
                    sid = str(payload.get("station_id") or payload.get("name") or "").strip()
                    if not sid:
                        self._json(400, {"success": False, "message": "station_id required"})
                        return
                    self._json(
                        200,
                        node.add_station_to_map(
                            sid,
                            float(payload.get("x", 0.0)),
                            float(payload.get("y", 0.0)),
                            float(payload.get("yaw", 0.0) or 0.0),
                            connect_from=str(payload.get("connect_from") or payload.get("link") or ""),
                        ),
                    )
                    return
                if path == "/api/stations/connect":
                    a = str(payload.get("from_id") or payload.get("from") or "").strip()
                    b = str(payload.get("to_id") or payload.get("to") or "").strip()
                    if not a or not b:
                        self._json(400, {"success": False, "message": "from_id and to_id required"})
                        return
                    self._json(200, node.connect_map_stations(a, b))
                    return
                if path == "/api/stations/rename":
                    old_id = str(payload.get("old_id") or "").strip()
                    new_id = str(payload.get("new_id") or "").strip()
                    if not old_id or not new_id:
                        self._json(400, {"success": False, "message": "old_id and new_id required"})
                        return
                    self._json(200, node.rename_map_station(old_id, new_id))
                    return
                if path in ("/api/stations/delete", "/api/stations/remove"):
                    sid = str(payload.get("station_id") or payload.get("name") or "").strip()
                    if not sid:
                        self._json(400, {"success": False, "message": "station_id required"})
                        return
                    self._json(200, node.delete_map_station(sid))
                    return
                if path == "/api/stations":
                    name = str(payload.get("name", "")).strip()
                    if str(payload.get("action", "add")) in ("remove", "delete"):
                        self._json(200, node.remove_station(name))
                        return
                    self._json(200, node.add_station(name, float(payload.get("x", 0)), float(payload.get("y", 0))))
                    return
                if path == "/api/detect_qr":
                    self._json(
                        200,
                        node.detect_qr(str(payload.get("camera_name", "cam_front"))),
                    )
                    return
                if path == "/api/face/identify":
                    self._json(
                        200,
                        node.identify_face(
                            str(payload.get("camera_name", "cam_front") or "cam_front"),
                            float(payload.get("min_confidence", 0.5) or 0.5),
                        ),
                    )
                    return
                if path == "/api/env":
                    self._json(
                        200,
                        node.set_env(
                            str(payload.get("mode", "dev")),
                            str(payload.get("agv_host") or "") or None,
                        ),
                    )
                    return
                if path == "/api/log/toggle":
                    module = str(payload.get("module", "")).strip()
                    enable = bool(payload.get("enable", True))
                    self._json(200, node.verbose_log_toggle(module, enable))
                    return
                if path == "/api/log/toggle_all":
                    enable = bool(payload.get("enable", True))
                    self._json(200, node.verbose_log_toggle_all(enable))
                    return
                if path == "/api/sim/map":
                    name = str(payload.get("smap") or payload.get("map_file") or "").strip()
                    if not name:
                        self._json(400, {"success": False, "message": "smap required"})
                        return
                    self._json(200, node.set_map(name))
                    return
                if path == "/api/sim/obstacles":
                    action = str(payload.get("action") or "add").strip().lower()
                    if action == "clear":
                        self._json(200, node.clear_obstacles())
                        return
                    if action == "remove":
                        self._json(200, node.remove_obstacle(str(payload.get("id") or "")))
                        return
                    self._json(
                        200,
                        node.add_obstacle(
                            float(payload.get("x", 0.0)),
                            float(payload.get("y", 0.0)),
                            float(payload.get("w", 0.6) or 0.6),
                            float(payload.get("h", 0.6) or 0.6),
                        ),
                    )
                    return
                if path == "/api/sim/route":
                    stations = payload.get("stations") or []
                    if not isinstance(stations, list):
                        self._json(400, {"success": False, "message": "stations list required"})
                        return
                    out = node.set_route([str(s) for s in stations])
                    if out.get("success") and bool(payload.get("run")):
                        if bool(payload.get("async", True)):
                            out = node.start_route_async(wait=bool(payload.get("wait", True)))
                        else:
                            out = node.run_route(wait=bool(payload.get("wait", True)))
                    self._json(200, out)
                    return
                if path == "/api/sim/route/run":
                    self._json(200, node.run_route(wait=bool(payload.get("wait", True))))
                    return
                if path == "/api/watch":
                    paths = payload.get("paths")
                    if isinstance(paths, list):
                        with node._lock:
                            node._watch = [str(x) for x in paths][:80]
                        self._json(200, {"success": True, "watch": node._watch})
                        return
                    add = str(payload.get("add", "")).strip()
                    if add:
                        with node._lock:
                            if add not in node._watch:
                                node._watch.append(add)
                        self._json(200, {"success": True, "watch": node._watch})
                        return
                    self._json(400, {"success": False, "message": "paths or add required"})
                    return
                self._json(404, {"success": False, "message": "unknown api"})

        return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
