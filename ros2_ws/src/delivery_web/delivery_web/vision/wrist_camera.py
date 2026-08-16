"""Wrist camera bridge — Basler Pylon ROS image + QR/AprilTag detections."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from collections import deque
from typing import Any, Callable, Dict, List, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from delivery_web.device_config import DeviceConfig
from delivery_web.vision.models import DetectionResult, WristCameraStatus
from delivery_web.vision.ros_probe import RosTopicProbe, ping_host, ros_env

OFFLINE_SEC = 8.0
LIVE_FRAME_SEC = 8.0  # drop last JPEG only after ROS really stops, not after a slow encode


class WristCameraBridge:
    """Mechanical-arm wrist camera — snapshot polling only (no blocking MJPEG on main page)."""

    def __init__(
        self,
        node: Node,
        lock: threading.Lock,
        encode_jpeg: Callable[[Image], Optional[bytes]],
        logger: Callable[..., None],
        cfg: Optional[DeviceConfig] = None,
    ) -> None:
        self._node = node
        self._lock = lock
        self._encode_jpeg = encode_jpeg
        self._log = logger
        self._cfg = cfg or DeviceConfig.load()
        self._qr_lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._last_frame = 0.0
        self._last_ros = 0.0
        self._pending_raw = None
        self._raw_stop = False
        self._width = 0
        self._height = 0
        self._encoding = ""
        self._fps = 0.0
        self._frame_times: deque = deque(maxlen=60)
        self._qr_running = False
        self._qr_proc: Optional[subprocess.Popen] = None
        self._detections: List[DetectionResult] = []
        self._last_qr = ""
        self._last_apriltag = ""
        self._apriltag_pose: Dict[str, Any] = {}
        self._cam_k = None
        self._cam_d = None
        self._tag_size_m = float(os.environ.get("APRILTAG_SIZE_M", "0.16") or 0.16)
        self._pnp_note = False
        self._last_local_qr_try = 0.0
        self._msg_received = False
        self._net_ping_ok = False
        self._network_checked = 0.0
        self._topic_probe = RosTopicProbe(
            self._cfg.wrist_camera_image_topic,
            domain_id=self._cfg.ros_domain_id,
        )
        cg = getattr(node, "_cg", None)
        comp = str(self._cfg.wrist_camera_compressed_topic or "").strip()
        if not comp:
            img_t = str(self._cfg.wrist_camera_image_topic or "")
            if img_t.endswith("/image_raw"):
                comp = img_t + "/compressed"
        # Preview MUST use compressed JPEG. Encoding 5MP raw on the ROS executor
        # stalls HTTP (arm timeout) and eventually kills the live view.
        if comp:
            reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            for qos, _label in ((qos_profile_sensor_data, "BEST_EFFORT"), (reliable, "RELIABLE")):
                node.create_subscription(
                    CompressedImage, comp, self._on_compressed, qos, callback_group=cg
                )
            self._log(f"Wrist camera subscribe CompressedImage {comp} BEST_EFFORT+RELIABLE")
        # Pylon on this NUC currently has 0 compressed publishers — preview uses raw
        # on a worker thread (never encode on the ROS executor).
        t = str(self._cfg.wrist_camera_image_topic or "").strip()
        if t:
            node.create_subscription(
                Image, t, self._on_img, qos_profile_sensor_data, callback_group=cg
            )
            self._log(f"Wrist camera subscribe Image {t} (worker encode, drop-old)")
        threading.Thread(target=self._raw_encode_loop, daemon=True).start()
        # Reliable for String decode topics
        str_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(
            String,
            self._cfg.wrist_camera_qr_topic,
            self._on_qr_decoded,
            str_qos,
            callback_group=cg,
        )
        tag_topic = str(getattr(self._cfg, "wrist_camera_apriltag_topic", "") or "").strip()
        if tag_topic:
            try:
                from apriltag_msgs.msg import AprilTagDetectionArray

                node.create_subscription(
                    AprilTagDetectionArray,
                    tag_topic,
                    self._on_apriltag_detections,
                    qos_profile_sensor_data,
                    callback_group=cg,
                )
                self._log(f"AprilTag subscribe detections {tag_topic}")
            except Exception as exc:  # noqa: BLE001
                self._log(f"AprilTagDetectionArray unavailable, using pose/tf: {exc}")
        try:
            from geometry_msgs.msg import PoseStamped, TransformStamped
            from sensor_msgs.msg import CameraInfo
            from tf2_msgs.msg import TFMessage

            info_topic = str(getattr(self._cfg, "wrist_camera_info_topic", "") or "").strip()
            if not info_topic:
                img_t = str(self._cfg.wrist_camera_image_topic or "")
                info_topic = img_t.replace("/image_raw", "/camera_info") if img_t else "/my_camera/pylon_ros2_camera_node/camera_info"
            # Pylon CameraInfo is RELIABLE; BEST_EFFORT never receives K and PnP stays empty.
            caminfo_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(
                CameraInfo, info_topic, self._on_cam_info, caminfo_qos, callback_group=cg
            )
            self._log(f"AprilTag camera_info {info_topic}")
            # /apriltag/pose has no publisher in stock apriltag_ros; keep sub + /tf + PnP from /detections
            node.create_subscription(
                TransformStamped, "/apriltag/transform", self._on_apriltag_tf,
                qos_profile_sensor_data, callback_group=cg,
            )
            node.create_subscription(
                PoseStamped, "/apriltag/pose", self._on_apriltag_pose,
                qos_profile_sensor_data, callback_group=cg,
            )
            node.create_subscription(TFMessage, "/tf", self._on_tf_tags, 50, callback_group=cg)
        except Exception as exc:  # noqa: BLE001
            self._log(f"AprilTag pose/tf subscribe failed: {exc}")

    def _ensure_qr_off_at_boot(self) -> None:
        time.sleep(2.0)
        self.stop_detection(silent=True)

    def _store_jpeg(self, jpg: Optional[bytes], width: int, height: int, encoding: str = "jpeg") -> None:
        if not jpg:
            return
        now = time.time()
        with self._lock:
            self._last_frame = now
            self._last_ros = now
            self._width = int(width)
            self._height = int(height)
            self._encoding = encoding
            self._frame_times.append(now)
            if len(self._frame_times) >= 2:
                span = self._frame_times[-1] - self._frame_times[0]
                if span > 0:
                    self._fps = (len(self._frame_times) - 1) / span
            self._jpeg = jpg
        self._maybe_local_qr_async(jpg)

    def _apply_qr_text(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        det = DetectionResult(
            detected_type="QR",
            detected_id=text,
            confidence=1.0,
            timestamp=time.time(),
        )
        with self._lock:
            self._last_qr = text
            self._detections = [det] + [d for d in self._detections if d.detected_type != "QR"][:8]

    def _decode_qr_jpeg(self, jpg: bytes) -> str:
        """Decode QR from dashboard JPEG. Does not modify Jason's qrcode_node."""
        if not jpg or len(jpg) < 100:
            return ""
        try:
            from pyzbar.pyzbar import decode as zbar_decode
            import cv2
            import numpy as np

            arr = np.frombuffer(jpg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                return ""
            for item in zbar_decode(img):
                text = (item.data or b"").decode("utf-8", "ignore").strip()
                if text:
                    return text
        except Exception as exc:  # noqa: BLE001
            now = time.time()
            if now - getattr(self, "_last_local_qr_err", 0.0) > 15.0:
                self._last_local_qr_err = now
                self._log(f"local QR decode unavailable: {exc}")
        return ""

    def _maybe_local_qr_async(self, jpg: bytes) -> None:
        now = time.time()
        if now - self._last_local_qr_try < 0.8:
            return
        if self._qr_busy:
            return
        self._last_local_qr_try = now
        self._qr_busy = True

        def _run() -> None:
            try:
                text = self._decode_qr_jpeg(jpg)
                if text:
                    self._apply_qr_text(text)
            finally:
                self._qr_busy = False

        threading.Thread(target=_run, daemon=True).start()

    def _raw_encode_loop(self) -> None:
        while not self._raw_stop:
            payload = None
            with self._lock:
                payload = self._pending_raw
                self._pending_raw = None
            if payload is None:
                time.sleep(0.02)
                continue
            try:
                jpg = self._encode_payload(payload)
                self._store_jpeg(jpg, int(payload.get("w") or 0), int(payload.get("h") or 0), str(payload.get("enc") or "raw"))
            except Exception:  # noqa: BLE001
                time.sleep(0.05)

    def _encode_payload(self, payload: Dict[str, Any]) -> Optional[bytes]:
        class _Tmp:
            pass

        tmp = _Tmp()
        tmp.width = int(payload.get("w") or 0)
        tmp.height = int(payload.get("h") or 0)
        tmp.encoding = str(payload.get("enc") or "rgb8")
        tmp.data = payload.get("data") or b""
        tmp.step = int(payload.get("step") or 0)
        return self._encode_jpeg(tmp)

    def _on_img(self, msg: Image) -> None:
        self._msg_received = True
        now = time.time()
        with self._lock:
            self._last_ros = now
            if self._pending_raw is not None:
                return
            if self._jpeg and (now - self._last_frame) < 0.35:
                return
        try:
            data = bytes(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._pending_raw = {
                "w": int(msg.width),
                "h": int(msg.height),
                "enc": str(msg.encoding or ""),
                "step": int(getattr(msg, "step", 0) or 0),
                "data": data,
            }

    def _on_compressed(self, msg: CompressedImage) -> None:
        self._msg_received = True
        with self._lock:
            self._last_ros = time.time()
        fmt = (msg.format or "").lower()
        data = bytes(msg.data)
        if not data:
            return
        if "jpeg" in fmt or "jpg" in fmt or data[:2] == b"\xff\xd8":
            self._store_jpeg(data, 0, 0, "jpeg")
            return
        if "png" in fmt or data[:8] == b"\x89PNG\r\n\x1a\n":
            self._store_jpeg(data, 0, 0, "png")

    def _on_qr_decoded(self, msg: String) -> None:
        self._apply_qr_text((msg.data or "").strip())

    def _apply_apriltag_text(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        det = DetectionResult(
            detected_type="AprilTag",
            detected_id=text,
            confidence=1.0,
            timestamp=time.time(),
        )
        with self._lock:
            self._last_apriltag = text
            self._detections = [det] + [d for d in self._detections if d.detected_type != "AprilTag"][:8]

    def _on_apriltag_detections(self, msg) -> None:
        detections = list(getattr(msg, "detections", None) or [])
        if not detections:
            return
        best = detections[0]
        family = str(getattr(best, "family", "") or "tag36h11")
        if not family.startswith("tag"):
            family = f"tag{family}"
        tag_id = int(getattr(best, "id", 0) or 0)
        text = f"{family}:{tag_id}"
        self._apply_apriltag_text(text)
        self._pose_from_detection(best, text)

    def _on_cam_info(self, msg) -> None:
        try:
            import numpy as np
            k = list(msg.k or [])
            if len(k) >= 9:
                self._cam_k = np.array(k, dtype=float).reshape(3, 3)
            d = list(msg.d or [])
            self._cam_d = np.array(d, dtype=float) if d else None
        except Exception:  # noqa: BLE001
            pass

    def _on_tf_tags(self, msg) -> None:
        for t in list(getattr(msg, "transforms", None) or []):
            child = str(getattr(t, "child_frame_id", "") or "")
            if "tag" not in child.lower() and "36h11" not in child:
                continue
            tf = getattr(t, "transform", None)
            if tf is None:
                continue
            self._store_apriltag_pose(
                getattr(tf, "translation", None),
                getattr(tf, "rotation", None),
                child,
            )

    def _pose_from_detection(self, det: Any, tag_id: str) -> None:
        pose = getattr(det, "pose", None)
        if pose is not None:
            inner = getattr(pose, "pose", pose)
            inner = getattr(inner, "pose", inner)
            pos = getattr(inner, "position", None)
            ori = getattr(inner, "orientation", None)
            if pos is not None and ori is not None:
                self._store_apriltag_pose(pos, ori, tag_id)
                return
        corners = list(getattr(det, "corners", None) or [])
        if len(corners) < 4:
            return
        if self._cam_k is None:
            now = time.time()
            if now - float(getattr(self, "_pnp_wait_log", 0.0) or 0.0) > 8.0:
                self._pnp_wait_log = now
                self._log("AprilTag 6D waiting camera_info (PnP)")
            return
        try:
            import cv2
            import numpy as np

            img = np.array([[float(c.x), float(c.y)] for c in corners[:4]], dtype=np.float64)
            s = max(0.01, float(self._tag_size_m) / 2.0)
            obj = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float64)
            dist = self._cam_d if self._cam_d is not None else np.zeros((5, 1))
            ok, rvec, tvec = cv2.solvePnP(obj, img, self._cam_k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return
            R, _ = cv2.Rodrigues(rvec)
            tr = float(R[0, 0] + R[1, 1] + R[2, 2])
            if tr > 0:
                s2 = (tr + 1.0) ** 0.5 * 2.0
                qw = 0.25 * s2
                qx = (R[2, 1] - R[1, 2]) / s2
                qy = (R[0, 2] - R[2, 0]) / s2
                qz = (R[1, 0] - R[0, 1]) / s2
            else:
                i = int(max(range(3), key=lambda k: R[k, k]))
                if i == 0:
                    s2 = (R[0, 0] - R[1, 1] - R[2, 2] + 1.0) ** 0.5 * 2.0
                    qx = 0.25 * s2
                    qy = (R[0, 1] + R[1, 0]) / s2
                    qz = (R[0, 2] + R[2, 0]) / s2
                    qw = (R[2, 1] - R[1, 2]) / s2
                elif i == 1:
                    s2 = (R[1, 1] - R[0, 0] - R[2, 2] + 1.0) ** 0.5 * 2.0
                    qx = (R[0, 1] + R[1, 0]) / s2
                    qy = 0.25 * s2
                    qz = (R[1, 2] + R[2, 1]) / s2
                    qw = (R[0, 2] - R[2, 0]) / s2
                else:
                    s2 = (R[2, 2] - R[0, 0] - R[1, 1] + 1.0) ** 0.5 * 2.0
                    qx = (R[0, 2] + R[2, 0]) / s2
                    qy = (R[1, 2] + R[2, 1]) / s2
                    qz = 0.25 * s2
                    qw = (R[1, 0] - R[0, 1]) / s2
            class _P:
                pass
            pos, ori = _P(), _P()
            pos.x, pos.y, pos.z = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
            ori.x, ori.y, ori.z, ori.w = float(qx), float(qy), float(qz), float(qw)
            self._store_apriltag_pose(pos, ori, tag_id)
            if not self._pnp_note:
                self._pnp_note = True
                self._log(f"AprilTag 6D PnP ok {tag_id} z={pos.z:.3f}")
        except Exception as exc:  # noqa: BLE001
            if not self._pnp_note:
                self._pnp_note = True
                self._log(f"AprilTag 6D PnP failed: {exc}")
            return

    @staticmethod
    def _quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> Dict[str, float]:
        import math
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        rad2deg = 180.0 / math.pi
        return {
            "roll_deg": round(roll * rad2deg, 1),
            "pitch_deg": round(pitch * rad2deg, 1),
            "yaw_deg": round(yaw * rad2deg, 1),
        }

    def _store_apriltag_pose(self, position: Any, orientation: Any, tag_id: str = "") -> None:
        px = float(getattr(position, "x", 0.0) or 0.0)
        py = float(getattr(position, "y", 0.0) or 0.0)
        pz = float(getattr(position, "z", 0.0) or 0.0)
        qx = float(getattr(orientation, "x", 0.0) or 0.0)
        qy = float(getattr(orientation, "y", 0.0) or 0.0)
        qz = float(getattr(orientation, "z", 0.0) or 0.0)
        qw = float(getattr(orientation, "w", 1.0) or 1.0)
        rpy = self._quat_to_rpy_deg(qx, qy, qz, qw)
        pose = {
            "x": round(px, 4),
            "y": round(py, 4),
            "z": round(pz, 4),
            "qx": round(qx, 4),
            "qy": round(qy, 4),
            "qz": round(qz, 4),
            "qw": round(qw, 4),
            **rpy,
            "tag": tag_id,
            "updated_at": time.time(),
        }
        with self._lock:
            self._apriltag_pose = pose
        if tag_id:
            self._apply_apriltag_text(tag_id)

    def _on_apriltag_tf(self, msg) -> None:
        child = str(getattr(msg, "child_frame_id", "") or "").strip()
        tf = getattr(msg, "transform", None)
        if tf is None:
            self._apply_apriltag_text(child)
            return
        self._store_apriltag_pose(
            getattr(tf, "translation", None),
            getattr(tf, "rotation", None),
            child,
        )

    def _on_apriltag_pose(self, msg) -> None:
        header = getattr(msg, "header", None)
        frame = str(getattr(header, "frame_id", "") or "").strip()
        pose = getattr(msg, "pose", None)
        if pose is None:
            if frame and "tag" in frame.lower():
                self._apply_apriltag_text(frame)
            return
        self._store_apriltag_pose(
            getattr(pose, "position", None),
            getattr(pose, "orientation", None),
            frame,
        )

    def latest_jpeg(self, max_age_sec: float = LIVE_FRAME_SEC) -> Optional[bytes]:
        now = time.time()
        with self._lock:
            jpg = self._jpeg
            last = float(self._last_frame or 0.0)
            last_ros = float(self._last_ros or 0.0)
        if not jpg or len(jpg) < 80:
            return None
        newest = max(last, last_ros)
        if newest <= 0:
            return None
        if (now - newest) > float(max_age_sec) and (now - last) > float(max_age_sec):
            return None
        return jpg

    def _network_online(self) -> bool:
        now = time.time()
        if (now - self._network_checked) > 10.0:
            self._net_ping_ok = ping_host(self._cfg.wrist_camera_host)
            self._network_checked = now
        return self._net_ping_ok

    def _cam_status(self) -> str:
        now = time.time()
        with self._lock:
            last = self._last_frame
            has_jpeg = self._jpeg is not None and len(self._jpeg or b"") > 100
            msg_rx = self._msg_received
        ros_pub = msg_rx or has_jpeg or self._topic_probe.publisher_count() >= 1
        net = self._network_online()
        if not net and not ros_pub and last <= 0:
            return "OFFLINE"
        if last <= 0:
            if ros_pub or net:
                return "ONLINE"  # node up, no frame yet
            return "OFFLINE"
        if (now - last) > OFFLINE_SEC:
            if ros_pub or net:
                return "ONLINE"
            return "OFFLINE"
        return "ONLINE" if has_jpeg else "ONLINE"

    def status(self) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            width = self._width
            height = self._height
            encoding = self._encoding
            fps = self._fps
            qr_running = self._qr_running
            dets = list(self._detections)
            last = self._last_frame
            last_ros = float(self._last_ros or 0.0)
            has_jpeg = self._jpeg is not None and len(self._jpeg or b"") > 100
            last_qr = self._last_qr
            last_tag = self._last_apriltag
            tag_pose = dict(self._apriltag_pose or {})
            msg_rx = self._msg_received
        ros_pub = msg_rx or has_jpeg or self._topic_probe.publisher_count() >= 1
        net = self._network_online()
        live_ref = max(last, last_ros)
        age_ms = int((now - live_ref) * 1000) if live_ref > 0 else None
        image_ok = has_jpeg and live_ref > 0 and (now - live_ref) <= LIVE_FRAME_SEC
        st = self._cam_status()
        display = st
        if st == "OFFLINE":
            display = "OFFLINE"
        elif st == "ONLINE" and not image_ok:
            display = "IMAGE UNAVAILABLE"
        # Never surface stale decode results when camera is offline or has no live image.
        show_dets = image_ok and st != "OFFLINE"
        det_qr = (last_qr or self._last_det(dets, "QR")) if show_dets else ""
        det_tag = (last_tag or self._last_det(dets, "AprilTag")) if show_dets else ""
        det_list = list(dets) if show_dets else []
        det_ok = bool(det_qr or det_tag)
        out = WristCameraStatus(
            host=self._cfg.wrist_camera_host,
            status=display,
            network_online=net,
            ros_node_online=ros_pub,
            image_available=image_ok,
            image_timestamp=last,
            detection_available=det_ok,
            has_frame=image_ok,
            fps=round(float(fps), 2),
            width=width,
            height=height,
            encoding=encoding,
            last_frame_ms_ago=age_ms,
            qr_recognition="RUNNING" if qr_running else "OFF",
            last_qr=det_qr,
            last_apriltag=det_tag,
            last_detections=det_list,
        ).to_dict()
        out["ros_image_topic"] = self._cfg.wrist_camera_image_topic
        out["ros_domain_id"] = self._cfg.ros_domain_id
        out["apriltag_pose"] = tag_pose if show_dets else {}
        return out

    @staticmethod
    def _last_det(dets: List[DetectionResult], kind: str) -> str:
        for d in dets:
            if d.detected_type.upper() == kind.upper():
                return d.detected_id
        return ""

    def _ros_env(self) -> Dict[str, str]:
        return ros_env(self._cfg.ros_domain_id)

    def _proc_running(self, pattern: str) -> bool:
        try:
            r = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            return r.returncode == 0 and bool((r.stdout or "").strip())
        except Exception:  # noqa: BLE001
            return False

    def _qr_node_running(self) -> bool:
        return self._proc_running("qrcode_node")

    def start_detection(self, mode: str = "qr") -> Dict[str, Any]:
        with self._qr_lock:
            if mode == "qr" and (self._qr_running or self._qr_node_running()):
                with self._lock:
                    self._qr_running = True
                return {"success": True, "message": "already running", "mode": mode}
            if self._qr_running and mode == "qr":
                return {"success": True, "message": "already running", "mode": mode}
            image_topic = self._cfg.wrist_camera_image_topic
            cam_info = image_topic.replace("/image_raw", "/camera_info")
            if mode == "apriltag":
                if self._proc_running("apriltag_node"):
                    return {"success": True, "message": "apriltag_node already running", "mode": mode}
                params = "/opt/delivery_ws/src/delivery_web/config/apriltag_36h11.yaml"
                cmd = [
                    "bash",
                    "-lc",
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/delivery_ws/install/setup.bash && "
                    "ros2 run apriltag_ros apriltag_node --ros-args "
                    f"-r image_rect:={image_topic} "
                    f"-r camera_info:={cam_info} "
                    f"--params-file {params}",
                ]
            else:
                if self._qr_node_running():
                    with self._lock:
                        self._qr_running = True
                    return {"success": True, "message": "qrcode_node already running", "mode": mode}
                cmd = [
                    "bash",
                    "-lc",
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/delivery_ws/install/setup.bash && "
                    f"ros2 launch qrcode_detector qrcode_detector.launch.py "
                    f"image_topic:={image_topic}",
                ]
            try:
                self._qr_proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=self._ros_env(),
                )
            except Exception as exc:  # noqa: BLE001
                self._log(f"Wrist camera detection start failed: {exc}")
                return {"success": False, "message": str(exc)}
            with self._lock:
                self._qr_running = True
            self._log(f"Wrist camera {mode} detection started")
            return {"success": True, "message": "started", "mode": mode}

    def stop_detection(self, silent: bool = False) -> Dict[str, Any]:
        with self._qr_lock:
            subprocess.run(["pkill", "-f", "qrcode_node"], check=False)
            subprocess.run(["pkill", "-f", "qrcode_detector.launch"], check=False)
            if self._qr_proc is not None:
                try:
                    self._qr_proc.terminate()
                except Exception:  # noqa: BLE001
                    pass
                self._qr_proc = None
            with self._lock:
                self._qr_running = False
            if not silent:
                self._log("Wrist camera detection stopped")
            return {"success": True, "message": "stopped"}

    def trigger_qr_burst(self, timeout_sec: float = 8.0) -> Dict[str, Any]:
        """Decode from live JPEG first (container OpenCV has no QUIRC)."""
        jpg = self.latest_jpeg()
        text = self._decode_qr_jpeg(jpg or b"")
        if text:
            self._apply_qr_text(text)
            return {
                "success": True,
                "found": True,
                "mode": "qr",
                "result": text,
                "message": text,
            }
        if not self._qr_running:
            started = self.start_detection("qr")
            if not started.get("success"):
                return {
                    "success": False,
                    "found": False,
                    "mode": "qr",
                    "result": "",
                    "message": started.get("message", "detector start failed"),
                }
            time.sleep(0.8)
        deadline = time.time() + max(1.5, float(timeout_sec))
        baseline_qr = ""
        with self._lock:
            baseline_qr = self._last_qr
        while time.time() < deadline:
            jpg = self.latest_jpeg()
            text = self._decode_qr_jpeg(jpg or b"")
            if text:
                self._apply_qr_text(text)
                return {
                    "success": True,
                    "found": True,
                    "mode": "qr",
                    "result": text,
                    "message": text,
                }
            with self._lock:
                text = self._last_qr or self._last_det(self._detections, "QR")
                if text and text != baseline_qr:
                    return {
                        "success": True,
                        "found": True,
                        "mode": "qr",
                        "result": text,
                        "message": text,
                    }
            time.sleep(0.08)
        with self._lock:
            text = self._last_qr or self._last_det(self._detections, "QR")
        if text:
            return {"success": True, "found": True, "mode": "qr", "result": text, "message": text}
        return {"success": True, "found": False, "mode": "qr", "result": "", "message": "NOT_FOUND"}

    def trigger_apriltag_burst(self, timeout_sec: float = 8.0) -> Dict[str, Any]:
        started = self.start_detection("apriltag")
        if not started.get("success"):
            return {
                "success": False,
                "found": False,
                "mode": "apriltag",
                "result": "",
                "message": started.get("message", "detector start failed"),
            }
        with self._lock:
            existing = self._last_apriltag or self._last_det(self._detections, "AprilTag")
        if existing:
            return {
                "success": True,
                "found": True,
                "mode": "apriltag",
                "result": existing,
                "tag_id": existing,
                "message": existing,
            }
        time.sleep(0.8)
        deadline = time.time() + max(1.5, float(timeout_sec))
        baseline = ""
        with self._lock:
            baseline = self._last_apriltag
        while time.time() < deadline:
            with self._lock:
                text = self._last_apriltag or self._last_det(self._detections, "AprilTag")
                if text and text != baseline:
                    return {
                        "success": True,
                        "found": True,
                        "mode": "apriltag",
                        "result": text,
                        "tag_id": text,
                        "message": text,
                    }
            time.sleep(0.08)
        with self._lock:
            text = self._last_apriltag or self._last_det(self._detections, "AprilTag")
        if text:
            return {"success": True, "found": True, "mode": "apriltag", "result": text, "tag_id": text, "message": text}
        return {"success": True, "found": False, "mode": "apriltag", "result": "", "message": "NOT_FOUND"}

    start_qr = start_detection
    stop_qr = stop_detection
