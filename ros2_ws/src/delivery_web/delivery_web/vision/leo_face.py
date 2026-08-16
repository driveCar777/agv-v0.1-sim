"""Leo face_executor upper-computer bridge (SOP V1.0, 2026-08-14).

Does not modify Jetson face_executor or Jason camera packages.
Phase-1: SetBool continuous_recognition + String recognition_result.
Capture Trigger is backup only and never concurrent with continuous.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from delivery_web.device_config import DeviceConfig
from delivery_web.vision.models import LeoFaceStatus

# Topic QoS must match Jetson publisher (doc §4.2).
_RESULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _parse_json(message: str) -> Dict[str, Any]:
    raw = str(message or "").strip()
    if not raw:
        return {}
    try:
        data = json.loads(raw)
        return data if isinstance(data, dict) else {"raw": raw}
    except json.JSONDecodeError:
        return {"raw": raw}


class LeoFaceBridge:
    """NUC-side client for /face/cam_front on ROS_DOMAIN_ID=30."""

    def __init__(
        self,
        node: Node,
        lock: threading.Lock,
        cfg: Optional[DeviceConfig] = None,
    ) -> None:
        self._node = node
        self._lock = lock
        self._cfg = cfg or DeviceConfig.load()
        self._last_json: Dict[str, Any] = {}
        self._last_seq = -1
        self._last_at = 0.0
        self._continuous = False
        self._svc_seen = False
        self._svc_checked = 0.0

        cam = self._cfg.leo_face_camera_name
        self._result_topic = self._cfg.leo_face_result_topic or f"/face/{cam}/recognition_result"
        self._continuous_svc = (
            self._cfg.leo_face_continuous_service or f"/face/{cam}/continuous_recognition"
        )
        self._capture_svc = (
            self._cfg.leo_face_capture_service or f"/face/{cam}/capture_and_recognize"
        )

        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger

        sub_cg = getattr(node, "_cg", None)
        cli_cg = ReentrantCallbackGroup()
        # Subscribe before SetBool(true) so startup results are not missed (§8.1).
        node.create_subscription(
            String, self._result_topic, self._on_result, _RESULT_QOS, callback_group=sub_cg
        )
        # Exact service Leo uses:
        #   ros2 service call /face/cam_front/continuous_recognition std_srvs/srv/SetBool '{data: true}'
        self._continuous_svc = "/face/cam_front/continuous_recognition"
        self._continuous_client = node.create_client(
            SetBool, self._continuous_svc, callback_group=cli_cg
        )
        self._capture_client = node.create_client(
            Trigger, self._capture_svc, callback_group=cli_cg
        )

    def _on_result(self, msg) -> None:
        raw = str(getattr(msg, "data", "") or "").strip()
        if not raw:
            return
        data = _parse_json(raw)
        if not data:
            return
        seq = int(data.get("sequence", -1) or -1)
        with self._lock:
            if seq >= 0 and seq <= self._last_seq:
                return
            self._last_json = data
            if seq >= 0:
                self._last_seq = seq
            self._last_at = time.time()

    def _service_available(self) -> bool:
        now = time.time()
        if (now - self._svc_checked) < 5.0:
            return self._svc_seen
        seen = False
        try:
            seen = bool(self._continuous_client.wait_for_service(timeout_sec=0.5))
        except Exception:  # noqa: BLE001
            seen = False
        self._svc_seen = seen
        self._svc_checked = now
        return seen

    @staticmethod
    def _identity(data: Dict[str, Any]) -> tuple[str, str, float]:
        faces = data.get("faces") or []
        face0 = faces[0] if faces and isinstance(faces[0], dict) else {}
        rec = face0.get("recognition") if isinstance(face0.get("recognition"), dict) else {}
        identity = str(rec.get("identity") or rec.get("best_candidate") or "")
        sim = float(rec.get("similarity", 0.0) or 0.0)
        return identity, identity, sim

    @staticmethod
    def _faces_brief(data: Dict[str, Any]) -> list:
        out = []
        for face in data.get("faces") or []:
            if not isinstance(face, dict):
                continue
            rec = face.get("recognition") if isinstance(face.get("recognition"), dict) else {}
            out.append({
                "bbox": face.get("bbox") or [],
                "identity": str(rec.get("identity") or ""),
                "similarity": float(rec.get("similarity", 0.0) or 0.0),
                "matched": bool(rec.get("matched")),
            })
        return out

    def _topic_fresh(self, now: Optional[float] = None, max_age: float = 3.0) -> bool:
        now = now or time.time()
        with self._lock:
            last_at = self._last_at
        return last_at > 0 and (now - last_at) <= max_age

    def status(self, face_state: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        del face_state
        now = time.time()
        with self._lock:
            data = dict(self._last_json)
            last_at = self._last_at
            last_seq = self._last_seq
            continuous = self._continuous
        if self._topic_fresh(now):
            continuous = True
        svc = self._service_available()
        raw_status = str(data.get("status") or "")
        json_ok = data.get("success", True) is not False
        person_id, person_name, confidence = self._identity(data)
        if svc and (continuous or last_at > 0):
            st = "ONLINE"
        elif svc:
            st = "READY"
        else:
            st = "OFFLINE"
        if st == "OFFLINE":
            msg = f"face_executor not reachable (domain {self._cfg.ros_domain_id})"
        elif not json_ok:
            msg = str(data.get("error") or "success=false")
        elif person_name and person_name.upper() not in ("", "UNKNOWN") and raw_status != "NO_FACE":
            msg = f"OK MATCH {person_name} ({confidence:.2f})"
        elif person_name.upper() == "UNKNOWN":
            msg = "OK UNKNOWN — face seen, below match threshold"
        elif raw_status == "NO_FACE":
            msg = "NO_FACE — recognition running, no face in frame"
        elif raw_status == "MULTIPLE_FACES":
            msg = "MULTIPLE_FACES"
        elif st == "READY":
            msg = "Service ready — SetBool(true) to start continuous"
        else:
            msg = raw_status or "RUNNING"
        return LeoFaceStatus(
            jetson_host=self._cfg.leo_jetson_host,
            camera_host=self._cfg.leo_camera_host,
            status=st,
            message=msg,
            connected=svc,
            person_id=person_id,
            person_name=person_name,
            confidence=confidence,
        ).to_dict() | {
            "ros_domain_id": self._cfg.ros_domain_id,
            "result_topic": self._result_topic,
            "continuous_service": self._continuous_svc,
            "capture_service": self._capture_svc,
            "last_sequence": last_seq,
            "last_result_at": last_at,
            "continuous_running": continuous,
            "raw_status": raw_status,
            "json_success": json_ok,
            "face_count": int(data.get("face_count", 0) or 0),
            "matched_face_count": int(data.get("matched_face_count", 0) or 0),
            "preview_available": False,
            "faces": self._faces_brief(data),
        }

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.time() + max(0.5, float(timeout_sec))
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.done()

    def _result_view(self, data: Dict[str, Any], svc_ok: bool) -> Dict[str, Any]:
        json_ok = data.get("success", svc_ok) is not False
        ok = bool(svc_ok) and json_ok
        person_id, person_name, confidence = self._identity(data)
        status = str(data.get("status") or "")
        matched = int(data.get("matched_face_count", 0) or 0) > 0
        named = bool(person_name) and person_name.upper() not in ("", "UNKNOWN")
        found = ok and status != "NO_FACE" and (matched or named)
        return {
            "success": ok,
            "found": found,
            "status": status,
            "person_id": person_id,
            "person_name": person_name,
            "confidence": confidence,
            "message": person_name or status or ("OK" if ok else "FAILED"),
            "payload": data,
        }

    def set_continuous(self, enabled: bool, timeout_sec: Optional[float] = None) -> Dict[str, Any]:
        """Match Leo's working CLI: SetBool {data: true|false} on /face/cam_front/continuous_recognition."""
        from std_srvs.srv import SetBool

        if timeout_sec is None:
            timeout_sec = 8.0
        yaml_body = "{data: true}" if enabled else "{data: false}"
        if not self._continuous_client.wait_for_service(timeout_sec=3.0):
            return self._set_continuous_via_cli(enabled, timeout_sec, yaml_body)
        req = SetBool.Request()
        req.data = bool(enabled)
        future = self._continuous_client.call_async(req)
        if not self._wait_future(future, timeout_sec):
            return self._set_continuous_via_cli(enabled, timeout_sec, yaml_body)
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            return self._set_continuous_via_cli(enabled, timeout_sec, yaml_body, note=str(exc))
        return self._set_continuous_result(enabled, bool(getattr(res, "success", False)), str(getattr(res, "message", "") or ""))

    def _set_continuous_via_cli(
        self,
        enabled: bool,
        timeout_sec: float,
        yaml_body: str,
        note: str = "",
    ) -> Dict[str, Any]:
        """Same command Leo uses in the terminal."""
        import os
        import subprocess

        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self._cfg.ros_domain_id)
        cmd = [
            "ros2",
            "service",
            "call",
            "/face/cam_front/continuous_recognition",
            "std_srvs/srv/SetBool",
            yaml_body,
        ]
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=max(5.0, float(timeout_sec)),
                env=env,
            )
        except Exception as exc:  # noqa: BLE001
            return {
                "success": False,
                "message": f"cli failed: {exc}" + (f" ({note})" if note else ""),
                "enabled": enabled,
                "command": " ".join(cmd),
            }
        text = (proc.stdout or "") + "\n" + (proc.stderr or "")
        success = "success: True" in text or "success: true" in text
        msg = ""
        for line in text.splitlines():
            s = line.strip()
            if s.startswith("message:"):
                msg = s.split(":", 1)[1].strip().strip("'").strip('"')
                break
        if not success and proc.returncode != 0:
            return {
                "success": False,
                "message": (msg or text.strip() or f"exit {proc.returncode}") + (f" ({note})" if note else ""),
                "enabled": enabled,
                "command": " ".join(cmd),
            }
        out = self._set_continuous_result(enabled, True if success else proc.returncode == 0, msg or text.strip())
        out["command"] = " ".join(cmd)
        out["via"] = "ros2_cli"
        return out

    def _set_continuous_result(self, enabled: bool, svc_ok: bool, message: str) -> Dict[str, Any]:
        payload = _parse_json(message)
        status = str(payload.get("status") or "")
        json_ok = payload.get("success", svc_ok) is not False
        ok = bool(svc_ok) and json_ok
        if enabled:
            running = ok and status in ("", "RUNNING", "ALREADY_RUNNING")
            if ok and not status:
                running = True
        else:
            running = False
        with self._lock:
            self._continuous = running
        return {
            "success": ok,
            "message": message or status or ("OK" if ok else "FAILED"),
            "enabled": enabled,
            "status": status or ("RUNNING" if running else ("STOPPED" if ok and not enabled else "")),
            "payload": payload,
            "command": f"ros2 service call /face/cam_front/continuous_recognition std_srvs/srv/SetBool '{{data: {'true' if enabled else 'false'}}}'",
        }

    def trigger_face(self) -> Dict[str, Any]:
        """Read latest topic JSON. Never call capture while continuous is on (§9.4)."""
        if self._continuous or self._topic_fresh():
            with self._lock:
                data = dict(self._last_json)
            if not data:
                return {
                    "success": True,
                    "found": False,
                    "status": "RUNNING",
                    "message": "continuous running, waiting for first recognition_result",
                    "payload": {},
                }
            out = self._result_view(data, True)
            out["from_topic"] = True
            return out
        return self.capture_once()

    def capture_once(self, timeout_sec: float = 30.0) -> Dict[str, Any]:
        from std_srvs.srv import Trigger

        if self._continuous or self._topic_fresh():
            return {
                "success": False,
                "found": False,
                "message": "cannot capture while continuous is running — SetBool(false) first",
            }
        if not self._capture_client.wait_for_service(timeout_sec=5.0):
            return {
                "success": False,
                "found": False,
                "message": f"capture service unavailable: {self._capture_svc}",
            }
        future = self._capture_client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout_sec):
            return {"success": False, "found": False, "message": "capture_and_recognize timeout"}
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "found": False, "message": str(exc)}
        data = _parse_json(str(res.message or ""))
        if data:
            seq = int(data.get("sequence", -1) or -1)
            with self._lock:
                self._last_json = data
                self._last_at = time.time()
                if seq > self._last_seq:
                    self._last_seq = seq
        return self._result_view(data, bool(res.success))

    def ensure_continuous(self) -> Dict[str, Any]:
        if self._continuous or self._topic_fresh():
            with self._lock:
                self._continuous = True
            return {"success": True, "status": "ALREADY_RUNNING", "message": "already running"}
        return self.set_continuous(True)
