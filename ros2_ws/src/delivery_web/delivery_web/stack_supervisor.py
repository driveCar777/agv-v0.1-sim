"""Auto-start and watchdog for NUC alpha hardware stack (camera, scanner, QR)."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Any, Callable, Dict, List, Optional

LOG_DIR = os.environ.get("DELIVERY_LOG_DIR", "/var/log/delivery")
AUTO_START = os.environ.get("STACK_AUTO_START", "1").strip().lower() not in (
    "0",
    "false",
    "no",
    "off",
)


WRIST_IMAGE_TOPIC = "/my_camera/pylon_ros2_camera_node/image_raw"
def _pgrep_count(pattern: str) -> int:
    try:
        r = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        if r.returncode != 0:
            return 0
        return len([ln for ln in (r.stdout or "").splitlines() if ln.strip()])
    except Exception:  # noqa: BLE001
        return 0


def _pgrep(pattern: str) -> bool:
    return _pgrep_count(pattern) > 0


def _kill_pattern(pattern: str) -> None:
    try:
        subprocess.run(["pkill", "-f", pattern], check=False, timeout=2.0)
    except Exception:  # noqa: BLE001
        pass


def _ros_env() -> Dict[str, str]:
    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", "30")
    env.setdefault("ROS_LOCALHOST_ONLY", "0")
    env.setdefault("PYLON_ROOT", "/opt/pylon")
    env.setdefault(
        "GENICAM_GENTL64_PATH",
        "/opt/pylon/lib/gentlproducer/gtl",
    )
    return env


class StackSupervisor:
    """Ensure dependent ROS processes are running on NUC alpha."""

    def __init__(
        self,
        cfg: Any,
        logger: Optional[Callable[..., None]] = None,
    ) -> None:
        self._cfg = cfg
        self._log = logger or (lambda *a, **k: None)
        self._lock = threading.Lock()
        self._started = False
        self._last_check = 0.0
        self._status: Dict[str, Any] = {}
        self._scanner_ip = str(getattr(cfg, "floor_qr_scanner_host", "172.31.0.91"))
        self._scanner_port = int(getattr(cfg, "floor_qr_scanner_port", 9004))
        self._image_topic = str(
            getattr(cfg, "wrist_camera_image_topic", WRIST_IMAGE_TOPIC) or WRIST_IMAGE_TOPIC
        )

    def _components(self) -> List[Dict[str, Any]]:
        return [
            {
                "name": "keyence",
                "pgrep": "keyence_sr_node",
                "cmd": (
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/delivery_ws/install/setup.bash && "
                    f"ros2 launch keyence_sr_wrapper keyence_sr_node.launch.py "
                    f"scanner_ip:={self._scanner_ip} scanner_port:={self._scanner_port}"
                ),
                "log": "keyence_sr_node.log",
                "sleep": 2.0,
            },
            {
                "name": "jason_camera",
                "pgrep": "pylon_ros2_camera",
                "cmd": (
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/delivery_ws/install/setup.bash && "
                    "ros2 launch delivery_bringup jason_camera.launch.py"
                ),
                "log": "jason_camera.log",
                "sleep": 4.0,
            },
            {
                "name": "wrist_qr",
                "pgrep": "qrcode_node",
                "cmd": (
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/delivery_ws/install/setup.bash && "
                    f"ros2 launch qrcode_detector qrcode_detector.launch.py "
                    f"image_topic:={self._image_topic}"
                ),
                "log": "wrist_qr.log",
                "sleep": 1.5,
                "after": ("jason_camera",),
            },
            {
                "name": "wrist_apriltag",
                "pgrep": "apriltag_node",
                "cmd": (
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/delivery_ws/install/setup.bash && "
                    "ros2 run apriltag_ros apriltag_node --ros-args "
                    f"-r image_rect:={self._image_topic} "
                    f"-r camera_info:={str(self._image_topic).replace('/image_raw', '/camera_info')} "
                    "--params-file /opt/delivery_ws/src/delivery_web/config/apriltag_36h11.yaml"
                ),
                "log": "wrist_apriltag.log",
                "sleep": 2.0,
                "after": ("jason_camera",),
            },
        ]

    def _start_one(self, comp: Dict[str, Any]) -> Dict[str, Any]:
        name = str(comp["name"])
        pattern = str(comp["pgrep"])
        count = _pgrep_count(pattern)
        if name == "wrist_qr" and count > 1:
            self._log(f"stack_supervisor: killing {count} duplicate {pattern} processes")
            _kill_pattern(pattern)
            time.sleep(1.0)
            count = 0
        if count > 0:
            return {"name": name, "action": "already_running", "ok": True, "count": count}
        for dep in comp.get("after") or ():
            dep_comp = next((c for c in self._components() if c["name"] == dep), None)
            if dep_comp and not _pgrep(str(dep_comp["pgrep"])):
                return {"name": name, "action": "waiting_dependency", "ok": False, "dep": dep}
        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, str(comp.get("log", f"{name}.log")))
        cmd = ["bash", "-lc", str(comp["cmd"])]
        try:
            with open(log_path, "a", encoding="utf-8") as logf:
                subprocess.Popen(
                    cmd,
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                    env=_ros_env(),
                    start_new_session=True,
                )
            time.sleep(float(comp.get("sleep", 1.0)))
            ok = _pgrep(pattern)
            self._log(f"stack_supervisor: started {name} ok={ok}")
            return {"name": name, "action": "started", "ok": ok, "log": log_path}
        except Exception as exc:  # noqa: BLE001
            self._log(f"stack_supervisor: failed to start {name}: {exc}")
            return {"name": name, "action": "error", "ok": False, "error": str(exc)}

    def ensure_all(self, *, force: bool = False) -> Dict[str, Any]:
        if not AUTO_START and not force:
            return {"enabled": False, "components": []}
        with self._lock:
            now = time.time()
            if self._started and not force and (now - self._last_check) < 15.0:
                return dict(self._status)
            results = []
            # Do NOT extra-connect to Keyence TCP 9004. SR-1000 is typically
            # single-client; a probe or restart drops the live wrapper socket
            # and causes first-scan-ok / later-timeout flakiness.
            for comp in self._components():
                results.append(self._start_one(comp))
            out = {
                "enabled": True,
                "ts": now,
                "components": results,
                "keyence": _pgrep("keyence_sr_node"),
                "pylon": _pgrep("pylon_ros2_camera"),
                "wrist_qr": _pgrep("qrcode_node"),
                "wrist_apriltag": _pgrep("apriltag_node"),
            }
            self._status = out
            self._started = True
            self._last_check = now
            return out

    def status(self) -> Dict[str, Any]:
        return {
            "auto_start": AUTO_START,
            "keyence": _pgrep("keyence_sr_node"),
            "pylon": _pgrep("pylon_ros2_camera"),
            "wrist_qr": _pgrep("qrcode_node"),
            "wrist_apriltag": _pgrep("apriltag_node"),
            "last": dict(self._status),
        }
