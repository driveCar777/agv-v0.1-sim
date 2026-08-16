"""Shared ROS topic / network probes for vision bridges."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Dict, Optional


def ros_env(domain_id: Optional[int] = None) -> Dict[str, str]:
    env = os.environ.copy()
    did = domain_id if domain_id is not None else int(os.environ.get("ROS_DOMAIN_ID", "30"))
    env["ROS_DOMAIN_ID"] = str(did)
    return env


def ping_host(host: str) -> bool:
    if not host:
        return False
    try:
        r = subprocess.run(
            ["ping", "-c", "1", "-W", "1", host],
            capture_output=True,
            timeout=2,
        )
        return r.returncode == 0
    except Exception:  # noqa: BLE001
        return False


def tcp_port_open(host: str, port: int, timeout: float = 1.5) -> bool:
    if not host or port <= 0:
        return False
    try:
        import socket

        with socket.create_connection((host, int(port)), timeout=float(timeout)):
            return True
    except Exception:  # noqa: BLE001
        return False


class RosTopicProbe:
    """Cached ros2 topic publisher count — refresh runs in background, never blocks HTTP."""

    def __init__(self, topic: str, domain_id: Optional[int] = None, interval: float = 5.0) -> None:
        self._topic = topic
        self._domain_id = domain_id
        self._interval = interval
        self._publishers = 0
        self._last_check = 0.0
        self._lock = threading.Lock()
        self._refreshing = False

    def publisher_count(self) -> int:
        now = time.time()
        with self._lock:
            stale = (now - self._last_check) >= self._interval
            count = self._publishers
            refreshing = self._refreshing
        if stale and not refreshing:
            threading.Thread(target=self._refresh, daemon=True).start()
        return count

    def _refresh(self) -> None:
        with self._lock:
            if self._refreshing:
                return
            self._refreshing = True
        try:
            did = self._domain_id if self._domain_id is not None else os.environ.get("ROS_DOMAIN_ID", "30")
            cmd = (
                f"export ROS_DOMAIN_ID={did}; "
                "source /opt/ros/humble/setup.bash 2>/dev/null; "
                "source /opt/delivery_ws/install/setup.bash 2>/dev/null; "
                f"ros2 topic info {self._topic} 2>/dev/null"
            )
            r = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=3,
                env=ros_env(int(did) if str(did).isdigit() else None),
            )
            count = 0
            for line in (r.stdout or "").splitlines():
                if "Publisher count:" in line:
                    try:
                        count = int(line.split(":")[-1].strip())
                    except ValueError:
                        count = 0
            with self._lock:
                self._publishers = count
                self._last_check = time.time()
        except Exception:  # noqa: BLE001
            with self._lock:
                self._last_check = time.time()
        finally:
            with self._lock:
                self._refreshing = False
