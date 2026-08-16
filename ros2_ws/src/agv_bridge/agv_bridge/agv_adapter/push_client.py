"""Robokit 19301 TCP push listener with 1004 polling fallback."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional

from agv_bridge.agv_adapter.models import RobotPose


class PushPoseCache:
    """Thread-safe cache updated by 19301 push or polling fallback."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Optional[RobotPose] = None
        self._last_push = 0.0
        self._connected = False
        self._source = "none"

    def update_from_push(self, raw: Dict[str, Any]) -> None:
        pose = RobotPose.from_robokit(raw)
        with self._lock:
            self._pose = pose
            self._last_push = time.time()
            self._connected = True
            self._source = "19301_push"

    def update_from_poll(self, pose: RobotPose) -> None:
        with self._lock:
            if self._last_push and (time.time() - self._last_push) < 2.0:
                return
            self._pose = pose
            self._source = "1004_poll"

    def get_pose(self) -> Optional[RobotPose]:
        with self._lock:
            return self._pose

    def status(self) -> Dict[str, Any]:
        with self._lock:
            age = time.time() - self._last_push if self._last_push else None
            return {
                "connected": self._connected,
                "source": self._source,
                "last_push_age_sec": round(age, 3) if age is not None else None,
            }


class RobokitPushClient:
    """Background TCP client for robot_push on port 19301."""

    def __init__(self, host: str, cache: PushPoseCache, logger: Optional[Callable[..., None]] = None) -> None:
        self._host = host
        self._cache = cache
        self._log = logger or (lambda *a, **k: None)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self._host, 19301), timeout=5.0)
                sock.settimeout(30.0)
                self._cache._connected = True
                buf = b""
                while not self._stop.is_set():
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf or len(buf) > 65536:
                        if b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                        else:
                            line, buf = buf, b""
                        try:
                            obj = json.loads(line.decode("utf-8", errors="ignore"))
                            if isinstance(obj, dict):
                                self._cache.update_from_push(obj)
                        except json.JSONDecodeError:
                            pass
            except Exception as exc:  # noqa: BLE001
                self._cache._connected = False
                self._log(f"19301 push disconnected: {exc}")
                time.sleep(2.0)
