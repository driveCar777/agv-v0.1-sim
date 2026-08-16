"""System-wide JSONL logger for delivery_web diagnostics."""

from __future__ import annotations

import json
import os
import threading
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional


def new_trace_id() -> str:
    return uuid.uuid4().hex[:12]


class SystemJsonlLogger:
    """Ring buffer + daily JSONL under /var/log/delivery/system/."""

    def __init__(self, log_dir: str = "/var/log/delivery/system", maxlen: int = 4000) -> None:
        self._dir = Path(log_dir)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._buf: Deque[Dict[str, Any]] = deque(maxlen=maxlen)
        day = datetime.now().strftime("%Y%m%d")
        self._path = self._dir / f"system_{day}.jsonl"

    @property
    def path(self) -> Path:
        return self._path

    def log(
        self,
        event: str,
        *,
        kind: str = "system",
        level: str = "INFO",
        msg: str = "",
        trace_id: str = "",
        error: str = "",
        **extra: Any,
    ) -> Dict[str, Any]:
        rec: Dict[str, Any] = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "t": datetime.now().timestamp(),
            "level": level.upper(),
            "kind": kind,
            "event": event,
            "trace_id": trace_id or new_trace_id(),
            "msg": msg,
            "error": error,
        }
        if extra:
            rec["extra"] = extra
        line = json.dumps(rec, ensure_ascii=False)
        with self._lock:
            self._buf.appendleft(rec)
            try:
                with self._path.open("a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except Exception:  # noqa: BLE001
                pass
        return rec

    def query(
        self,
        limit: int = 100,
        kind: str = "",
        level: str = "",
        event: str = "",
        contains: str = "",
    ) -> List[Dict[str, Any]]:
        limit = max(1, min(int(limit or 100), 500))
        with self._lock:
            rows = list(self._buf)
        out: List[Dict[str, Any]] = []
        for r in rows:
            if kind and r.get("kind") != kind:
                continue
            if level and str(r.get("level", "")).upper() != level.upper():
                continue
            if event and r.get("event") != event:
                continue
            if contains:
                blob = json.dumps(r, ensure_ascii=False)
                if contains not in blob:
                    continue
            out.append(r)
            if len(out) >= limit:
                break
        return out


def env_path() -> Path:
    return Path(os.environ.get("DELIVERY_ENV_FILE", "/var/log/delivery/env.json"))


def load_env_file(default_mode: str = "demo", default_agv: str = "192.168.18.198") -> Dict[str, Any]:
    p = env_path()
    base = {
        "mode": "demo",
        "agv_host": default_agv,
        "use_sim_cameras": False,
        "updated_at": 0.0,
    }
    if not p.is_file():
        return base
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            base.update(data)
    except Exception:  # noqa: BLE001
        pass
    return base


def save_env_file(data: Dict[str, Any]) -> None:
    p = env_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
