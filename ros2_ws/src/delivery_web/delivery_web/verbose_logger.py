"""Per-module verbose file logging for delivery_web diagnostics."""

from __future__ import annotations

import json
import logging
import os
import re
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional

MODULES = (
    "agv",
    "laser",
    "map",
    "station",
    "arm",
    "camera",
    "nav",
    "network",
    "system",
)

_SENSITIVE_RE = re.compile(
    r"(password|passwd|token|secret|api[_-]?key|authorization|credential)",
    re.IGNORECASE,
)


def _redact(obj: Any) -> Any:
    if isinstance(obj, dict):
        out: Dict[str, Any] = {}
        for k, v in obj.items():
            if _SENSITIVE_RE.search(str(k)):
                out[k] = "***"
            else:
                out[k] = _redact(v)
        return out
    if isinstance(obj, list):
        return [_redact(x) for x in obj]
    return obj


class VerboseLogger:
    """Module-scoped verbose logging with daily files and 7-day retention."""

    def __init__(self, log_dir: str = "/tmp/agv_verbose_logs") -> None:
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._modules: Dict[str, bool] = {m: False for m in MODULES}
        self._loggers: Dict[str, logging.Logger] = {}
        self._handlers: Dict[str, logging.FileHandler] = {}
        self._day = ""
        self._info_logger = logging.getLogger("agv.info")
        self._info_logger.setLevel(logging.INFO)
        self._info_logger.propagate = False
        self._ensure_handlers()

    def _ensure_handlers(self) -> None:
        today = datetime.now().strftime("%Y%m%d")
        if today == self._day:
            return
        self._day = today
        fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

        for module in MODULES:
            logger = self._loggers.get(module) or logging.getLogger(f"agv.{module}")
            logger.setLevel(logging.DEBUG)
            logger.propagate = False
            old = self._handlers.pop(module, None)
            if old:
                logger.removeHandler(old)
                old.close()
            fh = logging.FileHandler(
                self.log_dir / f"{module}_{today}.log",
                encoding="utf-8",
            )
            fh.setLevel(logging.DEBUG)
            fh.setFormatter(fmt)
            logger.addHandler(fh)
            self._loggers[module] = logger
            self._handlers[module] = fh

        self._info_logger.handlers.clear()
        info_fh = logging.FileHandler(
            self.log_dir / f"info_{today}.log",
            encoding="utf-8",
        )
        info_fh.setFormatter(logging.Formatter("%(asctime)s [INFO] %(message)s"))
        self._info_logger.addHandler(info_fh)
        self._cleanup_old_files()

    def _cleanup_old_files(self) -> None:
        cutoff = datetime.now() - timedelta(days=7)
        for p in self.log_dir.glob("*.log"):
            try:
                if datetime.fromtimestamp(p.stat().st_mtime) < cutoff:
                    p.unlink(missing_ok=True)
            except OSError:
                pass

    def enable(self, module: str) -> None:
        self._ensure_handlers()
        if module in self._modules:
            self._modules[module] = True
            self._loggers[module].info("=== Verbose logging ENABLED for [%s] ===", module)

    def disable(self, module: str) -> None:
        self._ensure_handlers()
        if module in self._modules:
            self._loggers[module].info("=== Verbose logging DISABLED for [%s] ===", module)
            self._modules[module] = False

    def is_enabled(self, module: str) -> bool:
        return bool(self._modules.get(module, False))

    def log(self, module: str, message: str, data: Any = None) -> None:
        self._ensure_handlers()
        self._info_logger.info("[%s] %s", module, message)
        if not self.is_enabled(module):
            return
        logger = self._loggers.get(module)
        if not logger:
            return
        if data is not None:
            safe = _redact(data)
            if isinstance(safe, (dict, list)):
                data_str = json.dumps(safe, ensure_ascii=False, indent=2)
            else:
                data_str = str(safe)
            logger.debug("%s\n  Data: %s", message, data_str)
        else:
            logger.debug(message)

    def log_request(
        self,
        module: str,
        method: str,
        url: str,
        status: Optional[int] = None,
        duration_ms: Optional[int] = None,
        request_body: Any = None,
        response_body: Any = None,
    ) -> None:
        self._ensure_handlers()
        brief = f"{method} {url} → {status or '?'} ({duration_ms if duration_ms is not None else '?'}ms)"
        self._info_logger.info("[%s] %s", module, brief)
        if not self.is_enabled(module):
            return
        msg = f"{method} {url}"
        if status is not None:
            msg += f" → {status}"
        if duration_ms is not None:
            msg += f" ({duration_ms}ms)"
        details: Dict[str, Any] = {}
        if request_body is not None:
            details["request"] = _redact(request_body)
        if response_body is not None:
            details["response"] = _redact(response_body)
        self.log(module, msg, details if details else None)

    def get_status(self) -> Dict[str, bool]:
        return {m: self._modules[m] for m in MODULES}

    def get_recent_logs(self, module: str, lines: int = 50) -> List[str]:
        self._ensure_handlers()
        today = datetime.now().strftime("%Y%m%d")
        filepath = self.log_dir / f"{module}_{today}.log"
        if module == "info":
            filepath = self.log_dir / f"info_{today}.log"
        if not filepath.is_file():
            return []
        try:
            with filepath.open("r", encoding="utf-8") as f:
                all_lines = f.readlines()
        except OSError:
            return []
        return all_lines[-max(1, min(int(lines or 50), 2000)) :]
