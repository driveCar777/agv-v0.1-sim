"""HTTP + mock-control client for V0.2 LIVE scenario traces."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Dict, Optional


class NavLiveClient:
    def __init__(self, base: str = "http://127.0.0.1:19999", host: str = "127.0.0.1") -> None:
        self.base = base.rstrip("/")
        self.host = host

    def get(self, path: str) -> dict:
        url = self.base + path
        with urllib.request.urlopen(url, timeout=10.0) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def post(self, path: str, body: Optional[dict] = None) -> dict:
        url = self.base + path
        data = json.dumps(body or {}).encode("utf-8")
        req = urllib.request.Request(url, data=data, method="POST")
        req.add_header("Content-Type", "application/json")
        with urllib.request.urlopen(req, timeout=20.0) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def mock_control(self, payload: dict) -> dict:
        return self.post("/api/mock/control", payload)

    def ping(self) -> bool:
        try:
            self.get("/api/state")
            return True
        except (urllib.error.URLError, TimeoutError, OSError):
            return False
