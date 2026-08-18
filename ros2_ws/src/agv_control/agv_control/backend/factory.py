"""Backend factory — selects backend by name/environment variable.

M3.9.3 safety rules:
  - Default backend is ALWAYS mock (offline-first).
  - 'real' mode requires REAL_ROBOT_ENABLED=true AND AGV_HOST configured.
  - 'tcp'/'api' mode (for connecting to Mock server) requires explicit AGV_HOST.
  - No default AGV IP (192.168.192.5) allowed for real mode.
  - If any safety gate is missing, raises RuntimeError — backend is NOT created.

backend modes:
  mock      — pure kinematics simulation (default, offline, no network)
  sim       — same as mock with configurable speed
  tcp/api   — Robokit TCP protocol against Mock server (requires AGV_HOST)
  real      — Robokit TCP against real vehicle (requires REAL_ROBOT_ENABLED=true + AGV_HOST)
"""

from __future__ import annotations

import os

from agv_control.backend.base import IAGVControlBackend
from agv_control.backend.mock import MockBackend


def create_backend(backend_name: str | None = None, **kwargs) -> IAGVControlBackend:
    name = (backend_name or os.environ.get("AGV_BACKEND") or "mock").lower()

    if name in ("mock",):
        return MockBackend(**kwargs)

    if name == "sim":
        return MockBackend(sim_speed=kwargs.pop("sim_speed", 1.0), **kwargs)

    if name in ("tcp", "api", "robokit"):
        # tcp/api: connects to Mock server for protocol verification
        # Still requires explicit AGV_HOST — no accidental real robot connections
        from agv_control.backend.real import RealBackend  # noqa: WPS433

        host = kwargs.pop("agv_host", None) or os.environ.get("AGV_HOST", "")
        if not host:
            raise RuntimeError(
                f"backend={name!r} requires AGV_HOST to be set explicitly. "
                "For local Mock server use AGV_HOST=127.0.0.1. "
                "No default host — to prevent accidental real-robot connections."
            )
        # tcp/api mode: require_host=False allows the resolved host (already checked above)
        return RealBackend(agv_host=host, require_host=False, **kwargs)

    if name == "real":
        # Gate 1: explicit opt-in required
        if os.environ.get("REAL_ROBOT_ENABLED", "false").lower() != "true":
            raise RuntimeError(
                "real backend requires REAL_ROBOT_ENABLED=true. "
                "Default is off to enforce offline-first development."
            )
        # Gate 2: AGV_HOST must be explicitly configured — no default IP
        host = kwargs.pop("agv_host", None) or os.environ.get("AGV_HOST", "")
        if not host:
            raise RuntimeError(
                "real backend requires AGV_HOST to be configured. "
                "No default IP is allowed (M3.9.3 safety rule). "
                "Set AGV_HOST=<vehicle-ip> before enabling real mode."
            )
        # Gate 3: Log critical warning about unverified APIs
        import logging
        logging.getLogger(__name__).critical(
            "REAL BACKEND ENABLED: host=%s — "
            "API 2010 (velocity) UNVERIFIED on real vehicle. "
            "PHYSICAL_TEST_ALLOWED = NO until 2010 is confirmed. "
            "Emergency API 1012 UNVERIFIED on real hardware.",
            host,
        )
        from agv_control.backend.real import RealBackend  # noqa: WPS433
        return RealBackend(agv_host=host, require_host=True, **kwargs)

    raise ValueError(f"unknown backend: {name!r}")
