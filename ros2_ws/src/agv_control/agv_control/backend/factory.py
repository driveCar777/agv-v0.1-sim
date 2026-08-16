"""按环境变量 / 参数选择 Backend。

backend:
  mock     — 纯运动学（不经 API）
  sim      — 同 mock
  tcp/api  — 走 Robokit TCP（连 Mock 服务器，验证实车协议）
  real     — 走 Robokit TCP（连真车，需 REAL_ROBOT_ENABLED=true）
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
        from agv_control.backend.real import RealBackend  # noqa: WPS433

        host = kwargs.pop("agv_host", None) or os.environ.get("AGV_HOST", "127.0.0.1")
        return RealBackend(agv_host=host, **kwargs)
    if name == "real":
        if os.environ.get("REAL_ROBOT_ENABLED", "false").lower() != "true":
            raise RuntimeError(
                "real backend requires REAL_ROBOT_ENABLED=true "
                "(default off to enforce offline-first)."
            )
        from agv_control.backend.real import RealBackend  # noqa: WPS433

        host = kwargs.pop("agv_host", None) or os.environ.get("AGV_HOST", "192.168.192.5")
        return RealBackend(agv_host=host, **kwargs)
    raise ValueError(f"unknown backend: {name!r}")
