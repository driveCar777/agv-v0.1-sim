"""Factory for AGV API adapters."""

from __future__ import annotations

from agv_bridge.agv_adapter.base import AgvApiAdapter
from agv_bridge.agv_adapter.config import AgvAdapterConfig
from agv_bridge.agv_adapter.dev import DevAdapter
from agv_bridge.agv_adapter.mock import MockAdapter
from agv_bridge.agv_adapter.real import RealAdapter

_VALID_MODES = frozenset({"dev", "demo", "mock"})


def create_agv_adapter(mode: str, config: AgvAdapterConfig | None = None) -> AgvApiAdapter:
    """Return adapter for ``dev`` | ``demo`` | ``mock``."""
    cfg = config or AgvAdapterConfig()
    m = (mode or "dev").strip().lower()
    if m == "dev":
        return DevAdapter(cfg)
    if m == "demo":
        return RealAdapter(cfg)
    if m == "mock":
        return MockAdapter(cfg)
    raise ValueError(f"unsupported adapter mode {mode!r}; expected dev|demo|mock")
