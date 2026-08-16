"""Mock adapter placeholder — same Robokit wire protocol against mock server host."""

from __future__ import annotations

from typing import Optional

from agv_bridge.agv_adapter.config import AgvAdapterConfig
from agv_bridge.agv_adapter.real import RealAdapter


class MockAdapter(RealAdapter):
    """Mock mode (P1-A placeholder): TCP to ``robokit_mock_server`` on mock_host.

    Full mock behaviour is deferred to P1-C; this class reuses RealAdapter so
    partial mock APIs (1004/1009/1020/3051) work when the mock server is running.
    """

    def __init__(self, config: AgvAdapterConfig):
        super().__init__(config)

    @property
    def mode(self) -> str:
        return "mock"

    def connect(self, host: Optional[str] = None) -> bool:
        target = (host or self._config.mock_host or "127.0.0.1").strip()
        return super().connect(target)
