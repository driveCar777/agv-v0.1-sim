"""ModbusTCP 客户端 — 写开环速度寄存器 + 读位置。

Phase 5 接入真车。第一阶段只作占位接口（用 pymodbus stub）。
"""

from __future__ import annotations

import logging
from typing import Optional


LOG = logging.getLogger(__name__)


class ModbusClient:
    """ModbusTCP 客户端封装。

    真实部署需要 pymodbus：
        pip install pymodbus==3.6.9

    寄存器地址必须从项目原始 `读写寄存器[4x].docx` 校对，
    本文件只占位，不写死寄存器号。
    """

    def __init__(
        self,
        host: str = "192.168.192.5",
        port: int = 502,
        timeout: float = 1.0,
        slave_id: Optional[int] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.slave_id = slave_id
        self._client = None

    def connect(self) -> bool:
        try:
            from pymodbus.client import ModbusTcpClient  # type: ignore

            self._client = ModbusTcpClient(
                host=self.host, port=self.port, timeout=self.timeout
            )
            return bool(self._client.connect())
        except Exception as exc:  # noqa: BLE001
            LOG.warning("pymodbus not available: %s (stub only)", exc)
            self._client = None
            return False

    def close(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass

    def write_open_loop_velocity(self, vx: float, vy: float, w: float) -> bool:
        """写开环速度寄存器。

        必须先核对 4x 寄存器表。本接口仅作占位。
        """
        if self._client is None:
            LOG.debug("modbus stub: vx=%.3f vy=%.3f w=%.3f", vx, vy, w)
            return True
        LOG.warning(
            "open_loop_velocity register addresses need to be confirmed against"
            " 读写寄存器[4x].docx — not yet wired (Phase 5)"
        )
        return False

    def read_position(self) -> Optional[tuple]:
        """读 3x 位置寄存器（机器人 x/y/angle）。

        寄存器地址必须从 `只读寄存器[3x].docx` 核对。
        """
        if self._client is None:
            return None
        return None