"""安全看门狗：cmd_vel stale / 急停 / 控制权丢失 → 安全停止。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable


class SafetyState(str, Enum):
    NORMAL = "NORMAL"
    CMD_STALE = "CMD_STALE"
    CONTROL_LOST = "CONTROL_LOST"
    EMERGENCY = "EMERGENCY"
    SAFE_STOP = "SAFE_STOP"


@dataclass
class WatchdogConfig:
    cmd_stale_ms: int = 500
    control_lost_check_period_s: float = 1.0


class Watchdog:
    def __init__(
        self,
        cfg: WatchdogConfig,
        has_control: Callable[[], bool],
        on_safe_stop: Callable[[], None],
        clock_s: Callable[[], float],
    ) -> None:
        self._cfg = cfg
        self._has_control = has_control
        self._on_safe_stop = on_safe_stop
        self._clock = clock_s
        self._state = SafetyState.NORMAL
        self._last_cmd_stamp = clock_s()
        self._last_control_check = clock_s()

    def feed_cmd(self) -> None:
        self._last_cmd_stamp = self._clock()

    @property
    def state(self) -> SafetyState:
        return self._state

    def tick(self) -> SafetyState:
        now = self._clock()
        age_ms = (now - self._last_cmd_stamp) * 1000.0

        if now - self._last_control_check > self._cfg.control_lost_check_period_s:
            if not self._has_control():
                self._state = SafetyState.CONTROL_LOST
            self._last_control_check = now

        if age_ms > self._cfg.cmd_stale_ms:
            self._state = SafetyState.CMD_STALE

        if self._state in (
            SafetyState.CMD_STALE,
            SafetyState.CONTROL_LOST,
            SafetyState.EMERGENCY,
        ):
            self._on_safe_stop()
            self._state = SafetyState.SAFE_STOP

        return self._state

    def trigger_emergency(self) -> None:
        self._state = SafetyState.EMERGENCY

    def reset(self) -> None:
        self._state = SafetyState.NORMAL
        self._last_cmd_stamp = self._clock()