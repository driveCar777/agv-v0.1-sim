"""底盘控制权状态机（DISCONNECTED → CONNECTED → ...）。"""

from __future__ import annotations

from enum import Enum
from typing import Callable, List, Optional


class ControlPhase(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    REQUEST_CONTROL = "REQUEST_CONTROL"
    CONTROL_GRANTED = "CONTROL_GRANTED"
    READY = "READY"
    MOVING = "MOVING"
    STOPPING = "STOPPING"
    RELEASE = "RELEASE"
    CONTROL_LOST = "CONTROL_LOST"
    API_TIMEOUT = "API_TIMEOUT"
    API_ERROR = "API_ERROR"
    INVALID_RESPONSE = "INVALID_RESPONSE"
    EMERGENCY = "EMERGENCY"


LEGAL_TRANSITIONS = {
    ControlPhase.DISCONNECTED: {ControlPhase.CONNECTED},
    ControlPhase.CONNECTED: {ControlPhase.REQUEST_CONTROL, ControlPhase.API_ERROR},
    ControlPhase.REQUEST_CONTROL: {
        ControlPhase.CONTROL_GRANTED,
        ControlPhase.API_TIMEOUT,
        ControlPhase.API_ERROR,
    },
    ControlPhase.CONTROL_GRANTED: {
        ControlPhase.READY, ControlPhase.RELEASE, ControlPhase.CONTROL_LOST
    },
    ControlPhase.READY: {
        ControlPhase.MOVING, ControlPhase.STOPPING, ControlPhase.EMERGENCY,
        ControlPhase.RELEASE, ControlPhase.CONTROL_LOST,
    },
    ControlPhase.MOVING: {
        ControlPhase.STOPPING, ControlPhase.EMERGENCY,
        ControlPhase.CONTROL_LOST, ControlPhase.READY,
    },
    ControlPhase.STOPPING: {
        ControlPhase.READY, ControlPhase.EMERGENCY,
        ControlPhase.RELEASE, ControlPhase.CONTROL_LOST,
    },
    ControlPhase.RELEASE: {ControlPhase.DISCONNECTED, ControlPhase.REQUEST_CONTROL},
    ControlPhase.CONTROL_LOST: {ControlPhase.STOPPING, ControlPhase.REQUEST_CONTROL},
    ControlPhase.API_TIMEOUT: {ControlPhase.STOPPING, ControlPhase.REQUEST_CONTROL},
    ControlPhase.API_ERROR: {ControlPhase.STOPPING, ControlPhase.RELEASE},
    ControlPhase.INVALID_RESPONSE: {ControlPhase.STOPPING, ControlPhase.RELEASE},
    ControlPhase.EMERGENCY: {ControlPhase.STOPPING, ControlPhase.RELEASE},
}


class ControlStateMachine:
    def __init__(self, on_transition: Optional[Callable] = None) -> None:
        self._phase = ControlPhase.DISCONNECTED
        self._history: List = []
        self._on_transition = on_transition

    @property
    def phase(self) -> ControlPhase:
        return self._phase

    def can_transition(self, target: ControlPhase) -> bool:
        return target in LEGAL_TRANSITIONS.get(self._phase, set())

    def transition(self, target: ControlPhase) -> bool:
        if not self.can_transition(target):
            return False
        self._history.append((self._phase, target))
        self._phase = target
        if self._on_transition:
            try:
                self._on_transition(self._phase)
            except Exception:  # noqa: BLE001
                pass
        return True

    def force(self, target: ControlPhase) -> None:
        """强制设置（异常恢复用），不校验合法性。"""
        self._history.append((self._phase, target))
        self._phase = target
        if self._on_transition:
            try:
                self._on_transition(self._phase)
            except Exception:  # noqa: BLE001
                pass

    def is_safe(self) -> bool:
        return self._phase not in {
            ControlPhase.EMERGENCY,
            ControlPhase.CONTROL_LOST,
            ControlPhase.API_TIMEOUT,
        }