"""Mission 状态机 — 纯逻辑，不依赖 ROS2，便于单测。"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Dict, Optional, Set


class MissionPhase(str, enum.Enum):
    IDLE = "IDLE"
    RECEIVED = "RECEIVED"
    GO_TO_PICKUP = "GO_TO_PICKUP"
    ARRIVED_PICKUP = "ARRIVED_PICKUP"
    IDENTIFY_OBJECT = "IDENTIFY_OBJECT"
    PICK = "PICK"
    VERIFY_PICK = "VERIFY_PICK"
    GO_TO_USER = "GO_TO_USER"
    IDENTIFY_USER = "IDENTIFY_USER"
    VERIFY_USER = "VERIFY_USER"
    DELIVER = "DELIVER"
    VERIFY_DELIVER = "VERIFY_DELIVER"
    FINISHED = "FINISHED"

    NAVIGATION_ERROR = "NAVIGATION_ERROR"
    VISION_ERROR = "VISION_ERROR"
    ARM_ERROR = "ARM_ERROR"
    SAFETY_STOP = "SAFETY_STOP"
    TIMEOUT = "TIMEOUT"
    RECOVERY = "RECOVERY"


@dataclass
class Mission:
    mission_id: str = ""
    pickup_pose: tuple = (0.0, 0.0, 0.0)
    user_pose: tuple = (0.0, 0.0, 0.0)
    phase: MissionPhase = MissionPhase.IDLE
    last_error: Optional[str] = None
    history: list = field(default_factory=list)

    def transition(self, next_phase: MissionPhase, reason: str = "") -> bool:
        self.history.append((self.phase.value, next_phase.value, reason))
        self.phase = next_phase
        return True

    def is_terminal(self) -> bool:
        return self.phase in (MissionPhase.FINISHED, MissionPhase.TIMEOUT)


LEGAL_TRANSITIONS: Dict[MissionPhase, Set[MissionPhase]] = {
    MissionPhase.IDLE: {MissionPhase.RECEIVED},
    MissionPhase.RECEIVED: {MissionPhase.GO_TO_PICKUP, MissionPhase.NAVIGATION_ERROR, MissionPhase.TIMEOUT},
    MissionPhase.GO_TO_PICKUP: {
        MissionPhase.ARRIVED_PICKUP, MissionPhase.NAVIGATION_ERROR, MissionPhase.SAFETY_STOP,
    },
    MissionPhase.ARRIVED_PICKUP: {
        MissionPhase.IDENTIFY_OBJECT, MissionPhase.VISION_ERROR, MissionPhase.RECOVERY,
    },
    MissionPhase.IDENTIFY_OBJECT: {MissionPhase.PICK, MissionPhase.VISION_ERROR, MissionPhase.RECOVERY},
    MissionPhase.PICK: {MissionPhase.VERIFY_PICK, MissionPhase.ARM_ERROR, MissionPhase.RECOVERY},
    MissionPhase.VERIFY_PICK: {
        MissionPhase.GO_TO_USER, MissionPhase.ARM_ERROR, MissionPhase.RECOVERY,
    },
    MissionPhase.GO_TO_USER: {
        MissionPhase.IDENTIFY_USER, MissionPhase.NAVIGATION_ERROR, MissionPhase.SAFETY_STOP,
    },
    MissionPhase.IDENTIFY_USER: {
        MissionPhase.DELIVER, MissionPhase.VISION_ERROR, MissionPhase.RECOVERY,
    },
    MissionPhase.VERIFY_USER: {MissionPhase.DELIVER, MissionPhase.VISION_ERROR, MissionPhase.RECOVERY},
    MissionPhase.DELIVER: {MissionPhase.VERIFY_DELIVER, MissionPhase.ARM_ERROR, MissionPhase.RECOVERY},
    MissionPhase.VERIFY_DELIVER: {MissionPhase.FINISHED},
    MissionPhase.RECOVERY: {MissionPhase.GO_TO_PICKUP, MissionPhase.GO_TO_USER, MissionPhase.IDLE},
    MissionPhase.FINISHED: set(),
    MissionPhase.NAVIGATION_ERROR: {MissionPhase.RECOVERY, MissionPhase.TIMEOUT},
    MissionPhase.VISION_ERROR: {MissionPhase.RECOVERY, MissionPhase.TIMEOUT},
    MissionPhase.ARM_ERROR: {MissionPhase.RECOVERY, MissionPhase.TIMEOUT},
    MissionPhase.SAFETY_STOP: {MissionPhase.RECOVERY, MissionPhase.NAVIGATION_ERROR},
    MissionPhase.TIMEOUT: set(),
}


def can_transition(m: Mission, target: MissionPhase) -> bool:
    return target in LEGAL_TRANSITIONS.get(m.phase, set())


def safe_transition(m: Mission, target: MissionPhase, reason: str = "") -> bool:
    """合法才转，否则返回 False。"""
    if not can_transition(m, target):
        return False
    return m.transition(target, reason)