"""Arm state / command DTOs for Web adapter."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class ArmMode(str, Enum):
    SIMULATION = "simulation"
    REAL = "real"


@dataclass
class JointState:
    names: List[str] = field(default_factory=lambda: [f"joint{i}" for i in range(1, 8)])
    positions_deg: List[float] = field(default_factory=lambda: [0.0] * 7)
    velocities: List[float] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class TcpPose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    rx_deg: float = 0.0
    ry_deg: float = 0.0
    rz_deg: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class GripperState:
    state: str = "UNKNOWN"  # OPEN / CLOSE / UNKNOWN
    raw: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ArmError:
    code: int = 0
    message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ArmState:
    online: bool = False
    mode: str = "simulation"
    motion: str = "OFFLINE"  # IDLE / MOVING / ERROR / OFFLINE
    robot_mode: str = "unknown"
    connected: bool = False
    busy: bool = False
    error: ArmError = field(default_factory=ArmError)
    warning: str = ""
    joints: JointState = field(default_factory=JointState)
    tcp: TcpPose = field(default_factory=TcpPose)
    gripper: GripperState = field(default_factory=GripperState)
    last_text: str = ""
    updated_at: float = 0.0
    stale_ms: Optional[int] = None
    source: str = "mock"
    robot_ip: str = ""
    motion_allowed: bool = False

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["error"] = self.error.to_dict()
        d["joints"] = self.joints.to_dict()
        d["tcp"] = self.tcp.to_dict()
        d["gripper"] = self.gripper.to_dict()
        return d


@dataclass
class ArmCommand:
    kind: str = ""  # move_joint / move_tcp / jog_joint / jog_tcp / stop / enable / disable
    joint_index: int = 0
    delta_deg: float = 0.0
    target_deg: Optional[float] = None
    tcp_axis: str = ""
    delta_mm: float = 0.0
    keep_tcp_fixed: bool = False

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ArmCommand":
        return cls(
            kind=str(data.get("kind", "")),
            joint_index=int(data.get("joint_index", 0)),
            delta_deg=float(data.get("delta_deg", 0.0)),
            target_deg=float(data["target_deg"]) if data.get("target_deg") is not None else None,
            tcp_axis=str(data.get("tcp_axis", "")),
            delta_mm=float(data.get("delta_mm", 0.0)),
            keep_tcp_fixed=bool(data.get("keep_tcp_fixed", False)),
        )
