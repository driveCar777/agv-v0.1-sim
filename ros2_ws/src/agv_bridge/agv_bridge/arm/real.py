"""Real xArm7 adapter — READ-ONLY state until motion explicitly authorized."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Optional

from agv_bridge.arm.base import ArmAdapter
from agv_bridge.arm.models import ArmCommand, ArmError, ArmMode, ArmState, GripperState, JointState, TcpPose
from agv_bridge.arm.safety import check_motion_allowed


class RealArmAdapter(ArmAdapter):
    """Bridges pick_place_node.py (V4.0) ROS topics — no motion without authorization."""

    def __init__(
        self,
        node: Any,
        robot_ip: str = "172.31.0.123",
        real_motion_enabled: bool = False,
    ) -> None:
        self._node = node
        self._robot_ip = robot_ip
        self._real_motion_enabled = real_motion_enabled
        self._lock = threading.Lock()
        self._joints_deg: List[float] = [0.0] * 7
        self._last_text = ""
        self._updated_at = 0.0
        self._online = False
        self._busy = False
        self._gripper = "UNKNOWN"
        self._tcp = TcpPose()
        self._sub = None
        self._state_sub = None
        try:
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String

            self._sub = node.create_subscription(
                JointState,
                "/joint_states",
                self._on_joints,
                10,
            )
            self._state_sub = node.create_subscription(
                String,
                "/pick_place_state",
                self._on_state_text,
                10,
            )
        except Exception:  # noqa: BLE001
            pass

    def _on_joints(self, msg) -> None:
        with self._lock:
            if msg.position:
                self._joints_deg = [float(p) * 180.0 / math.pi for p in msg.position[:7]]
                while len(self._joints_deg) < 7:
                    self._joints_deg.append(0.0)
            self._online = True
            self._updated_at = time.time()

    def _on_state_text(self, msg) -> None:
        with self._lock:
            self._last_text = str(msg.data or "")
            if "CLOSE" in self._last_text.upper():
                self._gripper = "CLOSE"
            elif "OPEN" in self._last_text.upper():
                self._gripper = "OPEN"
            if "BUSY" in self._last_text.upper() or "Studio" in self._last_text:
                self._busy = True
            self._updated_at = time.time()

    def get_state(self) -> ArmState:
        with self._lock:
            now = time.time()
            stale = int((now - self._updated_at) * 1000) if self._updated_at > 0 else None
            online = self._online and stale is not None and stale < 5000
            motion = "MOVING" if self._busy else ("IDLE" if online else "OFFLINE")
            return ArmState(
                online=online,
                mode=ArmMode.REAL.value,
                motion=motion,
                robot_mode="Position",
                connected=online,
                busy=self._busy,
                error=ArmError(),
                warning="" if online else "No /joint_states",
                joints=JointState(positions_deg=list(self._joints_deg)),
                tcp=self._tcp,
                gripper=GripperState(state=self._gripper, raw=self._last_text),
                last_text=self._last_text,
                updated_at=self._updated_at,
                stale_ms=stale,
                source="real",
                robot_ip=self._robot_ip,
                motion_allowed=self._real_motion_enabled,
            )

    def _blocked(self, action: str) -> Dict[str, Any]:
        return {
            "success": False,
            "message": f"REAL {action} blocked — awaiting explicit motion authorization",
            "blocked": True,
        }

    def move_joint(self, cmd: ArmCommand) -> Dict[str, Any]:
        ok, msg = check_motion_allowed(ArmMode.REAL, self._real_motion_enabled)
        if not ok:
            return self._blocked("move_joint")
        return self._blocked("move_joint")

    def move_tcp(self, cmd: ArmCommand) -> Dict[str, Any]:
        return self._blocked("move_tcp")

    def stop(self) -> Dict[str, Any]:
        return self._blocked("stop")

    def enable(self) -> Dict[str, Any]:
        return self._blocked("enable")

    def disable(self) -> Dict[str, Any]:
        return self._blocked("disable")
