"""Simulation arm — joint/TCP control for Web closed-loop testing."""

from __future__ import annotations

import math
import threading
import time
from typing import List

from agv_bridge.arm.base import ArmAdapter
from agv_bridge.arm.models import ArmCommand, ArmError, ArmMode, ArmState, GripperState, JointState, TcpPose
from agv_bridge.arm.safety import ArmSafetyLimits, check_motion_allowed, clamp_joint_delta


def _fk_tcp_deg(joints_deg: List[float]) -> TcpPose:
    """Lightweight FK placeholder for mock visualization (not production IK)."""
    j = joints_deg + [0.0] * 7
    base = 350.0
    l1 = 180.0 + j[1] * 2.0
    l2 = 140.0 + j[2] * 1.5
    yaw = math.radians(j[0])
    pitch = math.radians(j[1] + j[2] * 0.5)
    x = base + l1 * math.cos(yaw) * math.cos(pitch) + l2 * math.cos(yaw) * 0.6
    y = l1 * math.sin(yaw) * math.cos(pitch) + l2 * math.sin(yaw) * 0.6
    z = 220.0 + l1 * math.sin(pitch) + j[3] * 2.0
    return TcpPose(
        x_mm=round(x, 1),
        y_mm=round(y, 1),
        z_mm=round(z, 1),
        rx_deg=round(j[4], 1),
        ry_deg=round(j[5], 1),
        rz_deg=round(j[0] + j[6], 1),
    )


class MockArmAdapter(ArmAdapter):
    def __init__(self, robot_ip: str = "172.31.0.123") -> None:
        self._lock = threading.Lock()
        self._limits = ArmSafetyLimits()
        self._robot_ip = robot_ip
        self._joints = [0.0, 15.0, -30.0, 70.0, -168.0, -58.0, 80.0]
        self._gripper = "OPEN"
        self._motion = "IDLE"
        self._busy_until = 0.0
        self._last_text = "Mock arm ready"
        self._updated_at = time.time()
        self._enabled = True

    def _state_unlocked(self) -> ArmState:
        now = time.time()
        if now < self._busy_until:
            motion = "MOVING"
        else:
            motion = "IDLE" if self._enabled else "OFFLINE"
        tcp = _fk_tcp_deg(self._joints)
        return ArmState(
            online=True,
            mode=ArmMode.SIMULATION.value,
            motion=motion,
            robot_mode="Position",
            connected=True,
            busy=motion == "MOVING",
            error=ArmError(),
            warning="",
            joints=JointState(positions_deg=list(self._joints)),
            tcp=tcp,
            gripper=GripperState(state=self._gripper),
            last_text=self._last_text,
            updated_at=self._updated_at,
            stale_ms=int((now - self._updated_at) * 1000),
            source="mock",
            robot_ip=self._robot_ip,
            motion_allowed=True,
        )

    def get_state(self) -> ArmState:
        with self._lock:
            return self._state_unlocked()

    def _apply_joint_delta(self, idx: int, delta: float, keep_tcp: bool) -> None:
        idx = max(0, min(6, int(idx)))
        delta = clamp_joint_delta(ArmCommand(delta_deg=delta), self._limits)
        old_tcp = _fk_tcp_deg(self._joints)
        self._joints[idx] += delta
        if keep_tcp:
            # Mock constraint: nudge other joints slightly to approximate fixed TCP
            others = [i for i in range(7) if i != idx]
            share = -delta / max(len(others), 1)
            for o in others[:3]:
                self._joints[o] += share * 0.35
        new_tcp = _fk_tcp_deg(self._joints)
        self._last_text = (
            f"J{idx+1} {'keep-tcp ' if keep_tcp else ''}Δ{delta:.1f}° "
            f"TCP ({old_tcp.x_mm:.0f},{old_tcp.y_mm:.0f},{old_tcp.z_mm:.0f})"
            f"→({new_tcp.x_mm:.0f},{new_tcp.y_mm:.0f},{new_tcp.z_mm:.0f})"
        )
        self._busy_until = time.time() + 0.35
        self._updated_at = time.time()

    def move_joint(self, cmd: ArmCommand) -> dict:
        ok, msg = check_motion_allowed(ArmMode.SIMULATION, False)
        if not ok:
            return {"success": False, "message": msg}
        with self._lock:
            if cmd.target_deg is not None:
                idx = max(0, min(6, cmd.joint_index))
                delta = float(cmd.target_deg) - self._joints[idx]
                self._apply_joint_delta(idx, delta, cmd.keep_tcp_fixed)
            else:
                self._apply_joint_delta(cmd.joint_index, cmd.delta_deg, cmd.keep_tcp_fixed)
        return {"success": True, "message": "mock joint updated", "state": self.get_state().to_dict()}

    def move_tcp(self, cmd: ArmCommand) -> dict:
        ok, msg = check_motion_allowed(ArmMode.SIMULATION, False)
        if not ok:
            return {"success": False, "message": msg}
        axis = (cmd.tcp_axis or "z").lower()
        delta = max(-self._limits.max_tcp_delta_mm, min(self._limits.max_tcp_delta_mm, cmd.delta_mm))
        with self._lock:
            tcp = _fk_tcp_deg(self._joints)
            if axis == "x":
                self._joints[0] += delta * 0.08
                self._joints[1] += delta * 0.04
            elif axis == "y":
                self._joints[0] += delta * 0.06
            else:
                self._joints[2] += delta * 0.05
                self._joints[3] += delta * 0.03
            self._last_text = f"TCP {axis} Δ{delta:.1f}mm (mock IK)"
            self._busy_until = time.time() + 0.35
            self._updated_at = time.time()
        return {"success": True, "message": "mock tcp updated", "state": self.get_state().to_dict()}

    def stop(self) -> dict:
        with self._lock:
            self._busy_until = 0.0
            self._motion = "IDLE"
            self._last_text = "stopped"
            self._updated_at = time.time()
        return {"success": True, "message": "mock stop"}

    def enable(self) -> dict:
        with self._lock:
            self._enabled = True
            self._last_text = "enabled"
            self._updated_at = time.time()
        return {"success": True, "message": "mock enabled"}

    def disable(self) -> dict:
        with self._lock:
            self._enabled = False
            self._last_text = "disabled"
            self._updated_at = time.time()
        return {"success": True, "message": "mock disabled"}
