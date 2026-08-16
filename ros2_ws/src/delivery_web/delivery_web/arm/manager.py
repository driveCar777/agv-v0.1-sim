"""Arm manager — mock simulation + real xArm7 via ArmBridge."""

from __future__ import annotations

import os
import threading
from typing import Any, Callable, Dict, Optional

from rclpy.node import Node

from agv_bridge.arm.factory import create_arm_adapter
from agv_bridge.arm.models import ArmCommand, ArmMode
from agv_bridge.arm_bridge import ArmBridge


class ArmManager:
    def __init__(
        self,
        node: Node,
        get_env_mode: Callable[[], str],
        robot_ip: str = "172.31.0.123",
        get_agv_busy: Optional[Callable[[], bool]] = None,
    ) -> None:
        self._node = node
        self._get_env_mode = get_env_mode
        self._get_agv_busy = get_agv_busy or (lambda: False)
        self._robot_ip = robot_ip
        self._lock = threading.Lock()
        self._arm_mode = os.environ.get("ARM_MODE", "simulation").strip().lower()
        self._real_motion_enabled = os.environ.get("ARM_REAL_MOTION", "0") == "1"
        self._adapter = None
        self._bridge: Optional[ArmBridge] = None
        self._refresh_adapter()

    def _ensure_bridge(self) -> ArmBridge:
        if self._bridge is None:
            self._bridge = ArmBridge(self._node, robot_ip=self._robot_ip)
        return self._bridge

    def _refresh_adapter(self) -> None:
        use_real = self._arm_mode in ("real", "hardware", "live")
        if use_real:
            self._ensure_bridge()
            self._adapter = create_arm_adapter(
                "real",
                node=self._node,
                robot_ip=self._robot_ip,
                real_motion_enabled=self._real_motion_enabled,
            )
        else:
            self._adapter = create_arm_adapter("simulation", robot_ip=self._robot_ip)

    def set_arm_mode(self, mode: str) -> Dict[str, Any]:
        with self._lock:
            self._arm_mode = (mode or "simulation").strip().lower()
            self._refresh_adapter()
        return {"success": True, "arm_mode": self._arm_mode, "real_motion_enabled": self._real_motion_enabled}

    def _is_real(self) -> bool:
        return self._arm_mode in ("real", "hardware", "live")

    def _normalize_mock_status(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        motion = str(raw.get("motion", "OFFLINE")).upper()
        state = "idle"
        if not raw.get("online"):
            state = "offline"
        elif raw.get("busy") or motion == "MOVING":
            state = "running"
        elif motion == "ERROR":
            state = "error"
        joints = raw.get("joints") or {}
        grip = raw.get("gripper") or {}
        gs = str(grip.get("state", "unknown")).lower()
        if gs in ("open", "opened"):
            gripper_state = "open"
        elif gs in ("close", "closed"):
            gripper_state = "closed"
        else:
            gripper_state = "unknown"
        out = dict(raw)
        out.update({
            "state": state,
            "joints_deg": list(joints.get("positions_deg") or []),
            "gripper_state": gripper_state,
            "is_busy": bool(raw.get("busy")),
            "state_text": raw.get("last_text") or "mock idle",
            "current_block": None,
            "state_history": [],
        })
        return out

    def status(self) -> Dict[str, Any]:
        if self._is_real():
            st = self._ensure_bridge().get_status()
        else:
            with self._lock:
                assert self._adapter is not None
                st = self._normalize_mock_status(self._adapter.get_state().to_dict())
        st["arm_mode"] = self._arm_mode
        st["real_motion_enabled"] = self._real_motion_enabled
        return st

    def pose(self) -> Dict[str, Any]:
        if self._is_real():
            return self._ensure_bridge().get_pose()
        with self._lock:
            assert self._adapter is not None
            s = self._adapter.get_state()
            return {
                "online": s.online,
                "joints_deg": list(s.joints.positions_deg),
                "joints_rad": [d * 3.14159265 / 180.0 for d in s.joints.positions_deg],
                "joint_names": list(s.joints.names),
            }

    def _motion_gate(self) -> Optional[Dict[str, Any]]:
        if not self._real_motion_enabled and self._is_real():
            return {
                "success": False,
                "message": "REAL motion blocked — set ARM_REAL_MOTION=1 after safety approval",
                "blocked": True,
            }
        if self._get_agv_busy():
            return {"success": False, "message": "AGV 正在导航中，请先停车"}
        return None

    def unlock(self) -> Dict[str, Any]:
        gate = self._motion_gate()
        if gate:
            return gate
        if self._is_real():
            return self._ensure_bridge().call_unlock()
        return {"success": True, "message": "mock unlock OK", "mock": True}

    def pick_place(self, cycles: int = 1) -> Dict[str, Any]:
        gate = self._motion_gate()
        if gate:
            return gate
        if self._is_real():
            return self._ensure_bridge().call_pick_place(cycles=cycles)
        with self._lock:
            assert self._adapter is not None
        return {
            "success": True,
            "message": f"Mock pick_place x{cycles}",
            "mock": True,
            "duration_s": 0.1,
            "warning": "Simulation only",
        }

    def gripper_get(self) -> Dict[str, Any]:
        if self._is_real():
            st = self._ensure_bridge().get_status()
            return {"state": st.get("gripper_state", "unknown")}
        return {"state": "open", "mock": True}

    def gripper_set(self, open_cmd: bool) -> Dict[str, Any]:
        gate = self._motion_gate()
        if gate:
            return gate
        if self._is_real():
            return self._ensure_bridge().call_gripper(open_cmd)
        return {"success": True, "message": f"mock gripper {'open' if open_cmd else 'close'}", "mock": True}

    def set_cycles(self, cycles: int) -> Dict[str, Any]:
        if self._is_real():
            return self._ensure_bridge().set_cycles(cycles)
        return {"success": True, "cycles": cycles, "mock": True}

    def stop(self) -> Dict[str, Any]:
        if self._is_real():
            return self._ensure_bridge().call_stop()
        with self._lock:
            assert self._adapter is not None
            return self._adapter.stop()

    def command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        cmd = ArmCommand.from_dict(payload)
        with self._lock:
            assert self._adapter is not None
            kind = cmd.kind.lower()
            if kind == "move_joint":
                return self._adapter.move_joint(cmd)
            if kind == "move_tcp":
                return self._adapter.move_tcp(cmd)
            if kind == "jog_joint":
                return self._adapter.jog_joint(cmd)
            if kind == "jog_tcp":
                return self._adapter.jog_tcp(cmd)
            if kind == "stop":
                return self.stop()
            if kind == "enable":
                return self._adapter.enable()
            if kind == "disable":
                return self._adapter.disable()
        return {"success": False, "message": f"unknown arm command: {cmd.kind}"}
