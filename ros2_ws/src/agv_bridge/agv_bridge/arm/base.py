"""Abstract arm adapter."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict

from agv_bridge.arm.models import ArmCommand, ArmState


class ArmAdapter(ABC):
    @abstractmethod
    def get_state(self) -> ArmState:
        ...

    def get_joint_state(self) -> Dict[str, Any]:
        return self.get_state().joints.to_dict()

    def get_tcp_pose(self) -> Dict[str, Any]:
        return self.get_state().tcp.to_dict()

    def get_gripper_state(self) -> Dict[str, Any]:
        return self.get_state().gripper.to_dict()

    def move_joint(self, cmd: ArmCommand) -> Dict[str, Any]:
        return {"success": False, "message": "not implemented"}

    def move_tcp(self, cmd: ArmCommand) -> Dict[str, Any]:
        return {"success": False, "message": "not implemented"}

    def jog_joint(self, cmd: ArmCommand) -> Dict[str, Any]:
        return self.move_joint(cmd)

    def jog_tcp(self, cmd: ArmCommand) -> Dict[str, Any]:
        return self.move_tcp(cmd)

    def stop(self) -> Dict[str, Any]:
        return {"success": False, "message": "not implemented"}

    def enable(self) -> Dict[str, Any]:
        return {"success": False, "message": "not implemented"}

    def disable(self) -> Dict[str, Any]:
        return {"success": False, "message": "not implemented"}
