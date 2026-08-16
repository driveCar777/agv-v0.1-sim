"""Development / Gazebo adapter — ROS pose + local smap; no Robokit TCP."""

from __future__ import annotations

from typing import Any, Dict, Optional

from agv_bridge.agv_adapter.base import AgvApiAdapter
from agv_bridge.agv_adapter.config import AgvAdapterConfig
from agv_bridge.agv_adapter.models import LaserScan, NavigationStatus, RobotPose


class DevAdapter(AgvApiAdapter):
    """Dev mode: dashboard uses ROS ``agv/status`` and ``smap_sim`` laser stub."""

    def __init__(self, config: AgvAdapterConfig):
        super().__init__(config)

    @property
    def mode(self) -> str:
        return "dev"

    def connect(self, host: Optional[str] = None) -> bool:
        self._host = host or "localhost"
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False
        self._host = ""

    def get_pose(self) -> RobotPose:
        return RobotPose()

    def get_laser(self) -> LaserScan:
        return LaserScan(ok=False, source="dev_ros", message="laser handled by dashboard sim stub")

    def get_task_status(self) -> NavigationStatus:
        return NavigationStatus()

    def goto_station(self, target_id: str, **kwargs: Any) -> Dict[str, Any]:
        return {"success": False, "message": "dev mode uses ROS agv/navigate service"}

    def cancel_navigation(self) -> Dict[str, Any]:
        return {"success": False, "message": "dev mode uses ROS agv/cancel service"}
