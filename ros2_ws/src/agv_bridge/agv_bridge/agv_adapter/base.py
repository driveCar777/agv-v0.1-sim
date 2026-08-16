"""Abstract AGV API adapter interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

from agv_bridge.agv_adapter.config import AgvAdapterConfig
from agv_bridge.agv_adapter.models import (
    LaserScan,
    MapInfo,
    NavigationStatus,
    RobotPose,
    RobotState,
    Station,
)


class AgvApiAdapter(ABC):
    """Unified facade over Robokit TCP (demo/mock) or local sim (dev)."""

    def __init__(self, config: AgvAdapterConfig):
        self._config = config
        self._host = ""
        self._connected = False

    @property
    def config(self) -> AgvAdapterConfig:
        return self._config

    @property
    def host(self) -> str:
        return self._host

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    @abstractmethod
    def mode(self) -> str:
        """``dev`` | ``demo`` | ``mock``."""

    @property
    def uses_ros_pose(self) -> bool:
        """When True, dashboard keeps pose from ROS ``agv/status`` subscription."""
        return self.mode == "dev"

    @abstractmethod
    def connect(self, host: Optional[str] = None) -> bool:
        """Establish backend connection (no-op for dev)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release backend resources."""

    # ---- core polling (P1-A minimum) ----

    @abstractmethod
    def get_pose(self) -> RobotPose:
        ...

    @abstractmethod
    def get_laser(self) -> LaserScan:
        ...

    @abstractmethod
    def get_task_status(self) -> NavigationStatus:
        ...

    def poll_robot_state(self) -> RobotState:
        """Convenience bundle for dashboard laser tick."""
        return RobotState(
            pose=self.get_pose(),
            navigation=self.get_task_status(),
            laser=self.get_laser(),
            connected=self.connected,
            host=self.host,
            mode=self.mode,
        )

    # ---- navigation ----

    @abstractmethod
    def goto_station(self, target_id: str, **kwargs: Any) -> Dict[str, Any]:
        ...

    @abstractmethod
    def cancel_navigation(self) -> Dict[str, Any]:
        ...

    def wait_navigation_done(
        self,
        timeout_sec: float = 120.0,
        poll_sec: float = 0.3,
    ) -> Tuple[bool, NavigationStatus]:
        raise NotImplementedError(f"{self.mode} adapter: wait_navigation_done not implemented")

    def navigate_path(self, path: List[str], **kwargs: Any) -> Dict[str, Any]:
        raise NotImplementedError("navigate_path — planned for P1-D")

    # ---- map / station (interface only; dev uses local smap in dashboard) ----

    def get_maps(self) -> List[MapInfo]:
        raise NotImplementedError("get_maps — planned for P1-B")

    def get_current_map(self) -> Optional[MapInfo]:
        raise NotImplementedError("get_current_map — planned for P1-B")

    def switch_map(self, map_name: str) -> Dict[str, Any]:
        raise NotImplementedError("switch_map — planned for P1-B")

    def get_stations(self) -> List[Station]:
        raise NotImplementedError("get_stations — planned for P1-B")

    # ---- future APIs (defined, not implemented in P1-A) ----

    def upload_map(self, path: str, **kwargs: Any) -> Dict[str, Any]:
        raise NotImplementedError("upload_map — planned for P1-E")

    def download_map(self, map_name: str, dest: str) -> Dict[str, Any]:
        raise NotImplementedError("download_map — planned for P1-E")

    def upload_and_switch_map(self, path: str, **kwargs: Any) -> Dict[str, Any]:
        raise NotImplementedError("upload_and_switch_map — planned for P1-E")

    def dynamic_obstacle_add(self, **kwargs: Any) -> Dict[str, Any]:
        raise NotImplementedError("dynamic_obstacle_add — planned for P1-F")

    def dynamic_obstacle_add_world(self, **kwargs: Any) -> Dict[str, Any]:
        raise NotImplementedError("dynamic_obstacle_add_world — planned for P1-F")

    def dynamic_obstacle_remove(self, **kwargs: Any) -> Dict[str, Any]:
        raise NotImplementedError("dynamic_obstacle_remove — planned for P1-F")

    # ---- demo-mode helpers (relocation + lock) ----

    def lock(self, nick_name: Optional[str] = None) -> Dict[str, Any]:
        raise NotImplementedError(f"{self.mode} adapter: lock not supported")

    def unlock(self) -> Dict[str, Any]:
        raise NotImplementedError(f"{self.mode} adapter: unlock not supported")

    def relocate(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        angle: Optional[float] = None,
        length: Optional[float] = None,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        raise NotImplementedError(f"{self.mode} adapter: relocate not supported")

    def wait_reloc_done(
        self,
        timeout_sec: float = 25.0,
        poll_sec: float = 0.4,
        auto_confirm_if_needed: bool = True,
    ) -> Tuple[int, Dict[str, Any]]:
        raise NotImplementedError(f"{self.mode} adapter: wait_reloc_done not supported")

    def get_battery(self) -> Dict[str, Any]:
        return {}
