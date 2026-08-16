"""SimBackend — 与 Gazebo/外部仿真器协同。

Phase 2 实现。第一阶段直接复用 MockBackend。
"""

from __future__ import annotations

from agv_control.backend.mock import MockBackend


class SimBackend(MockBackend):
    """仿真后端：默认继承 MockBackend 的运动学。

    后续会接入 Gazebo ROS2 接口（clock、joint_states、odom）。
    """

    @property
    def mode(self) -> str:
        return "sim"