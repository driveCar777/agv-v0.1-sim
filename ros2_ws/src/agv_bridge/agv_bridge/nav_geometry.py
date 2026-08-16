"""统一车辆几何 / 碰撞 / 安全阈值（Web 仿真）。

差异必须来自此配置，禁止再散落魔法数字。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class VehicleGeometry:
    """AMB-150 近似。

    - planner_radius：A* 膨胀栅格上的圆足迹
    - local_radius：局部 MPPI 碰撞圆（略小于 planner，避免双重过保守）
    - safety_radius：物理层机身碰撞守卫
    """

    length: float = 1.05
    width: float = 0.55
    bumper_l: float = 0.55  # 中心到头/尾
    planner_radius: float = 0.25
    local_radius: float = 0.24
    safety_radius: float = 0.28
    # Safety Supervisor（最终裁决）
    front_stop_m: float = 0.70
    front_clear_m: float = 1.20
    rear_stop_m: float = 0.55
    # MPPI 仅作代价软提示，不最终停车
    front_cost_m: float = 0.90
    # 速度限制
    max_vx: float = 0.40
    max_w: float = 0.45
    acc_v: float = 0.9
    acc_w: float = 0.55

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


DEFAULT_GEOM = VehicleGeometry()

# Stuck / Recovery（dt=0.05、局部 0.35s、巡航~0.15m/s）
# 8s 内沿路径至少前进 0.30m；否则视为无进展
STUCK_NO_PROGRESS_S = 8.0
STUCK_PROGRESS_MIN_M = 0.30
STUCK_REVERSE_TRIGGER_S = 8.0
REVERSE_MAX_S = 4.0
RECOVERY_COOLDOWN_S = 8.0
MAX_RECOVERY_ATTEMPTS = 3
GLOBAL_SPLICE_S = 25.0
GLOBAL_ALT_S = 45.0
