"""Configuration for AGV adapter factory."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class AgvAdapterConfig:
    agv_host: str = "192.168.18.198"
    mock_host: str = "127.0.0.1"
    timeout: float = 3.0
    protocol_version: int = 1
    laser_max_points: int = 800
    maps_dir: Optional[Path] = None
    lock_nickname: str = "ros2_delivery"
