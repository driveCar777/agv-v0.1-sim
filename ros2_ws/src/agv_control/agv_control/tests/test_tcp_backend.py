"""RealBackend 离线单测：对着本地 Mock TCP 验证 3055 链路。"""

from __future__ import annotations

import os
import socket
import threading
import time

import pytest

# 仅当本机 Mock 端口可连时跑集成测
def _port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


@pytest.mark.skipif(
    not _port_open("127.0.0.1", 19204),
    reason="robokit mock not running on 19204",
)
def test_tcp_backend_against_mock():
    from agv_control.backend.real import RealBackend
    from agv_control.backend.base import Twist2D

    b = RealBackend(agv_host="127.0.0.1", timeout=1.0)
    assert b.connect()
    assert b.request_control("pytest")
    b.set_velocity(Twist2D(vx=0.1, w=0.0))
    time.sleep(0.3)
    odom = b.get_odom()
    assert isinstance(odom.x, float)
    b.stop()
    b.release_control()
    b.disconnect()


def test_factory_tcp_default_host(monkeypatch):
    monkeypatch.setenv("AGV_BACKEND", "tcp")
    monkeypatch.setenv("AGV_HOST", "127.0.0.1")
    from agv_control.backend.factory import create_backend

    b = create_backend("tcp", agv_host="127.0.0.1")
    assert b.mode == "real"
