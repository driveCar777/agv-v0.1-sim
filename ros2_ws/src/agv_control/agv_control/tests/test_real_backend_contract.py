"""M3.9.3 — Unit tests: Real Backend API Contract.

All tests use mock sockets. No real vehicle required.
PASS here means CODE CONTRACT PASS, not physical vehicle PASS.
"""

from __future__ import annotations

import io
import json
import os
import socket
import struct
import threading
from typing import Any, Dict, Optional, Tuple
from unittest.mock import MagicMock, patch

import pytest

from agv_control.transport.tcp_client import (
    TcpApiClient,
    API_EMERGENCY,
    API_VELOCITY,
    API_TRANSLATE_TASK,
    API_ROTATE_TASK,
    API_SOFT_STOP,
    PORT_CONTROL,
    PORT_STATUS,
    PORT_CONFIG,
    HEADER_FMT,
    HEADER_SIZE,
)
from agv_control.backend.base import Twist2D
from agv_control.backend.real import RealBackend
from agv_control.backend.factory import create_backend


# ---- Helpers ----

def _make_response(api_type: int, payload: Dict[str, Any] = None, ret_code: int = 0) -> bytes:
    """Build a Robokit-format response header + JSON body."""
    body = json.dumps({**(payload or {}), "ret_code": ret_code}).encode("utf-8")
    header = struct.pack(HEADER_FMT, 0x5A, 1, 0, len(body), api_type + 10000, b"\x00" * 6)
    return header + body


def _mock_socket_response(response_bytes: bytes):
    """Context manager that patches socket.create_connection to return canned response."""
    mock_sock = MagicMock()
    mock_sock.__enter__ = lambda s: s
    mock_sock.__exit__ = MagicMock(return_value=False)
    buf = io.BytesIO(response_bytes)
    mock_sock.recv = lambda n: buf.read(n)
    mock_sock.sendall = MagicMock()
    mock_sock.settimeout = MagicMock()
    return patch("socket.create_connection", return_value=mock_sock)


# ============================================================
# CHECK 1: Emergency API is 1012, NOT 1011
# ============================================================

def test_emergency_api_id_is_1012():
    """CRITICAL: Emergency API must be 1012. Using 1011 is a P0 contract bug."""
    assert API_EMERGENCY == 1012, f"Emergency API must be 1012, got {API_EMERGENCY}"


def test_wrong_emergency_api_documented():
    """Tombstone 1011 must exist as documentation."""
    from agv_control.transport.tcp_client import _WRONG_EMERGENCY_1011
    assert _WRONG_EMERGENCY_1011 == 1011


# ============================================================
# CHECK 2: 2010 velocity payload
# ============================================================

def test_2010_velocity_payload():
    """set_velocity_cmd sends correct fields to API 2010."""
    client = TcpApiClient("127.0.0.1")
    captured = {}

    def fake_call(port, api_type, payload=None):
        captured["port"] = port
        captured["api"] = api_type
        captured["payload"] = payload
        return {"ret_code": 0}, {"latency_ms": 1.0, "result": "ok", "api_id": api_type, "tcp_port": port, "request": payload, "request_ts": 0, "response_ts": 0, "error": None}

    client.call = fake_call
    ok, tel = client.set_velocity_cmd(vx=0.2, vy=0.0, w=0.1, duration=0)

    assert captured["api"] == API_VELOCITY == 2010, f"Must use API 2010, got {captured.get('api')}"
    assert captured["port"] == PORT_CONTROL
    p = captured["payload"]
    assert "vx" in p
    assert "vy" in p
    assert "w" in p
    assert "duration" in p
    assert ok is True


def test_2010_does_not_call_3055():
    """set_velocity_cmd must NEVER call API 3055 (translate task)."""
    client = TcpApiClient("127.0.0.1")
    calls = []

    def fake_call(port, api_type, payload=None):
        calls.append(api_type)
        return {"ret_code": 0}, {"latency_ms": 1.0, "result": "ok", "api_id": api_type, "tcp_port": port, "request": payload, "request_ts": 0, "response_ts": 0, "error": None}

    client.call = fake_call
    client.set_velocity_cmd(vx=0.1, vy=0.0, w=0.05)
    assert API_TRANSLATE_TASK not in calls, "set_velocity_cmd must not call 3055 (translate task)"
    assert API_VELOCITY in calls


# ============================================================
# CHECK 3: 3055 translate_distance payload
# ============================================================

def test_3055_payload():
    """translate_distance sends {dist, vx, mode} to API 3055."""
    client = TcpApiClient("127.0.0.1")
    captured = {}

    def fake_call(port, api_type, payload=None):
        captured["api"] = api_type
        captured["payload"] = payload
        return {"ret_code": 0}, {"latency_ms": 1.0, "result": "ok", "api_id": api_type, "tcp_port": port, "request": payload, "request_ts": 0, "response_ts": 0, "error": None}

    client.call = fake_call
    ok, tel = client.translate_distance(dist=1.0, vx=0.3, mode=0)

    assert captured["api"] == API_TRANSLATE_TASK == 3055
    p = captured["payload"]
    assert "dist" in p, "3055 payload must contain 'dist'"
    assert "vx" in p, "3055 payload must contain 'vx'"
    assert "mode" in p, "3055 payload must contain 'mode'"
    assert "w" not in p, "3055 payload must NOT contain 'w' (velocity omega)"
    assert "vy" not in p, "3055 payload must NOT contain 'vy'"
    assert ok is True


def test_translate_distance_no_omega_param():
    """translate_distance must not accept omega/w as control parameter."""
    import inspect
    sig = inspect.signature(TcpApiClient.translate_distance)
    params = list(sig.parameters.keys())
    assert "w" not in params, "translate_distance must not accept 'w' param"
    assert "omega" not in params, "translate_distance must not accept 'omega' param"


# ============================================================
# CHECK 4: 3056 rotate_angle payload
# ============================================================

def test_3056_payload():
    """rotate_angle sends {angle, vw, mode} to API 3056."""
    client = TcpApiClient("127.0.0.1")
    captured = {}

    def fake_call(port, api_type, payload=None):
        captured["api"] = api_type
        captured["payload"] = payload
        return {"ret_code": 0}, {"latency_ms": 1.0, "result": "ok", "api_id": api_type, "tcp_port": port, "request": payload, "request_ts": 0, "response_ts": 0, "error": None}

    client.call = fake_call
    ok, tel = client.rotate_angle(angle=1.57, vw=0.3, mode=0)

    assert captured["api"] == API_ROTATE_TASK == 3056
    p = captured["payload"]
    assert "angle" in p, "3056 payload must contain 'angle'"
    assert "vw" in p, "3056 payload must contain 'vw'"
    assert "mode" in p, "3056 payload must contain 'mode'"
    assert "w" not in p, "3056 payload must NOT contain bare 'w'"
    assert ok is True


def test_rotate_angle_requires_angle_and_vw():
    """rotate_angle must require angle and vw, not just w."""
    import inspect
    sig = inspect.signature(TcpApiClient.rotate_angle)
    params = list(sig.parameters.keys())
    assert "angle" in params
    assert "vw" in params


# ============================================================
# CHECK 5: stop() — no 3055 fallback
# ============================================================

def test_stop_no_invalid_fallback():
    """stop() must return STOP_FAILED when 2000 fails. No 3055 fallback."""
    client = TcpApiClient("127.0.0.1")
    apis_called = []

    def fake_call(port, api_type, payload=None):
        apis_called.append(api_type)
        # Simulate 2000 failure
        return None, {"latency_ms": 1.0, "result": "exception", "api_id": api_type, "tcp_port": port, "request": payload, "request_ts": 0, "response_ts": 0, "error": "timeout"}

    client.call = fake_call
    # Override _call to use fake_call
    client._call = lambda port, api_type, payload=None: fake_call(port, api_type, payload)[0]

    ok, status = client.stop()
    assert ok is False, "stop() must return False when 2000 fails"
    assert status == "STOP_FAILED", f"stop() must return 'STOP_FAILED', got {status!r}"
    assert API_TRANSLATE_TASK not in apis_called, (
        "stop() must NOT call 3055 (translate_distance) as fallback. "
        f"APIs called: {apis_called}"
    )


def test_stop_ok_when_2000_succeeds():
    """stop() returns True, STOP_OK when 2000 succeeds."""
    client = TcpApiClient("127.0.0.1")

    def fake_call(port, api_type, payload=None):
        return {"ret_code": 0}, {"latency_ms": 1.0, "result": "ok", "api_id": api_type, "tcp_port": port, "request": payload, "request_ts": 0, "response_ts": 0, "error": None}

    client._call = lambda port, api_type, payload=None: fake_call(port, api_type, payload)[0]
    ok, status = client.stop()
    assert ok is True
    assert status == "STOP_OK"


# ============================================================
# CHECK 6: Missing AGV_HOST blocks real mode
# ============================================================

def test_missing_agv_host_blocks_real_mode(monkeypatch):
    """RealBackend and factory must refuse to start real mode without AGV_HOST."""
    monkeypatch.delenv("AGV_HOST", raising=False)
    monkeypatch.setenv("REAL_ROBOT_ENABLED", "true")

    with pytest.raises((ValueError, RuntimeError)):
        create_backend("real")


def test_missing_agv_host_blocks_tcp_mode(monkeypatch):
    """factory must refuse tcp mode without AGV_HOST."""
    monkeypatch.delenv("AGV_HOST", raising=False)

    with pytest.raises((ValueError, RuntimeError)):
        create_backend("tcp")


# ============================================================
# CHECK 7: REAL_ROBOT_ENABLED gate
# ============================================================

def test_unknown_emergency_api_blocks_real_mode(monkeypatch):
    """factory blocks real mode without REAL_ROBOT_ENABLED=true."""
    monkeypatch.delenv("REAL_ROBOT_ENABLED", raising=False)
    monkeypatch.setenv("AGV_HOST", "192.168.1.100")

    with pytest.raises((RuntimeError, ValueError)):
        create_backend("real")


# ============================================================
# CHECK 8: API port mapping
# ============================================================

def test_api_port_mapping():
    """Verify all APIs are mapped to correct ports."""
    from agv_control.transport.tcp_client import PORT_STATUS, PORT_CONTROL, PORT_CONFIG
    assert PORT_STATUS == 19204
    assert PORT_CONTROL == 19205
    assert PORT_CONFIG == 19207


# ============================================================
# CHECK 9: timeout disables motion
# ============================================================

def test_timeout_disables_motion():
    """After stop() returns STOP_FAILED, set_velocity() must be blocked."""
    backend = RealBackend.__new__(RealBackend)
    from threading import RLock
    backend._lock = RLock()
    backend._has_control = True
    backend._emergency = False
    backend._stop_failed = True
    backend._last_cmd = Twist2D()
    backend._last_cmd_ts = 0.0
    backend._cmd_sequence = 0
    backend._client = MagicMock()

    # set_velocity should NOT call set_velocity_cmd because stop_failed=True
    backend.set_velocity(Twist2D(vx=0.2, w=0.0))
    backend._client.set_velocity_cmd.assert_not_called()


# ============================================================
# CHECK 10: mock backend stays mock (no auto-switch to real)
# ============================================================

def test_mock_backend_does_not_auto_switch_real(monkeypatch):
    """Default create_backend() returns MockBackend, never RealBackend."""
    monkeypatch.delenv("AGV_BACKEND", raising=False)
    b = create_backend()
    assert b.mode == "mock", f"Default backend must be 'mock', got {b.mode!r}"


def test_mock_backend_explicit(monkeypatch):
    """Explicit 'mock' backend returns MockBackend."""
    b = create_backend("mock")
    assert b.mode == "mock"


if __name__ == "__main__":
    import subprocess, sys
    result = subprocess.run(
        [sys.executable, "-m", "pytest", __file__, "-v", "--tb=short"],
        cwd=str(Path(__file__).parent.parent.parent.parent.parent),
    )
    sys.exit(result.returncode)
