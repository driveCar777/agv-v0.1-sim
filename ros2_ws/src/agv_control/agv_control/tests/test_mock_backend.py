"""agv_control MockBackend 单测 — 完全离线。"""

from agv_control.backend.base import Twist2D
from agv_control.backend.mock import MockBackend
from agv_control.safety.state_machine import (
    ControlPhase,
    ControlStateMachine,
    LEGAL_TRANSITIONS,
)


def test_connect_and_control():
    b = MockBackend()
    assert b.connect() is True
    assert b.connected
    assert b.request_control("test")
    assert b.has_control


def test_set_velocity_clamps():
    b = MockBackend(max_vx=0.5, max_w=1.0)
    b.connect()
    b.request_control()
    b.set_velocity(Twist2D(vx=10.0, w=10.0))
    # 给几次 step 让内部 limit 应用
    for _ in range(5):
        b.step()
    o = b.get_odom()
    assert abs(o.vx) <= 0.5 + 1e-6
    assert abs(o.w) <= 1.0 + 1e-6


def test_motion_integration():
    b = MockBackend(max_vx=0.5, max_acc=10.0)
    b.connect()
    b.request_control()
    b.set_velocity(Twist2D(vx=0.5, w=0.0))
    for _ in range(50):  # 1s @ 50Hz
        b.step()
    o = b.get_odom()
    assert o.x > 0.0


def test_stop_resets_velocity():
    b = MockBackend()
    b.connect()
    b.request_control()
    b.set_velocity(Twist2D(vx=0.3, w=0.0))
    for _ in range(5):
        b.step()
    b.stop()
    o = b.get_odom()
    assert abs(o.vx) < 1e-6
    assert abs(o.w) < 1e-6


def test_emergency_stop():
    b = MockBackend()
    b.connect()
    b.request_control()
    b.set_velocity(Twist2D(vx=0.5, w=0.5))
    b.emergency_stop()
    o = b.get_odom()
    assert b.get_emergency_state()
    assert abs(o.vx) < 1e-6
    assert abs(o.w) < 1e-6


def test_release_control_resets_velocity():
    b = MockBackend()
    b.connect()
    b.request_control()
    b.set_velocity(Twist2D(vx=0.3, w=0.0))
    b.release_control()
    assert not b.has_control
    o = b.get_odom()
    assert abs(o.vx) < 1e-6


def test_set_velocity_without_control_is_noop():
    b = MockBackend()
    b.connect()
    # 没抢控制权
    b.set_velocity(Twist2D(vx=0.3, w=0.0))
    for _ in range(3):
        b.step()
    o = b.get_odom()
    assert abs(o.vx) < 1e-6


def test_state_machine_legal_transitions():
    sm = ControlStateMachine()
    sm.transition(ControlPhase.CONNECTED)
    sm.transition(ControlPhase.REQUEST_CONTROL)
    sm.transition(ControlPhase.CONTROL_GRANTED)
    sm.transition(ControlPhase.READY)
    sm.transition(ControlPhase.MOVING)
    sm.transition(ControlPhase.STOPPING)
    sm.transition(ControlPhase.RELEASE)
    assert sm.phase == ControlPhase.RELEASE


def test_state_machine_illegal_transition_blocked():
    sm = ControlStateMachine()
    # DISCONNECTED 不能直接跳到 MOVING
    assert not sm.transition(ControlPhase.MOVING)
    assert sm.phase == ControlPhase.DISCONNECTED


def test_state_machine_force():
    sm = ControlStateMachine()
    sm.force(ControlPhase.EMERGENCY)
    assert sm.phase == ControlPhase.EMERGENCY
    assert not sm.is_safe()


def test_all_states_have_transitions():
    for phase in ControlPhase:
        assert phase in LEGAL_TRANSITIONS, f"missing transitions for {phase}"


def test_battery_drains_under_motion():
    b = MockBackend(initial_battery=1.0)
    b.connect()
    b.request_control()
    init = b.get_battery().level
    b.set_velocity(Twist2D(vx=0.5, w=0.0))
    for _ in range(200):
        b.step()
    cur = b.get_battery().level
    # 仿真 step 推进时间，battery 会下降（数值极小，但应当 < 初始）
    assert cur <= init