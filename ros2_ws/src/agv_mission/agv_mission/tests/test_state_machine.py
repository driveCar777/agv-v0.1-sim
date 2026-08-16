"""Mission 状态机单测 — 完全离线。"""

from agv_mission.state_machine import (
    LEGAL_TRANSITIONS,
    Mission,
    MissionPhase,
    can_transition,
    safe_transition,
)


def test_initial_state():
    m = Mission()
    assert m.phase == MissionPhase.IDLE


def test_received_to_pickup():
    m = Mission()
    m.transition(MissionPhase.RECEIVED)
    assert can_transition(m, MissionPhase.GO_TO_PICKUP)


def test_full_flow():
    m = Mission()
    seq = [
        MissionPhase.RECEIVED,
        MissionPhase.GO_TO_PICKUP,
        MissionPhase.ARRIVED_PICKUP,
        MissionPhase.IDENTIFY_OBJECT,
        MissionPhase.PICK,
        MissionPhase.VERIFY_PICK,
        MissionPhase.GO_TO_USER,
        MissionPhase.IDENTIFY_USER,
        MissionPhase.DELIVER,
        MissionPhase.VERIFY_DELIVER,
        MissionPhase.FINISHED,
    ]
    for s in seq:
        m.transition(s)
    assert m.phase == MissionPhase.FINISHED
    assert m.is_terminal()


def test_illegal_transition_blocked():
    m = Mission()
    m.transition(MissionPhase.RECEIVED)
    # 直接跳到 FINISHED 不合法
    assert not safe_transition(m, MissionPhase.FINISHED)
    assert m.phase == MissionPhase.RECEIVED


def test_error_recovery():
    m = Mission()
    m.transition(MissionPhase.RECEIVED)
    m.transition(MissionPhase.GO_TO_PICKUP)
    m.transition(MissionPhase.NAVIGATION_ERROR)
    assert m.phase == MissionPhase.NAVIGATION_ERROR
    assert safe_transition(m, MissionPhase.RECOVERY)
    assert safe_transition(m, MissionPhase.GO_TO_PICKUP)


def test_safety_stop():
    m = Mission()
    m.transition(MissionPhase.RECEIVED)
    m.transition(MissionPhase.GO_TO_PICKUP)
    m.transition(MissionPhase.SAFETY_STOP)
    assert m.phase == MissionPhase.SAFETY_STOP
    assert safe_transition(m, MissionPhase.RECOVERY)


def test_all_phases_have_transitions():
    for phase in MissionPhase:
        assert phase in LEGAL_TRANSITIONS, f"missing transitions for {phase}"


def test_history_recorded():
    m = Mission()
    m.transition(MissionPhase.RECEIVED, "test1")
    m.transition(MissionPhase.GO_TO_PICKUP, "test2")
    assert len(m.history) == 2
    assert m.history[0][2] == "test1"