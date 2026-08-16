"""agv_safety 单元测试 — 完全离线。"""

from agv_safety.levels import SafetyLevel


def test_safety_level_ordering():
    assert SafetyLevel.OK.value < SafetyLevel.WARN.value
    assert SafetyLevel.WARN.value < SafetyLevel.SLOW.value
    assert SafetyLevel.SLOW.value < SafetyLevel.STOP.value
    assert SafetyLevel.STOP.value < SafetyLevel.EMERGENCY.value


def test_safety_level_names():
    assert SafetyLevel.OK.name == "OK"
    assert SafetyLevel.STOP.name == "STOP"
    assert SafetyLevel.EMERGENCY.name == "EMERGENCY"