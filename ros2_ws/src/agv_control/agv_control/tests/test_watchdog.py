"""Watchdog 单测 — 完全离线。"""

import time

from agv_control.safety.watchdog import (
    SafetyState,
    Watchdog,
    WatchdogConfig,
)


def _make_watchdog(has_control: bool = True):
    cfg = WatchdogConfig(cmd_stale_ms=100, control_lost_check_period_s=0.5)
    calls = {"stop": 0}
    w = Watchdog(
        cfg=cfg,
        has_control=lambda: has_control,
        on_safe_stop=lambda: calls.__setitem__("stop", calls["stop"] + 1),
        clock_s=lambda: time.time(),
    )
    return w, calls


def test_normal_state():
    w, calls = _make_watchdog()
    w.feed_cmd()
    s = w.tick()
    assert s == SafetyState.NORMAL
    assert calls["stop"] == 0


def test_cmd_stale_triggers_safe_stop():
    w, calls = _make_watchdog()
    w.feed_cmd()
    time.sleep(0.2)
    s = w.tick()
    assert s == SafetyState.SAFE_STOP
    assert calls["stop"] >= 1


def test_control_lost_triggers_safe_stop():
    w, calls = _make_watchdog(has_control=False)
    w.feed_cmd()
    s = w.tick()
    # control_lost 要 0.5s 才检查一次，所以第一次 tick 仍 NORMAL
    assert s == SafetyState.NORMAL
    time.sleep(0.6)
    s = w.tick()
    assert s == SafetyState.SAFE_STOP


def test_emergency_override():
    w, calls = _make_watchdog()
    w.trigger_emergency()
    s = w.tick()
    assert s == SafetyState.SAFE_STOP
    assert calls["stop"] >= 1


def test_reset():
    w, _ = _make_watchdog()
    w.trigger_emergency()
    w.tick()
    w.reset()
    assert w.state == SafetyState.NORMAL