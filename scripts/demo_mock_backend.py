#!/usr/bin/env python3
"""Windows / 任意环境可跑的 MockBackend 演示（无需 ROS2）。

验证：cmd_vel(vx) → 运动学积分 → odom.x 增加；停发后速度归零。
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_control"))

from agv_control.backend.base import Twist2D  # noqa: E402
from agv_control.backend.mock import MockBackend  # noqa: E402
from agv_control.safety.watchdog import Watchdog, WatchdogConfig  # noqa: E402


def main() -> int:
    backend = MockBackend(max_vx=0.5, max_acc=2.0, max_jerk=5.0)
    assert backend.connect()
    assert backend.request_control("demo")

    clock = {"t": time.time()}
    stopped = {"n": 0}

    def now() -> float:
        return clock["t"]

    wd = Watchdog(
        cfg=WatchdogConfig(cmd_stale_ms=500),
        has_control=lambda: backend.has_control,
        on_safe_stop=lambda: (backend.stop(), stopped.__setitem__("n", stopped["n"] + 1)),
        clock_s=now,
    )

    print("=== V0.1 MockBackend demo (no ROS2) ===")
    print("1) drive forward vx=0.3 for 1.0s @ 50Hz")
    for _ in range(50):
        backend.set_velocity(Twist2D(vx=0.3, w=0.0))
        wd.feed_cmd()
        clock["t"] += 0.02
        wd.tick()
        backend.step()
        time.sleep(0.001)

    odom = backend.get_odom()
    print(f"   odom: x={odom.x:.3f} y={odom.y:.3f} yaw={odom.yaw:.3f} vx={odom.vx:.3f}")
    assert odom.x > 0.05, "expected forward motion"

    print("2) stop publishing cmd → watchdog safe-stop")
    for _ in range(40):  # 0.8s without feed_cmd
        clock["t"] += 0.02
        wd.tick()
        backend.step()
    odom2 = backend.get_odom()
    print(f"   after stale: vx={odom2.vx:.3f} safe_stops={stopped['n']} state={wd.state}")
    assert abs(odom2.vx) < 1e-3

    print("3) rotate in place w=0.5 for 0.5s")
    for _ in range(25):
        backend.set_velocity(Twist2D(vx=0.0, w=0.5))
        wd.feed_cmd()
        clock["t"] += 0.02
        wd.tick()
        backend.step()
    odom3 = backend.get_odom()
    print(f"   odom: yaw={odom3.yaw:.3f} (delta≈{odom3.yaw - odom2.yaw:.3f})")
    assert abs(odom3.yaw - odom2.yaw) > 0.05

    backend.release_control()
    backend.disconnect()
    print("OK — MockBackend + Watchdog offline demo passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
