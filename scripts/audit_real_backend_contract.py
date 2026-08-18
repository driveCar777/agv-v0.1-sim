"""M3.9.3 — Static audit: Real Backend API Contract.

Checks:
  1. Emergency API must be 1012, NOT 1011
  2. translate_distance payload must contain dist/vx/mode, NOT vx/vy/w as velocity
  3. rotate_angle payload must contain angle/vw/mode, NOT just w
  4. set_velocity_cmd must use API 2010, NOT 3055
  5. stop() must NOT fallback to translate/3055 on failure
  6. real mode must NOT have default IP
  7. factory must block real mode without REAL_ROBOT_ENABLED=true
  8. factory must block real mode without AGV_HOST
  9. tcp mode must block without AGV_HOST
 10. /cmd_vel path must use set_velocity() → 2010, NOT translate() → 3055

All checks are static (source inspection or import-only). No real vehicle needed.
"""

from __future__ import annotations

import ast
import importlib.util
import inspect
import os
import sys
import textwrap
from pathlib import Path

# Add package to path
ROOT = Path(__file__).parent.parent
AGV_CONTROL = ROOT / "ros2_ws/src/agv_control"
sys.path.insert(0, str(AGV_CONTROL))

PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"


def check(name: str, condition: bool, detail: str = "") -> str:
    status = PASS if condition else FAIL
    prefix = "  [PASS]" if condition else "  [FAIL]"
    print(f"{prefix} {name}")
    if detail:
        for line in textwrap.wrap(detail, width=90, initial_indent="         ", subsequent_indent="         "):
            print(line)
    return status


def main() -> int:
    print("=" * 70)
    print("M3.9.3 REAL BACKEND API CONTRACT AUDIT")
    print("=" * 70)
    failures = 0

    # --- Import modules ---
    try:
        from agv_control.transport.tcp_client import (
            TcpApiClient,
            API_EMERGENCY,
            API_VELOCITY,
            API_TRANSLATE_TASK,
            API_ROTATE_TASK,
            API_SOFT_STOP,
            _WRONG_EMERGENCY_1011,
        )
        tcp_import_ok = True
    except Exception as e:
        print(f"  [FAIL] tcp_client import failed: {e}")
        tcp_import_ok = False
        failures += 1

    try:
        from agv_control.backend.real import RealBackend
        real_import_ok = True
    except Exception as e:
        print(f"  [FAIL] real.py import failed: {e}")
        real_import_ok = False
        failures += 1

    try:
        from agv_control.backend.factory import create_backend
        factory_import_ok = True
    except Exception as e:
        print(f"  [FAIL] factory import failed: {e}")
        factory_import_ok = False
        failures += 1

    print()
    print("── API ID CONTRACT ─────────────────────────────────────────────────")

    if tcp_import_ok:
        # Check 1: Emergency = 1012
        r = check(
            "Emergency API = 1012 (NOT 1011)",
            API_EMERGENCY == 1012,
            f"Found API_EMERGENCY={API_EMERGENCY}. Must be 1012 (confirmed by robokit_client.py).",
        )
        if r == FAIL: failures += 1

        # Check 2: _WRONG_EMERGENCY_1011 exists as documentation tombstone
        r = check(
            "_WRONG_EMERGENCY_1011 = 1011 (tombstone, must not be used)",
            _WRONG_EMERGENCY_1011 == 1011,
        )
        if r == FAIL: failures += 1

        # Check 3: Velocity API = 2010
        r = check(
            "Continuous velocity API = 2010",
            API_VELOCITY == 2010,
            f"Found API_VELOCITY={API_VELOCITY}. Must be 2010.",
        )
        if r == FAIL: failures += 1

        # Check 4: Translate task = 3055
        r = check(
            "Translate motion task = 3055 (distance task, NOT velocity)",
            API_TRANSLATE_TASK == 3055,
        )
        if r == FAIL: failures += 1

        # Check 5: Rotate task = 3056
        r = check(
            "Rotate motion task = 3056 (angle task, NOT angular velocity stream)",
            API_ROTATE_TASK == 3056,
        )
        if r == FAIL: failures += 1

    print()
    print("── PAYLOAD CONTRACT ────────────────────────────────────────────────")

    if tcp_import_ok:
        client = TcpApiClient.__new__(TcpApiClient)

        # Check 6: set_velocity_cmd uses 2010, NOT 3055
        # Check by inspecting the actual call() invocation — comments may mention 3055
        import ast, textwrap
        src_vel = inspect.getsource(TcpApiClient.set_velocity_cmd)
        uses_api_velocity = "API_VELOCITY" in src_vel
        # Check for actual call to translate_distance (not just comment mentioning 3055)
        calls_translate_distance = "translate_distance(" in src_vel
        r = check(
            "set_velocity_cmd routes to API 2010 (not 3055)",
            uses_api_velocity and not calls_translate_distance,
            "set_velocity_cmd must call PORT_CONTROL + API_VELOCITY (2010). "
            "3055 is a motion task, not velocity streaming.",
        )
        if r == FAIL: failures += 1

        # Check 7: set_velocity_cmd does NOT call translate_distance()
        r = check(
            "set_velocity_cmd does NOT call translate_distance()",
            not calls_translate_distance,
            "Any call to translate_distance() from set_velocity_cmd would use 3055.",
        )
        if r == FAIL: failures += 1

        # Check 8: translate_distance payload has dist/vx/mode, NOT vx/vy/w
        src_trans = inspect.getsource(TcpApiClient.translate_distance)
        has_dist = '"dist"' in src_trans
        has_mode = '"mode"' in src_trans
        no_w_field = '"w"' not in src_trans and '"vy"' not in src_trans
        r = check(
            "translate_distance payload: {dist, vx, mode} — no omega/w/vy",
            has_dist and has_mode and no_w_field,
            "3055 is a distance task: {dist, vx, mode}. Must NOT contain w/vy velocity fields.",
        )
        if r == FAIL: failures += 1

        # Check 9: rotate_angle payload has angle/vw/mode, NOT just w
        src_rot = inspect.getsource(TcpApiClient.rotate_angle)
        has_angle = '"angle"' in src_rot
        has_vw = '"vw"' in src_rot
        has_mode_rot = '"mode"' in src_rot
        no_bare_w = '"w"' not in src_rot
        r = check(
            "rotate_angle payload: {angle, vw, mode} — no bare w",
            has_angle and has_vw and has_mode_rot and no_bare_w,
            "3056 is a rotation task: {angle, vw, mode}. Must NOT contain bare 'w' velocity field.",
        )
        if r == FAIL: failures += 1

        # Check 10: stop() does NOT fallback to translate/3055
        src_stop = inspect.getsource(TcpApiClient.stop)
        # Check actual calls, not comments (comments may say "DO NOT call translate")
        calls_translate_in_stop = "translate_distance(" in src_stop or "self.translate(" in src_stop
        has_stop_failed = "STOP_FAILED" in src_stop
        r = check(
            "stop() does NOT fallback to translate()/3055 on failure",
            not calls_translate_in_stop and has_stop_failed,
            "If 2000 fails, stop() must return STOP_FAILED, NOT call translate(0,0,0). "
            "Zero-velocity 3055 is a contract violation.",
        )
        if r == FAIL: failures += 1

    print()
    print("── HOST / MODE SAFETY GATES ────────────────────────────────────────")

    if real_import_ok:
        # Check 11: RealBackend requires explicit host in real mode
        try:
            old_host = os.environ.pop("AGV_HOST", None)
            try:
                RealBackend(require_host=True)
                r = check(
                    "RealBackend requires AGV_HOST in real mode (no default IP)",
                    False,
                    "Expected ValueError — should reject missing host.",
                )
                failures += 1
            except (ValueError, RuntimeError) as e:
                r = check(
                    "RealBackend requires AGV_HOST in real mode (no default IP)",
                    True,
                    f"Correctly rejected: {e}",
                )
            finally:
                if old_host is not None:
                    os.environ["AGV_HOST"] = old_host
        except Exception as e:
            print(f"  [FAIL] Host check error: {e}")
            failures += 1

        # Check 12: RealBackend source does NOT have hardcoded 192.168.x.x default
        src_real = inspect.getsource(RealBackend.__init__)
        no_hardcoded_ip = "192.168." not in src_real
        r = check(
            "RealBackend.__init__ has no hardcoded 192.168.x.x default IP",
            no_hardcoded_ip,
            "Production default IPs are forbidden. Host must come from AGV_HOST env or explicit param.",
        )
        if r == FAIL: failures += 1

    if factory_import_ok:
        # Check 13: factory blocks real mode without REAL_ROBOT_ENABLED
        try:
            old_rre = os.environ.pop("REAL_ROBOT_ENABLED", None)
            old_host = os.environ.get("AGV_HOST", "")
            os.environ["AGV_HOST"] = "192.168.1.1"
            try:
                create_backend("real")
                r = check(
                    "factory blocks real mode without REAL_ROBOT_ENABLED=true",
                    False,
                    "Expected RuntimeError.",
                )
                failures += 1
            except (RuntimeError, ValueError) as e:
                r = check(
                    "factory blocks real mode without REAL_ROBOT_ENABLED=true",
                    True,
                    f"Correctly blocked: {e}",
                )
            finally:
                if old_rre is not None:
                    os.environ["REAL_ROBOT_ENABLED"] = old_rre
                else:
                    os.environ.pop("REAL_ROBOT_ENABLED", None)
                if old_host:
                    os.environ["AGV_HOST"] = old_host
                else:
                    os.environ.pop("AGV_HOST", None)
        except Exception as e:
            print(f"  [FAIL] Factory gate check error: {e}")
            failures += 1

        # Check 14: factory blocks real mode without AGV_HOST
        try:
            old_rre = os.environ.get("REAL_ROBOT_ENABLED", "")
            old_host = os.environ.pop("AGV_HOST", None)
            os.environ["REAL_ROBOT_ENABLED"] = "true"
            try:
                create_backend("real")
                r = check(
                    "factory blocks real mode without AGV_HOST",
                    False,
                    "Expected RuntimeError.",
                )
                failures += 1
            except (RuntimeError, ValueError) as e:
                r = check(
                    "factory blocks real mode without AGV_HOST",
                    True,
                    f"Correctly blocked: {e}",
                )
            finally:
                if old_rre:
                    os.environ["REAL_ROBOT_ENABLED"] = old_rre
                else:
                    os.environ.pop("REAL_ROBOT_ENABLED", None)
                if old_host is not None:
                    os.environ["AGV_HOST"] = old_host
        except Exception as e:
            print(f"  [FAIL] Factory AGV_HOST gate check error: {e}")
            failures += 1

        # Check 15: factory blocks tcp mode without AGV_HOST
        try:
            old_host = os.environ.pop("AGV_HOST", None)
            try:
                create_backend("tcp")
                r = check(
                    "factory blocks tcp mode without explicit AGV_HOST",
                    False,
                    "Expected RuntimeError.",
                )
                failures += 1
            except (RuntimeError, ValueError) as e:
                r = check(
                    "factory blocks tcp mode without explicit AGV_HOST",
                    True,
                    f"Correctly blocked: {e}",
                )
            finally:
                if old_host is not None:
                    os.environ["AGV_HOST"] = old_host
        except Exception as e:
            print(f"  [FAIL] Factory tcp gate check error: {e}")
            failures += 1

    print()
    print("── cmd_vel NODE WIRING ──────────────────────────────────────────────")

    cmd_vel_path = AGV_CONTROL / "agv_control/ros/cmd_vel_node.py"
    if cmd_vel_path.exists():
        src = cmd_vel_path.read_text(encoding="utf-8")
        # Check 16: cmd_vel_node calls set_velocity(), not translate()
        calls_set_velocity = "set_velocity" in src
        no_direct_translate = "translate(" not in src and "API_TRANSLATE" not in src
        r = check(
            "cmd_vel_node calls backend.set_velocity(), NOT translate()",
            calls_set_velocity and no_direct_translate,
            "cmd_vel → backend.set_velocity() → API 2010. Direct translate() call is P0 forbidden.",
        )
        if r == FAIL: failures += 1
    else:
        print("  [WARN] cmd_vel_node.py not found at expected path")

    print()
    print("── STOP CONTRACT ────────────────────────────────────────────────────")

    if real_import_ok:
        src_real_stop = inspect.getsource(RealBackend.stop)
        no_translate_fallback = "translate_distance(" not in src_real_stop and "self._client.translate(" not in src_real_stop
        has_stop_failed = "STOP_FAILED" in src_real_stop
        r = check(
            "RealBackend.stop() handles STOP_FAILED, no 3055 fallback",
            no_translate_fallback and has_stop_failed,
        )
        if r == FAIL: failures += 1

    print()
    print("=" * 70)
    if failures == 0:
        print("AUDIT RESULT: ALL CHECKS PASS")
        print()
        print("NOTE: Code contract PASS ≠ Real vehicle PASS.")
        print("      API 2010 (velocity) is UNVERIFIED on real vehicle.")
        print("      PHYSICAL_TEST_ALLOWED = NO")
        print("      RC1_READY = NO")
    else:
        print(f"AUDIT RESULT: {failures} CHECK(S) FAILED — M3.9.3 NOT COMPLETE")
    print("=" * 70)
    return failures


if __name__ == "__main__":
    sys.exit(main())
