#!/usr/bin/env python3
"""M3.9 motion dynamics / limiter / oscillation / curvature regression."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "agv_bridge"))

from agv_bridge.nav_dynamics_limiter import DynamicsLimiter  # noqa: E402
from agv_bridge.nav_motion_dynamics import (  # noqa: E402
    OscillationDetector,
    OvershootDetector,
    curvature,
    curve_vmax,
    get_motion_limits,
    lateral_accel_kappa,
    lateral_accel_vw,
    limit_cycle_suspected,
    preview_max_abs_kappa,
)
from agv_bridge.nav_trajectory_validator import (  # noqa: E402
    REASON_AY_LIMIT,
    validate_dynamics,
)


def _run(name: str, fn: Callable[[], None]) -> Tuple[str, bool, str]:
    try:
        fn()
        return name, True, "ok"
    except AssertionError as e:
        return name, False, str(e) or "assertion failed"
    except Exception as e:
        return name, False, f"{type(e).__name__}: {e}"


def test_accel_limit() -> None:
    lim = DynamicsLimiter()
    lim.vx = 0.0
    out = lim.limit(0.40, 0.0, dt=0.05)
    assert out.vx <= 0.9 * 0.05 + 1e-6, out
    assert "ACCEL" in out.reasons, out


def test_decel_limit() -> None:
    lim = DynamicsLimiter()
    lim.vx = 0.30
    out = lim.limit(0.0, 0.0, dt=0.05)
    assert out.vx >= 0.30 - 0.9 * 0.05 - 1e-6, out
    assert out.vx < 0.30, out


def test_alpha_limit() -> None:
    lim = DynamicsLimiter()
    lim.omega = 0.0
    out = lim.limit(0.20, 0.45, dt=0.05)
    assert abs(out.omega) <= 0.55 * 0.05 + 1e-6, out
    assert "ALPHA" in out.reasons, out


def test_lateral_accel_limit() -> None:
    lims = get_motion_limits()
    ay = lateral_accel_vw(0.40, 0.45)
    assert abs(ay - lims.max_lateral_accel_mps2) < 1e-9
    kap = curvature(0.20, 0.10)
    assert kap is not None
    ay2 = lateral_accel_kappa(0.20, kap)
    assert ay2 is not None and abs(ay2 - lateral_accel_vw(0.20, 0.10)) < 1e-9
    bad = validate_dynamics(vx=0.40, omega=0.60)
    assert not bad.valid
    assert REASON_AY_LIMIT in bad.reasons or bad.reason == REASON_AY_LIMIT


def test_longitudinal_jerk_limit() -> None:
    lims = get_motion_limits()
    assert lims.max_longitudinal_jerk_mps3 is None
    assert lims.sources["max_longitudinal_jerk_mps3"] == "CALIBRATION_REQUIRED"


def test_angular_jerk_limit() -> None:
    lims = get_motion_limits()
    assert lims.max_angular_jerk_rps3 is None
    assert lims.sources["max_angular_jerk_rps3"] == "CALIBRATION_REQUIRED"


def test_curvature_speed_limit() -> None:
    v = curve_vmax(0.0, 0.18, 0.40)
    assert abs(v - 0.40) < 1e-9
    v2 = curve_vmax(2.0, 0.18, 0.40)
    expect = math.sqrt(0.18 / 2.0)
    assert abs(v2 - expect) < 1e-9
    assert curvature(0.01, 0.2) is None


def test_curvature_preview() -> None:
    # quarter-circle-ish polyline
    path = [(math.cos(a) * 5, math.sin(a) * 5) for a in [i * 0.08 for i in range(40)]]
    peak, samples = preview_max_abs_kappa(path, path[0][0], path[0][1])
    assert peak > 0.05, (peak, samples)
    assert "kappa_1.0m" in samples


def test_turn_ramp() -> None:
    lim = DynamicsLimiter()
    lim.omega = 0.0
    prev = 0.0
    for _ in range(8):
        out = lim.limit(0.16, 0.35, dt=0.05)
        assert out.omega >= prev - 1e-9
        assert out.omega - prev <= 0.55 * 0.05 + 1e-6
        prev = out.omega
    assert prev < 0.35


def test_oscillation_detection() -> None:
    det = OscillationDetector(window_s=4.0, flip_limit=6)
    t = 0.0
    w = 0.10
    for i in range(12):
        w = -w
        r = det.update(t, w)
        t += 0.3
    assert r["omega_sign_changes"] >= 6
    assert r["control_chatter"] is True


def test_overshoot_detection() -> None:
    det = OvershootDetector(mag_rad=0.25)
    det.update(0.40)
    r = det.update(-0.35)
    assert r["heading_overshoot"] is True


def test_limit_cycle() -> None:
    assert limit_cycle_suspected(progress_m=0.02, omega_sign_changes=5) is True
    assert limit_cycle_suspected(progress_m=1.0, omega_sign_changes=5) is False


def test_post_turn_decay() -> None:
    lim = DynamicsLimiter()
    lim.omega = 0.35
    lim.vx = 0.14
    out = lim.limit(0.14, 0.0, dt=0.05)
    assert out.omega < 0.35
    assert out.omega >= 0.35 - 0.55 * 0.05 - 1e-6
    # no instant sign flip
    out2 = lim.limit(0.14, -0.35, dt=0.05)
    assert out2.omega > -0.20, out2


def test_command_smoothing() -> None:
    lim = DynamicsLimiter()
    lim.omega = 0.20
    a = lim.limit(0.20, 0.50, dt=0.05)
    b = lim.limit(0.20, -0.50, dt=0.05)
    assert abs(b.omega - a.omega) <= 0.55 * 0.05 + 1e-6
    assert not (a.omega > 0.05 and b.omega < -0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description="M3.9 motion dynamics regression")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--base", default="http://127.0.0.1:19999")
    ap.add_argument("--live-seconds", type=float, default=14.0)
    args = ap.parse_args()
    tests = [
        ("TEST_ACCEL_LIMIT", test_accel_limit),
        ("TEST_DECEL_LIMIT", test_decel_limit),
        ("TEST_ALPHA_LIMIT", test_alpha_limit),
        ("TEST_LATERAL_ACCEL_LIMIT", test_lateral_accel_limit),
        ("TEST_LONGITUDINAL_JERK_LIMIT", test_longitudinal_jerk_limit),
        ("TEST_ANGULAR_JERK_LIMIT", test_angular_jerk_limit),
        ("TEST_CURVATURE_SPEED_LIMIT", test_curvature_speed_limit),
        ("TEST_CURVATURE_PREVIEW", test_curvature_preview),
        ("TEST_TURN_RAMP", test_turn_ramp),
        ("TEST_OSCILLATION_DETECTION", test_oscillation_detection),
        ("TEST_OVERSHOOT_DETECTION", test_overshoot_detection),
        ("TEST_LIMIT_CYCLE", test_limit_cycle),
        ("TEST_POST_TURN_DECAY", test_post_turn_decay),
        ("TEST_COMMAND_SMOOTHING", test_command_smoothing),
    ]
    results = [_run(n, fn) for n, fn in tests]
    failed = [r for r in results if not r[1]]
    print("=" * 41)
    print("AGV NAVIGATION V0.2 M3.9")
    print("MOTION DYNAMICS / CONTROL INTEGRITY")
    print("=" * 41)
    print("\nREGRESSION")
    for name, ok, msg in results:
        print(f"  {name}: {'PASS' if ok else 'FAIL'}  {'' if ok else msg}")

    live: Dict[str, Any] = {"status": "NOT_RUN"}
    if args.live:
        live = _live(args.base, args.live_seconds)
        print("\nLIVE", live.get("status"), live.get("error") or "")
        for f in live.get("files") or []:
            print(f"  {f.get('scene')}: {f.get('summary')}")

    out_dir = ROOT / "logs" / "navigation_v02"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = out_dir / f"M3.9-MOTION-FORENSICS-{stamp}.json"
    path.write_text(json.dumps({"regression": results, "live": live}, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print("FORENSICS", path)
    print("RESULT:", "FAIL" if failed else "PASS")
    return 1 if failed else 0


def _live(base: str, seconds: float) -> Dict[str, Any]:
    import urllib.request

    try:
        with urllib.request.urlopen(f"{base}/api/state?lite=1", timeout=3) as resp:
            if resp.status != 200:
                return {"status": "NOT_RUN", "reason": "SIM_NOT_RUNNING"}
    except Exception:
        return {"status": "NOT_RUN", "reason": "SIM_NOT_RUNNING"}
    sys.path.insert(0, str(ROOT / "scripts"))
    from _trace_navigation_v02_live import run_scene  # type: ignore
    from agv_bridge.nav_live_client import NavLiveClient
    from agv_bridge.sim_world import SimWorld

    client = NavLiveClient(base=base)
    world = SimWorld()
    out_dir = ROOT / "logs" / "navigation_v02"
    out_dir.mkdir(parents=True, exist_ok=True)
    report: Dict[str, Any] = {"status": "RAN", "files": [], "worst": {}}
    seq = 0
    for scene, dur in (("M32-OPEN-STRAIGHT", min(12.0, seconds)), ("SCENE-CURVE-01", max(18.0, seconds)), ("ONLINE-LEFT", max(36.0, seconds))):
        rows, seq, setup, fname = run_scene(client, world, scene, dur, 5.0, seq)
        if not fname:
            report.setdefault("setup_fail", []).append({"scene": scene, "setup": setup})
            continue
        path = out_dir / fname
        with path.open("w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, ensure_ascii=False, default=str) + "\n")
        samples = [r for r in rows if r.get("vehicle_pose") and not r.get("event")]
        summary = _summarize_motion(samples, scene)
        report["files"].append({"scene": scene, "file": fname, "samples": len(samples), "summary": summary})
    return report


def _summarize_motion(samples: List[dict], scene: str) -> Dict[str, Any]:
    if not samples:
        return {"empty": True}
    def _f(key):
        vals = []
        for r in samples:
            v = r.get(key)
            if v is None and isinstance(r.get("motion"), dict):
                v = (r.get("motion") or {}).get(key)
            if isinstance(v, (int, float)):
                vals.append(abs(float(v)))
        return max(vals) if vals else None

    ay = [abs(float(r["lateral_acceleration"])) for r in samples if r.get("lateral_acceleration") is not None]
    alpha = [abs(float(r["alpha"])) for r in samples if r.get("alpha") is not None]
    osc = [int(r["oscillation_score"]) for r in samples if r.get("oscillation_score") is not None]
    over = sum(1 for r in samples if r.get("heading_overshoot"))
    lc = sum(1 for r in samples if r.get("limit_cycle_suspected"))
    age = [float(r["trajectory_age_ms"]) for r in samples if r.get("trajectory_age_ms") is not None]
    return {
        "max_abs_omega": _f("approved_omega") or _f("actual_omega"),
        "max_ay": max(ay) if ay else None,
        "max_alpha": max(alpha) if alpha else None,
        "max_oscillation_score": max(osc) if osc else 0,
        "heading_overshoot_count": over,
        "limit_cycle_count": lc,
        "max_trajectory_age_ms": max(age) if age else None,
        "collision": any(r.get("collision") for r in samples),
    }


if __name__ == "__main__":
    raise SystemExit(main())
