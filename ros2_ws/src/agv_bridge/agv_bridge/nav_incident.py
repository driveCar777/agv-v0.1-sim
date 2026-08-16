"""Incident / approach / divergence / recovery diagnostics (read-only).

Used by NavDebugHub. Does not change navigation algorithms.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

INCIDENT_TRIGGERS = frozenset(
    {
        "RECOVERY_START",
        "REVERSE_START",
        "PATH_PROGRESS_STALLED",
        "SAFETY_BLOCK",
        "COLLISION_GUARD",
        "RECOVERY_EXHAUSTED",
        "FAILED",
        "DIVERGENCE_START",
        "ANGULAR_OSCILLATION",
        "REVERSE_WITH_FORWARD_AVAILABLE",
        "APPROACHING_OBSTACLE_WHILE_TRACKING",
        "APPROACHED_OBSTACLE_THEN_REVERSE",
    }
)


class IncidentRecorder:
    """Rolling buffer + auto-capture T-10s .. T+5s around incidents."""

    PRE_S = 10.0
    POST_S = 5.0

    def __init__(self) -> None:
        self._ring: Deque[Dict[str, Any]] = deque(maxlen=400)
        self._active: Optional[Dict[str, Any]] = None
        self.incidents: List[Dict[str, Any]] = []

    def reset(self) -> None:
        self._ring.clear()
        self._active = None
        # keep last incidents for UI until next session begin clears explicitly

    def clear_session(self) -> None:
        self.reset()
        self.incidents = []

    def push_sample(self, sample: Dict[str, Any]) -> None:
        row = dict(sample)
        self._ring.append(row)
        if self._active is not None and not self._active.get("closed"):
            self._active["samples"].append(row)
            t0 = float(self._active["t0"])
            if float(row.get("timestamp") or 0) >= t0 + self.POST_S:
                self._finalize()

    def note_event(self, event: str, payload: Optional[Dict[str, Any]] = None) -> None:
        if event not in INCIDENT_TRIGGERS:
            return
        now = time.time()
        if self._active is not None and not self._active.get("closed"):
            self._active["events"].append({"ts": now, "event": event, **(payload or {})})
            return
        pre = [r for r in self._ring if now - float(r.get("timestamp") or 0) <= self.PRE_S]
        self._active = {
            "t0": now,
            "trigger": event,
            "payload": payload or {},
            "samples": list(pre),
            "events": [{"ts": now, "event": event, **(payload or {})}],
            "closed": False,
        }

    def _finalize(self) -> None:
        if not self._active:
            return
        self._active["closed"] = True
        self._active["t_end"] = time.time()
        samples = self._active["samples"]
        self._active["keyframes"] = _keyframes(samples, float(self._active["t0"]))
        self._active["summary"] = _incident_summary(samples, self._active["trigger"])
        self.incidents.append(self._active)
        if len(self.incidents) > 5:
            self.incidents = self.incidents[-5:]
        self._active = None

    def latest(self) -> Optional[Dict[str, Any]]:
        if self._active and not self._active.get("closed"):
            # live preview
            snap = dict(self._active)
            snap["keyframes"] = _keyframes(snap["samples"], float(snap["t0"]))
            snap["live"] = True
            return snap
        return self.incidents[-1] if self.incidents else None


def _keyframes(samples: List[Dict[str, Any]], t0: float) -> List[Dict[str, Any]]:
    targets = [-10, -8, -6, -4, -2, 0, 2, 5]
    out = []
    for off in targets:
        want = t0 + off
        best = None
        best_d = 1e9
        for s in samples:
            d = abs(float(s.get("timestamp") or 0) - want)
            if d < best_d:
                best_d = d
                best = s
        if best and best_d < 1.2:
            out.append(
                {
                    "offset_s": off,
                    "label": f"T{off:+d}s",
                    "timestamp": best.get("timestamp"),
                    "x": best.get("x"),
                    "y": best.get("y"),
                    "theta": best.get("theta"),
                    "vx": best.get("state_vx"),
                    "w": best.get("state_w"),
                    "ax": best.get("ax"),
                    "alpha": best.get("alpha"),
                    "front_near": best.get("front_near"),
                    "rear_near": best.get("rear_near"),
                    "collision": best.get("collision"),
                    "path_progress": best.get("path_progress_s"),
                    "lateral_error": best.get("lateral_error"),
                    "heading_error": best.get("heading_error"),
                    "mppi_vx": best.get("mppi_vx"),
                    "safe_vx": best.get("safe_vx"),
                    "state_vx": best.get("state_vx"),
                    "selected_candidate": best.get("selected_candidate"),
                    "phase": best.get("phase"),
                    "stop_reason": best.get("stop_reason"),
                    "actual_clearance": best.get("actual_clearance"),
                    "path_clearance": best.get("path_clearance"),
                }
            )
    return out


def _incident_summary(samples: List[Dict[str, Any]], trigger: str) -> Dict[str, Any]:
    if not samples:
        return {"trigger": trigger}
    first, last = samples[0], samples[-1]
    lats = [float(s.get("lateral_error") or 0) for s in samples]
    clrs = [float(s.get("actual_clearance") or s.get("nearest_obstacle_distance") or 99) for s in samples]
    return {
        "trigger": trigger,
        "lat_start": round(lats[0], 3),
        "lat_end": round(lats[-1], 3),
        "lat_delta": round(lats[-1] - lats[0], 3),
        "clr_start": round(clrs[0], 3),
        "clr_end": round(clrs[-1], 3),
        "clr_delta": round(clrs[-1] - clrs[0], 3),
        "progress_start": round(float(first.get("path_progress_s") or 0), 3),
        "progress_end": round(float(last.get("path_progress_s") or 0), 3),
        "phase_end": last.get("phase"),
    }


class ApproachAnalyzer:
    """Wall/obstacle approach + tracking divergence + oscillation (diagnostics)."""

    def __init__(self) -> None:
        self._prev_clr: Optional[float] = None
        self._prev_lat: Optional[float] = None
        self._prev_herr: Optional[float] = None
        self._prev_t: Optional[float] = None
        self._lat_rise = 0
        self._clr_drop = 0
        self._diverged = False
        self.first_divergence: Optional[Dict[str, Any]] = None
        self._w_signs: Deque[Tuple[float, int]] = deque(maxlen=40)
        self._approach_state = ""
        self.steering_weak_s = 0.0

    def reset(self) -> None:
        self.__init__()

    def update(self, sample: Dict[str, Any], events: Any, session_start: Optional[float]) -> Dict[str, Any]:
        now = float(sample.get("timestamp") or time.time())
        dt = 0.05 if self._prev_t is None else max(1e-3, now - self._prev_t)
        clr = float(
            sample.get("actual_clearance")
            if sample.get("actual_clearance") is not None
            else sample.get("nearest_obstacle_distance")
            if sample.get("nearest_obstacle_distance") is not None
            else sample.get("front_near")
            or 99.0
        )
        lat = abs(float(sample.get("lateral_error") or 0.0))
        herr = abs(float(sample.get("heading_error") or 0.0))
        vx = float(sample.get("state_vx") or 0.0)
        w = float(sample.get("state_w") or 0.0)
        desired_w = float(sample.get("desired_w", sample.get("mppi_w", 0.0)) or 0.0)
        phase = str(sample.get("phase") or "")
        fwd = str(sample.get("forward_trajectory") or "")

        clr_rate = 0.0 if self._prev_clr is None else (clr - self._prev_clr) / dt
        lat_rate = 0.0 if self._prev_lat is None else (lat - self._prev_lat) / dt
        herr_rate = 0.0 if self._prev_herr is None else (herr - self._prev_herr) / dt

        ratio = 0.0
        if abs(desired_w) > 0.05:
            ratio = abs(w) / abs(desired_w)
            if ratio < 0.35:
                self.steering_weak_s += dt
                if self.steering_weak_s > 0.8:
                    events.push(
                        "STEERING_RESPONSE_WEAK",
                        {
                            "desired_w": desired_w,
                            "state_w": w,
                            "ratio": round(ratio, 3),
                            "category": "ERROR",
                            "message": f"steer ratio={ratio:.0%}",
                        },
                        min_interval_s=1.5,
                        category="ERROR",
                    )
            else:
                self.steering_weak_s = 0.0
        else:
            self.steering_weak_s = max(0.0, self.steering_weak_s - dt)

        # divergence
        if lat_rate > 0.05 and lat > 0.18:
            self._lat_rise += 1
        else:
            self._lat_rise = max(0, self._lat_rise - 1)
        if self._lat_rise >= 4 and not self._diverged:
            self._diverged = True
            since = (now - session_start) if session_start else 0.0
            self.first_divergence = {
                "timestamp": now,
                "since_session_s": round(since, 2),
                "lateral_error": lat,
                "heading_error": float(sample.get("heading_error") or 0),
                "vx": vx,
                "w": w,
                "path_progress": sample.get("path_progress_s"),
                "front_near": sample.get("front_near"),
                "rear_near": sample.get("rear_near"),
                "selected_candidate": sample.get("selected_candidate"),
            }
            events.push(
                "DIVERGENCE_START",
                {**self.first_divergence, "category": "ERROR", "message": f"lat={lat:.2f}"},
                min_interval_s=0.0,
                category="ERROR",
            )
            if hasattr(events, "hub_incident"):
                pass

        if clr_rate < -0.08:
            self._clr_drop += 1
        else:
            self._clr_drop = max(0, self._clr_drop - 1)

        tag = ""
        if self._clr_drop >= 3 and self._lat_rise >= 2 and vx > 0.03:
            tag = "APPROACHING_OBSTACLE_WHILE_TRACKING"
            events.push(
                tag,
                {
                    "clearance": clr,
                    "clearance_rate": round(clr_rate, 3),
                    "lateral_error": lat,
                    "category": "OBSTACLE",
                    "message": f"clr↓{clr:.2f} lat↑{lat:.2f}",
                },
                min_interval_s=1.0,
                category="OBSTACLE",
            )
            self._approach_state = tag
        elif self._approach_state == "APPROACHING_OBSTACLE_WHILE_TRACKING" and abs(vx) < 0.02:
            tag = "APPROACHED_OBSTACLE_AND_STOPPED"
            events.push(tag, {"clearance": clr, "category": "STOP", "message": "stopped near obstacle"}, min_interval_s=1.0, category="STOP")
            self._approach_state = tag
        elif self._approach_state in ("APPROACHING_OBSTACLE_WHILE_TRACKING", "APPROACHED_OBSTACLE_AND_STOPPED") and (
            vx < -0.03 or phase == "reverse_escape"
        ):
            tag = "APPROACHED_OBSTACLE_THEN_REVERSE"
            events.push(tag, {"clearance": clr, "category": "REVERSE", "message": "approach→reverse"}, min_interval_s=1.0, category="REVERSE")
            self._approach_state = tag

        # oscillation
        sign = 0 if abs(w) < 0.08 else (1 if w > 0 else -1)
        self._w_signs.append((now, sign))
        flips = 0
        prev = 0
        for t, s in self._w_signs:
            if now - t > 2.5:
                continue
            if s != 0 and prev != 0 and s != prev:
                flips += 1
            if s != 0:
                prev = s
        if flips >= 5:
            events.push(
                "ANGULAR_OSCILLATION",
                {
                    "turn_flip_count": flips,
                    "lateral_error": lat,
                    "heading_error": float(sample.get("heading_error") or 0),
                    "front_near": sample.get("front_near"),
                    "path_clearance": sample.get("path_clearance"),
                    "selected_candidate": sample.get("selected_candidate"),
                    "category": "ERROR",
                    "message": f"w flips={flips}/2.5s",
                },
                min_interval_s=1.5,
                category="ERROR",
            )

        if phase == "reverse_escape" and fwd == "AVAILABLE":
            events.push(
                "REVERSE_WITH_FORWARD_AVAILABLE",
                {
                    "phase": phase,
                    "forward_trajectory": fwd,
                    "front_near": sample.get("front_near"),
                    "rear_near": sample.get("rear_near"),
                    "path_progress": sample.get("path_progress_s"),
                    "stuck_s": sample.get("stuck_s"),
                    "best_forward_cost": sample.get("best_forward_cost"),
                    "best_reverse_cost": sample.get("best_reverse_cost"),
                    "forward_vx": sample.get("best_forward_vx"),
                    "reverse_vx": sample.get("best_reverse_vx"),
                    "category": "ERROR",
                    "message": "reverse while forward AVAILABLE",
                },
                min_interval_s=1.0,
                category="ERROR",
            )

        if lat_rate > 0.12:
            events.push(
                "LATERAL_ERROR_DIVERGING",
                {"rate": round(lat_rate, 3), "lat": lat, "category": "ERROR", "message": f"lat_rate={lat_rate:.2f}"},
                min_interval_s=1.0,
                category="ERROR",
            )

        self._prev_clr, self._prev_lat, self._prev_herr, self._prev_t = clr, lat, herr, now
        return {
            "clearance_rate": round(clr_rate, 4),
            "lateral_error_rate": round(lat_rate, 4),
            "heading_error_rate": round(herr_rate, 4),
            "steering_execution_ratio": round(ratio, 3),
            "turn_flip_count": flips,
            "approach_tag": self._approach_state,
            "first_divergence": self.first_divergence,
        }
