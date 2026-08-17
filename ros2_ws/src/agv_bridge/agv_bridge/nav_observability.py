"""P0-B-0 Navigation Observability — Event / Trace / Snapshot / API access log.

Instrumentation only. Does NOT change planning, FSM, Safety, or Recovery behavior.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import traceback
import uuid
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

from agv_bridge.nav_log_forensics import VX_EPS, W_EPS, infer_likely_owner

LEVELS = ("DEBUG", "INFO", "NOTICE", "WARN", "ERROR", "CRITICAL")
LEVEL_RANK = {lv: i for i, lv in enumerate(LEVELS)}

CATEGORIES = (
    "SYSTEM",
    "NAVIGATION",
    "GLOBAL_PLANNING",
    "LOCAL_PLANNING",
    "KINEMATIC",
    "CANDIDATE",
    "PROBE",
    "POLICY",
    "FSM",
    "CONTROL",
    "SAFETY",
    "RECOVERY",
    "REPLAN",
    "EXECUTION",
    "PHYSICS",
    "API",
    "NETWORK",
    "ERROR",
    "PERFORMANCE",
    "DIAGNOSTIC",
)

CRITICAL_EVENTS = frozenset(
    {
        "GOAL_REACHED",
        "NO_CANDIDATE",
        "ALL_CANDIDATES_INVALID",
        "CANDIDATE_SELECTED",
        "SAFETY_BLOCK",
        "SAFETY_CLAMP",
        "REPLAN",
        "REPLAN_FAILED",
        "RECOVERY_ENTER",
        "RECOVERY_EXECUTION",
        "RECOVERY_EXIT",
        "NAVIGATION_STOP_DIAGNOSTIC",
        "NAVIGATION_DEADLOCK_SUSPECTED",
        "SPIN_LOOP_SUSPECTED",
        "PLANNING_STARVATION_SUSPECTED",
        "PLAN_EXECUTION_MISMATCH",
        "NAV_CYCLE_OVERRUN",
        "FSM_TRANSITION",
        "GLOBAL_PREVIEW_UPDATED",
        "GLOBAL_PREVIEW_LIMITED",
        "GLOBAL_LOCAL_HORIZON_MISMATCH",
        "KINEMATIC_VALIDATION_STARTED",
        "KINEMATIC_VALIDATION_RESULT",
        "KINEMATIC_PATH_REJECTED",
        "KINEMATIC_CLEARANCE_WARNING",
        "KINEMATIC_SPEED_LIMITED",
        "LOCAL_PLAN_CREATED",
        "LOCAL_PLAN_SELECTED",
        "LOCAL_PLAN_REPLACED",
        "LOCAL_PLAN_EXPIRED",
        "LOCAL_PLAN_REJECTED",
        "LOCAL_PLAN_FALLBACK",
        "SPEED_TARGET_UPDATED",
        "MPPI_TRACKING_LOCAL_PLAN",
        "MPPI_LOCAL_REFERENCE_UPDATED",
        "LOOKAHEAD_UPDATED",
        "LOOKAHEAD_JUMP",
        "LOOKAHEAD_SOURCE_SWITCH",
        "LOOKAHEAD_INSIDE_OBSTACLE",
        "LOOKAHEAD_TOO_CLOSE_TO_OBSTACLE",
        "LOOKAHEAD_OUTSIDE_LOCAL_PLAN",
        "REFERENCE_AUTHORITY_MISMATCH",
        "LOCAL_PLAN_GLOBAL_PULL_SUSPECTED",
        "LOOKAHEAD_TURNBACK",
        "LOCAL_PLAN_RECAPTURE",
        "TRACE_START",
        "TRACE_UPDATE",
        "TRACE_END",
        "TRACE_ABORT",
    }
)

FOCUS_EVENTS = {
    "STOP": {
        "NAVIGATION_STOP_DIAGNOSTIC",
        "SAFETY_BLOCK",
        "SAFETY_CLAMP",
        "NO_CANDIDATE",
        "ALL_CANDIDATES_INVALID",
        "COMMAND_ZERO",
        "NAVIGATION_DEADLOCK_SUSPECTED",
    },
    "SPIN": {"SPIN_LOOP_SUSPECTED", "PLAN_EXECUTION_MISMATCH", "FSM_TRANSITION"},
    "RECOVERY": {"RECOVERY_ENTER", "RECOVERY_EXECUTION", "RECOVERY_EXIT"},
    "PLANNING": {
        "NO_CANDIDATE",
        "ALL_CANDIDATES_INVALID",
        "CANDIDATE_SELECTED",
        "PLANNING_STARVATION_SUSPECTED",
        "REPLAN",
        "REPLAN_FAILED",
        "GLOBAL_PREVIEW_UPDATED",
        "GLOBAL_PREVIEW_LIMITED",
        "GLOBAL_LOCAL_HORIZON_MISMATCH",
        "KINEMATIC_VALIDATION_STARTED",
        "KINEMATIC_VALIDATION_RESULT",
        "KINEMATIC_PATH_REJECTED",
        "KINEMATIC_CLEARANCE_WARNING",
        "KINEMATIC_SPEED_LIMITED",
    },
    "SAFETY": {"SAFETY_BLOCK", "SAFETY_CLAMP", "SAFETY_RELEASE"},
}

DEFAULT_RING_SIZE = int(os.environ.get("NAV_LOG_RING_SIZE", "8000") or 8000)
DEFAULT_API_RING = int(os.environ.get("NAV_LOG_API_RING_SIZE", "2000") or 2000)
DEFAULT_SNAP_RING = int(os.environ.get("NAV_LOG_SNAP_RING_SIZE", "400") or 400)
DEFAULT_LEVEL = (os.environ.get("NAV_LOG_LEVEL", "INFO") or "INFO").upper()
DEFAULT_SAMPLE_HZ = float(os.environ.get("NAV_LOG_SAMPLE_HZ", "4") or 4)
STOP_HOLD_S = 0.40
DEADLOCK_S = 3.0
SPIN_S = 1.2
STARVE_S = 2.0
CYCLE_DEADLINE_MS = 50.0
NOMINAL_CAND_DIST_M = 0.25  # legacy baseline only; starvation uses dynamic expected
LOCAL_HORIZON_S = 1.5
CYCLE_SEMANTICS = "diagnostic_control_refresh_cycle"  # ~20Hz debug refresh; not a planner decision gate


@dataclass
class NavLogEvent:
    ts: float
    event_id: str
    cycle_id: str
    trace_id: str
    level: str
    category: str
    event: str
    source: str = "nav"
    component: str = "observability"
    reason: Optional[str] = None
    message: Optional[str] = None
    data: Dict[str, Any] = field(default_factory=dict)
    data_ref: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        for k in ("data_ref", "reason", "message"):
            if d.get(k) is None:
                d.pop(k, None)
        if not d.get("data"):
            d.pop("data", None)
        return d


def redact_mapping(obj: Any) -> Any:
    sensitive = ("password", "token", "api_key", "apikey", "authorization", "secret", "credential")
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            kl = str(k).lower()
            out[k] = "***REDACTED***" if any(s in kl for s in sensitive) else redact_mapping(v)
        return out
    if isinstance(obj, list):
        return [redact_mapping(x) for x in obj[:64]]
    return obj


class _IdSeq:
    def __init__(self, prefix: str) -> None:
        self.prefix = prefix
        self._n = 0
        self._lock = threading.Lock()

    def next(self) -> str:
        with self._lock:
            self._n += 1
            return f"{self.prefix}-{self._n:06d}"


class NavObservability:
    """Unified diagnostic bus: events, cycles, traces, snapshots, API access.

    Read-only. Never calls planners, FSM, Safety, or Recovery.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.enabled = True
        self.level = DEFAULT_LEVEL if DEFAULT_LEVEL in LEVEL_RANK else "INFO"
        self.candidate_detail = (os.environ.get("NAV_LOG_CANDIDATE_DETAIL", "0") or "0") in (
            "1",
            "true",
            "TRUE",
            "yes",
        )
        self.api_body = (os.environ.get("NAV_LOG_API_BODY", "0") or "0") in ("1", "true", "TRUE")
        self.sample_hz = max(0.5, DEFAULT_SAMPLE_HZ)
        self._evt_seq = _IdSeq("EVT")
        self._cyc_seq = _IdSeq("NAV")
        self._trc_seq = _IdSeq("TRACE")
        self._snap_seq = _IdSeq("SNAP")
        self.events: Deque[Dict[str, Any]] = deque(maxlen=DEFAULT_RING_SIZE)
        self.api_events: Deque[Dict[str, Any]] = deque(maxlen=DEFAULT_API_RING)
        self.snapshots: Deque[Dict[str, Any]] = deque(maxlen=DEFAULT_SNAP_RING)
        self.cycles: Deque[Dict[str, Any]] = deque(maxlen=min(2000, DEFAULT_RING_SIZE))
        self.traces: Dict[str, Dict[str, Any]] = {}
        self._trace_order: Deque[str] = deque(maxlen=500)
        self._current_cycle_id = "NAV-000000"
        self._current_trace_id = "TRACE-IDLE-000000"
        self._last_summary: Dict[str, Any] = {}
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._last_cycle_ts = 0.0
        self._stop_since: Optional[float] = None
        self._deadlock_since: Optional[float] = None
        self._spin_since: Optional[float] = None
        self._starve_since: Optional[float] = None
        self._last_progress_xy: Optional[Tuple[float, float]] = None
        self._progress_ref_ts = 0.0
        self._yaw_accum = 0.0
        self._last_yaw: Optional[float] = None
        self._last_sample_emit = 0.0
        self._last_emit_ts: Dict[str, float] = {}
        self._last_api_hot = 0.0
        self._overhead_ms_ema = 0.0
        self._jsonl_path: Optional[Path] = None
        self._jsonl_fp = None
        self._jsonl_bytes = 0
        self._jsonl_max = int(os.environ.get("NAV_LOG_FILE_MAX_BYTES", str(8 * 1024 * 1024)))
        path = os.environ.get("NAV_LOG_FILE", "").strip()
        if path:
            self._open_jsonl(Path(path))
        self._prev_fsm = ""
        self._prev_recovery_action = ""
        self._prev_preview_m: Optional[float] = None
        self._prev_preview_reason: Optional[str] = None
        self._prev_path_revision: Optional[int] = None
        self._starve_streak = 0
        self._mismatch_ratio_emitted = 0.0

    def configure(
        self,
        *,
        level: Optional[str] = None,
        candidate_detail: Optional[bool] = None,
        enabled: Optional[bool] = None,
        sample_hz: Optional[float] = None,
    ) -> Dict[str, Any]:
        with self._lock:
            if enabled is not None:
                self.enabled = bool(enabled)
            if level is not None:
                lv = str(level).upper()
                if lv in LEVEL_RANK:
                    self.level = lv
            if candidate_detail is not None:
                self.candidate_detail = bool(candidate_detail)
            if sample_hz is not None:
                self.sample_hz = max(0.5, float(sample_hz))
            return {
                "enabled": self.enabled,
                "level": self.level,
                "candidate_detail": self.candidate_detail,
                "sample_hz": self.sample_hz,
                "ring_size": self.events.maxlen,
                "overhead_ms_ema": round(self._overhead_ms_ema, 3),
            }

    def _open_jsonl(self, path: Path) -> None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._jsonl_path = path
            self._jsonl_fp = open(path, "a", encoding="utf-8")
            self._jsonl_bytes = path.stat().st_size if path.exists() else 0
        except Exception:
            self._jsonl_fp = None

    def _rotate_jsonl(self) -> None:
        if not self._jsonl_path or not self._jsonl_fp:
            return
        if self._jsonl_bytes < self._jsonl_max:
            return
        try:
            self._jsonl_fp.close()
            bak = self._jsonl_path.with_suffix(self._jsonl_path.suffix + ".1")
            if bak.exists():
                bak.unlink()
            self._jsonl_path.rename(bak)
            self._jsonl_fp = open(self._jsonl_path, "a", encoding="utf-8")
            self._jsonl_bytes = 0
        except Exception:
            pass

    def _persist(self, row: Dict[str, Any]) -> None:
        if not self._jsonl_fp:
            return
        try:
            line = json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
            self._jsonl_fp.write(line)
            self._jsonl_fp.flush()
            self._jsonl_bytes += len(line.encode("utf-8"))
            self._rotate_jsonl()
        except Exception:
            pass

    def _level_ok(self, level: str) -> bool:
        return LEVEL_RANK.get(level, 0) >= LEVEL_RANK.get(self.level, 1)

    def start_cycle(self, *, kind: str = "NAV") -> str:
        cid = self._cyc_seq.next()
        with self._lock:
            self._current_cycle_id = cid
            if kind and kind != "NAV":
                pass
        return cid

    def ensure_trace(self, prefix: str = "AVOID") -> str:
        with self._lock:
            old_id = self._current_trace_id
            old = self.traces.get(old_id) if old_id else None
            self._trc_seq.next()
            n = self._trc_seq._n
            tid = f"TRACE-{prefix}-{n:06d}"
            self._current_trace_id = tid
            self.traces[tid] = {
                "trace_id": tid,
                "prefix": prefix,
                "t0": time.time(),
                "events": [],
                "outcome": None,
                "summary": {},
                "lifecycle": "ACTIVE",
            }
            self._trace_order.append(tid)
            while len(self.traces) > 500:
                old_k = self._trace_order.popleft() if self._trace_order else None
                if old_k and old_k in self.traces and old_k != tid:
                    self.traces.pop(old_k, None)
        if old and old.get("lifecycle") == "ACTIVE" and old_id and old_id != tid:
            old["lifecycle"] = "ENDED"
            self.emit(
                "TRACE_END",
                level="INFO",
                category="SYSTEM",
                reason="TRACE_SWITCH",
                data={"ended_trace_id": old_id, "next_trace_id": tid, "prefix": old.get("prefix")},
                force=True,
            )
        self.emit(
            "TRACE_START",
            level="INFO",
            category="SYSTEM",
            reason=prefix,
            data={"trace_id": tid, "prefix": prefix},
            force=True,
        )
        return tid

    def current_ids(self) -> Tuple[str, str]:
        with self._lock:
            return self._current_cycle_id, self._current_trace_id

    def emit(
        self,
        event: str,
        *,
        level: str = "INFO",
        category: str = "NAVIGATION",
        source: str = "nav",
        component: str = "observability",
        reason: Optional[str] = None,
        message: Optional[str] = None,
        data: Optional[Dict[str, Any]] = None,
        data_ref: Optional[str] = None,
        force: bool = False,
        min_interval_s: float = 0.25,
    ) -> Optional[Dict[str, Any]]:
        if not self.enabled and not force:
            return None
        lv = str(level).upper()
        if lv not in LEVEL_RANK:
            lv = "INFO"
        ev = str(event)
        critical = ev in CRITICAL_EVENTS
        if not critical and not force and not self._level_ok(lv):
            return None
        now = time.time()
        if not force and min_interval_s > 0:
            last = float(self._last_emit_ts.get(ev, 0.0) or 0.0)
            if now - last < float(min_interval_s):
                return None
            self._last_emit_ts[ev] = now
        cat = str(category).upper()
        if cat not in CATEGORIES:
            cat = "NAVIGATION"
        eid = self._evt_seq.next()
        with self._lock:
            cid = self._current_cycle_id
            tid = self._current_trace_id
        row = NavLogEvent(
            ts=now,
            event_id=eid,
            cycle_id=cid,
            trace_id=tid,
            level=lv,
            category=cat,
            event=ev,
            source=source,
            component=component,
            reason=reason,
            message=message,
            data=dict(data or {}),
            data_ref=data_ref,
        ).to_dict()
        with self._lock:
            self.events.append(row)
            tr = self.traces.get(tid)
            if tr is not None:
                tr["events"].append(eid)
                if len(tr["events"]) > 200:
                    tr["events"] = tr["events"][-200:]
        self._persist({"kind": "event", **row})
        return row

    def emit_exception(
        self,
        event: str,
        exc: BaseException,
        *,
        category: str = "ERROR",
        component: str = "observability",
    ) -> Optional[Dict[str, Any]]:
        return self.emit(
            event,
            level="ERROR",
            category=category,
            component=component,
            reason="UNKNOWN",
            message=str(exc),
            data={
                "exception_type": type(exc).__name__,
                "exception_message": str(exc),
                "stacktrace": traceback.format_exc()[-4000:],
            },
            force=True,
        )

    def store_snapshot(self, snap: Dict[str, Any]) -> str:
        sid = self._snap_seq.next()
        row = {
            "snapshot_id": sid,
            "ts": time.time(),
            "cycle_id": self._current_cycle_id,
            "data": snap,
        }
        with self._lock:
            self.snapshots.append(row)
        self._persist(
            {
                "kind": "snapshot_meta",
                "snapshot_id": sid,
                "ts": row["ts"],
                "cycle_id": row["cycle_id"],
                "keys": list(snap.keys())[:40],
            }
        )
        return sid

    def log_api_access(
        self,
        *,
        method: str,
        path: str,
        status: int,
        duration_ms: float,
        client: str = "",
        response_size: int = 0,
        error: Optional[str] = None,
        request_id: Optional[str] = None,
        body_summary: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Separate HTTP access log. Hot poll paths (especially /api/state) are sampled."""
        p = str(path or "")
        hot = p in ("/api/state", "/api/heartbeat", "/api/route/status", "/api/jason/camera/status")
        is_err = int(status) >= 400 or bool(error)
        is_slow = float(duration_ms) >= 100.0
        now = time.time()
        if hot and not is_err and not is_slow:
            last = float(self._last_api_hot or 0.0)
            if now - last < 1.0:
                return None
            self._last_api_hot = now
        rid = request_id or f"REQ-{uuid.uuid4().hex[:10]}"
        row = {
            "ts": now,
            "event_id": self._evt_seq.next(),
            "request_id": rid,
            "kind": "api_access",
            "level": "ERROR" if is_err else ("WARN" if is_slow else "INFO"),
            "category": "API",
            "event": "API_ERROR" if is_err else ("API_SLOW" if is_slow else "API_ACCESS"),
            "source": "api",
            "component": "http",
            "method": method,
            "path": p,
            "status": int(status),
            "duration_ms": round(float(duration_ms), 2),
            "client": client,
            "response_size": int(response_size),
            "error": error,
            "sampled": bool(hot and not is_err),
        }
        if self.api_body and body_summary is not None:
            row["body_summary"] = redact_mapping(body_summary)
        with self._lock:
            self.api_events.append(row)
        self._persist(row)
        return row

    def ingest_debug_snapshot(self, dbg: Dict[str, Any], cycle_ms: Optional[float] = None) -> Dict[str, Any]:
        """Build decision trace + detectors from existing NavDebugHub snapshot. No side effects."""
        t0 = time.perf_counter()
        if not self.enabled:
            return {"skipped": True}
        if not isinstance(dbg, dict):
            dbg = {}
        self.start_cycle()
        now = time.time()

        pose = dbg.get("pose") if isinstance(dbg.get("pose"), dict) else {}
        if not pose:
            pt = dbg.get("pose_trace")
            if isinstance(pt, list) and pt and isinstance(pt[-1], dict):
                pose = pt[-1]
        if not pose and "x" in dbg:
            pose = {"x": dbg.get("x"), "y": dbg.get("y"), "yaw": dbg.get("yaw") or dbg.get("theta")}

        vc = dbg.get("velocity_chain") if isinstance(dbg.get("velocity_chain"), dict) else {}
        phase4 = dbg.get("phase4") if isinstance(dbg.get("phase4"), dict) else {}
        man = dbg.get("maneuver") if isinstance(dbg.get("maneuver"), dict) else {}
        policy = dbg.get("nav_policy") if isinstance(dbg.get("nav_policy"), dict) else {}
        if not policy:
            policy = dbg.get("policy") if isinstance(dbg.get("policy"), dict) else {}
        if not policy:
            policy = phase4.get("policy") if isinstance(phase4.get("policy"), dict) else {}
        recovery = dbg.get("recovery") if isinstance(dbg.get("recovery"), dict) else {}
        if not recovery:
            recovery = phase4.get("recovery") if isinstance(phase4.get("recovery"), dict) else {}
        probe = dbg.get("probe") if isinstance(dbg.get("probe"), dict) else {}
        if not probe:
            probe = phase4.get("probe") if isinstance(phase4.get("probe"), dict) else {}
        local = dbg.get("local_maneuver") if isinstance(dbg.get("local_maneuver"), dict) else {}
        if not local:
            local = man.get("local_compare") if isinstance(man.get("local_compare"), dict) else {}
        safety = dbg.get("safety") if isinstance(dbg.get("safety"), dict) else {}
        if not safety:
            safety = phase4.get("safety") if isinstance(phase4.get("safety"), dict) else {}
        geom = dbg.get("geometry") if isinstance(dbg.get("geometry"), dict) else {}
        if not geom and isinstance(phase4.get("vehicle_geometry"), dict):
            geom = phase4["vehicle_geometry"]
        radar = dbg.get("radar") if isinstance(dbg.get("radar"), dict) else {}
        status = dbg.get("status") if isinstance(dbg.get("status"), dict) else {}
        selector = phase4.get("selector") if isinstance(phase4.get("selector"), dict) else {}

        req_vx = _f(vc.get("cmd_vx"), vc.get("mppi_vx"), dbg.get("cmd_vx"), dbg.get("mppi_vx"))
        req_w = _f(vc.get("cmd_w"), vc.get("mppi_w"), dbg.get("cmd_w"), dbg.get("mppi_w"))
        safe_vx = _f(vc.get("safe_vx"), dbg.get("safe_vx"), safety.get("safe_vx"))
        safe_w = _f(vc.get("safe_w"), dbg.get("safe_w"), safety.get("safe_w"))
        state_vx = _f(vc.get("state_vx"), dbg.get("state_vx"), status.get("vx"))
        state_w = _f(vc.get("state_w"), dbg.get("state_w"), status.get("w"))

        candidates_raw = _extract_candidates(dbg, local, man)
        cand_summaries = [_candidate_summary(c, detail=self.candidate_detail) for c in candidates_raw]
        cand_count = len(cand_summaries)
        valid_count = sum(1 for c in cand_summaries if c.get("valid"))

        selected = (
            local.get("selected")
            or local.get("decision")
            or man.get("decision")
            or dbg.get("selected_candidate")
            or selector.get("selected")
            or selector.get("selected_token")
            or selector.get("selected_side")
        )
        if selected is None:
            mode_tok = str(man.get("mode") or "")
            if mode_tok.upper() in (
                "LEFT",
                "RIGHT",
                "FORWARD",
                "REVERSE",
                "ALIGN",
                "REPOSITION",
                "WAIT",
            ):
                selected = mode_tok
        if selected is not None:
            selected = str(selected)

        paths = dbg.get("paths") if isinstance(dbg.get("paths"), dict) else {}
        gpath = (
            paths.get("processed_global")
            or dbg.get("processed_path")
            or dbg.get("global_path")
            or []
        )
        if isinstance(gpath, dict):
            gpath = gpath.get("points") or []
        path_len = _path_length(gpath)
        goal_dist = _f(dbg.get("goal_distance"), status.get("goal_distance"), (dbg.get("goal") or {}).get("distance") if isinstance(dbg.get("goal"), dict) else None)

        fsm_mode = str(man.get("mode") or status.get("phase") or dbg.get("phase") or dbg.get("nav_mode") or "")
        pol_dec = policy.get("decision") if isinstance(policy.get("decision"), dict) else {}
        policy_action = str(
            pol_dec.get("action")
            or policy.get("action")
            or policy.get("behavior")
            or policy.get("state")
            or ""
        )
        rec_action_raw = str(recovery.get("action") or "")
        rec_action = "" if rec_action_raw.upper() in ("NONE", "", "NULL", "NA", "NOT_AVAILABLE", "NOT_IMPLEMENTED") else rec_action_raw
        rec_active = bool(recovery.get("active")) or bool(rec_action)

        want_prefix = "NAV"
        if rec_active:
            want_prefix = "RECOVERY"
        elif "REPLAN" in fsm_mode.upper() or str(dbg.get("stop_reason") or status.get("stop_reason") or "").upper().startswith("REPLAN"):
            want_prefix = "REPLAN"
        elif policy_action or fsm_mode:
            want_prefix = "AVOID"
        with self._lock:
            cur = self._current_trace_id
        if want_prefix not in cur or cur.endswith("000000"):
            self.ensure_trace(want_prefix)

        max_cand_dist = 0.0
        avg_cand_dist = 0.0
        if cand_summaries:
            dists = [float(c.get("distance_m") or 0.0) for c in cand_summaries]
            max_cand_dist = max(dists) if dists else 0.0
            avg_cand_dist = sum(dists) / max(1, len(dists))

        gref = dbg.get("global_reference") if isinstance(dbg.get("global_reference"), dict) else {}
        gvl = dbg.get("global_vs_local") if isinstance(dbg.get("global_vs_local"), dict) else {}
        loc_layer = dbg.get("local_candidates") if isinstance(dbg.get("local_candidates"), dict) else {}
        if loc_layer.get("max_distance_m") is not None:
            try:
                max_cand_dist = max(max_cand_dist, float(loc_layer.get("max_distance_m") or 0.0))
            except (TypeError, ValueError):
                pass
        try:
            from agv_bridge.nav_global_preview import expected_local_distance_m

            expected_nom = float(
                gvl.get("expected_local_distance_m")
                if gvl.get("expected_local_distance_m") is not None
                else expected_local_distance_m(state_vx=state_vx, requested_vx=req_vx, horizon_s=LOCAL_HORIZON_S)
            )
        except Exception:
            expected_nom = float(NOMINAL_CAND_DIST_M)

        x = _f(pose.get("x"), dbg.get("x")) or 0.0
        y = _f(pose.get("y"), dbg.get("y")) or 0.0
        yaw = _f(pose.get("yaw"), pose.get("theta"), dbg.get("yaw"), dbg.get("theta")) or 0.0

        prev_pose = self._last_pose
        dx = dy = dist_prog = 0.0
        yaw_delta = 0.0
        if prev_pose is not None:
            dx = x - prev_pose[0]
            dy = y - prev_pose[1]
            dist_prog = math.hypot(dx, dy)
            yaw_delta = _wrap(yaw - prev_pose[2])
        self._last_pose = (x, y, yaw)

        if self._last_progress_xy is None:
            self._last_progress_xy = (x, y)
            self._progress_ref_ts = now
        else:
            moved = math.hypot(x - self._last_progress_xy[0], y - self._last_progress_xy[1])
            if moved >= 0.08:
                self._last_progress_xy = (x, y)
                self._progress_ref_ts = now

        nav_mode = str(dbg.get("nav_mode") or status.get("nav_mode") or "")
        stop_reason_raw = str(dbg.get("stop_reason") or status.get("stop_reason") or "")
        intentional_stop = nav_mode in ("idle", "arrived", "manual") or stop_reason_raw in (
            "GOAL_REACHED",
            "STOP_GOAL",
            "MANUAL_STOP",
        )
        goal_reached = nav_mode == "arrived" or stop_reason_raw in ("GOAL_REACHED", "STOP_GOAL")

        owner = infer_likely_owner(
            candidate_count=cand_count,
            valid_candidate_count=valid_count,
            selected_candidate=selected,
            requested_vx=req_vx,
            requested_w=req_w,
            safe_vx=safe_vx,
            safe_w=safe_w,
            state_vx=state_vx,
            state_w=state_w,
            recovery_active=rec_active,
            intentional_stop=intentional_stop,
            goal_reached=goal_reached,
        )
        immediate_owner = owner
        likely_root_owner = None if owner in ("UNKNOWN", "NONE") else owner

        front_near = _rf(dbg.get("front_near") or radar.get("front_near"))
        rear_near = _rf(dbg.get("rear_near") or radar.get("rear_near"))

        decision = _build_decision(
            x=x,
            y=y,
            yaw=yaw,
            state_vx=state_vx,
            state_w=state_w,
            front_near=front_near,
            rear_near=rear_near,
            left_near=_rf(dbg.get("left_near")),
            right_near=_rf(dbg.get("right_near")),
            goal_dist=goal_dist,
            gpath=gpath if isinstance(gpath, list) else [],
            path_len=path_len,
            fsm_mode=fsm_mode,
            geom=geom,
            probe=probe,
            policy=policy,
            pol_dec=pol_dec,
            policy_action=policy_action,
            recovery=recovery,
            rec_action=rec_action,
            rec_active=rec_active,
            req_vx=req_vx,
            req_w=req_w,
            safe_vx=safe_vx,
            safe_w=safe_w,
            cand_count=cand_count,
            valid_count=valid_count,
            cand_summaries=cand_summaries,
            max_cand_dist=max_cand_dist,
            avg_cand_dist=avg_cand_dist,
            expected_nom=expected_nom,
            candidate_detail=self.candidate_detail,
            selected=selected,
            local=local,
            man=man,
            prev_fsm=self._prev_fsm,
            vc=vc,
            dbg=dbg,
            safety=safety,
            prev_pose=prev_pose,
            dx=dx,
            dy=dy,
            dist_prog=dist_prog,
            last_yaw=self._last_yaw,
            yaw_delta=yaw_delta,
            owner=owner,
            immediate_owner=immediate_owner,
            likely_root_owner=likely_root_owner,
            cycle_ms=cycle_ms,
            phase4=phase4,
            gref=gref,
            gvl=gvl,
            loc_layer=loc_layer,
        )

        self._run_detectors(
            now=now,
            decision=decision,
            stopped=state_vx is not None and abs(state_vx) < VX_EPS,
            intentional_stop=intentional_stop,
            goal_reached=goal_reached,
            owner=owner,
            fsm_mode=fsm_mode,
            policy_action=policy_action,
            rec_action=rec_action,
            rec_active=rec_active,
            cand_count=cand_count,
            valid_count=valid_count,
            cand_summaries=cand_summaries,
            selected=selected,
            req_vx=req_vx,
            req_w=req_w,
            safe_vx=safe_vx,
            safe_w=safe_w,
            state_vx=state_vx,
            state_w=state_w,
            gpath=gpath if isinstance(gpath, list) else [],
            dist_prog=dist_prog,
            yaw=yaw,
            max_cand_dist=max_cand_dist,
            avg_cand_dist=avg_cand_dist,
            expected_nom=expected_nom,
            goal_dist=goal_dist,
            man=man,
            cycle_ms=cycle_ms,
            gref=gref,
            gvl=gvl,
        )

        overhead_ms = (time.perf_counter() - t0) * 1000.0
        decision["performance"]["logging_overhead_ms"] = round(overhead_ms, 3)
        self._overhead_ms_ema = (
            overhead_ms if self._overhead_ms_ema <= 0 else (0.85 * self._overhead_ms_ema + 0.15 * overhead_ms)
        )
        lf = dbg.get("lookahead_forensics") if isinstance(dbg.get("lookahead_forensics"), dict) else {}
        if lf:
            decision["lookahead_forensics"] = {
                "display": lf.get("display_lookahead") or {},
                "pp": lf.get("pp_lookahead") or {},
                "authority": lf.get("authority") or {},
                "controller": lf.get("controller") or {},
                "three_headings": lf.get("three_headings") or {},
                "diagnostics": lf.get("diagnostics") or {},
            }
            diag_lf = lf.get("diagnostics") if isinstance(lf.get("diagnostics"), dict) else {}
            for k in (
                "global_heading_deg",
                "local_heading_deg",
                "lookahead_heading_deg",
                "global_error_deg",
                "local_error_deg",
                "lookahead_error_deg",
                "reference_conflict",
                "obstacle_pass_state",
                "display_vs_pp_separation_m",
            ):
                if k in diag_lf:
                    decision["diagnostics"][k] = diag_lf[k]
            auth = lf.get("authority") if isinstance(lf.get("authority"), dict) else {}
            if auth.get("active_reference"):
                decision["diagnostics"]["active_reference_authority"] = auth.get("active_reference")

        cycle_row = {
            "ts": now,
            "cycle_id": self._current_cycle_id,
            "trace_id": self._current_trace_id,
            "decision": decision,
            "logging_overhead_ms": round(overhead_ms, 3),
        }
        with self._lock:
            self.cycles.append(cycle_row)
            self._last_cycle_ts = now
            self._last_summary = {
                "ts": now,
                "cycle_id": self._current_cycle_id,
                "trace_id": self._current_trace_id,
                "current_mode": fsm_mode,
                "current_policy": policy_action,
                "current_candidate": selected or "NONE",
                "candidate_count": cand_count,
                "valid_candidate_count": valid_count,
                "requested_vx": req_vx,
                "requested_w": req_w,
                "safe_vx": safe_vx,
                "safe_w": safe_w,
                "state_vx": state_vx,
                "state_w": state_w,
                "last_event": self.events[-1] if self.events else None,
                "last_warning": _last_level(self.events, "WARN"),
                "last_error": _last_level(self.events, "ERROR"),
                "stop_reason": decision["diagnostics"].get("stop_reason"),
                "likely_owner": owner,
                "immediate_owner": immediate_owner,
                "likely_root_owner": likely_root_owner,
                "spin_suspected": decision["diagnostics"]["spin_suspected"],
                "deadlock_suspected": decision["diagnostics"]["deadlock_suspected"],
                "planning_starvation_suspected": decision["diagnostics"]["planning_starvation_suspected"],
                "plan_execution_mismatch": decision["diagnostics"]["plan_execution_mismatch"],
                "logging_overhead_ms_ema": round(self._overhead_ms_ema, 3),
                "max_candidate_distance_m": round(max_cand_dist, 3),
                "expected_nominal_distance_m": round(expected_nom, 3),
                "legacy_nominal_baseline_m": NOMINAL_CAND_DIST_M,
                "global_preview_m": gref.get("preview_m") if gref else gvl.get("global_preview_m"),
                "global_remaining_m": gref.get("remaining_m") if gref else gvl.get("global_remaining_m"),
                "global_preview_reason": gref.get("preview_reason") if gref else gvl.get("global_preview_reason"),
                "global_path_revision": gref.get("path_revision") if gref else gvl.get("global_path_revision"),
                "kinematic_status": (dbg.get("kinematic_validation") or {}).get("kinematic_status")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else gref.get("kinematic_status"),
                "kinematic_valid": (dbg.get("kinematic_validation") or {}).get("kinematic_valid")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else gref.get("kinematic_valid"),
                "max_curvature": (dbg.get("kinematic_validation") or {}).get("max_curvature")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else gref.get("max_curvature"),
                "min_turn_radius_m": (dbg.get("kinematic_validation") or {}).get("min_turn_radius_m")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else gref.get("min_turn_radius_m"),
                "first_invalid_distance_m": (dbg.get("kinematic_validation") or {}).get("first_invalid_distance_m")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else gref.get("first_invalid_distance_m"),
                "speed_limited": (dbg.get("kinematic_validation") or {}).get("speed_limited")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else gref.get("speed_limited"),
                "validation_revision": (dbg.get("kinematic_validation") or {}).get("validation_revision")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else None,
                "cache_hit": (dbg.get("kinematic_validation") or {}).get("cache_hit")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else None,
                "compute_ms": (dbg.get("kinematic_validation") or {}).get("compute_ms")
                if isinstance(dbg.get("kinematic_validation"), dict)
                else None,
                "local_max_distance_m": round(max_cand_dist, 3),
                "global_vs_local": {
                    "global_preview_m": gref.get("preview_m") if gref else gvl.get("global_preview_m"),
                    "local_max_distance_m": round(max_cand_dist, 3),
                    "selected_candidate": selected or "NONE",
                    "expected_local_distance_m": round(expected_nom, 3),
                },
            }
            tr = self.traces.get(self._current_trace_id)
            if tr is not None:
                tr["summary"] = dict(self._last_summary)
                tr["outcome"] = {
                    "likely_owner": owner,
                    "immediate_owner": immediate_owner,
                    "likely_root_owner": likely_root_owner,
                    "mode": fsm_mode,
                    "selected": selected or "NONE",
                }

        return {
            "cycle_id": self._current_cycle_id,
            "trace_id": self._current_trace_id,
            "likely_owner": owner,
            "immediate_owner": immediate_owner,
            "likely_root_owner": likely_root_owner,
            "logging_overhead_ms": round(overhead_ms, 3),
            "decision": decision,
        }

    def _run_detectors(
        self,
        *,
        now: float,
        decision: Dict[str, Any],
        stopped: bool,
        intentional_stop: bool,
        goal_reached: bool,
        owner: str,
        fsm_mode: str,
        policy_action: str,
        rec_action: str,
        rec_active: bool,
        cand_count: int,
        valid_count: int,
        cand_summaries: List[Dict[str, Any]],
        selected: Any,
        req_vx: Optional[float],
        req_w: Optional[float],
        safe_vx: Optional[float],
        safe_w: Optional[float],
        state_vx: Optional[float],
        state_w: Optional[float],
        gpath: List[Any],
        dist_prog: float,
        yaw: float,
        max_cand_dist: float,
        avg_cand_dist: float,
        expected_nom: float,
        goal_dist: Optional[float],
        man: Dict[str, Any],
        cycle_ms: Optional[float],
        gref: Optional[Dict[str, Any]] = None,
        gvl: Optional[Dict[str, Any]] = None,
    ) -> None:
        gref = gref or {}
        gvl = gvl or {}
        # STOP diagnostic: vx≈0 for ~0.4s, then cooldown
        if stopped and not intentional_stop and not goal_reached:
            if self._stop_since is None:
                self._stop_since = now
            elif now - self._stop_since >= STOP_HOLD_S:
                decision["diagnostics"]["stop_reason"] = decision["diagnostics"]["stop_reason"] or "VX_NEAR_ZERO"
                snap_id = self.store_snapshot(
                    {
                        "decision": decision,
                        "selected": selected,
                        "candidates": cand_summaries[:12],
                        "global_vs_local": {
                            "global_preview_m": gref.get("preview_m") or gvl.get("global_preview_m"),
                            "local_max_distance_m": round(max_cand_dist, 3),
                            "selected_candidate": selected or "NONE",
                        },
                    }
                )
                self.emit(
                    "NAVIGATION_STOP_DIAGNOSTIC",
                    level="WARN",
                    category="DIAGNOSTIC",
                    reason=_stop_reason_enum(decision),
                    message=f"stop likely_owner={owner}",
                    data={
                        "likely_owner": owner,
                        "immediate_owner": owner,
                        "likely_root_owner": None if owner in ("UNKNOWN", "NONE") else owner,
                        "mode": fsm_mode,
                        "policy_action": policy_action,
                        "recovery_action": rec_action,
                        "candidate_count": cand_count,
                        "valid_candidate_count": valid_count,
                        "selected_candidate": selected or "NONE",
                        "requested_vx": req_vx,
                        "requested_w": req_w,
                        "safe_vx": safe_vx,
                        "safe_w": safe_w,
                        "state_vx": state_vx,
                        "state_w": state_w,
                        "safety_intervention": decision["safety"]["blocked"] or decision["safety"]["clamped"],
                        "global_path_exists": bool(gpath),
                        "global_preview_m": gref.get("preview_m") or gvl.get("global_preview_m"),
                        "global_remaining_m": gref.get("remaining_m") or gvl.get("global_remaining_m"),
                        "local_max_distance_m": round(max_cand_dist, 3),
                        "expected_nominal_distance_m": round(expected_nom, 3),
                        "stop_reason": decision["diagnostics"]["stop_reason"],
                    },
                    data_ref=snap_id,
                    force=True,
                )
                self._stop_since = now + 2.0
        else:
            self._stop_since = None

        stall_s = now - self._progress_ref_ts if self._progress_ref_ts else 0.0
        if (
            not goal_reached
            and not intentional_stop
            and stall_s >= DEADLOCK_S
            and (state_vx is None or abs(state_vx) < VX_EPS)
        ):
            decision["diagnostics"]["deadlock_suspected"] = True
            last_dl = float(self._deadlock_since or 0.0)
            if now - last_dl >= 2.0:
                self.emit(
                    "NAVIGATION_DEADLOCK_SUSPECTED",
                    level="WARN",
                    category="DIAGNOSTIC",
                    reason="NO_TRANSLATION_PROGRESS",
                    data={
                        "duration_s": round(stall_s, 2),
                        "last_progress_xy": self._last_progress_xy,
                        "last_selected_candidate": selected or "NONE",
                        "last_requested_command": {"vx": req_vx, "w": req_w},
                        "last_safe_command": {"vx": safe_vx, "w": safe_w},
                        "last_state_command": {"vx": state_vx, "w": state_w},
                        "likely_owner": owner,
                    },
                    force=True,
                )
                self._deadlock_since = now

        if self._last_yaw is not None:
            self._yaw_accum += abs(_wrap(yaw - self._last_yaw))
        self._last_yaw = yaw
        spinning = (
            state_vx is not None
            and abs(state_vx) < VX_EPS
            and state_w is not None
            and abs(state_w) > W_EPS
            and abs(dist_prog) < 0.02
        )
        if spinning:
            if self._spin_since is None:
                self._spin_since = now
                self._yaw_accum = 0.0
            elif now - self._spin_since >= SPIN_S:
                decision["diagnostics"]["spin_suspected"] = True
                self.emit(
                    "SPIN_LOOP_SUSPECTED",
                    level="WARN",
                    category="DIAGNOSTIC",
                    reason="NO_TRANSLATION_PROGRESS",
                    data={
                        "spin_duration_s": round(now - self._spin_since, 2),
                        "yaw_delta": round(self._yaw_accum, 3),
                        "translation_delta": round(dist_prog, 4),
                        "current_fsm_mode": fsm_mode,
                        "policy_action": policy_action,
                        "candidate": selected or "NONE",
                        "state_w": state_w,
                    },
                    force=True,
                )
                self._spin_since = now
        else:
            self._spin_since = None

        abs_min = 0.08
        starve_thresh = max(0.5 * max(expected_nom, 1e-3), abs_min)
        short = cand_count > 0 and max_cand_dist > 0 and max_cand_dist < starve_thresh
        if short and not intentional_stop and not goal_reached:
            if self._starve_since is None:
                self._starve_since = now
                self._starve_streak = 1
            else:
                self._starve_streak += 1
            if now - self._starve_since >= STARVE_S and self._starve_streak >= 3:
                decision["diagnostics"]["planning_starvation_suspected"] = True
                self.emit(
                    "PLANNING_STARVATION_SUSPECTED",
                    level="WARN",
                    category="LOCAL_PLANNING",
                    data={
                        "candidate_count": cand_count,
                        "valid_candidate_count": valid_count,
                        "average_candidate_distance": round(avg_cand_dist, 3),
                        "max_candidate_distance": round(max_cand_dist, 3),
                        "expected_nominal_distance": round(expected_nom, 3),
                        "starve_threshold_m": round(starve_thresh, 3),
                        "legacy_baseline_m": NOMINAL_CAND_DIST_M,
                        "global_path_remaining": _rf(goal_dist),
                        "global_preview_m": gref.get("preview_m") or gvl.get("global_preview_m"),
                        "note": "fact only — short local vs dynamic expected; not a bug verdict",
                    },
                    force=True,
                )
                self._starve_since = now
                self._starve_streak = 0
        else:
            self._starve_since = None
            self._starve_streak = 0

        # P0-B Global Preview events (diagnostics only)
        try:
            preview_m = _f(gref.get("preview_m"), gvl.get("global_preview_m"))
            reason = str(gref.get("preview_reason") or gvl.get("global_preview_reason") or "")
            rev = gref.get("path_revision")
            if rev is None:
                rev = gvl.get("global_path_revision")
            try:
                rev_i = int(rev) if rev is not None else None
            except (TypeError, ValueError):
                rev_i = None
            meaningful = False
            if preview_m is not None and self._prev_preview_m is not None:
                meaningful = abs(preview_m - self._prev_preview_m) >= 0.40
            if self._prev_preview_reason is not None and reason and reason != self._prev_preview_reason:
                meaningful = True
            if rev_i is not None and self._prev_path_revision is not None and rev_i != self._prev_path_revision:
                meaningful = True
            if self._prev_preview_m is None and preview_m is not None and preview_m > 0.5:
                meaningful = True
            if meaningful:
                self.emit(
                    "GLOBAL_PREVIEW_UPDATED",
                    level="INFO",
                    category="GLOBAL_PLANNING",
                    data={
                        "preview_m": _rf(preview_m),
                        "remaining_m": _rf(gref.get("remaining_m") or gvl.get("global_remaining_m")),
                        "preview_reason": reason or None,
                        "path_revision": rev_i,
                        "first_turn_distance_m": _rf(gref.get("first_turn_distance_m")),
                        "heading_change_deg": _rf(gref.get("heading_change_deg")),
                        "kinematic_valid": gref.get("kinematic_valid"),
                        "kinematic_status": gref.get("kinematic_status"),
                        "status": gref.get("status"),
                    },
                    force=True,
                )
            if reason in ("GOAL_LIMITED", "PATH_LIMITED") and reason != self._prev_preview_reason:
                self.emit(
                    "GLOBAL_PREVIEW_LIMITED",
                    level="INFO",
                    category="GLOBAL_PLANNING",
                    data={
                        "preview_m": _rf(preview_m),
                        "remaining_m": _rf(gref.get("remaining_m") or gvl.get("global_remaining_m")),
                        "preview_reason": reason,
                        "path_revision": rev_i,
                    },
                    force=True,
                )
            if preview_m is not None and max_cand_dist > 0.05 and preview_m >= 2.0:
                ratio = preview_m / max(max_cand_dist, 1e-3)
                if ratio >= 8.0 and abs(ratio - self._mismatch_ratio_emitted) >= 2.0:
                    self.emit(
                        "GLOBAL_LOCAL_HORIZON_MISMATCH",
                        level="NOTICE",
                        category="DIAGNOSTIC",
                        reason="HORIZON_SCALE_GAP",
                        data={
                            "global_preview_m": round(preview_m, 3),
                            "local_max_distance_m": round(max_cand_dist, 3),
                            "ratio": round(ratio, 2),
                            "expected_local_distance_m": round(expected_nom, 3),
                            "note": "fact only — scale gap expected until P1; not a bug verdict",
                        },
                        force=True,
                    )
                    self._mismatch_ratio_emitted = ratio
            if preview_m is not None:
                self._prev_preview_m = preview_m
            if reason:
                self._prev_preview_reason = reason
            if rev_i is not None:
                self._prev_path_revision = rev_i
        except Exception:
            pass

        mismatch = False
        mismatch_why = None
        if selected and str(selected).upper() not in ("NONE", "SAFE_STOP", "IDLE"):
            if req_vx is not None and abs(req_vx) > VX_EPS and safe_vx is not None and abs(safe_vx) < VX_EPS:
                mismatch = True
                mismatch_why = "SELECTED_FORWARD_BUT_SAFE_ZERO"
            elif safe_vx is not None and abs(safe_vx) > VX_EPS and state_vx is not None and abs(state_vx) < VX_EPS:
                mismatch = True
                mismatch_why = "SAFE_VX_BUT_STATE_ZERO"
            sel_u = str(selected).upper()
            fsm_u = fsm_mode.upper()
            if ("LEFT" in sel_u or "RIGHT" in sel_u or "FORWARD" in sel_u) and "TURN" in fsm_u:
                mismatch = True
                mismatch_why = "SELECTED_SIDE_BUT_FSM_TURN"
        if mismatch:
            decision["diagnostics"]["plan_execution_mismatch"] = True
            self.emit(
                "PLAN_EXECUTION_MISMATCH",
                level="WARN",
                category="DIAGNOSTIC",
                reason=mismatch_why or "UNKNOWN",
                data={
                    "planned": selected,
                    "fsm": fsm_mode,
                    "requested": {"vx": req_vx, "w": req_w},
                    "safe": {"vx": safe_vx, "w": safe_w},
                    "actual": {"vx": state_vx, "w": state_w},
                    "why": mismatch_why,
                },
                min_interval_s=1.0,
            )

        if req_vx is not None and safe_vx is not None and abs(req_vx) > VX_EPS and abs(safe_vx) < VX_EPS:
            self.emit(
                "SAFETY_CLAMP",
                level="WARN",
                category="SAFETY",
                reason="SAFETY_CLAMP",
                data={
                    "original_vx": req_vx,
                    "original_w": req_w,
                    "safe_vx": safe_vx,
                    "safe_w": safe_w,
                    "front_distance": decision["safety"]["front_distance"],
                    "reason": decision["safety"]["reason"],
                },
                min_interval_s=0.5,
            )

        if self._prev_fsm and fsm_mode and self._prev_fsm != fsm_mode:
            self.emit(
                "FSM_TRANSITION",
                level="NOTICE",
                category="FSM",
                data={"from": self._prev_fsm, "to": fsm_mode, "reason": man.get("reason")},
                force=True,
            )
        self._prev_fsm = fsm_mode

        prev_rec = self._prev_recovery_action
        prev_rec_active = bool(prev_rec) and prev_rec.upper() not in (
            "NONE",
            "",
            "NULL",
            "NA",
            "NOT_AVAILABLE",
            "NOT_IMPLEMENTED",
        )
        if rec_action != prev_rec:
            if rec_active:
                self.emit(
                    "RECOVERY_ENTER" if not prev_rec_active else "RECOVERY_EXECUTION",
                    level="NOTICE",
                    category="RECOVERY",
                    data={
                        "action": rec_action,
                        "requested_vx": req_vx,
                        "safe_vx": safe_vx,
                        "state_vx": state_vx,
                        "signed_progress_m": decision["recovery"].get("signed_progress_m"),
                    },
                    force=True,
                )
            elif prev_rec_active:
                self.emit(
                    "RECOVERY_EXIT",
                    level="NOTICE",
                    category="RECOVERY",
                    data={"previous_action": prev_rec},
                    force=True,
                )
            self._prev_recovery_action = rec_action

        if cand_count == 0 and not intentional_stop:
            self.emit(
                "NO_CANDIDATE",
                level="WARN",
                category="CANDIDATE",
                reason="NO_CANDIDATE",
                data={"mode": fsm_mode},
                min_interval_s=0.5,
            )
        elif cand_count > 0 and valid_count == 0:
            self.emit(
                "ALL_CANDIDATES_INVALID",
                level="WARN",
                category="CANDIDATE",
                reason="ALL_CANDIDATES_INVALID",
                data={"candidates": [c.get("candidate_id") for c in cand_summaries[:8]]},
                min_interval_s=0.5,
            )
        elif selected and str(selected).upper() != "NONE":
            if now - self._last_sample_emit >= (1.0 / max(self.sample_hz, 0.5)):
                self.emit(
                    "CANDIDATE_SELECTED",
                    level="INFO",
                    category="CANDIDATE",
                    data={"selected": selected, "valid_count": valid_count, "count": cand_count},
                )
                self._last_sample_emit = now

        if cycle_ms is not None and float(cycle_ms) > CYCLE_DEADLINE_MS:
            self.emit(
                "NAV_CYCLE_OVERRUN",
                level="WARN",
                category="PERFORMANCE",
                data={"cycle_duration_ms": round(float(cycle_ms), 2), "deadline_ms": CYCLE_DEADLINE_MS},
                force=True,
            )

    def query_events(
        self,
        *,
        level: Optional[str] = None,
        category: Optional[str] = None,
        event: Optional[str] = None,
        source: Optional[str] = None,
        component: Optional[str] = None,
        trace_id: Optional[str] = None,
        cycle_id: Optional[str] = None,
        from_ts: Optional[float] = None,
        to_ts: Optional[float] = None,
        since: Optional[str] = None,
        focus: Optional[str] = None,
        limit: int = 200,
        cursor: Optional[str] = None,
        include_api: bool = False,
    ) -> Dict[str, Any]:
        limit = max(1, min(int(limit or 200), 200))
        with self._lock:
            rows = list(self.events)
            if include_api:
                rows = rows + list(self.api_events)
        rows.sort(key=lambda r: float(r.get("ts") or 0))

        since_ts = None
        since_id = None
        if since not in (None, ""):
            try:
                since_ts = float(since)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                since_id = str(since)

        focus_set = FOCUS_EVENTS.get(str(focus or "").upper())
        level_set = None
        level_min = None
        if level:
            raw = str(level).upper().strip()
            if "," in raw:
                level_set = {x.strip() for x in raw.split(",") if x.strip()}
            elif raw.endswith("+"):
                base = raw[:-1]
                if base in LEVEL_RANK:
                    level_min = LEVEL_RANK[base]
            elif raw in LEVEL_RANK:
                level_min = LEVEL_RANK[raw]

        out: List[Dict[str, Any]] = []
        started = cursor is None and since_id is None
        for r in rows:
            eid = str(r.get("event_id") or "")
            ts = float(r.get("ts") or 0)
            if cursor and not started:
                if eid == str(cursor):
                    started = True
                continue
            if since_id and not started:
                if eid == since_id:
                    started = True
                    continue
                continue
            if since_ts is not None and ts < since_ts:
                continue
            if not started:
                started = True
            if level_set is not None:
                if str(r.get("level") or "").upper() not in level_set:
                    continue
            elif level_min is not None:
                if LEVEL_RANK.get(str(r.get("level") or ""), -1) < level_min:
                    continue
            if category and str(r.get("category") or "").upper() != str(category).upper():
                continue
            if event and str(r.get("event") or "") != str(event):
                continue
            if source and str(r.get("source") or "") != str(source):
                continue
            if component and str(r.get("component") or "") != str(component):
                continue
            if trace_id and str(r.get("trace_id") or "") != str(trace_id):
                continue
            if cycle_id and str(r.get("cycle_id") or "") != str(cycle_id):
                continue
            if from_ts is not None and ts < float(from_ts):
                continue
            if to_ts is not None and ts > float(to_ts):
                continue
            if focus_set and str(r.get("event") or "") not in focus_set:
                continue
            out.append(r)
            if len(out) >= limit + 1:
                break
        has_more = len(out) > limit
        out = out[:limit]
        next_cursor = out[-1]["event_id"] if out else cursor
        return {
            "success": True,
            "events": out,
            "count": len(out),
            "has_more": has_more,
            "next_cursor": next_cursor,
            "limit": limit,
        }

    def summary(self) -> Dict[str, Any]:
        with self._lock:
            s = dict(self._last_summary or {})
            s["success"] = True
            s["overhead_ms_ema"] = round(self._overhead_ms_ema, 3)
            s["event_count"] = len(self.events)
            s["api_event_count"] = len(self.api_events)
            s["config"] = {
                "level": self.level,
                "candidate_detail": self.candidate_detail,
                "enabled": self.enabled,
            }
            return s

    def get_trace(self, trace_id: str) -> Dict[str, Any]:
        with self._lock:
            tr = self.traces.get(trace_id)
            if not tr:
                return {"success": False, "message": "trace not found", "trace_id": trace_id}
            eids = set(tr.get("events") or [])
            events = [e for e in self.events if e.get("event_id") in eids]
            return {
                "success": True,
                "trace_id": trace_id,
                "metadata": {"t0": tr.get("t0"), "prefix": tr.get("prefix")},
                "summary": tr.get("summary") or {},
                "outcome": tr.get("outcome"),
                "events": events,
            }

    def get_cycle(self, cycle_id: str) -> Dict[str, Any]:
        with self._lock:
            for c in reversed(self.cycles):
                if c.get("cycle_id") == cycle_id:
                    return {"success": True, "cycle": c}
            events = [e for e in self.events if e.get("cycle_id") == cycle_id]
            if events:
                return {"success": True, "cycle_id": cycle_id, "events": events}
        return {"success": False, "message": "cycle not found", "cycle_id": cycle_id}

    def diagnostics_window(self, window_s: float = 10.0) -> Dict[str, Any]:
        window_s = max(1.0, min(float(window_s or 10.0), 60.0))
        now = time.time()
        with self._lock:
            events = [e for e in self.events if now - float(e.get("ts") or 0) <= window_s]
            cycles = [c for c in self.cycles if now - float(c.get("ts") or 0) <= window_s]
            summary = dict(self._last_summary or {})
        return {
            "success": True,
            "window_s": window_s,
            "summary": summary,
            "events": events[-200:],
            "cycles": [
                {
                    "ts": c.get("ts"),
                    "cycle_id": c.get("cycle_id"),
                    "trace_id": c.get("trace_id"),
                    "likely_owner": ((c.get("decision") or {}).get("diagnostics") or {}).get("likely_owner"),
                    "selected": ((c.get("decision") or {}).get("selection") or {}).get("selected_candidate"),
                    "requested_vx": ((c.get("decision") or {}).get("command") or {}).get("requested_vx"),
                    "safe_vx": ((c.get("decision") or {}).get("command") or {}).get("safe_vx"),
                    "state_vx": ((c.get("decision") or {}).get("command") or {}).get("state_vx"),
                    "lookahead_source": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("display") or {}
                    ).get("path_source"),
                    "pp_follow_source": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("pp") or {}
                    ).get("path_source"),
                    "active_reference": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("authority") or {}
                    ).get("active_reference"),
                    "lookahead_x": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("display") or {}
                    ).get("x"),
                    "lookahead_y": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("display") or {}
                    ).get("y"),
                    "lookahead_inside_obstacle": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("display") or {}
                    ).get("lookahead_inside_obstacle"),
                    "reference_conflict": ((c.get("decision") or {}).get("diagnostics") or {}).get(
                        "reference_conflict"
                    ),
                    "obstacle_pass_state": ((c.get("decision") or {}).get("diagnostics") or {}).get(
                        "obstacle_pass_state"
                    ),
                    "pp_w": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("controller") or {}
                    ).get("pp_w"),
                    "w_cmd": (
                        ((c.get("decision") or {}).get("lookahead_forensics") or {}).get("controller") or {}
                    ).get("w_cmd"),
                }
                for c in cycles[-80:]
            ],
            "event_count": len(events),
            "cycle_count": len(cycles),
        }


OBS = NavObservability()


def _f(*vals: Any) -> Optional[float]:
    for v in vals:
        if v is None:
            continue
        try:
            return float(v)
        except (TypeError, ValueError):
            continue
    return None


def _rf(v: Any) -> Optional[float]:
    x = _f(v)
    return None if x is None else round(x, 3)


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _path_length(pts: Any) -> float:
    if not isinstance(pts, list) or len(pts) < 2:
        return 0.0
    s = 0.0
    prev = None
    for p in pts:
        if isinstance(p, dict):
            x, y = float(p.get("x") or 0), float(p.get("y") or 0)
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            x, y = float(p[0]), float(p[1])
        else:
            continue
        if prev is not None:
            s += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
    return s


def _as_cand_list(obj: Any) -> List[Dict[str, Any]]:
    if isinstance(obj, list):
        out = []
        for x in obj:
            if isinstance(x, dict):
                out.append(x)
        return out
    if isinstance(obj, dict) and obj:
        vals = list(obj.values())
        if vals and all(isinstance(v, dict) for v in vals):
            rows = []
            for k, v in obj.items():
                row = dict(v)
                row.setdefault("candidate_id", k)
                row.setdefault("type", k)
                rows.append(row)
            return rows
    return []


def _extract_candidates(dbg: Dict[str, Any], local: Dict[str, Any], man: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("candidates", "items"):
        found = _as_cand_list(local.get(key))
        if found:
            return found
    rows = local.get("rows")
    if isinstance(rows, list) and rows:
        found = []
        for r in rows:
            if isinstance(r, dict):
                row = dict(r)
                if "action" in row:
                    row.setdefault("candidate_id", row.get("action"))
                    row.setdefault("type", row.get("action"))
                found.append(row)
        if found:
            return found
    cmp_ = local.get("snapshot") if isinstance(local.get("snapshot"), dict) else {}
    if not cmp_:
        cmp_ = man.get("local_compare") if isinstance(man.get("local_compare"), dict) else {}
    found = _as_cand_list(cmp_.get("candidates")) if isinstance(cmp_, dict) else []
    if found:
        return found
    found = _as_cand_list(dbg.get("candidates"))
    if found:
        return found
    lp = dbg.get("local_planner") if isinstance(dbg.get("local_planner"), dict) else {}
    found = _as_cand_list(lp.get("candidates"))
    if found:
        return found
    return []


def _candidate_summary(c: Dict[str, Any], *, detail: bool = False) -> Dict[str, Any]:
    cid = str(c.get("candidate_id") or c.get("id") or c.get("type") or c.get("kind") or c.get("action") or "UNK")
    valid = c.get("valid")
    if valid is None and "feasible" in c:
        valid = bool(c.get("feasible")) and not bool(c.get("collision"))
    elif valid is None:
        valid = not bool(c.get("collision"))
    primary = c.get("invalid_reason") or c.get("reason") or c.get("primary_reason")
    if c.get("collision"):
        primary = primary or "COLLISION"
    reject = list(c.get("reject_reasons") or [])
    if c.get("collision") and "COLLISION" not in reject:
        reject.append("COLLISION")
    if primary and primary not in reject and not valid:
        reject.append(str(primary))
    costs = c.get("score_breakdown") or c.get("cost_breakdown") or c.get("costs") or {}
    if not isinstance(costs, dict):
        costs = {}
    if not costs:
        costs = {
            "progress_cost": c.get("progress_cost"),
            "clearance_cost": c.get("clearance_cost"),
            "heading_cost": c.get("heading_cost"),
            "capture_cost": c.get("capture_cost"),
            "turn_cost": c.get("turn_cost"),
            "collision_cost": c.get("obstacle_cost") or c.get("collision_cost"),
            "recoverability_cost": c.get("recoverability_cost"),
        }
    path = c.get("path") or []
    dist = c.get("distance_m") or c.get("path_length") or c.get("length_m")
    if dist is None and isinstance(path, list) and len(path) >= 2:
        dist = _path_length(path)
    ep = c.get("endpoint") if isinstance(c.get("endpoint"), dict) else None
    if not ep:
        ep = {
            "x": c.get("endpoint_x") or c.get("ex"),
            "y": c.get("endpoint_y") or c.get("ey"),
            "yaw": c.get("endpoint_yaw") or c.get("eyaw") or c.get("endpoint_theta"),
        }
    if ep.get("x") is None and isinstance(path, list) and path:
        last = path[-1]
        if isinstance(last, dict):
            ep = {"x": last.get("x"), "y": last.get("y"), "yaw": last.get("yaw") or last.get("theta")}
        elif isinstance(last, (list, tuple)) and len(last) >= 2:
            ep = {"x": last[0], "y": last[1], "yaw": last[2] if len(last) > 2 else None}
    out = {
        "candidate_id": cid,
        "kind": c.get("kind") or c.get("type") or c.get("action") or cid,
        "valid": bool(valid),
        "invalid_reason": None if valid else (primary or "UNKNOWN"),
        "primary_reason": None if valid else (primary or "UNKNOWN"),
        "reject_reasons": reject if not valid else [],
        "score": c.get("score") if c.get("score") is not None else c.get("total_cost") or c.get("cost"),
        "score_breakdown": {k: v for k, v in costs.items() if v is not None},
        "total_score": c.get("total_cost") or c.get("score") or c.get("cost"),
        "duration_s": c.get("duration") or c.get("duration_s"),
        "distance_m": None if dist is None else round(float(dist), 3),
        "endpoint": ep,
        "progress_m": c.get("progress_m") or c.get("path_progress_gain") or c.get("progress"),
        "min_clearance_m": c.get("min_clearance") or c.get("min_clearance_m") or c.get("path_clearance") or c.get("clr"),
        "collision": bool(c.get("collision")),
        "capture": c.get("capture") if "capture" in c else c.get("path_capture_ok") or c.get("path_capture_available"),
        "heading_error": c.get("heading_error") or c.get("heading_error_deg") or c.get("heading"),
        "requested_vx": c.get("vx") or c.get("requested_vx"),
        "requested_w": c.get("w") or c.get("requested_w"),
    }
    if detail and isinstance(path, list):
        out["path_sample"] = path[:8]
    return out


def _selected_score(cands: List[Dict[str, Any]], selected: Any) -> Any:
    if not selected:
        return None
    su = str(selected).upper()
    for c in cands:
        if su in str(c.get("candidate_id") or "").upper() or su in str(c.get("kind") or "").upper():
            return c.get("total_score") or c.get("score")
    return None


def _selection_failure(
    cand_count: int, valid_count: int, selected: Any, rec_active: bool, policy_action: str
) -> Optional[str]:
    sel_none = selected is None or str(selected).upper() in ("NONE", "", "NULL")
    if not sel_none:
        return None
    if rec_active:
        return "RECOVERY_PRIORITY"
    if "STOP" in str(policy_action).upper():
        return "POLICY_STOP"
    if cand_count <= 0:
        return "NO_CANDIDATE"
    if valid_count <= 0:
        return "ALL_INVALID"
    return "SELECTED_NONE"


def _probe_block(probe: Dict[str, Any]) -> Dict[str, Any]:
    def one(key: str) -> Dict[str, Any]:
        p = probe.get(key) or {}
        if not isinstance(p, dict):
            return {"valid": None}
        return {
            "valid": p.get("status") == "VALID" if "status" in p else p.get("valid"),
            "status": p.get("status"),
            "distance": p.get("distance") or p.get("distance_m") or p.get("length_m") or p.get("horizon_m"),
            "clearance": p.get("min_clearance") or p.get("clearance"),
            "endpoint": p.get("endpoint"),
            "reason": p.get("failure_reason") or p.get("reason"),
        }

    return {
        "forward": one("forward"),
        "backward": one("backward") if "backward" in probe else one("back"),
        "left": one("left"),
        "right": one("right"),
        "selected_evidence": probe.get("selected_evidence") or probe.get("evidence"),
    }


def _stop_reason_enum(decision: Dict[str, Any]) -> str:
    if decision.get("safety", {}).get("blocked"):
        return "SAFETY_CLAMP"
    sel = decision.get("selection") or {}
    if int(sel.get("candidate_count") or 0) <= 0:
        return "NO_CANDIDATE"
    if int(sel.get("valid_candidate_count") or 0) <= 0:
        return "ALL_CANDIDATES_INVALID"
    cmd = decision.get("command") or {}
    if abs(float(cmd.get("requested_vx") or 0)) < VX_EPS:
        return "COMMAND_ZERO"
    return "NO_TRANSLATION_PROGRESS"


def _last_level(events: Deque[Dict[str, Any]], level: str) -> Optional[Dict[str, Any]]:
    for e in reversed(events):
        if str(e.get("level") or "") == level:
            return e
    return None


def _build_decision(
    *,
    x: float,
    y: float,
    yaw: float,
    state_vx: Optional[float],
    state_w: Optional[float],
    front_near: Optional[float],
    rear_near: Optional[float],
    left_near: Optional[float],
    right_near: Optional[float],
    goal_dist: Optional[float],
    gpath: List[Any],
    path_len: float,
    fsm_mode: str,
    geom: Dict[str, Any],
    probe: Dict[str, Any],
    policy: Dict[str, Any],
    pol_dec: Dict[str, Any],
    policy_action: str,
    recovery: Dict[str, Any],
    rec_action: str,
    rec_active: bool,
    req_vx: Optional[float],
    req_w: Optional[float],
    safe_vx: Optional[float],
    safe_w: Optional[float],
    cand_count: int,
    valid_count: int,
    cand_summaries: List[Dict[str, Any]],
    max_cand_dist: float,
    avg_cand_dist: float,
    expected_nom: float,
    candidate_detail: bool,
    selected: Any,
    local: Dict[str, Any],
    man: Dict[str, Any],
    prev_fsm: str,
    vc: Dict[str, Any],
    dbg: Dict[str, Any],
    safety: Dict[str, Any],
    prev_pose: Optional[Tuple[float, float, float]],
    dx: float,
    dy: float,
    dist_prog: float,
    last_yaw: Optional[float],
    yaw_delta: float,
    owner: str,
    immediate_owner: str,
    likely_root_owner: Optional[str],
    cycle_ms: Optional[float],
    phase4: Dict[str, Any],
    gref: Optional[Dict[str, Any]] = None,
    gvl: Optional[Dict[str, Any]] = None,
    loc_layer: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    gref = gref or {}
    gvl = gvl or {}
    loc_layer = loc_layer or {}
    kv = dbg.get("kinematic_validation") if isinstance(dbg.get("kinematic_validation"), dict) else {}
    rec_exec = recovery.get("execution") if isinstance(recovery.get("execution"), dict) else {}
    if not rec_exec:
        rec_exec = recovery.get("progress") if isinstance(recovery.get("progress"), dict) else {}
    corridor = pol_dec.get("corridor") if isinstance(pol_dec.get("corridor"), dict) else (policy.get("corridor") if isinstance(policy.get("corridor"), dict) else {})
    return {
        "observe": {
            "pose": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)},
            "vx": None if state_vx is None else round(state_vx, 3),
            "w": None if state_w is None else round(state_w, 3),
            "front_near": front_near,
            "rear_near": rear_near,
            "left_near": left_near,
            "right_near": right_near,
            "goal_distance": _rf(goal_dist),
            "global_path_exists": bool(gpath),
            "global_path_length": round(path_len, 3),
            "current_mode": fsm_mode,
            "vehicle_geometry": {
                "length_m": geom.get("length_m") or geom.get("length"),
                "width_m": geom.get("width_m") or geom.get("width"),
            }
            if geom
            else None,
        },
        "global": {
            "path_exists": bool(gpath) or bool(gref.get("path_exists")),
            "path_points": len(gpath) if isinstance(gpath, list) else 0,
            "path_length_m": round(float(gref.get("path_length_m") or path_len or 0.0), 3),
            "preview_m": _rf(gref.get("preview_m") or gvl.get("global_preview_m")),
            "preview_status": gref.get("status") or gref.get("geometry_status"),
            "preview_reason": gref.get("preview_reason") or gvl.get("global_preview_reason"),
            "remaining_m": _rf(gref.get("remaining_m") or gvl.get("global_remaining_m")),
            "preview_points": gref.get("preview_point_count"),
            "first_turn_distance_m": _rf(gref.get("first_turn_distance_m")),
            "max_heading_change_deg": _rf(gref.get("heading_change_deg") or gref.get("max_heading_change_deg")),
            "path_revision": gref.get("path_revision") or gvl.get("global_path_revision"),
            "max_curvature": kv.get("max_curvature") if kv.get("max_curvature") is not None else gref.get("max_curvature"),
            "min_turn_radius_m": kv.get("min_turn_radius_m") if kv.get("min_turn_radius_m") is not None else gref.get("min_turn_radius_m"),
            "max_required_w": kv.get("max_required_w_rad_s"),
            "max_feasible_speed_mps": kv.get("max_feasible_speed_mps"),
            "reference_speed_mps": kv.get("reference_speed_mps"),
            "speed_limited": kv.get("speed_limited") if "speed_limited" in kv else gref.get("speed_limited"),
            "min_clearance_m": kv.get("min_clearance_m"),
            "swept_collision": kv.get("swept_collision"),
            "first_invalid_index": kv.get("first_invalid_index"),
            "first_invalid_distance_m": kv.get("first_invalid_distance_m")
            if kv.get("first_invalid_distance_m") is not None
            else gref.get("first_invalid_distance_m"),
            "validation_revision": kv.get("validation_revision"),
            "kinematic_valid": kv.get("kinematic_valid") if "kinematic_valid" in kv else gref.get("kinematic_valid"),
            "kinematic_status": kv.get("kinematic_status") or kv.get("status") or gref.get("kinematic_status") or "NOT_VALIDATED",
            "geometry_status": gref.get("geometry_status") or "REFERENCE_ONLY",
            "controls_vehicle": False,
        },
        "probe": _probe_block(probe),
        "policy": {
            "state": policy.get("state") or policy.get("behavior"),
            "action": policy_action or None,
            "reason": pol_dec.get("reason") if pol_dec else policy.get("reason"),
            "corridor_half_width": corridor.get("half_width") if isinstance(corridor, dict) else None,
            "vx_profile": pol_dec.get("vx_profile") or pol_dec.get("profile") or policy.get("profile"),
            "allow_recovery": recovery.get("allowed") if "allowed" in recovery else recovery.get("allow_recovery"),
            "allow_reverse": recovery.get("allow_reverse"),
            "authority": policy.get("authority")
            or (
                (phase4.get("ownership") or {}).get("authority")
                if isinstance(phase4.get("ownership"), dict)
                else None
            ),
        },
        "recovery": {
            "active": rec_active,
            "action": rec_action or None,
            "reason": recovery.get("reason"),
            "classification": recovery.get("classification"),
            "execution": rec_exec,
            "requested_vx": req_vx,
            "safe_vx": safe_vx,
            "state_vx": state_vx,
            "signed_progress_m": rec_exec.get("signed_progress_m"),
            "elapsed_s": rec_exec.get("elapsed_s"),
            "stall": rec_exec.get("stall"),
            "timeout": rec_exec.get("timeout"),
        },
        "candidates": {
            "count": cand_count,
            "valid_count": valid_count,
            "items": cand_summaries if (candidate_detail or cand_count <= 12) else cand_summaries[:8],
            "max_distance_m": round(max_cand_dist, 3),
            "avg_distance_m": round(avg_cand_dist, 3),
            "expected_nominal_distance_m": round(expected_nom, 3),
            "legacy_nominal_baseline_m": NOMINAL_CAND_DIST_M,
            "local_layer_count": loc_layer.get("count"),
        },
        "selection": {
            "candidate_count": cand_count,
            "valid_candidate_count": valid_count,
            "selected_candidate": selected if selected else "NONE",
            "selected_score": _selected_score(cand_summaries, selected),
            "selection_reason": local.get("reason") or man.get("reason") or man.get("compare_reason"),
            "selection_failure_reason": _selection_failure(
                cand_count, valid_count, selected, rec_active, policy_action
            ),
        },
        "fsm": {
            "previous_mode": prev_fsm or None,
            "current_mode": fsm_mode or None,
            "transition": f"{prev_fsm}->{fsm_mode}" if prev_fsm and prev_fsm != fsm_mode else None,
            "transition_reason": man.get("reason"),
            "force_vx": man.get("force_vx"),
            "force_w": man.get("force_w"),
        },
        "command": {
            "policy_vx": None,
            "policy_w": None,
            "fsm_vx": man.get("force_vx"),
            "fsm_w": man.get("force_w"),
            "controller_vx": _f(vc.get("mppi_vx"), dbg.get("mppi_vx")),
            "controller_w": _f(vc.get("mppi_w"), dbg.get("mppi_w")),
            "requested_vx": req_vx,
            "requested_w": req_w,
            "safe_vx": safe_vx,
            "safe_w": safe_w,
            "final_vx": safe_vx,
            "final_w": safe_w,
            "state_vx": state_vx,
            "state_w": state_w,
        },
        "safety": {
            "action": safety.get("action") or dbg.get("stop_reason"),
            "original_vx": req_vx,
            "original_w": req_w,
            "safe_vx": safe_vx,
            "safe_w": safe_w,
            "blocked": bool(
                req_vx is not None and safe_vx is not None and abs(req_vx) > VX_EPS and abs(safe_vx) < VX_EPS
            ),
            "clamped": bool(
                req_vx is not None and safe_vx is not None and abs(safe_vx) + 0.02 < abs(req_vx)
            ),
            "reason": safety.get("reason") or dbg.get("stop_reason"),
            "front_distance": front_near,
            "rear_distance": rear_near,
            "left_distance": left_near,
            "right_distance": right_near,
            "stop_distance": None,
        },
        "execution": {
            "state_vx": state_vx,
            "state_w": state_w,
            "pose_before": None
            if prev_pose is None
            else {"x": round(prev_pose[0], 3), "y": round(prev_pose[1], 3)},
            "pose_after": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)},
            "dx": round(dx, 4),
            "dy": round(dy, 4),
            "distance_progress": round(dist_prog, 4),
            "yaw_before": None if last_yaw is None else round(last_yaw, 4),
            "yaw_after": round(yaw, 4),
            "yaw_progress": round(yaw_delta, 4),
            "execution_stall": bool(
                (safe_vx is not None and abs(safe_vx) > VX_EPS)
                and (state_vx is not None and abs(state_vx) < VX_EPS)
            ),
        },
        "diagnostics": {
            "stop_reason": dbg.get("stop_reason"),
            "likely_owner": owner,
            "immediate_owner": immediate_owner,
            "likely_root_owner": likely_root_owner,
            "spin_suspected": False,
            "deadlock_suspected": False,
            "planning_starvation_suspected": False,
            "plan_execution_mismatch": False,
            "global_preview_m": _rf(gref.get("preview_m") or gvl.get("global_preview_m")),
            "local_max_distance_m": round(max_cand_dist, 3),
            "expected_nominal_distance_m": round(expected_nom, 3),
            "cycle_semantics": CYCLE_SEMANTICS,
        },
        "performance": {
            "cycle_duration_ms": None if cycle_ms is None else round(float(cycle_ms), 2),
            "cycle_deadline_ms": CYCLE_DEADLINE_MS,
            "cycle_overrun": bool(cycle_ms is not None and float(cycle_ms) > CYCLE_DEADLINE_MS),
            "logging_overhead_ms": None,
        },
    }
