"""xArm7 pick_place_node ROS2 bridge — local DDS on ROS_DOMAIN_ID=30.

Release (NUC): delivery_gazebo_soft + Zack xarm7_real share host network — same machine.
Alpha (VM): cross-machine DDS to NUC when stacks run on different hosts.

Subscribes /joint_states + /pick_place_state, calls services with timeouts.
Does NOT modify pick_place_node.py in Zack's stack.
"""

from __future__ import annotations

import math
import re
import threading
import time
from typing import Any, Dict, List, Optional

# Studio block sequence (pick-place cycle)
PICK_PLACE_BLOCKS = [1, 2, 3, 4, 6, 7, 8, 9, 11]

_BLOCK_RE = re.compile(r"\[Studio B(\d+)\]", re.IGNORECASE)


def parse_pick_place_state(msg: str) -> Dict[str, Any]:
    """Parse plain-text /pick_place_state into structured fields."""
    text = (msg or "").strip()
    out: Dict[str, Any] = {
        "raw": text,
        "block": None,
        "action": "",
        "target": "",
        "gripper_hint": "unknown",
    }
    if not text:
        return out
    m = _BLOCK_RE.search(text)
    if m:
        out["block"] = int(m.group(1))
    if "->" in text:
        out["action"] = "move"
        parts = text.split("->", 1)
        if len(parts) > 1:
            out["target"] = parts[1].strip().split()[0]
    elif "OPEN" in text.upper():
        out["action"] = "gripper_open"
        out["gripper_hint"] = "open"
    elif "CLOSE" in text.upper():
        out["action"] = "gripper_close"
        out["gripper_hint"] = "closed"
    elif "CYCLE" in text.upper() and "done" in text.lower():
        out["action"] = "cycle_complete"
        out["block"] = 99
    elif "UNLOCK" in text.upper():
        out["action"] = "unlock_complete"
        out["block"] = -1
    elif text.startswith("Done "):
        out["action"] = "service_done"
    return out


_GAZEBO_JOINT_MARKERS = ("left_j", "right_j", "gazebo")


class ArmBridge:
    """ROS2 bridge to Zack pick_place_node via local DDS (domain 30)."""

    HEARTBEAT_SEC = 5.0
    NODE_NAME = "/pick_place_server"

    def __init__(self, node: Any, robot_ip: str = "172.31.0.123") -> None:
        self._node = node
        self._robot_ip = robot_ip
        self._lock = threading.Lock()
        self._current_state_msg = ""
        self._state_history: List[Dict[str, Any]] = []
        self._joint_angles_rad: List[float] = [0.0] * 7
        self._joint_angles_deg: List[float] = [0.0] * 7
        self._joint_names: List[str] = [f"joint{i}" for i in range(1, 8)]
        self._is_busy = False
        self._last_result: Optional[Dict[str, Any]] = None
        self._last_heartbeat = 0.0
        self._arm_online = False
        self._gripper_state = "unknown"
        self._cycles = 1
        self._unlock_done = False
        self._svc_lock = threading.Lock()

        self._pick_place_seen = False
        self._unlock_client = None
        self._pickplace_client = None
        self._gripper_client = None
        self._param_client = None

        try:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            from std_srvs.srv import SetBool, Trigger

            sub_cg = MutuallyExclusiveCallbackGroup()
            cli_cg = ReentrantCallbackGroup()
            node.create_subscription(String, "/pick_place_state", self._on_state, 10, callback_group=sub_cg)
            node.create_subscription(JointState, "/joint_states", self._on_joints, 10, callback_group=sub_cg)
            self._unlock_client = node.create_client(Trigger, "/unlock_and_home", callback_group=cli_cg)
            self._pickplace_client = node.create_client(Trigger, "/do_pick_place", callback_group=cli_cg)
            self._gripper_client = node.create_client(SetBool, "/set_gripper", callback_group=cli_cg)
            try:
                from rclpy.parameter_client import SyncParametersClient

                self._param_client = SyncParametersClient(node, self.NODE_NAME)
            except Exception:  # noqa: BLE001
                self._param_client = None
        except Exception as exc:  # noqa: BLE001
            if hasattr(node, "get_logger"):
                node.get_logger().warn(f"ArmBridge init partial: {exc}")

        try:
            node.create_timer(1.0, self._check_heartbeat)
        except Exception:  # noqa: BLE001
            pass

    @staticmethod
    def _looks_like_gazebo_joints(names: List[str]) -> bool:
        lowered = [str(n or "").lower() for n in names]
        if not lowered:
            return False
        return any("left_j" in n or "right_j" in n or "gazebo" in n for n in lowered)

    def _on_state(self, msg) -> None:
        text = str(getattr(msg, "data", "") or "")
        parsed = parse_pick_place_state(text)
        with self._lock:
            self._pick_place_seen = True
            self._current_state_msg = text
            self._state_history.append({"time": time.time(), "msg": text, "parsed": parsed})
            if len(self._state_history) > 50:
                self._state_history.pop(0)
            if parsed.get("gripper_hint") in ("open", "closed"):
                self._gripper_state = parsed["gripper_hint"]
            if "[Studio B" in text or "BUSY" in text.upper():
                self._is_busy = True
            if parsed.get("action") in ("cycle_complete", "service_done"):
                self._is_busy = False
            self._arm_online = True
            self._last_heartbeat = time.time()

    def _on_joints(self, msg) -> None:
        names = list(getattr(msg, "name", None) or [])
        if self._looks_like_gazebo_joints(names):
            return
        with self._lock:
            if not self._pick_place_seen:
                return
            if getattr(msg, "position", None):
                pos = list(msg.position[:7])
                while len(pos) < 7:
                    pos.append(0.0)
                self._joint_angles_rad = pos
                self._joint_angles_deg = [float(p) * 180.0 / math.pi for p in pos]
            if getattr(msg, "name", None):
                self._joint_names = list(msg.name)[:7]
            self._last_heartbeat = time.time()

    def _check_heartbeat(self) -> None:
        svc_online = False
        try:
            if self._unlock_client is not None:
                svc_online = self._unlock_client.service_is_ready()
        except Exception:  # noqa: BLE001
            svc_online = False
        with self._lock:
            fresh = self._last_heartbeat > 0 and (time.time() - self._last_heartbeat) < self.HEARTBEAT_SEC
            if self._pick_place_seen and fresh:
                self._arm_online = True
            elif svc_online:
                self._arm_online = True
            else:
                self._arm_online = False

    def _wait_future(self, future, timeout_sec: float) -> bool:
        # Never spin_until_future_complete on a node the MultiThreadedExecutor
        # already owns — that deadlocks arm services and shows up as HTTP timeout.
        deadline = time.time() + max(0.1, float(timeout_sec))
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.done()

    def _call_trigger(self, client, name: str, timeout_sec: float) -> Dict[str, Any]:
        if client is None:
            return {"success": False, "message": f"client {name} not initialized"}
        if not client.wait_for_service(timeout_sec=3.0):
            return {"success": False, "message": f"Service {name} not available (xArm stack offline?)"}
        from std_srvs.srv import Trigger

        req = Trigger.Request()
        future = client.call_async(req)
        if not self._wait_future(future, timeout_sec):
            return {"success": False, "message": f"Timeout ({timeout_sec}s)"}
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "message": str(exc)}
        if result is None:
            return {"success": False, "message": "No response"}
        out = {"success": bool(result.success), "message": str(result.message or "")}
        with self._lock:
            self._last_result = {**out, "timestamp": time.time()}
        return out

    def set_cycles(self, cycles: int) -> Dict[str, Any]:
        cycles = max(1, min(100, int(cycles)))
        self._cycles = cycles
        if self._param_client is None:
            return {"success": True, "cycles": cycles, "message": "cached locally (param client unavailable)"}
        try:
            from rclpy.parameter import Parameter

            ok = self._param_client.set_parameters([Parameter("cycles", Parameter.Type.INTEGER, cycles)])
            if ok:
                return {"success": True, "cycles": cycles}
            return {"success": False, "message": "param set failed"}
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "message": str(exc)}

    def call_unlock(self, timeout: float = 30.0) -> Dict[str, Any]:
        with self._svc_lock:
            if not self.get_status().get("online"):
                return {"success": False, "message": "Arm offline"}
            self._is_busy = True
            try:
                out = self._call_trigger(self._unlock_client, "/unlock_and_home", timeout)
            finally:
                self._is_busy = False
        if out.get("success"):
            self._unlock_done = True
        return out

    def call_pick_place(self, cycles: Optional[int] = None, timeout: float = 120.0) -> Dict[str, Any]:
        if cycles is not None:
            self.set_cycles(cycles)
        with self._svc_lock:
            st = self.get_status()
            if not st.get("online"):
                return {"success": False, "message": "Arm offline", "duration_s": 0.0}
            if st.get("is_busy"):
                return {"success": False, "message": "Arm busy", "duration_s": 0.0}
            self._is_busy = True
            t0 = time.time()
            try:
                out = self._call_trigger(self._pickplace_client, "/do_pick_place", timeout)
            finally:
                self._is_busy = False
            out["duration_s"] = round(time.time() - t0, 1)
            if out.get("success"):
                out["warning"] = "⚠️ 夹爪失败不报错，请确认物体已抓取"
            return out

    def call_gripper(self, open_command: bool, timeout: float = 12.0) -> Dict[str, Any]:
        if self._gripper_client is None:
            return {"success": False, "message": "gripper client not initialized"}
        if not self._gripper_client.wait_for_service(timeout_sec=3.0):
            return {"success": False, "message": "Service /set_gripper not available"}
        from std_srvs.srv import SetBool

        with self._svc_lock:
            req = SetBool.Request()
            req.data = bool(open_command)
            future = self._gripper_client.call_async(req)
            if not self._wait_future(future, timeout):
                return {"success": False, "message": f"Timeout ({int(timeout)}s)"}
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001
                return {"success": False, "message": str(exc)}
            if result is None:
                return {"success": False, "message": "No response"}
            if result.success:
                self._gripper_state = "open" if open_command else "closed"
            return {"success": bool(result.success), "message": str(result.message or "")}

    def call_stop(self) -> Dict[str, Any]:
        return {
            "success": False,
            "message": "Soft stop not implemented — use physical E-stop or wait for cycle end",
        }

    def get_status(self) -> Dict[str, Any]:
        svc_online = False
        try:
            if self._unlock_client is not None:
                svc_online = bool(self._unlock_client.service_is_ready())
        except Exception:  # noqa: BLE001
            svc_online = False
        with self._lock:
            online = self._arm_online or svc_online
            msg = self._current_state_msg
            parsed = parse_pick_place_state(msg)
            block = parsed.get("block")
            state = "idle"
            if not online:
                state = "offline"
            elif self._is_busy:
                state = "running"
            elif "error" in msg.lower():
                state = "error"

            progress = 0
            if block and block > 0 and block < 99:
                done = sum(1 for b in PICK_PLACE_BLOCKS if b < block)
                progress = int(done / max(len(PICK_PLACE_BLOCKS), 1) * 100)
            elif block == 99:
                progress = 100

            return {
                "online": online,
                "state": state,
                "state_text": msg,
                "current_block": block,
                "current_action": parsed.get("action", ""),
                "current_target": parsed.get("target", ""),
                "gripper_state": self._gripper_state,
                "is_busy": self._is_busy,
                "joints_deg": list(self._joint_angles_deg),
                "joints_rad": list(self._joint_angles_rad),
                "joint_names": list(self._joint_names),
                "last_result": self._last_result,
                "state_history": list(self._state_history[-10:]),
                "progress_percent": progress,
                "cycles": self._cycles,
                "unlock_done": self._unlock_done,
                "ros_online": online,
                "nuc_reachable": online,
                "robot_ip": self._robot_ip,
                "ros_local": True,
                "source": "arm_bridge",
            }

    def get_pose(self) -> Dict[str, Any]:
        st = self.get_status()
        return {
            "online": st["online"],
            "joints_deg": st["joints_deg"],
            "joints_rad": st["joints_rad"],
            "joint_names": st["joint_names"],
        }
