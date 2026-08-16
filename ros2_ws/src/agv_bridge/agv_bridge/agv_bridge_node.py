"""Real AGV bridge: ROS2 services/topics <-> Robokit TCP API."""

# DEPRECATION NOTICE (V0.1仿真版, 2026-08-16)
# ---------------------------------------------------------------------------
# agv_bridge_node 是「真车 Robokit TCP + 站点导航」旧桥。
# 已被取代: agv_control + agv_navigation
# 仅在 ENABLE_LEGACY_BRIDGE=true 时允许启动。
# ---------------------------------------------------------------------------



from __future__ import annotations

import uuid

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from delivery_interfaces.msg import AgvPose, AgvStatus
from delivery_interfaces.srv import AgvCancel, AgvLock, AgvNavigate

from agv_bridge.robokit_client import RobokitClient, RobokitError


class AgvBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_bridge_node")
        self.declare_parameter("agv_host", "192.168.18.198")
        self.declare_parameter("nick_name", "ros2_delivery")
        self.declare_parameter("protocol_version", 1)
        self.declare_parameter("timeout_sec", 3.0)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("auto_lock", True)

        host = self.get_parameter("agv_host").get_parameter_value().string_value
        timeout = self.get_parameter("timeout_sec").get_parameter_value().double_value
        ver = self.get_parameter("protocol_version").get_parameter_value().integer_value
        self._client = RobokitClient(host=host, timeout=timeout, protocol_version=ver)
        self._cg = ReentrantCallbackGroup()
        self._locked = False

        self._pose_pub = self.create_publisher(AgvPose, "agv/pose", 10)
        self._status_pub = self.create_publisher(AgvStatus, "agv/status", 10)
        self.create_service(AgvNavigate, "agv/navigate", self._on_navigate, callback_group=self._cg)
        self.create_service(AgvCancel, "agv/cancel", self._on_cancel, callback_group=self._cg)
        self.create_service(AgvLock, "agv/lock", self._on_lock, callback_group=self._cg)

        hz = self.get_parameter("status_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 0.5), self._poll_status)
        self.get_logger().info(f"AGV bridge -> {host} (real Robokit TCP)")

        if self.get_parameter("auto_lock").get_parameter_value().bool_value:
            try:
                nick = self.get_parameter("nick_name").get_parameter_value().string_value
                self._client.lock(nick)
                self._locked = True
                self.get_logger().info(f"control locked as {nick}")
            except RobokitError as exc:
                self.get_logger().warn(f"auto lock failed: {exc}")

    def destroy_node(self) -> bool:
        try:
            if self._locked:
                self._client.unlock()
        except Exception:  # noqa: BLE001
            pass
        self._client.close()
        return super().destroy_node()

    def _on_lock(self, req: AgvLock.Request, res: AgvLock.Response) -> AgvLock.Response:
        try:
            if req.lock:
                nick = req.nick_name or self.get_parameter("nick_name").get_parameter_value().string_value
                self._client.lock(nick)
                self._locked = True
                res.success = True
                res.message = f"locked by {nick}"
            else:
                self._client.unlock()
                self._locked = False
                res.success = True
                res.message = "unlocked"
        except RobokitError as exc:
            res.success = False
            res.message = str(exc)
        return res

    def _on_cancel(self, _req: AgvCancel.Request, res: AgvCancel.Response) -> AgvCancel.Response:
        try:
            self._client.cancel_nav()
            res.success = True
            res.message = "canceled"
        except RobokitError as exc:
            res.success = False
            res.message = str(exc)
        return res

    def _on_navigate(self, req: AgvNavigate.Request, res: AgvNavigate.Response) -> AgvNavigate.Response:
        task_id = req.task_id.strip() or str(uuid.uuid4())[:8]
        source = req.source_id.strip() or "SELF_POSITION"
        try:
            self._client.goto_station(
                target_id=req.target_id,
                source_id=source,
                task_id=task_id,
                angle=req.angle if abs(req.angle) > 1e-9 else None,
                method=req.method or None,
                max_speed=req.max_speed if req.max_speed > 0 else None,
            )
            if req.wait_until_done:
                timeout = req.timeout_sec if req.timeout_sec > 0 else 180.0
                ok, last = self._client.wait_nav_done(timeout_sec=timeout)
                res.success = ok
                res.final_status = int(last.get("task_status", 5 if not ok else 4))
                res.message = "arrived" if ok else last.get("err_msg", "nav failed/timeout")
                res.task_id = task_id
            else:
                res.success = True
                res.final_status = 2
                res.message = "accepted"
                res.task_id = task_id
        except RobokitError as exc:
            res.success = False
            res.final_status = 5
            res.message = str(exc)
            res.task_id = task_id
        return res

    def _poll_status(self) -> None:
        try:
            loc = self._client.get_pose()
            task = self._client.get_task_status(simple=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"status poll failed: {exc}", throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()
        pose = AgvPose()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.x = float(loc.get("x", 0.0))
        pose.y = float(loc.get("y", 0.0))
        pose.angle = float(loc.get("angle", 0.0))
        pose.confidence = float(loc.get("confidence", 0.0))
        pose.current_station = str(loc.get("current_station", "") or "")
        pose.last_station = str(loc.get("last_station", "") or "")
        self._pose_pub.publish(pose)

        st = AgvStatus()
        st.header.stamp = now
        st.header.frame_id = "map"
        st.task_status = int(task.get("task_status", 0))
        st.task_type = int(task.get("task_type", 0))
        st.target_id = str(task.get("target_id", "") or "")
        st.finished_path = [str(x) for x in task.get("finished_path", []) or []]
        st.unfinished_path = [str(x) for x in task.get("unfinished_path", []) or []]
        st.x = pose.x
        st.y = pose.y
        st.angle = pose.angle
        st.confidence = pose.confidence
        st.current_station = pose.current_station
        st.mode = "real"
        self._status_pub.publish(st)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgvBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
