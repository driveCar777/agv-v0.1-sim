"""MissionManager — Nav2 NavigateToPose + 离线业务推进。

完全离线：Nav2 / 视觉 / 机械臂不可用时按 auto_offline_skip 推进状态机。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from agv_mission.state_machine import Mission, MissionPhase, safe_transition


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("agv_mission_manager")
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("phase_topic", "/agv/mission/phase")
        self.declare_parameter("auto_offline_skip", True)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("pickup_x", 2.0)
        self.declare_parameter("pickup_y", 1.5)
        self.declare_parameter("user_x", -2.0)
        self.declare_parameter("user_y", 3.0)
        self.declare_parameter("start_delay_s", 2.0)

        self._mission = Mission()
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("nav_action").get_parameter_value().string_value,
        )
        self._phase_pub = self.create_publisher(
            String,
            self.get_parameter("phase_topic").get_parameter_value().string_value,
            10,
        )
        self.create_timer(0.5, self._publish_phase)
        self.get_logger().info("mission manager ready")

        self._auto_started = False
        if self.get_parameter("auto_start").get_parameter_value().bool_value:
            delay = self.get_parameter("start_delay_s").get_parameter_value().double_value
            self._auto_timer = self.create_timer(max(delay, 0.1), self._auto_start_once)

    def _auto_start_once(self) -> None:
        if self._auto_started:
            return
        self._auto_started = True
        if hasattr(self, "_auto_timer"):
            self._auto_timer.cancel()
        if self._mission.phase != MissionPhase.IDLE:
            return
        pickup = (
            self.get_parameter("pickup_x").get_parameter_value().double_value,
            self.get_parameter("pickup_y").get_parameter_value().double_value,
            0.0,
        )
        user = (
            self.get_parameter("user_x").get_parameter_value().double_value,
            self.get_parameter("user_y").get_parameter_value().double_value,
            3.14,
        )
        self.start_mission(pickup, user, mission_id="auto_sim")

    def start_mission(self, pickup_xy, user_xy, mission_id: str = "local") -> None:
        self._mission = Mission(
            mission_id=mission_id,
            pickup_pose=tuple(pickup_xy),
            user_pose=tuple(user_xy),
        )
        self._mission.transition(MissionPhase.RECEIVED)
        self.get_logger().info(
            f"mission started: id={mission_id} pickup={pickup_xy} user={user_xy}"
        )
        self._next_step()

    def _next_step(self) -> None:
        phase = self._mission.phase
        offline = self.get_parameter("auto_offline_skip").get_parameter_value().bool_value

        if phase == MissionPhase.RECEIVED:
            self._mission.transition(MissionPhase.GO_TO_PICKUP)
            self._send_nav_goal(self._mission.pickup_pose, on_done=self._on_arrived_pickup)
            return

        if phase == MissionPhase.ARRIVED_PICKUP:
            safe_transition(self._mission, MissionPhase.IDENTIFY_OBJECT, "arrived_pickup")
            self._next_step()
            return

        if phase == MissionPhase.IDENTIFY_OBJECT:
            safe_transition(self._mission, MissionPhase.PICK, "vision_ok" if offline else "vision")
            self._next_step()
            return

        if phase == MissionPhase.PICK:
            safe_transition(self._mission, MissionPhase.VERIFY_PICK, "arm_pick_ok")
            self._next_step()
            return

        if phase == MissionPhase.VERIFY_PICK:
            safe_transition(self._mission, MissionPhase.GO_TO_USER, "pick_verified")
            self._send_nav_goal(self._mission.user_pose, on_done=self._on_arrived_user)
            return

        if phase == MissionPhase.IDENTIFY_USER:
            safe_transition(self._mission, MissionPhase.DELIVER, "user_ok")
            self._next_step()
            return

        if phase == MissionPhase.DELIVER:
            safe_transition(self._mission, MissionPhase.VERIFY_DELIVER, "arm_deliver_ok")
            self._next_step()
            return

        if phase == MissionPhase.VERIFY_DELIVER:
            safe_transition(self._mission, MissionPhase.FINISHED, "done")
            self.get_logger().info("mission FINISHED")
            return

    def _send_nav_goal(self, pose, on_done) -> None:
        offline = self.get_parameter("auto_offline_skip").get_parameter_value().bool_value
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("nav2 action server unavailable (offline sim OK)")
            if offline:
                on_done(success=True)
            else:
                safe_transition(self._mission, MissionPhase.NAVIGATION_ERROR, "nav_unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(pose[0])
        goal.pose.pose.position.y = float(pose[1])
        if len(pose) > 2:
            goal.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
            goal.pose.pose.orientation.w = math.cos(pose[2] / 2.0)

        future = self._nav_client.send_goal_async(goal)

        def _after_accept(f):
            try:
                handle = f.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"nav goal rejected: {exc}")
                on_done(success=False)
                return
            if handle is None or not handle.accepted:
                on_done(success=False)
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda _rf: on_done(success=True))

        future.add_done_callback(_after_accept)

    def _on_arrived_pickup(self, success: bool) -> None:
        if not success:
            safe_transition(self._mission, MissionPhase.NAVIGATION_ERROR, "pickup_nav_fail")
            return
        safe_transition(self._mission, MissionPhase.ARRIVED_PICKUP, "pickup_arrived")
        self._next_step()

    def _on_arrived_user(self, success: bool) -> None:
        if not success:
            safe_transition(self._mission, MissionPhase.NAVIGATION_ERROR, "user_nav_fail")
            return
        # GO_TO_USER → IDENTIFY_USER
        safe_transition(self._mission, MissionPhase.IDENTIFY_USER, "user_arrived")
        self._next_step()

    def _publish_phase(self) -> None:
        self._phase_pub.publish(String(data=self._mission.phase.value))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
