"""CLI demo: navigate AGV A->B via ROS2 service (sim or real)."""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from delivery_interfaces.srv import AgvNavigate


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("agv_demo_nav")
    target = "LM2"
    source = "SELF_POSITION"
    if len(sys.argv) >= 2:
        target = sys.argv[1]
    if len(sys.argv) >= 3:
        source = sys.argv[2]

    client = node.create_client(AgvNavigate, "agv/navigate")
    node.get_logger().info(f"waiting for agv/navigate ... target={target}")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("service not available")
        rclpy.shutdown()
        sys.exit(1)

    req = AgvNavigate.Request()
    req.target_id = target
    req.source_id = source
    req.wait_until_done = True
    req.timeout_sec = 120.0
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=130.0)
    if not future.done():
        node.get_logger().error("timeout waiting response")
        sys.exit(2)
    res = future.result()
    node.get_logger().info(
        f"success={res.success} status={res.final_status} msg={res.message} task={res.task_id}"
    )
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if res.success else 3)


if __name__ == "__main__":
    main()
