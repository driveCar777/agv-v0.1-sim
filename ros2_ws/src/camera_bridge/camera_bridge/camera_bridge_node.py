"""Real industrial camera bridge skeleton (GigE / vendor SDK later).

For M1.3 acceptance, Jason may use Keyence/SICK vendor tools; this node
provides the ROS2 interface contract and a GigE/Aravis hook point.
"""

from __future__ import annotations

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from delivery_interfaces.msg import CameraInfoLite, QrDetectionArray
from delivery_interfaces.srv import CameraCapture, DetectQr


class CameraBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_bridge_node")
        self.declare_parameter("camera_names", ["cam_front", "cam_left", "cam_right"])
        self.declare_parameter("camera_ids", ["", "", ""])
        self.declare_parameter("backend", "stub")  # stub | aravis | vendor
        self.declare_parameter("publish_hz", 5.0)

        names = self.get_parameter("camera_names").get_parameter_value().string_array_value
        if not names:
            names = ["cam_front", "cam_left", "cam_right"]
        self._names = list(names)
        self._backend = self.get_parameter("backend").get_parameter_value().string_value
        self._cg = ReentrantCallbackGroup()

        self._info_pubs = {
            n: self.create_publisher(CameraInfoLite, f"camera/{n}/info", 10) for n in self._names
        }
        self._qr_pubs = {
            n: self.create_publisher(QrDetectionArray, f"camera/{n}/qr", 10) for n in self._names
        }
        self.create_service(CameraCapture, "camera/capture", self._on_capture, callback_group=self._cg)
        self.create_service(DetectQr, "camera/detect_qr", self._on_detect, callback_group=self._cg)

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 0.5), self._tick)
        self.get_logger().warn(
            f"camera_bridge backend={self._backend} — real GigE capture not wired yet; "
            "use camera_sim_node for interface acceptance, or set backend after SDK install."
        )

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        for name in self._names:
            info = CameraInfoLite()
            info.header.stamp = now
            info.header.frame_id = name
            info.camera_name = name
            info.connected = False
            info.transport = self._backend
            self._info_pubs[name].publish(info)
            arr = QrDetectionArray()
            arr.header.stamp = now
            arr.camera_name = name
            self._qr_pubs[name].publish(arr)

    def _on_capture(self, req: CameraCapture.Request, res: CameraCapture.Response) -> CameraCapture.Response:
        res.success = False
        res.message = f"backend={self._backend} not implemented for capture yet"
        return res

    def _on_detect(self, req: DetectQr.Request, res: DetectQr.Response) -> DetectQr.Response:
        res.success = False
        res.message = f"backend={self._backend} not implemented for detect yet"
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
