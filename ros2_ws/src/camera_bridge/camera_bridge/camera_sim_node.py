"""Industrial camera simulation — 3 cameras + synthetic QR/AprilTag detections."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image

from delivery_interfaces.msg import CameraInfoLite, QrDetection, QrDetectionArray
from delivery_interfaces.srv import CameraCapture, DetectQr


class CameraSimNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_sim_node")
        self.declare_parameter("camera_names", ["cam_front", "cam_left", "cam_right"])
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("inject_qr", True)
        self.declare_parameter("qr_payload", "STATION_LM2")
        self.declare_parameter("code_type", "AprilTag")

        names = self.get_parameter("camera_names").get_parameter_value().string_array_value
        if not names:
            names = ["cam_front", "cam_left", "cam_right"]
        self._names = list(names)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        self._cg = ReentrantCallbackGroup()
        self._t0 = time.time()

        self._info_pubs = {
            n: self.create_publisher(CameraInfoLite, f"camera/{n}/info", 10) for n in self._names
        }
        self._img_pubs = {
            n: self.create_publisher(Image, f"camera/{n}/image_raw", 10) for n in self._names
        }
        self._qr_pubs = {
            n: self.create_publisher(QrDetectionArray, f"camera/{n}/qr", 10) for n in self._names
        }
        self.create_service(CameraCapture, "camera/capture", self._on_capture, callback_group=self._cg)
        self.create_service(DetectQr, "camera/detect_qr", self._on_detect, callback_group=self._cg)

        fps = float(self.get_parameter("fps").value)
        self.create_timer(1.0 / max(fps, 1.0), self._tick)
        self.get_logger().info(f"camera sim ready: {self._names}")

    def _synthetic_gray(self, cam_idx: int) -> bytes:
        # lightweight patterned frame (no OpenCV required)
        t = time.time() - self._t0
        row = bytearray()
        # generate one scanline pattern then repeat — keep CPU light
        for x in range(self._w):
            v = int((math.sin(x * 0.05 + t + cam_idx) * 0.5 + 0.5) * 200) + 20
            row.append(v % 256)
        return bytes(row) * self._h

    def _make_detection(self, cam: str) -> QrDetection:
        t = time.time() - self._t0
        d = QrDetection()
        d.header.stamp = self.get_clock().now().to_msg()
        d.header.frame_id = cam
        d.camera_name = cam
        d.code_type = self.get_parameter("code_type").get_parameter_value().string_value
        d.payload = self.get_parameter("qr_payload").get_parameter_value().string_value
        d.x = 0.35 + 0.02 * math.sin(t)
        d.y = 0.0
        d.z = 0.85
        d.yaw = 0.05 * math.sin(t * 0.5)
        d.pitch = 0.0
        d.roll = 0.0
        d.confidence = 0.92
        cx, cy = self._w / 2.0, self._h / 2.0
        s = 40.0
        d.corners_2d = [
            cx - s, cy - s,
            cx + s, cy - s,
            cx + s, cy + s,
            cx - s, cy + s,
        ]
        return d

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        inject = self.get_parameter("inject_qr").get_parameter_value().bool_value
        for i, name in enumerate(self._names):
            info = CameraInfoLite()
            info.header.stamp = now
            info.header.frame_id = name
            info.camera_name = name
            info.width = self._w
            info.height = self._h
            info.fps = float(self.get_parameter("fps").value)
            info.connected = True
            info.transport = "sim"
            self._info_pubs[name].publish(info)

            img = Image()
            img.header.stamp = now
            img.header.frame_id = name
            img.height = self._h
            img.width = self._w
            img.encoding = "mono8"
            img.step = self._w
            img.data = self._synthetic_gray(i)
            self._img_pubs[name].publish(img)

            arr = QrDetectionArray()
            arr.header.stamp = now
            arr.header.frame_id = name
            arr.camera_name = name
            if inject and name == self._names[0]:
                arr.detections = [self._make_detection(name)]
            self._qr_pubs[name].publish(arr)

    def _on_capture(self, req: CameraCapture.Request, res: CameraCapture.Response) -> CameraCapture.Response:
        name = req.camera_name or self._names[0]
        if name not in self._names:
            res.success = False
            res.message = f"unknown camera: {name}"
            return res
        res.success = True
        res.message = "ok"
        res.encoding = "mono8"
        res.width = self._w
        res.height = self._h
        res.data = self._synthetic_gray(self._names.index(name))
        return res

    def _on_detect(self, req: DetectQr.Request, res: DetectQr.Response) -> DetectQr.Response:
        name = req.camera_name or self._names[0]
        if name not in self._names:
            res.success = False
            res.message = f"unknown camera: {name}"
            return res
        det = self._make_detection(name)
        if req.code_type:
            det.code_type = req.code_type
        res.success = True
        res.message = "ok"
        res.detections = [det]
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
