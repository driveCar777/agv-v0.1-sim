"""Pull Xavier NX face-camera MJPEG and publish as ROS cam_front.

Xavier face project (python+opencv) serves:
  http://<FACE_JETSON_IP>:8080/stream   multipart MJPEG
  http://<FACE_JETSON_IP>:8080/         HTML viewer

Dev wiring: cam_front (相机2) comes from Xavier, not gz_physics_proxy.
"""

from __future__ import annotations

import os
import threading
import time
from io import BytesIO
from typing import Optional
from urllib.error import URLError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from delivery_interfaces.msg import CameraInfoLite


def _decode_jpeg_to_rgb(blob: bytes) -> Optional[tuple[int, int, bytes]]:
    try:
        from PIL import Image as PILImage  # type: ignore

        im = PILImage.open(BytesIO(blob)).convert("RGB")
        w, h = im.size
        return w, h, im.tobytes()
    except Exception:  # noqa: BLE001
        pass
    try:
        import numpy as np  # type: ignore
        import cv2  # type: ignore

        arr = np.frombuffer(blob, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        h, w = bgr.shape[:2]
        rgb = bgr[:, :, ::-1].tobytes()
        return w, h, rgb
    except Exception:  # noqa: BLE001
        return None


class XavierCamFrontBridge(Node):
    def __init__(self) -> None:
        super().__init__("xavier_cam_front_bridge")
        default_ip = os.environ.get("FACE_JETSON_IP", "192.168.0.225")
        default_url = os.environ.get(
            "XAVIER_CAM_STREAM_URL",
            f"http://{default_ip}:8080/stream",
        )
        self.declare_parameter("stream_url", default_url)
        self.declare_parameter("camera_name", "cam_front")
        self.declare_parameter("frame_id", "cam_front_link")
        self.declare_parameter("reconnect_sec", 2.0)
        self.declare_parameter("read_timeout_sec", 8.0)

        self._url = str(self.get_parameter("stream_url").value).strip()
        self._name = str(self.get_parameter("camera_name").value).strip() or "cam_front"
        self._frame_id = str(self.get_parameter("frame_id").value).strip() or f"{self._name}_link"
        self._reconnect = float(self.get_parameter("reconnect_sec").value)
        self._timeout = float(self.get_parameter("read_timeout_sec").value)

        self._img_pub = self.create_publisher(Image, f"/camera/{self._name}/image_raw", 5)
        self._info_pub = self.create_publisher(CameraInfoLite, f"camera/{self._name}/info", 10)

        self._lock = threading.Lock()
        self._last_wh = (0, 0)
        self._last_frame_wall = 0.0
        self._frames = 0
        self._connected = False
        self._last_error = ""
        self._stop = False

        self._thread = threading.Thread(target=self._pull_loop, name="xavier-mjpeg", daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._status_tick)
        self.get_logger().warn(
            f"xavier_cam_front_bridge: pulling {self._url} -> /camera/{self._name}/image_raw"
        )

    def destroy_node(self) -> bool:
        self._stop = True
        return super().destroy_node()

    def _status_tick(self) -> None:
        with self._lock:
            connected = self._connected and (time.time() - self._last_frame_wall) < 3.0
            w, h = self._last_wh
            err = self._last_error
            n = self._frames
        info = CameraInfoLite()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self._frame_id
        info.camera_name = self._name
        info.connected = bool(connected)
        info.width = int(w)
        info.height = int(h)
        info.fps = 10.0 if connected else 0.0
        info.transport = "xavier_mjpeg" if connected else "xavier_waiting"
        self._info_pub.publish(info)
        if not connected:
            self.get_logger().warn(
                f"cam_front Xavier not ready url={self._url} err={err or 'no-frame'} frames={n}"
            )
        elif n % 30 == 0:
            self.get_logger().info(f"cam_front Xavier ok {w}x{h} frames={n}")

    def _publish_rgb(self, w: int, h: int, rgb: bytes) -> None:
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self._frame_id
        img.height = h
        img.width = w
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = w * 3
        img.data = rgb
        self._img_pub.publish(img)
        with self._lock:
            self._last_wh = (w, h)
            self._last_frame_wall = time.time()
            self._frames += 1
            self._connected = True
            self._last_error = ""

    def _pull_loop(self) -> None:
        while not self._stop and rclpy.ok():
            try:
                self._consume_stream()
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._connected = False
                    self._last_error = str(exc)
                self.get_logger().error(f"Xavier stream error: {exc}")
            if self._stop:
                break
            time.sleep(max(0.5, self._reconnect))

    def _consume_stream(self) -> None:
        req = Request(self._url, headers={"User-Agent": "xavier_cam_front_bridge/0.2"})
        self.get_logger().info(f"connecting Xavier MJPEG {self._url}")
        with urlopen(req, timeout=self._timeout) as resp:
            ctype = (resp.headers.get("Content-Type") or "").lower()
            boundary = b""
            if "boundary=" in ctype:
                boundary = ctype.split("boundary=", 1)[1].strip().encode("ascii", "ignore")
                if boundary and not boundary.startswith(b"--"):
                    boundary = b"--" + boundary
            if not boundary:
                boundary = b"--frame"
            buf = b""
            while not self._stop and rclpy.ok():
                chunk = resp.read(4096)
                if not chunk:
                    raise RuntimeError("stream EOF")
                buf += chunk
                while True:
                    data, buf = self._pop_jpeg(buf)
                    if data is None:
                        break
                    if not data:
                        continue
                    parsed = _decode_jpeg_to_rgb(data)
                    if parsed:
                        self._publish_rgb(*parsed)
            _ = boundary  # multipart boundary reserved; JPEG markers are sufficient

    @staticmethod
    def _pop_jpeg(buf: bytes) -> tuple[Optional[bytes], bytes]:
        """Return (jpeg_bytes|None, remainder). None jpeg => need more data."""
        soi = buf.find(b"\xff\xd8")
        if soi < 0:
            return None, (buf[-16:] if len(buf) > 16 else buf)
        eoi = buf.find(b"\xff\xd9", soi + 2)
        if eoi < 0:
            trimmed = buf[soi:]
            if len(trimmed) > 8_000_000:
                trimmed = trimmed[-1_000_000:]
            return None, trimmed
        return buf[soi : eoi + 2], buf[eoi + 2 :]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XavierCamFrontBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
