"""Vision mock 预置数据 — 无 ROS 依赖。"""

PRESET_USERS = ["alice", "bob", "carol", "dave", "erin", "frank"]
PRESET_QRS = ["QR-PICKUP-A1", "QR-DROP-B2", "QR-CHARGE-01", "QR-USER-001"]
PRESET_OBJECTS = ["meal_box", "drink_bottle", "document", "package"]
CAMERA_LAYOUT = {
    "front": ("/camera/front/image_raw", "camera_front", 640, 480),
    "rear": ("/camera/rear/image_raw", "camera_rear", 640, 480),
    "wrist": ("/camera/wrist/image_raw", "wrist_camera", 320, 240),
}
