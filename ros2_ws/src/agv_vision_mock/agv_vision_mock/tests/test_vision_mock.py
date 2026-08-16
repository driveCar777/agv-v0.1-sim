"""agv_vision_mock 单测 — 完全离线。"""

import random

from agv_vision_mock.presets import (
    CAMERA_LAYOUT,
    PRESET_OBJECTS,
    PRESET_QRS,
    PRESET_USERS,
)


def test_preset_users_default():
    assert len(PRESET_USERS) >= 3
    assert all(isinstance(u, str) for u in PRESET_USERS)


def test_preset_qrs_default():
    assert len(PRESET_QRS) >= 3


def test_preset_objects_default():
    assert len(PRESET_OBJECTS) >= 3


def test_camera_layout():
    assert "front" in CAMERA_LAYOUT
    assert "wrist" in CAMERA_LAYOUT
    for k, (topic, frame, w, h) in CAMERA_LAYOUT.items():
        assert isinstance(topic, str)
        assert isinstance(frame, str)
        assert w > 0 and h > 0


def test_random_image_size_matches_layout():
    for cam, (topic, frame, w, h) in CAMERA_LAYOUT.items():
        random.seed(0)
        data = bytearray(random.randint(0, 255) for _ in range(w * h * 3))
        assert len(data) == w * h * 3
