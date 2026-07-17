from __future__ import annotations

import unittest
import uuid

import numpy as np

from tiptop_client.core.protocol import PROTOCOL_VERSION
from tiptop_client.ros.camera_bridge import CameraRosBridge
from tiptop_client.services.camera import CobotMagicCameraServer

from .test_camera_ros import camera_config


def request(op, params=None):
    return {
        "protocol_version": PROTOCOL_VERSION,
        "request_id": str(uuid.uuid4()),
        "op": op,
        "params": {} if params is None else params,
    }


class CameraServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.camera = CameraRosBridge("front", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False)
        self.camera.update_snapshot(
            timestamp=1.0,
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            depth_m=np.ones((2, 2), dtype=np.float32),
            K_color=np.eye(3),
            distortion_color=np.zeros(5),
        )
        self.server = CobotMagicCameraServer({"front": self.camera}, "127.0.0.1", 5556)

    def test_list_and_read(self) -> None:
        listed = self.server.handle_request(request("list_cameras"))
        self.assertTrue(listed["success"])
        read = self.server.handle_request(request("read_camera", {"serial": "419622072184"}))
        self.assertTrue(read["success"])
        self.assertEqual(read["result"]["depth"].shape, (2, 2))
        self.assertEqual(read["result"]["K_color"].shape, (3, 3))
        np.testing.assert_array_equal(read["result"]["K_color"], read["result"]["intrinsics"])

    def test_unknown_camera_and_unknown_operation(self) -> None:
        response = self.server.handle_request(request("read_camera", {"serial": "not-present"}))
        self.assertFalse(response["success"])
        self.assertEqual(response["error"]["code"], "INVALID_REQUEST")
        response = self.server.handle_request(request("not-present"))
        self.assertFalse(response["success"])
        self.assertEqual(response["error"]["code"], "UNKNOWN_OPERATION")
