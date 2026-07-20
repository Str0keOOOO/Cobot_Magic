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
        self.camera.update_intrinsics(
            K_color=np.eye(3),
            distortion_color=np.zeros(5),
            K_ir=np.eye(3) * np.float32(2),
            baseline_ir=0.055,
            T_color_from_ir=np.eye(4),
        )
        self.server = CobotMagicCameraServer({"front": self.camera}, "127.0.0.1", 5556)

    def test_list_and_rpc_order_is_independent(self) -> None:
        listed = self.server.handle_request(request("list_cameras"))
        self.assertTrue(listed["success"])
        intrinsics = self.server.handle_request(request("get_intrinsics", {"serial": "339222070351"}))
        self.assertTrue(intrinsics["success"])
        result = intrinsics["result"]
        self.assertEqual(
            set(result),
            {"serial", "K_color", "distortion_color", "K_ir", "baseline_ir", "T_color_from_ir"},
        )
        self.assertEqual(result["baseline_ir"], 0.055)
        self.assertEqual(result["distortion_color"].shape, (5,))

        self.camera.update_snapshot(
            timestamp=1.0,
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            ir_left=np.ones((2, 2), dtype=np.uint8),
            ir_right=np.full((2, 2), 2, dtype=np.uint8),
        )
        read = self.server.handle_request(request("read_camera", {"serial": "339222070351"}))
        self.assertTrue(read["success"])
        result = read["result"]
        self.assertEqual(set(result), {"serial", "timestamp", "rgb", "ir_left", "ir_right"})
        self.assertEqual(result["rgb"].shape, (2, 2, 3))
        self.assertEqual(result["ir_left"].shape, (2, 2))
        self.assertEqual(result["ir_right"].shape, (2, 2))
        self.assertEqual(result["ir_left"].dtype, np.uint8)
        self.assertNotIn("left_ir_rgb", result)
        self.assertNotIn("right_ir_rgb", result)

    def test_unknown_camera_and_unknown_operation(self) -> None:
        response = self.server.handle_request(request("read_camera", {"serial": "not-present"}))
        self.assertFalse(response["success"])
        self.assertEqual(response["error"]["code"], "INVALID_REQUEST")
        response = self.server.handle_request(request("not-present"))
        self.assertFalse(response["success"])
        self.assertEqual(response["error"]["code"], "UNKNOWN_OPERATION")
