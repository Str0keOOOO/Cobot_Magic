from __future__ import annotations

import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

from tiptop_client.core.errors import CameraNotReadyError
from tiptop_client.ros.camera_bridge import CameraRosBridge


def camera_config():
    return {
        "namespace": "/camera_f",
        "serial": "419622072184",
        "color_topic": "/camera_f/color/image_raw",
        "aligned_depth_topic": "/camera_f/aligned_depth_to_color/image_raw",
        "color_info_topic": "/camera_f/color/camera_info",
        "left_ir_topic": None,
        "right_ir_topic": None,
        "ir_info_topic": None,
        "depth_scale": 0.001,
    }


def rgb_only_camera_config():
    config = camera_config()
    config["aligned_depth_topic"] = None
    config["depth_scale"] = None
    return config


class CameraRosTest(unittest.TestCase):
    def test_start_initializes_ros_before_subscribing(self) -> None:
        init_calls = []
        subscriber_topics = []

        rospy = types.ModuleType("rospy")
        rospy.core = types.SimpleNamespace(is_initialized=lambda: False)
        rospy.init_node = lambda *args, **kwargs: init_calls.append((args, kwargs))

        class Subscriber:
            def __init__(self, topic, *_args, **_kwargs) -> None:
                subscriber_topics.append(topic)

            def unregister(self) -> None:
                return None

        class Synchronizer:
            def __init__(self, *_args, **_kwargs) -> None:
                return None

            def registerCallback(self, _callback) -> None:
                return None

        message_filters = types.ModuleType("message_filters")
        message_filters.Subscriber = Subscriber
        message_filters.ApproximateTimeSynchronizer = Synchronizer

        cv_bridge = types.ModuleType("cv_bridge")
        cv_bridge.CvBridge = type("CvBridge", (), {})

        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.CameraInfo = type("CameraInfo", (), {})
        sensor_msgs_msg.Image = type("Image", (), {})
        sensor_msgs.msg = sensor_msgs_msg

        modules = {
            "rospy": rospy,
            "message_filters": message_filters,
            "cv_bridge": cv_bridge,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
        }
        with patch.dict(sys.modules, modules):
            CameraRosBridge(
                "front", rgb_only_camera_config(), {"max_snapshot_age_s": 1.0}
            )

        self.assertEqual(
            init_calls,
            [(("cobot_magic_tiptop_camera_bridge",), {"anonymous": True})],
        )
        self.assertEqual(
            subscriber_topics,
            ["/camera_f/color/image_raw", "/camera_f/color/camera_info"],
        )

    def test_snapshot_rgb_depth_and_intrinsics(self) -> None:
        bridge = CameraRosBridge("front", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False)
        bgr = np.array([[[1, 2, 3]]], dtype=np.uint8)
        bridge.update_snapshot(
            timestamp=10.0,
            rgb=bgr[..., ::-1],
            depth_m=np.array([[1.25]], dtype=np.float32),
            K_color=np.eye(3),
            distortion_color=np.zeros(5),
        )
        snapshot = bridge.read_snapshot()
        np.testing.assert_array_equal(snapshot.rgb, np.array([[[3, 2, 1]]], dtype=np.uint8))
        self.assertEqual(snapshot.depth_m.dtype, np.float32)
        self.assertEqual(snapshot.K_color.shape, (3, 3))

    def test_snapshot_timeout(self) -> None:
        bridge = CameraRosBridge("front", camera_config(), {"max_snapshot_age_s": 0.001}, autostart=False)
        bridge.update_snapshot(
            timestamp=1.0,
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            depth_m=None,
            K_color=np.eye(3),
            distortion_color=np.zeros(5),
        )
        bridge._snapshot_received_monotonic -= 1.0
        with self.assertRaises(CameraNotReadyError):
            bridge.read_snapshot()

    def test_ir_shape_is_validated(self) -> None:
        bridge = CameraRosBridge("front", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False)
        with self.assertRaisesRegex(ValueError, "Both IR"):
            bridge.update_snapshot(
                timestamp=1.0,
                rgb=np.zeros((2, 2, 3), dtype=np.uint8),
                depth_m=None,
                K_color=np.eye(3),
                distortion_color=np.zeros(5),
                left_ir_rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            )

    def test_rgb_only_camera_needs_no_depth_scale(self) -> None:
        bridge = CameraRosBridge(
            "front", rgb_only_camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False
        )
        bridge.update_snapshot(
            timestamp=1.0,
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            depth_m=None,
            K_color=np.eye(3),
            distortion_color=np.zeros(5),
        )
        self.assertIsNone(bridge.read_snapshot().depth_m)
