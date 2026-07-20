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
        "namespace": "/camera_r",
        "serial": "339222070351",
        "color_topic": "/camera_r/color/image_raw",
        "left_ir_topic": "/camera_r/infra1/image_rect_raw",
        "right_ir_topic": "/camera_r/infra2/image_rect_raw",
        "color_info_topic": "/camera_r/color/camera_info",
        "left_ir_info_topic": "/camera_r/infra1/camera_info",
        "right_ir_info_topic": "/camera_r/infra2/camera_info",
    }


class FakeStamp:
    def __init__(self, seconds: float) -> None:
        self.seconds = seconds

    def to_sec(self) -> float:
        return self.seconds


class FakeImage:
    def __init__(self, seconds: float) -> None:
        self.header = types.SimpleNamespace(stamp=FakeStamp(seconds))


class CameraRosTest(unittest.TestCase):
    def test_start_initializes_ros_and_subscribes_rgb_ir_and_info(self) -> None:
        init_calls = []
        image_topics = []
        info_topics = []

        rospy = types.ModuleType("rospy")
        rospy.core = types.SimpleNamespace(is_initialized=lambda: False)
        rospy.init_node = lambda *args, **kwargs: init_calls.append((args, kwargs))

        class RosSubscriber:
            def __init__(self, topic, *_args, **_kwargs) -> None:
                info_topics.append(topic)

            def unregister(self) -> None:
                return None

        rospy.Subscriber = RosSubscriber
        rospy.Time = lambda value: value
        rospy.Duration = lambda value: value

        class ImageSubscriber:
            def __init__(self, topic, *_args, **_kwargs) -> None:
                image_topics.append(topic)

            def unregister(self) -> None:
                return None

        class Synchronizer:
            def __init__(self, *_args, **_kwargs) -> None:
                return None

            def registerCallback(self, _callback) -> None:
                return None

            def unregister(self) -> None:
                return None

        message_filters = types.ModuleType("message_filters")
        message_filters.Subscriber = ImageSubscriber
        message_filters.ApproximateTimeSynchronizer = Synchronizer

        cv_bridge = types.ModuleType("cv_bridge")
        cv_bridge.CvBridge = type("CvBridge", (), {})

        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.CameraInfo = type("CameraInfo", (), {})
        sensor_msgs_msg.Image = type("Image", (), {})
        sensor_msgs.msg = sensor_msgs_msg

        tf2_ros = types.ModuleType("tf2_ros")
        tf2_ros.Buffer = type("Buffer", (), {})
        tf2_ros.TransformListener = lambda _buffer: object()

        modules = {
            "rospy": rospy,
            "message_filters": message_filters,
            "cv_bridge": cv_bridge,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "tf2_ros": tf2_ros,
        }
        with patch.dict(sys.modules, modules):
            CameraRosBridge("right_wrist", camera_config(), {"max_snapshot_age_s": 1.0})

        self.assertEqual(
            init_calls,
            [(('cobot_magic_tiptop_camera_bridge',), {"anonymous": True})],
        )
        self.assertEqual(
            image_topics,
            [
                "/camera_r/color/image_raw",
                "/camera_r/infra1/image_rect_raw",
                "/camera_r/infra2/image_rect_raw",
            ],
        )
        self.assertEqual(
            info_topics,
            [
                "/camera_r/color/camera_info",
                "/camera_r/infra1/camera_info",
                "/camera_r/infra2/camera_info",
            ],
        )

    def test_snapshot_is_rgb_and_two_single_channel_ir_images(self) -> None:
        bridge = CameraRosBridge(
            "right_wrist", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False
        )
        rgb = np.array([[[3, 2, 1], [6, 5, 4]]], dtype=np.uint8)
        ir_left = np.array([[7, 8]], dtype=np.uint8)
        ir_right = np.array([[9, 10]], dtype=np.uint8)
        bridge.update_snapshot(
            timestamp=10.0, rgb=rgb, ir_left=ir_left, ir_right=ir_right
        )

        snapshot = bridge.read_snapshot()
        np.testing.assert_array_equal(snapshot.rgb, rgb)
        np.testing.assert_array_equal(snapshot.ir_left, ir_left)
        np.testing.assert_array_equal(snapshot.ir_right, ir_right)
        self.assertEqual(snapshot.rgb.dtype, np.uint8)
        self.assertEqual(snapshot.rgb.shape, (1, 2, 3))
        self.assertEqual(snapshot.ir_left.dtype, np.uint8)
        self.assertEqual(snapshot.ir_left.shape, (1, 2))
        self.assertEqual(snapshot.ir_right.dtype, np.uint8)
        self.assertEqual(snapshot.ir_right.shape, (1, 2))

    def test_ros_callback_uses_header_stamp_and_preserves_ir_grayscale(self) -> None:
        bridge = CameraRosBridge(
            "right_wrist", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False
        )
        color, left, right = FakeImage(12.5), FakeImage(12.5), FakeImage(12.5)
        images = {
            id(color): np.array([[[1, 2, 3]]], dtype=np.uint8),
            id(left): np.array([[4]], dtype=np.uint8),
            id(right): np.array([[5]], dtype=np.uint8),
        }

        class CvBridge:
            def imgmsg_to_cv2(self, image, desired_encoding):
                if image is color:
                    self.assertEqual(desired_encoding, "rgb8")
                else:
                    self.assertEqual(desired_encoding, "passthrough")
                return images[id(image)]

            def assertEqual(self, actual, expected):
                self_test.assertEqual(actual, expected)

        self_test = self
        bridge._cv_bridge = CvBridge()
        bridge._ros_image_callback(color, left, right)
        snapshot = bridge.read_snapshot()
        self.assertEqual(snapshot.timestamp, 12.5)
        self.assertEqual(snapshot.ir_left.shape, (1, 1))
        self.assertEqual(snapshot.ir_right.shape, (1, 1))

    def test_snapshot_rejects_mismatched_or_non_grayscale_ir(self) -> None:
        bridge = CameraRosBridge(
            "right_wrist", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False
        )
        with self.assertRaisesRegex(ValueError, "resolutions must match"):
            bridge.update_snapshot(
                timestamp=1.0,
                rgb=np.zeros((2, 2, 3), dtype=np.uint8),
                ir_left=np.zeros((2, 2), dtype=np.uint8),
                ir_right=np.zeros((1, 2), dtype=np.uint8),
            )
        with self.assertRaisesRegex(ValueError, "single-channel"):
            bridge.update_snapshot(
                timestamp=1.0,
                rgb=np.zeros((2, 2, 3), dtype=np.uint8),
                ir_left=np.zeros((2, 2, 3), dtype=np.uint8),
                ir_right=np.zeros((2, 2), dtype=np.uint8),
            )

    def test_snapshot_timeout(self) -> None:
        bridge = CameraRosBridge(
            "right_wrist", camera_config(), {"max_snapshot_age_s": 0.001}, autostart=False
        )
        bridge.update_snapshot(
            timestamp=1.0,
            rgb=np.zeros((2, 2, 3), dtype=np.uint8),
            ir_left=np.zeros((2, 2), dtype=np.uint8),
            ir_right=np.zeros((2, 2), dtype=np.uint8),
        )
        bridge._snapshot_received_monotonic -= 1.0
        with self.assertRaises(CameraNotReadyError):
            bridge.read_snapshot()

    def test_intrinsics_are_independent_of_image_snapshots(self) -> None:
        bridge = CameraRosBridge(
            "right_wrist", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False
        )
        bridge.update_intrinsics(
            K_color=np.eye(3),
            distortion_color=np.zeros(5),
            K_ir=np.diag((2, 2, 1)),
            baseline_ir=0.055,
            T_color_from_ir=np.eye(4),
        )

        intrinsics = bridge.read_intrinsics()
        self.assertEqual(intrinsics.serial, "339222070351")
        self.assertEqual(intrinsics.K_color.dtype, np.float32)
        self.assertEqual(intrinsics.distortion_color.shape, (5,))
        self.assertEqual(intrinsics.K_ir.dtype, np.float32)
        self.assertEqual(intrinsics.baseline_ir, 0.055)
        np.testing.assert_array_equal(intrinsics.T_color_from_ir[3], [0, 0, 0, 1])
        with self.assertRaises(CameraNotReadyError):
            bridge.read_snapshot()

    def test_color_distortion_requires_exactly_five_coefficients(self) -> None:
        info = types.SimpleNamespace(K=np.eye(3).reshape(-1), D=np.zeros(4))
        with self.assertRaisesRegex(ValueError, "exactly five"):
            CameraRosBridge._camera_info(info, require_five_distortion=True)

    def test_camera_info_uses_left_ir_and_tf_color_from_ir_direction(self) -> None:
        bridge = CameraRosBridge(
            "right_wrist", camera_config(), {"max_snapshot_age_s": 1.0}, autostart=False
        )
        lookup_calls = []
        transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=0.01, y=0.02, z=0.03),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

        class TfBuffer:
            def lookup_transform(self, *args):
                lookup_calls.append(args)
                return transform

        bridge._tf_buffer = TfBuffer()
        bridge._rospy = types.SimpleNamespace(
            Time=lambda seconds: ("time", seconds),
            Duration=lambda seconds: ("duration", seconds),
        )
        header = lambda frame_id: types.SimpleNamespace(frame_id=frame_id)
        color = types.SimpleNamespace(
            K=np.diag((500, 501, 1)).reshape(-1),
            D=np.array((0.1, 0.2, 0.3, 0.4, 0.5)),
            header=header("camera_r_color_optical_frame"),
        )
        left = types.SimpleNamespace(
            K=np.diag((600, 601, 1)).reshape(-1),
            D=np.zeros(5),
            header=header("camera_r_infra1_optical_frame"),
        )
        P = np.zeros(12)
        P[0], P[3] = 600.0, -33.0
        right = types.SimpleNamespace(P=P, header=header("camera_r_infra2_optical_frame"))

        bridge._color_info_callback(color)
        bridge._left_ir_info_callback(left)
        bridge._right_ir_info_callback(right)

        expected_lookup = (
            "camera_r_color_optical_frame",
            "camera_r_infra1_optical_frame",
            ("time", 0),
            ("duration", 0.2),
        )
        self.assertEqual(lookup_calls, [expected_lookup])
        intrinsics = bridge.read_intrinsics()
        self.assertTrue(all(call == expected_lookup for call in lookup_calls))
        np.testing.assert_array_equal(intrinsics.K_ir, np.diag((600, 601, 1)))
        self.assertAlmostEqual(intrinsics.baseline_ir, 0.055)
        np.testing.assert_allclose(intrinsics.T_color_from_ir[:3, 3], [0.01, 0.02, 0.03])

    def test_baseline_uses_right_ir_projection_matrix(self) -> None:
        P = np.zeros(12)
        P[0] = 600.0
        P[3] = -33.0
        info = types.SimpleNamespace(P=P)
        self.assertAlmostEqual(CameraRosBridge._baseline_from_right_info(info), 0.055)

    def test_transform_has_color_from_left_ir_direction(self) -> None:
        transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=0.1, y=-0.2, z=0.3),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        T = CameraRosBridge._transform_matrix(transform)
        np.testing.assert_allclose(T, [[1, 0, 0, 0.1], [0, 1, 0, -0.2], [0, 0, 1, 0.3], [0, 0, 0, 1]])
