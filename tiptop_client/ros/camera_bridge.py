"""ROS RealSense implementation of TiPToP's remote camera contract."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable

import numpy as np

from ..core.errors import CameraNotReadyError, ConfigurationError


@dataclass(frozen=True)
class RemoteCameraSnapshot:
    """One hardware-synchronised RGB/left-IR/right-IR frame triplet."""

    serial: str
    timestamp: float
    rgb: np.ndarray
    ir_left: np.ndarray
    ir_right: np.ndarray


@dataclass(frozen=True)
class RemoteCameraIntrinsics:
    """Calibration required by TiPToP's ``RemoteRealsenseCamera``."""

    serial: str
    K_color: np.ndarray
    distortion_color: np.ndarray
    K_ir: np.ndarray
    baseline_ir: float
    T_color_from_ir: np.ndarray


@dataclass(frozen=True)
class _ColorInfo:
    K: np.ndarray
    D: np.ndarray
    frame_id: str


@dataclass(frozen=True)
class _LeftIrInfo:
    K: np.ndarray
    frame_id: str


@dataclass(frozen=True)
class _RightIrInfo:
    baseline_ir: float


class CameraRosBridge:
    """Cache RealSense RGB/IR frames and calibration for RPC reads.

    RGB, IR1 and IR2 are the only members of the image synchroniser.  Their
    CameraInfo messages are subscribed independently because they are usually
    latched and must make ``get_intrinsics`` available before any image has
    been received.
    """

    def __init__(
        self,
        role: str,
        camera_config: dict[str, Any],
        server_config: dict[str, Any],
        *,
        autostart: bool = True,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.role = str(role)
        self.config = camera_config
        self.serial = self._required_str("serial")
        self.namespace = self._required_str("namespace")
        self.color_topic = self._required_str("color_topic")
        self.left_ir_topic = self._required_str("left_ir_topic")
        self.right_ir_topic = self._required_str("right_ir_topic")
        self.color_info_topic = self._required_str("color_info_topic")
        self.left_ir_info_topic = self._required_str("left_ir_info_topic")
        self.right_ir_info_topic = self._required_str("right_ir_info_topic")

        self.max_snapshot_age_s = float(server_config.get("max_snapshot_age_s", 0.25))
        self.sync_queue_size = int(server_config.get("sync_queue_size", 10))
        self.sync_slop_s = float(server_config.get("sync_slop_s", 0.05))
        self.tf_timeout_s = float(server_config.get("tf_timeout_s", 0.2))
        if (
            self.max_snapshot_age_s <= 0
            or self.sync_queue_size <= 0
            or self.sync_slop_s < 0
            or self.tf_timeout_s < 0
        ):
            raise ConfigurationError("Invalid camera snapshot/synchronization configuration")

        self._monotonic = monotonic
        self._snapshot_lock = threading.Lock()
        self._latest_snapshot: RemoteCameraSnapshot | None = None
        self._snapshot_received_monotonic: float | None = None

        self._intrinsics_lock = threading.Lock()
        self._latest_intrinsics: RemoteCameraIntrinsics | None = None
        self._intrinsics_error: str | None = None
        self._color_info: _ColorInfo | None = None
        self._left_ir_info: _LeftIrInfo | None = None
        self._right_ir_info: _RightIrInfo | None = None

        self._subscriber_handles: list[Any] = []
        self._rospy: Any | None = None
        self._tf_buffer: Any | None = None
        if autostart:
            self.start()

    def _required_str(self, key: str) -> str:
        value = str(self.config.get(key, "")).strip()
        if not value:
            raise ConfigurationError(f"camera {self.role!r} requires {key}")
        return value

    def start(self) -> None:
        """Start long-lived image, CameraInfo and TF subscribers."""
        try:
            import message_filters  # type: ignore
            import rospy  # type: ignore
            import tf2_ros  # type: ignore
            from cv_bridge import CvBridge  # type: ignore
            from sensor_msgs.msg import CameraInfo, Image  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "CameraRosBridge must run in a sourced ROS1/cv_bridge environment"
            ) from exc

        self._rospy = rospy
        self._ensure_ros_node()
        self._cv_bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        image_subscribers = [
            message_filters.Subscriber(self.color_topic, Image),
            message_filters.Subscriber(self.left_ir_topic, Image),
            message_filters.Subscriber(self.right_ir_topic, Image),
        ]
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            image_subscribers,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_s,
        )
        synchronizer.registerCallback(self._ros_image_callback)

        info_subscribers = [
            rospy.Subscriber(self.color_info_topic, CameraInfo, self._color_info_callback),
            rospy.Subscriber(
                self.left_ir_info_topic, CameraInfo, self._left_ir_info_callback
            ),
            rospy.Subscriber(
                self.right_ir_info_topic, CameraInfo, self._right_ir_info_callback
            ),
        ]
        self._subscriber_handles = image_subscribers + [synchronizer] + info_subscribers

    def _ensure_ros_node(self) -> None:
        """Initialise the ROS node before registering camera subscribers."""
        if self._rospy is None:
            raise RuntimeError("ROS has not been imported")
        core = getattr(self._rospy, "core", None)
        is_initialized = getattr(core, "is_initialized", None)
        if callable(is_initialized) and is_initialized():
            return
        init_node = getattr(self._rospy, "init_node", None)
        if not callable(init_node):
            raise RuntimeError("rospy does not provide init_node")
        init_node("cobot_magic_tiptop_camera_bridge", anonymous=True)

    def _ros_image_callback(self, color: Any, left: Any, right: Any) -> None:
        """Convert one synchronised RGB/IR1/IR2 ROS frame triplet."""
        rgb = self._as_rgb(self._cv_bridge.imgmsg_to_cv2(color, desired_encoding="rgb8"))
        ir_left = self._as_ir_gray(
            self._cv_bridge.imgmsg_to_cv2(left, desired_encoding="passthrough")
        )
        ir_right = self._as_ir_gray(
            self._cv_bridge.imgmsg_to_cv2(right, desired_encoding="passthrough")
        )
        timestamps = (
            self._stamp_seconds(color),
            self._stamp_seconds(left),
            self._stamp_seconds(right),
        )
        if max(timestamps) - min(timestamps) > self.sync_slop_s:
            raise ValueError("RGB and IR image stamps exceed the configured sync slop")
        self.update_snapshot(
            timestamp=timestamps[0], rgb=rgb, ir_left=ir_left, ir_right=ir_right
        )

    @staticmethod
    def _stamp_seconds(message: Any) -> float:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        to_sec = getattr(stamp, "to_sec", None)
        if not callable(to_sec):
            raise ValueError("Synchronized ROS image is missing header.stamp")
        timestamp = float(to_sec())
        if not np.isfinite(timestamp):
            raise ValueError("Synchronized ROS image header.stamp must be finite")
        return timestamp

    @staticmethod
    def _as_rgb(image: Any) -> np.ndarray:
        result = np.asarray(image)
        if result.dtype != np.uint8 or result.ndim != 3 or result.shape[2] != 3:
            raise ValueError("RGB image must be uint8 [H,W,3] in RGB channel order")
        return np.ascontiguousarray(result)

    @staticmethod
    def _as_ir_gray(image: Any) -> np.ndarray:
        result = np.asarray(image)
        if result.dtype != np.uint8 or result.ndim != 2:
            raise ValueError("IR image must be a uint8 [H,W] single-channel grayscale image")
        return np.ascontiguousarray(result)

    def _color_info_callback(self, info: Any) -> None:
        K, D = self._camera_info(info, require_five_distortion=True)
        color_info = _ColorInfo(K=K, D=D, frame_id=self._frame_id(info, "color"))
        with self._intrinsics_lock:
            self._color_info = color_info
            self._latest_intrinsics = None
            self._intrinsics_error = None
        self._refresh_intrinsics_from_ros()

    def _left_ir_info_callback(self, info: Any) -> None:
        K, _ = self._camera_info(info, require_five_distortion=False)
        left_ir_info = _LeftIrInfo(K=K, frame_id=self._frame_id(info, "left IR"))
        with self._intrinsics_lock:
            self._left_ir_info = left_ir_info
            self._latest_intrinsics = None
            self._intrinsics_error = None
        self._refresh_intrinsics_from_ros()

    def _right_ir_info_callback(self, info: Any) -> None:
        right_ir_info = _RightIrInfo(baseline_ir=self._baseline_from_right_info(info))
        with self._intrinsics_lock:
            self._right_ir_info = right_ir_info
            self._latest_intrinsics = None
            self._intrinsics_error = None
        self._refresh_intrinsics_from_ros()

    @staticmethod
    def _camera_info(
        info: Any, *, require_five_distortion: bool
    ) -> tuple[np.ndarray, np.ndarray]:
        K = np.asarray(getattr(info, "K", ()), dtype=np.float32)
        D = np.asarray(getattr(info, "D", ()), dtype=np.float32)
        if K.shape != (9,) or not np.all(np.isfinite(K)):
            raise ValueError("CameraInfo.K must contain nine finite values")
        if not np.all(np.isfinite(D)):
            raise ValueError("CameraInfo.D must contain finite values")
        if require_five_distortion and D.shape != (5,):
            raise ValueError("color CameraInfo.D must contain exactly five values")
        return np.ascontiguousarray(K.reshape(3, 3)), np.ascontiguousarray(D)

    @staticmethod
    def _frame_id(info: Any, stream_name: str) -> str:
        frame_id = str(getattr(getattr(info, "header", None), "frame_id", "")).strip()
        if not frame_id:
            raise ValueError(f"{stream_name} CameraInfo.header.frame_id is required")
        return frame_id

    @staticmethod
    def _baseline_from_right_info(info: Any) -> float:
        P = np.asarray(getattr(info, "P", ()), dtype=np.float64)
        if P.shape != (12,) or not np.all(np.isfinite(P)):
            raise ValueError("right IR CameraInfo.P must contain twelve finite values")
        if P[0] == 0:
            raise ValueError("right IR CameraInfo.P[0,0] must be non-zero")
        baseline_ir = float(abs(P[3] / P[0]))
        if not np.isfinite(baseline_ir) or baseline_ir <= 0:
            raise ValueError("right IR CameraInfo.P must encode a positive baseline in metres")
        return baseline_ir

    def _refresh_intrinsics_from_ros(self) -> None:
        """Attempt calibration refresh after CameraInfo or an RPC request.

        A TF lookup can legitimately fail while the driver is starting.  That
        failure is cached as readiness information and retried by each later
        CameraInfo callback and by ``read_intrinsics``.
        """
        with self._intrinsics_lock:
            color_info = self._color_info
            left_ir_info = self._left_ir_info
            right_ir_info = self._right_ir_info
        if color_info is None or left_ir_info is None or right_ir_info is None:
            return
        if self._tf_buffer is None or self._rospy is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                color_info.frame_id,
                left_ir_info.frame_id,
                self._rospy.Time(0),
                self._rospy.Duration(self.tf_timeout_s),
            )
            T_color_from_ir = self._transform_matrix(transform)
            self.update_intrinsics(
                K_color=color_info.K,
                distortion_color=color_info.D,
                K_ir=left_ir_info.K,
                baseline_ir=right_ir_info.baseline_ir,
                T_color_from_ir=T_color_from_ir,
            )
        except Exception as exc:
            with self._intrinsics_lock:
                self._intrinsics_error = (
                    "Waiting for TF transform "
                    f"{color_info.frame_id!r} <- {left_ir_info.frame_id!r}: {exc}"
                )

    @staticmethod
    def _transform_matrix(transform_stamped: Any) -> np.ndarray:
        transform = getattr(transform_stamped, "transform", transform_stamped)
        translation = getattr(transform, "translation", None)
        rotation = getattr(transform, "rotation", None)
        values = np.asarray(
            [
                getattr(translation, "x", np.nan),
                getattr(translation, "y", np.nan),
                getattr(translation, "z", np.nan),
                getattr(rotation, "x", np.nan),
                getattr(rotation, "y", np.nan),
                getattr(rotation, "z", np.nan),
                getattr(rotation, "w", np.nan),
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("TF color-from-left-IR transform must be finite")
        tx, ty, tz, x, y, z, w = values
        norm = float(np.linalg.norm((x, y, z, w)))
        if norm == 0:
            raise ValueError("TF color-from-left-IR transform has a zero quaternion")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = np.asarray(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )
        T[:3, 3] = np.asarray((tx, ty, tz), dtype=np.float32)
        return T

    def update_intrinsics(
        self,
        *,
        K_color: np.ndarray,
        distortion_color: np.ndarray,
        K_ir: np.ndarray,
        baseline_ir: float,
        T_color_from_ir: np.ndarray,
    ) -> None:
        """Store calibration independently of image arrival (also test hook)."""
        K_color = self._finite_array(K_color, (3, 3), "K_color")
        distortion_color = self._finite_array(
            distortion_color, (5,), "distortion_color"
        )
        K_ir = self._finite_array(K_ir, (3, 3), "K_ir")
        T_color_from_ir = self._finite_array(
            T_color_from_ir, (4, 4), "T_color_from_ir"
        )
        if not np.array_equal(
            T_color_from_ir[3], np.asarray((0, 0, 0, 1), dtype=np.float32)
        ):
            raise ValueError("T_color_from_ir last row must be [0, 0, 0, 1]")
        baseline_ir = float(baseline_ir)
        if not np.isfinite(baseline_ir) or baseline_ir <= 0:
            raise ValueError("baseline_ir must be finite and positive (metres)")
        intrinsics = RemoteCameraIntrinsics(
            serial=self.serial,
            K_color=K_color.copy(),
            distortion_color=distortion_color.copy(),
            K_ir=K_ir.copy(),
            baseline_ir=baseline_ir,
            T_color_from_ir=T_color_from_ir.copy(),
        )
        with self._intrinsics_lock:
            self._latest_intrinsics = intrinsics
            self._intrinsics_error = None

    @staticmethod
    def _finite_array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
        result = np.ascontiguousarray(value, dtype=np.float32)
        if result.shape != shape or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must have shape {shape} with finite values")
        return result

    def update_snapshot(
        self,
        *,
        timestamp: float,
        rgb: np.ndarray,
        ir_left: np.ndarray,
        ir_right: np.ndarray,
    ) -> None:
        """Store one validated RGB/IR triplet; used directly by unit tests."""
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        rgb = self._as_rgb(rgb)
        ir_left = self._as_ir_gray(ir_left)
        ir_right = self._as_ir_gray(ir_right)
        if rgb.shape[:2] != ir_left.shape or ir_left.shape != ir_right.shape:
            raise ValueError("RGB, left IR and right IR resolutions must match exactly")
        snapshot = RemoteCameraSnapshot(
            serial=self.serial,
            timestamp=timestamp,
            rgb=rgb.copy(),
            ir_left=ir_left.copy(),
            ir_right=ir_right.copy(),
        )
        with self._snapshot_lock:
            self._latest_snapshot = snapshot
            self._snapshot_received_monotonic = self._monotonic()

    def read_snapshot(self) -> RemoteCameraSnapshot:
        with self._snapshot_lock:
            snapshot = self._latest_snapshot
            received = self._snapshot_received_monotonic
        if snapshot is None or received is None:
            raise CameraNotReadyError(
                f"No synchronized RGB/IR snapshot received for {self.namespace}"
            )
        age = self._monotonic() - received
        if age > self.max_snapshot_age_s:
            raise CameraNotReadyError(
                f"Camera snapshot is stale ({age:.3f}s > {self.max_snapshot_age_s:.3f}s)"
            )
        return snapshot

    def read_intrinsics(self) -> RemoteCameraIntrinsics:
        """Return calibration even when no image snapshot has arrived yet."""
        self._refresh_intrinsics_from_ros()
        with self._intrinsics_lock:
            intrinsics = self._latest_intrinsics
            error = self._intrinsics_error
        if intrinsics is None:
            detail = f" ({error})" if error else ""
            raise CameraNotReadyError(
                f"Camera calibration is not ready for {self.namespace}{detail}"
            )
        return intrinsics

    def health(self) -> dict[str, Any]:
        with self._snapshot_lock:
            received = self._snapshot_received_monotonic
            snapshot = self._latest_snapshot
        with self._intrinsics_lock:
            intrinsics = self._latest_intrinsics
            intrinsics_error = self._intrinsics_error
        age = None if received is None else max(0.0, self._monotonic() - received)
        return {
            "namespace": self.namespace,
            "serial": self.serial,
            "role": self.role,
            "snapshot_received": snapshot is not None,
            "snapshot_age_s": age,
            "calibration_ready": intrinsics is not None,
            "calibration_error": intrinsics_error,
            "has_ir": True,
        }

    def close(self) -> None:
        for handle in self._subscriber_handles:
            unregister = getattr(handle, "unregister", None)
            if callable(unregister):
                unregister()
        self._subscriber_handles = []
