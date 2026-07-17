"""ROS RealSense snapshot cache for the Cobot Magic upper computer."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable

import numpy as np

from ..core.errors import CameraNotReadyError, ConfigurationError


@dataclass(frozen=True)
class RemoteCameraSnapshot:
    serial: str
    timestamp: float
    rgb: np.ndarray
    depth_m: np.ndarray | None
    K_color: np.ndarray
    distortion_color: np.ndarray
    left_ir_rgb: np.ndarray | None = None
    right_ir_rgb: np.ndarray | None = None
    K_ir: np.ndarray | None = None
    baseline_ir_m: float | None = None
    T_color_from_ir: np.ndarray | None = None


class CameraRosBridge:
    """Continuously cache synchronized RGB/depth/CameraInfo frames.

    RPC reads a copy of this cache; it never creates a subscriber per request.
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
        self.aligned_depth_topic = self._optional_str("aligned_depth_topic")
        self.color_info_topic = self._required_str("color_info_topic")
        self.left_ir_topic = self._optional_str("left_ir_topic")
        self.right_ir_topic = self._optional_str("right_ir_topic")
        self.ir_info_topic = self._optional_str("ir_info_topic")
        raw_depth_scale = camera_config.get("depth_scale")
        if self.aligned_depth_topic and raw_depth_scale is None:
            raise ConfigurationError(
                f"camera {self.role!r} depth_scale is unverified; refusing to guess depth units"
            )
        self.depth_scale = None if raw_depth_scale is None else float(raw_depth_scale)
        if self.depth_scale is not None and (
            not np.isfinite(self.depth_scale) or self.depth_scale <= 0
        ):
            raise ConfigurationError("camera depth_scale must be finite and positive")
        self.max_snapshot_age_s = float(server_config.get("max_snapshot_age_s", 0.25))
        self.sync_queue_size = int(server_config.get("sync_queue_size", 10))
        self.sync_slop_s = float(server_config.get("sync_slop_s", 0.05))
        if self.max_snapshot_age_s <= 0 or self.sync_queue_size <= 0 or self.sync_slop_s < 0:
            raise ConfigurationError("Invalid camera snapshot/synchronization configuration")
        if bool(self.left_ir_topic) != bool(self.right_ir_topic):
            raise ConfigurationError("Configure both IR streams or neither IR stream")
        if self.left_ir_topic and not self.ir_info_topic:
            raise ConfigurationError("IR streams require ir_info_topic")

        self._monotonic = monotonic
        self._snapshot_lock = threading.Lock()
        self._latest_snapshot: RemoteCameraSnapshot | None = None
        self._snapshot_received_monotonic: float | None = None
        self._subscriber_handles: list[Any] = []
        if autostart:
            self.start()

    def _required_str(self, key: str) -> str:
        value = str(self.config.get(key, "")).strip()
        if not value:
            raise ConfigurationError(f"camera {self.role!r} requires {key}")
        return value

    def _optional_str(self, key: str) -> str | None:
        value = self.config.get(key)
        if value is None:
            return None
        value = str(value).strip()
        return value or None

    def start(self) -> None:
        """Create the one long-lived ApproximateTimeSynchronizer for this camera."""
        try:
            import rospy  # type: ignore
            import message_filters  # type: ignore
            from cv_bridge import CvBridge  # type: ignore
            from sensor_msgs.msg import CameraInfo, Image  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "CameraRosBridge must run in a sourced ROS1/cv_bridge environment"
            ) from exc
        self._rospy = rospy
        self._ensure_ros_node()
        self._cv_bridge = CvBridge()
        subscribers = [
            message_filters.Subscriber(self.color_topic, Image),
        ]
        if self.aligned_depth_topic:
            subscribers.append(message_filters.Subscriber(self.aligned_depth_topic, Image))
        subscribers.append(message_filters.Subscriber(self.color_info_topic, CameraInfo))
        if self.left_ir_topic:
            subscribers.extend(
                [
                    message_filters.Subscriber(self.left_ir_topic, Image),
                    message_filters.Subscriber(self.right_ir_topic, Image),
                    message_filters.Subscriber(self.ir_info_topic, CameraInfo),
                ]
            )
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            subscribers,
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_s,
        )
        synchronizer.registerCallback(self._ros_callback)
        self._subscriber_handles = subscribers + [synchronizer]

    def _ensure_ros_node(self) -> None:
        """Initialise the ROS node before registering camera subscribers."""
        core = getattr(self._rospy, "core", None)
        is_initialized = getattr(core, "is_initialized", None)
        if callable(is_initialized) and is_initialized():
            return
        init_node = getattr(self._rospy, "init_node", None)
        if not callable(init_node):
            raise RuntimeError("rospy does not provide init_node")
        init_node("cobot_magic_tiptop_camera_bridge", anonymous=True)

    def _ros_callback(self, *messages: Any) -> None:
        color = messages[0]
        if self.aligned_depth_topic:
            depth = messages[1]
            color_info = messages[2]
            ir = messages[3:]
        else:
            depth = None
            color_info = messages[1]
            ir = messages[2:]
        bgr = self._cv_bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
        rgb = np.ascontiguousarray(bgr[..., ::-1], dtype=np.uint8)
        depth_m = None
        if depth is not None:
            raw_depth = self._cv_bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
            depth_m = self._depth_to_meters(raw_depth, getattr(depth, "encoding", ""))
        K_color, distortion_color = self._camera_info(color_info)
        left_ir_rgb = right_ir_rgb = K_ir = None
        if ir:
            left, right, ir_info = ir
            left_ir_rgb = self._ir_to_rgb(
                self._cv_bridge.imgmsg_to_cv2(left, desired_encoding="passthrough")
            )
            right_ir_rgb = self._ir_to_rgb(
                self._cv_bridge.imgmsg_to_cv2(right, desired_encoding="passthrough")
            )
            K_ir, _ = self._camera_info(ir_info)
        stamp = getattr(getattr(color, "header", None), "stamp", None)
        timestamp = float(stamp.to_sec()) if stamp is not None else time.time()
        self.update_snapshot(
            timestamp=timestamp,
            rgb=rgb,
            depth_m=depth_m,
            K_color=K_color,
            distortion_color=distortion_color,
            left_ir_rgb=left_ir_rgb,
            right_ir_rgb=right_ir_rgb,
            K_ir=K_ir,
        )

    def _depth_to_meters(self, raw_depth: Any, encoding: str) -> np.ndarray:
        if self.depth_scale is None:
            raise RuntimeError("Depth image received without a configured depth_scale")
        depth = np.asarray(raw_depth)
        if depth.ndim != 2:
            raise ValueError(f"Depth image must be [H,W], got {depth.shape}")
        if np.issubdtype(depth.dtype, np.floating):
            # RealSense 32FC1 images conventionally carry metres.  Do not apply
            # the uint16 depth scale a second time.
            scale = 1.0
        elif np.issubdtype(depth.dtype, np.integer):
            scale = self.depth_scale
        else:
            raise ValueError(f"Unsupported depth dtype {depth.dtype} ({encoding})")
        result = np.ascontiguousarray(depth, dtype=np.float32) * np.float32(scale)
        if not np.all(np.isfinite(result) | np.isnan(result)):
            raise ValueError("Depth contains unsupported non-finite values")
        return result

    @staticmethod
    def _ir_to_rgb(raw_ir: Any) -> np.ndarray:
        image = np.asarray(raw_ir)
        if image.ndim != 2 or image.dtype != np.uint8:
            raise ValueError("IR image must be an 8-bit single-channel image")
        return np.ascontiguousarray(np.repeat(image[..., None], 3, axis=2))

    @staticmethod
    def _camera_info(info: Any) -> tuple[np.ndarray, np.ndarray]:
        K = np.asarray(getattr(info, "K", ()), dtype=np.float32)
        D = np.asarray(getattr(info, "D", ()), dtype=np.float32)
        if K.shape != (9,) or not np.all(np.isfinite(K)):
            raise ValueError("CameraInfo.K must contain nine finite values")
        if not np.all(np.isfinite(D)):
            raise ValueError("CameraInfo.D must contain finite values")
        return np.ascontiguousarray(K.reshape(3, 3)), np.ascontiguousarray(D)

    def update_snapshot(
        self,
        *,
        timestamp: float,
        rgb: np.ndarray,
        depth_m: np.ndarray | None,
        K_color: np.ndarray,
        distortion_color: np.ndarray,
        left_ir_rgb: np.ndarray | None = None,
        right_ir_rgb: np.ndarray | None = None,
        K_ir: np.ndarray | None = None,
        baseline_ir_m: float | None = None,
        T_color_from_ir: np.ndarray | None = None,
    ) -> None:
        """Store an immutable-by-convention copy; used directly by unit tests."""
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"RGB image must be uint8 [H,W,3], got {rgb.shape}")
        if depth_m is not None:
            depth_m = np.ascontiguousarray(depth_m, dtype=np.float32)
            if depth_m.shape != rgb.shape[:2]:
                raise ValueError("Depth must have the same [H,W] as RGB")
        K_color = np.ascontiguousarray(K_color, dtype=np.float32)
        distortion_color = np.ascontiguousarray(distortion_color, dtype=np.float32)
        if K_color.shape != (3, 3):
            raise ValueError("K_color must have shape (3, 3)")
        if (left_ir_rgb is None) != (right_ir_rgb is None):
            raise ValueError("Both IR frames must be present or both omitted")
        if left_ir_rgb is not None:
            left_ir_rgb = np.ascontiguousarray(left_ir_rgb, dtype=np.uint8)
            right_ir_rgb = np.ascontiguousarray(right_ir_rgb, dtype=np.uint8)
            if left_ir_rgb.ndim != 3 or left_ir_rgb.shape[2] != 3:
                raise ValueError("Left IR RGB image must be [H,W,3]")
            if right_ir_rgb.shape != left_ir_rgb.shape:
                raise ValueError("IR image shapes must match")
        if K_ir is not None:
            K_ir = np.ascontiguousarray(K_ir, dtype=np.float32)
            if K_ir.shape != (3, 3):
                raise ValueError("K_ir must have shape (3, 3)")
        if baseline_ir_m is not None and (
            not np.isfinite(baseline_ir_m) or baseline_ir_m <= 0
        ):
            raise ValueError("baseline_ir_m must be finite and positive")
        if T_color_from_ir is not None:
            T_color_from_ir = np.ascontiguousarray(T_color_from_ir, dtype=np.float32)
            if T_color_from_ir.shape != (4, 4):
                raise ValueError("T_color_from_ir must have shape (4, 4)")
        snapshot = RemoteCameraSnapshot(
            serial=self.serial,
            timestamp=float(timestamp),
            rgb=rgb.copy(),
            depth_m=None if depth_m is None else depth_m.copy(),
            K_color=K_color.copy(),
            distortion_color=distortion_color.copy(),
            left_ir_rgb=None if left_ir_rgb is None else left_ir_rgb.copy(),
            right_ir_rgb=None if right_ir_rgb is None else right_ir_rgb.copy(),
            K_ir=None if K_ir is None else K_ir.copy(),
            baseline_ir_m=baseline_ir_m,
            T_color_from_ir=(
                None if T_color_from_ir is None else T_color_from_ir.copy()
            ),
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
                f"No synchronized snapshot received for {self.namespace}"
            )
        age = self._monotonic() - received
        if age > self.max_snapshot_age_s:
            raise CameraNotReadyError(
                f"Camera snapshot is stale ({age:.3f}s > {self.max_snapshot_age_s:.3f}s)"
            )
        return snapshot

    def health(self) -> dict[str, Any]:
        with self._snapshot_lock:
            received = self._snapshot_received_monotonic
            snapshot = self._latest_snapshot
        age = None if received is None else max(0.0, self._monotonic() - received)
        return {
            "namespace": self.namespace,
            "serial": self.serial,
            "role": self.role,
            "snapshot_received": snapshot is not None,
            "snapshot_age_s": age,
            "has_ir": bool(self.left_ir_topic),
            "has_depth": snapshot is not None and snapshot.depth_m is not None,
        }

    def close(self) -> None:
        for handle in self._subscriber_handles:
            unregister = getattr(handle, "unregister", None)
            if callable(unregister):
                unregister()
        self._subscriber_handles = []
