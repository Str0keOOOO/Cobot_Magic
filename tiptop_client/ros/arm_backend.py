"""ROS1 control backend which runs only on the robot upper computer.

The GPU server sends complete trajectories through RPC.  Interpolation and
publishing stay here, beside the CAN-connected Piper ROS driver.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Any, Callable, Sequence

import numpy as np

from ..core.errors import ConfigurationError, MotionBusyError, RobotNotReadyError


class CobotMagicRosBackend:
    """Drive the right Cobot Magic arm through its existing ROS topics."""

    def __init__(
        self,
        config: dict[str, Any],
        *,
        ros_api: Any | None = None,
        joint_state_type: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        robot = config.get("robot", config)
        if not isinstance(robot, dict):
            raise ConfigurationError("robot configuration must be a mapping")
        self._cfg = robot
        self.arm_dof = self._positive_int("arm_dof", 6)
        self.ros_joint_count = self._positive_int("ros_joint_count", self.arm_dof + 1)
        if self.ros_joint_count < self.arm_dof + 1:
            raise ConfigurationError(
                "ros_joint_count must include all arm joints and the gripper"
            )
        self.state_topic = self._nonempty_str("state_topic", "/puppet/joint_right")
        self.command_topic = self._nonempty_str(
            "command_topic", "/master/joint_right"
        )
        self.servo_rate_hz = self._positive_float("servo_rate_hz", 200.0)
        self.state_timeout_s = self._positive_float("state_timeout_s", 5.0)
        self.max_state_age_s = self._positive_float("max_state_age_s", 0.25)
        self.max_initial_error_rad = self._positive_float(
            "max_initial_error_rad", math.radians(1.0)
        )
        self.gripper_open_position = float(
            self._cfg.get("gripper_open_position", 0.08)
        )
        self.gripper_closed_position = float(
            self._cfg.get("gripper_closed_position", 0.0)
        )
        self.gripper_min_move_time_s = self._positive_float(
            "gripper_min_move_time_s", 0.25
        )
        self.gripper_max_move_time_s = self._positive_float(
            "gripper_max_move_time_s", 1.5
        )
        if self.gripper_max_move_time_s < self.gripper_min_move_time_s:
            raise ConfigurationError(
                "gripper_max_move_time_s must be >= gripper_min_move_time_s"
            )
        self.trajectory_duration_mode = str(
            self._cfg.get("trajectory_duration_mode", "intervals")
        )
        if self.trajectory_duration_mode not in {"intervals", "timestamps"}:
            raise ConfigurationError(
                "trajectory_duration_mode must be 'intervals' or 'timestamps'"
            )

        self._monotonic = monotonic
        self._sleep = sleep
        self._state_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state_event = threading.Event()
        self._latest_state: Any | None = None
        self._latest_state_stamp: float | None = None
        self._latest_joint_names: list[str] = []
        self._trajectory_running = False
        self._last_command: np.ndarray | None = None

        if ros_api is None or joint_state_type is None:
            try:
                import rospy  # type: ignore
                from sensor_msgs.msg import JointState  # type: ignore
            except ImportError as exc:
                raise RuntimeError(
                    "CobotMagicRosBackend must run in a sourced ROS1 environment"
                ) from exc
            ros_api = rospy
            joint_state_type = JointState
        self._ros = ros_api
        self._joint_state_type = joint_state_type
        self._ensure_ros_node()
        self._publisher = self._ros.Publisher(
            self.command_topic,
            self._joint_state_type,
            queue_size=1,
            tcp_nodelay=True,
        )
        self._subscriber = self._ros.Subscriber(
            self.state_topic,
            self._joint_state_type,
            self._state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

    @property
    def dof(self) -> int:
        return self.arm_dof

    def _ensure_ros_node(self) -> None:
        """Initialise one anonymous bridge node, without double initialising ROS."""
        core = getattr(self._ros, "core", None)
        is_initialized = getattr(core, "is_initialized", None)
        if callable(is_initialized) and is_initialized():
            return
        init_node = getattr(self._ros, "init_node", None)
        if callable(init_node):
            init_node("cobot_magic_tiptop_bridge", anonymous=True)

    def _positive_int(self, key: str, default: int) -> int:
        value = int(self._cfg.get(key, default))
        if value <= 0:
            raise ConfigurationError(f"{key} must be positive")
        return value

    def _positive_float(self, key: str, default: float) -> float:
        value = float(self._cfg.get(key, default))
        if not math.isfinite(value) or value <= 0:
            raise ConfigurationError(f"{key} must be finite and positive")
        return value

    def _nonempty_str(self, key: str, default: str) -> str:
        value = str(self._cfg.get(key, default)).strip()
        if not value:
            raise ConfigurationError(f"{key} must not be empty")
        return value

    def _state_callback(self, message: Any) -> None:
        position = getattr(message, "position", ())
        if len(position) < self.ros_joint_count:
            return
        with self._state_lock:
            self._latest_state = copy.deepcopy(message)
            self._latest_state_stamp = self._monotonic()
            self._latest_joint_names = list(getattr(message, "name", ()) or ())
            self._state_event.set()

    def health(self) -> dict[str, Any]:
        with self._state_lock:
            state_received = self._latest_state is not None
            stamp = self._latest_state_stamp
            trajectory_running = self._trajectory_running
        age = None if stamp is None else max(0.0, self._monotonic() - stamp)
        return {
            "ros_initialized": True,
            "robot_state_received": state_received,
            "state_age_s": age,
            "trajectory_running": trajectory_running,
            "dof": self.arm_dof,
            "state_topic": self.state_topic,
            "command_topic": self.command_topic,
            "servo_rate_hz": self.servo_rate_hz,
            "joint_limits_configured": self._limits_configured(),
            "safe_stop_interface_available": False,
        }

    def _get_current_state(self, *, require_fresh: bool = True) -> Any:
        if not self._state_event.wait(self.state_timeout_s):
            raise RobotNotReadyError(
                f"No joint state received on {self.state_topic} within "
                f"{self.state_timeout_s:.3f}s"
            )
        with self._state_lock:
            message = copy.deepcopy(self._latest_state)
            stamp = self._latest_state_stamp
        if message is None or stamp is None:
            raise RobotNotReadyError("Joint state cache is empty")
        if require_fresh:
            age = self._monotonic() - stamp
            if age > self.max_state_age_s:
                raise RobotNotReadyError(
                    f"Joint state is stale ({age:.3f}s > {self.max_state_age_s:.3f}s)"
                )
        return message

    def get_joint_positions(self) -> list[float]:
        state = self._get_current_state()
        q = np.asarray(state.position[: self.arm_dof], dtype=np.float64)
        if q.shape != (self.arm_dof,) or not np.all(np.isfinite(q)):
            raise RobotNotReadyError("Received invalid arm joint state")
        return q.tolist()

    def _normalised(self, name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be a finite value in [0, 1]")
        return value

    def open_gripper(self, speed: float = 1.0, force: float = 0.1) -> dict[str, Any]:
        return self._move_gripper(self.gripper_open_position, speed, force)

    def close_gripper(self, speed: float = 1.0, force: float = 0.1) -> dict[str, Any]:
        return self._move_gripper(self.gripper_closed_position, speed, force)

    def _move_gripper(
        self, target: float, speed: float, force: float
    ) -> dict[str, Any]:
        speed = self._normalised("speed", speed)
        force = self._normalised("force", force)
        if not self._motion_lock.acquire(blocking=False):
            raise MotionBusyError("Another motion command is already running")
        try:
            state = self._get_current_state()
            start = np.asarray(state.position[: self.ros_joint_count], dtype=np.float64)
            if start.shape != (self.ros_joint_count,) or not np.all(np.isfinite(start)):
                raise RobotNotReadyError("Received invalid joint/gripper state")
            target_q = start.copy()
            target_q[self.arm_dof] = float(target)
            duration = self.gripper_max_move_time_s - speed * (
                self.gripper_max_move_time_s - self.gripper_min_move_time_s
            )
            self._stop_event.clear()
            completed = self._publish_segment(start, target_q, duration)
            if not completed:
                return {"success": False, "error": "Gripper motion was stopped"}
            # Piper's ROS JointState command has no force field.  Do not pretend
            # the normalized force value changes hardware force.
            return {
                "success": True,
                "force_supported": False,
                "requested_force": force,
            }
        finally:
            self._motion_lock.release()

    def execute_joint_impedance_path(
        self,
        joint_confs: Sequence[Sequence[float]] | np.ndarray,
        joint_vels: Sequence[Sequence[float]] | np.ndarray,
        durations: Sequence[float] | np.ndarray,
    ) -> dict[str, Any]:
        if not self._motion_lock.acquire(blocking=False):
            raise MotionBusyError("Another motion command is already running")
        try:
            self._validate_trajectory(joint_confs, joint_vels, durations)
            positions = np.asarray(joint_confs, dtype=np.float64)
            velocities = np.asarray(joint_vels, dtype=np.float64)
            intervals = self._trajectory_intervals(np.asarray(durations, dtype=np.float64))
            # self._validate_limits(positions, velocities)

            current = np.asarray(self.get_joint_positions(), dtype=np.float64)
            position_error = float(np.max(np.abs(current - positions[0])))
            # if position_error > self.max_initial_error_rad:
            #     raise RobotNotReadyError(
            #         "First trajectory waypoint is too far from current position "
            #         f"({position_error:.6f} rad > {self.max_initial_error_rad:.6f} rad)"
            #     )
            self._validate_interpolated_segment_speeds(current, positions, intervals)

            self._stop_event.clear()
            with self._state_lock:
                self._trajectory_running = True
            previous = current
            for target, duration in zip(positions, intervals):
                full_target = self._full_command(target)
                full_previous = self._full_command(previous)
                if not self._publish_segment(full_previous, full_target, float(duration)):
                    return {"success": False, "error": "Trajectory execution stopped"}
                previous = target
            return {"success": True}
        finally:
            with self._state_lock:
                self._trajectory_running = False
            self._motion_lock.release()

    def _validate_trajectory(
        self,
        joint_confs: Sequence[Sequence[float]] | np.ndarray,
        joint_vels: Sequence[Sequence[float]] | np.ndarray,
        durations: Sequence[float] | np.ndarray,
    ) -> None:
        positions = np.asarray(joint_confs, dtype=np.float64)
        velocities = np.asarray(joint_vels, dtype=np.float64)
        times = np.asarray(durations, dtype=np.float64)
        if positions.ndim != 2 or positions.shape[1:] != (self.arm_dof,):
            raise ValueError(
                f"joint_confs must have shape (N, {self.arm_dof}), got {positions.shape}"
            )
        if positions.shape[0] == 0:
            raise ValueError("Trajectory must contain at least one waypoint")
        if velocities.shape != positions.shape:
            raise ValueError(
                f"joint_vels must have shape {positions.shape}, got {velocities.shape}"
            )
        if times.shape != (positions.shape[0],):
            raise ValueError(
                f"durations must have shape ({positions.shape[0]},), got {times.shape}"
            )
        if not np.all(np.isfinite(positions)):
            raise ValueError("Trajectory contains non-finite joint positions")
        if not np.all(np.isfinite(velocities)):
            raise ValueError("Trajectory contains non-finite joint velocities")
        if not np.all(np.isfinite(times)):
            raise ValueError("Trajectory contains non-finite durations")

    def _trajectory_intervals(self, durations: np.ndarray) -> np.ndarray:
        if self.trajectory_duration_mode == "intervals":
            if np.any(durations < 0):
                raise ValueError("Trajectory intervals must be non-negative")
            return durations
        if np.any(durations < 0) or np.any(np.diff(durations) < 0):
            raise ValueError("Trajectory timestamps must be non-negative and monotonic")
        return np.diff(np.concatenate((np.array([0.0]), durations)))

    def _limits_configured(self) -> bool:
        return all(
            self._cfg.get(key) is not None
            for key in (
                "joint_lower_rad",
                "joint_upper_rad",
                "max_joint_velocity_rad_s",
            )
        )

    def _limit_vector(self, key: str) -> np.ndarray:
        raw = self._cfg.get(key)
        if raw is None:
            raise ConfigurationError(
                f"{key} is not configured from verified robot/URDF/vendor data; "
                "refusing trajectory execution"
            )
        values = np.asarray(raw, dtype=np.float64)
        if values.shape != (self.arm_dof,) or not np.all(np.isfinite(values)):
            raise ConfigurationError(
                f"{key} must contain {self.arm_dof} finite values"
            )
        return values

    def _validate_limits(self, positions: np.ndarray, velocities: np.ndarray) -> None:
        lower = self._limit_vector("joint_lower_rad")
        upper = self._limit_vector("joint_upper_rad")
        max_velocity = self._limit_vector("max_joint_velocity_rad_s")
        if np.any(lower > upper):
            raise ConfigurationError("joint_lower_rad must be <= joint_upper_rad")
        if np.any(max_velocity <= 0):
            raise ConfigurationError("max_joint_velocity_rad_s must be positive")
        if np.any(positions < lower) or np.any(positions > upper):
            raise ValueError("Trajectory violates configured joint limits")
        if np.any(np.abs(velocities) > max_velocity):
            raise ValueError("Trajectory violates configured joint velocity limits")

    def _validate_interpolated_segment_speeds(
        self,
        current: np.ndarray,
        positions: np.ndarray,
        intervals: np.ndarray,
    ) -> None:
        """Check the speed actually produced by local trajectory interpolation."""
        max_velocity = self._limit_vector("max_joint_velocity_rad_s")
        previous = current
        for index, (target, duration) in enumerate(zip(positions, intervals)):
            delta = np.abs(target - previous)
            if duration == 0.0:
                if np.any(delta > 1e-12):
                    raise ValueError(
                        f"Trajectory segment {index} has non-zero motion and zero duration"
                    )
            elif np.any(delta / duration > max_velocity):
                raise ValueError(
                    f"Trajectory segment {index} exceeds configured joint velocity limits"
                )
            previous = target

    def _full_command(self, arm_q: np.ndarray) -> np.ndarray:
        state = self._get_current_state()
        full = np.asarray(state.position[: self.ros_joint_count], dtype=np.float64)
        if full.shape != (self.ros_joint_count,) or not np.all(np.isfinite(full)):
            raise RobotNotReadyError("Received invalid joint/gripper state")
        full[: self.arm_dof] = arm_q
        return full

    def _publish_segment(
        self, start: np.ndarray, target: np.ndarray, duration_s: float
    ) -> bool:
        if duration_s < 0 or not math.isfinite(duration_s):
            raise ValueError("Segment duration must be a finite non-negative value")
        if start.shape != target.shape or start.shape != (self.ros_joint_count,):
            raise ValueError("Invalid full joint command shape")
        steps = max(1, int(math.ceil(duration_s * self.servo_rate_hz)))
        start_time = self._monotonic()
        for step in range(1, steps + 1):
            if self._stop_event.is_set():
                self._publish_hold()
                return False
            alpha = float(step) / float(steps)
            self._publish_joint_command(start + (target - start) * alpha)
            deadline = start_time + float(step) / self.servo_rate_hz
            sleep_s = deadline - self._monotonic()
            if sleep_s > 0:
                self._sleep(sleep_s)
        # The last waypoint is always explicitly published, including zero-time
        # and single-waypoint trajectories.
        self._publish_joint_command(target)
        return True

    def _publish_joint_command(self, positions: np.ndarray) -> None:
        command = self._joint_state_type()
        with self._state_lock:
            existing_names = list(self._latest_joint_names)
        command.name = (
            existing_names[: self.ros_joint_count]
            if len(existing_names) >= self.ros_joint_count
            else [f"joint{i}" for i in range(self.ros_joint_count)]
        )
        command.position = [float(value) for value in positions]
        command.velocity = []
        command.effort = []
        header = getattr(command, "header", None)
        ros_time = getattr(self._ros, "Time", None)
        if header is not None and ros_time is not None and hasattr(ros_time, "now"):
            header.stamp = ros_time.now()
        self._publisher.publish(command)
        with self._state_lock:
            self._last_command = np.asarray(positions, dtype=np.float64).copy()

    def _publish_hold(self) -> None:
        with self._state_lock:
            state = copy.deepcopy(self._latest_state)
            last_command = None if self._last_command is None else self._last_command.copy()
        if state is not None:
            hold = np.asarray(state.position[: self.ros_joint_count], dtype=np.float64)
            if hold.shape == (self.ros_joint_count,) and np.all(np.isfinite(hold)):
                self._publish_joint_command(hold)
                return
        if last_command is not None:
            self._publish_joint_command(last_command)

    def stop(self) -> dict[str, Any]:
        """Stop local publication and hold the current command; not a hardware E-stop."""
        self._stop_event.set()
        self._publish_hold()
        return {
            "success": True,
            "safety_stop_interface_available": False,
            "message": (
                "Stopped bridge trajectory publication and commanded a hold. "
                "This is not a hardware emergency stop."
            ),
        }

    def close(self) -> None:
        self.stop()
        for handle in (getattr(self, "_subscriber", None), getattr(self, "_publisher", None)):
            unregister = getattr(handle, "unregister", None)
            if callable(unregister):
                unregister()
