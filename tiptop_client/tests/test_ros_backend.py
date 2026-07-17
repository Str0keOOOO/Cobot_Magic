from __future__ import annotations

import unittest

import numpy as np

from tiptop_client.core.errors import ConfigurationError, MotionBusyError, RobotNotReadyError
from tiptop_client.ros.arm_backend import CobotMagicRosBackend

from .fakes import FakeJointState, FakeRos


def safe_config():
    return {
        "robot": {
            "state_topic": "/puppet/joint_right",
            "command_topic": "/master/joint_right",
            "arm_dof": 6,
            "ros_joint_count": 7,
            "servo_rate_hz": 200.0,
            "state_timeout_s": 0.01,
            "max_state_age_s": 1.0,
            "max_initial_error_rad": 0.017453292519943295,
            "gripper_open_position": 0.08,
            "gripper_closed_position": 0.0,
            "gripper_min_move_time_s": 0.01,
            "gripper_max_move_time_s": 0.01,
            "joint_lower_rad": [-1.0] * 6,
            "joint_upper_rad": [1.0] * 6,
            "max_joint_velocity_rad_s": [1.0] * 6,
            "trajectory_duration_mode": "intervals",
        }
    }


class RosBackendTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ros = FakeRos()
        self.backend = CobotMagicRosBackend(
            safe_config(), ros_api=self.ros, joint_state_type=FakeJointState, sleep=lambda _: None
        )
        self.backend._state_callback(
            FakeJointState(position=[0.0] * 7, name=[f"joint{i}" for i in range(7)])
        )

    def test_joint_positions_exclude_gripper(self) -> None:
        self.assertEqual(self.backend.get_joint_positions(), [0.0] * 6)
        self.assertEqual(self.backend.health()["servo_rate_hz"], 200.0)

    def test_first_waypoint_uses_max_absolute_error(self) -> None:
        q = np.zeros((1, 6))
        q[0, 0] = 0.02
        with self.assertRaises(RobotNotReadyError):
            self.backend.execute_joint_impedance_path(q, np.zeros_like(q), np.array([0.01]))

    def test_nonfinite_and_limits_are_rejected(self) -> None:
        q = np.zeros((1, 6))
        q[0, 1] = np.nan
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.backend.execute_joint_impedance_path(q, np.zeros_like(q), np.array([0.0]))
        q = np.full((1, 6), 1.1)
        with self.assertRaisesRegex(ValueError, "joint limits"):
            self.backend.execute_joint_impedance_path(q, np.zeros_like(q), np.array([0.0]))

    def test_final_waypoint_is_published(self) -> None:
        q = np.array([[0.0] * 6, [0.01] * 6])
        result = self.backend.execute_joint_impedance_path(q, np.zeros_like(q), np.array([0.0, 0.01]))
        self.assertTrue(result["success"])
        self.assertEqual(self.ros.publisher.messages[-1].position[:6], [0.01] * 6)

    def test_interpolation_speed_is_limited_by_duration(self) -> None:
        q = np.array([[0.01] + [0.0] * 5])
        with self.assertRaisesRegex(ValueError, "segment 0 exceeds"):
            self.backend.execute_joint_impedance_path(q, np.zeros_like(q), np.array([0.001]))

    def test_trajectory_requires_verified_limits(self) -> None:
        config = safe_config()
        config["robot"]["joint_lower_rad"] = None
        backend = CobotMagicRosBackend(
            config, ros_api=FakeRos(), joint_state_type=FakeJointState, sleep=lambda _: None
        )
        backend._state_callback(FakeJointState(position=[0.0] * 7))
        with self.assertRaises(ConfigurationError):
            backend.execute_joint_impedance_path(np.zeros((1, 6)), np.zeros((1, 6)), np.array([0.0]))

    def test_motion_lock_and_gripper_normalization(self) -> None:
        self.backend._motion_lock.acquire()
        try:
            with self.assertRaises(MotionBusyError):
                self.backend.open_gripper()
        finally:
            self.backend._motion_lock.release()
        with self.assertRaisesRegex(ValueError, r"\[0, 1\]"):
            self.backend.close_gripper(speed=1.2)

    def test_stop_is_explicitly_not_hardware_estop(self) -> None:
        result = self.backend.stop()
        self.assertTrue(result["success"])
        self.assertFalse(result["safety_stop_interface_available"])
