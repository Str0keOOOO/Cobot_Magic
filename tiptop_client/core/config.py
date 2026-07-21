"""Configuration loading and conservative defaults for the upper computer."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml

from .errors import ConfigurationError


DEFAULT_CONFIG: dict[str, Any] = {
    "controller": {
        "bind_host": "127.0.0.1",
        "port": 5555,
        "protocol_version": "1.0",
        "max_message_bytes": 64 * 1024 * 1024,
    },
    "robot": {
        "state_topic": "/puppet/joint_right",
        "command_topic": "/master/joint_right",
        "arm_dof": 6,
        "ros_joint_count": 7,
        # Piper's installed ROS driver publishes its state loop at 200 Hz.
        "servo_rate_hz": 200.0,
        "state_timeout_s": 5.0,
        "max_state_age_s": 0.25,
        "gripper_open_position": 0.08,
        "gripper_closed_position": 0.0,
        "gripper_min_move_time_s": 0.25,
        "gripper_max_move_time_s": 1.5,
        # "intervals": durations[i] is the duration ending at waypoint i.
        # "timestamps": durations[i] is a cumulative timestamp from start.
        "trajectory_duration_mode": "intervals",
    },
    "camera_server": {
        "bind_host": "127.0.0.1",
        "port": 5556,
        "max_message_bytes": 128 * 1024 * 1024,
        "max_snapshot_age_s": 0.25,
        "sync_queue_size": 10,
        "sync_slop_s": 0.05,
        "tf_timeout_s": 0.2,
        "baseline_consistency_tolerance_m": 0.005,
    },
    "cameras": {},
}


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: str | Path) -> dict[str, Any]:
    """Load YAML, merge defaults, and reject malformed top-level sections."""
    config_path = Path(path).expanduser()
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
    except OSError as exc:
        raise ConfigurationError(f"Cannot read configuration {config_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"Invalid YAML in {config_path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise ConfigurationError("Bridge configuration must be a YAML mapping")
    config = _merge(deepcopy(DEFAULT_CONFIG), loaded)
    for section in ("controller", "robot", "camera_server", "cameras"):
        if not isinstance(config.get(section), dict):
            raise ConfigurationError(f"Configuration section {section!r} must be a mapping")
    return config


def require_mapping(config: dict[str, Any], key: str) -> dict[str, Any]:
    value = config.get(key)
    if not isinstance(value, dict):
        raise ConfigurationError(f"Missing mapping configuration: {key}")
    return value
