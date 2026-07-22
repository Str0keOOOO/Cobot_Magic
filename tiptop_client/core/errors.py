"""Errors which are safe to expose through the bridge RPC protocol."""

from __future__ import annotations


class BridgeError(RuntimeError):
    """Base error with a stable RPC error code."""

    code = "EXECUTION_FAILED"
    retryable = False

    def __init__(self, message: str, *, retryable: bool | None = None) -> None:
        super().__init__(message)
        if retryable is not None:
            self.retryable = retryable


class InvalidRequestError(BridgeError):
    code = "INVALID_REQUEST"


class RobotNotReadyError(BridgeError):
    code = "ROBOT_NOT_READY"
    retryable = True


class MotionBusyError(BridgeError):
    code = "MOTION_BUSY"
    retryable = True


class CameraNotReadyError(BridgeError):
    code = "CAMERA_NOT_READY"
    retryable = True


class DepthUnavailableError(BridgeError):
    code = "DEPTH_UNAVAILABLE"


class ConfigurationError(BridgeError):
    code = "CONFIGURATION_ERROR"
