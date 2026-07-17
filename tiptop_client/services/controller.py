"""ZeroMQ controller service hosted by the Cobot Magic upper computer."""

from __future__ import annotations

import signal
from typing import Any

import numpy as np
import zmq

from ..core.errors import (
    BridgeError,
    ConfigurationError,
    MotionBusyError,
    RobotNotReadyError,
)
from ..core.protocol import (
    DEFAULT_MAX_MESSAGE_BYTES,
    PROTOCOL_VERSION,
    make_error,
    make_success,
    pack_message,
    unpack_message,
    validate_request,
)


class CobotMagicControllerServer:
    """One bounded REP endpoint for high-level robot RPCs.

    A trajectory is sent as one complete request and interpolated locally by
    ``CobotMagicRosBackend``.  It never accepts per-servo setpoints over the
    network.
    """

    SERVER_NAME = "cobot_magic_controller"
    OPERATIONS = {
        "ping",
        "health",
        "get_joint_positions",
        "open_gripper",
        "close_gripper",
        "execute_joint_impedance_path",
        "stop",
    }

    def __init__(
        self,
        backend: Any,
        bind_host: str,
        port: int,
        *,
        max_message_bytes: int = DEFAULT_MAX_MESSAGE_BYTES,
        context: zmq.Context | None = None,
    ) -> None:
        self.backend = backend
        self.bind_host = str(bind_host)
        self.port = int(port)
        self.max_message_bytes = int(max_message_bytes)
        if not self.bind_host:
            raise ValueError("bind_host must not be empty")
        if not 0 <= self.port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        if self.max_message_bytes <= 0:
            raise ValueError("max_message_bytes must be positive")
        self._context = context or zmq.Context.instance()
        self._socket: zmq.Socket | None = None
        self._stopped = False

    @property
    def endpoint(self) -> str:
        return f"tcp://{self.bind_host}:{self.port}"

    def _open_socket(self) -> zmq.Socket:
        socket = self._context.socket(zmq.REP)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, 1000)
        socket.setsockopt(zmq.SNDTIMEO, 5000)
        socket.setsockopt(zmq.MAXMSGSIZE, self.max_message_bytes)
        if self.port == 0:
            self.port = socket.bind_to_random_port(f"tcp://{self.bind_host}")
        else:
            socket.bind(self.endpoint)
        self._socket = socket
        return socket

    def serve_forever(self) -> None:
        socket = self._open_socket()
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        try:
            while not self._stopped:
                events = dict(poller.poll(1000))
                if socket not in events:
                    continue
                response: dict[str, Any]
                request_id: str | None = None
                try:
                    payload = socket.recv()
                    request = unpack_message(
                        payload, max_message_bytes=self.max_message_bytes
                    )
                    request_id = request.get("request_id")
                    response = self.handle_request(request)
                except BridgeError as exc:
                    response = make_error(
                        request_id, exc.code, str(exc), exc.retryable
                    )
                except Exception as exc:  # A REP socket must always be answered.
                    response = make_error(request_id, "EXECUTION_FAILED", str(exc))
                try:
                    socket.send(pack_message(response))
                except zmq.ZMQError:
                    if not self._stopped:
                        raise
        finally:
            poller.unregister(socket)
            socket.close(0)
            self._socket = None

    def handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        validate_request(request)
        request_id = request["request_id"]
        op = request["op"]
        params = request["params"]
        if op not in self.OPERATIONS:
            return make_error(
                request_id,
                "UNKNOWN_OPERATION",
                f"Unsupported controller operation: {op}",
            )
        try:
            result = self._dispatch(op, params)
        except MotionBusyError as exc:
            return make_error(request_id, exc.code, str(exc), exc.retryable)
        except RobotNotReadyError as exc:
            return make_error(request_id, exc.code, str(exc), exc.retryable)
        except (ValueError, TypeError, ConfigurationError) as exc:
            return make_error(request_id, "INVALID_REQUEST", str(exc))
        except BridgeError as exc:
            return make_error(request_id, exc.code, str(exc), exc.retryable)
        except Exception as exc:
            return make_error(request_id, "EXECUTION_FAILED", str(exc))
        return make_success(request_id, result)

    def _dispatch(self, op: str, params: dict[str, Any]) -> dict[str, Any]:
        if op == "ping":
            return {"server": self.SERVER_NAME, "protocol_version": PROTOCOL_VERSION}
        if op == "health":
            return {"server": self.SERVER_NAME, **self.backend.health()}
        if op == "get_joint_positions":
            q = np.asarray(self.backend.get_joint_positions(), dtype=np.float64)
            if q.shape != (int(self.backend.dof),) or not np.all(np.isfinite(q)):
                raise RobotNotReadyError("Backend returned invalid joint positions")
            return {"joint_positions": q, "unit": "rad"}
        if op in {"open_gripper", "close_gripper"}:
            self._require_only(params, {"speed", "force"})
            speed = float(params.get("speed", 1.0))
            force = float(params.get("force", 0.1))
            result = getattr(self.backend, op)(speed=speed, force=force)
            return self._command_result(result)
        if op == "execute_joint_impedance_path":
            self._require_only(params, {"joint_confs", "joint_vels", "durations"})
            missing = {"joint_confs", "joint_vels", "durations"}.difference(params)
            if missing:
                raise ValueError(f"Missing trajectory parameters: {sorted(missing)}")
            result = self.backend.execute_joint_impedance_path(
                params["joint_confs"], params["joint_vels"], params["durations"]
            )
            return self._command_result(result)
        if op == "stop":
            self._require_only(params, set())
            return self._command_result(self.backend.stop())
        raise AssertionError(f"Operation {op!r} was validated but not dispatched")

    @staticmethod
    def _require_only(params: dict[str, Any], allowed: set[str]) -> None:
        unknown = set(params).difference(allowed)
        if unknown:
            raise ValueError(f"Unexpected operation parameters: {sorted(unknown)}")

    @staticmethod
    def _command_result(result: Any) -> dict[str, Any]:
        if not isinstance(result, dict):
            raise RuntimeError("Backend command result must be a dictionary")
        if result.get("success") is not True:
            raise BridgeError(str(result.get("error", "Backend command failed")))
        return result

    def close(self) -> None:
        self._stopped = True
        close_backend = getattr(self.backend, "close", None)
        if callable(close_backend):
            close_backend()


def main() -> None:
    from ..cli import controller_server

    controller_server()


if __name__ == "__main__":
    main()
