"""ZeroMQ camera service hosted by the Cobot Magic upper computer."""

from __future__ import annotations

from typing import Any

import zmq

from ..core.errors import BridgeError, CameraNotReadyError
from ..ros.camera_bridge import (
    CameraRosBridge,
    RemoteCameraIntrinsics,
    RemoteCameraSnapshot,
)
from ..core.protocol import (
    PROTOCOL_VERSION,
    make_error,
    make_success,
    pack_message,
    unpack_message,
    validate_request,
)


class CobotMagicCameraServer:
    SERVER_NAME = "cobot_magic_camera"
    OPERATIONS = {"ping", "health", "list_cameras", "get_intrinsics", "read_camera"}

    def __init__(
        self,
        cameras: dict[str, CameraRosBridge],
        bind_host: str,
        port: int,
        *,
        max_message_bytes: int = 128 * 1024 * 1024,
        context: zmq.Context | None = None,
    ) -> None:
        self.cameras = dict(cameras)
        self.bind_host = str(bind_host)
        self.port = int(port)
        self.max_message_bytes = int(max_message_bytes)
        if not self.cameras:
            raise ValueError("At least one camera must be configured")
        if not self.bind_host or not 0 <= self.port <= 65535:
            raise ValueError("Invalid camera server bind host or port")
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
                if socket not in dict(poller.poll(1000)):
                    continue
                request_id: str | None = None
                try:
                    request = unpack_message(
                        socket.recv(), max_message_bytes=self.max_message_bytes
                    )
                    request_id = request.get("request_id")
                    response = self.handle_request(request)
                except BridgeError as exc:
                    response = make_error(request_id, exc.code, str(exc), exc.retryable)
                except Exception as exc:
                    response = make_error(request_id, "EXECUTION_FAILED", str(exc))
                socket.send(pack_message(response))
        finally:
            poller.unregister(socket)
            socket.close(0)
            self._socket = None

    def handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        try:
            validate_request(request)
        except BridgeError as exc:
            return make_error(request.get("request_id"), exc.code, str(exc))
        request_id = request["request_id"]
        op = request["op"]
        params = request["params"]
        if op not in self.OPERATIONS:
            return make_error(request_id, "UNKNOWN_OPERATION", f"Unsupported camera operation: {op}")
        try:
            result = self._dispatch(op, params)
        except CameraNotReadyError as exc:
            return make_error(request_id, exc.code, str(exc), exc.retryable)
        except (ValueError, TypeError) as exc:
            return make_error(request_id, "INVALID_REQUEST", str(exc))
        except BridgeError as exc:
            return make_error(request_id, exc.code, str(exc), exc.retryable)
        except Exception as exc:
            return make_error(request_id, "EXECUTION_FAILED", str(exc))
        return make_success(request_id, result)

    def _dispatch(self, op: str, params: dict[str, Any]) -> dict[str, Any]:
        if op == "ping":
            self._require_only(params, set())
            return {"server": self.SERVER_NAME, "protocol_version": PROTOCOL_VERSION}
        if op == "health":
            self._require_only(params, set())
            return {
                "server": self.SERVER_NAME,
                "cameras": [camera.health() for camera in self.cameras.values()],
            }
        if op == "list_cameras":
            self._require_only(params, set())
            return {
                "cameras": [
                    {
                        "namespace": camera.namespace,
                        "serial": camera.serial,
                        "role": camera.role,
                    }
                    for camera in self.cameras.values()
                ]
            }
        if op in {"get_intrinsics", "read_camera"}:
            self._require_only(params, {"serial"})
            serial = params.get("serial")
            if not isinstance(serial, str) or not serial:
                raise ValueError("serial must be a non-empty string")
            camera = self._camera_by_serial(serial)
            if op == "get_intrinsics":
                # Calibration is cached from CameraInfo/TF and intentionally
                # does not wait for a synchronized image triplet.
                return self._intrinsics_result(camera.read_intrinsics())
            return self._snapshot_result(camera.read_snapshot())
        raise AssertionError(op)

    @staticmethod
    def _require_only(params: dict[str, Any], allowed: set[str]) -> None:
        unknown = set(params).difference(allowed)
        if unknown:
            raise ValueError(f"Unexpected operation parameters: {sorted(unknown)}")

    def _camera_by_serial(self, serial: str) -> CameraRosBridge:
        for camera in self.cameras.values():
            if camera.serial == serial:
                return camera
        raise ValueError(f"Unknown camera serial: {serial}")

    @staticmethod
    def _intrinsics_result(intrinsics: RemoteCameraIntrinsics) -> dict[str, Any]:
        return {
            "serial": intrinsics.serial,
            "K_color": intrinsics.K_color,
            "distortion_color": intrinsics.distortion_color,
            "K_ir": intrinsics.K_ir,
            "baseline_ir": intrinsics.baseline_ir,
            "T_color_from_ir": intrinsics.T_color_from_ir,
        }

    @staticmethod
    def _snapshot_result(snapshot: RemoteCameraSnapshot) -> dict[str, Any]:
        result = {
            "serial": snapshot.serial,
            "timestamp": snapshot.timestamp,
            "rgb": snapshot.rgb,
            "ir_left": snapshot.ir_left,
            "ir_right": snapshot.ir_right,
        }
        if snapshot.depth is not None and snapshot.depth_raw is not None:
            result["depth"] = snapshot.depth
            result["depth_raw"] = snapshot.depth_raw
        return result

    def close(self) -> None:
        self._stopped = True
        for camera in self.cameras.values():
            camera.close()


def main() -> None:
    from ..cli import camera_server

    camera_server()


if __name__ == "__main__":
    main()
