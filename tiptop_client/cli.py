"""Command-line entry points executed on the robot upper computer."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import sys
import uuid
from typing import Any

import zmq

from .core.config import load_config
from .core.protocol import PROTOCOL_VERSION, pack_message, unpack_message
from .ros.arm_backend import CobotMagicRosBackend
from .ros.camera_bridge import CameraRosBridge
from .services.camera import CobotMagicCameraServer
from .services.controller import CobotMagicControllerServer


def _default_config_path() -> str:
    return str(Path(__file__).resolve().parent / "config.yaml")


def _server_parser(description: str, default_port: int) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument("--bind-host", default=None)
    parser.add_argument("--port", type=int, default=None)
    return parser


def controller_server(argv: list[str] | None = None) -> None:
    args = _server_parser("Start the Cobot Magic controller RPC service", 5555).parse_args(argv)
    config = load_config(args.config)
    section = config["controller"]
    backend = CobotMagicRosBackend(config)
    server = CobotMagicControllerServer(
        backend,
        args.bind_host or section["bind_host"],
        args.port if args.port is not None else section["port"],
        max_message_bytes=section["max_message_bytes"],
    )
    _run_server(server, "controller")


def camera_server(argv: list[str] | None = None) -> None:
    args = _server_parser("Start the Cobot Magic camera RPC service", 5556).parse_args(argv)
    config = load_config(args.config)
    section = config["camera_server"]
    cameras = {
        role: CameraRosBridge(role, camera_cfg, section)
        for role, camera_cfg in config["cameras"].items()
        if isinstance(camera_cfg, dict) and camera_cfg.get("enabled", True)
    }
    server = CobotMagicCameraServer(
        cameras,
        args.bind_host or section["bind_host"],
        args.port if args.port is not None else section["port"],
        max_message_bytes=section["max_message_bytes"],
    )
    _run_server(server, "camera")


def _run_server(server: Any, name: str) -> None:
    def stop_handler(_signum: int, _frame: Any) -> None:
        server.close()

    previous_int = signal.signal(signal.SIGINT, stop_handler)
    previous_term = signal.signal(signal.SIGTERM, stop_handler)
    try:
        print(f"Starting Cobot Magic {name} server on {server.endpoint}", flush=True)
        server.serve_forever()
    finally:
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
        server.close()


def _health_request(host: str, port: int, op: str, *, serial: str | None = None) -> dict[str, Any]:
    context = zmq.Context.instance()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, 5_000)
    socket.setsockopt(zmq.RCVTIMEO, 5_000)
    try:
        socket.connect(f"tcp://{host}:{port}")
        request = {
            "protocol_version": PROTOCOL_VERSION,
            "request_id": str(uuid.uuid4()),
            "op": op,
            "params": {} if serial is None else {"serial": serial},
        }
        socket.send(pack_message(request))
        response = unpack_message(socket.recv())
    except zmq.Again as exc:
        raise RuntimeError(f"Timed out connecting to tcp://{host}:{port}") from exc
    finally:
        socket.close(0)
    if response.get("request_id") != request["request_id"]:
        raise RuntimeError("Health response request_id did not match")
    if response.get("protocol_version") != PROTOCOL_VERSION:
        raise RuntimeError("Health response protocol version did not match")
    if response.get("success") is not True:
        error = response.get("error") or {}
        raise RuntimeError(f"{error.get('code', 'REMOTE_ERROR')}: {error.get('message')}")
    result = response.get("result")
    if not isinstance(result, dict):
        raise RuntimeError("Health response result must be a dictionary")
    return result


def health(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Check controller RPC health")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=int)
    args = parser.parse_args(argv)
    print(json.dumps(_health_request(args.host, args.port, "health"), indent=2, default=_json_default))


def camera_health(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Check camera RPC health")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--serial", default=None)
    args = parser.parse_args(argv)
    op = "get_intrinsics" if args.serial else "health"
    print(json.dumps(_health_request(args.host, args.port, op, serial=args.serial), indent=2, default=_json_default))


def controller_health_local(argv: list[str] | None = None) -> None:
    """Console-script health command, named to avoid a server-side ambiguity."""
    health(argv)


def robot_local_check(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Read, but never command, local robot state")
    parser.add_argument("--config", default=_default_config_path())
    args = parser.parse_args(argv)
    backend = CobotMagicRosBackend(load_config(args.config))
    try:
        result = {"health": backend.health(), "joint_positions_rad": backend.get_joint_positions()}
        print(json.dumps(result, indent=2, default=_json_default))
    finally:
        backend.close()


def _json_default(value: Any) -> Any:
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return tolist()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Cobot Magic TiPToP upper-computer client")
    subcommands = parser.add_subparsers(dest="command", required=True)
    for name in ("health", "camera-health", "robot-local-check", "controller-server", "camera-server"):
        subcommands.add_parser(name)
    args, remaining = parser.parse_known_args(argv)
    commands = {
        "health": health,
        "camera-health": camera_health,
        "robot-local-check": robot_local_check,
        "controller-server": controller_server,
        "camera-server": camera_server,
    }
    commands[args.command](remaining)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
