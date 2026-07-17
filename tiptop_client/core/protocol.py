"""Versioned MessagePack RPC messages shared with the GPU-side client.

This module deliberately contains no ROS imports, so it can be imported by
unit tests and by the server-side client implementation.
"""

from __future__ import annotations

from typing import Any

import msgpack
import msgpack_numpy

from .errors import InvalidRequestError

msgpack_numpy.patch()

PROTOCOL_VERSION = "1.0"
DEFAULT_MAX_MESSAGE_BYTES = 64 * 1024 * 1024


def pack_message(message: dict[str, Any]) -> bytes:
    """Encode a message without using pickle or executable payloads."""
    if not isinstance(message, dict):
        raise TypeError("RPC message must be a dictionary")
    return msgpack.packb(message, use_bin_type=True, default=msgpack_numpy.encode)


def unpack_message(
    payload: bytes, *, max_message_bytes: int = DEFAULT_MAX_MESSAGE_BYTES
) -> dict[str, Any]:
    """Decode one bounded MessagePack dictionary."""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("RPC payload must be bytes")
    if len(payload) > max_message_bytes:
        raise InvalidRequestError(
            f"RPC payload exceeds {max_message_bytes} byte limit"
        )
    value = msgpack.unpackb(
        payload,
        raw=False,
        object_hook=msgpack_numpy.decode,
        strict_map_key=False,
    )
    if not isinstance(value, dict):
        raise InvalidRequestError("RPC message must decode to a dictionary")
    return value


def validate_request(request: dict[str, Any]) -> None:
    """Validate the invariant request envelope; unknown fields are ignored."""
    if not isinstance(request, dict):
        raise InvalidRequestError("Request must be a dictionary")
    if request.get("protocol_version") != PROTOCOL_VERSION:
        raise InvalidRequestError(
            "Unsupported protocol_version "
            f"{request.get('protocol_version')!r}; expected {PROTOCOL_VERSION!r}"
        )
    request_id = request.get("request_id")
    if not isinstance(request_id, str) or not request_id.strip():
        raise InvalidRequestError("request_id must be a non-empty string")
    op = request.get("op")
    if not isinstance(op, str) or not op.strip():
        raise InvalidRequestError("op must be a non-empty string")
    if not isinstance(request.get("params"), dict):
        raise InvalidRequestError("params must be a dictionary")


def make_success(request_id: str, result: Any) -> dict[str, Any]:
    return {
        "protocol_version": PROTOCOL_VERSION,
        "request_id": str(request_id),
        "success": True,
        "result": result,
        "error": None,
    }


def make_error(
    request_id: str | None,
    code: str,
    message: str,
    retryable: bool = False,
) -> dict[str, Any]:
    return {
        "protocol_version": PROTOCOL_VERSION,
        "request_id": request_id if isinstance(request_id, str) else "",
        "success": False,
        "result": None,
        "error": {
            "code": str(code),
            "message": str(message),
            "retryable": bool(retryable),
        },
    }
