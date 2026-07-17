from __future__ import annotations

import unittest

import numpy as np

from tiptop_client.core.errors import InvalidRequestError
from tiptop_client.core.protocol import (
    PROTOCOL_VERSION,
    make_error,
    make_success,
    pack_message,
    unpack_message,
    validate_request,
)


class ProtocolTest(unittest.TestCase):
    def test_numpy_round_trip(self) -> None:
        message = {
            "protocol_version": PROTOCOL_VERSION,
            "request_id": "test-id",
            "op": "read_camera",
            "params": {"image": np.arange(12, dtype=np.uint8).reshape(2, 2, 3)},
        }
        decoded = unpack_message(pack_message(message))
        np.testing.assert_array_equal(decoded["params"]["image"], message["params"]["image"])

    def test_invalid_request_envelope_is_rejected(self) -> None:
        with self.assertRaises(InvalidRequestError):
            validate_request({"protocol_version": "0", "request_id": "x", "op": "ping", "params": {}})
        with self.assertRaises(InvalidRequestError):
            validate_request({"protocol_version": PROTOCOL_VERSION, "request_id": "", "op": "ping", "params": {}})
        with self.assertRaises(InvalidRequestError):
            validate_request({"protocol_version": PROTOCOL_VERSION, "request_id": "x", "op": "ping", "params": []})

    def test_success_and_error_shape(self) -> None:
        self.assertEqual(make_success("abc", {"ok": 1})["error"], None)
        error = make_error("abc", "ROBOT_NOT_READY", "state missing", True)
        self.assertFalse(error["success"])
        self.assertEqual(error["error"]["code"], "ROBOT_NOT_READY")
        self.assertTrue(error["error"]["retryable"])
