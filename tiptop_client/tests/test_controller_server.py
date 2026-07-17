from __future__ import annotations

import unittest
import uuid
import threading
import time
import os

import numpy as np
import zmq

from tiptop_client.core.protocol import PROTOCOL_VERSION, pack_message, unpack_message
from tiptop_client.services.controller import CobotMagicControllerServer

from .fakes import FakeBackend


def request(op, params=None):
    return {
        "protocol_version": PROTOCOL_VERSION,
        "request_id": str(uuid.uuid4()),
        "op": op,
        "params": {} if params is None else params,
    }


class ControllerServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = FakeBackend()
        self.server = CobotMagicControllerServer(self.backend, "127.0.0.1", 5555)

    def test_health_and_joint_positions(self) -> None:
        response = self.server.handle_request(request("health"))
        self.assertTrue(response["success"])
        response = self.server.handle_request(request("get_joint_positions"))
        np.testing.assert_array_equal(response["result"]["joint_positions"], np.zeros(6))

    def test_unknown_op_and_bad_shape_are_answered(self) -> None:
        response = self.server.handle_request(request("does-not-exist"))
        self.assertFalse(response["success"])
        self.assertEqual(response["error"]["code"], "UNKNOWN_OPERATION")
        response = self.server.handle_request(
            request("execute_joint_impedance_path", {"joint_confs": np.zeros((1, 6))})
        )
        self.assertFalse(response["success"])
        self.assertEqual(response["error"]["code"], "INVALID_REQUEST")

    def test_invalid_protocol_is_answered(self) -> None:
        malformed = request("ping")
        malformed["protocol_version"] = "2.0"
        with self.assertRaises(Exception):
            self.server.handle_request(malformed)

    @unittest.skipUnless(
        os.environ.get("RUN_ZMQ_SOCKET_TESTS") == "1",
        "requires TCP socket permission; enable on the upper computer",
    )
    def test_rep_socket_always_replies_to_protocol_error(self) -> None:
        server = CobotMagicControllerServer(FakeBackend(), "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        deadline = time.monotonic() + 1.0
        while server.port == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertNotEqual(server.port, 0)
        context = zmq.Context.instance()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, 1_000)
        try:
            socket.connect(server.endpoint)
            bad = request("ping")
            bad["protocol_version"] = "wrong"
            socket.send(pack_message(bad))
            response = unpack_message(socket.recv())
            self.assertFalse(response["success"])
            self.assertEqual(response["error"]["code"], "INVALID_REQUEST")
        finally:
            socket.close(0)
            server.close()
            thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
