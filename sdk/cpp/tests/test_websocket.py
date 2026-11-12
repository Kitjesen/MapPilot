"""Exercise the installed C++ WebSocket facade against a real WebSocket peer."""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import unittest
from contextlib import contextmanager
from http import HTTPStatus

from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

PROBE = sys.argv.pop(1)


@contextmanager
def peer(handler, *, authorize_camera=False):
    connections = []
    failures = []

    def inspect(connection, request):
        connections.append((request.path, {key.lower(): value for key, value in request.headers.raw_items()}))
        if authorize_camera and request.headers.get("Authorization") != "Bearer viewer-secret":
            return connection.respond(HTTPStatus.FORBIDDEN, "media_authorization_denied")
        return None

    def handle(connection):
        try:
            handler(connection)
        except ConnectionClosed:
            pass
        except Exception as error:
            failures.append(error)

    with serve(handle, "127.0.0.1", 0, process_request=inspect, close_timeout=0.2, compression=None) as server:
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            yield f"http://127.0.0.1:{server.socket.getsockname()[1]}", connections
        finally:
            server.shutdown()
            thread.join(timeout=5)
    if failures:
        raise failures[0]


def run(url, action):
    return subprocess.run([PROBE, url, action], capture_output=True, timeout=15)


class WebSocketTests(unittest.TestCase):
    def test_fragmented_binary_with_ping(self):
        payload = b"\x00\xffcloud\n\x00"

        def handler(socket):
            socket.ping(b"probe")
            socket.send([payload[:3], payload[3:]])
            socket.recv()

        with peer(handler) as (url, connections):
            result = run(url, "cloud")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout, payload)
        self.assertEqual(connections[0][0], "/ws/cloud")
        self.assertEqual(connections[0][1]["x-api-key"], "test-key")

    def test_large_preview_is_preserved(self):
        payload = b"x" * (10 * 1024 * 1024 + 1)

        def handler(socket):
            socket.send(payload)
            socket.recv()

        with peer(handler) as (url, _):
            result = run(url, "cloud")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, payload)

    def test_clean_peer_close(self):
        def handler(socket):
            socket.send(b"last")
            socket.close()

        with peer(handler) as (url, _):
            result = run(url, "feed_close")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout, b"last")

    def test_camera_bearer_is_separate_from_api_key(self):
        def handler(socket):
            socket.send(b"\xff\xd8camera\x00\xff\xd9")
            socket.recv()

        with peer(handler, authorize_camera=True) as (url, connections):
            result = run(url, "camera")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(connections[0][0], "/ws/camera")
        self.assertEqual(connections[0][1]["authorization"], "Bearer viewer-secret")

    def test_handshake_error_preserves_http_status(self):
        with peer(lambda socket: None, authorize_camera=True) as (url, _):
            result = run(url, "cloud")
        self.assertEqual(result.returncode, 2)
        self.assertTrue(result.stdout.startswith(b"403\n"), result.stdout)

    def test_teleop_windows_and_receipts(self):
        received = []

        def handler(socket):
            window = None
            sequence = 0
            for raw in socket:
                message = json.loads(raw)
                received.append(message)
                identity = message["request_id"]
                if message["type"] == "input_request":
                    sequence += 1
                    window = str(sequence)
                    socket.send(json.dumps({"type": "input_ack", "request_id": identity, "input_window": window}))
                elif message.get("deadman") is True:
                    self.assertEqual(message["input_window"], window)
                    window = None
                    socket.send(json.dumps({
                        "type": "ingress_ack", "request_id": identity, "ingress_accepted": True,
                        "stage": "gateway_queue_accepted", "final_cmd_vel_confirmed": False,
                        "motor_confirmed": False, "input_window": "unused-next-window",
                    }))
                else:
                    socket.send(json.dumps({"type": "control_ack", "request_id": identity, "accepted": True}))

        with peer(handler) as (url, connections):
            result = run(url, "teleop_twice")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        replies = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([reply["request_id"] for reply in replies], ["move-1", "move-2", "hold-1"])
        self.assertEqual(replies[0]["stage"], "gateway_queue_accepted")
        self.assertFalse(replies[0]["final_cmd_vel_confirmed"])
        samples = [item for item in received if item["type"] == "velocity" and item.get("deadman")]
        self.assertEqual(len(samples), 2)
        self.assertNotEqual(samples[0]["input_window"], samples[1]["input_window"])
        self.assertEqual([sample["manual_mode"] for sample in samples], [False, True])
        self.assertEqual(samples[0]["vx_mps"], 0.25)
        self.assertEqual(samples[0]["vy_mps"], -0.1)
        self.assertEqual(samples[0]["yaw_rps"], 0.3)
        self.assertEqual(connections[0][0], "/ws/teleop?client_id=cpp-client")

    def test_teleop_rejection_is_retained(self):
        received = []

        def handler(socket):
            for raw in socket:
                message = json.loads(raw)
                received.append(message)
                if message["type"] == "input_request":
                    socket.send(json.dumps({"type": "input_ack", "request_id": message["request_id"], "input_window": "one"}))
                else:
                    socket.send(json.dumps({"type": "control_rejected", "request_id": message["request_id"],
                                            "error": "control_unavailable"}))

        with peer(handler) as (url, _):
            result = run(url, "teleop")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(result.stdout.splitlines()[0])["type"], "control_rejected")
        self.assertEqual(sum(item.get("deadman") is True for item in received), 1)

    def test_early_session_rejection(self):
        received = []

        def handler(socket):
            socket.send(json.dumps({"type": "control_rejected", "error": "control_in_use"}))
            for raw in socket:
                received.append(json.loads(raw))

        with peer(handler) as (url, _):
            result = run(url, "teleop")
        self.assertEqual(result.returncode, 2)
        self.assertIn(b"control_in_use", result.stdout)
        self.assertFalse(any(item["type"] == "velocity" for item in received))


if __name__ == "__main__":
    unittest.main()
