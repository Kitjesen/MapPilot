"""Exercise the compiled C++ facade and Rust DLL against local HTTP exchanges."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import threading
import unittest
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

PROBE = sys.argv.pop(1)


@contextmanager
def server(responses=None):
    calls = []
    responses = list(responses or [(200, b'{"ok":true}')])

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def handle_request(self):
            raw = self.rfile.read(int(self.headers.get("Content-Length", "0")))
            calls.append((self.command, self.path, {key.lower(): value for key, value in self.headers.items()},
                          json.loads(raw) if raw else None))
            response = responses[min(len(calls) - 1, len(responses) - 1)]
            status, body = response[:2]
            self.send_response(status)
            self.send_header("Content-Length", str(response[2] if len(response) > 2 else len(body)))
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)

        do_GET = do_POST = do_PUT = do_DELETE = handle_request

    http = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=http.serve_forever)
    thread.start()
    try:
        yield f"http://127.0.0.1:{http.server_port}", calls
    finally:
        http.shutdown()
        http.server_close()
        thread.join()


def run(url, action="health", *args):
    environment = os.environ.copy()
    bypass = environment.get("NO_PROXY", environment.get("no_proxy", ""))
    environment["NO_PROXY"] = ",".join(filter(None, [bypass, "127.0.0.1", "localhost"]))
    return subprocess.run([PROBE, url, action, *map(str, args)], capture_output=True, timeout=15, env=environment)


class ClientTests(unittest.TestCase):
    def test_stop_resume_and_request_reconciliation(self):
        cases = [
            ("stop_product", "/api/v1/product-control/stop", {"expected_product_session_id": "session-1", "request_id": "stop-1"}),
            ("resume_motion", "/api/v1/navigation/resume", {"request_id": "resume-1", "client_id": "operator"}),
            ("stop_recording_session", "/api/v1/recordings/stop", {"expected_session_id": "recording-1"}),
            ("goal_status", "/api/v1/navigation/goals/goal%2Fa%3F", None),
        ]
        payload = {"state": "running", "previous_motion_restored": False, "found": False}
        for action, path, body in cases:
            with self.subTest(action=action), server([(200, json.dumps(payload).encode())]) as (url, calls):
                result = run(url, action)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(calls[0][1], path)
            self.assertEqual(calls[0][3], body)
            returned = json.loads(result.stdout)
            for field, expected in payload.items():
                self.assertEqual(returned[field], expected)

    def test_product_operation_retains_owner_fields(self):
        payload = {"state": "running", "request_id": "switch-1", "result": None}
        with server([(202, json.dumps(payload).encode())]) as (url, calls):
            result = run(url, "product")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(result.stdout), payload)
        self.assertEqual([call[1] for call in calls], [
            "/api/v1/product-control", "/api/v1/product-control/switch",
            "/api/v1/product-control/operations/switch%2Fa%3F",
        ])
        self.assertEqual(calls[1][3], {
            "product": "nav", "expected_product_session_id": "session-1",
            "map_name": "hall", "request_id": "switch-1",
            "initial_pose": {"x": 1, "y": 2, "z": 0, "yaw": 0},
        })

    def test_recording_management_and_binary_downloads(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "recording.tar"
            with server() as (url, calls):
                result = run(url, "recordings", target)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(target.read_bytes(), b'{"ok":true}')
        self.assertEqual(len(calls), 13)
        self.assertEqual(calls[0][1], "/api/v1/recordings?limit=7")
        self.assertEqual(parse_qs(urlsplit(calls[5][1]).query), {"field": ["pose.x"], "topic": ["/nav/state"]})
        self.assertEqual(parse_qs(urlsplit(calls[7][1]).query), {"path_topic": ["/path/a", "/path/b"]})
        self.assertEqual(parse_qs(urlsplit(calls[8][1]).query), {"t": ["1.5"], "segment": ["2"]})
        self.assertEqual(calls[9][1], "/api/v1/recordings/record-1/files/camera/front%20%2B%2Emkv")
        self.assertEqual(calls[10][1], "/api/v1/recordings/record-1/archive")
        self.assertEqual(calls[11][:2], ("POST", "/api/v1/recordings/record-1/validate"))
        self.assertEqual(calls[12][:2], ("DELETE", "/api/v1/recordings/record-1"))

    def test_inspection_edit_review_and_download(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "evidence.bin"
            with server() as (url, calls):
                result = run(url, "inspection", target)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(target.read_bytes(), b'{"ok":true}')
        self.assertEqual(len(calls), 9)
        self.assertEqual(parse_qs(urlsplit(calls[0][1]).query), {"map_id": ["hall +"]})
        self.assertEqual(calls[1][3]["revision"], 2)
        self.assertEqual(set(calls[1][3]), {"id", "map_id", "revision", "points"})
        self.assertEqual(calls[2][0], "DELETE")
        self.assertEqual(calls[6][3], {"verdict": "anomaly", "note": "glass\nwall"})
        self.assertEqual(calls[7][1], "/api/v1/inspection/evidence/evidence-1/artifacts/rgb")
        self.assertEqual(calls[8][1], "/api/v1/inspection/evidence/evidence-1/measurements/sensor%20%2B")

    def test_goal_identity_and_coordinates(self):
        with server([(202, b'{"accepted":true,"task_id":"same-task"}')]) as (url, calls):
            result = run(url, "goal")
        self.assertEqual(result.returncode, 0, result.stderr)
        body = calls[0][3]
        self.assertEqual((body["x"], body["y"], body["z"]), (1, 2, 3))
        self.assertEqual(body["request_id"], "same-request")
        self.assertEqual(json.loads(result.stdout)["request_id"], "same-request")
        self.assertEqual(calls[0][2]["x-api-key"], "test-key")

    def test_strict_request_fields(self):
        cases = {
            "save": ("/api/v1/map/save", {"name", "auto_process", "request_id"}),
            "reoptimize": ("/api/v1/maps/operations/save_1/reoptimize", {"request_id"}),
            "relocalize": ("/api/v1/localization/relocalizations", {"mode", "map_name", "initial_pose", "request_id"}),
            "validate": ("/api/v1/maps/hall/validate_plan", {"x", "y", "z"}),
            "no_go": ("/api/v1/maps/hall/no_go", {"region", "expected_content_epoch"}),
            "profile": ("/api/v1/motion/profile", {"name", "request_id", "client_id"}),
            "embedding": ("/api/v1/memory/temporal/semantic", {"embedding", "top_k", "label"}),
            "rename": ("/api/v1/map/rename", {"old_name", "new_name"}),
            "reset": ("/api/v1/map_cloud/reset", set()),
        }
        for action, (path, fields) in cases.items():
            with self.subTest(action=action), server() as (url, calls):
                result = run(url, action)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(calls[0][1], path)
            self.assertEqual(set(calls[0][3]), fields)

    def test_encoded_path_and_query(self):
        with server() as (url, calls):
            result = run(url, "follow")
        self.assertEqual(result.returncode, 0, result.stderr)
        path = urlsplit(calls[0][1])
        self.assertEqual(path.path, "/api/v1/follow/task%2Fa%3F")
        self.assertEqual(parse_qs(path.query), {"product_session_id": ["session +&?"]})
        with server() as (url, calls):
            self.assertEqual(run(url, "delete_no_go").returncode, 0)
        self.assertEqual(calls[0][0], "DELETE")
        self.assertEqual(parse_qs(urlsplit(calls[0][1]).query), {"expected_content_epoch": ["12"]})

    def test_json_strings_are_escaped(self):
        with server() as (url, calls):
            self.assertEqual(run(url, "instruction").returncode, 0)
        self.assertEqual(calls[0][3]["text"], 'line\n"quoted"')

    def test_http_status_and_reason_are_retained(self):
        body = b'{"detail":{"reason_code":"content_epoch_mismatch"}}'
        with server([(409, body)]) as (url, _):
            result = run(url)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, b"409\n" + body)

    def test_binary_snapshot_keeps_nul(self):
        body = b"\xff\xd8\x00bytes\nraw\xff\xd9"
        with server([(200, body)]) as (url, _):
            result = run(url, "snapshot")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, body)

    def test_snapshot_uses_constructor_media_token(self):
        with server([(200, b"jpeg")]) as (url, calls):
            result = run(url, "media_http")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(calls[0][2]["authorization"], "Bearer viewer-secret")

    def test_complete_download_replaces_and_failure_preserves(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "source.pcd"
            target.write_bytes(b"old")
            with server([(200, b"complete")]) as (url, calls):
                result = run(url, "download", target)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(target.read_bytes(), b"complete")
            self.assertEqual(calls[0][1], "/api/v1/maps/operations/save_1/source/pcd")
            with server([(200, b"partial", 100)]) as (url, _):
                result = run(url, "download", target)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(target.read_bytes(), b"complete")
            self.assertEqual(list(Path(directory).iterdir()), [target])

    def test_events_and_cpp_callback_exception(self):
        body = b"id: 7\nevent: state\ndata: first\ndata: second\n\ndata: unused\n\n"
        with server([(200, body)]) as (url, calls):
            result = run(url, "events")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout, b"state\n7\nfirst\nsecond")
        self.assertEqual(calls[0][2]["last-event-id"], "6")
        self.assertEqual(parse_qs(urlsplit(calls[0][1]).query), {"topic": ["/nav/state"]})
        with server([(200, body)]) as (url, _):
            result = run(url, "callback_error")
        self.assertEqual(result.returncode, 3)
        self.assertIn(b"callback failed", result.stderr)

    def test_position_uses_map_frame_evidence(self):
        body = b'{"localization":{"odometry":{"frame_id":"map","x":1,"y":2,"z":3}}}'
        with server([(200, body)]) as (url, _):
            result = run(url, "position")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(json.loads(result.stdout)["x"], 1)

    def test_waits_follow_original_task_and_map_operation(self):
        task = b'{"found":true,"status":{"task_id":"same-task","lifecycle_state_name":"SUCCESS"}}'
        with server([(200, task)]) as (url, calls):
            result = run(url, "arrived")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(calls[0][1], "/api/v1/navigation/tasks/same-task")
        accepted = b'{"accepted":true,"operation_id":"save_1","request_id":"save_1"}'
        archived = b'{"operation_id":"save_1","operation":{"job_id":"save_1","state":"SOURCE_SAVED","source_saved":true}}'
        with server([(202, accepted), (200, archived)]) as (url, calls):
            result = run(url, "source_wait")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual([call[0] for call in calls], ["POST", "GET"])

    def test_invalid_options_fail_before_network(self):
        with server() as (url, calls):
            result = run(url, "invalid")
        self.assertEqual(result.returncode, 2)
        self.assertIn(b"invalid_input", result.stdout)
        self.assertEqual(calls, [])

    def test_client_move_keeps_ownership(self):
        with server() as (url, calls):
            result = run(url, "move")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(calls), 1)


if __name__ == "__main__":
    unittest.main()
