import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from lingtu.sim.viewer_goal import ViewerGoal, world_point_to_map


def slam_snapshot():
    return {
        "native_product": {"product_session_id": "test-session"}, "has_odom": True,
        "snapshot_written_at_s": time.time(), "stamp_s": time.time(),
        "map_odom_tf": {
            "valid": True, "frame_id": "map", "child_frame_id": "odom",
            "tx": 10, "ty": 20, "tz": 2,
            "qx": 0, "qy": 0, "qz": np.sqrt(.5), "qw": np.sqrt(.5),
        },
        "odometry": {
            "frame_id": "odom", "child_frame_id": "body",
            "pose": {"x": 1, "y": 2, "z": .5, "qx": 0, "qy": 0, "qz": 0, "qw": 1},
        },
    }


def test_world_map_transform_includes_rotation_translation_and_height():
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    # One metre ahead of the robot in world, and half a metre below its body.
    target = world_point_to_map(np.array([3, 5, 0]), np.array([3, 4, .5]), rotation, slam_snapshot())
    np.testing.assert_allclose(target, [8, 22, 2], atol=1e-10)


@pytest.fixture
def bridge(tmp_path):
    plan = SimpleNamespace(
        host_config={"enable_goals": True, "gateway_port": 5050},
        has_process=lambda _: True,
        process=lambda _: SimpleNamespace(command=SimpleNamespace(env={"LINGTU_NAV_ODOM_MAX_AGE_S": "0.60"})),
    )
    (tmp_path / "slam.status.json").write_text(json.dumps(slam_snapshot()))
    goal = ViewerGoal(plan, tmp_path, "test-session")
    goal.observe_pose(time.time(), np.zeros(3), np.eye(3))
    yield goal
    goal.close()


def test_busy_network_coalesces_clicks_and_sends_only_latest_pending_goal(bridge, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    requests = []

    def request(path, payload=None):
        requests.append((path, payload))
        if payload is None:
            entered.set()
            assert release.wait(2)
            return {"env": "sim", "product_session_id": "test-session"}
        return {"accepted": True, "execution_confirmed": False}

    monkeypatch.setattr(bridge, "_request", request)
    try:
        bridge.submit(np.zeros(3))
        assert entered.wait(1)
        assert bridge.poll() == "Submitting goal..."
        bridge.submit(np.ones(3))
        bridge.submit(np.array([2., 3., 4.]))
        assert "Latest goal queued" in bridge.poll()
    finally:
        release.set()
    bridge._future.result(timeout=2)
    assert bridge.poll() == "Submitting latest goal..."
    bridge._future.result(timeout=2)
    assert "submitted" in bridge.poll()
    assert len(requests) == 6
    assert requests[1][0] == "/api/v1/navigation/resume"
    assert requests[2][0] == "/api/v1/navigate/click"
    assert requests[2][1]["z"] == 2.5
    assert requests[5][0] == "/api/v1/navigate/click"
    np.testing.assert_allclose([requests[5][1][key] for key in ("x", "y", "z")], [5., 23., 6.5])


@pytest.mark.parametrize("error", [TimeoutError("reply timed out"), ValueError("rejected")])
def test_failed_old_reply_does_not_drop_a_new_explicit_click(bridge, monkeypatch, error):
    entered, release = threading.Event(), threading.Event()
    sent = []

    def send(target):
        sent.append(target)
        if len(sent) == 1:
            entered.set()
            assert release.wait(2)
            raise error
        return "Latest goal submitted"

    monkeypatch.setattr(bridge, "_send", send)
    try:
        bridge.submit(np.zeros(3))
        assert entered.wait(1)
        bridge.submit(np.ones(3))
    finally:
        release.set()
    with pytest.raises(type(error)):
        bridge._future.result(timeout=2)
    assert bridge.poll() == "Submitting latest goal..."
    bridge._future.result(timeout=2)
    assert bridge.poll() == "Latest goal submitted"
    bridge.poll()
    assert len(sent) == 2
    assert sent[0] != sent[1]


def test_close_discards_queued_click(bridge, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    sent = []

    def send(target):
        sent.append(target)
        entered.set()
        assert release.wait(2)
        return "Goal submitted"

    monkeypatch.setattr(bridge, "_send", send)
    try:
        bridge.submit(np.zeros(3))
        assert entered.wait(1)
        bridge.submit(np.ones(3))
    finally:
        release.set()
    bridge.close()
    bridge.poll()
    assert len(sent) == 1


@pytest.mark.parametrize("session", [
    {"env": "real", "product_session_id": "test-session"},
    {"env": "sim", "product_session_id": "another-session"},
])
def test_wrong_gateway_cannot_receive_a_goal(bridge, monkeypatch, session):
    calls = []
    monkeypatch.setattr(bridge, "_request", lambda path: calls.append(path) or session)
    with pytest.raises(ValueError, match="current simulation"):
        bridge._send([1, 2, 0])
    assert calls == ["/api/v1/session"]


@pytest.mark.parametrize("change", [
    {"stamp_s": 0}, {"snapshot_written_at_s": 0}, {"native_product": {"product_session_id": "old"}},
    {"map_odom_tf": {"valid": False}}, {"has_odom": False},
    {"map_odom_tf": None}, {"odometry": None},
])
def test_stale_or_missing_localization_never_submits(bridge, tmp_path, change):
    snapshot = slam_snapshot()
    snapshot.update(change)
    (tmp_path / "slam.status.json").write_text(json.dumps(snapshot))
    with pytest.raises(ValueError):
        bridge.submit(np.zeros(3))
    assert bridge._future is None


def test_missing_odometry_reports_actual_slam_failure_before_submission(bridge, tmp_path):
    snapshot = slam_snapshot()
    snapshot.update(has_odom=False, state="LOST", reason="fastlio_lidar_update_rejected_streak")
    (tmp_path / "slam.status.json").write_text(json.dumps(snapshot))
    with pytest.raises(ValueError, match="LOST / fastlio_lidar_update_rejected_streak"):
        bridge.submit(np.zeros(3))
    assert bridge._future is None


def test_navigation_rejection_is_visible(bridge, monkeypatch):
    monkeypatch.setattr(bridge, "_request", lambda path, payload=None: (
        {"env": "sim", "product_session_id": "test-session"} if payload is None
        else {"accepted": False, "error": "navigation_not_ready"}
    ))
    bridge.submit(np.zeros(3))
    try:
        bridge._future.result(timeout=2)
    except ValueError:
        pass
    assert bridge.poll() == "Goal rejected: navigation_not_ready"


def test_transform_uses_historical_body_pose_not_current_moving_body(bridge, monkeypatch, tmp_path):
    snapshot = slam_snapshot()
    snapshot["stamp_s"] -= .2
    bridge._poses.clear()
    bridge.observe_pose(snapshot["stamp_s"], np.zeros(3), np.eye(3))
    bridge.observe_pose(time.time(), np.array([2, 0, 0]), np.eye(3))
    (tmp_path / "slam.status.json").write_text(json.dumps(snapshot))
    monkeypatch.setattr(bridge, "_send", lambda target: target)
    bridge.submit(np.zeros(3))
    np.testing.assert_allclose(bridge._future.result(timeout=2), [8, 21, 2.5])


def test_no_matching_simulation_sample_rejects_click(bridge):
    bridge._poses.clear()
    bridge.observe_pose(time.time() - 1, np.zeros(3), np.eye(3))
    with pytest.raises(ValueError, match="timestamp"):
        bridge.submit(np.zeros(3))


@pytest.mark.parametrize("accepted", [False, True])
def test_local_goal_http_ignores_system_proxy_and_reports_native_reply(bridge, monkeypatch, accepted):
    from http.server import BaseHTTPRequestHandler, HTTPServer

    from gateway.schemas import ClickNavRequest

    requests = []
    monkeypatch.setenv("http_proxy", "http://127.0.0.1:1")
    monkeypatch.setenv("HTTP_PROXY", "http://127.0.0.1:1")
    monkeypatch.setenv("no_proxy", "")
    monkeypatch.setenv("NO_PROXY", "")
    monkeypatch.setattr("urllib.request.proxy_bypass", lambda _: False)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps({"env": "sim", "product_session_id": "test-session"}).encode())

        def do_POST(self):
            request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            if self.path == "/api/v1/navigation/resume":
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'{"accepted":true}')
                return
            requests.append(ClickNavRequest.model_validate(request))
            self.send_response(200 if accepted else 409)
            self.end_headers()
            self.wfile.write(json.dumps(
                {"accepted": True} if accepted else {
                    "error": "navigation_not_ready", "message": "not ready " * 50,
                    "detail": {"blockers": ["local_collision_stale"]},
                }
            ).encode())

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    bridge._base_url = f"http://127.0.0.1:{server.server_port}"
    try:
        if accepted:
            assert "Goal submitted" in bridge._send([1, 2, .5])
        else:
            with pytest.raises(ValueError, match=r"HTTP 409: local_collision_stale$"):
                bridge._send([1, 2, .5])
        assert len(requests) == 1
        assert requests[0].z == .5
    finally:
        server.shutdown()
        thread.join()
        server.server_close()


def test_click_resumes_released_keyboard_hold_and_waits_for_public_status(bridge, monkeypatch):
    calls = []
    statuses = iter([{"control": {"resume_required": True}},
                     {"control": {"resume_required": False, "operator_takeover_latched": False}}])

    def request(path, payload=None):
        calls.append(path)
        if path == "/api/v1/session":
            return {"env": "sim", "product_session_id": "test-session"}
        if path == "/api/v1/navigation/resume":
            return {"accepted": True, "resume_was_required": True}
        if path == "/api/v1/navigation/status":
            return next(statuses)
        return {"accepted": True}

    monkeypatch.setattr(bridge, "_request", request)
    assert bridge.send_map_goal([1, 2, .5])["accepted"]
    assert calls == ["/api/v1/session", "/api/v1/navigation/resume",
                     "/api/v1/navigation/status", "/api/v1/navigation/status", "/api/v1/navigate/click"]


@pytest.mark.parametrize("reason", ["operator_authority_active", "estop_latched"])
def test_click_does_not_submit_when_native_resume_is_rejected(bridge, monkeypatch, reason):
    calls = []

    def request(path, payload=None):
        calls.append(path)
        if path == "/api/v1/session":
            return {"env": "sim", "product_session_id": "test-session"}
        raise ValueError(reason)

    monkeypatch.setattr(bridge, "_request", request)
    with pytest.raises(ValueError, match=reason):
        bridge.send_map_goal([1, 2, .5])
    assert calls == ["/api/v1/session", "/api/v1/navigation/resume"]


def test_lost_http_reply_is_reported_as_unknown_and_not_retried(bridge, monkeypatch):
    calls = []

    def send(target):
        calls.append(target)
        raise TimeoutError("reply timed out")

    monkeypatch.setattr(bridge, "_send", send)
    bridge.submit(np.zeros(3))
    with pytest.raises(TimeoutError):
        bridge._future.result(timeout=2)
    assert "reply unavailable" in bridge.poll()
    bridge.poll()
    assert len(calls) == 1


def test_compiled_truth_navigation_uses_world_goal_without_slam_file(tmp_path, monkeypatch):
    from lingtu.assembly.compiler import compile_run_plan

    plan = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco", "localization": "truth"})
    goal = ViewerGoal(plan, tmp_path, "test-session")
    monkeypatch.setattr(goal, "_send", lambda target: target)
    try:
        goal.submit(np.array([3.0, 4.0, .5]))
        assert goal._future.result(timeout=2) == [3.0, 4.0, .5]
    finally:
        goal.close()


@pytest.mark.parametrize("send_headers", [False, True])
def test_close_interrupts_running_goal_reply(bridge, send_headers):
    from http.server import BaseHTTPRequestHandler, HTTPServer

    entered, release = threading.Event(), threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"env":"sim","product_session_id":"test-session"}')

        def do_POST(self):
            self.rfile.read(int(self.headers["Content-Length"]))
            if send_headers:
                self.send_response(200)
                self.end_headers()
                self.wfile.flush()
            entered.set()
            release.wait(3)

        def log_message(self, *args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    bridge._base_url = f"http://127.0.0.1:{server.server_port}"
    try:
        bridge.submit(np.zeros(3))
        assert entered.wait(2)
        started = time.monotonic()
        bridge.close()
        assert time.monotonic() - started < 1.0
        assert bridge._future.done()
        with pytest.raises(ValueError, match="closed"):
            bridge.submit(np.ones(3))
    finally:
        release.set()
        server.shutdown()
        thread.join()
        server.server_close()


def test_physics_pose_history_matches_slam_without_any_render_frame(bridge, tmp_path):
    from drivers.sim.mujoco.runtime import LiveViewer

    viewer = LiveViewer.__new__(LiveViewer)
    viewer._goal_input = bridge
    snapshot = slam_snapshot()
    start = snapshot["stamp_s"] - .2
    snapshot["stamp_s"] = start + .1
    bridge._poses.clear()
    position = np.zeros(3)
    for step in range(41):
        position[:] = [step * .01, 0, 0]
        viewer.observe_pose(start + step * .005, position, [0, 0, 0, 1])
    # Mutating the producer's sample must not change its historical copy.
    position[:] = 999
    (tmp_path / "slam.status.json").write_text(json.dumps(snapshot))
    _rotation, translation = bridge.world_to_map_transform()
    expected = world_point_to_map(np.zeros(3), np.array([.2, 0, 0]), np.eye(3), snapshot)
    np.testing.assert_allclose(translation, expected, atol=1e-9)
    assert len(bridge._poses) == 41
