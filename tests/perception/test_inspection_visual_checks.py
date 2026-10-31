"""Visual-reader contracts and capture integration, not recognition accuracy."""

from __future__ import annotations

import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

import pytest

from perception.inspection.bridge_module import InspectionEvidenceModule
from perception.inspection.visual_checks import InspectionVisualReader
from runtime.msgs.geometry import Pose
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Image, ImageFormat


class VisionClient:
    supports_vision = True

    def __init__(self, response: Any) -> None:
        self.response = response
        self.messages: list[Any] = []

    def is_available(self) -> bool:
        return True

    async def chat(self, messages: list[Any], **_: Any) -> str:
        self.messages.append(messages)
        if isinstance(self.response, Exception):
            raise self.response
        return self.response if isinstance(self.response, str) else json.dumps(self.response)


def _observation(state: str, *, count: int = 1, visible: bool = True) -> dict[str, Any]:
    return {"observed_state": state, "target_count": count, "visible": visible,
            "reason": "fixture observation"}


@pytest.mark.parametrize(
    ("action", "state", "verdict"),
    [("capture:indicator_green", "green", "normal"),
     ("capture:indicator_green", "red", "anomaly"),
     ("capture:indicator_on", "yellow", "normal"),
     ("capture:indicator_off", "off", "normal"),
     ("capture:indicator_off", "on", "anomaly"),
     ("capture:indicator_red", "on", "inconclusive"),
     ("capture:door_closed", "open", "anomaly"),
     ("capture:door_open", "open", "normal")],
)
def test_reader_returns_observation_and_store_contract_compares_expected_state(
    action: str, state: str, verdict: str
) -> None:
    client = VisionClient({**_observation(state), "verdict": "normal"})
    finding = InspectionVisualReader(client=client).read(action, b"jpeg-fixture")
    assert finding.observed_state == state
    assert finding.analysis(action)["verdict"] == verdict
    assert finding.source == "inspection_vlm"
    assert client.messages[0][0]["content"][1]["image_url"]["url"] == (
        "data:image/jpeg;base64," + base64.b64encode(b"jpeg-fixture").decode("ascii")
    )


@pytest.mark.parametrize(
    ("response", "reason"),
    [(_observation("green", count=2), "ambiguous_visual_target"),
     (_observation("green", count=0), "visual_target_not_visible"),
     (_observation("green", visible=False), "visual_target_not_visible"),
     (_observation("unknown"), "visual_state_unclear"),
     (_observation("open"), "invalid_visual_observation"),
     ({**_observation("green"), "target_count": True}, "invalid_visual_observation"),
     ("not-json", "invalid_visual_observation"),
     (RuntimeError("model offline"), "vision_model_unavailable"),
     (TimeoutError(), "vision_model_timeout")],
)
def test_uncertain_or_unavailable_reader_never_produces_a_normal_finding(
    response: Any, reason: str
) -> None:
    finding = InspectionVisualReader(client=VisionClient(response)).read(
        "capture:indicator_green", b"jpeg-fixture"
    )
    assert finding.observed_state == "unknown"
    assert finding.reason == reason
    assert finding.analysis("capture:indicator_green")["verdict"] == "inconclusive"


def test_missing_or_text_only_model_is_inconclusive() -> None:
    missing = InspectionVisualReader().read("capture:door_closed", b"jpeg-fixture")
    assert missing.reason == "vision_model_not_configured"
    client = VisionClient(_observation("closed"))
    client.supports_vision = False
    finding = InspectionVisualReader(client=client).read("capture:door_closed", b"jpeg-fixture")
    assert finding.observed_state == "unknown"
    assert client.messages == []


class CaptureBridge:
    def __init__(self, action: str) -> None:
        now = time.time()
        self.requests = [{
            "task_id": "task-visual", "route_id": "route-visual", "route_revision": 1,
            "map_id": "map-visual", "map_content_epoch": 2, "point_index": 0,
            "request_id": "visual-capture-1", "action": action,
            "requested_at_s": now - 0.1, "deadline_s": now + 30.0,
        }]
        self.results: list[dict[str, Any]] = []

    def take_request(self) -> dict[str, Any] | None:
        return self.requests.pop(0) if self.requests else None

    def write_result(self, **result: Any) -> None:
        self.results.append(result)

    def close(self) -> None:
        pass


@pytest.mark.parametrize(
    ("action", "response", "verdict"),
    [("capture:indicator_green", _observation("green"), "normal"),
     ("capture:indicator_green", _observation("red"), "anomaly"),
     ("capture:door_closed", _observation("open"), "anomaly"),
     ("capture:door_closed", _observation("open", count=2), "inconclusive"),
     ("capture:door_closed", RuntimeError("model offline"), "inconclusive")],
)
def test_capture_persists_photo_and_reports_analysis_without_failing_route_handshake(
    tmp_path: Path, action: str, response: Any, verdict: str
) -> None:
    bridge = CaptureBridge(action)
    client = VisionClient(response)
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge, start_thread=False,
        jpeg_encoder=lambda _: b"\xff\xd8fixture\xff\xd9",
        visual_reader=InspectionVisualReader(client=client),
    )
    module.setup()
    module.start()
    now = time.time()
    module.color_image._deliver(Image(data=np.zeros((8, 8, 3), dtype=np.uint8),
                                     format=ImageFormat.BGR, ts=now))
    module.odometry._deliver(Odometry(pose=Pose(1.0, 2.0, 0.1), ts=now))
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    assert bridge.results[0]["analysis_verdict"] == verdict
    manifest_path = tmp_path / "evidence" / "requests" / "visual-capture-1" / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["persistence"]["persisted"] is True
    assert manifest["analysis"]["verdict"] == verdict
    assert manifest["analysis"]["observation"]["source"] == "inspection_vlm"
    assert any(item["kind"] == "rgb" for item in manifest["persistence"]["artifacts"])
    module.stop()


@pytest.mark.parametrize("broken_reader", [False, True])
def test_missing_service_or_unexpected_reader_error_still_commits_the_photo(
    tmp_path: Path, broken_reader: bool
) -> None:
    class BrokenReader:
        def read(self, *_: Any) -> Any:
            raise RuntimeError("fixture reader failure")

    bridge = CaptureBridge("capture:door_closed")
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge, start_thread=False,
        jpeg_encoder=lambda _: b"\xff\xd8fixture\xff\xd9",
        visual_reader=BrokenReader() if broken_reader else None,
    )
    module.setup()
    module.start()
    now = time.time()
    module.color_image._deliver(Image(data=np.zeros((8, 8, 3), dtype=np.uint8),
                                     format=ImageFormat.BGR, ts=now))
    module.odometry._deliver(Odometry(pose=Pose(1.0, 2.0, 0.1), ts=now))
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"
    manifest_path = tmp_path / "evidence" / "requests" / "visual-capture-1" / "manifest.json"
    analysis = json.loads(manifest_path.read_text(encoding="utf-8"))["analysis"]
    assert analysis["reason"] == (
        "vision_reader_failed" if broken_reader else "vision_model_not_configured"
    )
    assert analysis["observation"]["observed_state"] == "unknown"
    module.stop()


def test_retry_reuses_the_committed_finding_without_calling_model_again(tmp_path: Path) -> None:
    bridge = CaptureBridge("capture:indicator_green")
    bridge.requests.append(dict(bridge.requests[0]))
    client = VisionClient(_observation("green"))
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge, start_thread=False,
        jpeg_encoder=lambda _: b"\xff\xd8fixture\xff\xd9",
        visual_reader=InspectionVisualReader(client=client),
    )
    module.setup()
    module.start()
    now = time.time()
    module.color_image._deliver(Image(data=np.zeros((8, 8, 3), dtype=np.uint8),
                                     format=ImageFormat.BGR, ts=now))
    module.odometry._deliver(Odometry(pose=Pose(1.0, 2.0, 0.1), ts=now))
    assert module.poll_once()
    client.response = _observation("red")
    assert module.poll_once()
    assert len(client.messages) == 1
    assert [item["analysis_verdict"] for item in bridge.results] == ["normal", "normal"]
    module.stop()


def test_existing_remote_vision_http_contract_uses_real_jpeg_not_visual_accuracy() -> None:
    cv2 = pytest.importorskip("cv2")
    image = np.zeros((24, 32, 3), dtype=np.uint8)
    image[8:16, 12:20, 1] = 255
    encoded_ok, encoded = cv2.imencode(".jpg", image)
    assert encoded_ok
    received: list[dict[str, Any]] = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_: Any) -> None:
            pass

        def do_GET(self) -> None:
            assert self.path == "/health"
            self._reply({"ready": True})

        def do_POST(self) -> None:
            assert self.path == "/chat"
            received.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            self._reply({"text": json.dumps(_observation("green"))})

        def _reply(self, payload: dict[str, Any]) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        finding = InspectionVisualReader(server_url=f"http://127.0.0.1:{server.server_port}").read(
            "capture:indicator_green", bytes(encoded)
        )
        assert finding.observed_state == "green"
        image_url = received[0]["messages"][0]["content"][1]["image_url"]["url"]
        decoded = cv2.imdecode(np.frombuffer(base64.b64decode(image_url.split(",", 1)[1]),
                                            dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == image.shape
        assert received[0]["temperature"] == 0.0
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
