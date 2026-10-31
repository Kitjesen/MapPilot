from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import pytest

from perception.inspection.bridge_module import InspectionEvidenceModule
from runtime.msgs.geometry import Pose
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


class FakeBridge:
    def __init__(self, requests: list[dict[str, Any]] | None = None) -> None:
        self.requests = list(requests or [])
        self.results: list[dict[str, Any]] = []
        self.closed = False

    def take_request(self) -> dict[str, Any] | None:
        if not self.requests:
            return None
        return self.requests.pop(0)

    def write_result(self, **result: Any) -> None:
        self.results.append(result)

    def close(self) -> None:
        self.closed = True


def _request(
    *,
    request_id: str = "req-1",
    action: str = "capture:overview",
    requested_at_s: float | None = None,
    deadline_s: float | None = None,
) -> dict[str, Any]:
    now = time.time()
    requested_at_s = now - 0.1 if requested_at_s is None else requested_at_s
    deadline_s = now + 30.0 if deadline_s is None else deadline_s
    return {
        "task_id": "task-1",
        "route_id": "route-1",
        "route_revision": 1,
        "map_id": "map-1",
        "map_content_epoch": 2,
        "point_index": 0,
        "request_id": request_id,
        "action": action,
        "requested_at_s": requested_at_s,
        "deadline_s": deadline_s,
    }


def _image(ts: float | None = None) -> Image:
    data = np.zeros((8, 8, 3), dtype=np.uint8)
    data[:, :, 0] = 32
    return Image(data=data, format=ImageFormat.BGR, ts=time.time() if ts is None else ts)


def _odom(ts: float | None = None) -> Odometry:
    return Odometry(pose=Pose(1.0, 2.0, 0.1), ts=time.time() if ts is None else ts)


def _jpeg(_: Image) -> bytes:
    return b"\xff\xd8test-jpeg\xff\xd9"


def test_module_persists_fresh_rgb_pose_and_detections(tmp_path: Path) -> None:
    bridge = FakeBridge([_request(action="capture:bin_full")])
    status_file = tmp_path / "status.json"
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=status_file,
        bridge_factory=lambda: bridge,
        start_thread=False,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()
    module.color_image._deliver(_image())
    module.camera_info._deliver(CameraIntrinsics(fx=1.0, fy=1.0, cx=4.0, cy=4.0, width=8, height=8))
    module.odometry._deliver(_odom())
    module.detections_3d._deliver(
        [Detection3D(id="det-1", label="trash_bin", confidence=0.8)]
    )

    assert module.poll_once() is True

    assert bridge.results == [
        {
            "request_id": "req-1",
            "persisted": True,
            "reason": "persisted",
            "analysis_verdict": "inconclusive",
            "result_at_s": bridge.results[0]["result_at_s"],
        }
    ]
    evidence_dir = tmp_path / "evidence" / "requests" / "req-1"
    manifest = json.loads((evidence_dir / "manifest.json").read_text(encoding="utf-8"))
    artifacts = {record["kind"]: record for record in manifest["persistence"]["artifacts"]}
    assert {"rgb", "pose", "detections"} <= set(artifacts)
    assert artifacts["rgb"]["media_type"] == "image/jpeg"
    assert manifest["analysis"]["verdict"] == "inconclusive"
    assert manifest["analysis"]["reason"] == "analyzer_not_integrated"

    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is True
    assert status["state"] == "ready"
    assert status["heartbeat_ts"] == status["heartbeat_at_s"]
    assert status["last_request_id"] == "req-1"
    assert status["last_evidence_id"] == "req-1"
    assert {action: status["analyzers"][action] for action in (
        "capture:indicator_on", "capture:indicator_off", "capture:indicator_green",
        "capture:indicator_red", "capture:indicator_yellow", "capture:door_open", "capture:door_closed",
    )} == dict.fromkeys((
        "capture:indicator_on", "capture:indicator_off", "capture:indicator_green",
        "capture:indicator_red", "capture:indicator_yellow", "capture:door_open", "capture:door_closed",
    ), "visual_observation")


def test_module_replies_not_persisted_when_frame_is_stale(tmp_path: Path) -> None:
    bridge = FakeBridge([_request(request_id="req-stale")])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge,
        start_thread=False,
        max_frame_age_s=0.1,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()
    module.color_image._deliver(_image(ts=time.time() - 10.0))
    module.odometry._deliver(_odom())

    assert module.poll_once() is True

    assert bridge.results[0]["request_id"] == "req-stale"
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["reason"] == "rgb_stale"
    assert not (tmp_path / "evidence" / "requests" / "req-stale").exists()


def test_module_rejects_fresh_but_unsynchronized_rgb_and_odometry(tmp_path: Path) -> None:
    now = time.time()
    bridge = FakeBridge([_request(request_id="req-unsynchronized")])
    status_file = tmp_path / "status.json"
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=status_file,
        bridge_factory=lambda: bridge,
        start_thread=False,
        max_frame_age_s=2.0,
        max_odom_age_s=2.0,
        max_rgb_odom_skew_s=0.2,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()
    module.color_image._deliver(_image(ts=now))
    module.odometry._deliver(_odom(ts=now - 0.5))

    assert module.poll_once() is True

    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["reason"] == "rgb_odometry_unsynchronized"
    assert not (tmp_path / "evidence" / "requests" / "req-unsynchronized").exists()
    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is False
    assert status["readiness_reason"] == "rgb_odometry_unsynchronized"
    assert status["max_rgb_odom_skew_s"] == 0.2


@pytest.mark.parametrize("late_color", [False, True])
def test_low_rate_rgb_keeps_its_nearest_pose_for_readiness_and_persistence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, late_color: bool,
) -> None:
    clock = [1000.0]
    monkeypatch.setattr("perception.inspection.bridge_module.time.time", lambda: clock[0])
    bridge = FakeBridge([_request(request_id="req-matched")])
    status_file = tmp_path / "status.json"
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=status_file,
        bridge_factory=lambda: bridge, start_thread=False, jpeg_encoder=_jpeg,
        max_frame_age_s=3.0, max_odom_age_s=0.5,
    )
    module.setup()
    module.start()
    module.odometry._deliver(Odometry(pose=Pose(1.0, 0.0, 0.0), ts=999.9))
    closest = Odometry(pose=Pose(2.0, 0.0, 0.0), ts=1000.0)
    module.odometry._deliver(closest)
    if not late_color:
        module.color_image._deliver(_image(ts=1000.0))
    for index in range(1, 17):
        clock[0] = 1000.0 + index * 0.05
        module.odometry._deliver(Odometry(pose=Pose(2.0 + index, 0.0, 0.0), ts=clock[0]))
    if late_color:
        module.color_image._deliver(_image(ts=1000.0))

    module._write_status(ready=True, state="ready")
    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is True
    assert status["readiness_reason"] == "ready"
    assert module.poll_once() is True
    assert bridge.results[0]["persisted"] is True
    pose = json.loads(
        (tmp_path / "evidence" / "requests" / "req-matched" / "pose.json").read_text(encoding="utf-8")
    )
    assert pose["captured_at_s"] == 1000.0
    assert pose["odometry"]["ts"] == 1000.0
    assert pose["odometry"]["pose"] == closest.pose.to_dict()
    assert pose["rgb_odom_skew_s"] == 0.0


def test_matching_odometry_can_arrive_after_the_rgb_frame(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = [1000.0]
    monkeypatch.setattr("perception.inspection.bridge_module.time.time", lambda: clock[0])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=FakeBridge, start_thread=False, jpeg_encoder=_jpeg,
    )
    module.setup()
    module.color_image._deliver(_image(ts=1000.0))
    module.odometry._deliver(_odom(ts=999.5))
    assert module._snapshot(clock[0])["blocker"] == "rgb_odometry_unsynchronized"
    clock[0] = 1000.6
    matching = Odometry(pose=Pose(3.0, 0.0, 0.0), ts=1000.05)
    module.odometry._deliver(matching)
    snapshot = module._snapshot(clock[0])
    assert snapshot["blocker"] == ""
    assert snapshot["odom"].value is matching
    assert snapshot["rgb_odom_skew_s"] == pytest.approx(0.05)


def test_stale_odometry_stream_does_not_authorize_capture_with_a_matching_old_pose(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = [1000.0]
    monkeypatch.setattr("perception.inspection.bridge_module.time.time", lambda: clock[0])
    bridge = FakeBridge([_request(request_id="req-expired-pose")])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge, start_thread=False, jpeg_encoder=_jpeg,
        max_odom_age_s=0.4,
    )
    module.setup()
    module.color_image._deliver(_image(ts=1000.0))
    module.odometry._deliver(_odom(ts=1000.0))
    clock[0] = 1000.6
    assert module.poll_once() is True
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["reason"] == "odometry_stale"
    assert not (tmp_path / "evidence" / "requests" / "req-expired-pose").exists()


def test_expired_photo_and_pose_history_are_not_reused_with_a_fresh_odometry_stream(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = [1000.0]
    monkeypatch.setattr("perception.inspection.bridge_module.time.time", lambda: clock[0])
    bridge = FakeBridge([_request(request_id="req-expired-history")])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge, start_thread=False, jpeg_encoder=_jpeg,
        max_frame_age_s=1.0, max_odom_age_s=0.4,
    )
    module.setup()
    module.color_image._deliver(_image(ts=1000.0))
    module.odometry._deliver(_odom(ts=1000.0))
    clock[0] = 1001.5
    module.odometry._deliver(_odom(ts=clock[0]))
    assert module.poll_once() is True
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["reason"] == "rgb_stale"
    assert all(sample.message_ts_s > 1000.0 for sample in module._odom_history)
    assert not (tmp_path / "evidence" / "requests" / "req-expired-history").exists()


def test_module_replays_committed_evidence_idempotently(tmp_path: Path) -> None:
    request = _request(request_id="req-repeat", action="capture:plate_ocr")
    bridge = FakeBridge([request, dict(request)])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge,
        start_thread=False,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()
    module.color_image._deliver(_image())
    module.odometry._deliver(_odom())

    assert module.poll_once() is True
    assert module.poll_once() is True

    assert [result["persisted"] for result in bridge.results] == [True, True]
    assert [result["request_id"] for result in bridge.results] == ["req-repeat", "req-repeat"]
    assert bridge.results[1]["reason"] == "persisted"
    assert bridge.results[1]["analysis_verdict"] == "inconclusive"


def test_module_replays_committed_evidence_even_when_current_frame_is_stale(tmp_path: Path) -> None:
    request = _request(request_id="req-repeat-stale")
    bridge = FakeBridge([request, dict(request)])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge,
        start_thread=False,
        max_frame_age_s=0.1,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()
    module.color_image._deliver(_image())
    module.odometry._deliver(_odom())

    assert module.poll_once() is True
    module.color_image._deliver(_image(ts=time.time() - 10.0))
    assert module.poll_once() is True

    assert [result["persisted"] for result in bridge.results] == [True, True]
    assert bridge.results[1]["request_id"] == "req-repeat-stale"
    assert bridge.results[1]["reason"] == "persisted"


def test_module_accepts_native_revision_field_without_writing_revision_zero(tmp_path: Path) -> None:
    request = _request(request_id="req-native-revision")
    request["revision"] = request.pop("route_revision")
    bridge = FakeBridge([request])
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge,
        start_thread=False,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()
    module.color_image._deliver(_image())
    module.odometry._deliver(_odom())

    assert module.poll_once() is True

    assert bridge.results[0]["persisted"] is True
    manifest = json.loads(
        (tmp_path / "evidence" / "requests" / "req-native-revision" / "manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["request"]["route_revision"] == 1
    assert "revision" not in manifest["request"]


def test_stop_closes_bridge_and_marks_status_not_ready(tmp_path: Path) -> None:
    bridge = FakeBridge()
    status_file = tmp_path / "status.json"
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=status_file,
        bridge_factory=lambda: bridge,
        start_thread=False,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()

    module.stop()

    assert bridge.closed is True
    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is False
    assert status["state"] == "stopped"


def test_status_ready_requires_fresh_rgb_and_odometry(tmp_path: Path) -> None:
    bridge = FakeBridge()
    status_file = tmp_path / "status.json"
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence",
        status_file=status_file,
        bridge_factory=lambda: bridge,
        start_thread=False,
        max_frame_age_s=0.1,
        max_odom_age_s=0.1,
        jpeg_encoder=_jpeg,
    )
    module.setup()
    module.start()

    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is False
    assert status["readiness_reason"] == "rgb_missing"

    module.color_image._deliver(_image())
    module._last_heartbeat_s = 0.0
    assert module.poll_once() is False
    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is False
    assert status["readiness_reason"] == "odometry_missing"

    module.odometry._deliver(_odom())
    module._last_heartbeat_s = 0.0
    assert module.poll_once() is False
    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is True
    assert status["readiness_reason"] == "ready"

    module.color_image._deliver(_image(ts=time.time() - 10.0))
    module._last_heartbeat_s = 0.0
    assert module.poll_once() is False
    status = json.loads(status_file.read_text(encoding="utf-8"))
    assert status["ready"] is False
    assert status["readiness_reason"] == "rgb_stale"
