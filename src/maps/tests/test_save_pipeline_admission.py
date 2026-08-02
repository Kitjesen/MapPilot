from __future__ import annotations

import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from lingtu.product_lock import ProductControlLock
from maps.modules.service import MapsModule
from maps.services.pipeline import MapPipelineService


class _NativeSaveService:
    def __init__(self, capture_dir: Path) -> None:
        self.begin_calls = 0
        self.capture_dir = capture_dir
        self.status_queries = 0

    def begin_save_map(self, request_id: str, name: str, **_kwargs: Any) -> dict[str, Any]:
        self.begin_calls += 1
        return {
            "accepted": True,
            "status": {
                "job_id": request_id,
                "map_id": name,
                "state": "QUEUED",
                "capture_dir": str(self.capture_dir),
            },
        }

    def provide_save_map_snapshot(
        self,
        request_id: str,
        capture_dir: Path,
        **_kwargs: Any,
    ) -> dict[str, Any]:
        assert request_id == "stable-save-request"
        assert capture_dir == self.capture_dir
        return {
            "accepted": True,
            "status": {
                "job_id": request_id,
                "map_id": "field-map",
                "state": "RUNNING",
                "reason_code": "snapshot_accepted",
            },
        }

    def get_save_map_status(self, _job_id: str) -> dict[str, Any]:
        self.status_queries += 1
        raise AssertionError("save admission must not wait for the native terminal state")

    def cancel_save_map(self, _job_id: str) -> dict[str, Any]:
        raise AssertionError("accepted snapshots must not be cancelled")


class _ReplayedNativeSaveService(_NativeSaveService):
    def __init__(self, capture_dir: Path, state: str) -> None:
        super().__init__(capture_dir)
        self.state = state

    def begin_save_map(self, request_id: str, name: str, **_kwargs: Any) -> dict[str, Any]:
        return {
            "accepted": True,
            "replayed": True,
            "status": {
                "job_id": request_id,
                "map_id": name,
                "state": self.state,
            },
        }


class _Storage:
    def __init__(self, root: Path, native: _NativeSaveService) -> None:
        self.root = root
        self.native_service = native

    def map_path(self, name: str) -> Path:
        return self.root / name


class _RuntimeBridge:
    map_save_timeout_sec = 240.0
    latest_map_frame_info: dict[str, Any] = {}

    def resolve_slam_profile(self, value: str | None) -> str:
        return str(value or "native_dds")

    def map_save_capability_fields(self, _profile: str) -> dict[str, Any]:
        return {"map_save_supported": True}

    def save_map_with_adapter(self, target: Path) -> dict[str, Any]:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("VERSION 0.7\nDATA ascii\n", encoding="utf-8")
        return {"success": True}

    def snapshot_health(self) -> tuple[bool, str]:
        return True, "healthy"


def test_save_returns_after_native_accepts_snapshot_without_waiting_for_terminal(
    tmp_path: Path,
) -> None:
    capture_dir = tmp_path / "capture"
    native = _NativeSaveService(capture_dir)
    pipeline = MapPipelineService(
        storage=_Storage(tmp_path / "maps", native),
        runtime_bridge=_RuntimeBridge(),
        source_profile="test",
        runtime_data_source="test",
        build_octomap_on_save=False,
    )

    response = pipeline.save(
        "field-map",
        slam_profile="native_dds",
        request_id="stable-save-request",
    )

    assert response == {
        "action": "save",
        "success": None,
        "accepted": True,
        "status": "running",
        "reason_code": "save_job_in_progress",
        "message": "SaveMap snapshot accepted; query by job_id",
        "job_id": "stable-save-request",
        "request_id": "stable-save-request",
        "job": {
            "job_id": "stable-save-request",
            "map_id": "field-map",
            "state": "RUNNING",
            "reason_code": "snapshot_accepted",
        },
    }
    assert native.status_queries == 0


@pytest.mark.parametrize("state", ["QUEUED", "RUNNING"])
def test_idempotent_replay_reports_neutral_in_progress_admission(
    tmp_path: Path,
    state: str,
) -> None:
    native = _ReplayedNativeSaveService(tmp_path / "unused-capture", state)
    pipeline = MapPipelineService(
        storage=_Storage(tmp_path / "maps", native),
        runtime_bridge=_RuntimeBridge(),
        source_profile="test",
        runtime_data_source="test",
        build_octomap_on_save=False,
    )

    response = pipeline.save(
        "field-map",
        slam_profile="native_dds",
        request_id="stable-save-request",
    )

    assert response == {
        "action": "save",
        "success": None,
        "accepted": True,
        "status": "running",
        "reason_code": "save_job_in_progress",
        "message": "the idempotent SaveMap job is already running",
        "job_id": "stable-save-request",
        "request_id": "stable-save-request",
        "job": {
            "job_id": "stable-save-request",
            "map_id": "field-map",
            "state": state,
        },
        "replayed": True,
    }
    assert native.status_queries == 0


def test_save_rejects_before_native_begin_while_product_transition_holds_lock(
    monkeypatch,
    tmp_path: Path,
) -> None:
    state_dir = tmp_path / "runtime"
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(state_dir))
    native = _NativeSaveService(tmp_path / "capture")
    pipeline = MapPipelineService(
        storage=_Storage(tmp_path / "maps", native),
        runtime_bridge=_RuntimeBridge(),
        source_profile="test",
        runtime_data_source="test",
        build_octomap_on_save=False,
    )
    acquired = threading.Event()
    release = threading.Event()

    def hold_product_lock() -> None:
        with ProductControlLock(state_dir, timeout_s=1.0):
            acquired.set()
            release.wait(timeout=5.0)

    owner = threading.Thread(target=hold_product_lock)
    owner.start()
    assert acquired.wait(timeout=2.0)
    try:
        response = pipeline.save(
            "field-map",
            slam_profile="native_dds",
            request_id="stable-save-request",
        )
    finally:
        release.set()
        owner.join(timeout=2.0)

    assert response["success"] is False
    assert response["reason_code"] == "product_transition_in_progress"
    assert native.begin_calls == 0


def test_retry_rejects_before_native_call_while_product_transition_holds_lock(
    monkeypatch,
    tmp_path: Path,
) -> None:
    state_dir = tmp_path / "runtime"
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(state_dir))

    class Native:
        retry_calls = 0

        def retry_save_map(self, _job_id: str) -> dict[str, Any]:
            self.retry_calls += 1
            return {"accepted": True}

    native = Native()
    service = SimpleNamespace(storage=SimpleNamespace(native_service=native))
    acquired = threading.Event()
    release = threading.Event()

    def hold_product_lock() -> None:
        with ProductControlLock(state_dir, timeout_s=1.0):
            acquired.set()
            release.wait(timeout=5.0)

    owner = threading.Thread(target=hold_product_lock)
    owner.start()
    assert acquired.wait(timeout=2.0)
    try:
        response = MapsModule._retry_save_map(service, "save-job")
    finally:
        release.set()
        owner.join(timeout=2.0)

    assert response["success"] is False
    assert response["reason_code"] == "product_transition_in_progress"
    assert native.retry_calls == 0
