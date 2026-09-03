# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.coordinator.coordinator import CoordinatorError
from sim.runtime.coordinator.external_evidence import ExternalEvidenceWatcher


class _Target:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []
        self.sensor_reports: list[tuple[str, str, dict[str, object]]] = []

    def report_binding_prepared(self, facet: str, **_: object) -> None:
        self.calls.append(("binding_prepared", facet))

    def report_binding_active(self, facet: str, **_: object) -> None:
        self.calls.append(("binding_active", facet))

    def report_binding_failed(self, facet: str, **_: object) -> None:
        self.calls.append(("binding_failed", facet))

    def report_sensor_stream_prepared(self, sensor_id: str, **evidence: object) -> None:
        self.calls.append(("sensor_prepared", sensor_id))
        self.sensor_reports.append(("sensor_prepared", sensor_id, evidence))

    def report_sensor_stream_active(self, sensor_id: str, **evidence: object) -> None:
        self.calls.append(("sensor_active", sensor_id))
        self.sensor_reports.append(("sensor_active", sensor_id, evidence))

    def report_sensor_stream_retracted(self, sensor_id: str, **evidence: object) -> None:
        self.calls.append(("sensor_retracted", sensor_id))
        self.sensor_reports.append(("sensor_retracted", sensor_id, evidence))

    def report_sensor_stream_failed(self, sensor_id: str, **evidence: object) -> None:
        self.calls.append(("sensor_failed", sensor_id))
        self.sensor_reports.append(("sensor_failed", sensor_id, evidence))


def _document(*, basis: str, visual: str = "PREPARING") -> dict[str, object]:
    return {
        "schema": "lingtu.sim.sensor-readiness-evidence.v1",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 1,
        "source_id": "robotsimue-camera",
        "basis": basis,
        "visual": {"state": visual},
        "sensors": {"camera_streams": "ACTIVE", "overall": "PREPARING"},
        "streams": [
            {
                "sensor_id": "thunder_01.front_rgb",
                "state": "ACTIVE",
                "published_frames": 1,
                "last_sample_truth_sequence": 101,
                "last_sample_sim_time_ns": 1_010_000_000,
            }
        ],
    }


def _watcher(path: Path) -> ExternalEvidenceWatcher:
    return ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
    )


def _watcher_for_generation(
    path: Path,
    *,
    model_generation: int,
    reset_generation: int,
) -> ExternalEvidenceWatcher:
    return ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=model_generation,
        reset_generation=reset_generation,
        expected_source_id="robotsimue-camera",
    )


def test_camera_frame_activates_only_its_stream_and_is_idempotent(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    target = _Target()
    watcher = _watcher(path)

    assert watcher.apply(target)
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
    ]
    assert [report[2]["published_frames"] for report in target.sensor_reports] == [
        1,
        1,
    ]
    assert [
        (
            report[2]["last_sample_truth_sequence"],
            report[2]["last_sample_sim_time_ns"],
        )
        for report in target.sensor_reports
    ] == [(101, 1_010_000_000), (101, 1_010_000_000)]
    assert watcher.apply(target)
    assert len(target.calls) == 2


def test_unchanged_evidence_file_is_not_reparsed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    original = Path.read_text
    reads = 0

    def counted_read(self: Path, *args: Any, **kwargs: Any) -> str:
        nonlocal reads
        if self == path:
            reads += 1
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", counted_read)
    watcher = _watcher(path)

    assert watcher.apply(_Target()) is True
    assert watcher.apply(_Target()) is True
    assert reads == 1


def test_active_camera_frame_count_must_not_move_backward(tmp_path: Path) -> None:
    path = tmp_path / "sensor-readiness.json"
    first = _document(basis="real_rendered_frame_to_camera_shm")
    first["streams"][0]["published_frames"] = 2  # type: ignore[index]
    first["streams"][0]["last_sample_truth_sequence"] = 102  # type: ignore[index]
    first["streams"][0]["last_sample_sim_time_ns"] = 1_020_000_000  # type: ignore[index]
    path.write_text(json.dumps(first), encoding="utf-8")
    watcher = _watcher(path)
    assert watcher.apply(_Target())

    second = _document(basis="real_rendered_frame_to_camera_shm")
    replacement = tmp_path / "sensor-readiness.next.json"
    replacement.write_text(json.dumps(second), encoding="utf-8")
    replacement.replace(path)

    with pytest.raises(CoordinatorError, match="published_frames moved backward"):
        watcher.apply(_Target())


def test_active_camera_sample_stamp_must_advance_with_frame_count(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    watcher = _watcher(path)
    assert watcher.apply(_Target())

    unchanged_stamp = _document(basis="real_rendered_frame_to_camera_shm")
    unchanged_stamp["streams"][0]["published_frames"] = 2  # type: ignore[index]
    replacement = tmp_path / "sensor-readiness.next.json"
    replacement.write_text(json.dumps(unchanged_stamp), encoding="utf-8")
    replacement.replace(path)

    with pytest.raises(CoordinatorError, match="newer sample stamp"):
        watcher.apply(_Target())


@pytest.mark.parametrize(
    "missing_field",
    ("last_sample_truth_sequence", "last_sample_sim_time_ns"),
)
def test_camera_evidence_requires_both_actual_sample_stamp_fields(
    tmp_path: Path,
    missing_field: str,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    document = _document(basis="real_rendered_frame_to_camera_shm")
    del document["streams"][0][missing_field]  # type: ignore[index]
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(CoordinatorError, match=missing_field):
        _watcher(path).apply(_Target())


def test_visual_active_requires_snapshot_application_basis(tmp_path: Path) -> None:
    path = tmp_path / "visual-readiness.json"
    path.write_text(
        json.dumps(
            _document(
                basis="truth_snapshot_applied_to_visual_bindings",
                visual="ACTIVE",
            )
        ),
        encoding="utf-8",
    )
    target = _Target()

    assert _watcher(path).apply(target)
    assert ("binding_prepared", "visual") in target.calls
    assert ("binding_active", "visual") in target.calls


def test_generation_mismatch_and_partial_or_duplicate_json_fail_closed(
    tmp_path: Path,
) -> None:
    path = tmp_path / "bad.json"
    document = _document(basis="real_rendered_frame_to_camera_shm")
    document["reset_generation"] = 2
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(CoordinatorError, match="reset_generation mismatch"):
        _watcher(path).apply(_Target())

    path.write_text('{"schema":"x","schema":"y"}', encoding="utf-8")
    with pytest.raises(CoordinatorError, match="duplicate key"):
        _watcher(path).apply(_Target())


def test_transient_permission_denied_keeps_watcher_waiting(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "visual-readiness.json"
    path.write_text(
        json.dumps(
            _document(
                basis="truth_snapshot_applied_to_visual_bindings",
                visual="ACTIVE",
            )
        ),
        encoding="utf-8",
    )
    original = Path.read_text
    attempts = {"count": 0}

    def flaky_read_text(self: Path, *args: Any, **kwargs: Any) -> str:
        if self == path and attempts["count"] == 0:
            attempts["count"] += 1
            raise PermissionError("transient sharing violation")
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", flaky_read_text)
    watcher = _watcher(path)
    target = _Target()

    assert watcher.apply(target) is False
    assert target.calls == []
    assert watcher.apply(target) is True
    assert ("binding_prepared", "visual") in target.calls
    assert ("binding_active", "visual") in target.calls


def test_repeated_permission_denied_fails_after_retry_window(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "visual-readiness.json"
    path.write_text(
        json.dumps(
            _document(
                basis="truth_snapshot_applied_to_visual_bindings",
                visual="ACTIVE",
            )
        ),
        encoding="utf-8",
    )
    times = iter((10.0, 10.05, 10.11))

    def denied_read_text(self: Path, *args: Any, **kwargs: Any) -> str:
        if self == path:
            raise PermissionError("persistent sharing violation")
        return Path.read_text(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", denied_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
        sharing_lock_retry_window_s=0.1,
        clock=lambda: next(times),
    )

    assert watcher.apply(_Target()) is False
    assert watcher.apply(_Target()) is False
    with pytest.raises(CoordinatorError, match="sharing lock retry window expired"):
        watcher.apply(_Target())


def test_invalid_json_after_permission_denied_fails_immediately(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text("{}", encoding="utf-8")
    attempts = 0

    def interrupted_read_text(
        self: Path,
        *args: Any,
        **kwargs: Any,
    ) -> str:
        nonlocal attempts
        if self == path:
            attempts += 1
            if attempts == 1:
                raise PermissionError("transient sharing violation")
            return "{"
        raise AssertionError(f"unexpected read: {self}")

    monkeypatch.setattr(Path, "read_text", interrupted_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
        sharing_lock_retry_window_s=10.0,
        clock=lambda: 5.0,
    )

    assert watcher.apply(_Target()) is False
    with pytest.raises(CoordinatorError, match="cannot read external evidence"):
        watcher.apply(_Target())


def test_file_identity_replacement_during_permission_retry_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text("{}", encoding="utf-8")
    original_identity = (path.stat().st_dev, path.stat().st_ino)

    def denied_read_text(self: Path, *args: object, **kwargs: object) -> str:
        if self == path:
            raise PermissionError("sharing violation")
        raise AssertionError(f"unexpected read: {self}")

    monkeypatch.setattr(Path, "read_text", denied_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
        sharing_lock_retry_window_s=10.0,
        clock=lambda: 5.0,
    )

    assert watcher.apply(_Target()) is False
    watcher.advance_generation(model_generation=3, reset_generation=1)
    replacement = tmp_path / "replacement.json"
    replacement.write_text("{}", encoding="utf-8")
    replacement.replace(path)
    assert (path.stat().st_dev, path.stat().st_ino) != original_identity

    with pytest.raises(CoordinatorError, match="identity changed during sharing lock retry"):
        watcher.apply(_Target())


def test_existing_non_regular_evidence_path_fails_closed(tmp_path: Path) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.mkdir()

    with pytest.raises(CoordinatorError, match="is not a regular file"):
        _watcher(path).apply(_Target())


def test_atomic_replacement_between_identity_and_version_is_retried(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    document = json.dumps(_document(basis="real_rendered_frame_to_camera_shm"))
    path.write_text(document, encoding="utf-8")
    original_lstat = Path.lstat
    inspections = 0

    def replace_before_version(self: Path):
        nonlocal inspections
        if self == path:
            inspections += 1
            if inspections == 2:
                replacement = tmp_path / "replacement.json"
                replacement.write_text(document, encoding="utf-8")
                replacement.replace(path)
        return original_lstat(self)

    monkeypatch.setattr(Path, "lstat", replace_before_version)
    watcher = _watcher(path)
    target = _Target()

    assert watcher.apply(target) is False
    assert target.calls == []
    assert watcher.apply(target) is True
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
    ]


def test_continuous_atomic_replacement_expires_the_unstable_window(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    document = json.dumps(_document(basis="real_rendered_frame_to_camera_shm"))
    path.write_text(document, encoding="utf-8")
    original_lstat = Path.lstat
    inspections = 0
    replacements = 0
    times = iter((10.0, 10.05, 10.11))

    def always_replace_before_version(self: Path):
        nonlocal inspections, replacements
        if self == path:
            inspections += 1
            if inspections % 2 == 0:
                replacements += 1
                replacement = tmp_path / f"replacement-{replacements}.json"
                replacement.write_text(document, encoding="utf-8")
                replacement.replace(path)
        return original_lstat(self)

    monkeypatch.setattr(Path, "lstat", always_replace_before_version)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
        sharing_lock_retry_window_s=0.1,
        clock=lambda: next(times),
    )

    assert watcher.apply(_Target()) is False
    assert watcher.apply(_Target()) is False
    with pytest.raises(CoordinatorError, match="unstable replacement retry window expired"):
        watcher.apply(_Target())


def test_atomic_replacement_while_reading_is_discarded_then_accepted(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    document = json.dumps(_document(basis="real_rendered_frame_to_camera_shm"))
    path.write_text(document, encoding="utf-8")
    original_read_text = Path.read_text
    replaced = False

    def replacing_read_text(
        self: Path,
        *args: Any,
        **kwargs: Any,
    ) -> str:
        nonlocal replaced
        if self != path:
            return original_read_text(self, *args, **kwargs)
        payload = original_read_text(self, *args, **kwargs)
        if not replaced:
            replaced = True
            replacement = tmp_path / "replacement.json"
            replacement.write_text(document, encoding="utf-8")
            replacement.replace(path)
        return payload

    monkeypatch.setattr(Path, "read_text", replacing_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
    )
    target = _Target()

    assert watcher.apply(_Target()) is False
    assert watcher.apply(target) is True
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
    ]


def test_in_place_mutation_while_reading_still_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    document = json.dumps(_document(basis="real_rendered_frame_to_camera_shm"))
    path.write_text(document, encoding="utf-8")
    original_read_text = Path.read_text

    def mutating_read_text(self: Path, *args: Any, **kwargs: Any) -> str:
        payload = original_read_text(self, *args, **kwargs)
        if self == path:
            path.write_text(payload + " ", encoding="utf-8")
        return payload

    monkeypatch.setattr(Path, "read_text", mutating_read_text)

    with pytest.raises(CoordinatorError, match="changed while reading"):
        _watcher(path).apply(_Target())


def test_file_disappearance_while_reading_clears_permission_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text("{}", encoding="utf-8")
    attempts = 0
    times = iter((5.0, 100.0))

    def disappearing_read_text(
        self: Path,
        *args: object,
        **kwargs: object,
    ) -> str:
        nonlocal attempts
        if self != path:
            raise AssertionError(f"unexpected read: {self}")
        attempts += 1
        if attempts == 1:
            raise PermissionError("first sharing violation")
        if attempts == 2:
            path.unlink()
            raise FileNotFoundError(path)
        raise PermissionError("new sharing violation")

    monkeypatch.setattr(Path, "read_text", disappearing_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
        sharing_lock_retry_window_s=0.1,
        clock=lambda: next(times),
    )

    assert watcher.apply(_Target()) is False
    assert watcher.apply(_Target()) is False
    path.write_text("{}", encoding="utf-8")
    assert watcher.apply(_Target()) is False


def test_valid_stale_generation_clears_permission_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    original_read_text = Path.read_text
    attempts = 0
    times = iter((5.0, 100.0))

    def stale_read_text(
        self: Path,
        *args: Any,
        **kwargs: Any,
    ) -> str:
        nonlocal attempts
        if self != path:
            return original_read_text(self, *args, **kwargs)
        attempts += 1
        if attempts in {1, 3}:
            raise PermissionError("sharing violation")
        return original_read_text(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", stale_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=4,
        reset_generation=0,
        expected_source_id="robotsimue-camera",
        sharing_lock_retry_window_s=0.1,
        clock=lambda: next(times),
    )

    assert watcher.apply(_Target()) is False
    assert watcher.apply(_Target()) is False
    assert watcher.apply(_Target()) is False


@pytest.mark.parametrize(
    "retry_window",
    (-0.1, float("inf"), float("nan")),
)
def test_retry_window_must_be_finite_and_non_negative(
    tmp_path: Path,
    retry_window: float,
) -> None:
    with pytest.raises(ValueError, match="finite non-negative"):
        ExternalEvidenceWatcher(
            tmp_path / "sensor-readiness.json",
            session_id="a" * 64,
            model_generation=3,
            reset_generation=1,
            expected_source_id="robotsimue-camera",
            sharing_lock_retry_window_s=retry_window,
        )


@pytest.mark.parametrize("clock_value", (float("inf"), float("nan")))
def test_permission_retry_clock_must_return_a_finite_value(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    clock_value: float,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text("{}", encoding="utf-8")

    def denied_read_text(self: Path, *args: object, **kwargs: object) -> str:
        if self == path:
            raise PermissionError("sharing violation")
        raise AssertionError(f"unexpected read: {self}")

    monkeypatch.setattr(Path, "read_text", denied_read_text)
    watcher = ExternalEvidenceWatcher(
        path,
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        expected_source_id="robotsimue-camera",
        clock=lambda: clock_value,
    )

    with pytest.raises(CoordinatorError, match="clock returned a non-finite value"):
        watcher.apply(_Target())


def test_active_stream_requires_at_least_one_real_frame(tmp_path: Path) -> None:
    path = tmp_path / "empty.json"
    document = _document(basis="real_rendered_frame_to_camera_shm")
    document["streams"][0]["published_frames"] = 0  # type: ignore[index]
    document["streams"][0]["last_sample_truth_sequence"] = 0  # type: ignore[index]
    document["streams"][0]["last_sample_sim_time_ns"] = 0  # type: ignore[index]
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(CoordinatorError, match="has no produced frame"):
        _watcher(path).apply(_Target())


def test_watcher_accepts_generation_one_evidence_after_explicit_reset_advance(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    target = _Target()
    watcher = _watcher_for_generation(path, model_generation=3, reset_generation=0)
    watcher.advance_generation(model_generation=3, reset_generation=1)
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )

    assert watcher.apply(target)
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
    ]


@pytest.mark.parametrize(
    ("model_generation", "reset_generation"),
    ((3, 2), (4, 0)),
)
def test_generation_advance_accepts_atomically_replaced_evidence_after_denial(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    model_generation: int,
    reset_generation: int,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    original_read_text = Path.read_text
    denied = False

    def deny_once(self: Path, *args: Any, **kwargs: Any) -> str:
        nonlocal denied
        if self == path and not denied:
            denied = True
            raise PermissionError("transient sharing violation")
        return original_read_text(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", deny_once)
    watcher = _watcher(path)

    assert watcher.apply(_Target()) is False
    watcher.advance_generation(
        model_generation=model_generation,
        reset_generation=reset_generation,
    )
    replacement_document = _document(basis="real_rendered_frame_to_camera_shm")
    replacement_document["model_generation"] = model_generation
    replacement_document["reset_generation"] = reset_generation
    replacement = tmp_path / "replacement.json"
    replacement.write_text(json.dumps(replacement_document), encoding="utf-8")
    replacement.replace(path)
    target = _Target()

    assert watcher.apply(target) is True
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
    ]


def test_same_model_reset_replays_active_stream_evidence_for_new_generation(
    tmp_path: Path,
) -> None:
    path = tmp_path / "visual-readiness.json"
    target = _Target()
    watcher = _watcher(path)
    path.write_text(
        json.dumps(
            _document(
                basis="truth_snapshot_applied_to_visual_bindings",
                visual="ACTIVE",
            )
        ),
        encoding="utf-8",
    )
    assert watcher.apply(target)
    watcher.advance_generation(model_generation=3, reset_generation=2)
    document = _document(
        basis="truth_snapshot_applied_to_visual_bindings",
        visual="ACTIVE",
    )
    document["reset_generation"] = 2
    path.write_text(json.dumps(document), encoding="utf-8")

    assert watcher.apply(target)
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
        ("binding_prepared", "visual"),
        ("binding_active", "visual"),
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
    ]
    assert target.sensor_reports[-1][2]["published_frames"] == 1
    assert target.sensor_reports[-1][2]["last_sample_truth_sequence"] == 101
    assert target.sensor_reports[-1][2]["last_sample_sim_time_ns"] == 1_010_000_000
    assert target.sensor_reports[-1][2]["reset_generation"] == 2


@pytest.mark.parametrize("state", ("PREPARING", "PREPARED"))
def test_active_stream_evidence_can_retract_to_non_active_state(
    tmp_path: Path,
    state: str,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    target = _Target()
    watcher = _watcher(path)
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    assert watcher.apply(target)

    document = _document(basis="real_rendered_frame_to_camera_shm")
    document["streams"][0]["state"] = state  # type: ignore[index]
    path.write_text(json.dumps(document), encoding="utf-8")

    assert watcher.apply(target)
    assert target.calls == [
        ("sensor_prepared", "thunder_01.front_rgb"),
        ("sensor_active", "thunder_01.front_rgb"),
        ("sensor_retracted", "thunder_01.front_rgb"),
    ]
    assert target.sensor_reports[-1][2]["published_frames"] == 1
    assert target.sensor_reports[-1][2]["last_sample_truth_sequence"] == 101
    assert target.sensor_reports[-1][2]["last_sample_sim_time_ns"] == 1_010_000_000


def test_stale_reset_generation_evidence_is_ignored_after_reset_advance(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    watcher = _watcher_for_generation(path, model_generation=3, reset_generation=0)
    watcher.advance_generation(model_generation=3, reset_generation=1)
    document = _document(basis="real_rendered_frame_to_camera_shm")
    document["reset_generation"] = 0
    path.write_text(json.dumps(document), encoding="utf-8")
    target = _Target()

    assert watcher.apply(target) is False
    assert target.calls == []


def test_future_reset_generation_evidence_still_fails_after_reset_advance(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    watcher = _watcher_for_generation(path, model_generation=3, reset_generation=0)
    watcher.advance_generation(model_generation=3, reset_generation=1)
    document = _document(basis="real_rendered_frame_to_camera_shm")
    document["reset_generation"] = 2
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(CoordinatorError, match="reset_generation mismatch"):
        watcher.apply(_Target())


def test_stale_model_generation_evidence_is_ignored_after_model_advance(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    watcher = _watcher_for_generation(path, model_generation=3, reset_generation=1)
    watcher.advance_generation(model_generation=4, reset_generation=0)
    path.write_text(
        json.dumps(_document(basis="real_rendered_frame_to_camera_shm")),
        encoding="utf-8",
    )
    target = _Target()

    assert watcher.apply(target) is False
    assert target.calls == []


def test_stale_generation_bad_session_or_source_still_fails_closed(
    tmp_path: Path,
) -> None:
    path = tmp_path / "sensor-readiness.json"
    watcher = _watcher_for_generation(path, model_generation=3, reset_generation=0)
    watcher.advance_generation(model_generation=3, reset_generation=1)
    document = _document(basis="real_rendered_frame_to_camera_shm")
    document["reset_generation"] = 0
    document["source_id"] = "wrong-source"
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(CoordinatorError, match="source_id mismatch"):
        watcher.apply(_Target())

    document["source_id"] = "robotsimue-camera"
    document["session_id"] = "b" * 64
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(CoordinatorError, match="session_id mismatch"):
        watcher.apply(_Target())
