# ruff: noqa

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest
import tools.simstudio.service.run_service as run_service_module
from tools.simstudio.service.models import IdempotencyConflict, RevisionConflict, StoreValidationError
from tools.simstudio.service.run_service import ActiveRunConflict, RunService, RunStateError
from tools.simstudio.service.store import StudioStore


class FakeSession:
    def __init__(self, **_: object) -> None:
        self.calls: list[str] = []

    def prepare(self) -> dict[str, object]:
        self.calls.append("prepare")
        return {"readiness": {"physics": "ready"}, "sensor_summary": {"imu": "ready"}}

    def start(self) -> dict[str, object]:
        self.calls.append("start")
        return {"readiness": {"physics": "running"}}

    def pause(self) -> dict[str, object]:
        self.calls.append("pause")
        return {"readiness": {"physics": "paused"}}

    def reset(self) -> dict[str, object]:
        self.calls.append("reset")
        return {"reset_generation": 1, "sensor_summary": {"imu": "ready"}}

    def stop(self) -> dict[str, object]:
        self.calls.append("stop")
        return {"stopped": True}


class FailingStartSession(FakeSession):
    """Fake runtime that proves lifecycle failures trigger cleanup."""

    def start(self) -> dict[str, object]:
        self.calls.append("start")
        raise RuntimeError("start failed")


_COMPETE_WORKER = r"""
import json
import sys
import time
from pathlib import Path

from tools.simstudio.service.run_service import ActiveRunConflict, RunService
from tools.simstudio.service.store import StudioStore


class FakeSession:
    def prepare(self):
        return {"readiness": {"physics": "ready"}}

    def stop(self):
        return {"stopped": True}


root, run_id, barrier, release, result = sys.argv[1:]
service = RunService(StudioStore(Path(root)), lambda **_: FakeSession())
deadline = time.monotonic() + 10
while not Path(barrier).exists() and time.monotonic() < deadline:
    time.sleep(0.01)
try:
    service.prepare(run_id)
    outcome = ["ready", run_id]
    Path(result).write_text(json.dumps(outcome), encoding="utf-8")
    deadline = time.monotonic() + 10
    while not Path(release).exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    service.stop(run_id)
except ActiveRunConflict:
    outcome = ["conflict", run_id]
    Path(result).write_text(json.dumps(outcome), encoding="utf-8")
except BaseException as exc:
    outcome = ["error", type(exc).__name__, str(exc)]
    Path(result).write_text(json.dumps(outcome), encoding="utf-8")
    raise
"""


def _service(tmp_path: Path) -> tuple[RunService, StudioStore, dict[str, object]]:
    store = StudioStore(tmp_path / "store")
    bundle = store.create_bundle({"bundle_path": "bundles/demo", "session_id": "studio-session"})
    sessions: list[FakeSession] = []

    def factory(**kwargs: object) -> FakeSession:
        session = FakeSession(**kwargs)
        sessions.append(session)
        return session

    service = RunService(store, factory)
    return service, store, {"bundle": bundle, "sessions": sessions}


def test_run_lifecycle_persists_readiness_and_releases_slot(tmp_path: Path) -> None:
    service, _, context = _service(tmp_path)
    bundle = context["bundle"]
    run = service.create_run(bundle.id, "headless")
    assert run["status"] == "CREATED"
    assert run["payload"]["artifact_path"].endswith(run["id"])

    prepared = service.prepare(run["id"], expected_revision=run["revision"])
    assert prepared["status"] == "READY"
    assert prepared["payload"]["readiness"]["physics"] == "ready"
    assert prepared["payload"]["sensor_summary"]["imu"] == "ready"

    running = service.start(run["id"], expected_revision=prepared["revision"])
    paused = service.pause(run["id"], expected_revision=running["revision"])
    running_again = service.start(run["id"], expected_revision=paused["revision"])
    reset = service.reset(run["id"], expected_revision=running_again["revision"])
    stopped = service.stop(run["id"], expected_revision=reset["revision"])
    assert stopped["status"] == "STOPPED"
    assert context["sessions"][0].calls == ["prepare", "start", "pause", "start", "reset", "stop"]


def test_run_lifecycle_projects_authoritative_runtime_manifest_readiness(
    tmp_path: Path,
) -> None:
    store = StudioStore(tmp_path / "store")
    bundle = store.create_bundle(
        {"bundle_path": "bundles/demo", "session_id": "studio-session"}
    )

    class ManifestSession(FakeSession):
        def __init__(self, *, artifact_root: Path, run_id: str, **_: object) -> None:
            super().__init__()
            self.artifact_root = Path(artifact_root)
            self.run_id = run_id

        def prepare(self) -> dict[str, object]:
            self.calls.append("prepare")
            manifest = {
                "schema": "lingtu.sim.session-runtime.v1",
                "run_id": self.run_id,
                "session_id": "studio-session",
                "bindings": {
                    "physics": {"state": "ACTIVE"},
                    "visual": {"state": "ACTIVE"},
                    "sensors": {"state": "ACTIVE"},
                    "control": {"state": "ACTIVE"},
                },
                "sensor_streams": {
                    "streams": {
                        "robot.front_rgb": {
                            "stream_id": "robot.front_rgb",
                            "state": "ACTIVE",
                        },
                        "robot.imu": {
                            "stream_id": "robot.imu",
                            "state": "ACTIVE",
                        },
                    }
                },
            }
            (self.artifact_root / "session.runtime.json").write_text(
                json.dumps(manifest),
                encoding="utf-8",
            )
            return {"event": "ready"}

    service = RunService(store, lambda **kwargs: ManifestSession(**kwargs))
    run = service.create_run(bundle.id, "visual")

    prepared = service.prepare(run["id"], expected_revision=run["revision"])

    assert prepared["payload"]["readiness"] == {
        "control": "ACTIVE",
        "physics": "ACTIVE",
        "sensors": "ACTIVE",
        "visual": "ACTIVE",
    }
    assert prepared["payload"]["sensor_summary"] == {
        "robot.front_rgb": "ACTIVE",
        "robot.imu": "ACTIVE",
    }
    service.stop(run["id"], expected_revision=prepared["revision"])


def test_only_one_prepared_run_is_active(tmp_path: Path) -> None:
    service, _, context = _service(tmp_path)
    first = service.create_run(context["bundle"].id, "visual")
    second = service.create_run(context["bundle"].id, "visual")
    service.prepare(first["id"])
    with pytest.raises(ActiveRunConflict):
        service.prepare(second["id"])
    service.stop(first["id"])
    service.prepare(second["id"])


def test_two_independent_service_instances_share_the_runtime_slot(tmp_path: Path) -> None:
    service_a, store, context = _service(tmp_path)
    service_b = RunService(StudioStore(store.root), lambda **_: FakeSession())
    first = service_a.create_run(context["bundle"].id, "visual")
    second = service_a.create_run(context["bundle"].id, "visual")

    service_a.prepare(first["id"])
    with pytest.raises(ActiveRunConflict):
        service_b.prepare(second["id"])
    service_a.stop(first["id"])
    service_b.prepare(second["id"])
    service_b.stop(second["id"])


@pytest.mark.parametrize("sidecar_name", [".run-slot.lock", ".run-slot.json"])
def test_run_service_constructor_rejects_reparse_slot_sidecar(tmp_path: Path, sidecar_name: str) -> None:
    store = StudioStore(tmp_path / "store")
    outside = tmp_path / f"outside-{sidecar_name}"
    outside.write_bytes(b"outside-sentinel")
    try:
        (store.root / sidecar_name).symlink_to(outside)
    except (OSError, NotImplementedError):
        pytest.skip("file symbolic links are unavailable in this Windows test environment")

    with pytest.raises(StoreValidationError):
        RunService(store, lambda **_: FakeSession())

    assert outside.read_bytes() == b"outside-sentinel"


def test_prepare_rejects_file_symlink_runtime_slot_lock(tmp_path: Path) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    outside = tmp_path / "outside-slot.lock"
    outside.write_bytes(b"outside-sentinel")
    lock_path = store.root / ".run-slot.lock"
    try:
        lock_path.symlink_to(outside)
    except (OSError, NotImplementedError):
        pytest.skip("file symbolic links are unavailable in this Windows test environment")

    try:
        with pytest.raises(StoreValidationError):
            service.prepare(run["id"])
    finally:
        current = service.get_run(run["id"])
        if current["status"] == "READY":
            service.stop(run["id"], expected_revision=current["revision"])

    assert outside.read_bytes() == b"outside-sentinel"
    assert context["sessions"] == []


def test_prepare_does_not_create_slot_lock_through_windows_root_junction_race(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction regression test")

    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    root = store.root
    displaced = tmp_path / "displaced-store"
    outside = tmp_path / "outside-store-race"
    outside.mkdir()
    original_open = run_service_module._open_coordination_lock_file
    race_attempted = False
    race_blocked = False
    raced = False

    def swap_root_before_lock_creation(path: Path) -> object:
        nonlocal race_attempted, race_blocked, raced
        if not raced and path == root / ".run-slot.lock":
            race_attempted = True
            try:
                root.rename(displaced)
            except OSError:
                race_blocked = True
                return original_open(path)
            created = subprocess.run(
                ["cmd", "/c", "mklink", "/J", str(root), str(outside)],
                capture_output=True,
                text=True,
                check=False,
            )
            if created.returncode != 0:
                displaced.rename(root)
                pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
            raced = True
        return original_open(path)

    monkeypatch.setattr(
        run_service_module,
        "_open_coordination_lock_file",
        swap_root_before_lock_creation,
    )
    try:
        failure: StoreValidationError | None = None
        try:
            prepared = service.prepare(run["id"], expected_revision=run["revision"])
        except StoreValidationError as exc:
            failure = exc
        assert race_attempted
        assert race_blocked or failure is not None
        assert not (outside / ".run-slot.lock").exists()
        if failure is None:
            service.stop(run["id"], expected_revision=prepared["revision"])
        else:
            assert context["sessions"] == []
    finally:
        if raced:
            os.rmdir(root)
            displaced.rename(root)


def test_prepare_rejects_file_symlink_runtime_slot_record(tmp_path: Path) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    outside = tmp_path / "outside-slot.json"
    outside.write_text('{"outside":"sentinel"}', encoding="utf-8")
    record_path = store.root / ".run-slot.json"
    try:
        record_path.symlink_to(outside)
    except (OSError, NotImplementedError):
        pytest.skip("file symbolic links are unavailable in this Windows test environment")

    try:
        with pytest.raises(StoreValidationError):
            service.prepare(run["id"])
    finally:
        current = service.get_run(run["id"])
        if current["status"] == "READY":
            service.stop(run["id"], expected_revision=current["revision"])

    assert outside.read_text(encoding="utf-8") == '{"outside":"sentinel"}'
    assert context["sessions"] == []


@pytest.mark.parametrize("sidecar_name", [".run-slot.lock", ".run-slot.json"])
def test_prepare_rejects_hardlinked_runtime_slot_coordination_file(
    tmp_path: Path,
    sidecar_name: str,
) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    outside = tmp_path / f"outside-{sidecar_name}"
    sentinel = b"outside-hardlink-sentinel"
    outside.write_bytes(sentinel)
    os.link(outside, store.root / sidecar_name)

    try:
        with pytest.raises(StoreValidationError):
            service.prepare(run["id"], expected_revision=run["revision"])
    finally:
        current = service.get_run(run["id"])
        if current["status"] == "READY":
            service.stop(run["id"], expected_revision=current["revision"])

    assert service.get_run(run["id"])["status"] == "CREATED"
    assert outside.read_bytes() == sentinel
    assert context["sessions"] == []


def test_prepare_revalidates_slot_lock_after_late_hardlink_race(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    lock_path = store.root / ".run-slot.lock"
    outside = tmp_path / "outside-late-slot-hardlink"
    original_assert = run_service_module._assert_open_coordination_file
    linked = False

    def link_after_first_identity_check(handle: object, path: Path) -> None:
        nonlocal linked
        original_assert(handle, path)
        if not linked and path == lock_path:
            os.link(lock_path, outside)
            linked = True

    monkeypatch.setattr(
        run_service_module,
        "_assert_open_coordination_file",
        link_after_first_identity_check,
    )
    try:
        with pytest.raises(StoreValidationError):
            service.prepare(run["id"], expected_revision=run["revision"])
        assert linked
        assert outside.read_bytes() == b""
        assert context["sessions"] == []
    finally:
        current = service.get_run(run["id"])
        if current["status"] == "READY":
            service.stop(run["id"], expected_revision=current["revision"])
        outside.unlink(missing_ok=True)


def test_prepare_revalidates_slot_record_immediately_before_replace(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    record_path = store.root / ".run-slot.json"
    outside = tmp_path / "outside-raced-slot.json"
    outside.write_text('{"outside":"sentinel"}', encoding="utf-8")
    original_dump = run_service_module.json.dump
    raced = False

    def race_record_path(value: object, handle: object, *args: object, **kwargs: object) -> object:
        nonlocal raced
        if not raced:
            try:
                record_path.symlink_to(outside)
            except (OSError, NotImplementedError):
                pytest.skip("file symbolic links are unavailable in this Windows test environment")
            raced = True
        return original_dump(value, handle, *args, **kwargs)

    monkeypatch.setattr(run_service_module.json, "dump", race_record_path)
    try:
        with pytest.raises(StoreValidationError):
            service.prepare(run["id"])
    finally:
        current = service.get_run(run["id"])
        if current["status"] == "READY":
            service.stop(run["id"], expected_revision=current["revision"])

    assert raced
    assert outside.read_text(encoding="utf-8") == '{"outside":"sentinel"}'
    assert context["sessions"] == []


@pytest.mark.parametrize(
    ("operation", "idempotency_key"),
    [
        ("prepare", None),
        ("start", "polluted-lifecycle"),
    ],
)
def test_lifecycle_revalidates_coordination_paths_immediately_before_open(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    operation: str,
    idempotency_key: str | None,
) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    if operation == "start":
        prepared = service.prepare(run["id"])
        expected_revision = prepared["revision"]
    else:
        expected_revision = run["revision"]
    watched_path = store.root / ".run-slot.lock"
    if idempotency_key is not None:
        watched_path = store.root / ".run-idempotency" / f"{uuid.uuid5(uuid.NAMESPACE_URL, idempotency_key).hex}.lock"
    original_assert = StudioStore._assert_no_reparse_components
    original_open = run_service_module._open_coordination_lock_file
    clean_validations = 0
    armed = False

    def fake_assert(path: Path, *, below: Path | None = None) -> None:
        nonlocal armed, clean_validations
        candidate = Path(path)
        if candidate == watched_path:
            clean_validations += 1
            if armed:
                raise StoreValidationError(f"owned path contains a reparse point: {candidate}")
        original_assert(path, below=below)
        if candidate == watched_path and clean_validations == 2:
            armed = True

    def open_fails_if_polluted(path: Path) -> object:
        if armed and path == watched_path:
            raise AssertionError(f"opened polluted coordination path: {path}")
        return original_open(path)

    monkeypatch.setattr(StudioStore, "_assert_no_reparse_components", classmethod(lambda cls, path, *, below=None: fake_assert(path, below=below)))
    monkeypatch.setattr(
        run_service_module,
        "_open_coordination_lock_file",
        open_fails_if_polluted,
    )

    with pytest.raises(StoreValidationError):
        if operation == "prepare":
            service.prepare(run["id"], expected_revision=expected_revision)
        else:
            service.start(
                run["id"],
                expected_revision=expected_revision,
                idempotency_key=idempotency_key,
            )

    current = service.get_run(run["id"])
    assert current["status"] == ("READY" if operation == "start" else "CREATED")
    assert context["sessions"][0].calls == ["prepare"] if operation == "start" else context["sessions"] == []


def test_prepare_does_not_create_idempotency_directory_through_windows_root_junction_race(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction regression test")

    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    root = store.root
    coordination_parent = root / ".run-idempotency"
    displaced = tmp_path / "displaced-store-idempotency"
    outside = tmp_path / "outside-idempotency-directory-race"
    outside.mkdir()
    original_mkdir = Path.mkdir
    race_attempted = False
    race_blocked = False
    raced = False

    def swap_root_before_directory_creation(self: Path, *args: object, **kwargs: object) -> object:
        nonlocal race_attempted, race_blocked, raced
        if not raced and self == coordination_parent:
            race_attempted = True
            try:
                root.rename(displaced)
            except OSError:
                race_blocked = True
            else:
                created = subprocess.run(
                    ["cmd", "/c", "mklink", "/J", str(root), str(outside)],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if created.returncode != 0:
                    displaced.rename(root)
                    pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
                raced = True
        return original_mkdir(self, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", swap_root_before_directory_creation)
    try:
        failure: StoreValidationError | None = None
        try:
            prepared = service.prepare(
                run["id"],
                expected_revision=run["revision"],
                idempotency_key="directory-junction-race",
            )
        except StoreValidationError as exc:
            failure = exc
        assert race_attempted
        assert race_blocked or failure is not None
        assert not (outside / ".run-idempotency").exists()
        if failure is None:
            service.stop(run["id"], expected_revision=prepared["revision"])
        else:
            assert context["sessions"] == []
    finally:
        if raced:
            os.rmdir(root)
            displaced.rename(root)


def test_prepare_holds_windows_coordination_parent_against_junction_swap(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows coordination-parent handle regression test")

    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    coordination_parent = store.root / ".run-idempotency"
    displaced_parent = tmp_path / "displaced-idempotency"
    outside = tmp_path / "outside-idempotency-swap"
    outside.mkdir()
    original_replace = run_service_module.os.replace
    race_attempted = False
    race_blocked = False

    def acquire_without_file_handle(lock: object) -> None:
        Path(lock.path).parent.mkdir(parents=True, exist_ok=True)

    def replace_with_parent_swap(source: object, destination: object, *args: object, **kwargs: object) -> object:
        nonlocal race_attempted, race_blocked
        target = Path(destination)
        if not race_attempted and target.parent == coordination_parent:
            race_attempted = True
            try:
                coordination_parent.rename(displaced_parent)
            except OSError:
                race_blocked = True
            else:
                displaced_parent.rename(coordination_parent)
                raise AssertionError("coordination parent was swappable immediately before replace")
        return original_replace(source, destination, *args, **kwargs)

    monkeypatch.setattr(run_service_module._AdvisoryFileLock, "acquire", acquire_without_file_handle)
    monkeypatch.setattr(run_service_module.os, "replace", replace_with_parent_swap)

    prepared = service.prepare(
        run["id"],
        expected_revision=run["revision"],
        idempotency_key="parent-junction-swap",
    )
    assert prepared["status"] == "READY"
    assert race_attempted
    assert race_blocked
    assert list(outside.iterdir()) == []
    service.stop(run["id"], expected_revision=prepared["revision"])


def test_independent_service_cannot_stop_active_run_it_does_not_own(tmp_path: Path) -> None:
    service_a, store, context = _service(tmp_path)
    service_b = RunService(StudioStore(store.root), lambda **_: FakeSession())
    run = service_a.create_run(context["bundle"].id, "visual")
    prepared = service_a.prepare(run["id"])

    with pytest.raises(ActiveRunConflict):
        service_b.stop(run["id"], expected_revision=prepared["revision"])

    unchanged = store.get_run(run["id"])
    assert unchanged.status == "READY"
    assert unchanged.revision == prepared["revision"]
    assert context["sessions"][0].calls == ["prepare"]
    service_a.stop(run["id"], expected_revision=prepared["revision"])


def test_multiprocess_prepare_competition_has_one_winner(tmp_path: Path) -> None:
    service, store, context = _service(tmp_path)
    first = service.create_run(context["bundle"].id, "visual")
    second = service.create_run(context["bundle"].id, "visual")
    barrier = tmp_path / "prepare.barrier"
    release = tmp_path / "prepare.release"
    result_paths = [tmp_path / "result-a.json", tmp_path / "result-b.json"]
    processes = [
        subprocess.Popen(
            [
                sys.executable,
                "-c",
                _COMPETE_WORKER,
                str(store.root),
                run["id"],
                str(barrier),
                str(release),
                str(result_path),
            ],
            cwd=Path.cwd(),
        )
        for run, result_path in zip((first, second), result_paths)
    ]
    barrier.write_text("go", encoding="utf-8")
    deadline = time.monotonic() + 20
    while any(not path.exists() for path in result_paths) and time.monotonic() < deadline:
        time.sleep(0.01)
    outcomes = [json.loads(path.read_text(encoding="utf-8")) for path in result_paths]
    assert sorted(outcome[0] for outcome in outcomes) == ["conflict", "ready"]
    release.write_text("stop", encoding="utf-8")
    for process in processes:
        assert process.wait(timeout=20) == 0


def test_lifecycle_replays_before_revision_or_session_side_effects(tmp_path: Path) -> None:
    service, _, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    prepared = service.prepare(run["id"])
    first = service.start(run["id"], expected_revision=prepared["revision"], idempotency_key="start-timeout")
    replay = service.start(run["id"], expected_revision=prepared["revision"], idempotency_key="start-timeout")
    assert replay == first
    assert context["sessions"][0].calls == ["prepare", "start"]
    with pytest.raises(IdempotencyConflict):
        service.start(run["id"], expected_revision=first["revision"], idempotency_key="start-timeout")
    with pytest.raises(IdempotencyConflict):
        service.pause(run["id"], expected_revision=first["revision"], idempotency_key="start-timeout")
    other = service.create_run(context["bundle"].id, "headless")
    with pytest.raises(IdempotencyConflict):
        service.start(other["id"], expected_revision=other["revision"], idempotency_key="start-timeout")
    assert context["sessions"][0].calls == ["prepare", "start"]


def test_lifecycle_rejects_windows_junction_idempotency_directory(tmp_path: Path) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction regression test")
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    prepared = service.prepare(run["id"])
    outside = tmp_path / "outside-idempotency"
    outside.mkdir()
    junction = store.root / ".run-idempotency"
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(junction), str(outside)],
        capture_output=True,
        text=True,
        check=False,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    try:
        with pytest.raises(StoreValidationError):
            service.start(
                run["id"],
                expected_revision=prepared["revision"],
                idempotency_key="junction-escape",
            )
        assert list(outside.iterdir()) == []
        assert context["sessions"][0].calls == ["prepare"]
    finally:
        junction.rmdir()
        current = service.get_run(run["id"])
        service.stop(run["id"], expected_revision=current["revision"])


def test_prepare_replays_final_ready_without_second_session(tmp_path: Path) -> None:
    service, _, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")

    first = service.prepare(run["id"], expected_revision=run["revision"], idempotency_key="prepare-timeout")
    replay = service.prepare(run["id"], expected_revision=run["revision"], idempotency_key="prepare-timeout")

    assert replay == first
    assert first["status"] == "READY"
    assert first["revision"] == run["revision"] + 2
    assert len(context["sessions"]) == 1
    assert context["sessions"][0].calls == ["prepare"]
    with pytest.raises(IdempotencyConflict):
        service.prepare(run["id"], expected_revision=first["revision"], idempotency_key="prepare-timeout")
    with pytest.raises(IdempotencyConflict):
        service.start(run["id"], expected_revision=first["revision"], idempotency_key="prepare-timeout")
    other = service.create_run(context["bundle"].id, "headless")
    with pytest.raises(IdempotencyConflict):
        service.prepare(other["id"], expected_revision=other["revision"], idempotency_key="prepare-timeout")


def test_lifecycle_store_record_without_marker_does_not_cross_replay(tmp_path: Path) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    prepared = service.prepare(run["id"])
    service.start(run["id"], expected_revision=prepared["revision"], idempotency_key="lost-marker")
    marker = store.root / ".run-idempotency" / f"{uuid.uuid5(uuid.NAMESPACE_URL, 'lost-marker').hex}.json"
    marker.unlink()
    with pytest.raises(IdempotencyConflict):
        service.reset(run["id"], expected_revision=prepared["revision"], idempotency_key="lost-marker")


def test_incomplete_lifecycle_marker_is_not_replayed_after_state_changed(tmp_path: Path) -> None:
    service, store, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    prepared = service.prepare(run["id"])
    marker = store.root / ".run-idempotency" / f"{uuid.uuid5(uuid.NAMESPACE_URL, 'pending-start').hex}.json"
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(
        json.dumps(
            {
                "state": "pending",
                "operation": "start",
                "run_id": run["id"],
                "expected_revision": prepared["revision"],
                "initial_revision": prepared["revision"],
                "initial_status": "READY",
                "response": {"status": "RUNNING"},
            }
        ),
        encoding="utf-8",
    )
    service.start(run["id"], expected_revision=prepared["revision"])
    with pytest.raises(IdempotencyConflict):
        service.start(run["id"], expected_revision=prepared["revision"], idempotency_key="pending-start")


def test_run_artifact_directory_rejects_reparse_component(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    ids = iter(["b" * 32, "a" * 32])
    store = StudioStore(tmp_path / "store", id_factory=lambda: next(ids))
    bundle = store.create_bundle({"bundle_path": "bundles/demo", "session_id": "studio-session"})
    service = RunService(store, lambda **_: FakeSession())
    blocked = store.root / "artifacts" / "runs" / ("a" * 32)
    blocked.parent.mkdir(parents=True, exist_ok=True)
    original = StudioStore._is_reparse_point

    def fake_reparse(path: Path) -> bool:
        if Path(path) == blocked:
            return True
        return original(path)

    monkeypatch.setattr(StudioStore, "_is_reparse_point", staticmethod(fake_reparse))
    with pytest.raises(StoreValidationError):
        service.create_run(bundle.id, "headless")


def test_create_run_does_not_create_artifact_directory_through_windows_junction_race(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if sys.platform != "win32":
        pytest.skip("Windows junction regression test")

    service, store, context = _service(tmp_path)
    runs_root = service.artifact_root
    displaced = tmp_path / "displaced-runs-root"
    outside = tmp_path / "outside-runs-root-race"
    outside.mkdir()
    original_mkdir = Path.mkdir
    race_attempted = False
    race_blocked = False
    raced = False

    def swap_runs_root_before_run_directory_creation(self: Path, *args: object, **kwargs: object) -> object:
        nonlocal race_attempted, race_blocked, raced
        if not raced and self.parent == runs_root:
            race_attempted = True
            try:
                runs_root.rename(displaced)
            except OSError:
                race_blocked = True
            else:
                created = subprocess.run(
                    ["cmd", "/c", "mklink", "/J", str(runs_root), str(outside)],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if created.returncode != 0:
                    displaced.rename(runs_root)
                    pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")
                raced = True
        return original_mkdir(self, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", swap_runs_root_before_run_directory_creation)
    try:
        failure: StoreValidationError | None = None
        try:
            created_run = service.create_run(context["bundle"].id, "headless")
        except StoreValidationError as exc:
            failure = exc
        assert race_attempted
        assert race_blocked or failure is not None
        assert list(outside.iterdir()) == []
        if failure is None:
            assert (runs_root / created_run["id"]).is_dir()
    finally:
        if raced:
            os.rmdir(runs_root)
            displaced.rename(runs_root)


def test_revision_and_idempotency_are_enforced(tmp_path: Path) -> None:
    service, _, context = _service(tmp_path)
    first = service.create_run(context["bundle"].id, "headless", idempotency_key="create-1")
    replay = service.create_run(context["bundle"].id, "headless", idempotency_key="create-1")
    assert replay["id"] == first["id"]
    assert replay["revision"] == first["revision"]
    with pytest.raises(RevisionConflict):
        service.prepare(first["id"], expected_revision=1)


def test_invalid_transition_does_not_create_a_session(tmp_path: Path) -> None:
    service, _, context = _service(tmp_path)
    run = service.create_run(context["bundle"].id, "headless")
    with pytest.raises(RunStateError):
        service.start(run["id"])
    assert context["sessions"] == []


def test_runtime_failure_is_persisted_and_session_is_cleaned(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "store")
    bundle = store.create_bundle({"bundle_path": "bundles/demo"})
    sessions: list[FailingStartSession] = []

    def factory(**_: object) -> FailingStartSession:
        session = FailingStartSession()
        sessions.append(session)
        return session

    service = RunService(store, factory)
    run = service.create_run(bundle.id, "headless")
    prepared = service.prepare(run["id"])
    with pytest.raises(RuntimeError, match="start failed"):
        service.start(run["id"], expected_revision=prepared["revision"], idempotency_key="failed-start")
    failed = service.get_run(run["id"])
    assert failed["status"] == "FAILED"
    assert failed["payload"]["failure"]["message"] == "start failed"
    assert sessions[0].calls == ["prepare", "start", "stop"]

    replacement = service.create_run(bundle.id, "headless")
    service.prepare(replacement["id"])
    service.stop(replacement["id"])
