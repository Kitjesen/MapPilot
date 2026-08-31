# ruff: noqa
"""Focused contract tests for the isolated SimStudio JSON store."""

from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
from threading import Barrier
import uuid

import pytest
from tools.simstudio.service.models import (
    IdempotencyConflict,
    RecordNotFound,
    RevisionConflict,
    StoreValidationError,
    canonical_digest,
    canonical_json,
)
from tools.simstudio.service.store import StudioStore


def test_canonical_json_is_stable_and_rejects_non_json_values() -> None:
    assert canonical_json({"b": 2, "a": ["灵途", 1]}) == '{"a":["灵途",1],"b":2}'
    assert canonical_digest({"a": 1}) == canonical_digest({"a": 1})
    with pytest.raises(StoreValidationError):
        canonical_json({"value": float("nan")})


def test_records_are_atomic_durable_and_cas_versioned(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    created = store.create_session_draft({"session_id": "demo", "robots": []})
    assert len(created.id) == 32
    assert "/" not in created.id and "\\" not in created.id
    path = store.record_path("session_draft", created.id, must_exist=True)
    assert json.loads(path.read_text(encoding="utf-8"))["revision"] == 1
    assert not list(path.parent.glob("*.tmp"))

    restarted = StudioStore(tmp_path / "studio")
    loaded = restarted.get_session_draft(created.id)
    assert loaded.to_dict() == created.to_dict()
    updated = restarted.update_session_draft(
        created.id,
        expected_revision=1,
        payload={"session_id": "demo", "robots": [{"id": "thunder_01"}]},
    )
    assert updated.revision == 2
    with pytest.raises(RevisionConflict) as error:
        restarted.update_session_draft(created.id, expected_revision=1, payload={"stale": True})
    assert error.value.expected == 1
    assert error.value.actual == 2


def test_idempotency_replays_same_resource_and_rejects_different_request(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    first = store.create_import_job(
        {"source_path": "inbox/job.zip"},
        idempotency_key="import-001",
    )
    second = store.create_import_job(
        {"source_path": "inbox/job.zip"},
        idempotency_key="import-001",
    )
    assert second.id == first.id
    assert len(store.list_import_jobs()) == 1
    record = store.get_idempotency("create:import_job", "import-001")
    assert record is not None
    assert record.resource_id == first.id
    with pytest.raises(IdempotencyConflict):
        store.create_import_job({"source_path": "inbox/other.zip"}, idempotency_key="import-001")


def test_all_studio_resource_records_are_typed_and_sorted(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    job = store.create_import_job({"source_path": "job.zip"})
    draft = store.create_session_draft({"session_id": "s"})
    bundle = store.create_bundle({"bundle_path": "s-001"})
    run = store.create_run({"artifact_path": "run-001"})
    assert store.get_import_job(job.id).kind == "import_job"
    assert store.get_session_draft(draft.id).kind == "session_draft"
    assert store.get_bundle(bundle.id).kind == "bundle"
    assert store.get_run(run.id).kind == "run"
    with pytest.raises(RecordNotFound):
        store.get_run("0" * 32)


@pytest.mark.parametrize(
    "value",
    [
        "../escape.json",
        "a/../../escape.json",
        "C:/absolute.json",
        "/absolute.json",
        "a\\b.json",
        "artifact.json:secret",
        "a/./b.json",
        "a b.json",
    ],
)
def test_owned_paths_reject_traversal_absolute_windows_ads_and_ambiguous_names(tmp_path: Path, value: str) -> None:
    store = StudioStore(tmp_path / "studio")
    with pytest.raises(StoreValidationError):
        store.owned_path("inbox", value)


def test_owned_paths_reject_symlink_components(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    link = store.root / "inbox" / "linked"
    try:
        link.symlink_to(tmp_path, target_is_directory=True)
    except (OSError, NotImplementedError):
        pytest.skip("symbolic links are unavailable in this Windows test environment")
    with pytest.raises(StoreValidationError):
        store.owned_path("inbox", "linked/file.json")


def test_record_payload_paths_are_area_relative_and_not_arbitrary_filesystem_paths(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    with pytest.raises(StoreValidationError):
        store.create_run({"artifact_path": "D:/outside"})
    with pytest.raises(StoreValidationError):
        store.create_bundle({"bundle_path": "../outside"})


def test_nested_store_reparse_components_are_rejected(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    for area in ("session_draft", "idempotency"):
        link = store._roots[area] / "nested-link"
        try:
            link.symlink_to(foreign, target_is_directory=True)
        except (OSError, NotImplementedError):
            pytest.skip("directory symbolic links are unavailable in this Windows test environment")
        try:
            with pytest.raises(StoreValidationError):
                store.owned_path(area, "nested-link/escape.json")
        finally:
            link.unlink(missing_ok=True)


def test_records_root_reparse_replacement_is_rejected(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    records = store.root / "records"
    backup = store.root / "records-real"
    foreign = tmp_path / "foreign-records"
    foreign.mkdir()
    records.rename(backup)
    linked = False
    try:
        try:
            records.symlink_to(foreign, target_is_directory=True)
            linked = True
        except (OSError, NotImplementedError):
            backup.rename(records)
            pytest.skip("directory symbolic links are unavailable in this Windows test environment")
        with pytest.raises(StoreValidationError):
            store.owned_path("session_draft", (Path("nested") / "record.json").as_posix())
    finally:
        if linked:
            records.unlink(missing_ok=True)
        if backup.exists():
            backup.rename(records)


def test_two_store_instances_serialize_idempotent_create_and_cas(tmp_path: Path) -> None:
    root = tmp_path / "studio"
    first_store = StudioStore(root)
    second_store = StudioStore(root)
    barrier = Barrier(2)

    def create(store: StudioStore):
        barrier.wait()
        return store.create_session_draft({"session_id": "race"}, idempotency_key="same-request")

    with ThreadPoolExecutor(max_workers=2) as executor:
        created = list(executor.map(create, (first_store, second_store)))
    assert {record.id for record in created} == {created[0].id}
    assert len(first_store.list_session_drafts()) == 1

    record = first_store.get_session_draft(created[0].id)
    update_barrier = Barrier(2)

    def update(store: StudioStore):
        update_barrier.wait()
        try:
            return store.update_session_draft(
                record.id,
                expected_revision=1,
                payload={"session_id": "race", "winner": "one"},
            )
        except Exception as exc:  # return the race outcome for both workers
            return exc

    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(update, store) for store in (first_store, second_store)]
        results = [future.result() for future in futures]
    assert sum(result.__class__.__name__ == "SessionDraftRecord" for result in results) == 1
    assert sum(isinstance(result, RevisionConflict) for result in results) == 1
    assert first_store.get_session_draft(record.id).revision == 2


def test_two_store_processes_serialize_idempotent_create(tmp_path: Path) -> None:
    root = tmp_path / "studio"
    script = (
        "from pathlib import Path; import sys; "
        "from tools.simstudio.service.store import StudioStore; "
        "record = StudioStore(Path(sys.argv[1])).create_session_draft("
        "{'session_id': 'process'}, idempotency_key='process-race'); "
        "print(record.id, flush=True)"
    )
    processes = [
        subprocess.Popen(
            [sys.executable, "-c", script, str(root)],
            cwd=Path.cwd(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        for _ in range(2)
    ]
    outputs = []
    for process in processes:
        stdout, stderr = process.communicate(timeout=30)
        assert process.returncode == 0, stderr
        outputs.append(stdout.strip())
    assert outputs[0] == outputs[1]
    assert len(StudioStore(root).list_session_drafts()) == 1


def test_interrupted_multi_file_commit_is_recovered_on_restart(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = tmp_path / "studio"
    store = StudioStore(root)
    real_atomic_write = store._atomic_write
    interrupted = False

    def crash_after_resource(path: Path, value: object) -> None:
        nonlocal interrupted
        real_atomic_write(path, value)
        if path.parent == store._roots["session_draft"] and not interrupted:
            interrupted = True
            raise RuntimeError("simulated process crash")

    monkeypatch.setattr(store, "_atomic_write", crash_after_resource)
    with pytest.raises(RuntimeError, match="simulated process crash"):
        store.create_session_draft({"session_id": "recover"}, idempotency_key="recover-request")
    assert list((root / ".transactions").glob("*.json"))

    restarted = StudioStore(root)
    recovered = restarted.create_session_draft({"session_id": "recover"}, idempotency_key="recover-request")
    assert len(restarted.list_session_drafts()) == 1
    assert restarted.get_idempotency("create:session_draft", "recover-request").resource_id == recovered.id
    assert not list((root / ".transactions").glob("*.json"))


def test_nested_managed_paths_reject_escape_but_ue_identifiers_remain_valid(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    with pytest.raises(StoreValidationError):
        store.create_run({"metadata": {"nested_path": "D:/outside"}})
    with pytest.raises(StoreValidationError):
        store.create_run({"metadata": {"filesystem_path": "D:/outside"}})
    accepted = store.create_bundle({"visual": {"asset_path": "/Game/RobotSim/Maps/OpenField"}})
    assert accepted.payload["visual"]["asset_path"].startswith("/Game/")


def test_nested_draft_never_grants_an_absolute_path_exception(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    with pytest.raises(StoreValidationError):
        store.create_import_job(
            {"draft": {"root": "D:/outside", "locations": {"package": "package"}}}
        )
    with pytest.raises(StoreValidationError):
        store.create_import_job({"draft": {"metadata": {"source": "C:\\outside\\file"}}})


def test_reads_revalidate_tampered_record_envelope_and_payload(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    record = store.create_run({"artifact_path": "artifacts/runs/run"})
    path = store.record_path("run", record.id, must_exist=True)
    tampered = record.to_dict()
    tampered["payload"] = {"draft": {"root": "D:/outside"}}
    path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(StoreValidationError):
        store.get_run(record.id)

    tampered = record.to_dict()
    tampered["kind"] = "bundle"
    path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(StoreValidationError):
        store.get_run(record.id)


def test_transaction_recovery_revalidates_record_before_replaying(tmp_path: Path) -> None:
    root = tmp_path / "studio"
    store = StudioStore(root)
    record_id = "a" * 32
    transaction_id = uuid.uuid4().hex
    journal = {
        "schema": "lingtu.sim.studio.store-transaction.v1",
        "id": transaction_id,
        "writes": [
            {
                "area": "run",
                "relative": f"{record_id}.json",
                "value": {
                    "schema": "lingtu.sim.studio.run.v1",
                    "kind": "run",
                    "id": record_id,
                    "revision": 1,
                    "created_at": "2026-01-01T00:00:00Z",
                    "updated_at": "2026-01-01T00:00:00Z",
                    "status": "created",
                    "payload": {"draft": {"root": "D:/outside"}},
                },
            }
        ],
    }
    journal_path = root / ".transactions" / f"{transaction_id}.json"
    journal_path.write_text(json.dumps(journal), encoding="utf-8")
    with pytest.raises(StoreValidationError):
        StudioStore(root)
    assert not store.record_path("run", record_id).exists()


@pytest.mark.parametrize("field", ["asset_id", "asset_path", "primary_asset_id", "ue_asset"])
@pytest.mark.parametrize("prefix", ["/Game/", "/Engine/", "/Script/"])
def test_explicit_ue_identifier_fields_allow_engine_namespace_ids(
    tmp_path: Path, field: str, prefix: str
) -> None:
    store = StudioStore(tmp_path / "studio")
    record = store.create_bundle({"visual": {field: f"{prefix}RobotSim/Thing"}})
    assert record.payload["visual"][field].startswith(prefix)


def test_non_ue_fields_cannot_smuggle_engine_namespace_as_a_path(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    with pytest.raises(StoreValidationError):
        store.create_bundle({"metadata": {"source": "/Game/RobotSim/Thing"}})


@pytest.mark.parametrize("field", ["metadata_asset_path", "untrusted_asset_id", "custom_asset_path"])
def test_ue_namespace_is_allowed_only_in_explicit_semantic_fields(tmp_path: Path, field: str) -> None:
    store = StudioStore(tmp_path / "studio")
    with pytest.raises(StoreValidationError):
        store.create_bundle({"metadata": {field: "/Game/RobotSim/Thing"}})


def test_reads_revalidate_tampered_idempotency_index(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "studio")
    record = store.create_session_draft({"session_id": "stable"}, idempotency_key="stable-request")
    index_path = store.owned_path(
        "idempotency",
        store._idempotency_relative("create:session_draft", "stable-request"),
        must_exist=True,
    )
    tampered = json.loads(index_path.read_text(encoding="utf-8"))
    tampered["operation"] = "create:run"
    index_path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(StoreValidationError):
        store.get_idempotency("create:session_draft", "stable-request")
    assert store.get_session_draft(record.id).id == record.id
