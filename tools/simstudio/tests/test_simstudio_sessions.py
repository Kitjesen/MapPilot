# ruff: noqa: D103, S101
"""Contract tests for the isolated SimStudio session authoring service."""

from __future__ import annotations

import copy
import json
import multiprocessing
import threading
import time
from collections.abc import Mapping
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import pytest
from sim.catalog import CatalogError, CatalogResolver, SessionComposer, SimCatalog
from tools.simstudio.service.models import RevisionConflict, StoreValidationError
from tools.simstudio.service.session_service import SessionAuthoringService
from tools.simstudio.service.store import StudioStore

REPO_ROOT = Path(__file__).resolve().parents[3]


def _intent() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.session-intent.v1",
        "session": {
            "session_id": "studio_contract",
            "mujoco_version": "3.10.0",
            "seed": 7,
            "world": "open_field@1.0.0",
            "robots": [
                {
                    "instance_id": "robot_01",
                    "package": "omni_cart@1.0.0",
                    "spawn": {
                        "position_m": [0.0, 0.0, 0.0],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    },
                }
            ],
            "runtime": {
                "backend": "mujoco",
                "mode": "headless",
                "required_bindings": ["physics"],
            },
        },
    }


def _service(tmp_path: Path) -> SessionAuthoringService:
    return _service_at_store_root(tmp_path / "store")


def _service_at_store_root(store_root: Path) -> SessionAuthoringService:
    store = StudioStore(store_root)
    resolver = CatalogResolver.from_repository(REPO_ROOT)
    composer = SessionComposer(resolver, artifact_root=store_root / "untrusted-composer-root")
    return SessionAuthoringService(store, composer, SimCatalog(resolver))


def _write_process_result(path: Path, payload: Mapping[str, Any]) -> None:
    """Publish one child result through a replace-once filesystem marker."""

    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def _wait_for_files(paths: list[Path], *, timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while not all(path.is_file() for path in paths):
        if time.monotonic() >= deadline:
            missing = [str(path) for path in paths if not path.is_file()]
            raise AssertionError(f"timed out waiting for child markers: {missing}")
        time.sleep(0.01)


def _compose_in_child(
    store_root: str,
    draft_id: str,
    ready_path: str,
    release_path: str,
    result_path: str,
) -> None:
    """Compose through a fresh process using only filesystem coordination."""

    try:
        ready = Path(ready_path)
        release = Path(release_path)
        _write_process_result(ready, {"status": "ready"})
        deadline = time.monotonic() + 30.0
        while not release.is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"timed out waiting for release marker: {release}")
            time.sleep(0.01)
        bundle = _service_at_store_root(Path(store_root)).compose_session(draft_id, revision=1)
        _write_process_result(Path(result_path), {"status": "ok", "bundle": bundle})
    except Exception as exc:  # pragma: no cover - surfaced by the parent assertion
        _write_process_result(Path(result_path), {"status": "error", "error": repr(exc)})


def test_create_update_and_list_drafts_use_opaque_ids_and_cas(tmp_path: Path) -> None:
    service = _service(tmp_path)
    first = service.create_session_draft(_intent())

    assert len(first["id"]) == 32
    assert first["revision"] == 1
    assert set(first["payload"]) == {"schema", "intent"}
    assert first["payload"]["intent"]["session"]["session_id"] == "studio_contract"
    assert service.list_session_drafts()[0]["id"] == first["id"]

    changed = copy.deepcopy(_intent())
    changed["session"]["seed"] = 11
    updated = service.update_session_draft(first["id"], revision=1, intent=changed)
    assert updated["revision"] == 2
    with pytest.raises(RevisionConflict):
        service.update_session_draft(first["id"], revision=1, intent=_intent())


def test_intent_rejects_paths_outputs_and_unsafe_user_ids(tmp_path: Path) -> None:
    service = _service(tmp_path)
    invalid = copy.deepcopy(_intent())
    invalid["session"]["output_dir"] = "outside"
    with pytest.raises(StoreValidationError):
        service.create_session_draft(invalid)

    invalid = copy.deepcopy(_intent())
    invalid["session"]["session_id"] = "../outside"
    with pytest.raises(StoreValidationError):
        service.create_session_draft(invalid)

    invalid = copy.deepcopy(_intent())
    invalid["session"]["robots"][0]["instance_id"] = "robot/01"
    with pytest.raises(StoreValidationError):
        service.create_session_draft(invalid)


def test_compose_persists_session_id_and_service_owned_relative_artifacts(tmp_path: Path) -> None:
    service = _service(tmp_path)
    draft = service.create_session_draft(_intent())
    bundle = service.compose_session(draft["id"], revision=1)

    payload = bundle["payload"]
    assert payload["draft_id"] == draft["id"]
    assert payload["session_id"] == "studio_contract"
    assert set(payload) == {
        "schema",
        "draft_id",
        "draft_revision",
        "session_id",
        "bundle_path",
        "artifacts",
    }
    assert payload["bundle_path"] == f"bundles/bundle-{draft['id']}-1"
    assert all(not Path(path).is_absolute() for path in payload["artifacts"])
    assert (service.store.root / payload["bundle_path"] / "session.yaml").is_file()
    assert service.get_bundle(bundle["id"])["payload"] == payload

    # Composition is deterministic and does not create a second bundle for
    # the same draft revision.
    assert service.compose_session(draft["id"])["id"] == bundle["id"]


def test_unknown_package_reference_is_rejected_before_draft_creation(tmp_path: Path) -> None:
    service = _service(tmp_path)
    invalid = copy.deepcopy(_intent())
    invalid["session"]["robots"][0]["package"] = "does_not_exist@1.0.0"
    with pytest.raises(CatalogError):
        service.create_session_draft(invalid)
    assert service.list_bundles() == []
    assert not list(service.bundles_root.glob("bundle-*"))


def test_compose_serializes_same_draft_revision_across_threads(tmp_path: Path) -> None:
    service = _service(tmp_path)
    draft = service.create_session_draft(_intent())
    compose_started = threading.Event()
    compose_calls = 0
    calls_lock = threading.Lock()
    original_compose = service.composer.compose

    def delayed_compose(intent: Any, *, output_dir: Path) -> Any:
        nonlocal compose_calls
        with calls_lock:
            compose_calls += 1
        compose_started.set()
        return original_compose(intent, output_dir=output_dir)

    service.composer.compose = delayed_compose
    with ThreadPoolExecutor(max_workers=2) as executor:
        first = executor.submit(service.compose_session, draft["id"], revision=1)
        assert compose_started.wait(timeout=5)
        second = executor.submit(service.compose_session, draft["id"], revision=1)
        results = [first.result(timeout=10), second.result(timeout=10)]

    assert results[0] == results[1]
    assert compose_calls == 1
    assert len(service.list_bundles()) == 1


def test_compose_serializes_independent_services_sharing_store_root(tmp_path: Path) -> None:
    first_service = _service(tmp_path)
    second_service = _service(tmp_path)
    draft = first_service.create_session_draft(_intent())
    calls = [0, 0]
    calls_lock = threading.Lock()

    for index, service in enumerate((first_service, second_service)):
        original_compose = service.composer.compose

        def counted_compose(intent: Any, *, output_dir: Path, _original=original_compose, _index=index) -> Any:
            with calls_lock:
                calls[_index] += 1
            return _original(intent, output_dir=output_dir)

        service.composer.compose = counted_compose

    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(
            executor.map(
                lambda service: service.compose_session(draft["id"], revision=1),
                (first_service, second_service),
            )
        )

    assert results[0] == results[1]
    assert sum(calls) == 1
    assert len(first_service.list_bundles()) == 1


def test_compose_serializes_independent_processes_sharing_store_root(tmp_path: Path) -> None:
    service = _service(tmp_path)
    draft = service.create_session_draft(_intent())
    context = multiprocessing.get_context("spawn")
    coordination = tmp_path / "process-coordination"
    coordination.mkdir()
    release_path = coordination / "release.marker"
    child_paths = [
        (
            coordination / f"child-{index}.ready",
            coordination / f"child-{index}.result.json",
        )
        for index in range(2)
    ]
    processes = [
        context.Process(
            target=_compose_in_child,
            args=(
                str(service.store.root),
                draft["id"],
                str(ready_path),
                str(release_path),
                str(result_path),
            ),
        )
        for ready_path, result_path in child_paths
    ]
    try:
        for process in processes:
            process.start()
        _wait_for_files([ready_path for ready_path, _ in child_paths])
        release_path.write_text("release\n", encoding="utf-8")
        _wait_for_files([result_path for _, result_path in child_paths])
        for process in processes:
            process.join(timeout=30)

        results = [json.loads(result_path.read_text(encoding="utf-8")) for _, result_path in child_paths]
    finally:
        for process in processes:
            if process.is_alive():
                process.terminate()
            process.join(timeout=5)

    assert all(process.exitcode == 0 for process in processes)
    assert all(result["status"] == "ok" for result in results), results
    assert results[0]["bundle"] == results[1]["bundle"]
    assert len(service.list_bundles()) == 1


def test_compose_rejects_a_conflicting_draft_revision(tmp_path: Path) -> None:
    service = _service(tmp_path)
    draft = service.create_session_draft(_intent())
    changed = copy.deepcopy(_intent())
    changed["session"]["seed"] = 13
    service.update_session_draft(draft["id"], revision=1, intent=changed)

    with pytest.raises(RevisionConflict):
        service.compose_session(draft["id"], revision=1)
    assert service.list_bundles() == []


def test_compose_failure_releases_serialization_for_retry(tmp_path: Path) -> None:
    service = _service(tmp_path)
    draft = service.create_session_draft(_intent())
    original_compose = service.composer.compose
    attempts = 0

    def fail_once(intent: Any, *, output_dir: Path) -> Any:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise RuntimeError("synthetic compose failure")
        return original_compose(intent, output_dir=output_dir)

    service.composer.compose = fail_once
    with pytest.raises(RuntimeError, match="synthetic compose failure"):
        service.compose_session(draft["id"], revision=1)

    bundle = service.compose_session(draft["id"], revision=1)
    assert bundle["payload"]["draft_revision"] == 1
    assert attempts == 2
