# ruff: noqa

from __future__ import annotations

from pathlib import Path
import os

import pytest
from tools.simstudio.service.artifact_service import ArtifactNotFound, ArtifactSecurityError, ArtifactService
from tools.simstudio.service.run_service import RunService
from tools.simstudio.service.store import StudioStore


def _run(tmp_path: Path) -> tuple[StudioStore, str, Path]:
    run_id = "a" * 32
    store = StudioStore(tmp_path / "store", id_factory=iter([run_id]).__next__)
    run = store.create_run(
        {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": "b" * 32,
            "launch_profile": "headless",
            "artifact_path": f"artifacts/runs/{run_id}",
        },
        status="STOPPED",
    )
    root = store.root / "artifacts" / "runs" / run.id
    root.mkdir(parents=True)
    return store, run.id, root


def _replace_with_hardlink_or_skip(source: Path, target: Path) -> None:
    source.unlink()
    try:
        os.link(target, source)
    except OSError as exc:
        pytest.skip(f"hardlinks unavailable on this worker: {exc}")


def test_inventory_is_sorted_and_has_content_hash(tmp_path: Path) -> None:
    store, run_id, root = _run(tmp_path)
    (root / "z.txt").write_text("last", encoding="utf-8")
    (root / "a.json").write_text('{"ok":true}', encoding="utf-8")
    (root / "nested").mkdir()
    (root / "nested" / "m.txt").write_text("middle", encoding="utf-8")
    service = ArtifactService(store)
    entries = service.list_artifacts(run_id)
    assert [entry["path"] for entry in entries] == ["a.json", "nested", "nested/m.txt", "z.txt"]
    assert entries[0]["sha256"]
    assert service.preview(run_id, "a.json")["format"] == "json"


def test_preview_is_bounded_and_binary_safe(tmp_path: Path) -> None:
    store, run_id, root = _run(tmp_path)
    (root / "long.txt").write_text("0123456789", encoding="utf-8")
    (root / "blob.bin").write_bytes(b"\xff\x00\x01")
    service = ArtifactService(store, max_preview_bytes=4)
    text = service.preview(run_id, "long.txt")
    assert text["content"] == "0123"
    assert text["truncated"] is True
    assert service.preview(run_id, "blob.bin")["previewable"] is False


@pytest.mark.parametrize("relative", ["../secret", "/secret", "a\\b", "a:b", ""])
def test_unsafe_paths_are_rejected(tmp_path: Path, relative: str) -> None:
    store, run_id, _ = _run(tmp_path)
    with pytest.raises(ArtifactSecurityError):
        ArtifactService(store).get_artifact(run_id, relative)


def test_missing_artifact_is_explicit(tmp_path: Path) -> None:
    store, run_id, _ = _run(tmp_path)
    with pytest.raises(ArtifactNotFound):
        ArtifactService(store).get_artifact(run_id, "missing.json")


def test_cross_run_traversal_is_rejected(tmp_path: Path) -> None:
    store, run_id, root = _run(tmp_path)
    other = root.parent / "run-2"
    other.mkdir()
    (other / "secret.txt").write_text("secret", encoding="utf-8")
    service = ArtifactService(store)
    with pytest.raises(ArtifactSecurityError):
        service.get_artifact(run_id, "../run-2/secret.txt")
    with pytest.raises(ArtifactSecurityError):
        service.preview(run_id, "../run-2/secret.txt")


def test_symlink_is_rejected_when_supported(tmp_path: Path) -> None:
    store, run_id, root = _run(tmp_path)
    target = tmp_path / "outside.txt"
    target.write_text("outside", encoding="utf-8")
    link = root / "link.txt"
    try:
        link.symlink_to(target)
    except (OSError, NotImplementedError):
        pytest.skip("symlinks unavailable on this Windows worker")
    with pytest.raises(ArtifactSecurityError):
        ArtifactService(store).list_artifacts(run_id)


def test_cross_run_symlink_preview_is_rejected_when_supported(tmp_path: Path) -> None:
    store, run_id, root = _run(tmp_path)
    other = root.parent / "run-2"
    other.mkdir()
    target = other / "secret.txt"
    target.write_text("secret", encoding="utf-8")
    link = root / "link.txt"
    try:
        link.symlink_to(target)
    except (OSError, NotImplementedError):
        pytest.skip("symlinks unavailable on this Windows worker")
    service = ArtifactService(store)
    with pytest.raises(ArtifactSecurityError):
        service.get_artifact(run_id, "link.txt")
    with pytest.raises(ArtifactSecurityError):
        service.preview(run_id, "link.txt")


def test_preview_rejects_hardlink_swap_after_path_check(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store, run_id, root = _run(tmp_path)
    artifact = root / "report.txt"
    artifact.write_text("public", encoding="utf-8")
    outside = tmp_path / "outside.txt"
    outside.write_text("secret", encoding="utf-8")
    swapped = False
    original_is_file = Path.is_file

    def swap_after_check(path: Path) -> bool:
        nonlocal swapped
        result = original_is_file(path)
        if path == artifact and result and not swapped:
            swapped = True
            _replace_with_hardlink_or_skip(artifact, outside)
        return result

    monkeypatch.setattr(Path, "is_file", swap_after_check)
    with pytest.raises(ArtifactSecurityError):
        ArtifactService(store).preview(run_id, "report.txt")


def test_metadata_rejects_hardlink_swap_after_path_check(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store, run_id, root = _run(tmp_path)
    artifact = root / "report.txt"
    artifact.write_text("public", encoding="utf-8")
    outside = tmp_path / "outside.txt"
    outside.write_text("secret", encoding="utf-8")
    swapped = False
    original_is_file = Path.is_file

    def swap_after_check(path: Path) -> bool:
        nonlocal swapped
        result = original_is_file(path)
        if path == artifact and result and not swapped:
            swapped = True
            _replace_with_hardlink_or_skip(artifact, outside)
        return result

    monkeypatch.setattr(Path, "is_file", swap_after_check)
    with pytest.raises(ArtifactSecurityError):
        ArtifactService(store).get_artifact(run_id, "report.txt")


def test_reparse_components_are_rejected_for_inventory_and_preview(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store, run_id, root = _run(tmp_path)
    nested = root / "nested"
    nested.mkdir()
    artifact = nested / "secret.txt"
    artifact.write_text("secret", encoding="utf-8")
    original = StudioStore._is_reparse_point

    def fake_reparse(path: Path) -> bool:
        if Path(path) == nested:
            return True
        return original(path)

    monkeypatch.setattr(StudioStore, "_is_reparse_point", staticmethod(fake_reparse))
    service = ArtifactService(store)
    with pytest.raises(ArtifactSecurityError):
        service.list_artifacts(run_id)
    with pytest.raises(ArtifactSecurityError):
        service.get_artifact(run_id, "nested/secret.txt")
    with pytest.raises(ArtifactSecurityError):
        service.preview(run_id, "nested/secret.txt")


def test_custom_run_service_artifact_root_is_readable(tmp_path: Path) -> None:
    store = StudioStore(tmp_path / "store")
    bundle = store.create_bundle({"bundle_path": "bundles/demo", "session_id": "studio-session"})
    captured: dict[str, Path] = {}

    class FakeSession:
        def __init__(self, artifact_root: Path, **_: object) -> None:
            captured["artifact_root"] = artifact_root

        def prepare(self) -> dict[str, object]:
            captured["artifact_root"].joinpath("ready.txt").write_text("ready", encoding="utf-8")
            return {"readiness": {"physics": "ready"}}

        def stop(self) -> dict[str, object]:
            return {"stopped": True}

    service = RunService(store, FakeSession, artifact_root=store.root / "custom" / "runs")
    run = service.create_run(bundle.id, "headless")
    assert run["payload"]["artifact_path"] == f"custom/runs/{run['id']}"
    service.prepare(run["id"])

    preview = ArtifactService(store).preview(run["id"], "ready.txt")
    assert preview["content"] == "ready"


def test_artifact_root_must_be_bound_to_run_id(tmp_path: Path) -> None:
    ids = iter(["a" * 32, "b" * 32])
    store = StudioStore(tmp_path / "store", id_factory=lambda: next(ids))
    first = store.create_run(
        {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": "c" * 32,
            "launch_profile": "headless",
            "artifact_path": f"artifacts/runs/{'a' * 32}",
        },
        status="STOPPED",
    )
    second = store.create_run(
        {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": "c" * 32,
            "launch_profile": "headless",
            "artifact_path": f"artifacts/runs/{'b' * 32}",
        },
        status="STOPPED",
    )
    first_root = store.root / "artifacts" / "runs" / first.id
    first_root.mkdir(parents=True)
    (first_root / "secret.txt").write_text("secret", encoding="utf-8")
    payload = dict(second.payload)
    payload["artifact_path"] = f"artifacts/runs/{first.id}"
    store.update_run(second.id, expected_revision=second.revision, payload=payload, status=second.status)

    with pytest.raises(ArtifactSecurityError):
        ArtifactService(store).preview(second.id, "secret.txt")
