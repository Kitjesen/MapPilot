from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

import sim.tools.game_selection_launcher as launcher_module
from sim.catalog.importers.contracts import canonical_json_bytes, digest_document
from sim.tools.game_selection_launcher import (
    GameSelectionLauncherError,
    build_playable_coordinator_argv,
    create_selector_launch_plan,
    execute_playable_coordinator,
    execute_selector,
)


def _write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json_bytes(document))


def _bundle(repo: Path, root: Path) -> tuple[Path, str]:
    del repo
    session_id = "fixture"
    plans = {
        "physics.plan.json": {
            "schema": "lingtu.sim.physics-plan.v1",
            "session_id": session_id,
        },
        "visual.plan.json": {
            "schema": "lingtu.sim.visual-plan.v1",
            "session_id": session_id,
        },
        "sensor.plan.json": {
            "schema": "lingtu.sim.sensor-plan.v1",
            "session_id": session_id,
            "streams": {},
        },
        "control.plan.json": {
            "schema": "lingtu.sim.control-plan.v1",
            "session_id": session_id,
        },
        "transport.intent.json": {
            "schema": "lingtu.sim.transport-intent.v1",
            "session_id": session_id,
            "allocation_boundary": {
                "owner": "RunAllocation",
                "runtime_values_external": True,
            },
        },
    }
    root.mkdir(parents=True)
    (root / "session.yaml").write_text(
        "schema: lingtu.sim.session.v1\nsession_id: fixture\n",
        encoding="utf-8",
    )
    for filename, document in plans.items():
        _write_json(root / filename, document)
    return root, session_id


def _catalog(repo: Path) -> tuple[Path, Path, str]:
    catalog_root = repo / "build" / "game-selection" / "v1"
    bundle_dir, session_id = _bundle(
        repo,
        catalog_root / "bundles" / "fixture_selection",
    )
    bundle_artifacts = [
        {"path": path.name}
        for path in sorted(bundle_dir.iterdir(), key=lambda item: item.name)
        if path.is_file()
    ]
    body = {
        "schema": "lingtu.sim.game-selection-catalog.v1",
        "title": "Fixture selector",
        "asset_summary": {
            "catalog_package_count": 0,
            "source_candidate_count": 0,
            "quarantined_count": 0,
            "unverified_count": 0,
        },
        "entries": [
            {
                "id": "fixture_selection",
                "title": "Fixture",
                "description": "Compiled fixture",
                "order": 10,
                "availability": {"state": "runnable", "reason": "compiled"},
                "tags": ["fixture"],
                "bundle": {
                    "directory": "bundles/fixture_selection",
                    "session_id": session_id,
                    "artifacts": bundle_artifacts,
                },
                "robot": {"id": "fixture_robot", "version": "1.0.0", "label": "Fixture"},
                "world": {"id": "fixture_world", "version": "1.0.0", "label": "Fixture"},
                "scenario": None,
                "mode": "unreal",
            }
        ],
    }
    document = {**body, "digest": digest_document(body)}
    catalog_path = catalog_root / "game-selection.catalog.json"
    _write_json(catalog_path, document)
    return catalog_path, bundle_dir, session_id


def _external_files(repo: Path) -> tuple[Path, Path]:
    unreal = repo / "tools" / "UnrealEditor.exe"
    project = repo / "sim" / "runtime" / "visual" / "RobotSimUE" / "RobotSimUE.uproject"
    unreal.parent.mkdir(parents=True)
    unreal.write_bytes(b"not executed")
    project.parent.mkdir(parents=True)
    project.write_text("{}", encoding="utf-8")
    return unreal, project


def _plan(tmp_path: Path):
    repo = tmp_path / "repo"
    repo.mkdir()
    catalog_path, bundle_dir, session_id = _catalog(repo)
    unreal, project = _external_files(repo)
    plan = create_selector_launch_plan(
        repo_root=repo,
        catalog_path=catalog_path,
        unreal_executable=unreal,
        unreal_project=project,
        run_id="selector-fixture",
    )
    return repo, plan, bundle_dir, session_id


def _intent_document(
    bundle_dir: Path,
    expected_session_id: str,
    **overrides: Any,
) -> dict[str, Any]:
    document: dict[str, Any] = {
        "schema": "lingtu.sim.game-selection-intent.v1",
        "selection_id": "fixture_selection",
        "bundle_directory": str(bundle_dir.resolve()),
        "session_id": expected_session_id,
    }
    document.update(overrides)
    return document


def test_plan_is_no_process_dry_run_with_unique_owned_intent(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    catalog_path, _bundle_dir, _session_id = _catalog(repo)
    unreal, project = _external_files(repo)

    plan = create_selector_launch_plan(
        repo_root=repo,
        catalog_path=catalog_path,
        unreal_executable=unreal,
        unreal_project=project,
        run_id="selector-fixture",
        extra_unreal_args=("-ResX=1280", "-ResY=720"),
    )

    assert plan.run_dir == repo / "build" / "game-selection" / "runs" / "selector-fixture"
    assert plan.intent_path == plan.run_dir / "selection.intent.json"
    assert not plan.intent_path.exists()
    assert plan.plan_path.is_file()
    assert set(json.loads(plan.plan_path.read_text(encoding="utf-8"))) == {
        "schema",
        "run_id",
        "run_dir",
        "intent_path",
        "catalog_path",
        "catalog_digest",
        "selector_argv",
        "starts_processes",
    }
    assert plan.selector_argv == (
        str(unreal.resolve()),
        str(project.resolve()),
        "-game",
        "-windowed",
        "-NoSplash",
        "-unattended",
        "-UnattendedInput",
        "-NoCompile",
        "-LingTuRuntimeUI",
        "-LingTuGameSelector",
        "-LingTuGameSelectorExitOnConfirm",
        f"-LingTuGameSelectionCatalog={catalog_path.resolve()}",
        f"-LingTuGameSelectionIntent={plan.intent_path}",
        "-ResX=1280",
        "-ResY=720",
    )
    with pytest.raises(GameSelectionLauncherError, match="already exists"):
        create_selector_launch_plan(
            repo_root=repo,
            catalog_path=catalog_path,
            unreal_executable=unreal,
            unreal_project=project,
            run_id="selector-fixture",
        )


def test_default_selector_runner_uses_process_local_sdk_skip_environment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("UE_SKIP_UBT_SDK_SETUP", raising=False)
    parent_environment = dict(os.environ)
    observed: dict[str, Any] = {}

    def fake_run(
        argv: tuple[str, ...],
        *,
        cwd: Path,
        check: bool,
        timeout: float | None,
        env: dict[str, str],
    ) -> subprocess.CompletedProcess[str]:
        observed.update(
            argv=argv,
            cwd=cwd,
            check=check,
            timeout=timeout,
            env=env,
        )
        return subprocess.CompletedProcess(argv, 0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    argv = ("UnrealEditor.exe", "RobotSimUE.uproject")

    returncode = launcher_module._default_process_runner(
        argv,
        cwd=tmp_path,
        timeout=37.5,
    )

    assert returncode == 0
    assert observed == {
        "argv": argv,
        "cwd": tmp_path,
        "check": False,
        "timeout": 37.5,
        "env": {**parent_environment, "UE_SKIP_UBT_SDK_SETUP": "1"},
    }
    assert observed["env"] is not os.environ
    assert "UE_SKIP_UBT_SDK_SETUP" not in os.environ


def test_fake_selector_naturally_exits_then_validates_exact_bundle(tmp_path: Path) -> None:
    repo, plan, bundle_dir, session_id = _plan(tmp_path)
    calls: list[tuple[tuple[str, ...], Path]] = []

    def runner(argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        calls.append((argv, cwd))
        assert timeout == 900.0
        assert not plan.intent_path.exists()
        _write_json(
            plan.intent_path,
            {
                "schema": "lingtu.sim.game-selection-intent.v1",
                "selection_id": "fixture_selection",
                "bundle_directory": str(bundle_dir.resolve()),
                "session_id": session_id,
            },
        )
        return 0

    selection = execute_selector(plan, process_runner=runner)

    assert calls == [(plan.selector_argv, repo.resolve())]
    assert selection.bundle_dir == bundle_dir.resolve()
    assert selection.session_id == session_id
    assert set(selection.bundle_files) == {
        "session.yaml",
        "physics.plan.json",
        "visual.plan.json",
        "sensor.plan.json",
        "control.plan.json",
        "transport.intent.json",
    }
    assert selection.evidence_path.is_file()
    assert set(json.loads(selection.evidence_path.read_text(encoding="utf-8"))) == {
        "schema",
        "selection_id",
        "catalog_path",
        "catalog_digest",
        "intent_path",
        "bundle_directory",
        "session_id",
        "bundle_files",
    }


def test_selector_compares_catalog_content_not_json_formatting(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        plan.catalog_path.write_text(
            json.dumps(plan.catalog_document, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        _write_json(plan.intent_path, _intent_document(bundle_dir, session_id))
        return 0

    selection = execute_selector(plan, process_runner=runner)

    assert selection.session_id == session_id


def test_selector_rejects_duplicate_intent_keys(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        plan.intent_path.write_text(
            "{"
            '"schema":"lingtu.sim.game-selection-intent.v1",'
            '"selection_id":"fixture_selection",'
            '"selection_id":"fixture_selection",'
            f'"bundle_directory":{json.dumps(str(bundle_dir.resolve()))},'
            f'"session_id":"{session_id}"'
            "}\n",
            encoding="utf-8",
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="duplicate"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_unknown_intent_field(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            _intent_document(bundle_dir, session_id, unexpected=True),
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="exactly the v1 fields"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_unsupported_intent_schema(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            _intent_document(
                bundle_dir,
                session_id,
                schema="lingtu.sim.game-selection-intent.v2",
            ),
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="schema"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_relative_intent_bundle_path(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            _intent_document(
                bundle_dir,
                session_id,
                bundle_directory="bundles/fixture_selection",
            ),
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="absolute"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_absolute_intent_bundle_outside_catalog(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)
    outside = tmp_path / "outside" / "fixture_selection"
    outside.mkdir(parents=True)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            _intent_document(
                bundle_dir,
                session_id,
                bundle_directory=str(outside.resolve()),
            ),
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="does not match"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_selection_not_present_in_catalog(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            _intent_document(bundle_dir, session_id, selection_id="unknown"),
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="exactly one catalog entry"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_session_id_not_bound_to_catalog_entry(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            _intent_document(bundle_dir, session_id, session_id="different-session"),
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="session_id does not match"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_successful_exit_without_intent(tmp_path: Path) -> None:
    _repo, plan, _bundle_dir, _session_id = _plan(tmp_path)

    with pytest.raises(GameSelectionLauncherError, match=r"without.*intent"):
        execute_selector(plan, process_runner=lambda _argv, *, cwd, timeout: 0)


def test_selector_rejects_nonzero_ue_exit_before_reading_intent(tmp_path: Path) -> None:
    _repo, plan, _bundle_dir, _session_id = _plan(tmp_path)

    with pytest.raises(GameSelectionLauncherError, match="exited with code 23"):
        execute_selector(plan, process_runner=lambda _argv, *, cwd, timeout: 23)


def test_selector_rejects_ue_timeout(tmp_path: Path) -> None:
    _repo, plan, _bundle_dir, _session_id = _plan(tmp_path)
    observed_timeout: list[float] = []

    def runner(
        argv: tuple[str, ...],
        *,
        cwd: Path,
        timeout: float,
    ) -> int:
        del cwd
        observed_timeout.append(timeout)
        raise subprocess.TimeoutExpired(argv, timeout)

    with pytest.raises(GameSelectionLauncherError, match="timed out"):
        execute_selector(plan, process_runner=runner, timeout_seconds=2.5)
    assert observed_timeout == [2.5]
    assert not (plan.run_dir / "selection.validated.json").exists()


def test_selector_requires_positive_timeout_before_process_start(tmp_path: Path) -> None:
    _repo, plan, _bundle_dir, _session_id = _plan(tmp_path)

    def fail_runner(
        _argv: tuple[str, ...],
        *,
        cwd: Path,
        timeout: float,
    ) -> int:
        del cwd, timeout
        raise AssertionError("selector must not start with an invalid timeout")

    with pytest.raises(GameSelectionLauncherError, match="timeout"):
        execute_selector(plan, process_runner=fail_runner, timeout_seconds=0)


def test_selector_rejects_bundle_file_not_declared_by_catalog(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)
    (bundle_dir / "undeclared.plan.json").write_text("{}\n", encoding="utf-8")

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(
            plan.intent_path,
            {
                "schema": "lingtu.sim.game-selection-intent.v1",
                "selection_id": "fixture_selection",
                "bundle_directory": str(bundle_dir.resolve()),
                "session_id": session_id,
            },
        )
        return 0

    with pytest.raises(GameSelectionLauncherError, match="exactly match"):
        execute_selector(plan, process_runner=runner)


def test_selector_rejects_bundle_file_missing_from_catalog_snapshot(tmp_path: Path) -> None:
    _repo, plan, bundle_dir, session_id = _plan(tmp_path)
    (bundle_dir / "visual.plan.json").unlink()

    def runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        _write_json(plan.intent_path, _intent_document(bundle_dir, session_id))
        return 0

    with pytest.raises(GameSelectionLauncherError, match=r"exactly match|missing"):
        execute_selector(plan, process_runner=runner)


def test_playable_handoff_revalidates_bundle_then_uses_existing_entrypoint(
    tmp_path: Path,
) -> None:
    repo, plan, _bundle_dir, _session_id = _plan(tmp_path)

    def selector_runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        entry = plan.catalog_document["entries"][0]
        _write_json(
            plan.intent_path,
            {
                "schema": "lingtu.sim.game-selection-intent.v1",
                "selection_id": entry["id"],
                "bundle_directory": str((plan.catalog_path.parent / entry["bundle"]["directory"]).resolve()),
                "session_id": entry["bundle"]["session_id"],
            },
        )
        return 0

    selection = execute_selector(plan, process_runner=selector_runner)
    coordinator_argv = build_playable_coordinator_argv(
        selection,
        python_executable=Path(sys.executable),
        coordinator_arguments=("--repo-root", str(repo)),
    )
    calls: list[tuple[tuple[str, ...], Path]] = []

    def coordinator_runner(argv: tuple[str, ...], *, cwd: Path) -> int:
        calls.append((argv, cwd))
        return 0

    returncode = execute_playable_coordinator(
        plan,
        selection,
        coordinator_argv=coordinator_argv,
        process_runner=coordinator_runner,
    )

    assert returncode == 0
    assert coordinator_argv[:4] == (
        os.path.abspath(sys.executable),
        "-m",
        "sim.runtime.coordinator.playable_vertical_slice",
        str(selection.bundle_dir),
    )
    assert calls == [(coordinator_argv, repo.resolve())]


def test_playable_handoff_rejects_bundle_change_before_process_start(tmp_path: Path) -> None:
    _repo, plan, _bundle_dir, _session_id = _plan(tmp_path)

    def selector_runner(_argv: tuple[str, ...], *, cwd: Path, timeout: float) -> int:
        del cwd, timeout
        entry = plan.catalog_document["entries"][0]
        _write_json(
            plan.intent_path,
            {
                "schema": "lingtu.sim.game-selection-intent.v1",
                "selection_id": entry["id"],
                "bundle_directory": str((plan.catalog_path.parent / entry["bundle"]["directory"]).resolve()),
                "session_id": entry["bundle"]["session_id"],
            },
        )
        return 0

    selection = execute_selector(plan, process_runner=selector_runner)
    (selection.bundle_dir / "visual.plan.json").write_bytes(b"{}\n")

    def fail_runner(_argv: tuple[str, ...], *, cwd: Path) -> int:
        del cwd
        raise AssertionError("coordinator must not start")

    with pytest.raises(GameSelectionLauncherError, match=r"unsupported schema|changed"):
        execute_playable_coordinator(
            plan,
            selection,
            coordinator_argv=build_playable_coordinator_argv(selection),
            process_runner=fail_runner,
        )


def test_plan_creation_does_not_start_an_external_process(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    catalog_path, _bundle_dir, _session_id = _catalog(repo)
    unreal, project = _external_files(repo)

    def forbid_process(*_args: Any, **_kwargs: Any) -> Any:
        raise AssertionError("dry-run planning must not start an external process")

    monkeypatch.setattr(subprocess, "run", forbid_process)

    plan = create_selector_launch_plan(
        repo_root=repo,
        catalog_path=catalog_path,
        unreal_executable=unreal,
        unreal_project=project,
        run_id="no-process",
    )

    assert plan.plan_path.is_file()
