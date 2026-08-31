"""Trusted external handoff from the RobotSimUE selector to a compiled session.

RobotSimUE only displays a compiler-produced catalog and publishes one immutable
selection intent.  This module validates that catalog and the selected
:class:`ResolvedSessionBundle`, then constructs the existing playable coordinator
command.  Planning is the default and starts no process.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import stat
import subprocess
import sys
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Protocol

from sim.importers.contracts import canonical_json_bytes, digest_document
from sim.runtime.coordinator import STATIC_PLAN_FILES, load_resolved_session_bundle
from sim.runtime.coordinator.run_allocation import RunAllocationError

CATALOG_SCHEMA = "lingtu.sim.game-selection-catalog.v1"
INTENT_SCHEMA = "lingtu.sim.game-selection-intent.v1"
PLAN_SCHEMA = "lingtu.sim.game-selection-launch-plan.v1"
EVIDENCE_SCHEMA = "lingtu.sim.game-selection-validated.v1"
PLAYABLE_COORDINATOR_MODULE = "sim.runtime.coordinator.playable_vertical_slice"

_DIGEST_RE = re.compile(r"[0-9a-f]{64}\Z")
_IDENTITY_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
_BUNDLE_FILES = ("session.yaml", *STATIC_PLAN_FILES)
_CATALOG_KEYS = {"schema", "title", "asset_summary", "entries", "digest"}
_INTENT_KEYS = {"schema", "selection_id", "bundle_directory", "session_id"}
_FORBIDDEN_UNREAL_ARGUMENT_PREFIXES = (
    "-LingTuGameSelectionCatalog",
    "-LingTuGameSelectionIntent",
    "-LingTuGameSelector",
    "-ExecCmds",
)


class GameSelectionLauncherError(RuntimeError):
    """The selector handoff cannot be proven safe and deterministic."""


class ProcessRunner(Protocol):
    """Synchronous process seam; returning proves the child naturally ended."""

    def __call__(
        self,
        argv: tuple[str, ...],
        *,
        cwd: Path,
        timeout: float | None = None,
    ) -> int: ...


@dataclass(frozen=True)
class SelectorLaunchPlan:
    """Immutable selector command and its launcher-owned evidence paths."""

    repo_root: Path
    run_dir: Path
    intent_path: Path
    plan_path: Path
    catalog_path: Path
    catalog_digest: str
    catalog_document: Mapping[str, Any]
    selector_argv: tuple[str, ...]


@dataclass(frozen=True)
class ValidatedGameSelection:
    """Exact compiled bundle selected by UE and independently revalidated."""

    selection_id: str
    bundle_dir: Path
    session_id: str
    bundle_files: Mapping[str, str]
    evidence_path: Path


ProcessRunnerCallable = Callable[..., int]


def _strict_json(payload: bytes, context: str) -> Any:
    def object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise GameSelectionLauncherError(f"{context} contains duplicate JSON key {key!r}")
            result[key] = value
        return result

    def reject_constant(value: str) -> None:
        raise GameSelectionLauncherError(f"{context} contains non-finite JSON constant {value!r}")

    try:
        return json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=object_pairs,
            parse_constant=reject_constant,
        )
    except GameSelectionLauncherError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GameSelectionLauncherError(f"{context} is not strict UTF-8 JSON") from exc


def _lexical_absolute(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path)))


def _is_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except OSError as exc:
        raise GameSelectionLauncherError(f"cannot inspect path: {path}") from exc
    return stat.S_ISLNK(metadata.st_mode) or bool(getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT)


def _require_plain_directory(path: Path, context: str) -> Path:
    candidate = _lexical_absolute(path)
    if not candidate.is_dir() or _is_reparse(candidate):
        raise GameSelectionLauncherError(f"{context} must be a plain directory")
    if candidate.resolve(strict=True) != candidate:
        raise GameSelectionLauncherError(f"{context} must be canonical")
    return candidate


def _require_contained_link_free(path: Path, *, root: Path, context: str) -> Path:
    base = _require_plain_directory(root, f"{context} root")
    candidate = _lexical_absolute(path)
    try:
        relative = candidate.relative_to(base)
    except ValueError as exc:
        raise GameSelectionLauncherError(f"{context} escapes its trusted root") from exc
    current = base
    for part in relative.parts:
        current /= part
        if current.exists() and _is_reparse(current):
            raise GameSelectionLauncherError(f"{context} contains a link or reparse point: {current}")
    return candidate


def _require_plain_file(path: Path, *, context: str) -> Path:
    candidate = _lexical_absolute(path)
    if not candidate.is_file() or _is_reparse(candidate):
        raise GameSelectionLauncherError(f"{context} must be a plain file")
    if candidate.resolve(strict=True) != candidate:
        raise GameSelectionLauncherError(f"{context} must be canonical")
    return candidate


def _digest(value: object, context: str) -> str:
    if not isinstance(value, str) or _DIGEST_RE.fullmatch(value) is None:
        raise GameSelectionLauncherError(f"{context} must be a lowercase SHA-256 digest")
    return value


def _identity(value: object, context: str) -> str:
    if not isinstance(value, str) or _IDENTITY_RE.fullmatch(value) is None:
        raise GameSelectionLauncherError(f"{context} has an invalid identity")
    return value


def _read_catalog(path: Path) -> Mapping[str, Any]:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise GameSelectionLauncherError("game selection catalog cannot be read") from exc
    document = _strict_json(payload, "game selection catalog")
    if type(document) is not dict or set(document) != _CATALOG_KEYS:
        raise GameSelectionLauncherError("game selection catalog must contain exactly the v1 fields")
    if document.get("schema") != CATALOG_SCHEMA:
        raise GameSelectionLauncherError("game selection catalog schema is unsupported")
    observed_digest = _digest(document.get("digest"), "catalog digest")
    body = {key: value for key, value in document.items() if key != "digest"}
    if digest_document(body) != observed_digest:
        raise GameSelectionLauncherError("game selection catalog digest does not match its canonical content")
    entries = document.get("entries")
    if not isinstance(entries, list) or not entries:
        raise GameSelectionLauncherError("game selection catalog entries must be non-empty")
    seen: set[str] = set()
    for index, entry in enumerate(entries):
        if type(entry) is not dict:
            raise GameSelectionLauncherError(f"catalog entries[{index}] must be an object")
        entry_id = _identity(entry.get("id"), f"catalog entries[{index}].id")
        if entry_id in seen:
            raise GameSelectionLauncherError("game selection catalog ids must be unique")
        seen.add(entry_id)
        availability = entry.get("availability")
        if type(availability) is not dict or availability.get("state") not in {
            "runnable",
            "preview_only",
            "quarantined",
            "unavailable",
        }:
            raise GameSelectionLauncherError(f"catalog entries[{index}].availability is invalid")
        bundle = entry.get("bundle")
        if availability["state"] == "runnable":
            if type(bundle) is not dict:
                raise GameSelectionLauncherError(f"runnable catalog entry {entry_id!r} has no compiled bundle")
            _safe_bundle_relative(bundle.get("directory"), entry_id)
            _identity(bundle.get("session_id"), f"catalog entry {entry_id} session_id")
            _bundle_artifact_descriptors(bundle.get("artifacts"), entry_id)
        elif bundle is not None:
            raise GameSelectionLauncherError(f"non-runnable catalog entry {entry_id!r} must not bind a bundle")
    return document


def _safe_bundle_relative(value: object, selection_id: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value or ":" in value:
        raise GameSelectionLauncherError(f"catalog entry {selection_id!r} bundle directory is unsafe")
    relative = PurePosixPath(value)
    if (
        relative.is_absolute()
        or any(part in {"", ".", ".."} for part in relative.parts)
        or len(relative.parts) < 2
        or relative.parts[0] != "bundles"
        or relative.parts[1] != selection_id
    ):
        raise GameSelectionLauncherError(f"catalog entry {selection_id!r} bundle directory is unsafe")
    return relative


def _bundle_artifact_descriptors(
    value: object,
    selection_id: str,
) -> tuple[Mapping[str, Any], ...]:
    if not isinstance(value, list) or not value:
        raise GameSelectionLauncherError(f"catalog entry {selection_id!r} bundle artifacts must be non-empty")
    descriptors: list[Mapping[str, Any]] = []
    seen: set[str] = set()
    for index, descriptor in enumerate(value):
        context = f"catalog entry {selection_id!r} bundle artifacts[{index}]"
        if type(descriptor) is not dict or set(descriptor) != {"path"}:
            raise GameSelectionLauncherError(f"{context} must contain exactly path")
        filename = descriptor.get("path")
        if (
            not isinstance(filename, str)
            or not filename
            or Path(filename).name != filename
            or PurePosixPath(filename).name != filename
            or filename in {".", ".."}
        ):
            raise GameSelectionLauncherError(f"{context}.path is not a safe filename")
        if filename in seen:
            raise GameSelectionLauncherError(f"catalog entry {selection_id!r} bundle artifact paths must be unique")
        seen.add(filename)
        descriptors.append(descriptor)
    missing = sorted(set(_BUNDLE_FILES) - seen)
    if missing:
        raise GameSelectionLauncherError(
            f"catalog entry {selection_id!r} bundle artifacts omit required files: {missing}"
        )
    return tuple(descriptors)


def _write_new_json(path: Path, document: Mapping[str, Any], context: str) -> None:
    try:
        with path.open("xb") as stream:
            stream.write(canonical_json_bytes(document))
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as exc:
        raise GameSelectionLauncherError(f"{context} already exists") from exc
    except OSError as exc:
        raise GameSelectionLauncherError(f"cannot write {context}") from exc


def _validated_extra_unreal_args(values: Sequence[str]) -> tuple[str, ...]:
    result: list[str] = []
    for value in values:
        if (
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or any(character in value for character in "\x00\r\n")
            or value.startswith(_FORBIDDEN_UNREAL_ARGUMENT_PREFIXES)
        ):
            raise GameSelectionLauncherError(f"extra Unreal argument is invalid or owns a launcher boundary: {value!r}")
        result.append(value)
    return tuple(result)


def create_selector_launch_plan(
    *,
    repo_root: Path,
    catalog_path: Path,
    unreal_executable: Path,
    unreal_project: Path,
    run_root: Path = Path("build/game-selection/runs"),
    run_id: str | None = None,
    level: str | None = None,
    extra_unreal_args: Sequence[str] = (),
) -> SelectorLaunchPlan:
    """Create one unique run directory and selector argv without starting UE."""

    repository = _require_plain_directory(repo_root, "repo_root")
    build_root = repository / "build"
    build_root.mkdir(exist_ok=True)
    build_root = _require_plain_directory(build_root, "repository build root")
    requested_catalog = Path(catalog_path) if Path(catalog_path).is_absolute() else repository / catalog_path
    catalog = _require_contained_link_free(
        requested_catalog,
        root=build_root,
        context="catalog_path",
    )
    catalog = _require_plain_file(catalog, context="catalog_path")
    catalog_document = _read_catalog(catalog)

    requested_executable = (
        Path(unreal_executable) if Path(unreal_executable).is_absolute() else repository / unreal_executable
    )
    executable = _require_plain_file(requested_executable, context="unreal_executable")
    requested_project = Path(unreal_project) if Path(unreal_project).is_absolute() else repository / unreal_project
    project = _require_contained_link_free(
        requested_project,
        root=repository,
        context="unreal_project",
    )
    project = _require_plain_file(project, context="unreal_project")
    if project.name.casefold() != "robotsimue.uproject":
        raise GameSelectionLauncherError("unreal_project must be RobotSimUE.uproject")

    selection_run_id = run_id or f"selector-{uuid.uuid4().hex}"
    _identity(selection_run_id, "run_id")
    requested_run_root = run_root if Path(run_root).is_absolute() else repository / run_root
    owned_run_root = _require_contained_link_free(
        requested_run_root,
        root=build_root,
        context="run_root",
    )
    owned_run_root.mkdir(parents=True, exist_ok=True)
    owned_run_root = _require_plain_directory(owned_run_root, "run_root")
    run_dir = owned_run_root / selection_run_id
    try:
        run_dir.mkdir(exist_ok=False)
    except FileExistsError as exc:
        raise GameSelectionLauncherError(f"game selection run directory already exists: {run_dir}") from exc
    run_dir = _require_plain_directory(run_dir, "game selection run directory")
    intent_path = run_dir / "selection.intent.json"
    plan_path = run_dir / "launcher.plan.json"

    selector_args = [
        str(executable),
        str(project),
    ]
    if level is not None:
        if (
            not isinstance(level, str)
            or not level.startswith("/Game/")
            or any(character in level for character in "\x00\r\n")
        ):
            raise GameSelectionLauncherError("level must be one /Game/ object path")
        selector_args.append(level)
    selector_args.extend(
        (
            "-game",
            "-windowed",
            "-NoSplash",
            "-unattended",
            "-UnattendedInput",
            "-NoCompile",
            "-LingTuRuntimeUI",
            "-LingTuGameSelector",
            "-LingTuGameSelectorExitOnConfirm",
            f"-LingTuGameSelectionCatalog={catalog}",
            f"-LingTuGameSelectionIntent={intent_path}",
        )
    )
    selector_args.extend(_validated_extra_unreal_args(extra_unreal_args))
    selector_argv = tuple(selector_args)
    plan_document = {
        "schema": PLAN_SCHEMA,
        "run_id": selection_run_id,
        "run_dir": str(run_dir),
        "intent_path": str(intent_path),
        "catalog_path": str(catalog),
        "catalog_digest": catalog_document["digest"],
        "selector_argv": list(selector_argv),
        "starts_processes": False,
    }
    _write_new_json(plan_path, plan_document, "launcher plan")
    return SelectorLaunchPlan(
        repo_root=repository,
        run_dir=run_dir,
        intent_path=intent_path,
        plan_path=plan_path,
        catalog_path=catalog,
        catalog_digest=str(catalog_document["digest"]),
        catalog_document=catalog_document,
        selector_argv=selector_argv,
    )


def _default_process_runner(
    argv: tuple[str, ...],
    *,
    cwd: Path,
    timeout: float | None = None,
) -> int:
    process_environment = os.environ.copy()
    process_environment["UE_SKIP_UBT_SDK_SETUP"] = "1"
    completed = subprocess.run(  # noqa: S603 - caller pins the executable/entrypoint.
        argv,
        cwd=cwd,
        check=False,
        timeout=timeout,
        env=process_environment,
    )
    return completed.returncode


def _catalog_entry(catalog: Mapping[str, Any], selection_id: str) -> Mapping[str, Any]:
    matches = [entry for entry in catalog["entries"] if isinstance(entry, Mapping) and entry.get("id") == selection_id]
    if len(matches) != 1:
        raise GameSelectionLauncherError("selection intent does not identify exactly one catalog entry")
    entry = matches[0]
    availability = entry.get("availability")
    if not isinstance(availability, Mapping) or availability.get("state") != "runnable":
        raise GameSelectionLauncherError("selection intent targets a non-runnable entry")
    return entry


def _read_intent(path: Path) -> Mapping[str, Any]:
    if not path.is_file() or _is_reparse(path):
        raise GameSelectionLauncherError("selector exited without one plain no-replace selection intent")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise GameSelectionLauncherError("selection intent cannot be read") from exc
    if not payload or len(payload) > 64 * 1024:
        raise GameSelectionLauncherError("selection intent has an invalid size")
    document = _strict_json(payload, "selection intent")
    if type(document) is not dict or set(document) != _INTENT_KEYS:
        raise GameSelectionLauncherError("selection intent must contain exactly the v1 fields")
    if document.get("schema") != INTENT_SCHEMA:
        raise GameSelectionLauncherError("selection intent schema is unsupported")
    _identity(document.get("selection_id"), "selection intent selection_id")
    _identity(document.get("session_id"), "selection intent session_id")
    bundle_value = document.get("bundle_directory")
    if (
        not isinstance(bundle_value, str)
        or not bundle_value
        or bundle_value != bundle_value.strip()
        or any(character in bundle_value for character in "\x00\r\n")
        or not Path(bundle_value).is_absolute()
    ):
        raise GameSelectionLauncherError("selection intent bundle_directory must be an absolute path")
    return document


def _snapshot_bundle_files(
    bundle_dir: Path,
    descriptors: Sequence[Mapping[str, Any]],
) -> dict[str, str]:
    expected = {str(descriptor["path"]): descriptor for descriptor in descriptors}
    actual_paths: dict[str, Path] = {}
    try:
        children = tuple(bundle_dir.iterdir())
    except OSError as exc:
        raise GameSelectionLauncherError("selected bundle cannot be enumerated") from exc
    for path in children:
        if not path.is_file() or _is_reparse(path):
            raise GameSelectionLauncherError("selected bundle must contain only declared top-level regular files")
        actual_paths[path.name] = path
    if set(actual_paths) != set(expected):
        raise GameSelectionLauncherError("selected bundle top-level files do not exactly match catalog artifacts")
    result: dict[str, str] = {}
    for filename, descriptor in sorted(expected.items()):
        result[filename] = str(actual_paths[filename])
    return result


def _validate_selected_bundle(
    plan: SelectorLaunchPlan,
    intent: Mapping[str, Any],
) -> tuple[Path, str, dict[str, str]]:
    selection_id = str(intent["selection_id"])
    entry = _catalog_entry(plan.catalog_document, selection_id)
    bundle = entry.get("bundle")
    if not isinstance(bundle, Mapping):
        raise GameSelectionLauncherError("selected catalog entry has no bundle")
    relative = _safe_bundle_relative(bundle.get("directory"), selection_id)
    expected_bundle = plan.catalog_path.parent.joinpath(*relative.parts)
    expected_bundle = _require_contained_link_free(
        expected_bundle,
        root=plan.catalog_path.parent,
        context="selected bundle",
    )
    selected_bundle = _lexical_absolute(Path(str(intent["bundle_directory"])))
    if os.path.normcase(os.path.normpath(selected_bundle)) != os.path.normcase(os.path.normpath(expected_bundle)):
        raise GameSelectionLauncherError("selection intent bundle_directory does not match the compiled catalog entry")
    selected_bundle = _require_plain_directory(selected_bundle, "selected bundle")
    expected_digest = _identity(bundle.get("session_id"), "selected catalog session_id")
    descriptors = _bundle_artifact_descriptors(bundle.get("artifacts"), selection_id)
    if intent.get("session_id") != expected_digest:
        raise GameSelectionLauncherError("selection intent session_id does not match the compiled catalog entry")
    try:
        first = load_resolved_session_bundle(selected_bundle, repo_root=plan.repo_root)
        first_files = _snapshot_bundle_files(selected_bundle, descriptors)
        second = load_resolved_session_bundle(selected_bundle, repo_root=plan.repo_root)
        second_files = _snapshot_bundle_files(selected_bundle, descriptors)
    except RunAllocationError as exc:
        raise GameSelectionLauncherError(str(exc)) from exc
    if (
        first.session_id != expected_digest
        or second.session_id != expected_digest
        or first.plans != second.plans
        or first_files != second_files
    ):
        raise GameSelectionLauncherError("selected compiled bundle changed during validation")
    return selected_bundle, expected_digest, second_files


def execute_selector(
    plan: SelectorLaunchPlan,
    *,
    process_runner: ProcessRunnerCallable = _default_process_runner,
    timeout_seconds: float = 900.0,
) -> ValidatedGameSelection:
    """Run UE once, wait for its natural exit, and validate its new intent."""

    if not isinstance(plan, SelectorLaunchPlan):
        raise TypeError("plan must be SelectorLaunchPlan")
    if (
        isinstance(timeout_seconds, bool)
        or not isinstance(timeout_seconds, (int, float))
        or not math.isfinite(float(timeout_seconds))
        or float(timeout_seconds) <= 0.0
    ):
        raise GameSelectionLauncherError("selector timeout_seconds must be finite and positive")
    selector_timeout = float(timeout_seconds)
    if _read_catalog(plan.catalog_path) != plan.catalog_document:
        raise GameSelectionLauncherError("game selection catalog changed before selector execution")
    if plan.intent_path.exists() or Path(str(plan.intent_path) + ".tmp").exists():
        raise GameSelectionLauncherError("selection intent path already exists")
    try:
        returncode = process_runner(
            plan.selector_argv,
            cwd=plan.repo_root,
            timeout=selector_timeout,
        )
    except subprocess.TimeoutExpired as exc:
        raise GameSelectionLauncherError(f"RobotSimUE selector timed out after {selector_timeout:g} seconds") from exc
    if isinstance(returncode, bool) or not isinstance(returncode, int):
        raise GameSelectionLauncherError("selector process runner returned an invalid code")
    if returncode != 0:
        raise GameSelectionLauncherError(f"RobotSimUE selector exited with code {returncode}")
    current_catalog = _read_catalog(plan.catalog_path)
    if current_catalog != plan.catalog_document:
        raise GameSelectionLauncherError("game selection catalog changed during selector execution")
    if Path(str(plan.intent_path) + ".tmp").exists():
        raise GameSelectionLauncherError("selection intent temporary file was not retired")
    intent = _read_intent(plan.intent_path)
    bundle_dir, session_id, bundle_files = _validate_selected_bundle(plan, intent)
    evidence_path = plan.run_dir / "selection.validated.json"
    _write_new_json(
        evidence_path,
        {
            "schema": EVIDENCE_SCHEMA,
            "selection_id": intent["selection_id"],
            "catalog_path": str(plan.catalog_path),
            "catalog_digest": plan.catalog_digest,
            "intent_path": str(plan.intent_path),
            "bundle_directory": str(bundle_dir),
            "session_id": session_id,
            "bundle_files": bundle_files,
        },
        "validated selection evidence",
    )
    return ValidatedGameSelection(
        selection_id=str(intent["selection_id"]),
        bundle_dir=bundle_dir,
        session_id=session_id,
        bundle_files=bundle_files,
        evidence_path=evidence_path,
    )


def build_playable_coordinator_argv(
    selection: ValidatedGameSelection,
    *,
    python_executable: Path = Path(sys.executable),
    coordinator_arguments: Sequence[str] = (),
) -> tuple[str, ...]:
    """Build the existing coordinator CLI command with the exact selected bundle."""

    if not isinstance(selection, ValidatedGameSelection):
        raise TypeError("selection must be ValidatedGameSelection")
    python = _require_plain_file(python_executable, context="python_executable")
    arguments: list[str] = []
    for argument in coordinator_arguments:
        if not isinstance(argument, str) or not argument or any(character in argument for character in "\x00\r\n"):
            raise GameSelectionLauncherError("coordinator argument is invalid")
        arguments.append(argument)
    return (
        str(python),
        "-m",
        PLAYABLE_COORDINATOR_MODULE,
        str(selection.bundle_dir),
        *arguments,
    )


def execute_playable_coordinator(
    plan: SelectorLaunchPlan,
    selection: ValidatedGameSelection,
    *,
    coordinator_argv: tuple[str, ...],
    process_runner: ProcessRunnerCallable = _default_process_runner,
) -> int:
    """Revalidate pinned inputs immediately before the existing coordinator starts."""

    if _read_catalog(plan.catalog_path) != plan.catalog_document:
        raise GameSelectionLauncherError("game selection catalog changed before coordinator start")
    intent = _read_intent(plan.intent_path)
    entry = _catalog_entry(plan.catalog_document, selection.selection_id)
    bundle = entry.get("bundle")
    if not isinstance(bundle, Mapping):
        raise GameSelectionLauncherError("selected catalog entry has no bundle")
    descriptors = _bundle_artifact_descriptors(bundle.get("artifacts"), selection.selection_id)
    current_snapshot = _snapshot_bundle_files(selection.bundle_dir, descriptors)
    if current_snapshot != dict(selection.bundle_files):
        raise GameSelectionLauncherError("selected compiled bundle changed before coordinator start")
    bundle_dir, digest, current_files = _validate_selected_bundle(plan, intent)
    if (
        bundle_dir != selection.bundle_dir
        or digest != selection.session_id
        or current_files != dict(selection.bundle_files)
    ):
        raise GameSelectionLauncherError("selected compiled bundle changed before coordinator start")
    expected_prefix = (
        coordinator_argv[0] if coordinator_argv else "",
        "-m",
        PLAYABLE_COORDINATOR_MODULE,
        str(selection.bundle_dir),
    )
    if len(coordinator_argv) < 4 or coordinator_argv[:4] != expected_prefix:
        raise GameSelectionLauncherError("coordinator argv does not receive the exact validated bundle")
    returncode = process_runner(coordinator_argv, cwd=plan.repo_root)
    if isinstance(returncode, bool) or not isinstance(returncode, int):
        raise GameSelectionLauncherError("coordinator process runner returned an invalid code")
    if returncode != 0:
        raise GameSelectionLauncherError(f"playable coordinator exited with code {returncode}")
    return returncode


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Plan a RobotSimUE game selector; process execution is opt-in.")
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--unreal-executable", type=Path, required=True)
    parser.add_argument("--unreal-project", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, default=Path("build/game-selection/runs"))
    parser.add_argument("--run-id")
    parser.add_argument("--level")
    parser.add_argument("--unreal-arg", action="append", default=[])
    parser.add_argument("--execute-selector", action="store_true")
    parser.add_argument("--execute-playable", action="store_true")
    parser.add_argument("--selector-timeout-seconds", type=float, default=900.0)
    parser.add_argument("--python-executable", type=Path, default=Path(sys.executable))
    parser.add_argument("--coordinator-arg", action="append", default=[])
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Create a dry-run plan by default; execute only with explicit flags."""

    args = _parser().parse_args(argv)
    if args.execute_playable and not args.execute_selector:
        raise SystemExit("--execute-playable requires --execute-selector")
    try:
        plan = create_selector_launch_plan(
            repo_root=args.repo_root,
            catalog_path=args.catalog,
            unreal_executable=args.unreal_executable,
            unreal_project=args.unreal_project,
            run_root=args.run_root,
            run_id=args.run_id,
            level=args.level,
            extra_unreal_args=args.unreal_arg,
        )
        result: dict[str, Any] = {
            "ok": True,
            "mode": "dry_run",
            "plan": str(plan.plan_path),
            "selector_argv": list(plan.selector_argv),
        }
        if args.execute_selector:
            selection = execute_selector(
                plan,
                timeout_seconds=args.selector_timeout_seconds,
            )
            coordinator_argv = build_playable_coordinator_argv(
                selection,
                python_executable=args.python_executable,
                coordinator_arguments=args.coordinator_arg,
            )
            result.update(
                {
                    "mode": "selector_complete",
                    "selection": str(selection.evidence_path),
                    "bundle_directory": str(selection.bundle_dir),
                    "session_id": selection.session_id,
                    "coordinator_argv": list(coordinator_argv),
                }
            )
            if args.execute_playable:
                execute_playable_coordinator(
                    plan,
                    selection,
                    coordinator_argv=coordinator_argv,
                )
                result["mode"] = "playable_complete"
    except (GameSelectionLauncherError, OSError, TypeError, ValueError) as exc:
        print(
            json.dumps(
                {"ok": False, "error": str(exc)},
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI wrapper
    raise SystemExit(main())


__all__ = [
    "GameSelectionLauncherError",
    "SelectorLaunchPlan",
    "ValidatedGameSelection",
    "build_playable_coordinator_argv",
    "create_selector_launch_plan",
    "execute_playable_coordinator",
    "execute_selector",
    "main",
]
