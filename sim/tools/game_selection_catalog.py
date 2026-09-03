"""Compile deterministic, UI-facing game choices from explicit SessionSpecs.

This module deliberately treats :class:`CatalogResolver` as the sole session
compiler. It only projects display metadata from the resolved session;
it never re-resolves package manifests or invents a second package resolver.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import stat
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

import yaml
from sim.catalog.resolver import CatalogResolver
from sim.catalog.importers.contracts import canonical_json_bytes, digest_document

SELECTION_SPEC_SCHEMA = "lingtu.sim.game-selection-spec.v1"
SELECTION_CATALOG_SCHEMA = "lingtu.sim.game-selection-catalog.v1"
UE_ASSET_LIBRARY_SCHEMA = "lingtu.sim.ue-asset-library.v1"
ASSET_REVIEW_CATALOG_SCHEMA = "lingtu.sim.game-asset-review-catalog.v1"

_CATALOG_FILENAME = "game-selection.catalog.json"
_AVAILABILITY_STATES = frozenset(
    {"runnable", "preview_only", "quarantined", "unavailable"}
)
_ASSET_AVAILABILITY = {
    "CATALOG_PACKAGE": "catalog_package_count",
    "SOURCE_CANDIDATE": "source_candidate_count",
    "QUARANTINED": "quarantined_count",
    "UNVERIFIED": "unverified_count",
}
_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_VERSION = re.compile(r"^[A-Za-z0-9][A-Za-z0-9+_.-]*$")
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_WINDOWS_ABSOLUTE = re.compile(r"(?:^|[\s\"'])(?:[A-Za-z]:[\\/]|\\\\)")
_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)


class GameSelectionCatalogError(ValueError):
    """Raised when a selection cannot be compiled without weakening trust."""


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GameSelectionCatalogError(f"{context} must be an object")
    return value


def _sequence(value: Any, context: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise GameSelectionCatalogError(f"{context} must be a list")
    return value


def _string(value: Any, context: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str) or value != value.strip() or (
        not value and not allow_empty
    ):
        raise GameSelectionCatalogError(f"{context} must be a trimmed string")
    if _WINDOWS_ABSOLUTE.search(value) or value.startswith(
        ("/", "file://", "file:\\")
    ):
        raise GameSelectionCatalogError(
            f"{context} must not contain an absolute filesystem path"
        )
    return value


def _identity(value: Any, context: str, *, version: bool = False) -> str:
    result = _string(value, context)
    pattern = _VERSION if version else _IDENTITY
    if pattern.fullmatch(result) is None:
        raise GameSelectionCatalogError(f"{context} is not a safe identity")
    return result


def _strict_keys(
    value: Mapping[str, Any],
    *,
    required: set[str],
    optional: set[str],
    context: str,
) -> None:
    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing or unknown:
        raise GameSelectionCatalogError(
            f"{context} has invalid fields; missing={missing}, unknown={unknown}"
        )


def _lexical_absolute(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path)))


def _is_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    except OSError as exc:
        raise GameSelectionCatalogError(f"cannot inspect path {path}: {exc}") from exc
    return stat.S_ISLNK(metadata.st_mode) or bool(
        getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
    )


def _assert_link_free(path: Path, *, below: Path, context: str) -> Path:
    base = _lexical_absolute(below)
    candidate = _lexical_absolute(path)
    try:
        relative = candidate.relative_to(base)
    except ValueError as exc:
        raise GameSelectionCatalogError(f"{context} escapes the repository root") from exc
    current = base
    if _is_reparse(current):
        raise GameSelectionCatalogError(f"{context} repository root is a link or reparse point")
    for part in relative.parts:
        current /= part
        if _is_reparse(current):
            raise GameSelectionCatalogError(
                f"{context} contains a link or reparse point: {current}"
            )
    return candidate


def _safe_repo_file(repo_root: Path, relative_value: Any, context: str) -> Path:
    relative = _string(relative_value, context)
    posix = PurePosixPath(relative)
    if (
        posix.is_absolute()
        or "\\" in relative
        or ":" in relative
        or any(part in {"", ".", ".."} for part in posix.parts)
    ):
        raise GameSelectionCatalogError(
            f"{context} must be one safe repository-relative POSIX path"
        )
    candidate = _assert_link_free(
        repo_root.joinpath(*posix.parts), below=repo_root, context=context
    )
    if not candidate.is_file():
        raise GameSelectionCatalogError(f"{context} does not identify a regular file")
    return candidate


def _owned_output(repo_root: Path, output_root: Path) -> Path:
    requested = Path(output_root)
    candidate = requested if requested.is_absolute() else repo_root / requested
    candidate = _assert_link_free(
        candidate, below=repo_root, context="output_root"
    )
    if candidate == _lexical_absolute(repo_root):
        raise GameSelectionCatalogError("output_root must be a child of the repository")
    return candidate


def _read_json_file(path: Path, *, repo_root: Path, context: str) -> Mapping[str, Any]:
    candidate = _assert_link_free(path, below=repo_root, context=context)
    if not candidate.is_file():
        raise GameSelectionCatalogError(f"{context} must identify a regular file")
    try:
        return _mapping(json.loads(candidate.read_text(encoding="utf-8")), context)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GameSelectionCatalogError(f"cannot read {context}: {exc}") from exc


def _duplicate_free_object(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise GameSelectionCatalogError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _read_strict_json_file(
    path: Path, *, repo_root: Path, context: str
) -> Mapping[str, Any]:
    candidate = _assert_link_free(path, below=repo_root, context=context)
    if not candidate.is_file():
        raise GameSelectionCatalogError(f"{context} must identify a regular file")
    try:
        value = json.loads(
            candidate.read_text(encoding="utf-8"),
            object_pairs_hook=_duplicate_free_object,
        )
    except GameSelectionCatalogError as exc:
        raise GameSelectionCatalogError(f"cannot read {context}: {exc}") from exc
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GameSelectionCatalogError(f"cannot read {context}: {exc}") from exc
    return _mapping(value, context)


def _reject_absolute_paths(value: Any, context: str) -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            _reject_absolute_paths(item, f"{context}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_absolute_paths(item, f"{context}[{index}]")
    elif isinstance(value, str) and (
        _WINDOWS_ABSOLUTE.search(value) or value.startswith(("/", "file://", "file:\\"))
    ):
        raise GameSelectionCatalogError(
            f"{context} must not contain an absolute filesystem path"
        )


def _asset_review(
    repo_root: Path, asset_review_catalog_path: Path | None
) -> dict[str, Any]:
    if asset_review_catalog_path is None:
        return {
            "availability": {
                "state": "unavailable",
                "reason": "asset review catalog not provided",
            },
            "catalog": None,
        }
    requested = Path(asset_review_catalog_path)
    candidate = requested if requested.is_absolute() else repo_root / requested
    catalog = _read_strict_json_file(
        candidate, repo_root=repo_root, context="asset review catalog"
    )
    _strict_keys(
        catalog,
        required={"schema", "title", "policy", "coverage", "cards", "digest"},
        optional=set(),
        context="asset review catalog",
    )
    if catalog.get("schema") != ASSET_REVIEW_CATALOG_SCHEMA:
        raise GameSelectionCatalogError("asset review catalog schema is unsupported")
    observed_digest = catalog.get("digest")
    body = {key: value for key, value in catalog.items() if key != "digest"}
    if (
        not isinstance(observed_digest, str)
        or _DIGEST.fullmatch(observed_digest) is None
        or observed_digest != digest_document(body)
    ):
        raise GameSelectionCatalogError(
            "asset review catalog digest does not match its content"
        )
    cards = _sequence(catalog.get("cards"), "asset review catalog.cards")
    card_sort_keys: list[tuple[int, str]] = []
    seen_ids: set[str] = set()
    for index, raw_card in enumerate(cards):
        card = _mapping(raw_card, f"asset review catalog.cards[{index}]")
        card_id = _identity(card.get("id"), f"asset review catalog.cards[{index}].id")
        order = card.get("order")
        if isinstance(order, bool) or not isinstance(order, int) or order < 0:
            raise GameSelectionCatalogError(
                f"asset review catalog.cards[{index}].order must be a non-negative integer"
            )
        if card_id in seen_ids:
            raise GameSelectionCatalogError(
                f"asset review catalog.cards contains duplicate id: {card_id}"
            )
        seen_ids.add(card_id)
        card_sort_keys.append((order, card_id))
    if card_sort_keys != sorted(card_sort_keys):
        raise GameSelectionCatalogError(
            "asset review catalog.cards must be sorted by order then id"
        )
    _reject_absolute_paths(catalog, "asset review catalog")
    return {
        "availability": {
            "state": "available",
            "reason": "validated asset review catalog",
        },
        "catalog": dict(catalog),
    }


def _asset_summary(repo_root: Path, asset_library_path: Path | None) -> dict[str, Any]:
    result: dict[str, Any] = {
        field: 0 for field in _ASSET_AVAILABILITY.values()
    }
    if asset_library_path is None:
        return {
            **result,
            "availability": {
                "state": "unavailable",
                "reason": "asset library not provided",
            },
            "library_digest": None,
        }
    requested = Path(asset_library_path)
    candidate = requested if requested.is_absolute() else repo_root / requested
    library = _read_json_file(
        candidate, repo_root=repo_root, context="asset library"
    )
    if library.get("schema") != UE_ASSET_LIBRARY_SCHEMA:
        raise GameSelectionCatalogError("asset library schema is unsupported")
    observed_digest = library.get("digest")
    body = {key: value for key, value in library.items() if key != "digest"}
    if not isinstance(observed_digest, str) or observed_digest != digest_document(body):
        raise GameSelectionCatalogError("asset library digest does not match its content")
    for index, raw_entry in enumerate(_sequence(library.get("entries"), "asset library.entries")):
        entry = _mapping(raw_entry, f"asset library.entries[{index}]")
        availability = _string(
            entry.get("availability"), f"asset library.entries[{index}].availability"
        )
        field = _ASSET_AVAILABILITY.get(availability)
        if field is None:
            raise GameSelectionCatalogError(
                f"asset library.entries[{index}].availability is unsupported"
            )
        result[field] += 1
    return {
        **result,
        "availability": {
            "state": "available",
            "reason": "validated asset library",
        },
        "library_digest": observed_digest,
    }


def _load_selection_spec(
    selection_spec: Mapping[str, Any] | Path, *, repo_root: Path
) -> Mapping[str, Any]:
    if isinstance(selection_spec, Mapping):
        return selection_spec
    requested = Path(selection_spec)
    path = requested if requested.is_absolute() else repo_root / requested
    path = _assert_link_free(path, below=repo_root, context="selection spec")
    if not path.is_file():
        raise GameSelectionCatalogError("selection spec must identify a regular file")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise GameSelectionCatalogError(f"cannot read selection spec: {exc}") from exc
    return _mapping(value, "selection spec")


def _presentation_package(value: Any, context: str) -> dict[str, str]:
    package = _mapping(value, context)
    _strict_keys(
        package,
        required={"id", "version", "label"},
        optional=set(),
        context=context,
    )
    return {
        "id": _identity(package.get("id"), f"{context}.id"),
        "version": _identity(package.get("version"), f"{context}.version", version=True),
        "label": _string(package.get("label"), f"{context}.label"),
    }


def _presentation(value: Any, context: str) -> dict[str, Any]:
    presentation = _mapping(value, context)
    _strict_keys(
        presentation,
        required={"robot", "world", "scenario", "mode"},
        optional=set(),
        context=context,
    )
    scenario_value = presentation.get("scenario")
    scenario = (
        None
        if scenario_value is None
        else _presentation_package(scenario_value, f"{context}.scenario")
    )
    return {
        "robot": _presentation_package(presentation.get("robot"), f"{context}.robot"),
        "world": _presentation_package(presentation.get("world"), f"{context}.world"),
        "scenario": scenario,
        "mode": _string(presentation.get("mode"), f"{context}.mode"),
    }


def _normalized_entries(selection_spec: Mapping[str, Any]) -> list[dict[str, Any]]:
    _strict_keys(
        selection_spec,
        required={"schema", "title", "entries"},
        optional=set(),
        context="selection spec",
    )
    if selection_spec.get("schema") != SELECTION_SPEC_SCHEMA:
        raise GameSelectionCatalogError("selection spec schema is unsupported")
    entries: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    raw_entries = _sequence(selection_spec.get("entries"), "selection spec.entries")
    if not raw_entries:
        raise GameSelectionCatalogError("selection spec.entries must be non-empty")
    for index, raw_entry in enumerate(raw_entries):
        context = f"selection spec.entries[{index}]"
        entry = _mapping(raw_entry, context)
        _strict_keys(
            entry,
            required={
                "id",
                "title",
                "description",
                "order",
                "session_spec",
                "availability",
                "presentation",
            },
            optional={"tags"},
            context=context,
        )
        entry_id = _identity(entry.get("id"), f"{context}.id")
        if entry_id in seen_ids:
            raise GameSelectionCatalogError(f"duplicate selection id: {entry_id}")
        seen_ids.add(entry_id)
        order = entry.get("order")
        if isinstance(order, bool) or not isinstance(order, int) or order < 0:
            raise GameSelectionCatalogError(
                f"{context}.order must be a non-negative integer"
            )
        availability = _mapping(entry.get("availability"), f"{context}.availability")
        _strict_keys(
            availability,
            required={"state", "reason"},
            optional=set(),
            context=f"{context}.availability",
        )
        state = _string(availability.get("state"), f"{context}.availability.state")
        if state not in _AVAILABILITY_STATES:
            raise GameSelectionCatalogError(f"{context}.availability state is unsupported")
        tags = [
            _identity(tag, f"{context}.tags[{tag_index}]")
            for tag_index, tag in enumerate(_sequence(entry.get("tags", []), f"{context}.tags"))
        ]
        if len(set(tags)) != len(tags):
            raise GameSelectionCatalogError(f"{context}.tags must be unique")
        entries.append(
            {
                "id": entry_id,
                "title": _string(entry.get("title"), f"{context}.title"),
                "description": _string(entry.get("description"), f"{context}.description"),
                "order": order,
                "session_spec": _string(entry.get("session_spec"), f"{context}.session_spec"),
                "availability": {
                    "state": state,
                    "reason": _string(
                        availability.get("reason"),
                        f"{context}.availability.reason",
                        allow_empty=True,
                    ),
                },
                "presentation": _presentation(
                    entry.get("presentation"), f"{context}.presentation"
                ),
                "tags": sorted(tags),
            }
        )
    entries.sort(key=lambda item: (item["order"], item["id"]))
    return entries


def _validate_materialized_bundle(
    bundle_dir: Path,
    *,
    resolved: Any,
    repo_root: Path,
) -> Mapping[str, Any]:
    session = _load_selection_spec(bundle_dir / "session.yaml", repo_root=repo_root)
    expected_session_id = getattr(resolved, "session_id", None)
    if not isinstance(expected_session_id, str) or not expected_session_id.strip():
        raise GameSelectionCatalogError("resolver returned an invalid session_id")
    if session.get("session_id") != expected_session_id:
        raise GameSelectionCatalogError(
            "compiled bundle session_id does not match the resolver result"
        )
    return session


def _bundle_artifacts(bundle_dir: Path, *, repo_root: Path) -> list[dict[str, Any]]:
    """Pin the exact top-level bytes published by ``ResolvedSession.write_bundle``."""

    artifacts: list[dict[str, Any]] = []
    for path in sorted(bundle_dir.iterdir(), key=lambda item: item.name):
        candidate = _assert_link_free(
            path, below=repo_root, context="compiled bundle artifact"
        )
        if not candidate.is_file():
            raise GameSelectionCatalogError(
                "compiled bundle must contain only top-level regular files"
            )
        artifacts.append({"path": candidate.name})
    if not artifacts:
        raise GameSelectionCatalogError("compiled bundle must not be empty")
    return artifacts


def _compiled_entry(
    entry: Mapping[str, Any],
    *,
    repo_root: Path,
    staging_root: Path,
    resolver: Any,
) -> dict[str, Any]:
    common = {
        "id": entry["id"],
        "title": entry["title"],
        "description": entry["description"],
        "order": entry["order"],
        "availability": dict(entry["availability"]),
        "tags": list(entry["tags"]),
    }
    if entry["availability"]["state"] != "runnable":
        return {
            **common,
            "bundle": None,
            **dict(entry["presentation"]),
        }

    session_path = _safe_repo_file(
        repo_root, entry["session_spec"], f"selection {entry['id']}.session_spec"
    )
    resolved = resolver.resolve(session_path)
    relative_bundle = PurePosixPath("bundles", entry["id"])
    bundle_dir = staging_root.joinpath(*relative_bundle.parts)
    resolved.write_bundle(bundle_dir)
    if not bundle_dir.is_dir():
        raise GameSelectionCatalogError(
            f"selection {entry['id']} compiled bundle was not materialized"
        )
    shutil.copyfile(session_path, bundle_dir / "session.yaml")
    _validate_materialized_bundle(
        bundle_dir, resolved=resolved, repo_root=repo_root
    )
    compiled_presentation = dict(entry["presentation"])
    if compiled_presentation != dict(entry["presentation"]):
        raise GameSelectionCatalogError(
            f"selection {entry['id']}.presentation is invalid"
        )
    return {
        **common,
        "bundle": {
            "directory": relative_bundle.as_posix(),
            "session_id": resolved.session_id,
            "artifacts": _bundle_artifacts(bundle_dir, repo_root=repo_root),
        },
        **compiled_presentation,
    }


def _tree_snapshot(root: Path) -> dict[str, bytes]:
    _assert_link_free(root, below=root, context="existing output")
    if not root.is_dir():
        raise GameSelectionCatalogError("existing output_root is not a directory")
    snapshot: dict[str, bytes] = {}
    for path in sorted(root.rglob("*"), key=lambda item: item.as_posix()):
        _assert_link_free(path, below=root, context="existing output")
        if path.is_dir():
            continue
        if not path.is_file():
            raise GameSelectionCatalogError("existing output contains a non-regular file")
        snapshot[path.relative_to(root).as_posix()] = path.read_bytes()
    return snapshot


def _publish(staging: Path, target: Path) -> None:
    if target.exists():
        if _tree_snapshot(staging) == _tree_snapshot(target):
            shutil.rmtree(staging)
            return
        raise GameSelectionCatalogError(
            "output_root already contains conflicting game-selection artifacts"
        )
    staging.replace(target)


def build_game_selection_catalog(
    repo_root: Path,
    *,
    selection_spec: Mapping[str, Any] | Path,
    output_root: Path,
    asset_library_path: Path | None = None,
    asset_review_catalog_path: Path | None = None,
) -> dict[str, Any]:
    """Compile and transactionally publish one deterministic selection catalog."""

    repository = _lexical_absolute(Path(repo_root))
    if not repository.is_dir() or _is_reparse(repository):
        raise GameSelectionCatalogError("repo_root must be a regular, link-free directory")
    target = _owned_output(repository, Path(output_root))
    normalized_spec = _load_selection_spec(selection_spec, repo_root=repository)
    title = _string(normalized_spec.get("title"), "selection spec.title")
    entries = _normalized_entries(normalized_spec)
    asset_summary = _asset_summary(repository, asset_library_path)
    asset_review = _asset_review(repository, asset_review_catalog_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    _assert_link_free(target.parent, below=repository, context="output_root parent")
    staging = Path(
        tempfile.mkdtemp(prefix=f".{target.name}.staging-", dir=target.parent)
    )
    try:
        resolver = (
            CatalogResolver.from_repository(repository)
            if any(entry["availability"]["state"] == "runnable" for entry in entries)
            else None
        )
        compiled = [
            _compiled_entry(
                entry,
                repo_root=repository,
                staging_root=staging,
                resolver=resolver,
            )
            for entry in entries
        ]
        body: dict[str, Any] = {
            "schema": SELECTION_CATALOG_SCHEMA,
            "title": title,
            "asset_summary": asset_summary,
            "asset_review": asset_review,
            "entries": compiled,
        }
        document = {**body, "digest": digest_document(body)}
        (staging / _CATALOG_FILENAME).write_bytes(canonical_json_bytes(document))
        _publish(staging, target)
        return document
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def write_game_selection_catalog(
    repo_root: Path,
    *,
    selection_spec: Mapping[str, Any] | Path,
    output_root: Path,
    asset_library_path: Path | None = None,
    asset_review_catalog_path: Path | None = None,
) -> Path:
    """Compile the catalog and return its published deterministic JSON path."""

    repository = _lexical_absolute(Path(repo_root))
    target = _owned_output(repository, Path(output_root))
    build_game_selection_catalog(
        repository,
        selection_spec=selection_spec,
        output_root=target,
        asset_library_path=asset_library_path,
        asset_review_catalog_path=asset_review_catalog_path,
    )
    return target / _CATALOG_FILENAME


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compile deterministic, UI-facing LingTu game choices."
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--asset-library", type=Path)
    parser.add_argument("--asset-review-catalog", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Compile the requested catalog and print its repository-relative path."""

    args = _parser().parse_args(argv)
    repo_root = _lexical_absolute(args.repo_root)
    spec_path = args.spec if args.spec.is_absolute() else repo_root / args.spec
    spec_path = _assert_link_free(spec_path, below=repo_root, context="selection spec")
    output = write_game_selection_catalog(
        repo_root,
        selection_spec=spec_path,
        output_root=args.output_root,
        asset_library_path=args.asset_library,
        asset_review_catalog_path=args.asset_review_catalog,
    )
    print(output.relative_to(repo_root).as_posix())
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised through the CLI
    raise SystemExit(main())


__all__ = [
    "GameSelectionCatalogError",
    "build_game_selection_catalog",
    "main",
    "write_game_selection_catalog",
]
