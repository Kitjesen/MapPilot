"""Compile a deterministic, presentation-only asset review catalog.

The catalog is deliberately incapable of granting runtime, collision, or
visual qualification authority.  Every evidence file is repository-relative
and must match an explicitly pinned byte length and SHA256 before a card is
published.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import os
import re
import secrets
import stat
from collections.abc import Iterator, Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

import yaml
from sim.catalog.importers.contracts import canonical_json_bytes, digest_document

ASSET_REVIEW_SPEC_SCHEMA = "lingtu.sim.game-asset-review-spec.v1"
ASSET_REVIEW_CATALOG_SCHEMA = "lingtu.sim.game-asset-review-catalog.v1"

MAX_SPEC_BYTES = 1 * 1024 * 1024
MAX_CARDS = 256
MAX_EVIDENCE_PER_CARD = 16
MAX_EVIDENCE_BYTES = 64 * 1024 * 1024
MAX_TOTAL_EVIDENCE_BYTES = 512 * 1024 * 1024
MAX_OUTPUT_BYTES = 8 * 1024 * 1024
MAX_STRING_LENGTH = 16 * 1024
_READ_CHUNK_BYTES = 1024 * 1024

_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_WINDOWS_ABSOLUTE = re.compile(r"(?:^|[\s\"'])(?:[A-Za-z]:[\\/]|\\\\)")
_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
_STAGE_DISPOSITION = {
    "catalog_review": "unverified",
    "conditioned_review": "quarantined",
    "source_review": "quarantined",
    "proxy_only": "proxy_only",
    "unavailable": "unavailable",
}
_REQUIRED_ROLES = {
    "catalog_review": frozenset({"package_manifest", "visual_projection"}),
    "conditioned_review": frozenset({"conditioned_mesh", "conditioning_report"}),
    "source_review": frozenset({"source_model", "source_task"}),
    "proxy_only": frozenset({"physics_proxy"}),
    "unavailable": frozenset(),
}
_ASSET_CLASSES = frozenset(
    {
        "robot",
        "payload",
        "vegetation",
        "terrain_detail",
        "scenario_proxy",
        "movable_object",
    }
)

_POLICY = {
    "purpose": "presentation_review_only",
    "runnable": False,
    "qualified_visual": False,
    "unreal_collision_profile": "NoCollision",
    "unreal_simulate_physics": False,
    "physics_authority": "mujoco",
}


class GameAssetReviewCatalogError(ValueError):
    """Raised when review metadata cannot be compiled without weakening trust."""


def _evidence(
    role: str,
    path: str,
    byte_length: int,
    sha256: str,
) -> dict[str, Any]:
    return {"role": role, "path": path, "bytes": byte_length, "sha256": sha256}


def _default_card(
    card_id: str,
    title: str,
    description: str,
    order: int,
    asset_class: str,
    stage: str,
    disposition: str,
    reason: str,
    evidence: Sequence[Mapping[str, Any]],
    tags: Sequence[str],
) -> dict[str, Any]:
    return {
        "id": card_id,
        "title": title,
        "description": description,
        "order": order,
        "asset_class": asset_class,
        "review": {
            "stage": stage,
            "disposition": disposition,
            "reason": reason,
        },
        "evidence": [dict(item) for item in evidence],
        "tags": list(tags),
    }


DEFAULT_ASSET_REVIEW_SPEC: Mapping[str, Any] = {
    "schema": ASSET_REVIEW_SPEC_SCHEMA,
    "title": "LingTu Asset Review Library",
    "cards": [
        _default_card(
            "thunder_v4",
            "Thunder V4",
            "Catalogued robot visual for review; qualification remains unverified.",
            0,
            "robot",
            "catalog_review",
            "unverified",
            "Catalog package exists, but this review catalog grants no runtime qualification.",
            [
                _evidence(
                    "package_manifest",
                    "sim/packages/robots/doso/thunder_v4/robot.package.yaml",
                    1939,
                    "33fcb5e110d3fe08c5ee26cba4f6c7866d3bc60909eb8842faa5af45dc340bb0",
                ),
                _evidence(
                    "visual_projection",
                    "sim/packages/robots/doso/thunder_v4/visual/robot.visual-projection.json",
                    45909,
                    "20bf2a765724643412d26c8ba64dde59f10bc3a8bf1a634fa8b4fb6a9ecb9038",
                ),
            ],
            ["robot", "catalog", "review"],
        ),
        _default_card(
            "rws_01",
            "Fictional RWS 01",
            "Catalogued payload visual for review; qualification remains unverified.",
            1,
            "payload",
            "catalog_review",
            "unverified",
            "Catalog package exists, but this review catalog grants no runtime qualification.",
            [
                _evidence(
                    "package_manifest",
                    "sim/packages/payloads/fictional_rws_01/1.0.0/payload.package.yaml",
                    1969,
                    "171de39a4e9b8503ec832f5a51e19459ec579c3cca1539cfb9004cc7f51a32c4",
                ),
                _evidence(
                    "visual_projection",
                    "sim/packages/payloads/fictional_rws_01/1.0.0/visual/payload.visual-projection.json",
                    2460,
                    "6c8dfb3b1bba8836a2191c7b8f7bce95bf693f25cceed54edad5f97c127533c1",
                ),
            ],
            ["payload", "catalog", "review"],
        ),
        _default_card(
            "forest_birch",
            "Forest Birch",
            "No reproducible Forest Birch review artifact is registered.",
            2,
            "vegetation",
            "unavailable",
            "unavailable",
            "Ignored build output cannot be review evidence; publish a versioned artifact first.",
            [],
            ["forest", "tree", "tripo", "review"],
        ),
        _default_card(
            "forest_pine",
            "Forest Pine",
            "No reproducible Forest Pine review artifact is registered.",
            3,
            "vegetation",
            "unavailable",
            "unavailable",
            "Ignored build output cannot be review evidence; publish a versioned artifact first.",
            [],
            ["forest", "tree", "tripo", "review"],
        ),
        _default_card(
            "forest_boulder",
            "Forest Boulder",
            "No reproducible Forest Boulder review artifact is registered.",
            4,
            "terrain_detail",
            "unavailable",
            "unavailable",
            "Ignored build output cannot be review evidence; publish a versioned artifact first.",
            [],
            ["forest", "rock", "tripo", "review"],
        ),
        _default_card(
            "forest_grass",
            "Forest Grass Clump",
            "No reproducible Forest Grass review artifact is registered.",
            5,
            "vegetation",
            "unavailable",
            "unavailable",
            "Ignored build output cannot be review evidence; publish a versioned artifact first.",
            [],
            ["forest", "grass", "tripo", "review"],
        ),
        _default_card(
            "forest_fern",
            "Forest Fern Clump",
            "No reproducible Forest Fern review artifact is registered.",
            6,
            "vegetation",
            "unavailable",
            "unavailable",
            "Ignored build output cannot be review evidence; publish a versioned artifact first.",
            [],
            ["forest", "fern", "tripo", "review"],
        ),
        _default_card(
            "forest_litter",
            "Forest Floor Litter",
            "No reproducible Forest Litter review artifact is registered.",
            7,
            "terrain_detail",
            "unavailable",
            "unavailable",
            "Ignored build output cannot be review evidence; publish a versioned artifact first.",
            [],
            ["forest", "litter", "tripo", "review"],
        ),
        _default_card(
            "pedestrian_capsule",
            "Pedestrian Capsule Proxy",
            "MuJoCo pedestrian capsule proxy; no qualified pedestrian visual exists.",
            8,
            "scenario_proxy",
            "proxy_only",
            "proxy_only",
            "Physics proxy is evidence of simulation authority, not a qualified visual asset.",
            [
                _evidence(
                    "physics_proxy",
                    "sim/packages/scenarios/open_field_pedestrian_crossing/1.1.0/physics/pedestrian_capsule.xml",
                    306,
                    "3280e54d9d1b129132034c16605576472e2908b9cb886fe43faef179f2a8eb0c",
                )
            ],
            ["pedestrian", "proxy", "mujoco", "review"],
        ),
        _default_card(
            "movable_object_visual_unavailable",
            "Movable Object Visual",
            "No qualified movable-object visual asset is currently registered.",
            9,
            "movable_object",
            "unavailable",
            "unavailable",
            "No movable-object visual has passed conditioning, rights, UE import, and visual review.",
            [],
            ["movable", "unavailable", "review"],
        ),
    ],
}


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GameAssetReviewCatalogError(f"{context} must be an object")
    return value


def _sequence(value: Any, context: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise GameAssetReviewCatalogError(f"{context} must be a list")
    return value


def _string(value: Any, context: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str) or value != value.strip() or (
        not value and not allow_empty
    ):
        raise GameAssetReviewCatalogError(f"{context} must be a trimmed string")
    if len(value) > MAX_STRING_LENGTH:
        raise GameAssetReviewCatalogError(f"{context} exceeds the size limit")
    if _WINDOWS_ABSOLUTE.search(value) or value.startswith(("/", "file://", "file:\\")):
        raise GameAssetReviewCatalogError(
            f"{context} must not contain an absolute filesystem path"
        )
    return value


def _identity(value: Any, context: str) -> str:
    result = _string(value, context)
    if _IDENTITY.fullmatch(result) is None:
        raise GameAssetReviewCatalogError(f"{context} is not a safe identity")
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
        raise GameAssetReviewCatalogError(
            f"{context} has invalid fields; missing={missing}, unknown={unknown}"
        )


def _absolute(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path)))


def _is_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return False
    except OSError as exc:
        raise GameAssetReviewCatalogError(f"cannot inspect path {path}: {exc}") from exc
    return stat.S_ISLNK(metadata.st_mode) or bool(
        getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
    )


def _assert_link_free(path: Path, *, below: Path, context: str) -> Path:
    base = _absolute(below)
    candidate = _absolute(path)
    try:
        relative = candidate.relative_to(base)
    except ValueError as exc:
        raise GameAssetReviewCatalogError(f"{context} escapes the repository root") from exc
    current = base
    if _is_reparse(current):
        raise GameAssetReviewCatalogError(f"{context} repository root is a link or reparse point")
    for part in relative.parts:
        current /= part
        if _is_reparse(current):
            raise GameAssetReviewCatalogError(
                f"{context} contains a link or reparse point: {current}"
            )
    return candidate


def _open_regular_nofollow(
    path: Path,
    *,
    context: str,
    parent_descriptor: int | None = None,
) -> int:
    """Open one regular file without following its final path component."""

    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0)
    if os.name == "nt":
        import ctypes
        import msvcrt

        create_file = ctypes.windll.kernel32.CreateFileW
        create_file.argtypes = [
            ctypes.c_wchar_p,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_void_p,
        ]
        create_file.restype = ctypes.c_void_p
        handle = create_file(
            os.fspath(path),
            0x80000000,  # GENERIC_READ
            0x00000001 | 0x00000002,  # share read/write, but not delete
            None,
            3,  # OPEN_EXISTING
            0x00200000 | 0x08000000,  # OPEN_REPARSE_POINT | SEQUENTIAL_SCAN
            None,
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise GameAssetReviewCatalogError(f"cannot open {context}")
        try:
            descriptor = msvcrt.open_osfhandle(int(handle), flags)
        except Exception:
            ctypes.windll.kernel32.CloseHandle(handle)
            raise
    else:
        try:
            descriptor = os.open(
                path.name if parent_descriptor is not None else path,
                flags | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=parent_descriptor,
            )
        except OSError as exc:
            raise GameAssetReviewCatalogError(f"cannot open {context}: {exc}") from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or bool(
            getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
        ):
            raise GameAssetReviewCatalogError(f"{context} must identify a regular file")
    except Exception:
        os.close(descriptor)
        raise
    return descriptor


def _read_regular_bounded(path: Path, *, limit: int, context: str) -> bytes:
    descriptor = _open_regular_nofollow(path, context=context)
    try:
        before = os.fstat(descriptor)
        if before.st_size > limit:
            raise GameAssetReviewCatalogError(f"{context} exceeds the size limit")
        chunks: list[bytes] = []
        observed = 0
        while True:
            chunk = os.read(descriptor, min(_READ_CHUNK_BYTES, limit + 1 - observed))
            if not chunk:
                break
            observed += len(chunk)
            if observed > limit:
                raise GameAssetReviewCatalogError(f"{context} exceeds the size limit")
            chunks.append(chunk)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
        ) or observed != before.st_size:
            raise GameAssetReviewCatalogError(f"{context} changed while it was read")
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _hash_regular_bounded(
    path: Path,
    *,
    repository: Path,
    expected_size: int,
    limit: int,
    context: str,
) -> str:
    if expected_size > limit:
        raise GameAssetReviewCatalogError(f"{context} exceeds the size limit")
    with _stable_repository_parent(
        repository, path.parent, create=False, context=context
    ) as parent_descriptor:
        descriptor = _open_regular_nofollow(
            path,
            context=context,
            parent_descriptor=parent_descriptor,
        )
        try:
            before = os.fstat(descriptor)
            if before.st_size != expected_size:
                raise GameAssetReviewCatalogError(
                    f"{context} byte length does not match pinned evidence"
                )
            digest = hashlib.sha256()
            observed = 0
            while observed < expected_size:
                chunk = os.read(
                    descriptor,
                    min(_READ_CHUNK_BYTES, expected_size - observed),
                )
                if not chunk:
                    break
                digest.update(chunk)
                observed += len(chunk)
            if os.read(descriptor, 1) or observed != expected_size:
                raise GameAssetReviewCatalogError(f"{context} changed while it was read")
            after = os.fstat(descriptor)
            if (before.st_dev, before.st_ino, before.st_size) != (
                after.st_dev,
                after.st_ino,
                after.st_size,
            ):
                raise GameAssetReviewCatalogError(f"{context} changed while it was read")
            return digest.hexdigest()
        finally:
            os.close(descriptor)


@contextlib.contextmanager
def _stable_repository_parent(
    repository: Path,
    parent: Path,
    *,
    create: bool,
    context: str,
) -> Iterator[int | None]:
    """Hold every parent component against replacement while a file is used."""

    relative = parent.relative_to(repository)
    if os.name != "nt":
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
        descriptors: list[int] = []
        try:
            dir_descriptor = os.open(repository, flags)
            descriptors.append(dir_descriptor)
            for part in relative.parts:
                if create:
                    try:
                        os.mkdir(part, dir_fd=dir_descriptor)
                    except FileExistsError:
                        pass
                child = os.open(part, flags, dir_fd=dir_descriptor)
                descriptors.append(child)
                dir_descriptor = child
            yield dir_descriptor
        except OSError as exc:
            raise GameAssetReviewCatalogError(
                f"{context} parent is not a stable link-free directory: {exc}"
            ) from exc
        finally:
            for descriptor in reversed(descriptors):
                os.close(descriptor)
        return

    import ctypes

    handles: list[int] = []
    create_file = ctypes.windll.kernel32.CreateFileW
    create_file.argtypes = [
        ctypes.c_wchar_p,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_void_p,
    ]
    create_file.restype = ctypes.c_void_p
    close_handle = ctypes.windll.kernel32.CloseHandle
    close_handle.argtypes = [ctypes.c_void_p]

    class _FileAttributeTagInfo(ctypes.Structure):
        _fields_ = [
            ("file_attributes", ctypes.c_uint32),
            ("reparse_tag", ctypes.c_uint32),
        ]

    get_info = ctypes.windll.kernel32.GetFileInformationByHandleEx
    get_info.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_uint32,
    ]
    get_info.restype = ctypes.c_int

    def hold_directory(directory_path: Path) -> int:
        handle = create_file(
            os.fspath(directory_path),
            0x80000000,
            0x00000001 | 0x00000002,
            None,
            3,
            0x02000000 | 0x00200000,  # BACKUP_SEMANTICS | OPEN_REPARSE_POINT
            None,
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise GameAssetReviewCatalogError(
                f"cannot stabilize {context} parent: {directory_path}"
            )
        info = _FileAttributeTagInfo()
        if not get_info(handle, 9, ctypes.byref(info), ctypes.sizeof(info)):
            close_handle(handle)
            raise GameAssetReviewCatalogError(
                f"cannot inspect {context} parent handle: {directory_path}"
            )
        if not info.file_attributes & 0x10 or info.file_attributes & _REPARSE_POINT:
            close_handle(handle)
            raise GameAssetReviewCatalogError(
                f"{context} parent contains a link or reparse point: {directory_path}"
            )
        return int(handle)

    current_path = repository
    try:
        handles.append(hold_directory(repository))
        for part in relative.parts:
            current_path /= part
            if create:
                try:
                    current_path.mkdir()
                except FileExistsError:
                    pass
            handles.append(hold_directory(current_path))
        yield None
    finally:
        for handle in reversed(handles):
            close_handle(handle)


def _repo_path(
    repo_root: Path,
    relative_value: Any,
    context: str,
    *,
    must_exist: bool,
) -> tuple[str, Path]:
    path_value = (
        PurePosixPath(relative_value).as_posix()
        if isinstance(relative_value, Path)
        else relative_value
    )
    relative = _string(path_value, context)
    posix = PurePosixPath(relative)
    if (
        posix.is_absolute()
        or "\\" in relative
        or ":" in relative
        or any(part in {"", ".", ".."} for part in posix.parts)
    ):
        raise GameAssetReviewCatalogError(
            f"{context} must be one safe repository-relative POSIX path"
        )
    candidate = _assert_link_free(
        repo_root.joinpath(*posix.parts), below=repo_root, context=context
    )
    if must_exist and not candidate.is_file():
        raise GameAssetReviewCatalogError(f"{context} must identify a regular file")
    return posix.as_posix(), candidate


def _load_spec(spec: Mapping[str, Any] | Path, *, repo_root: Path) -> Mapping[str, Any]:
    if isinstance(spec, Mapping):
        return spec
    _, path = _repo_path(repo_root, spec, "asset review spec", must_exist=True)
    try:
        payload = _read_regular_bounded(
            path, limit=MAX_SPEC_BYTES, context="asset review spec"
        )
        value = yaml.safe_load(payload.decode("utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise GameAssetReviewCatalogError(f"cannot read asset review spec: {exc}") from exc
    return _mapping(value, "asset review spec")


def _preflight_evidence_budget(raw_cards: Sequence[Any]) -> None:
    """Reject excessive evidence declarations before opening any evidence file."""

    total = 0
    for card_index, raw_card in enumerate(raw_cards):
        card_context = f"asset review spec.cards[{card_index}]"
        card = _mapping(raw_card, card_context)
        evidence = _sequence(card.get("evidence"), f"{card_context}.evidence")
        if len(evidence) > MAX_EVIDENCE_PER_CARD:
            raise GameAssetReviewCatalogError(
                f"{card_context}.evidence contains too many items"
            )
        for evidence_index, raw_item in enumerate(evidence):
            item_context = f"{card_context}.evidence[{evidence_index}]"
            item = _mapping(raw_item, item_context)
            byte_length = item.get("bytes")
            if (
                isinstance(byte_length, bool)
                or not isinstance(byte_length, int)
                or byte_length < 0
            ):
                raise GameAssetReviewCatalogError(
                    f"{item_context}.bytes must be a non-negative integer"
                )
            if byte_length > MAX_EVIDENCE_BYTES:
                raise GameAssetReviewCatalogError(
                    f"{item_context} exceeds the size limit"
                )
            total += byte_length
            if total > MAX_TOTAL_EVIDENCE_BYTES:
                raise GameAssetReviewCatalogError(
                    "asset review spec evidence exceeds the size limit"
                )


def _compile_evidence(
    raw_items: Any,
    *,
    repo_root: Path,
    context: str,
) -> list[dict[str, Any]]:
    evidence: list[dict[str, Any]] = []
    seen_roles: set[str] = set()
    seen_paths: set[str] = set()
    raw_evidence = _sequence(raw_items, context)
    if len(raw_evidence) > MAX_EVIDENCE_PER_CARD:
        raise GameAssetReviewCatalogError(f"{context} contains too many items")
    for index, raw_item in enumerate(raw_evidence):
        item_context = f"{context}[{index}]"
        item = _mapping(raw_item, item_context)
        _strict_keys(
            item,
            required={"role", "path", "bytes", "sha256"},
            optional=set(),
            context=item_context,
        )
        role = _identity(item.get("role"), f"{item_context}.role")
        relative, path = _repo_path(
            repo_root, item.get("path"), f"{item_context}.path", must_exist=True
        )
        byte_length = item.get("bytes")
        if isinstance(byte_length, bool) or not isinstance(byte_length, int) or byte_length < 0:
            raise GameAssetReviewCatalogError(
                f"{item_context}.bytes must be a non-negative integer"
            )
        if byte_length > MAX_EVIDENCE_BYTES:
            raise GameAssetReviewCatalogError(f"{item_context} exceeds the size limit")
        expected_sha = _string(item.get("sha256"), f"{item_context}.sha256")
        if _DIGEST.fullmatch(expected_sha) is None:
            raise GameAssetReviewCatalogError(f"{item_context}.sha256 is invalid")
        observed_sha = _hash_regular_bounded(
            path,
            repository=repo_root,
            expected_size=byte_length,
            limit=MAX_EVIDENCE_BYTES,
            context=item_context,
        )
        if observed_sha != expected_sha:
            raise GameAssetReviewCatalogError(
                f"{item_context} sha256 does not match pinned evidence"
            )
        if role in seen_roles:
            raise GameAssetReviewCatalogError(f"{context} contains duplicate evidence role {role}")
        if relative.casefold() in seen_paths:
            raise GameAssetReviewCatalogError(f"{context} contains duplicate evidence path")
        seen_roles.add(role)
        seen_paths.add(relative.casefold())
        evidence.append(
            {
                "role": role,
                "path": relative,
                "bytes": byte_length,
                "sha256": expected_sha,
            }
        )
    evidence.sort(key=lambda item: (item["role"], item["path"]))
    return evidence


def _compile_card(
    raw_card: Any,
    *,
    repo_root: Path,
    index: int,
) -> dict[str, Any]:
    context = f"asset review spec.cards[{index}]"
    card = _mapping(raw_card, context)
    _strict_keys(
        card,
        required={
            "id",
            "title",
            "description",
            "order",
            "asset_class",
            "review",
            "evidence",
        },
        optional={"tags"},
        context=context,
    )
    card_id = _identity(card.get("id"), f"{context}.id")
    order = card.get("order")
    if isinstance(order, bool) or not isinstance(order, int) or order < 0:
        raise GameAssetReviewCatalogError(f"{context}.order must be a non-negative integer")
    asset_class = _identity(card.get("asset_class"), f"{context}.asset_class")
    if asset_class not in _ASSET_CLASSES:
        raise GameAssetReviewCatalogError(f"{context}.asset_class is unsupported")
    review = _mapping(card.get("review"), f"{context}.review")
    _strict_keys(
        review,
        required={"stage", "disposition", "reason"},
        optional=set(),
        context=f"{context}.review",
    )
    stage = _identity(review.get("stage"), f"{context}.review.stage")
    disposition = _identity(
        review.get("disposition"), f"{context}.review.disposition"
    )
    if _STAGE_DISPOSITION.get(stage) != disposition:
        raise GameAssetReviewCatalogError(
            f"{context}.review stage/disposition contract is unsupported"
        )
    evidence = _compile_evidence(
        card.get("evidence"), repo_root=repo_root, context=f"{context}.evidence"
    )
    roles = frozenset(item["role"] for item in evidence)
    if roles != _REQUIRED_ROLES[stage]:
        raise GameAssetReviewCatalogError(
            f"{context}.evidence roles must be {sorted(_REQUIRED_ROLES[stage])}"
        )
    tags = [
        _identity(value, f"{context}.tags[{tag_index}]")
        for tag_index, value in enumerate(
            _sequence(card.get("tags", []), f"{context}.tags")
        )
    ]
    if len(set(tags)) != len(tags):
        raise GameAssetReviewCatalogError(f"{context}.tags must be unique")
    available = stage != "unavailable"
    return {
        "id": card_id,
        "title": _string(card.get("title"), f"{context}.title"),
        "description": _string(card.get("description"), f"{context}.description"),
        "order": order,
        "asset_class": asset_class,
        "review": {
            "stage": stage,
            "disposition": disposition,
            "reason": _string(review.get("reason"), f"{context}.review.reason"),
        },
        "evidence": evidence,
        "render_policy": dict(_POLICY),
        "capabilities": {
            "selectable_for_review": available,
            "runnable": False,
            "qualified_visual": False,
        },
        "tags": sorted(tags),
    }


def build_game_asset_review_catalog(
    repo_root: Path,
    *,
    spec: Mapping[str, Any] | Path = DEFAULT_ASSET_REVIEW_SPEC,
) -> dict[str, Any]:
    """Validate pinned evidence and return one canonical review catalog."""

    repository = _absolute(Path(repo_root))
    if not repository.is_dir() or _is_reparse(repository):
        raise GameAssetReviewCatalogError(
            "repo_root must be a regular, link-free directory"
        )
    document_spec = _load_spec(spec, repo_root=repository)
    _strict_keys(
        document_spec,
        required={"schema", "title", "cards"},
        optional=set(),
        context="asset review spec",
    )
    if document_spec.get("schema") != ASSET_REVIEW_SPEC_SCHEMA:
        raise GameAssetReviewCatalogError("asset review spec schema is unsupported")
    raw_cards = _sequence(document_spec.get("cards"), "asset review spec.cards")
    if not raw_cards:
        raise GameAssetReviewCatalogError("asset review spec.cards must be non-empty")
    if len(raw_cards) > MAX_CARDS:
        raise GameAssetReviewCatalogError("asset review spec.cards contains too many cards")
    _preflight_evidence_budget(raw_cards)
    cards = [
        _compile_card(raw_card, repo_root=repository, index=index)
        for index, raw_card in enumerate(raw_cards)
    ]
    card_ids = [card["id"] for card in cards]
    if len(set(card_ids)) != len(card_ids):
        raise GameAssetReviewCatalogError("asset review spec.cards contains duplicate ids")
    cards.sort(key=lambda card: (card["order"], card["id"]))
    body: dict[str, Any] = {
        "schema": ASSET_REVIEW_CATALOG_SCHEMA,
        "title": _string(document_spec.get("title"), "asset review spec.title"),
        "policy": dict(_POLICY),
        "coverage": {
            "qualified_movable_object_visual": {
                "available": False,
                "reason": "no qualified movable-object visual is registered",
            }
        },
        "cards": cards,
    }
    return {**body, "digest": digest_document(body)}


def write_game_asset_review_catalog(
    repo_root: Path,
    output_path: Path,
    *,
    spec: Mapping[str, Any] | Path = DEFAULT_ASSET_REVIEW_SPEC,
) -> Path:
    """Publish a canonical catalog without overwriting conflicting evidence."""

    repository = _absolute(Path(repo_root))
    relative, target = _repo_path(
        repository, output_path, "output_path", must_exist=False
    )
    if not PurePosixPath(relative).name:
        raise GameAssetReviewCatalogError("output_path must name a file")
    payload = canonical_json_bytes(
        build_game_asset_review_catalog(repository, spec=spec)
    )
    if len(payload) > MAX_OUTPUT_BYTES:
        raise GameAssetReviewCatalogError("asset review catalog exceeds the size limit")
    with _stable_repository_parent(
        repository,
        target.parent,
        create=True,
        context="output_path",
    ) as parent_descriptor:
        temporary_name = f".{target.name}.{secrets.token_hex(16)}.tmp"
        if parent_descriptor is None:
            temporary = target.parent / temporary_name
            descriptor = os.open(
                temporary,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0),
                0o600,
            )
        else:
            temporary = None
            descriptor = os.open(
                temporary_name,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                0o600,
                dir_fd=parent_descriptor,
            )
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            try:
                if parent_descriptor is None:
                    assert temporary is not None
                    os.link(temporary, target, follow_symlinks=False)
                else:
                    os.link(
                        temporary_name,
                        target.name,
                        src_dir_fd=parent_descriptor,
                        dst_dir_fd=parent_descriptor,
                        follow_symlinks=False,
                    )
            except FileExistsError:
                if parent_descriptor is None:
                    observed = _read_regular_bounded(
                        target, limit=MAX_OUTPUT_BYTES, context="output_path"
                    )
                else:
                    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
                    existing = os.open(target.name, flags, dir_fd=parent_descriptor)
                    try:
                        metadata = os.fstat(existing)
                        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > MAX_OUTPUT_BYTES:
                            raise GameAssetReviewCatalogError(
                                "output_path is not a bounded regular file"
                            )
                        chunks: list[bytes] = []
                        remaining = MAX_OUTPUT_BYTES + 1
                        while remaining:
                            chunk = os.read(existing, min(_READ_CHUNK_BYTES, remaining))
                            if not chunk:
                                break
                            chunks.append(chunk)
                            remaining -= len(chunk)
                        observed = b"".join(chunks)
                    finally:
                        os.close(existing)
                if observed != payload:
                    raise GameAssetReviewCatalogError(
                        "output_path already contains conflicting asset review metadata"
                    )
        finally:
            if parent_descriptor is None:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            else:
                try:
                    os.unlink(temporary_name, dir_fd=parent_descriptor)
                except FileNotFoundError:
                    pass
    return target


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compile deterministic, presentation-only asset review cards."
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--spec", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Compile the requested catalog and print its repository-relative path."""

    args = _parser().parse_args(argv)
    repo_root = _absolute(args.repo_root)
    spec: Mapping[str, Any] | Path = (
        DEFAULT_ASSET_REVIEW_SPEC if args.spec is None else args.spec
    )
    output = write_game_asset_review_catalog(
        repo_root, args.output, spec=spec
    )
    print(output.relative_to(repo_root).as_posix())
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised through CLI
    raise SystemExit(main())


__all__ = [
    "ASSET_REVIEW_CATALOG_SCHEMA",
    "ASSET_REVIEW_SPEC_SCHEMA",
    "DEFAULT_ASSET_REVIEW_SPEC",
    "GameAssetReviewCatalogError",
    "build_game_asset_review_catalog",
    "main",
    "write_game_asset_review_catalog",
]
