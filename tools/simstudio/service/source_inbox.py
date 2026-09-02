"""Managed source intake for the local SimStudio service."""

from __future__ import annotations

import os
import re
import secrets
import shutil
from collections.abc import AsyncIterable, Mapping
from pathlib import Path
from typing import Any

from sim.catalog.importers.contracts import ImportFailure, SourceFile
from sim.catalog.importers.intake import SourceIntake
from sim.runtime.coordinator.atomic_file import replace_file_with_retry

from .store import StudioStore

_SOURCE_SCHEMA = "lingtu.sim.studio.inbox-source.v1"
_SOURCE_LIST_SCHEMA = "lingtu.sim.studio.inbox-source-list.v1"
_DEFAULT_MAX_UPLOAD_BYTES = 512 * 1024 * 1024
_ARCHIVE_SUFFIXES = {
    ".tar.gz": (".tar.gz", "tar.gz"),
    ".tgz": (".tar.gz", "tar.gz"),
    ".zip": (".zip", "zip"),
    ".tar": (".tar", "tar"),
}
_ROBOT_MODEL_FORMATS = {".xml": "mjcf", ".urdf": "urdf"}
_MESH_SUFFIXES = {".dae", ".fbx", ".glb", ".gltf", ".obj", ".ply", ".stl"}
_TEXTURE_SUFFIXES = {".bmp", ".exr", ".jpeg", ".jpg", ".png", ".tga", ".tif", ".tiff"}
_HEIGHTMAP_SUFFIXES = {".dem", ".exr", ".r16", ".raw", ".tif", ".tiff"}
_HEIGHTMAP_HINTS = {"dem", "elevation", "height", "heightmap", "terrain"}


class SourceInboxError(ValueError):
    """Stable fail-closed error raised by the managed source inbox."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.details = dict(details or {})


class SourceInboxService:
    """Store uploaded import archives below a private inbox."""

    def __init__(
        self,
        root: Path,
        *,
        max_upload_bytes: int = _DEFAULT_MAX_UPLOAD_BYTES,
        intake: SourceIntake | None = None,
    ) -> None:
        if isinstance(max_upload_bytes, bool) or not isinstance(max_upload_bytes, int) or max_upload_bytes < 1:
            raise ValueError("max_upload_bytes must be a positive integer")
        self.root = Path(os.path.abspath(os.fspath(root)))
        self.max_upload_bytes = max_upload_bytes
        self.intake = intake or SourceIntake()
        StudioStore._ensure_directory(self.root)
        StudioStore._assert_no_reparse_components(self.root)
        self._object_root = self.root / "objects"
        self._staging_root = self.root / ".staging"
        StudioStore._ensure_directory(self._object_root)
        StudioStore._ensure_directory(self._staging_root)
        StudioStore._assert_no_reparse_components(self._object_root, below=self.root)
        StudioStore._assert_no_reparse_components(self._staging_root, below=self.root)

    async def upload(
        self,
        filename: str,
        chunks: AsyncIterable[bytes],
    ) -> dict[str, Any]:
        """Stream one archive into the managed inbox atomically."""

        canonical_suffix, archive_format = self._archive_format(filename)
        temporary = self._staging_root / f"upload-{secrets.token_hex(16)}.tmp"
        size = 0
        try:
            StudioStore._assert_no_reparse_components(temporary, below=self.root)
            with temporary.open("xb") as handle:
                async for chunk in chunks:
                    if not isinstance(chunk, bytes):
                        raise SourceInboxError(
                            "SIMSTUDIO_INBOX_INVALID_CHUNK",
                            "uploaded source chunks must be bytes",
                        )
                    if not chunk:
                        continue
                    size += len(chunk)
                    if size > self.max_upload_bytes:
                        raise SourceInboxError(
                            "SIMSTUDIO_INBOX_SIZE_LIMIT",
                            "uploaded source exceeds the configured size limit",
                            details={"maximum_bytes": self.max_upload_bytes},
                        )
                    handle.write(chunk)
                handle.flush()
                os.fsync(handle.fileno())
            if size == 0:
                raise SourceInboxError(
                    "SIMSTUDIO_INBOX_EMPTY_SOURCE",
                    "uploaded source archive is empty",
                )

            source_id = secrets.token_hex(16)
            relative = f"objects/{source_id}{canonical_suffix}"
            target = self.root.joinpath(*relative.split("/"))
            StudioStore._assert_no_reparse_components(target.parent, below=self.root)
            replace_file_with_retry(temporary, target)
            StudioStore._assert_no_reparse_components(target, below=self.root)
            return {
                "schema": _SOURCE_SCHEMA,
                "source_id": source_id,
                "entry": relative,
                "original_name": filename,
                "bytes": size,
                "archive_format": archive_format,
            }
        finally:
            temporary.unlink(missing_ok=True)

    def list_sources(self) -> dict[str, Any]:
        """List managed archives in deterministic order."""

        StudioStore._assert_no_reparse_components(self._object_root, below=self.root)
        sources: list[dict[str, Any]] = []
        for path in sorted(self._object_root.iterdir(), key=lambda item: item.name):
            if not path.is_file() or path.is_symlink():
                continue
            StudioStore._assert_no_reparse_components(path, below=self.root)
            source_id, archive_format = self._identity_from_object(path)
            size = path.stat().st_size
            sources.append(
                {
                    "schema": _SOURCE_SCHEMA,
                    "source_id": source_id,
                    "entry": path.relative_to(self.root).as_posix(),
                    "bytes": size,
                    "archive_format": archive_format,
                }
            )
        return {"schema": _SOURCE_LIST_SCHEMA, "sources": sources}

    def inspect(self, source_id: str) -> dict[str, Any]:
        """Safely materialize and classify one managed source."""

        source = self._object_for_id(source_id)
        stored_source_id, archive_format = self._identity_from_object(source)
        source_size = source.stat().st_size
        destination = self._staging_root / f"inspection-{stored_source_id}-{secrets.token_hex(8)}"
        try:
            intake = self.intake.materialize(source, destination)
            files = list(intake.files)
            robot_models = self._robot_models(files, root=intake.root)
            licenses = self._licenses(files)
            heightmaps = self._heightmaps(files)
            meshes = self._by_suffix(files, _MESH_SUFFIXES)
            textures = self._by_suffix(files, _TEXTURE_SUFFIXES)
            return {
                "schema": "lingtu.sim.studio.source-inspection.v1",
                "source": {
                    "source_id": stored_source_id,
                    "entry": source.relative_to(self.root).as_posix(),
                    "bytes": source_size,
                    "archive_format": archive_format,
                },
                "summary": {
                    "files": len(files),
                    "total_bytes": sum(item.size for item in files),
                },
                "candidates": {
                    "robot_models": robot_models,
                    "licenses": licenses,
                    "heightmaps": heightmaps,
                    "meshes": meshes,
                    "textures": textures,
                },
                "recommendations": self._recommendations(
                    robot_models=robot_models,
                    licenses=licenses,
                    heightmaps=heightmaps,
                    meshes=meshes,
                ),
                "files": [item.to_dict() for item in files],
            }
        except ImportFailure as exc:
            diagnostic = exc.to_diagnostic().to_dict()
            raise SourceInboxError(
                "SIMSTUDIO_INBOX_INSPECTION_FAILED",
                "source archive failed safe inspection",
                details={"diagnostic": diagnostic},
            ) from exc
        finally:
            shutil.rmtree(destination, ignore_errors=True)

    def _object_for_id(self, source_id: str) -> Path:
        if (
            not isinstance(source_id, str)
            or not source_id
            or len(source_id) > 64
            or not source_id.isascii()
            or not source_id.isalnum()
        ):
            raise SourceInboxError(
                "SIMSTUDIO_INBOX_INVALID_SOURCE_ID",
                "source_id must be a plain opaque identifier",
            )
        for canonical_suffix, _ in set(_ARCHIVE_SUFFIXES.values()):
            path = self._object_root / f"{source_id}{canonical_suffix}"
            if path.is_file() and not path.is_symlink():
                StudioStore._assert_no_reparse_components(path, below=self.root)
                return path
        raise SourceInboxError(
            "SIMSTUDIO_INBOX_SOURCE_NOT_FOUND",
            "source was not found",
            details={"source_id": source_id},
        )

    @staticmethod
    def _record(item: SourceFile, **extra: str) -> dict[str, Any]:
        return {**item.to_dict(), **extra}

    @classmethod
    def _robot_models(cls, files: list[SourceFile], *, root: Path) -> list[dict[str, Any]]:
        candidates: list[dict[str, Any]] = []
        for item in files:
            suffix = Path(item.path).suffix.casefold()
            expected_format = _ROBOT_MODEL_FORMATS.get(suffix)
            if expected_format is None:
                continue
            source = root / Path(item.path)
            with source.open("rb") as handle:
                prefix = handle.read(256 * 1024)
            root_pattern = rb"<\s*mujoco(?:\s|>)" if expected_format == "mjcf" else rb"<\s*robot(?:\s|>)"
            if re.search(root_pattern, prefix) is None:
                continue
            candidates.append(cls._record(item, format=expected_format))
        priority = {"mjcf": 0, "urdf": 1}
        return sorted(candidates, key=lambda item: (priority[item["format"]], item["path"]))

    @classmethod
    def _licenses(cls, files: list[SourceFile]) -> list[dict[str, Any]]:
        candidates = [
            cls._record(item)
            for item in files
            if Path(item.path).name.casefold().startswith(("license", "copying", "notice"))
        ]
        return sorted(
            candidates,
            key=lambda item: (len(Path(item["path"]).parts), item["path"].casefold()),
        )

    @classmethod
    def _heightmaps(cls, files: list[SourceFile]) -> list[dict[str, Any]]:
        candidates: list[dict[str, Any]] = []
        for item in files:
            path = Path(item.path)
            suffix = path.suffix.casefold()
            hinted = any(hint in path.stem.casefold() for hint in _HEIGHTMAP_HINTS)
            if suffix in _HEIGHTMAP_SUFFIXES or (suffix in _TEXTURE_SUFFIXES and hinted):
                candidates.append(cls._record(item))
        return sorted(candidates, key=lambda item: item["path"])

    @classmethod
    def _by_suffix(cls, files: list[SourceFile], suffixes: set[str]) -> list[dict[str, Any]]:
        return [
            cls._record(item)
            for item in files
            if Path(item.path).suffix.casefold() in suffixes
        ]

    @staticmethod
    def _recommendations(
        *,
        robot_models: list[dict[str, Any]],
        licenses: list[dict[str, Any]],
        heightmaps: list[dict[str, Any]],
        meshes: list[dict[str, Any]],
    ) -> dict[str, Any]:
        license_file = licenses[0]["path"] if licenses else None
        robot = None
        if robot_models:
            robot = {
                "source_format": robot_models[0]["format"],
                "source_model": robot_models[0]["path"],
                "license_file": license_file,
            }
        world = None
        if heightmaps:
            world = {
                "heightmap": heightmaps[0]["path"],
                "mesh": meshes[0]["path"] if meshes else None,
                "license_file": license_file,
            }
        return {"robot": robot, "world": world}

    @staticmethod
    def _archive_format(filename: str) -> tuple[str, str]:
        if (
            not isinstance(filename, str)
            or not filename
            or len(filename) > 255
            or filename in {".", ".."}
            or "\x00" in filename
            or "/" in filename
            or "\\" in filename
            or ":" in filename
            or Path(filename).name != filename
        ):
            raise SourceInboxError(
                "SIMSTUDIO_INBOX_UNSAFE_NAME",
                "upload filename must be one plain archive filename",
            )
        lowered = filename.casefold()
        for suffix, result in _ARCHIVE_SUFFIXES.items():
            if lowered.endswith(suffix):
                return result
        raise SourceInboxError(
            "SIMSTUDIO_INBOX_UNSUPPORTED_FORMAT",
            "upload must be a ZIP, TAR, TAR.GZ, or TGZ archive",
            details={"supported_suffixes": sorted(_ARCHIVE_SUFFIXES)},
        )

    @staticmethod
    def _identity_from_object(path: Path) -> tuple[str, str]:
        lowered = path.name.casefold()
        for suffix, (_, archive_format) in _ARCHIVE_SUFFIXES.items():
            canonical_suffix = ".tar.gz" if suffix == ".tgz" else suffix
            if lowered.endswith(canonical_suffix):
                source_id = lowered[: -len(canonical_suffix)]
                if source_id and len(source_id) <= 64 and source_id.isascii() and source_id.isalnum():
                    return source_id, archive_format
        raise SourceInboxError(
            "SIMSTUDIO_INBOX_INTEGRITY_ERROR",
            "inbox object has an invalid name",
            details={"entry": path.name},
        )


__all__ = ["SourceInboxError", "SourceInboxService"]
