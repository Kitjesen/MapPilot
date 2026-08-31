"""Safe inspection of artifacts owned by a SimStudio run."""

from __future__ import annotations

import hashlib
import json
import mimetypes
import os
import stat
from contextlib import contextmanager
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO, Iterator

from .models import RecordNotFound, RunRecord, StoreValidationError
from .store import StudioStore


class ArtifactServiceError(RuntimeError):
    """Base error for run artifact inspection."""


class ArtifactNotFound(ArtifactServiceError, KeyError):
    """Raised when a run or artifact does not exist."""


class ArtifactSecurityError(ArtifactServiceError, ValueError):
    """Raised when an artifact path cannot be proven to be run-owned."""


class ArtifactService:
    """Expose only deterministic, bounded views below one run-owned root."""

    def __init__(self, store: StudioStore, *, max_preview_bytes: int = 1_048_576) -> None:
        if isinstance(max_preview_bytes, bool) or not isinstance(max_preview_bytes, int) or max_preview_bytes <= 0:
            raise ValueError("max_preview_bytes must be a positive integer")
        self.store = store
        self.max_preview_bytes = max_preview_bytes

    def list_artifacts(self, run_id: str) -> list[dict[str, Any]]:
        """Return a stable inventory of files below ``run_id``'s root."""

        root = self._root(self._run(run_id))
        self._reject_reparse(root)
        if not root.exists():
            return []
        if not root.is_dir():
            raise ArtifactSecurityError("run artifact root is not a directory")
        entries: list[dict[str, Any]] = []
        for current, directories, files in os.walk(root, topdown=True, followlinks=False):
            current_path = Path(current)
            directories[:] = sorted(directories)
            files[:] = sorted(files)
            self._reject_reparse(current_path)
            for name in directories:
                path = current_path / name
                self._reject_reparse(path)
                entries.append(self._metadata(root, path, kind="directory"))
            for name in files:
                path = current_path / name
                self._reject_reparse(path)
                if not path.is_file():
                    raise ArtifactSecurityError(f"artifact entry is not a regular file: {name}")
                entries.append(self._metadata(root, path, kind="file"))
        return sorted(entries, key=lambda item: (item["path"], item["kind"]))

    def trusted_run_root(self, run_id: str) -> Path:
        """Return a validated run-owned directory to another backend service.

        This is an internal application-service seam.  HTTP adapters must keep
        using opaque artifact identifiers and must never serialize this path.
        """

        root = self._root(self._run(run_id))
        self._reject_reparse(root)
        if root.exists() and not root.is_dir():
            raise ArtifactSecurityError("run artifact root is not a directory")
        return root

    def get_artifact(self, run_id: str, relative_path: str) -> dict[str, Any]:
        """Return metadata for one run-owned relative path."""

        root = self._root(self._run(run_id))
        path = self._path(root, relative_path, must_exist=True)
        if path.is_dir():
            return self._metadata(root, path, kind="directory")
        if not path.is_file():
            raise ArtifactNotFound(relative_path)
        return self._metadata(root, path, kind="file")

    def preview(self, run_id: str, relative_path: str, *, max_bytes: int | None = None) -> dict[str, Any]:
        """Return a bounded UTF-8 text/JSON preview without exposing file APIs."""

        limit = self.max_preview_bytes if max_bytes is None else max_bytes
        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0 or limit > self.max_preview_bytes:
            raise ValueError(f"max_bytes must be between 1 and {self.max_preview_bytes}")
        root = self._root(self._run(run_id))
        path = self._path(root, relative_path, must_exist=True)
        if not path.is_file():
            raise ArtifactNotFound(relative_path)
        with self._open_safe_file(path) as handle:
            metadata = self._metadata_from_handle(root, path, handle)
            handle.seek(0)
            data = handle.read(limit + 1)
        truncated = len(data) > limit
        sample = data[:limit]
        result = {
            "path": metadata["path"],
            "size": metadata["size"],
            "sha256": metadata["sha256"],
            "mime_type": mimetypes.guess_type(path.name)[0] or "application/octet-stream",
            "previewable": False,
            "encoding": None,
            "content": None,
            "truncated": truncated,
        }
        try:
            text = sample.decode("utf-8")
        except UnicodeDecodeError:
            result["reason"] = "not_utf8_text"
            return result
        result.update({"previewable": True, "encoding": "utf-8", "content": text})
        if path.suffix.lower() == ".json":
            try:
                json.loads(text)
            except (TypeError, ValueError):
                result["reason"] = "invalid_json_preview"
            else:
                result["format"] = "json"
        else:
            result["format"] = "text"
        return result

    def _run(self, run_id: str) -> RunRecord:
        try:
            return self.store.get_run(run_id)
        except (KeyError, RecordNotFound) as exc:
            raise ArtifactNotFound(f"run {run_id} was not found") from exc

    def _root(self, run: RunRecord) -> Path:
        relative = run.payload.get("artifact_path")
        try:
            safe = StudioStore.validate_relative_path(relative, context="run.artifact_path")
        except (StoreValidationError, TypeError) as exc:
            raise ArtifactSecurityError("run artifact path is invalid") from exc
        if PurePosixPath(safe).name != run.id:
            raise ArtifactSecurityError("run artifact path is not bound to the run id")
        base = self.store.root
        root = base.joinpath(*safe.split("/"))
        try:
            root.relative_to(base)
            StudioStore._assert_no_reparse_components(root, below=base)
            root.resolve().relative_to(base)
        except (StoreValidationError, ValueError, OSError) as exc:
            raise ArtifactSecurityError("artifact root escapes the Studio store") from exc
        return root

    def _reject_reparse(self, path: Path) -> None:
        try:
            StudioStore._assert_no_reparse_components(path, below=self.store.root)
        except StoreValidationError as exc:
            raise ArtifactSecurityError(f"artifact tree contains a reparse point: {path.name}") from exc

    def _path(self, root: Path, relative_path: str, *, must_exist: bool) -> Path:
        try:
            safe = StudioStore.validate_relative_path(relative_path, context="artifact path")
        except StoreValidationError as exc:
            raise ArtifactSecurityError(str(exc)) from exc
        candidate = root.joinpath(*safe.split("/"))
        try:
            candidate.relative_to(root)
            self._reject_reparse(candidate)
            resolved = candidate.resolve()
            resolved.relative_to(root.resolve())
        except (ValueError, OSError) as exc:
            raise ArtifactSecurityError("artifact path escapes the run-owned root") from exc
        if must_exist and not resolved.exists():
            raise ArtifactNotFound(safe)
        return resolved

    def _metadata(self, root: Path, path: Path, *, kind: str) -> dict[str, Any]:
        relative = path.relative_to(root).as_posix()
        if kind == "directory":
            return {"path": relative, "kind": kind, "size": 0, "sha256": None}
        with self._open_safe_file(path) as handle:
            return self._metadata_from_handle(root, path, handle)

    @staticmethod
    def _metadata_from_handle(root: Path, path: Path, handle: BinaryIO) -> dict[str, Any]:
        relative = path.relative_to(root).as_posix()
        digest = hashlib.sha256()
        size = 0
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
            size += len(chunk)
        return {"path": relative, "kind": "file", "size": size, "sha256": digest.hexdigest()}

    @contextmanager
    def _open_safe_file(self, path: Path) -> Iterator[BinaryIO]:
        try:
            if os.name == "nt":
                with _open_windows_artifact_file(path) as handle:
                    _assert_open_artifact_file(handle, path)
                    yield handle
                return
            flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
            descriptor = os.open(path, flags)
            try:
                with os.fdopen(descriptor, "rb") as handle:
                    descriptor = -1
                    _assert_open_artifact_file(handle, path)
                    yield handle
            finally:
                if descriptor >= 0:
                    os.close(descriptor)
        except (OSError, StoreValidationError) as exc:
            raise ArtifactSecurityError(f"artifact file identity is unsafe: {path.name}") from exc


def _assert_open_artifact_file(handle: BinaryIO, path: Path) -> None:
    opened = os.fstat(handle.fileno())
    try:
        named = os.lstat(path)
    except FileNotFoundError as exc:
        raise StoreValidationError(f"artifact path changed while opening: {path}") from exc
    if (
        not stat.S_ISREG(opened.st_mode)
        or opened.st_nlink != 1
        or not stat.S_ISREG(named.st_mode)
        or named.st_nlink != 1
        or (opened.st_dev, opened.st_ino) != (named.st_dev, named.st_ino)
    ):
        raise StoreValidationError(f"artifact file identity is unsafe: {path}")


@contextmanager
def _open_windows_artifact_file(path: Path) -> Iterator[BinaryIO]:
    import ctypes
    import msvcrt
    from ctypes import wintypes

    generic_read = 0x80000000
    file_read_attributes = 0x0080
    file_share_read = 0x00000001
    file_share_write = 0x00000002
    file_share_delete = 0x00000004
    open_existing = 3
    file_attribute_directory = 0x00000010
    file_attribute_reparse_point = 0x00000400
    file_flag_open_reparse_point = 0x00200000

    class ByHandleFileInformation(ctypes.Structure):
        _fields_ = [
            ("dwFileAttributes", wintypes.DWORD),
            ("ftCreationTime", wintypes.FILETIME),
            ("ftLastAccessTime", wintypes.FILETIME),
            ("ftLastWriteTime", wintypes.FILETIME),
            ("dwVolumeSerialNumber", wintypes.DWORD),
            ("nFileSizeHigh", wintypes.DWORD),
            ("nFileSizeLow", wintypes.DWORD),
            ("nNumberOfLinks", wintypes.DWORD),
            ("nFileIndexHigh", wintypes.DWORD),
            ("nFileIndexLow", wintypes.DWORD),
        ]

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    create_file = kernel32.CreateFileW
    create_file.argtypes = (
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.LPVOID,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.HANDLE,
    )
    create_file.restype = wintypes.HANDLE
    get_information = kernel32.GetFileInformationByHandle
    get_information.argtypes = (wintypes.HANDLE, ctypes.POINTER(ByHandleFileInformation))
    get_information.restype = wintypes.BOOL
    close_handle = kernel32.CloseHandle
    close_handle.argtypes = (wintypes.HANDLE,)
    close_handle.restype = wintypes.BOOL

    raw_handle = create_file(
        os.fspath(path),
        generic_read | file_read_attributes,
        file_share_read | file_share_write | file_share_delete,
        None,
        open_existing,
        file_flag_open_reparse_point,
        None,
    )
    if raw_handle == ctypes.c_void_p(-1).value:
        error = ctypes.get_last_error()
        raise StoreValidationError(f"cannot open artifact file: {path}") from OSError(error, os.strerror(error))

    try:
        information = ByHandleFileInformation()
        if not get_information(raw_handle, ctypes.byref(information)):
            error = ctypes.get_last_error()
            raise StoreValidationError(f"cannot inspect artifact file: {path}") from OSError(error, os.strerror(error))
        attributes = information.dwFileAttributes
        if (
            attributes & file_attribute_directory
            or attributes & file_attribute_reparse_point
            or information.nNumberOfLinks != 1
        ):
            raise StoreValidationError(f"artifact file is not an owned regular file: {path}")
        fd = msvcrt.open_osfhandle(raw_handle, os.O_RDONLY)
        raw_handle = None
        with os.fdopen(fd, "rb") as handle:
            yield handle
    finally:
        if raw_handle is not None:
            close_handle(raw_handle)


__all__ = [
    "ArtifactNotFound",
    "ArtifactSecurityError",
    "ArtifactService",
    "ArtifactServiceError",
]
