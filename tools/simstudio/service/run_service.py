"""Simulation-local run lifecycle service.

This module deliberately owns only SimStudio runs.  It does not know about
field Products, systemd, Gateway, DDS allocation, or arbitrary executables.
"""

from __future__ import annotations

import copy
import inspect
import json
import os
import stat
import time
import uuid
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from sim.runtime.coordinator import InteractiveSimulationSession
from sim.runtime.recording import (
    RECORDING_FILENAME,
    TIMELINE_FILENAME,
    SimulationRecordingWriter,
)

from .models import (
    BundleRecord,
    IdempotencyConflict,
    RecordNotFound,
    RevisionConflict,
    RunRecord,
    StoreValidationError,
)
from .store import StudioStore


class RunServiceError(RuntimeError):
    """Base error for simulation-local run operations."""


class RunNotFound(RunServiceError, KeyError):
    """Raised when an opaque run id is unknown."""


class RunStateError(RunServiceError):
    """Raised when a lifecycle operation is not valid for the current state."""


class ActiveRunConflict(RunServiceError):
    """Raised when another run already owns the Studio runtime slot."""


class LaunchProfile(str, Enum):
    """Supported Studio launch profiles."""

    HEADLESS = "headless"
    VISUAL = "visual"


ACTIVE_STATES = frozenset({"PREPARING", "READY", "RUNNING", "PAUSED"})
_RUNTIME_FACETS = ("control", "physics", "sensors", "visual")
_RUNTIME_BINDING_STATES = frozenset({"UNBOUND", "PREPARED", "ACTIVE", "FAILED"})


def _record_dict(record: RunRecord) -> dict[str, Any]:
    return record.to_dict()


def _event_payload(event: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(event, Mapping):
        return {}
    return copy.deepcopy(dict(event))


class _LockBusy(RuntimeError):
    """Internal signal for a non-blocking advisory lock attempt."""


@dataclass
class _ActiveRecording:
    """One writer attached at an ordered InteractiveSession event boundary."""

    writer: SimulationRecordingWriter
    session: Any
    observer_token: int


class _AdvisoryFileLock:
    """Small cross-process lock used for leases held across runtime calls."""

    def __init__(self, path: Path, *, wait: bool, owned_root: Path) -> None:
        self.path = path
        self.wait = wait
        self._owned_root = owned_root
        self._handle: Any | None = None
        self._directory_hold: Any | None = None

    def acquire(self) -> None:
        _assert_owned_coordination_path(self.path, owned_root=self._owned_root)
        directory_hold = _hold_coordination_directories(
            self.path,
            owned_root=self._owned_root,
            create=True,
        )
        directory_hold.__enter__()
        self._directory_hold = directory_hold
        try:
            _assert_owned_coordination_path(self.path, owned_root=self._owned_root)
            _assert_owned_coordination_path(self.path, owned_root=self._owned_root)
            handle = _open_coordination_lock_file(self.path)
        except BaseException:
            self._release_directory_hold()
            raise
        self._handle = handle
        try:
            _assert_owned_coordination_path(self.path, owned_root=self._owned_root)
            _assert_open_coordination_file(handle, self.path)
        except BaseException:
            handle.close()
            self._handle = None
            self._release_directory_hold()
            raise
        deadline = time.monotonic() + 30.0
        while True:
            handle.seek(0)
            try:
                if os.name == "nt":
                    import msvcrt

                    msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except (BlockingIOError, OSError) as exc:
                if not self.wait or time.monotonic() >= deadline:
                    handle.close()
                    self._handle = None
                    self._release_directory_hold()
                    raise _LockBusy(str(self.path)) from exc
                time.sleep(0.01)
        try:
            _assert_owned_coordination_path(self.path, owned_root=self._owned_root)
            _assert_open_coordination_file(handle, self.path)
        except BaseException:
            try:
                handle.seek(0)
                if os.name == "nt":
                    import msvcrt

                    msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            finally:
                handle.close()
                self._handle = None
                self._release_directory_hold()
            raise

    def release(self) -> None:
        handle = self._handle
        self._handle = None
        if handle is None:
            self._release_directory_hold()
            return
        try:
            handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()
            self._release_directory_hold()

    def _release_directory_hold(self) -> None:
        directory_hold = self._directory_hold
        self._directory_hold = None
        if directory_hold is not None:
            directory_hold.__exit__(None, None, None)


def _write_json(path: Path, value: Mapping[str, Any], *, owned_root: Path) -> None:
    """Atomically persist a service-owned coordination record."""

    _assert_owned_coordination_path(path, owned_root=owned_root)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    with _hold_coordination_directories(
        path,
        owned_root=owned_root,
        create=True,
    ) as parent_fd:
        try:
            _assert_owned_coordination_path(temporary, owned_root=owned_root)
            _assert_owned_coordination_path(path, owned_root=owned_root)
            with temporary.open("x", encoding="utf-8", newline="\n") as handle:
                _assert_owned_coordination_path(temporary, owned_root=owned_root)
                _assert_owned_coordination_path(path, owned_root=owned_root)
                _assert_open_coordination_file(handle, temporary)
                json.dump(dict(value), handle, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
                handle.flush()
                os.fsync(handle.fileno())
            _assert_owned_coordination_path(temporary, owned_root=owned_root)
            _assert_owned_coordination_path(path, owned_root=owned_root)
            if parent_fd is None:
                os.replace(temporary, path)
            else:
                os.replace(temporary.name, path.name, src_dir_fd=parent_fd, dst_dir_fd=parent_fd)
            _assert_owned_coordination_path(path, owned_root=owned_root)
        finally:
            if parent_fd is None:
                temporary.unlink(missing_ok=True)
            else:
                try:
                    os.unlink(temporary.name, dir_fd=parent_fd)
                except FileNotFoundError:
                    pass


def _read_json(path: Path, *, owned_root: Path) -> dict[str, Any] | None:
    _assert_owned_coordination_path(path, owned_root=owned_root)
    try:
        with path.open(encoding="utf-8") as handle:
            _assert_owned_coordination_path(path, owned_root=owned_root)
            _assert_open_coordination_file(handle, path)
            value = json.load(handle)
    except FileNotFoundError:
        return None
    if not isinstance(value, dict):
        raise RunServiceError(f"invalid Studio coordination record: {path}")
    return value


def _assert_owned_coordination_path(path: Path, *, owned_root: Path) -> None:
    """Reject reparse points anywhere in a service-owned coordination path."""

    StudioStore._assert_no_reparse_components(path.parent, below=owned_root)
    StudioStore._assert_no_reparse_components(path, below=owned_root)
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise StoreValidationError(f"Studio coordination path is not an owned regular file: {path}")


def _assert_open_coordination_file(handle: Any, path: Path) -> None:
    """Verify an opened coordination file still names one unlinked inode."""

    opened = os.fstat(handle.fileno())
    try:
        named = os.lstat(path)
    except FileNotFoundError as exc:
        raise StoreValidationError(f"Studio coordination path changed while opening: {path}") from exc
    opened_identity = (opened.st_dev, opened.st_ino)
    named_identity = (named.st_dev, named.st_ino)
    if (
        not stat.S_ISREG(opened.st_mode)
        or opened.st_nlink != 1
        or not stat.S_ISREG(named.st_mode)
        or named.st_nlink != 1
        or opened_identity != named_identity
    ):
        raise StoreValidationError(f"Studio coordination file identity is unsafe: {path}")


def _open_coordination_lock_file(path: Path) -> Any:
    if os.name != "nt":
        return path.open("a+b")
    return _open_windows_coordination_lock_file(path)


def _open_windows_coordination_lock_file(path: Path) -> Any:
    """Open or create a Windows lock file without traversing a reparse point."""

    import ctypes
    import msvcrt
    from ctypes import wintypes

    generic_read = 0x80000000
    generic_write = 0x40000000
    file_share_read = 0x00000001
    file_share_write = 0x00000002
    open_always = 4
    file_attribute_directory = 0x00000010
    file_attribute_normal = 0x00000080
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

    handle = create_file(
        os.fspath(path),
        generic_read | generic_write,
        file_share_read | file_share_write,
        None,
        open_always,
        file_attribute_normal | file_flag_open_reparse_point,
        None,
    )
    if handle == ctypes.c_void_p(-1).value:
        error = ctypes.get_last_error()
        raise StoreValidationError(f"cannot open Studio coordination lock: {path}") from OSError(
            error,
            os.strerror(error),
        )

    information = ByHandleFileInformation()
    if not get_information(handle, ctypes.byref(information)):
        error = ctypes.get_last_error()
        _close_windows_handle(handle)
        raise StoreValidationError(f"cannot inspect Studio coordination lock: {path}") from OSError(
            error,
            os.strerror(error),
        )
    attributes = information.dwFileAttributes
    if (
        attributes & file_attribute_directory
        or attributes & file_attribute_reparse_point
        or information.nNumberOfLinks != 1
    ):
        _close_windows_handle(handle)
        raise StoreValidationError(f"Studio coordination lock is not an owned regular file: {path}")

    try:
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDWR | getattr(os, "O_BINARY", 0))
    except BaseException:
        _close_windows_handle(handle)
        raise
    try:
        return os.fdopen(descriptor, "r+b")
    except BaseException:
        os.close(descriptor)
        raise


def _coordination_directories(path: Path, *, owned_root: Path) -> list[Path]:
    root = Path(os.path.abspath(os.fspath(owned_root)))
    parent = Path(os.path.abspath(os.fspath(path.parent)))
    try:
        relative = parent.relative_to(root)
    except ValueError as exc:
        raise StoreValidationError("coordination path is outside its owned root") from exc
    directories = [root]
    current = root
    for part in relative.parts:
        current /= part
        directories.append(current)
    return directories


def _ensure_owned_directory(directory: Path, *, owned_root: Path) -> Path:
    """Create one owned directory while every already-resolved parent is pinned."""

    candidate = Path(os.path.abspath(os.fspath(directory)))
    try:
        candidate.relative_to(owned_root)
    except ValueError as exc:
        raise StoreValidationError("directory path is outside its owned root") from exc
    probe = candidate / ".lingtu-directory-anchor"
    with _hold_coordination_directories(probe, owned_root=owned_root, create=True):
        StudioStore._assert_no_reparse_components(candidate, below=owned_root)
    return candidate


@contextmanager
def _hold_coordination_directories(
    path: Path,
    *,
    owned_root: Path,
    create: bool = False,
) -> Iterator[int | None]:
    """Pin every owned directory used to resolve an atomic coordination write."""

    directories = _coordination_directories(path, owned_root=owned_root)
    if os.name == "nt":
        handles: list[int] = []
        try:
            for index, directory in enumerate(directories):
                if create and index:
                    try:
                        directory.mkdir()
                    except FileExistsError:
                        pass
                StudioStore._assert_no_reparse_components(directory, below=owned_root)
                handles.append(_open_windows_coordination_directory(directory))
                StudioStore._assert_no_reparse_components(directory, below=owned_root)
            yield None
        finally:
            for handle in reversed(handles):
                _close_windows_handle(handle)
        return

    descriptors: list[int] = []
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        for index, directory in enumerate(directories):
            if create and index:
                try:
                    directory.mkdir()
                except FileExistsError:
                    pass
            StudioStore._assert_no_reparse_components(directory, below=owned_root)
            try:
                descriptor = os.open(directory, flags)
            except OSError as exc:
                raise StoreValidationError(f"cannot hold Studio coordination directory: {directory}") from exc
            descriptors.append(descriptor)
            _assert_open_coordination_directory(descriptor, directory)
        yield descriptors[-1]
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _assert_open_coordination_directory(descriptor: int, path: Path) -> None:
    opened = os.fstat(descriptor)
    try:
        named = os.lstat(path)
    except FileNotFoundError as exc:
        raise StoreValidationError(f"Studio coordination directory changed while opening: {path}") from exc
    if (
        not stat.S_ISDIR(opened.st_mode)
        or not stat.S_ISDIR(named.st_mode)
        or (opened.st_dev, opened.st_ino) != (named.st_dev, named.st_ino)
    ):
        raise StoreValidationError(f"Studio coordination directory identity is unsafe: {path}")


def _open_windows_coordination_directory(path: Path) -> int:
    import ctypes
    from ctypes import wintypes

    file_list_directory = 0x0001
    file_read_attributes = 0x0080
    file_share_read = 0x00000001
    file_share_write = 0x00000002
    open_existing = 3
    file_attribute_directory = 0x00000010
    file_attribute_reparse_point = 0x00000400
    file_flag_backup_semantics = 0x02000000
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

    handle = create_file(
        os.fspath(path),
        file_list_directory | file_read_attributes,
        file_share_read | file_share_write,
        None,
        open_existing,
        file_flag_backup_semantics | file_flag_open_reparse_point,
        None,
    )
    if handle == ctypes.c_void_p(-1).value:
        error = ctypes.get_last_error()
        raise StoreValidationError(f"cannot hold Studio coordination directory: {path}") from OSError(error, os.strerror(error))

    information = ByHandleFileInformation()
    if not get_information(handle, ctypes.byref(information)):
        error = ctypes.get_last_error()
        _close_windows_handle(handle)
        raise StoreValidationError(f"cannot inspect Studio coordination directory: {path}") from OSError(
            error, os.strerror(error)
        )
    attributes = information.dwFileAttributes
    if not attributes & file_attribute_directory or attributes & file_attribute_reparse_point:
        _close_windows_handle(handle)
        raise StoreValidationError(f"Studio coordination parent is not a plain directory: {path}")
    return handle


def _close_windows_handle(handle: int) -> None:
    import ctypes
    from ctypes import wintypes

    close_handle = ctypes.WinDLL("kernel32", use_last_error=True).CloseHandle
    close_handle.argtypes = (wintypes.HANDLE,)
    close_handle.restype = wintypes.BOOL
    close_handle(handle)


class _RuntimeSlotLease:
    """A durable lease record protected by a cross-process advisory lock."""

    def __init__(self, store: StudioStore, run_id: str) -> None:
        self.run_id = run_id
        self._owned_root = store.root
        self._record_path = store.root / ".run-slot.json"
        self._lock = _AdvisoryFileLock(store.root / ".run-slot.lock", wait=False, owned_root=store.root)
        self._token = uuid.uuid4().hex

    def acquire(self) -> bool:
        try:
            self._lock.acquire()
        except _LockBusy:
            return False
        try:
            _write_json(
                self._record_path,
                {"run_id": self.run_id, "state": "held", "token": self._token},
                owned_root=self._owned_root,
            )
        except BaseException:
            self._lock.release()
            raise
        return True

    def release(self) -> None:
        try:
            current = _read_json(self._record_path, owned_root=self._owned_root)
            if current is not None and current.get("token") == self._token:
                _write_json(
                    self._record_path,
                    {"run_id": self.run_id, "state": "released", "token": self._token},
                    owned_root=self._owned_root,
                )
        finally:
            self._lock.release()


class _LifecycleReservation:
    """Per-key durable reservation held while a lifecycle method runs."""

    def __init__(self, store: StudioStore, key: str) -> None:
        coordination_root = store.root / ".run-idempotency"
        self._owned_root = store.root
        digest = uuid.uuid5(uuid.NAMESPACE_URL, key).hex
        self._record_path = coordination_root / f"{digest}.json"
        self._lock = _AdvisoryFileLock(
            coordination_root / f"{digest}.lock",
            wait=True,
            owned_root=store.root,
        )
        self._token = uuid.uuid4().hex
        self._lock.acquire()

    @property
    def record_path(self) -> Path:
        return self._record_path

    def read(self) -> dict[str, Any] | None:
        return _read_json(self._record_path, owned_root=self._owned_root)

    def write(self, value: Mapping[str, Any]) -> None:
        _write_json(self._record_path, value, owned_root=self._owned_root)

    def owns(self, value: Mapping[str, Any] | None) -> bool:
        return value is not None and value.get("token") == self._token

    def bind(self, record: RunRecord, *, operation: str, target: str) -> None:
        current = self.read() or {}
        current.update(
            {
                "state": "pending",
                "operation": operation,
                "run_id": record.id,
                "target": target,
                "initial_revision": record.revision,
                "initial_status": record.status,
                "resolved_expected_revision": record.revision,
            }
        )
        self.write(current)

    def set_resolved_expected_revision(self, revision: int) -> None:
        current = self.read() or {}
        current["resolved_expected_revision"] = revision
        self.write(current)

    def complete(self, response: Mapping[str, Any]) -> None:
        current = self.read() or {}
        current.update({"state": "complete", "response": copy.deepcopy(dict(response))})
        self.write(current)
        self._lock.release()

    def abort(self) -> None:
        try:
            current = self.read()
            if self.owns(current):
                _assert_owned_coordination_path(self._record_path, owned_root=self._owned_root)
                self._record_path.unlink(missing_ok=True)
        finally:
            self._lock.release()


class RunService:
    """Persist and drive one opaque-id-addressed simulation run.

    ``session_factory`` is the only runtime seam.  Production code injects a
    factory that creates ``InteractiveSimulationSession``; tests inject a fake
    session and never start UE or MuJoCo.
    """

    def __init__(
        self,
        store: StudioStore,
        session_factory: Callable[..., InteractiveSimulationSession],
        *,
        artifact_root: Path | None = None,
    ) -> None:
        self.store = store
        for sidecar_name in (".run-slot.lock", ".run-slot.json"):
            StudioStore._assert_no_reparse_components(store.root / sidecar_name, below=store.root)
        self.session_factory = session_factory
        self._sessions: dict[str, InteractiveSimulationSession] = {}
        self._recordings: dict[str, _ActiveRecording] = {}
        self._lease: _RuntimeSlotLease | None = None
        self._artifact_root = self._owned_artifact_root(artifact_root)

    @property
    def artifact_root(self) -> Path:
        """Return the service-owned root used for run artifacts."""

        return self._artifact_root

    def create_run(
        self,
        bundle_id: str,
        launch_profile: str | LaunchProfile,
        *,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Create a dormant run from one stored BundleRecord."""

        profile = self._profile(launch_profile)
        bundle = self._bundle(bundle_id)
        if bundle.status not in {"ready", "READY", "available", "AVAILABLE"}:
            raise RunStateError(f"bundle {bundle_id} is not ready: {bundle.status}")
        payload = {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": bundle.id,
            "bundle_revision": bundle.revision,
            "launch_profile": profile.value,
            "artifact_path": "artifacts/runs/pending",
            "readiness": {},
            "sensor_summary": {},
            "runtime_event": {},
            "failure": None,
            "recording": {"state": "IDLE"},
        }
        record = self.store.create_run(
            payload,
            status="CREATED",
            idempotency_key=idempotency_key,
        )
        if record.payload.get("artifact_path") == "artifacts/runs/pending":
            payload["artifact_path"] = self._artifact_relative(record.id)
            record = self.store.update_run(
                record.id,
                expected_revision=record.revision,
                payload=payload,
                status=record.status,
            )
        self._ensure_artifact_directory(record.id)
        return _record_dict(record)

    def get_run(self, run_id: str) -> dict[str, Any]:
        """Return one run record by opaque id."""

        return _record_dict(self._run(run_id))

    def list_runs(self) -> list[dict[str, Any]]:
        """Return all historical and active runs in store order."""

        return [_record_dict(record) for record in self.store.list_runs()]

    def prepare(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Construct and prepare the injected interactive session."""

        reservation, replay = self._begin_lifecycle(
            run_id,
            operation="prepare",
            target="READY",
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
        )
        if replay is not None:
            return replay
        claimed = False
        try:
            record = self._run(run_id)
            self._check_revision(record, expected_revision)
            self._require_status(record, {"CREATED"}, "prepare")
            if reservation is not None:
                reservation.bind(record, operation="prepare", target="READY")
            self._claim(run_id)
            claimed = True
            preparing = self._update(
                record,
                status="PREPARING",
                event={},
                expected_revision=expected_revision,
                idempotency_key=None,
            )
            session = self._make_session(preparing)
            self._sessions[run_id] = session
            event = session.prepare()
            if reservation is not None:
                reservation.set_resolved_expected_revision(preparing.revision)
            ready = self._update(
                preparing,
                status="READY",
                event=event,
                expected_revision=preparing.revision,
                idempotency_key=idempotency_key,
            )
            result = _record_dict(ready)
            self._complete_lifecycle(reservation, result)
            return result
        except BaseException as exc:
            self._abort_lifecycle(reservation)
            if claimed:
                cleanup_error = self._cleanup_session(run_id)
                self._fail(run_id, exc, cleanup_error)
                self._release(run_id)
            raise

    def start(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Start a prepared or paused run."""

        return self._drive(
            run_id,
            operation="start",
            allowed={"READY", "PAUSED"},
            target="RUNNING",
            method="start",
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
        )

    def pause(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Pause a running run."""

        return self._drive(
            run_id,
            operation="pause",
            allowed={"RUNNING"},
            target="PAUSED",
            method="pause",
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
        )

    def reset(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Reset a prepared, running, or paused run."""

        reservation, replay = self._begin_lifecycle(
            run_id,
            operation="reset",
            target=None,
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
        )
        if replay is not None:
            return replay
        try:
            record = self._run(run_id)
            self._check_revision(record, expected_revision)
            self._require_status(record, {"READY", "RUNNING", "PAUSED"}, "reset")
            if reservation is not None:
                reservation.bind(record, operation="reset", target="RUNNING" if record.status == "RUNNING" else "READY")
            session = self._session(run_id)
        except BaseException:
            self._abort_lifecycle(reservation)
            raise
        try:
            event = session.reset()
            target = "RUNNING" if record.status == "RUNNING" else "READY"
            updated = self._update(
                record,
                status=target,
                event=event,
                expected_revision=expected_revision,
                idempotency_key=idempotency_key,
            )
            result = _record_dict(updated)
            self._complete_lifecycle(reservation, result)
            return result
        except BaseException as exc:
            self._abort_lifecycle(reservation)
            cleanup_error = self._cleanup_session(run_id)
            self._fail(run_id, exc, cleanup_error)
            self._release(run_id)
            raise

    def start_recording(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Attach a bounded truth recorder to a running or paused session."""

        replay = self._recording_idempotency_response(
            run_id,
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
            expected_state="CAPTURING",
        )
        if replay is not None:
            return replay
        record = self._run(run_id)
        self._check_revision(record, expected_revision)
        self._require_status(record, {"RUNNING", "PAUSED"}, "start_recording")
        if not self._owns_active_run(run_id):
            raise ActiveRunConflict(
                f"run {run_id} is active but is not owned by this service instance"
            )
        if run_id in self._recordings:
            raise RunStateError(f"run {run_id} is already recording")
        session = self._session(run_id)
        attach = getattr(session, "attach_event_observer", None)
        detach = getattr(session, "detach_event_observer", None)
        if not callable(attach) or not callable(detach):
            raise RunStateError(
                "the attached runtime does not support ordered recording observers"
            )
        root = self._ensure_artifact_directory(run_id)
        if (root / RECORDING_FILENAME).exists() or (root / TIMELINE_FILENAME).exists():
            raise RunStateError(f"run {run_id} already has recording artifacts")
        bundle_id = record.payload.get("bundle_id")
        if not isinstance(bundle_id, str):
            raise RunServiceError("run payload has no bundle_id")
        session_id = self._bundle(bundle_id).payload.get("session_id")
        if not isinstance(session_id, str):
            raise RunServiceError("run bundle has no session_id")
        writer = SimulationRecordingWriter(
            root,
            run_id=run_id,
            session_id=session_id,
        )

        def observe(event: Mapping[str, Any]) -> None:
            if event.get("event") == "snapshot":
                capture_payloads = getattr(session, "capture_sensor_payloads", None)
                sensor_payloads = (
                    capture_payloads(event) if callable(capture_payloads) else ()
                )
                writer.append(event, sensor_payloads=sensor_payloads)

        token: int | None = None
        try:
            token = attach(observe, replay_latest_snapshot=True)
            if isinstance(token, bool) or not isinstance(token, int) or token < 1:
                raise RunServiceError("runtime returned an invalid recording observer token")
            self._recordings[run_id] = _ActiveRecording(writer, session, token)
            payload = copy.deepcopy(dict(record.payload))
            payload["recording"] = {"state": "CAPTURING"}
            revision = record.revision if expected_revision is None else expected_revision
            updated = self.store.update_run(
                run_id,
                expected_revision=revision,
                payload=payload,
                status=record.status,
                idempotency_key=idempotency_key,
            )
            return _record_dict(updated)
        except BaseException:
            self._recordings.pop(run_id, None)
            if token is not None:
                detach(token)
            writer.abort()
            raise

    def stop_recording(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Detach and atomically commit one active truth recording."""

        replay = self._recording_idempotency_response(
            run_id,
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
            expected_state="COMMITTED",
        )
        if replay is not None:
            return replay
        record = self._run(run_id)
        self._check_revision(record, expected_revision)
        self._require_status(record, {"RUNNING", "PAUSED"}, "stop_recording")
        if not self._owns_active_run(run_id):
            raise ActiveRunConflict(
                f"run {run_id} is active but is not owned by this service instance"
            )
        if run_id not in self._recordings:
            raise RunStateError(f"run {run_id} is not recording")
        result = self._finalize_recording(run_id)
        payload = copy.deepcopy(dict(record.payload))
        payload["recording"] = result
        revision = record.revision if expected_revision is None else expected_revision
        updated = self.store.update_run(
            run_id,
            expected_revision=revision,
            payload=payload,
            status=record.status,
            idempotency_key=idempotency_key,
        )
        if result["state"] == "FAILED":
            raise RunServiceError(str(result["message"]))
        return _record_dict(updated)

    def stop(
        self,
        run_id: str,
        *,
        expected_revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Stop a run and release its Studio runtime slot."""

        reservation, replay = self._begin_lifecycle(
            run_id,
            operation="stop",
            target="STOPPED",
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
        )
        if replay is not None:
            return replay
        try:
            record = self._run(run_id)
            self._check_revision(record, expected_revision)
            self._require_status(record, ACTIVE_STATES | {"CREATED", "FAILED"}, "stop")
            if record.status in ACTIVE_STATES and not self._owns_active_run(run_id):
                raise ActiveRunConflict(f"run {run_id} is active but is not owned by this service instance")
            if reservation is not None:
                reservation.bind(record, operation="stop", target="STOPPED")
            session = self._sessions.get(run_id)
        except BaseException:
            self._abort_lifecycle(reservation)
            raise
        try:
            recording = self._finalize_recording(run_id, required=False)
            event = {} if session is None else dict(session.stop())
            if recording is not None:
                event["recording"] = recording
            updated = self._update(
                record,
                status="STOPPED",
                event=event,
                expected_revision=expected_revision,
                idempotency_key=idempotency_key,
            )
            result = _record_dict(updated)
            self._complete_lifecycle(reservation, result)
        except BaseException as exc:
            self._abort_lifecycle(reservation)
            cleanup_error = self._cleanup_session(run_id)
            self._fail(run_id, exc, cleanup_error)
            raise
        finally:
            self._sessions.pop(run_id, None)
            self._release(run_id)
        return result

    def _drive(
        self,
        run_id: str,
        *,
        operation: str,
        allowed: set[str],
        target: str,
        method: str,
        expected_revision: int | None,
        idempotency_key: str | None,
    ) -> dict[str, Any]:
        reservation, replay = self._begin_lifecycle(
            run_id,
            operation=operation,
            target=target,
            expected_revision=expected_revision,
            idempotency_key=idempotency_key,
        )
        if replay is not None:
            return replay
        try:
            record = self._run(run_id)
            self._check_revision(record, expected_revision)
            self._require_status(record, allowed, operation)
            if reservation is not None:
                reservation.bind(record, operation=operation, target=target)
            session = self._session(run_id)
        except BaseException:
            self._abort_lifecycle(reservation)
            raise
        try:
            event = getattr(session, method)()
            updated = self._update(
                record,
                status=target,
                event=event,
                expected_revision=expected_revision,
                idempotency_key=idempotency_key,
            )
            result = _record_dict(updated)
            self._complete_lifecycle(reservation, result)
            return result
        except BaseException as exc:
            self._abort_lifecycle(reservation)
            cleanup_error = self._cleanup_session(run_id)
            self._fail(run_id, exc, cleanup_error)
            self._release(run_id)
            raise

    def _update(
        self,
        record: RunRecord,
        *,
        status: str,
        event: Mapping[str, Any] | None,
        expected_revision: int | None,
        idempotency_key: str | None,
    ) -> RunRecord:
        payload = copy.deepcopy(dict(record.payload))
        accepted = _event_payload(event)
        accepted.update(self._runtime_readiness_projection(record))
        payload["runtime_event"] = accepted
        for source, target in (("readiness", "readiness"), ("sensor_summary", "sensor_summary"), ("sensors", "sensor_summary")):
            if isinstance(accepted.get(source), Mapping):
                payload[target] = copy.deepcopy(dict(accepted[source]))
        if isinstance(accepted.get("recording"), Mapping):
            payload["recording"] = copy.deepcopy(dict(accepted["recording"]))
        if "failure" in accepted:
            payload["failure"] = copy.deepcopy(accepted["failure"])
        revision = record.revision if expected_revision is None else expected_revision
        return self.store.update_run(
            record.id,
            expected_revision=revision,
            payload=payload,
            status=status,
            idempotency_key=idempotency_key,
        )

    def _runtime_readiness_projection(self, record: RunRecord) -> dict[str, Any]:
        """Project the authoritative Runtime manifest into the durable Run view."""

        manifest_path = self._artifact_root / record.id / "session.runtime.json"
        manifest = _read_json(manifest_path, owned_root=self.store.root)
        if manifest is None:
            return {}
        if manifest.get("schema") != "lingtu.sim.session-runtime.v1":
            raise RunServiceError("runtime manifest has an unsupported schema")
        if manifest.get("run_id") != record.id:
            raise RunServiceError("runtime manifest run_id does not match the Studio run")
        bundle_id = record.payload.get("bundle_id")
        if not isinstance(bundle_id, str):
            raise RunServiceError("run payload has no bundle_id")
        expected_digest = self._bundle(bundle_id).payload.get("session_id")
        if manifest.get("session_id") != expected_digest:
            raise RunServiceError(
                "runtime manifest session_id does not match the Studio Bundle"
            )

        bindings = manifest.get("bindings")
        if not isinstance(bindings, Mapping):
            raise RunServiceError("runtime manifest bindings must be an object")
        readiness: dict[str, str] = {}
        for facet in _RUNTIME_FACETS:
            binding = bindings.get(facet)
            if not isinstance(binding, Mapping):
                raise RunServiceError(f"runtime manifest has no {facet} binding")
            state = binding.get("state")
            if state not in _RUNTIME_BINDING_STATES:
                raise RunServiceError(
                    f"runtime manifest {facet} binding has an invalid state"
                )
            readiness[facet] = str(state)

        sensor_summary: dict[str, str] = {}
        sensor_runtime = manifest.get("sensor_streams")
        if sensor_runtime is not None:
            if not isinstance(sensor_runtime, Mapping) or not isinstance(
                sensor_runtime.get("streams"), Mapping
            ):
                raise RunServiceError("runtime manifest sensor_streams is invalid")
            for sensor_id, stream in sorted(sensor_runtime["streams"].items()):
                if not isinstance(sensor_id, str) or not isinstance(stream, Mapping):
                    raise RunServiceError("runtime manifest sensor stream is invalid")
                if stream.get("stream_id") != sensor_id:
                    raise RunServiceError(
                        "runtime manifest sensor stream identity does not match its key"
                    )
                state = stream.get("state")
                if state not in _RUNTIME_BINDING_STATES:
                    raise RunServiceError(
                        f"runtime manifest sensor {sensor_id} has an invalid state"
                    )
                sensor_summary[sensor_id] = str(state)
        return {"readiness": readiness, "sensor_summary": sensor_summary}

    def _begin_lifecycle(
        self,
        run_id: str,
        *,
        operation: str,
        target: str | None,
        expected_revision: int | None,
        idempotency_key: str | None,
    ) -> tuple[_LifecycleReservation | None, dict[str, Any] | None]:
        if idempotency_key is None:
            return None, None
        reservation = _LifecycleReservation(self.store, idempotency_key)
        try:
            existing = self.store.get_idempotency("update:run", idempotency_key)
            marker = reservation.read()
            if marker is not None:
                self._validate_lifecycle_marker(
                    marker,
                    run_id=run_id,
                    operation=operation,
                    expected_revision=expected_revision,
                )
                if marker.get("state") == "complete":
                    response = marker.get("response")
                    if not isinstance(response, Mapping):
                        raise RunServiceError("lifecycle idempotency record has no response")
                    reservation._lock.release()
                    return None, copy.deepcopy(dict(response))

            if existing is not None:
                if marker is None:
                    raise IdempotencyConflict("lifecycle idempotency key has no operation marker")
                response = copy.deepcopy(dict(existing.response))
                self._validate_idempotency_response(
                    response,
                    run_id=run_id,
                    target=str(marker.get("target") or target) if marker.get("target") is not None or target is not None else None,
                    expected_revision=marker.get("resolved_expected_revision")
                    if isinstance(marker.get("resolved_expected_revision"), int)
                    else expected_revision,
                )
                reservation.write(
                    {
                        "state": "complete",
                        "token": reservation._token,
                        "operation": operation,
                        "run_id": run_id,
                        "expected_revision": expected_revision,
                        "response": response,
                    }
                )
                reservation._lock.release()
                return None, response

            if marker is not None and marker.get("state") == "pending":
                initial_revision = marker.get("initial_revision")
                initial_status = marker.get("initial_status")
                if isinstance(initial_revision, int) and isinstance(initial_status, str):
                    current = self._run(run_id)
                    if current.revision != initial_revision or current.status != initial_status:
                        raise IdempotencyConflict("lifecycle idempotency key has an incomplete request")
                marker = None

            reservation.write(
                {
                    "state": "pending",
                    "token": reservation._token,
                    "operation": operation,
                    "run_id": run_id,
                    "expected_revision": expected_revision,
                }
            )
            return reservation, None
        except BaseException:
            reservation.abort()
            raise

    @staticmethod
    def _validate_lifecycle_marker(
        marker: Mapping[str, Any],
        *,
        run_id: str,
        operation: str,
        expected_revision: int | None,
    ) -> None:
        if marker.get("run_id") != run_id or marker.get("operation") != operation:
            raise IdempotencyConflict("idempotency key already belongs to a different lifecycle request")
        if marker.get("expected_revision") != expected_revision:
            raise IdempotencyConflict("idempotency key already belongs to a different lifecycle request")

    @staticmethod
    def _validate_idempotency_response(
        response: Mapping[str, Any],
        *,
        run_id: str,
        target: str | None,
        expected_revision: int | None,
    ) -> None:
        if response.get("id") != run_id:
            raise IdempotencyConflict("idempotency key already belongs to a different lifecycle request")
        if target is not None and response.get("status") != target:
            raise IdempotencyConflict("idempotency key already belongs to a different lifecycle request")
        if expected_revision is not None and response.get("revision") != expected_revision + 1:
            raise IdempotencyConflict("idempotency key already belongs to a different lifecycle request")

    @staticmethod
    def _complete_lifecycle(reservation: _LifecycleReservation | None, response: Mapping[str, Any]) -> None:
        if reservation is not None:
            reservation.complete(response)

    @staticmethod
    def _abort_lifecycle(reservation: _LifecycleReservation | None) -> None:
        if reservation is not None:
            reservation.abort()

    def _cleanup_session(self, run_id: str) -> str | None:
        self._abort_recording(run_id)
        session = self._sessions.pop(run_id, None)
        if session is None:
            return None
        try:
            session.stop()
        except BaseException as exc:
            return f"cleanup {type(exc).__name__}: {str(exc) or type(exc).__name__}"
        return None

    def _fail(self, run_id: str, error: BaseException, cleanup_error: str | None = None) -> None:
        try:
            record = self._run(run_id)
            payload = copy.deepcopy(dict(record.payload))
            message = str(error) or type(error).__name__
            if cleanup_error is not None:
                message = f"{message}; {cleanup_error}"
            payload["failure"] = {"type": type(error).__name__, "message": message}
            payload["runtime_event"] = {"failure": payload["failure"]}
            recording = payload.get("recording")
            if isinstance(recording, Mapping) and recording.get("state") == "CAPTURING":
                payload["recording"] = {
                    "state": "FAILED",
                    "message": "recording aborted because the runtime failed",
                }
            self.store.update_run(
                run_id,
                expected_revision=record.revision,
                payload=payload,
                status="FAILED",
            )
        except Exception:
            # The original lifecycle error is more useful to the caller; the
            # failed record is best-effort because the store may itself be down.
            return

    def _make_session(self, record: RunRecord) -> InteractiveSimulationSession:
        bundle = self._bundle(str(record.payload["bundle_id"]))
        artifact_root = self._ensure_artifact_directory(record.id)
        factory = self.session_factory
        try:
            signature = inspect.signature(factory)
        except (TypeError, ValueError):
            return factory(bundle, record.payload["launch_profile"], artifact_root)
        values = {
            "bundle": bundle,
            "bundle_record": bundle,
            "bundle_id": bundle.id,
            "launch_profile": record.payload["launch_profile"],
            "profile": record.payload["launch_profile"],
            "artifact_root": artifact_root,
            "artifact_path": artifact_root,
            "run_id": record.id,
        }
        kwargs = {
            name: values[name]
            for name, parameter in signature.parameters.items()
            if name in values and parameter.kind in {parameter.POSITIONAL_OR_KEYWORD, parameter.KEYWORD_ONLY}
        }
        if any(parameter.kind is parameter.VAR_KEYWORD for parameter in signature.parameters.values()):
            kwargs = values
        return factory(**kwargs)

    def _finalize_recording(
        self,
        run_id: str,
        *,
        required: bool = True,
    ) -> dict[str, Any] | None:
        active = self._recordings.pop(run_id, None)
        if active is None:
            if required:
                raise RunStateError(f"run {run_id} is not recording")
            return None
        detach = getattr(active.session, "detach_event_observer", None)
        try:
            if not callable(detach) or not detach(active.observer_token):
                raise RunServiceError("recording observer could not be detached")
            active.writer.close()
        except BaseException as exc:
            active.writer.abort()
            return {
                "state": "FAILED",
                "message": str(exc) or type(exc).__name__,
            }
        return {
            "state": "COMMITTED",
            "manifest": RECORDING_FILENAME,
            "timeline": TIMELINE_FILENAME,
        }

    def _abort_recording(self, run_id: str) -> None:
        active = self._recordings.pop(run_id, None)
        if active is None:
            return
        detach = getattr(active.session, "detach_event_observer", None)
        try:
            if callable(detach):
                detach(active.observer_token)
        finally:
            active.writer.abort()

    def _recording_idempotency_response(
        self,
        run_id: str,
        *,
        expected_revision: int | None,
        idempotency_key: str | None,
        expected_state: str,
    ) -> dict[str, Any] | None:
        if idempotency_key is None:
            return None
        existing = self.store.get_idempotency("update:run", idempotency_key)
        if existing is None:
            return None
        response = copy.deepcopy(dict(existing.response))
        payload = response.get("payload")
        recording = payload.get("recording") if isinstance(payload, Mapping) else None
        valid_revision = expected_revision is None or response.get("revision") == expected_revision + 1
        if (
            response.get("id") != run_id
            or not valid_revision
            or not isinstance(recording, Mapping)
            or recording.get("state") != expected_state
        ):
            raise IdempotencyConflict(
                "idempotency key already belongs to a different recording request"
            )
        return response

    def _session(self, run_id: str) -> InteractiveSimulationSession:
        try:
            return self._sessions[run_id]
        except KeyError as exc:
            raise RunStateError(f"run {run_id} has no attached interactive session") from exc

    def _bundle(self, bundle_id: str) -> BundleRecord:
        try:
            return self.store.get_bundle(bundle_id)
        except (KeyError, RecordNotFound) as exc:
            raise RunNotFound(f"bundle {bundle_id} was not found") from exc

    def _run(self, run_id: str) -> RunRecord:
        try:
            return self.store.get_run(run_id)
        except (KeyError, RecordNotFound) as exc:
            raise RunNotFound(f"run {run_id} was not found") from exc

    @staticmethod
    def _profile(value: str | LaunchProfile) -> LaunchProfile:
        try:
            return value if isinstance(value, LaunchProfile) else LaunchProfile(value)
        except (TypeError, ValueError) as exc:
            raise StoreValidationError("launch_profile must be 'headless' or 'visual'") from exc

    @staticmethod
    def _check_revision(record: RunRecord, expected: int | None) -> None:
        if expected is not None and expected != record.revision:
            raise RevisionConflict(
                f"stale run revision for {record.id}: expected {expected}, current {record.revision}",
                expected=expected,
                actual=record.revision,
            )

    @staticmethod
    def _require_status(record: RunRecord, allowed: set[str] | frozenset[str], operation: str) -> None:
        if record.status not in allowed:
            raise RunStateError(f"{operation} is invalid in state {record.status}")

    def _owned_artifact_root(self, root: Path | None) -> Path:
        candidate = self.store.root / "artifacts" / "runs" if root is None else Path(root)
        candidate = Path(os.path.abspath(os.fspath(candidate)))
        try:
            candidate.relative_to(self.store.root)
        except ValueError as exc:
            raise StoreValidationError("artifact root must be beneath the Studio store root") from exc
        return _ensure_owned_directory(candidate, owned_root=self.store.root)

    def _artifact_relative(self, bundle_id: str) -> str:
        directory = self._artifact_root / bundle_id
        try:
            relative = directory.relative_to(self.store.root).as_posix()
        except ValueError as exc:
            raise StoreValidationError("run artifact path must be beneath the Studio store root") from exc
        StudioStore.validate_relative_path(relative, context="run.artifact_path")
        return relative

    def _ensure_artifact_directory(self, run_id: str) -> Path:
        directory = self._artifact_root / run_id
        return _ensure_owned_directory(directory, owned_root=self.store.root)

    def _claim(self, run_id: str) -> None:
        lease = _RuntimeSlotLease(self.store, run_id)
        if not lease.acquire():
            current = _read_json(self.store.root / ".run-slot.json", owned_root=self.store.root) or {}
            owner = current.get("run_id", "another run")
            raise ActiveRunConflict(f"run {owner} already owns the Studio runtime slot")
        try:
            for record in self.store.list_runs():
                if record.id == run_id or record.status not in ACTIVE_STATES:
                    continue
                payload = copy.deepcopy(dict(record.payload))
                payload["failure"] = {
                    "type": "StaleRuntimeLease",
                    "message": "recovered a run whose runtime lease was no longer held",
                }
                payload["runtime_event"] = {"failure": payload["failure"]}
                self.store.update_run(
                    record.id,
                    expected_revision=record.revision,
                    payload=payload,
                    status="FAILED",
                )
            self._lease = lease
        except BaseException:
            lease.release()
            raise

    def _release(self, run_id: str) -> None:
        lease = self._lease
        if lease is not None and lease.run_id == run_id:
            self._lease = None
            lease.release()

    def _owns_active_run(self, run_id: str) -> bool:
        lease = self._lease
        return lease is not None and lease.run_id == run_id and run_id in self._sessions


__all__ = [
    "ACTIVE_STATES",
    "ActiveRunConflict",
    "LaunchProfile",
    "RunNotFound",
    "RunService",
    "RunServiceError",
    "RunStateError",
]
