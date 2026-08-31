"""Operating-system identity and recovery records for simulation children."""

from __future__ import annotations

import json
import os
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from lingtu.switch_contracts import is_product_session_id

_FIELDS = frozenset({"pid", "platform", "start_identity"})
_WINDOWS_START = re.compile(r"[1-9][0-9]{0,31}\Z")
_POSIX_START = re.compile(
    r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}:"
    r"[1-9][0-9]{0,31}\Z"
)
_CHILD_LEDGER_SCHEMA = "lingtu.sim_children.v3"
_TARGET = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.@:-]*\Z")


class ProcessIdentityError(RuntimeError):
    """Raised when an exact process start identity cannot be established."""


@dataclass(frozen=True)
class ProcessIdentity:
    """A PID plus OS start token that does not alias a reused PID."""

    pid: int
    platform: str
    start_identity: str

    def __post_init__(self) -> None:
        if isinstance(self.pid, bool) or not isinstance(self.pid, int) or self.pid <= 0:
            raise ProcessIdentityError("process pid must be a positive integer")
        pattern = {
            "windows": _WINDOWS_START,
            "posix-procfs": _POSIX_START,
        }.get(self.platform)
        if pattern is None:
            raise ProcessIdentityError("process identity platform is unsupported")
        if (
            not isinstance(self.start_identity, str)
            or pattern.fullmatch(self.start_identity) is None
        ):
            raise ProcessIdentityError("process start identity is invalid")

    @classmethod
    def current(cls, pid: int | None = None) -> ProcessIdentity:
        """Read the exact identity for *pid*, or for this process when omitted."""

        exact_pid = os.getpid() if pid is None else pid
        if (
            isinstance(exact_pid, bool)
            or not isinstance(exact_pid, int)
            or exact_pid <= 0
        ):
            raise ProcessIdentityError("process pid must be a positive integer")
        platform, start_identity = _read_process_start_identity(exact_pid)
        return cls(exact_pid, platform, start_identity)

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> ProcessIdentity:
        """Decode an exact, non-coercing process identity object."""

        if type(payload) is not dict or frozenset(payload) != _FIELDS:
            raise ProcessIdentityError("process identity fields are invalid")
        return cls(
            pid=payload["pid"],
            platform=payload["platform"],
            start_identity=payload["start_identity"],
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "pid": self.pid,
            "platform": self.platform,
            "start_identity": self.start_identity,
        }

    def matches(self) -> bool:
        """Return whether this identity still names the same live process start."""

        try:
            return ProcessIdentity.current(self.pid) == self
        except ProcessIdentityError:
            return False


def _read_process_start_identity(pid: int) -> tuple[str, str]:
    if os.name == "nt":
        return "windows", _windows_start_identity(pid)
    if os.name == "posix":
        return "posix-procfs", _posix_start_identity(pid)
    raise ProcessIdentityError("process identity platform is unsupported")


def _windows_start_identity(pid: int) -> str:
    import ctypes
    from ctypes import wintypes

    query_limited_information = 0x1000
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.GetProcessTimes.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(wintypes.FILETIME),
        ctypes.POINTER(wintypes.FILETIME),
        ctypes.POINTER(wintypes.FILETIME),
        ctypes.POINTER(wintypes.FILETIME),
    ]
    kernel32.GetProcessTimes.restype = wintypes.BOOL
    kernel32.GetExitCodeProcess.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(wintypes.DWORD),
    ]
    kernel32.GetExitCodeProcess.restype = wintypes.BOOL
    kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel32.CloseHandle.restype = wintypes.BOOL
    handle = kernel32.OpenProcess(query_limited_information, False, pid)
    if not handle:
        raise ProcessIdentityError("cannot read process start identity")
    creation = wintypes.FILETIME()
    exit_time = wintypes.FILETIME()
    kernel = wintypes.FILETIME()
    user = wintypes.FILETIME()
    exit_code = wintypes.DWORD()
    try:
        if not kernel32.GetProcessTimes(
            handle,
            ctypes.byref(creation),
            ctypes.byref(exit_time),
            ctypes.byref(kernel),
            ctypes.byref(user),
        ):
            raise ProcessIdentityError("cannot read process start identity")
        if not kernel32.GetExitCodeProcess(handle, ctypes.byref(exit_code)):
            raise ProcessIdentityError("cannot read process start identity")
        if exit_code.value != 259:  # STILL_ACTIVE
            raise ProcessIdentityError("process is not live")
        ticks = (creation.dwHighDateTime << 32) | creation.dwLowDateTime
        if ticks <= 0:
            raise ProcessIdentityError("cannot read process start identity")
        return str(ticks)
    finally:
        kernel32.CloseHandle(handle)


def _posix_start_identity(pid: int) -> str:
    try:
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(
            encoding="ascii"
        ).strip()
        raw = (Path("/proc") / str(pid) / "stat").read_text(encoding="ascii")
        closing = raw.rfind(")")
        if closing < 0:
            raise ValueError("malformed proc stat")
        fields = raw[closing + 2 :].split()
        if fields[0] in {"Z", "X", "x"}:
            raise ProcessIdentityError("process is not live")
        start_ticks = fields[19]
        identity = f"{boot_id}:{start_ticks}"
    except (OSError, IndexError, UnicodeError, ValueError) as exc:
        raise ProcessIdentityError("cannot read process start identity") from exc
    if _POSIX_START.fullmatch(identity) is None:
        raise ProcessIdentityError("cannot read process start identity")
    return identity


class SimChildLedgerError(RuntimeError):
    """A simulation child recovery record could not be loaded or published."""


@dataclass(frozen=True)
class SimChildRecord:
    """Persisted identity of one direct child process."""

    target: str
    process_identity: ProcessIdentity
    process_group: int
    started_wall_ns: int
    launch_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.target, str) or _TARGET.fullmatch(self.target) is None:
            raise SimChildLedgerError("simulation child target is invalid")
        if not isinstance(self.process_identity, ProcessIdentity):
            raise SimChildLedgerError("simulation child process identity is invalid")
        if self.process_group != self.process_identity.pid:
            raise SimChildLedgerError("simulation child process group is invalid")
        if not isinstance(self.started_wall_ns, int) or self.started_wall_ns <= 0:
            raise SimChildLedgerError("simulation child start time is invalid")
        if not isinstance(self.launch_id, str) or not self.launch_id:
            raise SimChildLedgerError("simulation child launch id is invalid")

    def as_dict(self) -> dict[str, Any]:
        return {
            "target": self.target,
            "process_identity": self.process_identity.as_dict(),
            "process_group": self.process_group,
            "started_wall_ns": self.started_wall_ns,
            "launch_id": self.launch_id,
        }


@dataclass(frozen=True)
class SimChildSnapshot:
    """Children owned by one published Product run."""

    product_session_id: str
    children: tuple[SimChildRecord, ...]

    def __post_init__(self) -> None:
        if not is_product_session_id(self.product_session_id):
            raise SimChildLedgerError("simulation child session identity is invalid")
        targets = [child.target for child in self.children]
        if any(not isinstance(child, SimChildRecord) for child in self.children) or len(
            targets
        ) != len(set(targets)):
            raise SimChildLedgerError("ledger children are invalid")

    @classmethod
    def create(
        cls,
        *,
        product_session_id: str,
        children: Iterable[SimChildRecord] = (),
    ) -> SimChildSnapshot:
        return cls(
            product_session_id=product_session_id,
            children=tuple(sorted(children, key=lambda child: child.target)),
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": _CHILD_LEDGER_SCHEMA,
            "product_session_id": self.product_session_id,
            "children": [child.as_dict() for child in self.children],
        }


def _parse_snapshot(payload: Any) -> SimChildSnapshot:
    if not isinstance(payload, dict) or payload.get("schema_version") != _CHILD_LEDGER_SCHEMA:
        raise SimChildLedgerError("simulation child ledger schema is unsupported")
    try:
        children = tuple(
            SimChildRecord(
                target=child["target"],
                process_identity=ProcessIdentity.from_dict(child["process_identity"]),
                process_group=child["process_group"],
                started_wall_ns=child["started_wall_ns"],
                launch_id=child["launch_id"],
            )
            for child in payload["children"]
        )
        return SimChildSnapshot(
            product_session_id=payload["product_session_id"],
            children=children,
        )
    except (KeyError, TypeError, ProcessIdentityError, SimChildLedgerError) as exc:
        raise SimChildLedgerError("simulation child ledger is invalid") from exc


class SimChildLedger:
    """Store one session's direct child ownership."""

    def __init__(self, session_root: Path) -> None:
        self._directory = session_root / "supervisor"
        self._path = self._directory / "children.json"

    def load(self) -> SimChildSnapshot | None:
        try:
            payload = json.loads(self._path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise SimChildLedgerError("simulation child ledger is invalid") from exc
        return _parse_snapshot(payload)

    def replace(self, snapshot: SimChildSnapshot) -> None:
        if not isinstance(snapshot, SimChildSnapshot):
            raise SimChildLedgerError("simulation child ledger snapshot is invalid")
        temporary = self._path.with_suffix(".tmp")
        try:
            self._directory.mkdir(exist_ok=True)
            temporary.write_text(
                json.dumps(snapshot.as_dict(), separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, self._path)
        except OSError as exc:
            raise SimChildLedgerError("simulation child ledger cannot be published") from exc
        finally:
            temporary.unlink(missing_ok=True)

    def clear(self) -> None:
        try:
            self._path.unlink()
        except FileNotFoundError:
            return
        except OSError as exc:
            raise SimChildLedgerError("simulation child ledger cannot be cleared") from exc


__all__ = [
    "ProcessIdentity",
    "ProcessIdentityError",
    "SimChildLedger",
    "SimChildLedgerError",
    "SimChildRecord",
    "SimChildSnapshot",
]
