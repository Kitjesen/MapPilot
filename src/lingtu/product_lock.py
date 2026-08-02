"""Cross-process serialization for ProductControl lifecycle mutations."""

from __future__ import annotations

import errno
import math
import os
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

LOCK_TIMEOUT_ENV = "LINGTU_PRODUCT_LOCK_TIMEOUT_S"
DEFAULT_LOCK_TIMEOUT_S = 30.0
LOCK_FILE_NAME = ".product-control.lock"
CURRENT_RUN_FILE_ENV = "LINGTU_CURRENT_FILE"
CURRENT_RUN_FILE_NAME = "current.json"


class ProductControlBusy(RuntimeError):
    """Raised when another process owns the ProductControl mutation lock."""


@dataclass
class _HeldLock:
    handle: BinaryIO
    depth: int = 1


_guard = threading.Lock()
_thread_mutexes: dict[str, threading.RLock] = {}
_local = threading.local()


def resolve_product_state_dir(
    state_dir: str | Path | None = None,
    *,
    environment: Mapping[str, str] | None = None,
) -> Path:
    """Resolve the state directory shared by switch records and their lock."""

    env = environment if environment is not None else os.environ
    configured = (
        state_dir
        or env.get("LINGTU_SESSION_ROOT")
        or "/run/lingtu"
    )
    return Path(configured).expanduser().resolve()


def resolve_current_run_path(
    state_dir: str | Path | None = None,
    *,
    environment: Mapping[str, str] | None = None,
) -> Path:
    """Resolve the canonical current-run record path."""

    env = environment if environment is not None else os.environ
    configured_file = str(env.get(CURRENT_RUN_FILE_ENV) or "").strip()
    if configured_file:
        return Path(configured_file).expanduser().resolve()
    return resolve_product_state_dir(state_dir, environment=env) / CURRENT_RUN_FILE_NAME


class ProductControlLock:
    """A reentrant thread and process lock scoped to one Product state directory."""

    def __init__(
        self,
        state_dir: str | Path | None = None,
        *,
        environment: Mapping[str, str] | None = None,
        timeout_s: float | None = None,
        poll_interval_s: float = 0.05,
    ) -> None:
        env = environment if environment is not None else os.environ
        self.state_dir = resolve_product_state_dir(state_dir, environment=env)
        self.path = self.state_dir / LOCK_FILE_NAME
        configured_timeout = (
            timeout_s
            if timeout_s is not None
            else float(env.get(LOCK_TIMEOUT_ENV, DEFAULT_LOCK_TIMEOUT_S))
        )
        if not math.isfinite(configured_timeout) or configured_timeout < 0:
            raise ValueError("ProductControl lock timeout must be finite and non-negative")
        if not math.isfinite(poll_interval_s) or poll_interval_s <= 0:
            raise ValueError("ProductControl lock poll interval must be finite and positive")
        self._timeout_s = configured_timeout
        self._poll_interval_s = poll_interval_s
        self._key = os.path.normcase(str(self.path))
        self._mutex: threading.RLock | None = None
        self._entered = False

    def __enter__(self) -> ProductControlLock:
        if self._entered:
            raise RuntimeError("ProductControlLock instance is already entered")
        self.state_dir.mkdir(parents=True, exist_ok=True)
        mutex = _mutex_for(self._key)
        if not mutex.acquire(timeout=self._timeout_s):
            raise ProductControlBusy(
                f"another ProductControl mutation owns {self.path}"
            )
        self._mutex = mutex
        try:
            held = _held_locks()
            existing = held.get(self._key)
            if existing is not None:
                existing.depth += 1
            else:
                handle = self.path.open("a+b")
                try:
                    _ensure_lock_byte(handle)
                    self._acquire_file_lock(handle)
                except BaseException:
                    handle.close()
                    raise
                held[self._key] = _HeldLock(handle=handle)
            self._entered = True
            return self
        except BaseException:
            self._mutex = None
            mutex.release()
            raise

    def __exit__(self, exc_type, exc, traceback) -> None:
        if not self._entered or self._mutex is None:
            raise RuntimeError("ProductControlLock is not entered")
        mutex = self._mutex
        try:
            held = _held_locks()
            entry = held[self._key]
            entry.depth -= 1
            if entry.depth == 0:
                try:
                    _release_file_lock(entry.handle)
                finally:
                    entry.handle.close()
                    del held[self._key]
        finally:
            self._entered = False
            self._mutex = None
            mutex.release()

    def _acquire_file_lock(self, handle: BinaryIO) -> None:
        deadline = time.monotonic() + self._timeout_s
        while True:
            try:
                _try_file_lock(handle)
                return
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ProductControlBusy(
                        f"another ProductControl mutation owns {self.path}"
                    ) from None
                time.sleep(min(self._poll_interval_s, remaining))


def _mutex_for(key: str) -> threading.RLock:
    with _guard:
        return _thread_mutexes.setdefault(key, threading.RLock())


def _held_locks() -> dict[str, _HeldLock]:
    held = getattr(_local, "held", None)
    if held is None:
        held = {}
        _local.held = held
    return held


def _ensure_lock_byte(handle: BinaryIO) -> None:
    handle.seek(0, os.SEEK_END)
    if handle.tell() == 0:
        handle.write(b"\0")
        handle.flush()
    handle.seek(0)


if os.name == "nt":
    import msvcrt

    def _try_file_lock(handle: BinaryIO) -> None:
        handle.seek(0)
        try:
            msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError as exc:
            raise BlockingIOError(exc.errno, str(exc)) from exc

    def _release_file_lock(handle: BinaryIO) -> None:
        handle.seek(0)
        msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)

else:
    import fcntl

    def _try_file_lock(handle: BinaryIO) -> None:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno not in {errno.EACCES, errno.EAGAIN}:
                raise
            raise BlockingIOError(exc.errno, str(exc)) from exc

    def _release_file_lock(handle: BinaryIO) -> None:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
