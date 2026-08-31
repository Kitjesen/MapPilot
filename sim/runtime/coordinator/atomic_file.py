"""Bounded atomic file replacement for transient Windows sharing failures."""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path

ReplaceFile = Callable[[Path, Path], object]
Sleep = Callable[[float], object]

_DEFAULT_ATTEMPTS = 101
_DEFAULT_DELAY_S = 0.02
_RETRYABLE_WINDOWS_ERRORS = frozenset({5, 32, 33})


def replace_file_with_retry(
    temporary: Path,
    destination: Path,
    *,
    attempts: int = _DEFAULT_ATTEMPTS,
    delay_s: float = _DEFAULT_DELAY_S,
    replace: ReplaceFile = Path.replace,
    sleep: Sleep = time.sleep,
) -> None:
    """Atomically replace a file, retrying only transient access conflicts."""

    if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts <= 0:
        raise ValueError("attempts must be a positive integer")
    if delay_s < 0:
        raise ValueError("delay_s must be non-negative")

    for attempt in range(attempts):
        try:
            replace(temporary, destination)
            return
        except OSError as exc:
            if not _is_retryable_replace_error(exc) or attempt + 1 >= attempts:
                raise
            sleep(delay_s)


def _is_retryable_replace_error(error: OSError) -> bool:
    if isinstance(error, PermissionError):
        return True
    return getattr(error, "winerror", None) in _RETRYABLE_WINDOWS_ERRORS


__all__ = ["replace_file_with_retry"]
