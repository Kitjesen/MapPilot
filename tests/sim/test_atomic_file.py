# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

import pytest

from sim.runtime.coordinator.atomic_file import replace_file_with_retry

TEMPORARY = Path("session.runtime.json.tmp")
DESTINATION = Path("session.runtime.json")


class _WindowsReplaceError(OSError):
    def __init__(self, winerror: int) -> None:
        super().__init__(f"Windows replace error {winerror}")
        self.winerror = winerror


def test_permission_error_is_retried_until_replace_succeeds() -> None:
    calls: list[tuple[Path, Path]] = []
    sleeps: list[float] = []

    def replace(source: Path, destination: Path) -> None:
        calls.append((source, destination))
        if len(calls) < 3:
            raise PermissionError("manifest is being read")

    replace_file_with_retry(
        TEMPORARY,
        DESTINATION,
        attempts=3,
        delay_s=0.002,
        replace=replace,
        sleep=sleeps.append,
    )

    assert calls == [(TEMPORARY, DESTINATION)] * 3
    assert sleeps == [0.002, 0.002]


def test_default_retry_window_survives_a_longer_windows_reader_conflict() -> None:
    calls = 0
    sleeps: list[float] = []

    def replace(_source: Path, _destination: Path) -> None:
        nonlocal calls
        calls += 1
        if calls <= 5:
            raise PermissionError("manifest reader still owns the destination")

    replace_file_with_retry(
        TEMPORARY,
        DESTINATION,
        replace=replace,
        sleep=sleeps.append,
    )

    assert calls == 6
    assert len(sleeps) == 5


@pytest.mark.parametrize("winerror", (5, 32, 33))
def test_windows_access_conflicts_are_retried(winerror: int) -> None:
    calls = 0
    sleeps: list[float] = []

    def replace(_source: Path, _destination: Path) -> None:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise _WindowsReplaceError(winerror)

    replace_file_with_retry(
        TEMPORARY,
        DESTINATION,
        attempts=2,
        delay_s=0,
        replace=replace,
        sleep=sleeps.append,
    )

    assert calls == 2
    assert sleeps == [0]


def test_other_os_error_is_raised_without_retry() -> None:
    failure = OSError("disk full")
    calls = 0
    sleeps: list[float] = []

    def replace(_source: Path, _destination: Path) -> None:
        nonlocal calls
        calls += 1
        raise failure

    with pytest.raises(OSError) as captured:
        replace_file_with_retry(
            TEMPORARY,
            DESTINATION,
            replace=replace,
            sleep=sleeps.append,
        )

    assert captured.value is failure
    assert calls == 1
    assert sleeps == []


def test_persistent_permission_error_is_raised_after_attempt_limit() -> None:
    failure = PermissionError("manifest remains locked")
    calls = 0
    sleeps: list[float] = []

    def replace(_source: Path, _destination: Path) -> None:
        nonlocal calls
        calls += 1
        raise failure

    with pytest.raises(PermissionError) as captured:
        replace_file_with_retry(
            TEMPORARY,
            DESTINATION,
            attempts=3,
            delay_s=0.005,
            replace=replace,
            sleep=sleeps.append,
        )

    assert captured.value is failure
    assert calls == 3
    assert sleeps == [0.005, 0.005]
