"""Root pytest configuration shared by all test trees."""

from __future__ import annotations

import os
import sys
import tempfile
import uuid
from pathlib import Path

import pytest

_WINDOWS_BASETEMP_ROOT = "lingtu-pytest-basetemp"


def _make_windows_basetemp(
    *,
    parent: Path | None = None,
    pid: int | None = None,
    token: str | None = None,
) -> Path:
    """Return a process-unique basetemp that bypasses pytest numbered dirs."""
    root = Path(parent or os.environ.get("PYTEST_DEBUG_TEMPROOT") or tempfile.gettempdir()).joinpath(
        _WINDOWS_BASETEMP_ROOT
    )
    root.mkdir(parents=True, exist_ok=True)
    process_id = pid if pid is not None else os.getpid()
    unique = token or uuid.uuid4().hex
    return root.joinpath(f"pid-{process_id}-{unique}")


def _should_assign_windows_basetemp(config: pytest.Config, *, platform: str | None = None) -> bool:
    return (platform or sys.platform) == "win32" and getattr(config.option, "basetemp", None) is None


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    """Set a Windows basetemp before pytest's tmpdir factory is configured."""
    if _should_assign_windows_basetemp(config):
        config.option.basetemp = _make_windows_basetemp()
