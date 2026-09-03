"""Repository-wide pytest setup."""

from __future__ import annotations

import asyncio
import os
import sys
import tempfile
import uuid
import warnings
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[1]
_WINDOWS_BASETEMP_ROOT = "ltp"

for _path in (_REPO, _REPO / "src"):
    value = str(_path)
    if value not in sys.path:
        sys.path.insert(0, value)


class _CompatEventLoopPolicy(asyncio.DefaultEventLoopPolicy):
    """Keep legacy get_event_loop() tests working on newer Python versions."""

    def get_event_loop(self):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="There is no current event loop",
                    category=DeprecationWarning,
                )
                return super().get_event_loop()
        except RuntimeError:
            loop = self.new_event_loop()
            self.set_event_loop(loop)
            return loop


asyncio.set_event_loop_policy(_CompatEventLoopPolicy())


@pytest.fixture
def allow_unbuilt_process_artifacts(monkeypatch: pytest.MonkeyPatch) -> None:
    """Let contract tests compile RunPlans without native build outputs."""

    from runtime.graph.processes import ProcessArtifact

    monkeypatch.setattr(
        ProcessArtifact,
        "from_repository_path",
        classmethod(lambda cls, root, path: cls(str(path))),
    )


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
    return root / f"{process_id}-{unique[:12]}"


def _should_assign_windows_basetemp(
    config: pytest.Config,
    *,
    platform: str | None = None,
) -> bool:
    return (platform or sys.platform) == "win32" and getattr(config.option, "basetemp", None) is None


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    """Set a Windows basetemp before pytest's tmpdir factory is configured."""
    if _should_assign_windows_basetemp(config):
        config.option.basetemp = _make_windows_basetemp()
