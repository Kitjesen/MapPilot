"""Runtime-specific pytest fixtures."""

import os
from inspect import signature
from pathlib import Path

import pytest


@pytest.fixture(autouse=True)
def _isolate_process_environment():
    """Keep in-process CLI tests from leaking a product runtime into later tests."""

    snapshot = dict(os.environ)
    yield
    os.environ.clear()
    os.environ.update(snapshot)

# Integration harnesses run module-level setup at import time.
# Both files now expose proper def test_*() functions and guard sys.exit()
# in `if __name__ == "__main__":`, so pytest can collect them safely.


def _pytest_ignore_collect_impl(candidate):
    try:
        from tests.runtime.numpy_guard import numpy_import_is_safe
    except Exception:
        return False

    candidate = Path(str(candidate))
    if candidate.suffix != ".py" or candidate.name == "conftest.py":
        return False
    if not str(candidate).startswith(os.path.dirname(__file__)):
        return False
    if numpy_import_is_safe():
        return False
    try:
        source = candidate.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return False
    return "import numpy as np" in source or "from numpy" in source


try:
    from _pytest import hookspec as _pytest_hookspec

    _ignore_collect_params = signature(_pytest_hookspec.pytest_ignore_collect).parameters
except Exception:
    _ignore_collect_params = {}

if "collection_path" in _ignore_collect_params:

    def pytest_ignore_collect(collection_path, config):
        return _pytest_ignore_collect_impl(collection_path)

else:

    def pytest_ignore_collect(path, config):
        return _pytest_ignore_collect_impl(path)
