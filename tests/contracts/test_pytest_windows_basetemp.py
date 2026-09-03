from __future__ import annotations

# ruff: noqa: S106
import importlib.util
import re
import sys
from pathlib import Path

import pytest

ROOT_CONFTEST = Path(__file__).resolve().parents[1] / "conftest.py"


def _load_root_conftest():
    spec = importlib.util.spec_from_file_location("_lingtu_root_conftest_under_test", ROOT_CONFTEST)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Config:
    def __init__(self):
        self.option = type("Option", (), {"basetemp": None})()


def test_windows_basetemp_is_unique_and_not_pytest_numbered(tmp_path):
    root_conftest = _load_root_conftest()

    first = root_conftest._make_windows_basetemp(parent=tmp_path, pid=1234, token="a")
    second = root_conftest._make_windows_basetemp(parent=tmp_path, pid=1234, token="b")

    assert first != second
    assert first.parent == second.parent == tmp_path / "ltp"
    assert "pytest-current" not in first.parts
    assert re.fullmatch(r"pytest-\d+", first.name) is None


def test_windows_default_assigns_basetemp_when_option_is_absent(monkeypatch, tmp_path):
    root_conftest = _load_root_conftest()
    config = _Config()

    monkeypatch.setattr(root_conftest.sys, "platform", "win32")
    monkeypatch.setenv("PYTEST_DEBUG_TEMPROOT", str(tmp_path))

    root_conftest.pytest_configure(config)

    assert config.option.basetemp.parent == tmp_path / "ltp"
    assert config.option.basetemp.name.startswith(f"{root_conftest.os.getpid()}-")


def test_explicit_basetemp_is_preserved_on_windows(monkeypatch, tmp_path):
    root_conftest = _load_root_conftest()
    explicit = tmp_path / "explicit"
    config = _Config()
    config.option.basetemp = explicit

    monkeypatch.setattr(root_conftest.sys, "platform", "win32")

    root_conftest.pytest_configure(config)

    assert config.option.basetemp == explicit


def test_non_windows_behavior_is_unchanged():
    root_conftest = _load_root_conftest()
    config = _Config()

    root_conftest.pytest_configure(config)

    if sys.platform == "win32":
        assert config.option.basetemp is not None
    else:
        assert config.option.basetemp is None


def test_pytest_tmp_path_factory_uses_root_hook_basetemp(pytestconfig, tmp_path_factory):
    if sys.platform != "win32":
        pytest.skip("Windows-specific regression coverage")

    basetemp = tmp_path_factory.getbasetemp()
    given_basetemp = tmp_path_factory._given_basetemp

    assert given_basetemp is not None
    assert Path(given_basetemp) == Path(pytestconfig.option.basetemp)
    assert "ltp" in basetemp.parts
    assert "pytest-current" not in basetemp.parts
    assert re.fullmatch(r"pytest-\d+", basetemp.name) is None
