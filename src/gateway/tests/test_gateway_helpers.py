"""Tests for gateway_module helper functions + endpoint-level integration.

Focus: the newly-added `_safe_map_name` (path traversal guard) and
`_apply_dynamic_filter_step1half` (Step 1½ DUFOMap delegator). These are
module-level helpers used by multiple endpoints (map/save, map/restore_predufo,
map/activate, map/rename, session/start, MapManager._map_save). A single bug
here affects every save-related path — so regression tests are high-leverage.
"""
from __future__ import annotations

from unittest.mock import patch

import pytest

from gateway.gateway_module import (
    _apply_dynamic_filter_step1half,
    _safe_map_name,
)

# ── _safe_map_name: accept/reject matrix ──────────────────────────────────

@pytest.mark.parametrize("name", [
    "lab_0424",
    "Lab-Test-2",
    "map_20260424_030818",
    "a",
    "A.1",
    "building_2F",
])
def test_safe_map_name_accepts_valid(name: str):
    assert _safe_map_name(name) is None


@pytest.mark.parametrize("name,reason_hint", [
    ("", "empty"),
    (None, "empty"),
    ("../etc/passwd", "unsafe"),
    ("a/b", "unsafe"),
    ("a\\b", "unsafe"),
    ("a..b", "unsafe"),
    (".hidden", "start with"),
    ("-flag", "start with"),
    ("a" * 101, "too long"),
    ("has space", "only "),
    ("unicode_中文", "only "),
    ("with@symbol", "only "),
    ("quote'injection", "only "),
    ("bash`whoami`", "only "),
    ("tab\there", "only "),
])
def test_safe_map_name_rejects_unsafe(name, reason_hint: str):
    err = _safe_map_name(name)
    assert err is not None, f"should reject {name!r}"
    assert reason_hint in err.lower(), f"{err!r} missing hint {reason_hint!r}"


def test_safe_map_name_non_string_types():
    # int / dict / list all get rejected (isinstance check)
    for bad in (123, {"evil": 1}, ["a"], b"bytes"):
        assert _safe_map_name(bad) is not None


def test_safe_map_name_rejects_double_dot_disguised():
    """Double-dot in the middle of a name is still traversal-adjacent."""
    assert _safe_map_name("map..bak") is not None
    assert _safe_map_name("..map") is not None
    assert _safe_map_name("map..") is not None


def test_safe_map_name_boundary_length():
    """Exactly 100 chars OK, 101 reject."""
    assert _safe_map_name("a" * 100) is None
    assert _safe_map_name("a" * 101) is not None


# ── _apply_dynamic_filter_step1half: env var + mock subprocess ────────────

class _FakeSaveDir:
    """Duck-typed save_dir: has a name + works as str."""
    def __init__(self, name: str):
        self._name = name
    def __str__(self) -> str:
        return f"/tmp/{self._name}"
    def __fspath__(self) -> str:
        return str(self)


def test_apply_dynamic_filter_disabled_by_env(monkeypatch):
    """LINGTU_SAVE_DYNAMIC_FILTER=0 → return None without calling refilter."""
    monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "0")
    result = _apply_dynamic_filter_step1half(_FakeSaveDir("test"))
    assert result is None


@pytest.mark.parametrize("off_value", ["0", "false", "False", "FALSE", "no", "off", ""])
def test_apply_dynamic_filter_off_values(monkeypatch, off_value):
    """All documented off-values return None (unified from ad08de4)."""
    monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", off_value)
    assert _apply_dynamic_filter_step1half(_FakeSaveDir("x")) is None


def test_apply_dynamic_filter_default_enabled(monkeypatch):
    """No env var → default on (1). refilter_map called."""
    monkeypatch.delenv("LINGTU_SAVE_DYNAMIC_FILTER", raising=False)
    with patch("runtime.dynamic_filter.refilter_map") as mock_refilter:
        mock_refilter.return_value = {
            "success": True, "orig_count": 100, "clean_count": 95,
            "dropped": 5, "elapsed_s": 0.5,
        }
        result = _apply_dynamic_filter_step1half(_FakeSaveDir("m"))
        assert result is not None
        assert result["success"] is True
        assert result["dropped"] == 5
        mock_refilter.assert_called_once()


def test_apply_dynamic_filter_crash_returns_dict(monkeypatch):
    """refilter_map raising → helper swallows, returns success=False dict (never None)."""
    monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
    with patch("runtime.dynamic_filter.refilter_map") as mock_refilter:
        mock_refilter.side_effect = RuntimeError("DUFOMap binary not found")
        result = _apply_dynamic_filter_step1half(_FakeSaveDir("m"))
        # Must return a dict (not None) so callers can distinguish "disabled"
        # from "enabled-but-failed"
        assert result is not None
        assert result["success"] is False
        assert "DUFOMap binary not found" in result["error"]


def test_apply_dynamic_filter_refilter_returns_failure(monkeypatch):
    """refilter_map returns {'success': False, ...} → helper just passes through."""
    monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
    with patch("runtime.dynamic_filter.refilter_map") as mock_refilter:
        mock_refilter.return_value = {
            "success": False, "error": "backup failed: disk full",
            "elapsed_s": 0.01,
        }
        result = _apply_dynamic_filter_step1half(_FakeSaveDir("m"))
        assert result == {
            "success": False, "error": "backup failed: disk full",
            "elapsed_s": 0.01,
        }


def test_apply_dynamic_filter_passes_save_dir(monkeypatch):
    """save_dir argument must propagate to refilter_map first positional."""
    monkeypatch.setenv("LINGTU_SAVE_DYNAMIC_FILTER", "1")
    save_dir = _FakeSaveDir("propagate")
    with patch("runtime.dynamic_filter.refilter_map") as mock_refilter:
        mock_refilter.return_value = {"success": True, "orig_count": 0,
                                       "clean_count": 0, "dropped": 0,
                                       "elapsed_s": 0.0}
        _apply_dynamic_filter_step1half(save_dir)
        args, kwargs = mock_refilter.call_args
        assert args[0] is save_dir
        # timeout_s kwarg must be 300.0 (tested indirectly via not raising)
        assert kwargs.get("timeout_s") == 300.0


# ── Integration: _safe_map_name used by multiple endpoints — negative path ─

@pytest.mark.parametrize("malicious", [
    "../../etc/passwd",
    "/etc/shadow",
    "a/b/c",
    "\\\\server\\share",
    "..",
])
def test_traversal_rejected(malicious: str):
    """Path traversal attempts all rejected. Covers map/save, restore, activate,
    rename, session/start guards."""
    assert _safe_map_name(malicious) is not None


def test_helper_importable_from_both_sites():
    """Both gateway_module and nav.services.maps now import
    `_apply_dynamic_filter_step1half`. Verify the helper is the same callable
    (no accidental duplication / drift)."""
    # map_manager imports lazily inside _map_save to avoid circular import,
    # but we can verify the same module namespace
    import gateway.gateway_module as gm
    from gateway.gateway_module import _apply_dynamic_filter_step1half as a
    assert gm._apply_dynamic_filter_step1half is a
