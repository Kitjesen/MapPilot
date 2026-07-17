"""Focused tests for the Gateway map-name boundary."""

from __future__ import annotations

import pytest

from gateway.gateway_module import _safe_map_name


@pytest.mark.parametrize(
    "name",
    ["lab_0424", "Lab-Test-2", "map_20260424_030818", "a", "A.1", "building_2F"],
)
def test_safe_map_name_accepts_valid(name: str) -> None:
    assert _safe_map_name(name) is None


@pytest.mark.parametrize(
    ("name", "reason_hint"),
    [
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
    ],
)
def test_safe_map_name_rejects_unsafe(name, reason_hint: str) -> None:
    error = _safe_map_name(name)
    assert error is not None
    assert reason_hint in error.lower()


def test_safe_map_name_rejects_non_strings() -> None:
    for value in (123, {"evil": 1}, ["a"], b"bytes"):
        assert _safe_map_name(value) is not None


def test_safe_map_name_boundary_length() -> None:
    assert _safe_map_name("a" * 100) is None
    assert _safe_map_name("a" * 101) is not None
