# ruff: noqa: D103, S101

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PRESETS = ROOT / "src" / "nav" / "cpp" / "CMakePresets.json"


def test_nav_portable_presets_cover_supported_build_hosts() -> None:
    document = json.loads(PRESETS.read_text(encoding="utf-8"))
    configure = {
        preset["name"]: preset
        for preset in document["configurePresets"]
        if not preset.get("hidden", False)
    }

    assert set(configure) == {
        "windows-x64-nav-portable",
        "windows-x64-nav-endpoint",
        "linux-x64-nav-portable",
        "linux-aarch64-nav-portable",
    }
    assert configure["windows-x64-nav-portable"]["generator"] == (
        "Visual Studio 17 2022"
    )
    assert configure["windows-x64-nav-portable"]["architecture"] == "x64"
    assert configure["windows-x64-nav-endpoint"]["inherits"] == (
        "windows-x64-nav-portable"
    )
    assert configure["windows-x64-nav-endpoint"]["cacheVariables"] == {
        "CMAKE_PREFIX_PATH": (
            "$env{LINGTU_CYCLONEDDS_PREFIX};$env{LINGTU_OCTOMAP_PREFIX}"
        ),
        "LINGTU_NAV_CPP_BUILD_ENDPOINT": "ON",
    }
    assert configure["linux-x64-nav-portable"]["generator"] == "Unix Makefiles"
    assert configure["linux-aarch64-nav-portable"]["generator"] == "Unix Makefiles"


def test_nav_portable_presets_keep_endpoint_out_of_baseline() -> None:
    document = json.loads(PRESETS.read_text(encoding="utf-8"))
    base = next(
        preset
        for preset in document["configurePresets"]
        if preset["name"] == "nav-portable-base"
    )

    assert base["hidden"] is True
    assert base["binaryDir"].startswith("${sourceDir}/../../../build/nav-cpp/")
    assert base["cacheVariables"] == {
        "LINGTU_NAV_CPP_BUILD_BENCHMARKS": "OFF",
        "LINGTU_NAV_CPP_BUILD_ENDPOINT": "OFF",
        "LINGTU_NAV_CPP_BUILD_TESTS": "ON",
    }
    configure = {
        preset["name"]: preset for preset in document["configurePresets"]
    }
    assert "CMAKE_BUILD_TYPE" not in configure[
        "windows-x64-nav-portable"
    ].get("cacheVariables", {})
    assert configure["linux-x64-nav-portable"]["cacheVariables"] == {
        "CMAKE_BUILD_TYPE": "Release"
    }
    assert configure["linux-aarch64-nav-portable"]["cacheVariables"] == {
        "CMAKE_BUILD_TYPE": "Release"
    }


def test_nav_portable_presets_have_matching_build_and_test_entries() -> None:
    document = json.loads(PRESETS.read_text(encoding="utf-8"))
    expected = {
        "windows-x64-nav-portable",
        "windows-x64-nav-endpoint",
        "linux-x64-nav-portable",
        "linux-aarch64-nav-portable",
    }

    build = {preset["name"]: preset for preset in document["buildPresets"]}
    test = {preset["name"]: preset for preset in document["testPresets"]}
    assert set(build) == expected
    assert set(test) == expected
    for name in expected:
        assert build[name]["configurePreset"] == name
        assert test[name]["configurePreset"] == name
        assert test[name]["output"]["outputOnFailure"] is True
