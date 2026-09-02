"""Executable contracts for the Unreal C++ build prerequisite gate."""

# ruff: noqa: S101, S603, S607

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PREFLIGHT = REPO_ROOT / "sim" / "tools" / "toolchains" / "windows_netfxsdk.ps1"
UE_SCRIPTS = REPO_ROOT / "sim" / "runtime" / "visual" / "RobotSimUE" / "Scripts"


def test_ue_build_preflight_rejects_missing_explicit_netfxsdk(tmp_path: Path) -> None:
    missing_sdk = tmp_path / "missing-netfxsdk"

    result = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-File",
            str(PREFLIGHT),
            "-NetFxSdkRoot",
            str(missing_sdk),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    combined = result.stdout + result.stderr
    assert result.returncode != 0
    assert ".NET Framework SDK 4.6 or newer" in combined
    assert ".NET Framework 4.8 SDK + Targeting Pack" in combined
    assert "".join(str(missing_sdk).split()) in "".join(combined.split())


def test_ue_build_preflight_accepts_explicit_netfxsdk_with_required_header(
    tmp_path: Path,
) -> None:
    sdk_root = tmp_path / "NETFXSDK" / "4.8"
    header = sdk_root / "Include" / "um" / "mscoree.h"
    header.parent.mkdir(parents=True)
    header.write_text("// test SDK marker\n", encoding="ascii")

    result = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-File",
            str(PREFLIGHT),
            "-NetFxSdkRoot",
            str(sdk_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(sdk_root.resolve())


def test_ue_build_preflight_default_detection_honors_netfxsdk_environment(
    tmp_path: Path,
) -> None:
    sdk_parent = tmp_path / "NETFXSDK"
    sdk_root = sdk_parent / "4.8"
    header = sdk_root / "Include" / "um" / "mscoree.h"
    header.parent.mkdir(parents=True)
    header.write_text("// test SDK marker\n", encoding="ascii")
    old_header = sdk_parent / "4.5.2" / "Include" / "um" / "mscoree.h"
    old_header.parent.mkdir(parents=True)
    old_header.write_text("// obsolete test SDK marker\n", encoding="ascii")
    environment = os.environ.copy()
    environment["LINGTU_NETFXSDK_ROOT"] = str(sdk_parent)

    result = subprocess.run(
        ["powershell", "-NoProfile", "-File", str(PREFLIGHT)],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(sdk_root.resolve())


def test_ue_build_preflight_rejects_netfxsdk_older_than_4_6(tmp_path: Path) -> None:
    sdk_root = tmp_path / "NETFXSDK" / "4.5.2"
    header = sdk_root / "Include" / "um" / "mscoree.h"
    header.parent.mkdir(parents=True)
    header.write_text("// obsolete test SDK marker\n", encoding="ascii")

    result = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-File",
            str(PREFLIGHT),
            "-NetFxSdkRoot",
            str(sdk_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    combined = result.stdout + result.stderr
    assert result.returncode != 0
    assert "NETFXSDK 4.5.2 is too old" in combined
    assert ".NET Framework SDK 4.6 or newer" in combined


@pytest.mark.parametrize(
    "launcher_name",
    [
        "run_thunderv4_preview.ps1",
        "run_open_field_hf.ps1",
        "run_factory_park_hf.ps1",
    ],
)
def test_ue_launcher_fails_netfxsdk_preflight_before_external_processes(
    tmp_path: Path, launcher_name: str,
) -> None:
    unreal_root = tmp_path / "UE"
    build_marker = tmp_path / "build-started.txt"
    blender_marker = tmp_path / "blender-started.txt"
    build_bat = unreal_root / "Engine" / "Build" / "BatchFiles" / "Build.bat"
    build_bat.parent.mkdir(parents=True)
    build_bat.write_text(f"@echo started>{build_marker}\n", encoding="ascii")
    blender = tmp_path / "fake-blender.cmd"
    blender.write_text(f"@echo started>{blender_marker}\n", encoding="ascii")

    result = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-File",
            str(UE_SCRIPTS / launcher_name),
            "-UnrealRoot",
            str(unreal_root),
            "-BlenderExe",
            str(blender),
            "-NetFxSdkRoot",
            str(tmp_path / "missing-netfxsdk"),
            "-Unattended",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    combined = result.stdout + result.stderr
    assert result.returncode != 0
    assert "Unreal C++ build prerequisite is missing" in combined
    assert not build_marker.exists()
    assert not blender_marker.exists()


@pytest.mark.parametrize(
    "launcher_name",
    [
        "run_thunderv4_preview.ps1",
        "run_open_field_hf.ps1",
        "run_factory_park_hf.ps1",
    ],
)
def test_ue_launcher_delegates_build_bat_to_the_owned_finite_runner(
    launcher_name: str,
) -> None:
    source = (UE_SCRIPTS / launcher_name).read_text(encoding="utf-8")

    assert "[int]$UnrealBuildTimeoutMinutes" in source
    assert "sim\\tools\\toolchains\\ue_build.py" in source
    assert "--timeout-seconds" in source
    assert "--target RobotSimUEEditor" in source
    assert "--platform Win64" in source
    assert "--configuration Development" in source
    assert "& $buildBat" not in source
