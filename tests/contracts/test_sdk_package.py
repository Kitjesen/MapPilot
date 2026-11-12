"""The external Python SDK artifacts install independently of the runtime tree."""

from __future__ import annotations

import os
import subprocess
import sys
import zipfile
from email.parser import Parser
from pathlib import Path

import pytest
import setuptools
from packaging.requirements import Requirement
from tools.sdk.package_python import SOURCE, build_package, stage_source


def test_package_stages_authoritative_sources_and_removes_stale_files(tmp_path: Path) -> None:
    stage = stage_source(tmp_path)
    stale = stage / "src" / "lingtu" / "sdk" / "removed.py"
    stale.write_text("obsolete = True", encoding="utf-8")
    stage_source(tmp_path)
    assert not stale.exists()
    expected = {file.name: file.read_bytes() for file in (SOURCE / "sdk").glob("*.py")}
    actual = {file.name: file.read_bytes() for file in (stage / "src" / "lingtu" / "sdk").glob("*.py")}
    assert actual == expected


def test_wheel_and_sdist_install_without_runtime_dependencies(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    distribution = build_package(tmp_path / "build")
    wheel = next(distribution.glob("*.whl"))
    source = next(distribution.glob("*.tar.gz"))
    with zipfile.ZipFile(wheel) as archive:
        metadata_path = next(name for name in archive.namelist() if name.endswith(".dist-info/METADATA"))
        metadata = Parser().parsestr(archive.read(metadata_path).decode("utf-8"))
        assert metadata["Name"] == "lingtu-sdk"
        requirements = [Requirement(value) for value in metadata.get_all("Requires-Dist", [])]
        assert all(not requirement.marker.evaluate({"extra": ""}) for requirement in requirements)
        assert [requirement.name for requirement in requirements] == ["websockets"]
        assert all(requirement.marker.evaluate({"extra": "websocket"}) for requirement in requirements)
    for artifact in (wheel, source):
        target = tmp_path / artifact.suffix.removeprefix(".")
        subprocess.run(
            [
                sys.executable, "-m", "pip", "install", "--no-index", "--no-deps",
                "--no-build-isolation", "--no-compile", "--target", str(target), str(artifact),
            ],
            cwd=tmp_path,
            check=True,
        )
        subprocess.run(
            [
                sys.executable, "-I", "-S", "-c",
                "import sys; sys.path.insert(0, sys.argv[1]); "
                "from lingtu.sdk import AsyncLingTuClient, LingTuClient; "
                "import lingtu.sdk; assert lingtu.sdk.__file__.startswith(sys.argv[1]); "
                "assert LingTuClient and AsyncLingTuClient",
                str(target),
            ],
            cwd=tmp_path,
            check=True,
        )
        command = next(target.rglob("lingtu-sdk.exe" if os.name == "nt" else "lingtu-sdk"))
        monkeypatch.setenv("PYTHONPATH", str(target))
        completed = subprocess.run(
            [str(command), "--help"],
            cwd=tmp_path,
            text=True,
            capture_output=True,
            check=True,
        )
        assert "LingTu Robot CLI" in completed.stdout
