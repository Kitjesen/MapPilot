"""The Linux platform name must not overwrite the selected GTSAM release."""

import io
import json
import os
import shutil
import subprocess
import tarfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.skipif(os.name == "nt", reason="Linux SDK preparation entrypoint")
def test_os_release_does_not_replace_the_dependency_version(tmp_path: Path) -> None:
    checkout = tmp_path / "checkout"
    scripts = checkout / "scripts" / "build"
    (scripts / "locks").mkdir(parents=True)
    shutil.copy(ROOT / "scripts/build/prepare_gtsam.sh", scripts)
    shutil.copy(ROOT / "scripts/build/locks/gtsam.json", scripts / "locks")
    version = json.loads((scripts / "locks/gtsam.json").read_text())["version"]
    archive = tmp_path / "upstream.tar.gz"
    with tarfile.open(archive, "w:gz") as package:
        contents = b"# fixture: CMake execution is intercepted\n"
        entry = tarfile.TarInfo(f"gtsam-{version}/CMakeLists.txt")
        entry.size = len(contents)
        package.addfile(entry, io.BytesIO(contents))
    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "curl").write_text(
        '#!/bin/bash\n'
        'printf "%s\\n" "$@" > "$DOWNLOAD_ARGS"\n'
        'while [[ "$1" != -o ]]; do shift; done\n'
        'cp "$FIXTURE_ARCHIVE" "$2"\n'
    )
    (tools / "cmake").write_text(
        '#!/bin/bash\nprintf "%s\\n" "$@" > "$CONFIGURE_ARGS"\nexit 73\n'
    )
    for tool in tools.iterdir():
        tool.chmod(0o755)
    env = {key: value for key, value in os.environ.items()
           if not key.startswith("LINGTU_GTSAM_")}
    env.update(PATH=f"{tools}:{env['PATH']}", FIXTURE_ARCHIVE=str(archive),
               DOWNLOAD_ARGS=str(tmp_path / "download.txt"),
               CONFIGURE_ARGS=str(tmp_path / "configure.txt"))
    result = subprocess.run(["bash", str(scripts / "prepare_gtsam.sh")], env=env,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 73, result.stdout + result.stderr
    assert f"/archive/refs/tags/{version}.tar.gz" in (tmp_path / "download.txt").read_text()
    arguments = (tmp_path / "configure.txt").read_text().splitlines()
    prefix = next(arg for arg in arguments if arg.startswith("-DCMAKE_INSTALL_PREFIX="))
    assert f"/gtsam-{version}-" in prefix
    assert "-DGTSAM_USE_SYSTEM_EIGEN=ON" in arguments
