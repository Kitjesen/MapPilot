"""Build the standalone Python SDK from the authoritative runtime sources."""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = ROOT / "sdk" / "python"
SOURCE = ROOT / "src" / "lingtu"


def stage_source(build_root: Path) -> Path:
    stage = build_root.resolve() / "python"
    if stage.exists():
        stage.resolve().relative_to(build_root.resolve())
        shutil.rmtree(stage)
    package = stage / "src" / "lingtu"
    package.mkdir(parents=True)
    shutil.copy2(SOURCE / "__init__.py", package / "__init__.py")
    shutil.copytree(
        SOURCE / "sdk",
        package / "sdk",
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "tests"),
    )
    project = (ROOT / "pyproject.toml").read_text(encoding="utf-8").split("[project]", 1)[1].split("\n[", 1)[0]
    match = re.search(r'^version\s*=\s*"([^"]+)"', project, re.MULTILINE)
    if match is None:
        raise ValueError("The repository project must declare its version")
    metadata = (TEMPLATE / "pyproject.toml.in").read_text(encoding="utf-8")
    (stage / "pyproject.toml").write_text(metadata.replace("@VERSION@", match[1]), encoding="utf-8")
    shutil.copy2(TEMPLATE / "README.md", stage / "README.md")
    return stage


def build_package(build_root: Path) -> Path:
    stage = stage_source(build_root)
    distribution = build_root.resolve() / "dist"
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys\n"
            "from setuptools.build_meta import build_sdist, build_wheel\n"
            "destination = sys.argv[1]\n"
            "build_sdist(destination)\n"
            "build_wheel(destination)\n",
            str(distribution),
        ],
        cwd=stage,
        check=True,
    )
    return distribution


if __name__ == "__main__":
    output = build_package(ROOT / "build" / "sdk")
    print(f"SDK wheel and source archive written to {output}")
