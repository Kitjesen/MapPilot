from __future__ import annotations

import shutil
import subprocess
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[2]


def _is_test_path(path: PurePosixPath) -> bool:
    if {"test", "tests"} & {part.lower() for part in path.parts[:-1]}:
        return True
    name = path.name.lower()
    return path.suffix.lower() in {".py", ".c", ".cc", ".cpp", ".cxx"} and (
        name.startswith("test_") or path.stem.lower().endswith("_test")
    )


def _is_allowed_exception(path: PurePosixPath) -> bool:
    value = path.as_posix()
    return (
        value.startswith("web/tests/")
        # Independently distributed SDKs retain their native package test layout.
        or value.startswith("sdk/")
        or value.startswith("tools/simstudio/ui/tests/")
        or value.startswith("tools/calibration/")
        or (
            value.startswith("sim/runtime/visual/RobotSimUE/")
            and "/Source/" in value
            and "/Private/Tests/" in value
        )
    )


def test_repository_tests_live_under_tests() -> None:
    git = shutil.which("git")
    assert git is not None
    files = subprocess.run(
        [git, "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()

    misplaced = []
    for value in files:
        if not (ROOT / value).is_file():
            continue
        path = PurePosixPath(value)
        if value.startswith(("tests/", "research/", "third_party/")):
            continue
        if _is_test_path(path) and not _is_allowed_exception(path):
            misplaced.append(value)

    assert misplaced == [], "Tests outside tests/:\n" + "\n".join(misplaced)
