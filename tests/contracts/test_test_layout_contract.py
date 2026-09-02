from __future__ import annotations

import shutil
import subprocess
from pathlib import PurePosixPath


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
    tracked = subprocess.run(  # noqa: S603 - repository-owned executable and arguments
        [git, "ls-files"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()

    misplaced = []
    for value in tracked:
        path = PurePosixPath(value)
        if value.startswith(("tests/", "research/", "third_party/")):
            continue
        if _is_test_path(path) and not _is_allowed_exception(path):
            misplaced.append(value)

    assert misplaced == [], "Tests outside tests/:\n" + "\n".join(misplaced)
