"""Smoke tests for repo-root tools/validate scripts after path moves."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]


def _run_validator(script: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, script],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def test_validate_topics_script_passes() -> None:
    result = _run_validator("tools/validate/validate_topics.py")
    assert result.returncode == 0, result.stdout + result.stderr


def test_validate_config_script_passes() -> None:
    result = _run_validator("tools/validate/validate_config.py")
    assert result.returncode == 0, result.stdout + result.stderr


def test_validate_architecture_boundaries_script_passes() -> None:
    result = _run_validator("tools/validate/validate_architecture_boundaries.py")
    assert result.returncode == 0, result.stdout + result.stderr


def test_validate_lite_package_script_passes() -> None:
    result = _run_validator("tools/validate/validate_lite_package.py")
    assert result.returncode == 0, result.stdout + result.stderr


def test_validate_real_deployment_script_passes() -> None:
    result = _run_validator("tools/validate/validate_real_deployment.py")
    assert result.returncode == 0, result.stdout + result.stderr


def test_validate_portable_lean_package_script_passes() -> None:
    result = _run_validator("tools/validate/validate_portable_lean_package.py")
    assert result.returncode == 0, result.stdout + result.stderr
