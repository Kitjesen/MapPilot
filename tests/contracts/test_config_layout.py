"""Contracts for the committed configuration surface."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_retired_config_surfaces_are_absent() -> None:
    """Retired ROS and compatibility configuration must not return."""
    retired = (
        "config/decision.yaml",
        "config/dufomap.toml",
        "config/endpoints.yaml",
        "config/fastdds_no_shm.xml",
        "config/perception.yaml",
        "config/semantic_exploration.yaml",
        "src/runtime/config_loader.py",
        "tests/integration/run_semantic_planner_test.sh",
        "tests/scripts/test_semantic_nav.sh",
    )

    present = [path for path in retired if (ROOT / path).exists()]

    assert present == []  # noqa: S101
