"""Semantic planner docs should not drift back to ROS2-owned runtime wording."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_semantic_planner_readme_describes_pure_module_orchestration() -> None:
    readme = (ROOT / "src" / "decision" / "README.md").read_text(encoding="utf-8")

    assert "Semantic planning is pure Python Module orchestration" in readme


def test_exploration_readme_keeps_ros2_bridge_as_compatibility_only() -> None:
    readme = (ROOT / "src" / "explore" / "tare" / "README.md").read_text(encoding="utf-8")

    assert "The CMU C++/ROS TARE project is not vendored here" in readme
    assert "adapters/ros2" in readme
