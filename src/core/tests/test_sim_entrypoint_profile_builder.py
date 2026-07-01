from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_sim_runtime_entrypoints_use_profile_builder() -> None:
    for rel_path in (
        "src/drivers/sim/mujoco_lingtu_stack.py",
        "sim/scripts/run_semantic_full_stack.py",
        "sim/scripts/policy_nav_smoke.py",
        "sim/scripts/record_policy_nav_video.py",
        "sim/scripts/cmu_unity_lingtu_stack.py",
        "sim/validation/full_system.py",
    ):
        text = (ROOT / rel_path).read_text(encoding="utf-8-sig")

        assert "core.blueprints.full_stack" not in text
        assert "full_stack_blueprint(" not in text
        assert "build_system_for_profile" in text


def test_multi_robot_blueprint_uses_profile_selector() -> None:
    text = (ROOT / "src/core/blueprints/multi_robot.py").read_text(encoding="utf-8-sig")

    assert "core.blueprints.full_stack" not in text
    assert "full_stack_blueprint(" not in text
    assert "blueprint_for_resolved_profile" in text
