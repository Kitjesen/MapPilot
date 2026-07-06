from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_sim_runtime_entrypoints_use_profile_builder() -> None:
    for rel_path in (
        "src/drivers/sim/mujoco/stack.py",
        "sim/scripts/run_semantic_full_stack.py",
        "sim/scripts/policy_nav_smoke.py",
        "sim/scripts/mujoco/record_policy_nav_video.py",
        "sim/validation/full_system.py",
    ):
        text = (ROOT / rel_path).read_text(encoding="utf-8-sig")

        assert "build_system_for_profile" in text


def test_cmu_unity_stack_entrypoint_is_deprecated_shim() -> None:
    text = (ROOT / "sim/scripts/cmu_unity_lingtu_stack.py").read_text(
        encoding="utf-8-sig"
    )

    assert "runtime was removed" in text.lower()
    assert "native simulation stack" in text.lower()
