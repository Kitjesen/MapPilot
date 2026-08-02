from __future__ import annotations

import pytest

from runtime.profiles.catalog.profile_adapters import ProfileAdapterError
from runtime.profiles.launcher import (
    build_external_launch_context,
    resolve_runtime_process_context,
)
from runtime.profiles.resolver import resolve_profile_config


def test_external_launch_context_uses_run_spec_command_and_overrides_env(tmp_path) -> None:
    config = resolve_profile_config(
        "sim_mujoco_live",
        profile_adapter="mujoco_live",
    )

    context = build_external_launch_context(
        "sim_mujoco_live",
        config,
        repo_root=tmp_path,
        record=True,
        base_env={
            "KEEP_ME": "1",
            "LINGTU_PROFILE": "nav",
            "LINGTU_PROFILE_ADAPTER": "thunder_dds",
        },
    )

    assert context.command == (
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "video",
    )
    assert context.launcher_path == (tmp_path / "sim/scripts/mujoco/launch_fastlio2_live.sh").resolve()
    assert context.env["KEEP_ME"] == "1"
    assert context.env["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert context.env["LINGTU_PROFILE_ADAPTER"] == "mujoco_live"
    assert context.env["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert context.env["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert context.env["LINGTU_SIMULATION_ONLY"] == "1"


def test_external_launch_context_rejects_profiles_without_launcher(tmp_path) -> None:
    config = resolve_profile_config("lite")

    with pytest.raises(ProfileAdapterError, match="_external_launcher"):
        build_external_launch_context("lite", config, repo_root=tmp_path)


def test_runtime_process_context_without_base_env_returns_runtime_env_only() -> None:
    config = resolve_profile_config("lite")

    context = resolve_runtime_process_context("lite", config)

    assert context.spec.launcher is None
    assert context.env == context.spec.env
