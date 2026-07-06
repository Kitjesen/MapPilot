from __future__ import annotations

import sys

import pytest

from runtime.profiles.catalog.endpoints import RuntimeEndpointError
from runtime.profiles.launcher import (
    build_external_launch_context,
    resolve_runtime_process_context,
)
from runtime.profiles.resolver import resolve_profile_config


def test_external_launch_context_uses_run_spec_command_and_overrides_env(tmp_path) -> None:
    config = resolve_profile_config("explore", runtime_endpoint="mujoco_live")

    context = build_external_launch_context(
        "explore",
        config,
        repo_root=tmp_path,
        record=True,
        base_env={
            "KEEP_ME": "1",
            "LINGTU_PROFILE": "nav",
            "LINGTU_ENDPOINT": "thunder_field",
        },
    )

    assert context.command == (
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "video",
    )
    assert context.launcher_path == (
        tmp_path / "sim/scripts/mujoco/launch_fastlio2_live.sh"
    ).resolve()
    assert context.env["KEEP_ME"] == "1"
    assert context.env["LINGTU_PROFILE"] == "explore"
    assert context.env["LINGTU_ENDPOINT"] == "mujoco_live"
    assert context.env["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert context.env["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert context.env["LINGTU_SIMULATION_ONLY"] == "1"


def test_external_launch_context_uses_python_for_python_launchers(tmp_path) -> None:
    config = resolve_profile_config("nav", runtime_endpoint="replay")

    context = build_external_launch_context("nav", config, repo_root=tmp_path)

    assert context.command == (
        sys.executable,
        "sim/scripts/fastlio2_rosbag_replay_gate.py",
        "gate",
    )


def test_external_launch_context_rejects_profiles_without_launcher(tmp_path) -> None:
    config = resolve_profile_config("thunder-lite")

    with pytest.raises(RuntimeEndpointError, match="_external_launcher"):
        build_external_launch_context("lite", config, repo_root=tmp_path)


def test_runtime_process_context_without_base_env_returns_runtime_env_only() -> None:
    config = resolve_profile_config("thunder-lite")

    context = resolve_runtime_process_context("lite", config)

    assert context.spec.launcher is None
    assert context.env == context.spec.env
