# ruff: noqa: S101, S603

from __future__ import annotations

import json
import runpy
import subprocess
import sys
from pathlib import Path

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.control import (
    CommandSubmitResult,
    ControllerCommand,
    ControllerRuntime,
    ControllerRuntimeError,
    ControllerState,
    GenerationStamp,
    create_production_components,
    load_control_plan,
)
from sim.runtime.coordinator import RuntimeCoordinator
from sim.runtime.coordinator.controlled_run import (
    BaseTwist,
    resolve_base_twist_target,
    run_base_twist_session,
)
from sim.runtime.coordinator.mujoco_process import MujocoProcess

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "omni_cart_controlled_headless"
    / "session.yaml"
)
MUJOCO_HOST = (
    REPO_ROOT
    / "build"
    / "mujoco-runtime-physics-win"
    / "Release"
    / "lingtu_mujoco_headless.exe"
)
HEADLESS_RUNNER = (
    REPO_ROOT
    / "sim" / "packages" / "controllers"
    / "omni_cart"
    / "differential_drive"
    / "headless.py"
)
_CONTROLLER_RUNTIME = runpy.run_path(
    str(
        REPO_ROOT
        / "sim" / "packages" / "controllers"
        / "omni_cart"
        / "differential_drive"
        / "runtime.py"
    )
)
_create_components = _CONTROLLER_RUNTIME["create_components"]


def test_omni_cart_session_compiles_a_named_differential_drive_control_plan(
    tmp_path: Path,
) -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")

    plan = load_control_plan(bundle / "control.plan.json")

    assert len(plan.controllers) == 1
    controller = plan.controllers[0]
    assert controller.controller_id == "cart_01.omni_cart_differential_drive"
    assert controller.instance_id == "cart_01"
    assert controller.adapter.plugin == "differential_drive_wheel_torque"
    assert controller.policy.runtime == "analytic"
    assert controller.actuators.channels == (
        "left_wheel_joint",
        "right_wheel_joint",
    )
    assert {
        plan.command_channel(channel_id).command_type
        for channel_id in controller.command_channels
    } == {"base_twist", "joint_torque"}


def test_controller_runtime_maps_forward_twist_to_equal_bounded_wheel_torque(
    tmp_path: Path,
) -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")
    plan = load_control_plan(bundle / "control.plan.json")
    controller = plan.controllers[0]
    generation = GenerationStamp(model_generation=1, reset_generation=0)
    adapter, policy = create_production_components(controller, REPO_ROOT)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id=controller.controller_id,
        generation=generation,
        adapter=adapter,
        policy=policy,
    )
    command_channel = next(
        plan.command_channel(channel_id)
        for channel_id in controller.command_channels
        if plan.command_channel(channel_id).direction == "subscribe"
    )
    command = ControllerCommand(
        channel_id=command_channel.channel_id,
        instance_id=controller.instance_id,
        generation=generation,
        sequence=1,
        apply_time_ns=0,
        payload={"linear_x": 0.44, "linear_y": 0.0, "angular_z": 0.0},
    )

    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED
    result = runtime.step(
        ControllerState(
            session_id=plan.session_id,
            instance_id=controller.instance_id,
            generation=generation,
            sequence=1,
            sim_time_ns=0,
            channels={
                "joint_position": (0.0, 0.0),
                "joint_velocity": (0.0, 0.0),
            },
        )
    )

    assert result.inference_ran is True
    assert result.actuator_command is not None
    assert result.actuator_command.command_type == "joint_torque"
    assert result.actuator_command.channels == (
        "left_wheel_joint",
        "right_wheel_joint",
    )
    assert result.actuator_command.values == (10.0, 10.0)
    assert result.actuator_command.safe_stop is False


def test_controller_runtime_rejects_lateral_velocity_as_nonholonomic(
    tmp_path: Path,
) -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")
    plan = load_control_plan(bundle / "control.plan.json")
    controller = plan.controllers[0]
    generation = GenerationStamp(model_generation=1, reset_generation=0)
    adapter, policy = _create_components(controller, REPO_ROOT)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id=controller.controller_id,
        generation=generation,
        adapter=adapter,
        policy=policy,
    )
    command_channel = next(
        plan.command_channel(channel_id)
        for channel_id in controller.command_channels
        if plan.command_channel(channel_id).direction == "subscribe"
    )
    command = ControllerCommand(
        channel_id=command_channel.channel_id,
        instance_id=controller.instance_id,
        generation=generation,
        sequence=1,
        apply_time_ns=0,
        payload={"linear_x": 0.0, "linear_y": 0.1, "angular_z": 0.0},
    )
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED

    with pytest.raises(
        ControllerRuntimeError,
        match="differential drive does not support non-zero linear_y",
    ):
        runtime.step(
            ControllerState(
                session_id=plan.session_id,
                instance_id=controller.instance_id,
                generation=generation,
                sequence=1,
                sim_time_ns=0,
                channels={
                    "joint_position": (0.0, 0.0),
                    "joint_velocity": (0.0, 0.0),
                },
            )
        )


@pytest.mark.skipif(not MUJOCO_HOST.is_file(), reason="native MuJoCo host is not built")
def test_real_headless_session_drives_omni_cart_forward(
    tmp_path: Path,
) -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=MujocoProcess(MUJOCO_HOST),
        controller_factory=create_production_components,
        run_id="omni-cart-forward-test",
    )
    target = resolve_base_twist_target(bundle, None)

    snapshot = run_base_twist_session(
        coordinator,
        target,
        BaseTwist(linear_x=0.4),
        steps=1_000,
        refresh_steps=20,
    )

    base = next(
        body for body in snapshot["bodies"] if body["stable_id"] == "cart_01/base_link"
    )
    actuators = {
        actuator["stable_id"]: actuator["control"]
        for actuator in snapshot["actuators"]
    }
    assert snapshot["physics_step"] == 1_000
    assert base["position_m"][0] > 0.10, {
        "base": base,
        "joints": snapshot["joints"],
        "actuators": actuators,
    }
    assert base["linear_velocity_mps"][0] > 0.05
    assert abs(actuators["cart_01/left_wheel_joint"]) > 0.01
    assert abs(actuators["cart_01/right_wheel_joint"]) > 0.01


@pytest.mark.skipif(not MUJOCO_HOST.is_file(), reason="native MuJoCo host is not built")
def test_headless_runner_writes_qualified_control_evidence(tmp_path: Path) -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")
    run_root = tmp_path / "runs"
    run_id = "omni-cart-control-evidence"

    result = subprocess.run(
        [
            sys.executable,
            "-B",
            str(HEADLESS_RUNNER),
            str(bundle),
            "--repo-root",
            str(REPO_ROOT),
            "--run-root",
            str(run_root),
            "--mujoco-host",
            str(MUJOCO_HOST),
            "--run-id",
            run_id,
            "--steps",
            "1000",
            "--refresh-steps",
            "20",
            "--linear-x",
            "0.4",
            "--minimum-displacement-m",
            "0.1",
            "--minimum-speed-mps",
            "0.05",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    evidence_path = run_root / run_id / "omni-cart-control-evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    assert evidence["schema"] == "lingtu.sim.omni-cart-control-evidence.v1"
    assert evidence["session_id"] == resolved.session_id
    assert evidence["controller"] == {
        "adapter_plugin": "differential_drive_wheel_torque",
        "controller_id": "cart_01.omni_cart_differential_drive",
        "controller_model": "two_wheel_differential_drive",
        "holonomic": False,
        "linear_y_supported": False,
        "output_command_type": "joint_torque",
        "policy_runtime": "analytic",
    }
    assert evidence["qualified"] is True
    assert evidence["result"]["displacement_m"] > 0.1
    assert evidence["result"]["base_speed_mps"] > 0.05
    assert evidence["result"]["physics_step"] == 1_000
    assert set(evidence["result"]["wheel_torque_nm"]) == {
        "cart_01/left_wheel_joint",
        "cart_01/right_wheel_joint",
    }
    episode = json.loads(
        (run_root / run_id / "episode_result.json").read_text(encoding="utf-8")
    )
    assert episode["status"] == "SUCCEEDED"
