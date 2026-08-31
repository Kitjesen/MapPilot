# ruff: noqa: S101

from pathlib import Path

import pytest

from sim.runtime.control.contracts import ControllerRuntimeError
from sim.runtime.control.factory import (
    PRODUCTION_CONTROLLER_COMPONENTS,
    ControllerComponentRegistry,
)
from sim.runtime.control.plan import (
    ActuatorLayout,
    AdapterSpec,
    ControllerSpec,
    PolicySpec,
)


def _controller(
    *,
    plugin: str = "quadruped_him",
    runtime: str = "torchscript",
) -> ControllerSpec:
    return ControllerSpec(
        controller_id="thunder_01.controller",
        instance_id="thunder_01",
        adapter=AdapterSpec(
            plugin=plugin,
            abi="lingtu.sim.controller-adapter.v1",
        ),
        policy=PolicySpec(
            runtime=runtime,
            artifact="policy.pt",
            manifest="policy.json",
        ),
        inference_hz=50.0,
        low_level_hz=500.0,
        state_channels=(),
        command_channels=("command",),
        actuators=ActuatorLayout(("joint",)),
    )


def test_registry_dispatches_by_adapter_abi_plugin_and_policy_runtime(
    tmp_path: Path,
) -> None:
    calls: list[tuple[ControllerSpec, Path]] = []
    expected = (object(), object())

    def provider(controller: ControllerSpec, repo_root: Path) -> tuple[object, object]:
        calls.append((controller, repo_root))
        return expected

    registry = ControllerComponentRegistry()
    registry.register(
        adapter_abi="lingtu.sim.controller-adapter.v1",
        adapter_plugin="quadruped_him",
        policy_runtime="torchscript",
        provider=provider,
    )
    controller = _controller()

    result = registry.create(controller, tmp_path)

    assert result is expected
    assert calls == [(controller, tmp_path.resolve())]


def test_registry_rejects_unsupported_component_contract(tmp_path: Path) -> None:
    registry = ControllerComponentRegistry()

    with pytest.raises(ControllerRuntimeError, match="no controller component provider"):
        registry.create(_controller(plugin="unknown"), tmp_path)


def test_registry_rejects_duplicate_contract_registration() -> None:
    registry = ControllerComponentRegistry()

    def provider(controller: ControllerSpec, repo_root: Path) -> tuple[object, object]:
        return controller, repo_root

    registry.register(
        adapter_abi="lingtu.sim.controller-adapter.v1",
        adapter_plugin="quadruped_him",
        policy_runtime="torchscript",
        provider=provider,
    )

    with pytest.raises(ValueError, match="already registered"):
        registry.register(
            adapter_abi="lingtu.sim.controller-adapter.v1",
            adapter_plugin="quadruped_him",
            policy_runtime="torchscript",
            provider=provider,
        )


def test_production_registry_supports_shipped_controller_contracts() -> None:
    assert PRODUCTION_CONTROLLER_COMPONENTS.contracts == (
        (
            "lingtu.sim.controller-adapter.v1",
            "differential_drive_wheel_torque",
            "analytic",
        ),
        (
            "lingtu.sim.controller-adapter.v1",
            "quadruped_him",
            "onnxruntime",
        ),
        (
            "lingtu.sim.controller-adapter.v1",
            "quadruped_him",
            "torchscript",
        ),
    )
