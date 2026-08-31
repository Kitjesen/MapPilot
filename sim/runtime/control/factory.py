"""Contract-keyed construction of production controller components."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from .contracts import ControllerAdapter, ControllerPolicy, ControllerRuntimeError
from .plan import ControllerSpec
from .thunderv4 import create_thunderv4_components, create_thunderv4_onnx_components

ControllerComponentProvider = Callable[
    [ControllerSpec, Path], tuple[ControllerAdapter, ControllerPolicy]
]
ControllerContractKey = tuple[str, str, str]


def _create_differential_drive_components(
    controller: ControllerSpec,
    repo_root: Path,
) -> tuple[ControllerAdapter, ControllerPolicy]:
    """Load the optional analytic adapter only after the control package is initialized."""

    from sim.packages.controllers.omni_cart_differential_drive.runtime import (
        create_components,
    )

    return create_components(controller, repo_root)


def _contract_value(value: str, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field} must be a non-empty trimmed string")
    return value


class ControllerComponentRegistry:
    """Map compiled controller contracts to product-independent providers."""

    def __init__(self) -> None:
        self._providers: dict[ControllerContractKey, ControllerComponentProvider] = {}

    def register(
        self,
        *,
        adapter_abi: str,
        adapter_plugin: str,
        policy_runtime: str,
        provider: ControllerComponentProvider,
    ) -> None:
        """Register exactly one implementation for a compiled contract tuple."""

        if not callable(provider):
            raise TypeError("provider must be callable")
        key = (
            _contract_value(adapter_abi, "adapter_abi"),
            _contract_value(adapter_plugin, "adapter_plugin"),
            _contract_value(policy_runtime, "policy_runtime"),
        )
        if key in self._providers:
            raise ValueError(f"controller component contract is already registered: {key!r}")
        self._providers[key] = provider

    @property
    def contracts(self) -> tuple[ControllerContractKey, ...]:
        """Return all registered contracts in deterministic order."""

        return tuple(sorted(self._providers))

    def create(
        self,
        controller: ControllerSpec,
        repo_root: Path,
    ) -> tuple[ControllerAdapter, ControllerPolicy]:
        """Construct components selected only by the compiled controller contract."""

        if not isinstance(controller, ControllerSpec):
            raise TypeError("controller must be a compiled ControllerSpec")
        key = (
            controller.adapter.abi,
            controller.adapter.plugin,
            controller.policy.runtime,
        )
        try:
            provider = self._providers[key]
        except KeyError as exc:
            raise ControllerRuntimeError(
                "no controller component provider for "
                f"adapter_abi={key[0]!r}, adapter_plugin={key[1]!r}, "
                f"policy_runtime={key[2]!r}"
            ) from exc
        return provider(controller, Path(repo_root).resolve())


PRODUCTION_CONTROLLER_COMPONENTS = ControllerComponentRegistry()
PRODUCTION_CONTROLLER_COMPONENTS.register(
    adapter_abi="lingtu.sim.controller-adapter.v1",
    adapter_plugin="quadruped_him",
    policy_runtime="torchscript",
    provider=create_thunderv4_components,
)
PRODUCTION_CONTROLLER_COMPONENTS.register(
    adapter_abi="lingtu.sim.controller-adapter.v1",
    adapter_plugin="quadruped_him",
    policy_runtime="onnxruntime",
    provider=create_thunderv4_onnx_components,
)
PRODUCTION_CONTROLLER_COMPONENTS.register(
    adapter_abi="lingtu.sim.controller-adapter.v1",
    adapter_plugin="differential_drive_wheel_torque",
    policy_runtime="analytic",
    provider=_create_differential_drive_components,
)


def create_production_components(
    controller: ControllerSpec,
    repo_root: Path,
) -> tuple[ControllerAdapter, ControllerPolicy]:
    """Construct the production implementation declared by ``controller``."""

    return PRODUCTION_CONTROLLER_COMPONENTS.create(controller, repo_root)


__all__ = [
    "PRODUCTION_CONTROLLER_COMPONENTS",
    "ControllerComponentProvider",
    "ControllerComponentRegistry",
    "ControllerContractKey",
    "create_production_components",
]
