"""Cross-check LingTu product assembly against Runtime Graph contracts."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from runtime.graph import ProcessSpec
from runtime.graph.validator import RuntimeGraphIssue


def validate_product(
    product: str,
    *,
    product_spec: Mapping[str, Any],
    implementation: Mapping[str, Any],
    config: Mapping[str, Any],
    module_names: tuple[str, ...],
    env_name: str,
    processes: tuple[ProcessSpec, ...] = (),
) -> list[RuntimeGraphIssue]:
    """Validate one already resolved Product assembly."""

    product_name = str(product)
    selected_env = str(env_name).strip()
    if selected_env not in {"real", "sim"}:
        raise ValueError(f"env_name must be 'real' or 'sim', received {selected_env!r}")
    modules = set(module_names)
    forbidden_modules = set(_string_tuple(implementation.get("forbidden_modules")))
    if selected_env == "real":
        forbidden_modules.update(_string_tuple(product_spec.get("forbidden_modules")))
    issues: list[RuntimeGraphIssue] = []

    expected = (
        ("_env", "real", "real_product_env_drift", "use the real Env"),
        (
            "localization_adapter",
            "cpp_slam_status",
            "real_product_localization_adapter_drift",
            "use cpp_slam_status localization_adapter",
        ),
        (
            "command_output_mode",
            "endpoint_only",
            "real_product_command_output_drift",
            "use endpoint_only command output",
        ),
        (
            "hardware_control_boundary",
            "driver",
            "real_product_driver_boundary_drift",
            "use the canonical driver hardware boundary",
        ),
    )
    if selected_env == "real":
        for key, value, code, requirement in expected:
            if config.get(key) != value:
                issues.append(
                    _product_issue(
                        code,
                        f"Product {product_name} must {requirement}",
                        product_name,
                    )
                )

        if config.get("enable_robot_driver") is not False:
            issues.append(
                _product_issue(
                    "real_product_duplicate_driver",
                    f"Product {product_name} must not add the Python robot driver beside the native driver",
                    product_name,
                )
            )

    required_modules = set(
        _string_tuple(
            product_spec.get("critical_modules")
            if selected_env == "real"
            else implementation.get("critical_modules")
        )
    )
    missing_required = sorted(required_modules - modules)
    if missing_required:
        issues.append(
            _product_issue(
                f"{selected_env}_product_critical_module_missing",
                f"Product {product_name} is missing critical modules: {', '.join(missing_required)}",
                product_name,
            )
        )

    forbidden_present = sorted(forbidden_modules & modules)
    if forbidden_present:
        issues.append(
            _product_issue(
                f"{selected_env}_product_forbidden_module",
                f"Product {product_name} includes forbidden modules: {', '.join(forbidden_present)}",
                product_name,
            )
        )
    return issues


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple | set):
        return tuple(str(item) for item in value)
    return ()


def _product_issue(code: str, message: str, product: str) -> RuntimeGraphIssue:
    return RuntimeGraphIssue(code=code, message=message, scope=f"product:{product}")
