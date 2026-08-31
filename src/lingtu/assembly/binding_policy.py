"""Runtime binding policy for endpoint transports, adapters, and backends.

Blueprint stacks add Modules. Adapter modules implement external protocols.
This L1 runtime model owns the small set of rules that decides which runtime
binding family a resolved config is asking for.
"""

from __future__ import annotations

from typing import Any, Mapping


def _string_value(config: Mapping[str, Any], *keys: str) -> str:
    for key in keys:
        value = config.get(key)
        if value:
            return str(value).strip()
    return ""


def endpoint_transport_for_config(
    config: Mapping[str, Any],
    *,
    default: str = "local",
) -> str:
    """Return the external endpoint transport."""
    return _string_value(config, "endpoint_transport", "_endpoint_transport") or default


def endpoint_contract_for_config(
    config: Mapping[str, Any],
    *,
    default: str = "",
) -> str:
    """Return the external endpoint contract."""
    return _string_value(config, "endpoint_contract", "_endpoint_contract") or default


def localization_adapter_for_config(
    config: Mapping[str, Any],
) -> str:
    """Return the explicitly selected Host localization adapter."""
    return _string_value(config, "localization_adapter", "_localization_adapter")


def exploration_backend_for_config(config: Mapping[str, Any]) -> str:
    """Return the selected exploration backend name."""

    return _adapter_name(config.get("exploration_backend") or "none")


def _adapter_name(value: Any) -> str:
    return str(value or "").strip().lower()
