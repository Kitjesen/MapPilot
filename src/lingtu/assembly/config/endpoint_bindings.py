"""Select endpoint transports, contracts, and localization adapters for Host assembly."""

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
