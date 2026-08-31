"""Operator Product lifecycle metadata from the Runtime Graph."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Literal, cast, get_args

from runtime.graph.loader import load_runtime_graph, resolve_product_variant_spec

ProductName = Literal[
    "teleop",
    "teleop_avoid",
    "map",
    "explore",
    "nav",
    "tracking",
    "inspection",
]
_PRODUCT_NAMES: frozenset[str] = frozenset(get_args(ProductName))


@dataclass(frozen=True)
class ProductLifecycle:
    """Lifecycle policy for one operator-switchable Product."""

    product: ProductName
    label: str
    session_mode: str
    native_control_mode: str
    slam_mode: str
    requires_map: bool
    switch_policy: str
    product_variant: str | None = None
    default_for_session_mode: bool = False
    online_hot_switch_supported: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready lifecycle data."""

        return {
            "product": self.product,
            "label": self.label,
            "session_mode": self.session_mode,
            "native_control_mode": self.native_control_mode,
            "slam_mode": self.slam_mode,
            "requires_map": self.requires_map,
            "switch_policy": self.switch_policy,
            "product_variant": self.product_variant,
            "default_for_session_mode": self.default_for_session_mode,
            "online_hot_switch_supported": self.online_hot_switch_supported,
        }


def product_name(value: str) -> ProductName:
    """Validate and narrow one canonical Runtime Graph Product name."""

    if value not in _PRODUCT_NAMES:
        raise ValueError(f"Unknown Product: {value}")
    return cast(ProductName, value)


def _required_text(product: Mapping[str, Any], field: str) -> str:
    value = str(product.get(field) or "").strip()
    if not value:
        name = str(product.get("name") or "unknown")
        raise ValueError(f"Product {name!r} is missing lifecycle field {field}")
    return value


def _lifecycle_from_product(name: str, product: Mapping[str, Any]) -> ProductLifecycle:
    return ProductLifecycle(
        product=product_name(name),
        label=str(product.get("label") or name.replace("_", " ").title()),
        session_mode=_required_text(product, "session_mode"),
        native_control_mode=_required_text(product, "native_control_mode"),
        slam_mode=_required_text(product, "slam_mode"),
        requires_map=bool(product.get("requires_map", False)),
        switch_policy=_required_text(product, "switch_policy"),
        product_variant=(
            str(product["product_variant"]).strip()
            if product.get("product_variant") is not None
            else None
        ),
        default_for_session_mode=bool(product.get("default_for_session_mode", False)),
        online_hot_switch_supported=bool(product.get("online_hot_switch_supported", False)),
    )


_PRODUCT_DEFINITIONS = load_runtime_graph().products
OPERATOR_PRODUCT_LIFECYCLES: Mapping[str, ProductLifecycle] = MappingProxyType(
    {
        product_name(name): _lifecycle_from_product(name, product)
        for name, product in _PRODUCT_DEFINITIONS.items()
        if product.get("operator_switchable") is True
    }
)


def product_lifecycle(
    product: ProductName,
    *,
    product_variant: str | None = None,
) -> ProductLifecycle:
    """Return lifecycle policy for an operator-switchable Product."""

    if product_variant is None:
        return OPERATOR_PRODUCT_LIFECYCLES[product]
    resolved = resolve_product_variant_spec(
        product,
        _PRODUCT_DEFINITIONS[product],
        product_variant=product_variant,
    )
    return _lifecycle_from_product(product, resolved)


__all__ = [
    "OPERATOR_PRODUCT_LIFECYCLES",
    "ProductLifecycle",
    "ProductName",
    "product_lifecycle",
    "product_name",
]
