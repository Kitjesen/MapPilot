"""Operator Product lifecycle metadata from the Runtime Graph."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from types import MappingProxyType
from typing import Any, Literal, cast

from lingtu.assembly.graph.loader import load_runtime_graph, resolve_product_variant_spec

_PRODUCT_DEFINITIONS = load_runtime_graph().products
_PRODUCT_NAMES = frozenset(_PRODUCT_DEFINITIONS)
ProductName = Literal.__getitem__(tuple(sorted(_PRODUCT_NAMES)))


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

    @classmethod
    def from_spec(cls, name: ProductName, product: Mapping[str, Any]) -> ProductLifecycle:
        """Read lifecycle values from the supplied resolved Product declaration."""
        return cls(
            product=name,
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

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready lifecycle data."""

        return asdict(self)


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


OPERATOR_PRODUCT_LIFECYCLES: Mapping[str, ProductLifecycle] = MappingProxyType(
    {
        name: ProductLifecycle.from_spec(
            product_name(name),
            resolve_product_variant_spec(name, product),
        )
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
    return ProductLifecycle.from_spec(product, resolved)


__all__ = [
    "OPERATOR_PRODUCT_LIFECYCLES",
    "ProductLifecycle",
    "ProductName",
    "product_lifecycle",
    "product_name",
]
