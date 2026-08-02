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
    product_mode: str
    product_session: str
    session_mode: str
    native_control_mode: str
    slam_mode: str
    requires_map: bool
    switch_policy: str
    product_variant: str | None = None
    default_for_session_mode: bool = False
    hot_switch_candidates: frozenset[ProductName] = frozenset()
    online_hot_switch_supported: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready lifecycle data."""

        return {
            "product": self.product,
            "label": self.label,
            "product_mode": self.product_mode,
            "product_session": self.product_session,
            "session_mode": self.session_mode,
            "native_control_mode": self.native_control_mode,
            "slam_mode": self.slam_mode,
            "requires_map": self.requires_map,
            "switch_policy": self.switch_policy,
            "product_variant": self.product_variant,
            "default_for_session_mode": self.default_for_session_mode,
            "hot_switch_candidates": sorted(self.hot_switch_candidates),
            "online_hot_switch_supported": self.online_hot_switch_supported,
        }


def product_name(value: str) -> ProductName:
    """Validate and narrow one canonical operator Product name."""

    if value not in _PRODUCT_NAMES:
        raise ValueError(f"Unknown Product: {value}")
    return cast(ProductName, value)


def _product_names(value: Any) -> frozenset[ProductName]:
    if value is None:
        return frozenset()
    if isinstance(value, str):
        return frozenset((product_name(value),))
    if isinstance(value, list | tuple | set | frozenset):
        return frozenset(
            product_name(str(item)) for item in value if str(item)
        )
    raise TypeError(f"Product lifecycle list must be a sequence, got {type(value).__name__}")


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
        product_mode=_required_text(product, "product_mode"),
        product_session=_required_text(product, "product_session"),
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
        hot_switch_candidates=_product_names(product.get("hot_switch_candidates")),
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


def product_transition_plan(
    current_product: str | None,
    target_product: ProductName,
) -> dict[str, Any]:
    """Describe only the lifecycle policy for a Product transition."""

    current = (
        OPERATOR_PRODUCT_LIFECYCLES[cast(ProductName, current_product)]
        if current_product in OPERATOR_PRODUCT_LIFECYCLES
        else None
    )
    target = product_lifecycle(target_product)
    same_graph_candidate = bool(
        current is not None and target_product in current.hot_switch_candidates
    )
    online_supported = bool(
        current is not None
        and current.online_hot_switch_supported
        and target.online_hot_switch_supported
        and same_graph_candidate
    )
    required_lifecycle = "hot_switch" if online_supported else target.switch_policy
    return {
        "current": (
            current.as_dict()
            if current is not None
            else {"product": None, "operator_switchable": False}
        ),
        "target": target.as_dict(),
        "same_graph_candidate": same_graph_candidate,
        "online_hot_switch_supported": online_supported,
        "required_lifecycle": required_lifecycle,
        "reason": (
            "same graph hot switch is supported"
            if online_supported
            else f"target Product requires {required_lifecycle}"
        ),
    }


__all__ = [
    "OPERATOR_PRODUCT_LIFECYCLES",
    "ProductName",
    "ProductLifecycle",
    "product_name",
    "product_lifecycle",
    "product_transition_plan",
]
