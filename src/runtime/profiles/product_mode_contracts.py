"""Product mode views loaded from the Runtime Graph source of truth."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from runtime.graph.loader import load_runtime_graph


@dataclass(frozen=True)
class ProductModeContract:
    """Operator-facing mode contract shared by CLI, Gateway, and tests."""

    profile: str
    label: str
    product_mode: str
    product_session: str
    session_mode: str
    native_control_mode: str
    slam_mode: str
    requires_map: bool
    switch_policy: str
    default_for_session_mode: bool = False
    processes: frozenset[str] = frozenset()
    required_topics: frozenset[str] = frozenset()
    required_capabilities: frozenset[str] = frozenset()
    forbidden_modules: frozenset[str] = frozenset()
    native_nav: Mapping[str, Any] = MappingProxyType({})
    hot_switch_candidates: frozenset[str] = frozenset()
    online_hot_switch_supported: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return the operator-facing contract as deterministic JSON data."""

        return {
            "profile": self.profile,
            "label": self.label,
            "product_mode": self.product_mode,
            "product_session": self.product_session,
            "session_mode": self.session_mode,
            "native_control_mode": self.native_control_mode,
            "slam_mode": self.slam_mode,
            "requires_map": self.requires_map,
            "default_for_session_mode": self.default_for_session_mode,
            "processes": sorted(self.processes),
            "required_topics": sorted(self.required_topics),
            "required_capabilities": sorted(self.required_capabilities),
            "forbidden_modules": sorted(self.forbidden_modules),
            "native_nav": dict(self.native_nav),
            "switch_policy": self.switch_policy,
            "hot_switch_candidates": sorted(self.hot_switch_candidates),
            "online_hot_switch_supported": self.online_hot_switch_supported,
        }


def _strings(value: Any) -> frozenset[str]:
    if value is None:
        return frozenset()
    if isinstance(value, str):
        return frozenset((value,))
    if isinstance(value, list | tuple | set | frozenset):
        return frozenset(str(item) for item in value if str(item))
    raise TypeError(f"mode contract list must be a sequence, got {type(value).__name__}")


def _required_text(product: Mapping[str, Any], field: str) -> str:
    value = str(product.get(field) or "").strip()
    if not value:
        name = str(product.get("name") or "unknown")
        raise ValueError(f"product mode {name!r} is missing {field}")
    return value


def _contract_from_product(name: str, product: Mapping[str, Any]) -> ProductModeContract:
    native_nav = product.get("native_nav") or {}
    if not isinstance(native_nav, Mapping):
        raise TypeError(f"product mode {name!r} native_nav must be a mapping")
    native_control_mode = _required_text(product, "native_control_mode")
    native_nav_control_mode = str(native_nav.get("control_mode") or native_control_mode).strip()
    if native_nav_control_mode != native_control_mode:
        raise ValueError(
            f"product mode {name!r} native_nav.control_mode must match native_control_mode"
        )
    return ProductModeContract(
        profile=name,
        label=str(product.get("label") or name.replace("_", " ").title()),
        product_mode=_required_text(product, "product_mode"),
        product_session=_required_text(product, "product_session"),
        session_mode=_required_text(product, "session_mode"),
        native_control_mode=native_control_mode,
        slam_mode=_required_text(product, "slam_mode"),
        requires_map=bool(product.get("requires_map", False)),
        switch_policy=_required_text(product, "switch_policy"),
        default_for_session_mode=bool(product.get("default_for_session_mode", False)),
        processes=_strings(product.get("processes")),
        required_topics=_strings(product.get("required_topics")),
        required_capabilities=_strings(product.get("required_capabilities")),
        forbidden_modules=_strings(product.get("forbidden_modules")),
        native_nav=MappingProxyType(dict(native_nav)),
        hot_switch_candidates=_strings(product.get("hot_switch_candidates")),
        online_hot_switch_supported=bool(product.get("online_hot_switch_supported", False)),
    )


def _load_product_contracts(
    products: Mapping[str, Mapping[str, Any]],
) -> dict[str, ProductModeContract]:
    return {
        name: _contract_from_product(name, product)
        for name, product in products.items()
    }


_PRODUCT_DEFINITIONS = load_runtime_graph().products
PRODUCT_CONTRACTS: dict[str, ProductModeContract] = _load_product_contracts(_PRODUCT_DEFINITIONS)
PRODUCT_MODE_CONTRACTS: dict[str, ProductModeContract] = {
    name: contract
    for name, contract in PRODUCT_CONTRACTS.items()
    if _PRODUCT_DEFINITIONS[name].get("operator_switchable") is True
}


def product_mode_contract(profile: str) -> ProductModeContract:
    """Return the operator-switchable contract for *profile*."""

    return PRODUCT_MODE_CONTRACTS[profile]


def product_mode_switch_plan(
    current_profile: str,
    target_profile: str,
    *,
    product: Mapping[str, Any] | None = None,
    native_nav_config: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Describe a transition using an already compiled Product.

    Mode contracts own lifecycle semantics only. Product assembly owns
    compilation, so this view must never compile a second Product behind the
    caller's back.
    """

    target = product_mode_contract(target_profile)
    current = PRODUCT_MODE_CONTRACTS.get(current_profile)
    same_graph_candidate = bool(current and target_profile in current.hot_switch_candidates)
    online_supported = (
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
            else {"profile": current_profile, "operator_switchable": False}
        ),
        "target": target.as_dict(),
        "same_graph_candidate": same_graph_candidate,
        "online_hot_switch_supported": online_supported,
        "product": dict(product) if product is not None else None,
        "native_nav_config": dict(native_nav_config) if native_nav_config is not None else None,
        "required_lifecycle": required_lifecycle,
        "reason": (
            "same graph hot switch is supported"
            if online_supported
            else f"target mode requires {required_lifecycle}"
        ),
    }
