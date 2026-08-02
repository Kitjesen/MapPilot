"""Load Runtime Graph YAML contracts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from collections.abc import Mapping
from typing import Any

from runtime.yaml_helpers import load_yaml


RUNTIME_GRAPH_DIR = Path(__file__).resolve().parents[3] / "config" / "runtime_graph"


@dataclass(frozen=True)
class RuntimeGraph:
    """In-memory view of the Runtime Graph contract files."""

    root: Path
    topics: dict[str, Any]
    products: dict[str, dict[str, Any]]
    envs: dict[str, dict[str, Any]]

    @property
    def topic_contracts(self) -> dict[str, dict[str, Any]]:
        topics = self.topics.get("topics", {})
        return topics if isinstance(topics, dict) else {}

    @property
    def native_contract_topics(self) -> tuple[str, ...]:
        topics = self.topics.get("native_contract_topics", ())
        if not isinstance(topics, list | tuple):
            return ()
        return tuple(str(topic) for topic in topics)


def load_runtime_graph(root: str | Path | None = None) -> RuntimeGraph:
    """Load Runtime Graph YAML files from *root* or the repo default."""

    graph_root = Path(root) if root is not None else RUNTIME_GRAPH_DIR
    topics = _load_mapping(graph_root / "topics.yaml")
    products = {
        name: _with_default_product_variant(name, product)
        for name, product in _load_named_dir(graph_root / "products").items()
    }
    envs = _load_named_dir(graph_root / "envs")
    return RuntimeGraph(
        root=graph_root,
        topics=topics,
        products=products,
        envs=envs,
    )


def resolve_product_variant_spec(
    product: str,
    spec: Mapping[str, Any],
    *,
    product_variant: str | None = None,
) -> dict[str, Any]:
    """Resolve one Product variant without changing the public Product name."""

    variants = spec.get("variants")
    if variants is None:
        if product_variant is not None:
            raise ValueError(f"Product {product!r} does not declare variants")
        return dict(spec)
    if not isinstance(variants, Mapping) or not variants:
        raise ValueError(f"Product {product!r} variants must be a non-empty mapping")

    default_variant = str(spec.get("default_variant") or "").strip()
    selected_variant = (
        default_variant
        if product_variant is None
        else str(product_variant).strip()
    )
    if not selected_variant or selected_variant not in variants:
        available = ", ".join(sorted(str(name) for name in variants))
        raise ValueError(
            f"Product {product!r} has no variant {selected_variant!r}; "
            f"available: {available}"
        )
    variant = variants[selected_variant]
    if not isinstance(variant, Mapping):
        raise ValueError(
            f"Product {product!r} variant {selected_variant!r} must be a mapping"
        )

    resolved = {
        str(key): value
        for key, value in spec.items()
        if key not in {"default_variant", "product_variant", "variants"}
    }
    resolved.update({str(key): value for key, value in variant.items()})
    resolved["product_variant"] = selected_variant
    return resolved


def product_variant_names(spec: Mapping[str, Any]) -> tuple[str, ...]:
    """Return the declared Product variant names in deterministic order."""

    variants = spec.get("variants")
    if variants is None:
        return ()
    if not isinstance(variants, Mapping):
        return ()
    return tuple(sorted(str(name) for name in variants))


def _load_mapping(path: Path) -> dict[str, Any]:
    data = load_yaml(path, default={})
    return data if isinstance(data, dict) else {}


def _with_default_product_variant(
    product: str,
    spec: dict[str, Any],
) -> dict[str, Any]:
    if "variants" not in spec:
        return spec
    resolved = resolve_product_variant_spec(product, spec)
    materialized = dict(spec)
    materialized.update(resolved)
    return materialized


def _load_named_dir(path: Path) -> dict[str, dict[str, Any]]:
    items: dict[str, dict[str, Any]] = {}
    if not path.exists():
        return items
    for file_path in sorted(path.glob("*.yaml")):
        data = _load_mapping(file_path)
        name = str(data.get("name") or file_path.stem)
        data.setdefault("name", name)
        data.setdefault("_path", str(file_path))
        items[name] = data
    return items
