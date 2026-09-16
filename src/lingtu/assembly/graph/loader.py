"""Load Runtime Graph YAML contracts."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from message.catalog import load_topics
from runtime.yaml_helpers import load_yaml

RUNTIME_GRAPH_DIR = Path(__file__).resolve().parents[4] / "config" / "runtime_graph"
PRODUCT_HOST_CAPABILITIES = frozenset(
    {
        "gateway",
        "operator_motion",
        "navigation_skills",
        "goal_commands",
        "semantic",
        "semantic_planning",
        "inspection_evidence",
        "exploration_adapter",
    }
)
PRODUCT_HOST_FIELDS = frozenset(
    {
        "capabilities",
        "run_startup_checks",
        "encoder",
        "llm",
        "inspection_evidence_max_rgb_odom_skew_s",
        "map_artifact_gate_required",
    }
)


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
    entries = load_topics()
    topics = {
        "schema_version": "lingtu.runtime_graph.topics.v2",
        "topics": {spec["topic"]: spec for spec in entries},
        "native_contract_topics": [spec["topic"] for spec in entries if spec.get("native_contract") is True],
    }
    products = _load_named_dir(graph_root / "products")
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
    selected_variant = default_variant if product_variant is None else str(product_variant).strip()
    if not selected_variant or selected_variant not in variants:
        available = ", ".join(sorted(str(name) for name in variants))
        raise ValueError(f"Product {product!r} has no variant {selected_variant!r}; available: {available}")
    variant = variants[selected_variant]
    if not isinstance(variant, Mapping):
        raise ValueError(f"Product {product!r} variant {selected_variant!r} must be a mapping")

    resolved = {
        str(key): value for key, value in spec.items() if key not in {"default_variant", "product_variant", "variants"}
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


def product_requirements(product: str, spec: Mapping[str, Any]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Validate topics/capabilities of an already resolved Product; never select a variant."""
    return (
        _names(spec.get("topics"), owner=f"Product {product!r} topics"),
        _names(spec.get("capabilities"), owner=f"Product {product!r} capabilities"),
    )


def _names(value: Any, *, owner: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"{owner} must be a list")
    names = tuple(value)
    if (
        not names
        or any(not isinstance(name, str) or not name or name != name.strip() for name in names)
        or len(set(names)) != len(names)
    ):
        raise ValueError(f"{owner} contains invalid or duplicate names")
    return names
