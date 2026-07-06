"""Load Runtime Graph YAML contracts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from runtime.yaml_helpers import load_yaml


RUNTIME_GRAPH_DIR = Path(__file__).resolve().parents[3] / "config" / "runtime_graph"


@dataclass(frozen=True)
class RuntimeGraph:
    """In-memory view of the Runtime Graph contract files."""

    root: Path
    topics: dict[str, Any]
    products: dict[str, dict[str, Any]]
    endpoints: dict[str, dict[str, Any]]

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
    products = _load_named_dir(graph_root / "products")
    endpoints = _load_named_dir(graph_root / "endpoints")
    return RuntimeGraph(
        root=graph_root,
        topics=topics,
        products=products,
        endpoints=endpoints,
    )


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
