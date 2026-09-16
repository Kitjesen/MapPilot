"""Load runtime route contracts."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from message.catalog import load_topics
from runtime.yaml_helpers import load_yaml

from .model import RouteContract, RouteSpec, TopicContract
from .routes import route_preset

REPO_ROOT = Path(__file__).resolve().parents[3]
ROUTE_CONFIG_DIR = REPO_ROOT / "config" / "routes"


def load_route(name: str = "robot", *, root: str | Path | None = None) -> RouteSpec:
    """Load a built-in route preset or a custom route table.

    Product route presets live in code so protocol choices stay close to typed
    message contracts. YAML route files are only for external/custom overlays.
    """

    if root is None:
        try:
            return route_preset(name)
        except KeyError:
            pass

    route_root = Path(root) if root is not None else ROUTE_CONFIG_DIR
    path = route_root / "routes" / f"{name}.yaml"
    if not path.exists():
        raise KeyError(f"Unknown route {name!r}; no preset or {path}")
    data = load_yaml(path, default={})
    if not isinstance(data, Mapping):
        data = {}
    routes = data.get("routes") or {}
    bindings = data.get("bindings") or {}
    return RouteSpec(
        name=str(data.get("name") or name),
        description=str(data.get("description") or ""),
        default=str(data.get("default") or "local").strip().lower(),
        routes={str(topic): str(backend).strip().lower() for topic, backend in dict(routes).items()},
        bindings=_normalize_bindings(bindings),
    )


def load_route_contract(
    route: str | RouteSpec = "robot",
    *,
    name: str = "runtime",
    route_root: str | Path | None = None,
) -> RouteContract:
    """Load the runtime topic contract with one route selected.

    The canonical topic ownership source is the message catalogue.
    Routes are separate so DDS, LCM, SHM, ROS2, and local delivery can
    be selected without duplicating topic ownership.
    """

    entries = load_topics()
    route_spec = load_route(route, root=route_root) if isinstance(route, str) else route
    topics = {spec["topic"]: TopicContract.from_mapping(spec["topic"], spec) for spec in entries}
    return RouteContract(
        name=name,
        topics=topics,
        route=route_spec,
        native_contract_topics=tuple(spec["topic"] for spec in entries if spec.get("native_contract") is True),
    )


def _normalize_bindings(value: Any) -> dict[str, dict[str, dict[str, Any]]]:
    if not isinstance(value, Mapping):
        return {}
    normalized: dict[str, dict[str, dict[str, Any]]] = {}
    for backend, topic_bindings in value.items():
        if not isinstance(topic_bindings, Mapping):
            continue
        normalized[str(backend).strip().lower()] = {
            str(topic): dict(binding) if isinstance(binding, Mapping) else {}
            for topic, binding in topic_bindings.items()
        }
    return normalized
