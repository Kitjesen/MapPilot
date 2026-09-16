"""Load the message-owned topic catalogue for generation and runtime wiring."""

from pathlib import Path
from typing import Any

import yaml

CATALOG_DIR = Path(__file__).resolve().with_name("topics")


def load_topics() -> list[dict[str, Any]]:
    """Load and validate every domain catalogue in stable order."""

    topics: list[dict[str, Any]] = []
    names: set[str] = set()
    paths: set[str] = set()
    for path in sorted(CATALOG_DIR.glob("*.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for name, raw in (data.get("topics") or {}).items():
            spec = {"name": str(name), **dict(raw)}
            topic = str(spec.get("topic") or "")
            if not topic.startswith("/"):
                raise ValueError(f"{path}: {name} has invalid topic {topic!r}")
            if name in names or topic in paths:
                raise ValueError(f"{path}: duplicate topic {name!r} / {topic!r}")
            transport = str(spec.get("transport") or "dds")
            if transport == "dds":
                for field in ("dds_topic", "message_type", "qos_profile", "cpp_name"):
                    if not spec.get(field):
                        raise ValueError(f"{path}: {name} is missing {field}")
            elif transport != "local":
                raise ValueError(f"{path}: {name} has invalid transport {transport!r}")
            names.add(str(name))
            paths.add(topic)
            topics.append(spec)
    if not topics:
        raise ValueError(f"no topics found in {CATALOG_DIR}")
    return sorted(topics, key=lambda item: item["name"])
