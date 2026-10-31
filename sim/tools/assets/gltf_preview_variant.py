"""Stage a single static glTF mesh variant without copying its large buffers."""

from __future__ import annotations

import json
import os
from pathlib import Path


def stage_mesh_variant(source: Path, destination: Path, *, node_name: str) -> dict:
    document = json.loads(source.read_text(encoding="utf-8"))
    node = next(node for node in document["nodes"] if node.get("name") == node_name)
    if node.get("children") or "skin" in node or "matrix" in node:
        raise ValueError("Preview selection requires one independent static mesh root")
    mesh = document["meshes"][node["mesh"]]
    node["mesh"] = 0
    node["translation"] = [0, 0, 0]
    document["nodes"] = [node]
    document["meshes"] = [mesh]
    document["scenes"] = [{"nodes": [0]}]
    document["scene"] = 0
    for item in [*document.get("buffers", []), *document.get("images", [])]:
        uri = item.get("uri")
        if uri and not uri.startswith("data:"):
            resource = source.parent / uri
            item["uri"] = Path(os.path.relpath(resource, destination.parent)).as_posix()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
    return {
        "source": str(source),
        "variant": node_name,
        "selected_roots": 1,
        "selected_vertices": sum(
            document["accessors"][p["attributes"]["POSITION"]]["count"] for p in mesh["primitives"]
        ),
        "lod_reduced": False,
    }
