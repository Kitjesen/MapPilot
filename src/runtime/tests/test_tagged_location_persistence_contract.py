from __future__ import annotations

import json
from pathlib import Path

from memory.spatial.tagged_locations import TaggedLocationStore
from runtime.blueprints.stacks.memory import memory


def test_memory_stack_persists_tagged_locations_under_semantic_directory(tmp_path: Path) -> None:
    blueprint = memory(save_dir=str(tmp_path))

    tagged = next(
        entry
        for entry in blueprint._entries
        if entry.alias == "TaggedLocationsModule"
    )

    assert tagged.config["json_path"] == str(tmp_path / "tagged_locations.json")


def test_tagged_location_save_replaces_file_atomically(
    tmp_path: Path,
    monkeypatch,
) -> None:
    path = tmp_path / "tagged_locations.json"
    store = TaggedLocationStore(json_path=str(path))
    store.tag("dock", x=1.0, y=2.0)
    original = path.read_text(encoding="utf-8")

    def fail_after_partial_write(data, stream, **kwargs):
        stream.write("[")
        raise OSError("simulated interrupted write")

    monkeypatch.setattr(json, "dump", fail_after_partial_write)
    store.tag("pump", x=3.0, y=4.0)

    assert path.read_text(encoding="utf-8") == original
    assert json.loads(original)[0]["name"] == "dock"
