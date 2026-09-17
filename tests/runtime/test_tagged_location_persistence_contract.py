from __future__ import annotations

import json
from pathlib import Path

import pytest

from lingtu.assembly.stacks.memory import memory
from memory.spatial.tagged_locations import TaggedLocationStore


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
    with pytest.raises(OSError, match="interrupted write"):
        store.tag("pump", x=3.0, y=4.0)

    assert path.read_text(encoding="utf-8") == original
    assert json.loads(original)[0]["name"] == "dock"
    assert store.query("pump") is None
    assert list(tmp_path.glob("*.tmp")) == []


@pytest.mark.parametrize("operation", ["update", "remove"])
def test_failed_location_replace_preserves_memory_and_file(tmp_path, monkeypatch, operation):
    import os

    path = tmp_path / "tags.json"
    store = TaggedLocationStore(str(path))
    store.tag("dock", x=1.0, y=2.0)
    original = path.read_bytes()

    def fail_replace(*args):
        raise OSError("disk write failed")

    monkeypatch.setattr(os, "replace", fail_replace)
    with pytest.raises(OSError, match="disk write failed"):
        if operation == "update":
            store.tag("dock", x=10.0, y=20.0)
        else:
            store.remove("dock")

    assert store.query("dock")["position"] == [1.0, 2.0, 0.0]
    assert path.read_bytes() == original
    assert TaggedLocationStore(str(path)).list_all() == store.list_all()
    assert list(tmp_path.glob("*.tmp")) == []
