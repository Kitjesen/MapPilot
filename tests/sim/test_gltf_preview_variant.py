"""Preview one independent tree without importing a whole source grove."""

import json
from pathlib import Path


def test_stage_selects_one_root_and_keeps_original_resource_files(tmp_path):
    from sim.tools.assets.gltf_preview_variant import stage_mesh_variant

    source_dir = tmp_path / "source"
    source_dir.mkdir()
    source = source_dir / "tree.gltf"
    original = {
        "asset": {"version": "2.0"},
        "nodes": [
            {"name": "tree_a", "mesh": 0, "translation": [-6, 0, 0]},
            {"name": "tree_b", "mesh": 1, "translation": [1, 0, 0]},
        ],
        "scenes": [{"nodes": [0, 1]}],
        "scene": 0,
        "meshes": [
            {"primitives": [{"attributes": {"POSITION": 0}}]},
            {"primitives": [{"attributes": {"POSITION": 1}}]},
        ],
        "accessors": [{"count": 200}, {"count": 100}],
        "buffers": [{"uri": "tree.bin"}],
        "images": [{"uri": "bark.jpg"}],
    }
    source.write_text(json.dumps(original), encoding="utf-8")
    output = tmp_path / "preview" / "single.gltf"

    report = stage_mesh_variant(source, output, node_name="tree_b")

    staged = json.loads(output.read_text(encoding="utf-8"))
    assert staged["nodes"] == [{"name": "tree_b", "mesh": 0, "translation": [0, 0, 0]}]
    assert staged["scenes"] == [{"nodes": [0]}]
    assert len(staged["meshes"]) == 1
    assert (output.parent / Path(staged["buffers"][0]["uri"])).resolve() == source_dir / "tree.bin"
    assert (output.parent / Path(staged["images"][0]["uri"])).resolve() == source_dir / "bark.jpg"
    assert json.loads(source.read_text(encoding="utf-8")) == original
    assert report["selected_vertices"] == 100
