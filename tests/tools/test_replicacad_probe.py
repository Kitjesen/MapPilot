from __future__ import annotations

import importlib.util
import json
import struct
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "datasets" / "replicacad_probe.py"
SPEC = importlib.util.spec_from_file_location("replicacad_probe", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_minimal_glb(path: Path, minimum: list[float], maximum: list[float]) -> None:
    document = {
        "asset": {"version": "2.0"},
        "accessors": [
            {
                "componentType": 5126,
                "count": 8,
                "type": "VEC3",
                "min": minimum,
                "max": maximum,
            }
        ],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "nodes": [{"mesh": 0}],
    }
    encoded = json.dumps(document, separators=(",", ":")).encode("utf-8")
    encoded += b" " * ((4 - len(encoded) % 4) % 4)
    path.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(encoded))
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
    )


def _write_hierarchical_glb(path: Path) -> None:
    """Write a bounds-only GLB with the same root/child TRS shape as ReplicaCAD."""
    half_turn = 2**-0.5
    document = {
        "asset": {"version": "2.0"},
        "accessors": [
            {
                "componentType": 5126,
                "count": 8,
                "type": "VEC3",
                "min": [0, 0, 0],
                "max": [1, 2, 3],
            }
        ],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "nodes": [
            {"children": [1], "rotation": [half_turn, 0, 0, half_turn]},
            {"mesh": 0, "rotation": [half_turn, 0, 0, half_turn]},
        ],
        "scenes": [{"nodes": [0]}],
    }
    encoded = json.dumps(document, separators=(",", ":")).encode("utf-8")
    encoded += b" " * ((4 - len(encoded) % 4) % 4)
    path.write_bytes(
        struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(encoded))
        + struct.pack("<II", len(encoded), 0x4E4F534A)
        + encoded
    )


def test_glb_bounds_apply_scene_root_transform() -> None:
    # Two +90° X rotations compose to 180°.  Ignoring the scene root would
    # report y=[0, 3], z=[0, 2]; the complete glTF transform reports y=[-2, 0]
    # and z=[-3, 0].
    import tempfile

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "hierarchical.glb"
        _write_hierarchical_glb(path)
        lower, upper, vertices = MODULE._glb_bounds(path)

    assert vertices == 8
    assert lower == pytest.approx([0.0, -2.0, -3.0])
    assert upper == pytest.approx([1.0, 0.0, 0.0])


def test_floor_triangle_sampling_fills_sparse_face() -> None:
    document = {
        "accessors": [
            {"bufferView": 0, "byteOffset": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
            {"bufferView": 1, "byteOffset": 0, "componentType": 5123, "count": 3, "type": "SCALAR"},
        ],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": 36},
            {"buffer": 0, "byteOffset": 36, "byteLength": 6},
        ],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
        "nodes": [{"mesh": 0, "name": "frl_apartment_floor"}],
    }
    binary = struct.pack(
        "<9f3H",
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0,
        1,
        2,
    )

    samples = MODULE._sample_floor_triangles(document, binary, 0, MODULE._node_matrix(document["nodes"][0]), 0.25)

    assert len(samples) > 3
    assert all(point[2] == pytest.approx(0.0) for point in samples)


def test_replicacad_scene_binds_geometry_and_converts_to_native_frame(tmp_path: Path) -> None:
    dataset = tmp_path / "replicacad"
    (dataset / "configs" / "objects").mkdir(parents=True)
    (dataset / "configs" / "scenes").mkdir(parents=True)
    (dataset / "configs" / "ssd").mkdir(parents=True)
    (dataset / "objects").mkdir()
    (dataset / "navmeshes").mkdir()

    _write_minimal_glb(dataset / "objects" / "frl_apartment_table.glb", [0, 0, 0], [2, 1, 3])
    _write_minimal_glb(dataset / "objects" / "frl_apartment_box.glb", [0, 0, 0], [1, 1, 1])
    for name, semantic_id in (("table", 10), ("box", 15)):
        (dataset / "configs" / "objects" / f"frl_apartment_{name}.object_config.json").write_text(
            json.dumps({"render_asset": f"../../objects/frl_apartment_{name}.glb", "semantic_id": semantic_id}),
            encoding="utf-8",
        )
    (dataset / "configs" / "ssd" / "replicaCAD_semantic_lexicon.json").write_text(
        json.dumps({"classes": [{"id": 10, "name": "table"}, {"id": 15, "name": "box"}]}),
        encoding="utf-8",
    )
    (dataset / "configs" / "scenes" / "apt_0.scene_instance.json").write_text(
        json.dumps(
            {
                "object_instances": [
                    {"template_name": "objects/frl_apartment_table", "translation": [0, 0, 0]},
                    {"template_name": "objects/frl_apartment_box", "translation": [1, 0, 0]},
                ]
            }
        ),
        encoding="utf-8",
    )
    (dataset / "navmeshes" / "apt_0.navmesh").write_bytes(b"fixture")

    result = MODULE.convert_scene(dataset, "apt_0", near_radius_m=2.0)

    assert result["frame_id"] == "map"
    assert result["graph_level"] == "hierarchical"
    assert result["hierarchy_metadata"]["room_segmentation"] == "unavailable"
    assert result["floors"][0]["object_ids"] == ["object:apt_0:0", "object:apt_0:1"]
    assert {group["name"] for group in result["groups"]} == {"box", "table"}
    assert len(result["hierarchy_edges"]) == 4
    assert result["source"]["axis_transform"] == "[x, y, z] -> [x, z, y]"
    assert result["stats"] == {
        "node_count": 2,
        "relation_count": 1,
        "geometry_bound_nodes": 2,
        "scene_instances": 2,
        "missing_assets": 0,
    }
    assert result["geometry_binding"]["navmesh_present"] is True
    assert len(result["instance_records"]) == 2
    assert result["instance_records"][1]["instance_id"] == "object:apt_0:1"
    assert result["instance_records"][1]["map_identity"] == "replicacad:apt_0"
    assert result["instance_records"][1]["persistence_status"] == "dataset_artifact_only"
    assert result["relations"][0]["subject"] == "object:apt_0:1"
    assert result["relations"][0]["object"] == "object:apt_0:0"
    # The box center is (1.5, 0.5, 0.5) in ReplicaCAD Y-up and (1.5, 0.5, 0.5)
    # in this symmetric fixture; the explicit metadata prevents silent frame mixups.
    assert result["nodes"][1]["geometry_binding"]["status"] == "bound"
    assert result["nodes"][1]["geometry_binding"]["source_up_axis"] == "y"
