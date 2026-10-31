from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "bench" / "semantic_pcd_scene.py"
SPEC = importlib.util.spec_from_file_location("semantic_pcd_scene", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_geometry_semantic_scene_keeps_structure_surface_and_navigation_layers() -> None:
    rng = np.random.default_rng(4)
    floor_a = np.column_stack((rng.uniform(0.0, 2.0, 500), rng.uniform(0.0, 2.0, 500), rng.normal(0.0, 0.025, 500)))
    floor_b = np.column_stack((rng.uniform(4.0, 6.0, 500), rng.uniform(0.0, 2.0, 500), rng.normal(0.0, 0.025, 500)))
    wall_a = np.column_stack((rng.uniform(0.0, 2.0, 500), rng.uniform(0.0, 2.0, 500), rng.uniform(0.0, 1.5, 500)))
    wall_b = np.column_stack((rng.uniform(4.0, 6.0, 500), rng.uniform(0.0, 2.0, 500), rng.uniform(0.0, 1.5, 500)))

    scene = MODULE.build_scene(np.vstack((floor_a, floor_b, wall_a, wall_b)).astype(np.float32))

    kinds = {node["kind"] for node in scene["nodes"]}
    assert {"site", "vertical_structure_group", "support_surface_region", "surface_level"} <= kinds
    assert len([node for node in scene["nodes"] if node["kind"] == "support_surface_region"]) >= 2
    assert {layer["id"] for layer in scene["navigation_layers"]} == {
        "support_candidate",
        "collision_geometry",
        "unknown_space",
    }
    assert all(relation["evidence"] for relation in scene["relations"])
