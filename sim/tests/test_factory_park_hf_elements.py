# ruff: noqa: S101

"""Behavior contracts for direct FactoryPark_HF element production."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from sim.tools.worlds.factory_park_hf.elements import compile_element_batch, element_catalog
from sim.tools.worlds.factory_park_hf.generate import (
    generate_factory_park_hf,
)
from sim.tools.worlds.factory_park_hf.generate import (
    main as generate_main,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
ELEMENT_SCHEMA_PATH = (
    REPO_ROOT / "sim/contracts/schemas/factory-park.v1.json"
)


def _layout() -> dict[str, object]:
    return {
        "schema": "lingtu.sim.expanded-world-layout.v1",
        "world_package": "factory_park_hf@1.0.0",
        "coordinate_system": {
            "frame": "mujoco_rh_z_up_m",
            "handedness": "right",
            "up_axis": "z",
            "linear_unit": "metre",
        },
        "site_boundary_m": {
            "minimum": [-20.0, -15.0, -1.0],
            "maximum": [20.0, 15.0, 16.0],
        },
        "spawn": {
            "position_m": [-15.0, -10.0, 0.0],
            "clearance_radius_m": 4.0,
        },
        "objects": [
            {
                "id": "yard_apron",
                "semantic_class": "container_yard",
                "shape": "box",
                "position_m": [0.0, 0.0, 0.025],
                "size_m": [20.0, 12.0, 0.05],
                "yaw_deg": 0.0,
                "material": "concrete",
                "collision": True,
                "visual_only": False,
            }
        ],
    }


def test_user_batch_compiles_a_catalog_element_into_source_frame_geometry() -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "dock_safety",
        "elements": [
            {
                "instance_key": "bollard_01",
                "element_type": "safety_bollard",
                "surface_id": "yard_apron",
                "position_xy_m": [1.0, 2.0],
                "yaw_deg": 90.0,
            }
        ],
    }

    compiled = compile_element_batch(batch, _layout())

    assert compiled.batch_id == "dock_safety"
    assert len(compiled.digest) == 64
    assert compiled.objects == (
        {
            "id": "element__dock_safety__bollard_01",
            "semantic_class": "safety_bollard",
            "shape": "cylinder",
            "position_m": [1.0, 2.0, 0.6],
            "radius_m": 0.12,
            "half_height_m": 0.55,
            "yaw_deg": 90.0,
            "material": "painted_steel",
            "collision": True,
            "visual_only": False,
            "element_provenance": {
                "batch_id": "dock_safety",
                "element_type": "safety_bollard",
                "instance_key": "bollard_01",
                "surface_id": "yard_apron",
            },
        },
    )


def test_element_catalog_exposes_only_supported_factory_primitives() -> None:
    catalog = element_catalog()

    assert set(catalog) == {
        "equipment_cabinet",
        "fire_cabinet",
        "industrial_drum",
        "jersey_barrier",
        "lane_marker",
        "pallet_stack",
        "safety_bollard",
        "safety_sign",
        "traffic_cone",
        "wheel_stop",
    }
    for item in catalog.values():
        assert item["shape"] in {"box", "cylinder"}
        assert item["authority"] in {"PhysicsShared", "VisualOnly"}
        assert item["material"]
        assert item["allowed_surface_classes"]


def test_element_batch_json_schema_tracks_the_runtime_catalog() -> None:
    schema = json.loads(ELEMENT_SCHEMA_PATH.read_text(encoding="utf-8"))

    assert schema["additionalProperties"] is False
    assert schema["properties"]["elements"]["minItems"] == 1
    assert set(
        schema["properties"]["elements"]["items"]["properties"]["element_type"]["enum"]
    ) == set(element_catalog())


def test_element_must_use_a_catalog_approved_support_surface() -> None:
    layout = _layout()
    layout["objects"][0]["semantic_class"] = "main_factory_building"  # type: ignore[index]
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "bad_support",
        "elements": [
            {
                "instance_key": "bollard_01",
                "element_type": "safety_bollard",
                "surface_id": "yard_apron",
                "position_xy_m": [0.0, 0.0],
            }
        ],
    }

    with pytest.raises(ValueError, match="not allowed on surface class"):
        compile_element_batch(batch, layout)


def test_rotated_element_footprint_must_fit_inside_its_support_surface() -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "edge_case",
        "elements": [
            {
                "instance_key": "barrier_01",
                "element_type": "jersey_barrier",
                "surface_id": "yard_apron",
                "position_xy_m": [9.5, 0.0],
                "yaw_deg": 45.0,
            }
        ],
    }

    with pytest.raises(ValueError, match="footprint does not fit inside support surface"):
        compile_element_batch(batch, _layout())


def test_element_batch_rejects_duplicate_stable_ids() -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "duplicate",
        "elements": [
            {
                "instance_key": "cone_01",
                "element_type": "traffic_cone",
                "surface_id": "yard_apron",
                "position_xy_m": [-1.0, 0.0],
            },
            {
                "instance_key": "cone_01",
                "element_type": "traffic_cone",
                "surface_id": "yard_apron",
                "position_xy_m": [1.0, 0.0],
            },
        ],
    }

    with pytest.raises(ValueError, match="duplicate stable id"):
        compile_element_batch(batch, _layout())


@pytest.mark.parametrize("field,value", [("batch_id", "dock/safety"), ("instance_key", "Cone 01")])
def test_element_identity_must_be_blender_and_mujoco_safe(field: str, value: str) -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "safe_batch",
        "elements": [
            {
                "instance_key": "cone_01",
                "element_type": "traffic_cone",
                "surface_id": "yard_apron",
                "position_xy_m": [0.0, 0.0],
            }
        ],
    }
    if field == "batch_id":
        batch["batch_id"] = value
    else:
        batch["elements"][0]["instance_key"] = value  # type: ignore[index]

    with pytest.raises(ValueError, match=f"{field} must match"):
        compile_element_batch(batch, _layout())


def test_element_batch_flows_into_layout_semantics_and_mujoco(tmp_path) -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "parking_safety",
        "elements": [
            {
                "instance_key": "bollard_01",
                "element_type": "safety_bollard",
                "surface_id": "parking_apron",
                "position_xy_m": [10.0, -48.0],
            }
        ],
    }

    generated = generate_factory_park_hf(tmp_path, element_batches=(batch,))
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )
    semantics = json.loads(
        (generated.world_root / "generated/semantic-entities.json").read_text(encoding="utf-8")
    )
    manifest = json.loads(
        (generated.world_root / "generated/asset-manifest.json").read_text(encoding="utf-8")
    )
    recipe = json.loads(
        (generated.package_root / "visual/ue_import.recipe.json").read_text(encoding="utf-8")
    )
    provenance = json.loads(
        (generated.package_root / "provenance/factory-park.provenance.json").read_text(
            encoding="utf-8"
        )
    )
    mjcf = (generated.world_root / "physics" / "factory_park_hf.xml").read_text(encoding="utf-8")

    stable_id = "element__parking_safety__bollard_01"
    assert layout["element_batches"] == [
        {
            "batch_id": "parking_safety",
            "digest": layout["element_batches"][0]["digest"],
            "object_count": 1,
        }
    ]
    assert len(layout["element_batches"][0]["digest"]) == 64
    assert manifest["element_batches"] == layout["element_batches"]
    assert recipe["element_batches"] == layout["element_batches"]
    assert provenance["element_batches"] == layout["element_batches"]
    assert any(item["id"] == stable_id for item in layout["objects"])
    assert any(item["id"] == stable_id for item in semantics["entities"])
    assert f'name="{stable_id}"' in mjcf
    assert 'contype="1"' in mjcf[mjcf.index(f'name="{stable_id}"') :]


def test_physics_shared_element_cannot_enter_spawn_clearance() -> None:
    layout = _layout()
    layout["spawn"] = {"position_m": [0.0, 0.0, 0.0], "clearance_radius_m": 4.0}
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "unsafe_spawn",
        "elements": [
            {
                "instance_key": "barrier_01",
                "element_type": "jersey_barrier",
                "surface_id": "yard_apron",
                "position_xy_m": [3.0, 0.0],
            }
        ],
    }

    with pytest.raises(ValueError, match="spawn clearance"):
        compile_element_batch(batch, layout)


def test_visual_only_catalog_element_is_non_colliding_in_layout_and_mujoco(tmp_path) -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "parking_markers",
        "elements": [
            {
                "instance_key": "marker_01",
                "element_type": "lane_marker",
                "surface_id": "parking_apron",
                "position_xy_m": [25.0, -34.0],
                "yaw_deg": 90.0,
            }
        ],
    }

    generated = generate_factory_park_hf(tmp_path, element_batches=(batch,))
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )
    stable_id = "element__parking_markers__marker_01"
    item = next(item for item in layout["objects"] if item["id"] == stable_id)
    assert item["collision"] is False
    assert item["visual_only"] is True

    mjcf = (generated.world_root / "physics" / "factory_park_hf.xml").read_text(encoding="utf-8")
    geom = mjcf[mjcf.index(f'name="{stable_id}"') :].split("/>", 1)[0]
    assert 'contype="0"' in geom
    assert 'conaffinity="0"' in geom


def test_generator_cli_accepts_repeatable_element_batch_files(tmp_path, capsys) -> None:
    batch_path = tmp_path / "parking-elements.json"
    batch_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.factory-park-element-batch.v1",
                "batch_id": "cli_batch",
                "elements": [
                    {
                        "instance_key": "cone_01",
                        "element_type": "traffic_cone",
                        "surface_id": "parking_apron",
                        "position_xy_m": [10.0, -48.0],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    assert (
        generate_main(
            [
                "--repo-root",
                str(tmp_path),
                "--element-batch",
                str(batch_path),
            ]
        )
        == 0
    )
    result = json.loads(capsys.readouterr().out)
    layout = json.loads(
        (tmp_path / "sim/packages/worlds/factory_park_hf/generated/expanded-layout.json").read_text(
            encoding="utf-8"
        )
    )

    assert result["element_batches"] == [
        "operations_safety",
        "cli_batch",
    ]
    assert [item["batch_id"] for item in layout["element_batches"]] == [
        "operations_safety",
        "cli_batch",
    ]


def test_generator_refuses_to_overwrite_changed_outputs_without_explicit_force(tmp_path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    layout_path = generated.world_root / "generated/expanded-layout.json"
    user_bytes = b'{"user_owned":true}\n'
    layout_path.write_bytes(user_bytes)

    with pytest.raises(FileExistsError, match="--force-overwrite"):
        generate_factory_park_hf(tmp_path)

    assert layout_path.read_bytes() == user_bytes


def test_generator_explicit_overwrite_restores_canonical_outputs(tmp_path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    layout_path = generated.world_root / "generated/expanded-layout.json"
    layout_path.write_bytes(b'{"user_owned":true}\n')

    generate_factory_park_hf(tmp_path, overwrite_existing=True)

    layout = json.loads(layout_path.read_text(encoding="utf-8"))
    assert layout["schema"] == "lingtu.sim.expanded-world-layout.v1"
    assert layout["layout_digest"] == generated.layout_digest


def test_catalog_materials_are_declared_for_mujoco(tmp_path) -> None:
    generated = generate_factory_park_hf(
        tmp_path,
        element_batches=(
            {
                "schema": "lingtu.sim.factory-park-element-batch.v1",
                "batch_id": "material_probe",
                "elements": [
                    {
                        "instance_key": "pallet_01",
                        "element_type": "pallet_stack",
                        "surface_id": "container_yard_apron",
                        "position_xy_m": [-25.0, -14.0],
                    }
                ],
            },
        ),
    )

    mjcf = (generated.world_root / "physics" / "factory_park_hf.xml").read_text(encoding="utf-8")
    assert '<material name="pallet_wood"' in mjcf


def test_element_batch_rejects_attempts_to_override_catalog_authority() -> None:
    batch = {
        "schema": "lingtu.sim.factory-park-element-batch.v1",
        "batch_id": "authority_override",
        "elements": [
            {
                "instance_key": "marker_01",
                "element_type": "lane_marker",
                "surface_id": "yard_apron",
                "position_xy_m": [0.0, 0.0],
                "collision": True,
            }
        ],
    }

    with pytest.raises(ValueError, match=r"unsupported fields.*collision"):
        compile_element_batch(batch, _layout())
