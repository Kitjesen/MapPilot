# ruff: noqa: S101, S314

"""Contracts for the deterministic FactoryPark_HF world package."""

from __future__ import annotations

import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import pytest
import yaml

from sim.catalog import CatalogResolver
from sim.tools.worlds.factory_park_hf.generate import generate_factory_park_hf

REPO_ROOT = Path(__file__).resolve().parents[2]
LIVE_SESSION_RELATIVE_PATH = Path(
    "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
)
SESSION_PATH = REPO_ROOT / LIVE_SESSION_RELATIVE_PATH
LIVE_WORLD_PACKAGE_PATH = (
    REPO_ROOT / "sim" / "packages" / "worlds" / "factory_park_hf" / "1.2.1"
)
LIVE_WORLD_WRAPPER_PATH = LIVE_WORLD_PACKAGE_PATH / "physics" / "factory_park_hf.xml"
GENERATOR_OWNED_PATHS = (
    Path("sim/packages/worlds/factory_park_hf/world.package.yaml"),
    Path("sim/packages/worlds/factory_park_hf/provenance/factory-park.provenance.json"),
    Path("sim/packages/worlds/factory_park_hf/visual/realism.recipe.json"),
    Path("sim/packages/worlds/factory_park_hf/visual/ue_import.recipe.json"),
    Path("sim/packages/worlds/factory_park_hf/physics/factory_park_hf.xml"),
    Path("sim/packages/worlds/factory_park_hf/generated/heightfield_r16.png"),
    Path("sim/packages/worlds/factory_park_hf/generated/heightfield_f32.bin"),
    Path("sim/packages/worlds/factory_park_hf/generated/terrain.obj"),
    Path("sim/packages/worlds/factory_park_hf/generated/expanded-layout.json"),
    Path("sim/packages/worlds/factory_park_hf/generated/semantic-entities.json"),
    Path("sim/packages/worlds/factory_park_hf/generated/site-plan.svg"),
    Path("sim/packages/worlds/factory_park_hf/generated/asset-manifest.json"),
)


def _generator_owned_bytes(repo_root: Path) -> dict[Path, bytes]:
    return {path: (repo_root / path).read_bytes() for path in GENERATOR_OWNED_PATHS}


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _records_digest(records: list[dict[str, object]]) -> str:
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: str(item["path"]))
    ]
    return hashlib.sha256(_canonical_json(identity)).hexdigest()


def _read_png_u16(path: Path) -> tuple[int, int, int, int, list[int]]:
    payload = path.read_bytes()
    assert payload.startswith(b"\x89PNG\r\n\x1a\n")
    offset = 8
    width = height = bit_depth = color_type = 0
    compressed = bytearray()
    while offset < len(payload):
        length = struct.unpack_from(">I", payload, offset)[0]
        kind = payload[offset + 4 : offset + 8]
        data = payload[offset + 8 : offset + 8 + length]
        offset += 12 + length
        if kind == b"IHDR":
            width, height, bit_depth, color_type = struct.unpack_from(">IIBB", data)
        elif kind == b"IDAT":
            compressed.extend(data)
        elif kind == b"IEND":
            break
    raw = zlib.decompress(bytes(compressed))
    row_bytes = 1 + width * 2
    assert len(raw) == row_bytes * height
    samples: list[int] = []
    for row in range(height):
        row_start = row * row_bytes
        assert raw[row_start] == 0
        samples.extend(struct.unpack_from(f">{width}H", raw, row_start + 1))
    return width, height, bit_depth, color_type, samples


def _sample_index(contract: dict[str, object], x_m: float, y_m: float) -> int:
    spacing_x, spacing_y = contract["sample_spacing_m"]
    image = contract["image_mapping"]
    column = round((x_m - image["column_0_x_m"]) / spacing_x)
    row = round((image["row_0_y_m"] - y_m) / spacing_y)
    width = contract["grid_px"][0]
    return row * width + column


def _world_height_m(contract: dict[str, object], sample: int) -> float:
    return (
        sample / 65_535 * contract["height_encoding"]["elevation_scale_m"]
        + contract["vertical_origin_m"]
    )


def _distance_to_footprint(item: dict[str, object], point: tuple[float, float]) -> float:
    position = item["position_m"]
    offset_x = point[0] - position[0]
    offset_y = point[1] - position[1]
    if item["shape"] == "cylinder":
        return max(0.0, math.hypot(offset_x, offset_y) - item["radius_m"])
    yaw = math.radians(item["yaw_deg"])
    local_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
    local_y = -math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
    half_x = item["size_m"][0] / 2.0
    half_y = item["size_m"][1] / 2.0
    outside_x = max(0.0, abs(local_x) - half_x)
    outside_y = max(0.0, abs(local_y) - half_y)
    return math.hypot(outside_x, outside_y)


def test_generator_emits_the_factory_park_layout_contract(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)

    layout_path = generated.world_root / "generated" / "expanded-layout.json"
    layout = json.loads(layout_path.read_text(encoding="utf-8"))

    assert layout["world_package"] == "factory_park_hf@1.0.0"
    assert layout["extent_m"] == [220.0, 180.0]
    assert layout["coordinate_system"] == {
        "frame": "mujoco_rh_z_up_m",
        "handedness": "right",
        "up_axis": "z",
        "linear_unit": "metre",
    }


def test_generator_owned_manifest_keeps_the_base_two_millisecond_policy(
    tmp_path: Path,
) -> None:
    generated = generate_factory_park_hf(tmp_path)
    manifest = yaml.safe_load(
        (generated.package_root / "world.package.yaml").read_text(encoding="utf-8")
    )

    assert manifest["version"] == "1.0.0"
    assert manifest["physics"] == {
        "mjcf": "physics/factory_park_hf.xml",
        "global_policy": {
            "timestep_s": 0.002,
            "integrator": "rk4",
            "solver": "newton",
            "iterations": 100,
            "gravity_mps2": [0.0, 0.0, -9.81],
        },
    }


def test_generator_emits_an_authority_safe_industrial_realism_recipe(
    tmp_path: Path,
) -> None:
    generated = generate_factory_park_hf(tmp_path)
    recipe = json.loads(
        (generated.package_root / "visual/realism.recipe.json").read_text(encoding="utf-8")
    )
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )

    assert recipe["schema"] == "lingtu.sim.factory-park-realism-recipe.v1"
    assert recipe["profile"] == "industrial_realism_v2"
    assert recipe["world_package"] == "factory_park_hf@1.0.0"
    assert recipe["layout_digest"] == generated.layout_digest == layout["layout_digest"]
    assert set(recipe["detail_targets"]) == {
        "clutter",
        "facades",
        "gate",
        "lighting_support",
        "loading",
        "roads",
        "tanks",
        "utilities",
        "vegetation",
    }
    assert len(recipe["pbr_material_profiles"]) >= 8
    assert len(recipe["preview_targets"]) >= 3
    assert recipe["performance_budgets"]["target_frame_rate_fps"] == 60
    assert recipe["hard_rules"] == {
        "added_dressing_collision": False,
        "added_dressing_classification": "VisualOnly",
        "allow_new_physics_authority": False,
        "layout_unchanged": True,
        "mujoco_world_unchanged": True,
        "physics_authority": "mujoco",
    }


def test_checked_in_generator_owned_1_0_bytes_match_the_deterministic_generator(
    tmp_path: Path,
) -> None:
    generated_repo = tmp_path / "generated-repo"

    generate_factory_park_hf(generated_repo, seed=20260808)

    assert _generator_owned_bytes(REPO_ROOT) == _generator_owned_bytes(generated_repo)


def test_generator_does_not_write_or_claim_the_live_session(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    provenance = json.loads(
        (generated.package_root / "provenance/factory-park.provenance.json").read_text(
            encoding="utf-8"
        )
    )

    assert not (tmp_path / LIVE_SESSION_RELATIVE_PATH).exists()
    assert LIVE_SESSION_RELATIVE_PATH.as_posix() not in {
        record["path"] for record in provenance["outputs"]
    }


def test_generation_is_byte_for_byte_deterministic(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"

    first_result = generate_factory_park_hf(first, seed=20260808)
    second_result = generate_factory_park_hf(second, seed=20260808)

    assert first_result.layout_digest == second_result.layout_digest
    assert first_result.asset_set_digest == second_result.asset_set_digest
    assert _generator_owned_bytes(first) == _generator_owned_bytes(second)


def test_layout_objects_have_complete_source_frame_geometry(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )
    required = {
        "id",
        "semantic_class",
        "shape",
        "position_m",
        "yaw_deg",
        "material",
        "collision",
        "visual_only",
    }

    assert len(layout["objects"]) >= 100
    assert len({item["id"] for item in layout["objects"]}) == len(layout["objects"])
    for item in layout["objects"]:
        assert required <= set(item)
        assert item["shape"] in {"box", "cylinder"}
        assert len(item["position_m"]) == 3
        assert all(math.isfinite(value) for value in item["position_m"])
        assert math.isfinite(item["yaw_deg"])
        assert isinstance(item["collision"], bool)
        assert isinstance(item["visual_only"], bool)
        assert not (item["collision"] and item["visual_only"])
        if item["shape"] == "box":
            assert len(item["size_m"]) == 3
            assert all(value > 0 for value in item["size_m"])
        else:
            assert item["radius_m"] > 0
            assert item["half_height_m"] > 0


def test_site_extent_and_semantic_zones_are_complete(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )
    by_id = {item["id"]: item for item in layout["objects"]}
    semantic_classes = {item["semantic_class"] for item in layout["objects"]}

    assert layout["extent_m"] == [220.0, 180.0]
    assert set(layout["validation"]["required_semantic_classes"]) <= semantic_classes
    assert by_id["main_factory"]["size_m"] == [70.0, 40.0, 12.0]
    assert by_id["warehouse"]["size_m"] == [60.0, 32.0, 10.0]
    assert by_id["gravel_acceptance_pad"]["size_m"] == [28.0, 14.0, 0.05]
    assert len([item for item in layout["objects"] if item["semantic_class"] == "shipping_container"]) == 7
    assert len([item for item in layout["objects"] if item["semantic_class"] == "storage_tank"]) == 3
    assert len(layout["checkpoints"]) == 6

    half_x, half_y = 110.0, 90.0
    for item in layout["objects"]:
        x_m, y_m, _ = item["position_m"]
        if item["shape"] == "cylinder":
            radius_x = radius_y = item["radius_m"]
        else:
            yaw = math.radians(item["yaw_deg"])
            size_x, size_y, _ = item["size_m"]
            radius_x = abs(math.cos(yaw)) * size_x / 2 + abs(math.sin(yaw)) * size_y / 2
            radius_y = abs(math.sin(yaw)) * size_x / 2 + abs(math.cos(yaw)) * size_y / 2
        assert -half_x <= x_m - radius_x <= x_m + radius_x <= half_x
        assert -half_y <= y_m - radius_y <= y_m + radius_y <= half_y


def test_roads_routes_gate_and_spawn_clearance_are_navigable(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )
    roads = [item for item in layout["objects"] if item["semantic_class"] == "road"]
    routes = {route["id"]: route for route in layout["routes"]}

    assert len(roads) == 9
    assert min(road["clear_width_m"] for road in roads) >= 8.0
    assert routes["robot_acceptance_loop"]["width_m"] == 4.0
    assert routes["forklift_logistics_route"]["width_m"] == 3.5
    assert routes["pedestrian_safe_route"]["width_m"] == 1.8
    assert layout["validation"]["south_gate_clear_width_m"] == 18.0
    for route in routes.values():
        assert route["closed"] is True
        assert route["waypoints_m"][0] == route["waypoints_m"][-1]
        assert all(-110 <= point[0] <= 110 and -90 <= point[1] <= 90 for point in route["waypoints_m"])

    spawn = tuple(layout["spawn"]["position_m"][:2])
    traversable_ids = {
        *(road["id"] for road in roads),
        "parking_apron",
        "container_yard_apron",
        "tank_bund_floor",
        "gravel_acceptance_pad",
    }
    obstacles = [
        item
        for item in layout["objects"]
        if item["collision"] and item["id"] not in traversable_ids
    ]
    assert min(_distance_to_footprint(item, spawn) for item in obstacles) >= layout["spawn"]["clearance_radius_m"]


def test_spawn_and_engineered_surfaces_share_one_vertical_datum(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    generated_root = generated.world_root / "generated"
    manifest = json.loads((generated_root / "asset-manifest.json").read_text(encoding="utf-8"))
    contract = manifest["coordinate_contract"]
    width, height, bit_depth, color_type, samples = _read_png_u16(
        generated_root / "heightfield_r16.png"
    )

    assert [width, height] == contract["grid_px"] == [221, 181]
    assert bit_depth == 16
    assert color_type == 0
    assert contract["extent_m"] == [220.0, 180.0]
    assert contract["sample_spacing_m"] == [1.0, 1.0]

    datum_points = (
        (0.0, -76.0),
        (0.0, -44.0),
        (0.0, 31.0),
        (0.0, 62.0),
        (-93.0, 0.0),
        (93.0, 0.0),
        (50.0, 9.0),
        (25.0, -34.0),
        (-52.0, -33.0),
        (65.0, -36.0),
    )
    for x_m, y_m in datum_points:
        sample = samples[_sample_index(contract, x_m, y_m)]
        assert _world_height_m(contract, sample) == pytest.approx(0.0, abs=1e-12)

    spawn = contract["spawn_reference"]
    spawn_index = _sample_index(contract, spawn["world_position_m"][0], spawn["world_position_m"][1])
    assert spawn["sample_u16"] == samples[spawn_index]
    assert _world_height_m(contract, samples[spawn_index]) == pytest.approx(0.0, abs=1e-12)


def test_obj_png_and_mujoco_use_the_same_world_z(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    generated_root = generated.world_root / "generated"
    manifest = json.loads((generated_root / "asset-manifest.json").read_text(encoding="utf-8"))
    contract = manifest["coordinate_contract"]
    width, height, _, _, samples = _read_png_u16(generated_root / "heightfield_r16.png")
    vertices = [
        [float(value) for value in line.split()[1:]]
        for line in (generated_root / "terrain.obj").read_text(encoding="ascii").splitlines()
        if line.startswith("v ")
    ]

    assert len(vertices) == len(samples) == width * height
    selected_points = ((0.0, -76.0), (50.0, 9.0), (85.0, 30.0), (-76.0, 76.0))
    for x_m, y_m in selected_points:
        index = _sample_index(contract, x_m, y_m)
        world_z_m = _world_height_m(contract, samples[index])
        assert vertices[index] == pytest.approx([x_m * 100.0, -y_m * 100.0, world_z_m * 100.0], abs=1e-5)

    xml_root = ET.parse(generated.world_root / "physics" / "factory_park_hf.xml").getroot()
    terrain_geom = xml_root.find("./worldbody/geom[@name='terrain']")
    assert terrain_geom is not None
    assert [float(value) for value in terrain_geom.attrib["pos"].split()] == pytest.approx(
        [0.0, 0.0, contract["vertical_origin_m"]], abs=1e-9
    )
    assert terrain_geom.attrib["hfield"] == "factory_park_terrain"


def test_mujoco_xml_contains_every_physics_shared_object(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    layout = json.loads(
        (generated.world_root / "generated/expanded-layout.json").read_text(encoding="utf-8")
    )
    xml_root = ET.parse(generated.world_root / "physics" / "factory_park_hf.xml").getroot()
    geoms = {geom.attrib["name"]: geom for geom in xml_root.findall("./worldbody/geom")}

    for item in layout["objects"]:
        geom = geoms[item["id"]]
        assert geom.attrib["type"] == item["shape"]
        if item["collision"]:
            assert geom.attrib["contype"] == "1"
            assert geom.attrib["conaffinity"] == "1"
        else:
            assert geom.attrib["contype"] == "0"
            assert geom.attrib["conaffinity"] == "0"


def test_layout_asset_and_provenance_digests_cover_exact_bytes(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    generated_root = generated.world_root / "generated"
    package_root = generated.package_root
    layout = json.loads((generated_root / "expanded-layout.json").read_text(encoding="utf-8"))
    manifest = json.loads((generated_root / "asset-manifest.json").read_text(encoding="utf-8"))
    recipe = json.loads((package_root / "visual/ue_import.recipe.json").read_text(encoding="utf-8"))
    realism_recipe_path = package_root / "visual/realism.recipe.json"
    realism_recipe = json.loads(realism_recipe_path.read_text(encoding="utf-8"))
    provenance = json.loads(
        (package_root / "provenance/factory-park.provenance.json").read_text(encoding="utf-8")
    )

    digest = layout.pop("layout_digest")
    assert digest == hashlib.sha256(_canonical_json(layout)).hexdigest()
    assert digest == "1bb084b64a5d10baaddbee62bee879eec679cd666f0c331ac984542a74f9907f"
    assert (
        digest
        == manifest["layout_digest"]
        == recipe["layout_digest"]
        == realism_recipe["layout_digest"]
        == provenance["layout_digest"]
    )

    realism_relative_path = "sim/packages/worlds/factory_park_hf/visual/realism.recipe.json"
    realism_payload = realism_recipe_path.read_bytes()
    realism_sha256 = hashlib.sha256(realism_payload).hexdigest()
    manifest_realism = next(
        record for record in manifest["assets"] if record["path"] == realism_relative_path
    )
    provenance_realism = next(
        record for record in provenance["outputs"] if record["path"] == realism_relative_path
    )
    assert manifest_realism == recipe["sources"]["realism_recipe"]
    assert manifest_realism["role"] == "visual_realism_recipe"
    assert manifest_realism["sha256"] == provenance_realism["sha256"] == realism_sha256
    assert manifest_realism["bytes"] == provenance_realism["bytes"] == len(realism_payload)

    for record in manifest["assets"]:
        payload = (tmp_path / record["path"]).read_bytes()
        assert record["bytes"] == len(payload)
        assert record["sha256"] == hashlib.sha256(payload).hexdigest()
    assert manifest["asset_set_digest"] == _records_digest(manifest["assets"])

    for record in provenance["outputs"]:
        payload = (tmp_path / record["path"]).read_bytes()
        assert record["bytes"] == len(payload)
        assert record["sha256"] == hashlib.sha256(payload).hexdigest()
    assert provenance["asset_set_digest"] == _records_digest(provenance["outputs"])
    assert provenance["source"] == {
        "type": "procedural",
        "owner": "LingTu project",
        "license": "LicenseRef-LingTu-Project-Owned",
        "third_party_assets": [],
    }


def test_live_session_routes_through_the_realtime_world_wrapper() -> None:
    session = yaml.safe_load(SESSION_PATH.read_text(encoding="utf-8"))
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION_PATH)

    assert session["world"] == "factory_park_hf@1.2.1"
    assert resolved.visual_plan["world"]["binding"] == "WorldVisual:FactoryParkHF"
    assert resolved.visual_plan["world"]["level"] == (
        "/Game/RobotSim/Generated/FactoryParkHF/Maps/"
        "FactoryPark_HF_Instanced_9d314c86ef83ea06"
    )
    assert resolved.visual_plan["world"]["package"]["id"] == "factory_park_hf"
    assert resolved.visual_plan["world"]["package"]["version"] == "1.2.1"
    assert (
        resolved.physics_plan["world"]["mjcf"]
        == "sim/packages/worlds/factory_park_hf/1.2.1/physics/factory_park_hf.xml"
    )
    assert resolved.physics_plan["robots"][0]["spawn"] == {
        "position_m": [0.0, -76.0, 0.0],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    assert resolved.session == session

    manifest = yaml.safe_load(
        (LIVE_WORLD_PACKAGE_PATH / "world.package.yaml").read_text(encoding="utf-8")
    )
    assert manifest["id"] == resolved.visual_plan["world"]["package"]["id"]
    assert manifest["version"] == resolved.visual_plan["world"]["package"]["version"]
    assert manifest["physics"]["mjcf"] == "physics/factory_park_hf.xml"
    assert manifest["visual"] == {
        "binding": "WorldVisual:FactoryParkHF",
        "level": (
            "/Game/RobotSim/Generated/FactoryParkHF/Maps/"
            "FactoryPark_HF_Instanced_9d314c86ef83ea06"
        ),
    }


def test_live_realtime_wrapper_owns_the_one_millisecond_implicitfast_policy() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION_PATH)
    manifest = yaml.safe_load(
        (LIVE_WORLD_PACKAGE_PATH / "world.package.yaml").read_text(encoding="utf-8")
    )
    wrapper = ET.parse(LIVE_WORLD_WRAPPER_PATH).getroot()
    include = wrapper.find("include")
    option = wrapper.find("option")

    expected_policy = {
        "timestep_s": 0.001,
        "integrator": "implicitfast",
        "solver": "newton",
        "iterations": 100,
        "gravity_mps2": [0.0, 0.0, -9.81],
    }
    assert manifest["physics"]["global_policy"] == expected_policy
    assert resolved.physics_plan["global_policy"] == {
        "owner": "world",
        **expected_policy,
    }
    assert include is not None
    assert include.attrib == {"file": "../../physics/factory_park_hf.xml"}
    assert option is not None
    assert option.attrib == {
        "gravity": "0 0 -9.81",
        "timestep": "0.001",
        "integrator": "implicitfast",
        "solver": "Newton",
        "iterations": "100",
    }


def test_site_plan_is_valid_svg_with_routes_and_semantic_ids(tmp_path: Path) -> None:
    generated = generate_factory_park_hf(tmp_path)
    svg_root = ET.parse(generated.world_root / "generated/site-plan.svg").getroot()
    namespace = {"svg": "http://www.w3.org/2000/svg"}
    ids = {element.attrib["id"] for element in svg_root.iter() if "id" in element.attrib}

    assert svg_root.attrib["viewBox"] == "0 0 1100 900"
    assert {"objects", "routes", "checkpoint-labels", "main_factory", "warehouse"} <= ids
    assert len(svg_root.findall(".//svg:polyline", namespace)) == 3


def test_mujoco_compiles_generated_factory_when_available(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    generated = generate_factory_park_hf(tmp_path)
    model = mujoco.MjModel.from_xml_path(str(generated.world_root / "physics" / "factory_park_hf.xml"))

    hfield_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "factory_park_terrain")
    factory_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "main_factory")
    speed_bump_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "speed_bump_gate")
    assert hfield_id >= 0
    assert factory_geom_id >= 0
    assert speed_bump_id >= 0
    assert model.hfield_nrow[hfield_id] == 181
    assert model.hfield_ncol[hfield_id] == 221
    assert model.hfield_size[hfield_id].tolist() == pytest.approx([110.0, 90.0, 0.75, 1.0])
