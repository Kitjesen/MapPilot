# ruff: noqa: S101

"""No-network tests for deterministic Forest_HF UE PBR binding."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import struct
import zlib
from pathlib import Path
from typing import Any, Mapping

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "sim/runtime/visual/RobotSimUE/Scripts/build_forest_hf_pbr_mcp.py"


def _builder() -> Any:
    spec = importlib.util.spec_from_file_location("lingtu_forest_hf_pbr_mcp", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_manifest(tmp_path: Path, builder: Any) -> Path:
    materials = []
    assets = []
    for material_id in builder.MATERIAL_NAMES:
        textures = {}
        for channel, suffix in (("base_color", "basecolor"), ("normal", "normal"), ("orm", "orm")):
            relative = f"textures/forest/{material_id}_{suffix}.png"
            payload = f"{material_id}:{channel}:deterministic".encode()
            target = tmp_path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
            sha256 = hashlib.sha256(payload).hexdigest()
            textures[channel] = {
                "path": relative,
                "bytes": len(payload),
                "sha256": sha256,
                "source": f"repo://forest/{material_id}:{channel}",
                "license": "LicenseRef-LingTu-Project-Owned",
            }
            assets.append(
                {
                    "role": f"texture_{material_id}_{suffix}",
                    "path": relative,
                    "bytes": len(payload),
                    "sha256": sha256,
                }
            )
        materials.append(
            {
                "id": material_id,
                "shader": "principled_pbr",
                "textures": textures,
                "bindings": dict(builder.EXPECTED_BINDINGS),
            }
        )
    assets.sort(key=lambda item: (item["role"], item["path"]))
    identity = [{"role": item["role"], "path": item["path"], "sha256": item["sha256"]} for item in assets]
    body = {
        "schema": builder.MANIFEST_SCHEMA,
        "world_package": builder.WORLD_PACKAGE,
        "generator": {"deterministic_seed": 20260813},
        "authority": {
            "classification": "VisualOnly",
            "unreal_collision_profile": "NoCollision",
            "physics_authority": "mujoco",
            "visual_meshes_are_colliders": False,
            "simulate_physics": False,
            "can_ever_affect_navigation": False,
        },
        "layout": {"schema": "lingtu.sim.forest-layout.v1", "tree_count": 3, "digest": "a" * 64},
        "materials": materials,
        "assets": assets,
        "artifact_set_digest": hashlib.sha256(builder.canonical_json_bytes(identity)).hexdigest(),
    }
    manifest = {**body, "digest": hashlib.sha256(builder._compact_json(body).encode()).hexdigest()}
    path = tmp_path / "authoring.manifest.json"
    path.write_bytes(builder.canonical_json_bytes(manifest))
    return path


def _rewrite_manifest(path: Path, builder: Any, transform: Any) -> None:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    transform(manifest)
    body = {key: value for key, value in manifest.items() if key != "digest"}
    manifest["digest"] = hashlib.sha256(builder._compact_json(body).encode()).hexdigest()
    path.write_bytes(builder.canonical_json_bytes(manifest))


def _write_forest_authoring_plan(tmp_path: Path, builder: Any, *, pine: int = 2, birch: int = 1) -> Path:
    slot_counts = {"forest.asset.birch": birch, "forest.asset.pine": pine}
    content_digest = hashlib.sha256(builder.canonical_json_bytes(slot_counts)).hexdigest()
    points = {
        "schema": builder.FOREST_POINTS_SCHEMA,
        "world_package": builder.WORLD_PACKAGE,
        "seed": 20260813,
        "selection_policy": "exact_mesh_slot_branches",
        "point_count": pine + birch,
        "slot_counts": slot_counts,
        "content_digest": content_digest,
        "groups": [],
    }
    points_path = tmp_path / "forest-points.json"
    points_path.write_bytes(builder.canonical_json_bytes(points))
    points_sha256 = hashlib.sha256(points_path.read_bytes()).hexdigest()
    plan = {
        "schema": builder.FOREST_AUTHORING_PLAN_SCHEMA,
        "world_package": builder.WORLD_PACKAGE,
        "seed": 20260813,
        "selection_policy": "exact_mesh_slot_branches",
        "target": {"map": builder.MAP_PATH},
        "inputs": {
            "create_points": {
                "path": str(points_path),
                "sha256": points_sha256,
                "content_digest": content_digest,
                "point_count": pine + birch,
                "slot_point_counts": slot_counts,
            },
            "slot_assets": [
                {
                    "slot": slot,
                    "asset_ref": builder.TREE_MESHES[species]["asset_ref"],
                }
                for slot, species in builder.TREE_SLOT_TO_SPECIES.items()
            ],
        },
    }
    path = tmp_path / "forest-authoring-plan.json"
    path.write_bytes(builder.canonical_json_bytes(plan))
    return path


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", zlib.crc32(chunk_type + data))


def _write_heightmap(
    path: Path,
    width: int = 4033,
    height: int = 4033,
    bit_depth: int = 16,
    color_type: int = 0,
) -> Path:
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[color_type]
    row_bytes = (width * channels * bit_depth + 7) // 8
    compressor = zlib.compressobj(level=1)
    compressed = bytearray()
    row = b"\x00" + bytes(row_bytes)
    for _ in range(height):
        compressed.extend(compressor.compress(row))
    compressed.extend(compressor.flush())
    ihdr = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", bytes(compressed))
        + _png_chunk(b"IEND", b"")
    )
    return path


def _write_landscape_receipt(tmp_path: Path, builder: Any, heightmap: Path) -> Path:
    body = {
        "schema": builder.LANDSCAPE_IMPORT_RECEIPT_SCHEMA,
        "world_package": builder.WORLD_PACKAGE,
        "map_path": builder.MAP_PATH,
        "actor_label": builder.PRODUCTION_TERRAIN_LABEL,
        "actor_class": builder.LANDSCAPE_CLASS,
        "heightmap": {
            "sha256": hashlib.sha256(heightmap.read_bytes()).hexdigest(),
            "width": 4033,
            "height": 4033,
            "bit_depth": 16,
            "color_type": 0,
        },
        "topology": {
            "grid_components_x": 32,
            "grid_components_y": 32,
            "component_size_quads": 126,
            "subsection_size_quads": 63,
            "num_subsections": 2,
        },
        "importer": {"tool": "test.landscape_importer", "version": "1.0"},
    }
    receipt = {**body, "digest": hashlib.sha256(builder._compact_json(body).encode()).hexdigest()}
    path = tmp_path / "landscape-import-receipt.json"
    path.write_bytes(builder.canonical_json_bytes(receipt))
    return path


def _validation_inputs(tmp_path: Path, builder: Any) -> Any:
    return builder.load_pbr_inputs(
        _write_manifest(tmp_path, builder),
        builder.VALIDATION_PROFILE,
        forest_authoring_plan=_write_forest_authoring_plan(tmp_path, builder),
    )


def _production_inputs(tmp_path: Path, builder: Any) -> Any:
    manifest = _write_manifest(tmp_path, builder)
    plan = _write_forest_authoring_plan(tmp_path, builder)
    heightmap = _write_heightmap(tmp_path / "heightfield_r16.png")
    receipt = _write_landscape_receipt(tmp_path, builder, heightmap)
    return builder.load_pbr_inputs(
        manifest,
        builder.PRODUCTION_PROFILE,
        heightmap,
        plan,
        receipt,
    )


class FakeTransport:
    """Stateful fake with separate in-memory and saved World Partition state."""

    def __init__(self, builder: Any, *, terrain_class: str | None = None) -> None:
        self.builder = builder
        self.initialized = False
        self.notified = False
        self.current_level = builder.MAP_PATH
        self.calls: list[tuple[str, str, dict[str, object]]] = []
        self.folders = {builder.ASSET_ROOT}
        self.assets = {
            builder.MAP_PATH,
            builder.OPEN_WORLD_TEMPLATE,
            builder.VALIDATION_TERRAIN_MESH,
            *(str(record["asset_path"]) for record in builder.TREE_MESHES.values()),
        }
        self.asset_classes = {
            builder.VALIDATION_TERRAIN_MESH: "StaticMesh",
            **{str(record["asset_path"]): "StaticMesh" for record in builder.TREE_MESHES.values()},
        }
        self.metadata: dict[str, dict[str, str]] = {}
        self.dirty: set[str] = set()
        self.texture_properties: dict[str, dict[str, object]] = {}
        self.material_expressions: dict[str, list[str]] = {}
        self.expression_properties: dict[str, dict[str, object]] = {}
        self.material_outputs: dict[str, dict[str, tuple[str, str]]] = {}
        self.mesh_slots = {
            builder.VALIDATION_TERRAIN_MESH_REF: ["defaultMat"],
            **{str(record["asset_ref"]): list(record["slots"]) for record in builder.TREE_MESHES.values()},
        }
        self.mesh_materials: dict[tuple[str, str], str] = {}
        self.terrain_actor = f"{builder.MAP_PATH}.{builder.MAP_PATH.rsplit('/', 1)[-1]}:PersistentLevel.StaticTerrain"
        self.terrain_component = f"{self.terrain_actor}.StaticMeshComponent0"
        self.partition_actors = tuple(
            f"{builder.MAP_PATH}.{builder.MAP_PATH.rsplit('/', 1)[-1]}:PersistentLevel."
            f"PCGPartitionGridActor_25600_{x}_{y}"
            for x, y in sorted(builder.EXPECTED_PARTITION_COORDINATES)
        )
        self.pine_hism = f"{self.partition_actors[0]}.PineHISM"
        self.birch_hism = f"{self.partition_actors[1]}.BirchHISM"
        self.actor_labels = {self.terrain_actor: builder.VALIDATION_TERRAIN_LABEL}
        self.classes = {
            self.terrain_actor: terrain_class or builder.STATIC_MESH_ACTOR_CLASS,
            self.terrain_component: builder.STATIC_MESH_COMPONENT_CLASS,
            self.pine_hism: builder.HISM_COMPONENT_CLASS,
            self.birch_hism: builder.HISM_COMPONENT_CLASS,
            builder.VALIDATION_TERRAIN_MESH_REF: "/Script/Engine.StaticMesh",
            **{path: "/Script/Engine.PCGPartitionGridActor" for path in self.partition_actors},
            **{str(record["asset_ref"]): "/Script/Engine.StaticMesh" for record in builder.TREE_MESHES.values()},
        }
        self.actor_properties: dict[str, dict[str, object]] = {
            self.terrain_actor: {"tags": [], "bGenerateOverlapEventsDuringLevelStreaming": True},
        }
        authority = {
            "bodyInstance": {
                "collisionProfileName": "NoCollision",
                "collisionEnabled": "QueryAndPhysics",
                "bSimulatePhysics": False,
                "bEnableGravity": False,
                "bNotifyRigidBodyCollision": False,
            },
            "bGenerateOverlapEvents": False,
            "bCanEverAffectNavigation": False,
            "componentTags": ["VisualOnly", "NoCollision"],
        }
        self.component_properties: dict[str, dict[str, object]] = {
            self.terrain_component: {
                "staticMesh": {"refPath": builder.VALIDATION_TERRAIN_MESH_REF},
                "bodyInstance": {
                    "collisionProfileName": "BlockAll",
                    "collisionEnabled": "QueryAndPhysics",
                    "bSimulatePhysics": False,
                    "bEnableGravity": True,
                    "bNotifyRigidBodyCollision": False,
                },
                "bGenerateOverlapEvents": False,
                "bCanEverAffectNavigation": True,
                "componentTags": [],
            },
            self.pine_hism: {
                "staticMesh": {"refPath": str(builder.TREE_MESHES["pine"]["asset_ref"])},
                "perInstanceSMData": [{"transform": {}}, {"transform": {}}],
                **copy.deepcopy(authority),
            },
            self.birch_hism: {
                "staticMesh": {"refPath": str(builder.TREE_MESHES["birch"]["asset_ref"])},
                "perInstanceSMData": [{"transform": {}}],
                **copy.deepcopy(authority),
            },
        }
        self.collidable_actors = {self.terrain_actor}
        self.corrupt_texture_metadata_after_reload = False
        self.disk_actor_properties = copy.deepcopy(self.actor_properties)
        self.disk_component_properties = copy.deepcopy(self.component_properties)
        self.disk_collidable_actors = set(self.collidable_actors)
        self.disk_asset_state: dict[str, object] | None = None

    def initialize(self) -> None:
        self.initialized = True

    def notify_initialized(self) -> None:
        assert self.initialized
        self.notified = True

    @staticmethod
    def _ref(value: object) -> str:
        assert isinstance(value, Mapping) and isinstance(value.get("refPath"), str)
        return str(value["refPath"])

    def _object_properties(self, ref: str) -> dict[str, object]:
        for store in (
            self.actor_properties,
            self.component_properties,
            self.expression_properties,
            self.texture_properties,
        ):
            if ref in store:
                return store[ref]
        return {}

    def _save_assets_to_disk(self) -> None:
        self.disk_asset_state = copy.deepcopy(
            {
                "assets": self.assets,
                "asset_classes": self.asset_classes,
                "metadata": self.metadata,
                "texture_properties": self.texture_properties,
                "material_expressions": self.material_expressions,
                "expression_properties": self.expression_properties,
                "material_outputs": self.material_outputs,
                "mesh_materials": self.mesh_materials,
                "classes": self.classes,
            }
        )

    def _restore_map_from_disk(self) -> None:
        self.actor_properties = copy.deepcopy(self.disk_actor_properties)
        self.component_properties = copy.deepcopy(self.disk_component_properties)
        self.collidable_actors = set(self.disk_collidable_actors)
        if self.disk_asset_state is not None:
            state = copy.deepcopy(self.disk_asset_state)
            self.assets = state["assets"]
            self.asset_classes = state["asset_classes"]
            self.metadata = state["metadata"]
            self.texture_properties = state["texture_properties"]
            self.material_expressions = state["material_expressions"]
            self.expression_properties = state["expression_properties"]
            self.material_outputs = state["material_outputs"]
            self.mesh_materials = state["mesh_materials"]
            self.classes = state["classes"]
        if self.corrupt_texture_metadata_after_reload and self.metadata:
            first = next(path for path in sorted(self.metadata) if self.builder.SOURCE_SHA_TAG in self.metadata[path])
            self.metadata[first].pop(self.builder.SOURCE_SHA_TAG)

    def call_tool(self, toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        args = dict(arguments)
        self.calls.append((toolset_name, tool_name, args))
        b = self.builder
        if toolset_name == b.ASSET_TOOLS:
            if tool_name == "exists":
                return str(args["path"]) in self.assets or str(args["path"]) in self.folders
            if tool_name == "create_folder":
                self.folders.add(str(args["path"]))
                return True
            if tool_name == "get_asset_class":
                return self.asset_classes[str(args["asset_path"])]
            if tool_name == "get_metadata_tags":
                return dict(self.metadata.get(str(args["asset_path"]), {}))
            if tool_name == "update_metadata_tags":
                self.metadata.setdefault(str(args["asset_path"]), {}).update(dict(args.get("set_tags", {})))
                self.dirty.add(str(args["asset_path"]))
                return None
            if tool_name == "save_assets":
                self._save_assets_to_disk()
                self.dirty.difference_update(str(path) for path in args["asset_paths"])
                return True
            if tool_name == "is_dirty":
                return str(args["asset_path"]) in self.dirty
        if toolset_name == b.SCENE_TOOLS:
            if tool_name == "load_level":
                prior = self.current_level
                self.current_level = str(args["level_path"])
                if prior != b.MAP_PATH and self.current_level == b.MAP_PATH:
                    self._restore_map_from_disk()
                return None
            if tool_name == "get_current_level":
                return {"refPath": self.current_level}
            if tool_name == "save_actor":
                actor = self._ref(args["actor"])
                assert actor == self.terrain_actor
                self.disk_actor_properties = copy.deepcopy(self.actor_properties)
                self.disk_component_properties = copy.deepcopy(self.component_properties)
                self.disk_collidable_actors = set(self.collidable_actors)
                return True
            if tool_name == "get_collision_channels":
                return ["ObjectTypeQuery1", "ObjectTypeQuery2"]
            if tool_name == "find_actors":
                if args.get("root") is not None and args.get("collision_channels"):
                    root = self._ref(args["root"])
                    return [{"refPath": root}] if root in self.collidable_actors else []
                name = str(args.get("name", "")).lower()
                actors = [self.terrain_actor, *self.partition_actors]
                return [{"refPath": actor} for actor in actors if name in self.actor_labels.get(actor, actor).lower()]
        if toolset_name == b.ACTOR_TOOLS:
            actor = self._ref(args["actor"])
            if tool_name == "get_label":
                return self.actor_labels.get(actor, actor.rsplit(".", 1)[-1])
            if tool_name == "get_tags":
                return list(self.actor_properties.setdefault(actor, {"tags": []}).setdefault("tags", []))
            if tool_name == "has_tag":
                return str(args["tag"]) in self.actor_properties.setdefault(actor, {"tags": []}).setdefault("tags", [])
            if tool_name == "add_tag":
                tags = self.actor_properties.setdefault(actor, {"tags": []}).setdefault("tags", [])
                if str(args["tag"]) not in tags:
                    tags.append(str(args["tag"]))
                return True
            if tool_name == "get_actor_bounds":
                return {
                    "min": {"x": -100000.0, "y": -100000.0, "z": -1000.0},
                    "max": {"x": 100000.0, "y": 100000.0, "z": 5000.0},
                    "isValid": True,
                }
            if tool_name == "get_components":
                component_class = self._ref(args["component_type"])
                if actor == self.terrain_actor and component_class == b.STATIC_MESH_COMPONENT_CLASS:
                    return [{"refPath": self.terrain_component}]
                if component_class == b.HISM_COMPONENT_CLASS:
                    if actor == self.partition_actors[0]:
                        return [{"refPath": self.pine_hism}]
                    if actor == self.partition_actors[1]:
                        return [{"refPath": self.birch_hism}]
                return []
        if toolset_name == b.OBJECT_TOOLS:
            ref = self._ref(args["instance"])
            if tool_name == "get_class":
                return {"refPath": self.classes[ref]}
            if tool_name == "get_properties":
                source = self._object_properties(ref)
                return json.dumps({name: source[name] for name in args["properties"]})
            if tool_name == "set_properties":
                values = json.loads(str(args["values"]))
                target = self._object_properties(ref)
                for name, value in values.items():
                    if name == "bodyInstance" and isinstance(target.get(name), dict):
                        target[name].update(value)
                    else:
                        target[name] = value
                self.dirty.add(ref.split(".", 1)[0])
                return True
        if toolset_name == b.TEXTURE_TOOLS and tool_name == "import_file":
            name = str(args["asset_name"])
            path = f"{args['folder_path']}/{name}"
            if path in self.assets:
                raise AssertionError("UE TextureTools.import_file rejects existing destinations")
            ref = f"{path}.{name}"
            self.assets.add(path)
            self.asset_classes[path] = "Texture2D"
            self.texture_properties[ref] = {"sRGB": True, "compressionSettings": "TC_Default"}
            self.dirty.add(path)
            return [{"refPath": ref}]
        if toolset_name == b.MATERIAL_TOOLS:
            if tool_name == "create_material":
                name = str(args["asset_name"])
                path = f"{args['folder_path']}/{name}"
                ref = f"{path}.{name}"
                self.assets.add(path)
                self.asset_classes[path] = "Material"
                self.material_expressions[ref] = []
                self.material_outputs[ref] = {}
                self.dirty.add(path)
                return {"refPath": ref}
            owner = args.get("material_or_function") or args.get("material")
            if owner is not None:
                material_ref = self._ref(owner)
            elif args.get("expression") is not None:
                material_ref = self._ref(args["expression"]).split(":", 1)[0]
            else:
                raise AssertionError(f"material tool call has no owner: {tool_name}")
            if tool_name == "get_expressions":
                return [{"refPath": ref} for ref in self.material_expressions[material_ref]]
            if tool_name == "delete_expression":
                expression = self._ref(args["expression"])
                self.material_expressions[material_ref].remove(expression)
                self.expression_properties.pop(expression, None)
                return None
            if tool_name == "add_expression":
                index = len(self.material_expressions[material_ref])
                expression = f"{material_ref}:MaterialExpressionTextureSample_{index}"
                self.material_expressions[material_ref].append(expression)
                self.expression_properties[expression] = {"texture": "None", "samplerType": "SAMPLERTYPE_Color"}
                self.classes[expression] = b.TEXTURE_SAMPLE_CLASS
                return {"refPath": expression}
            if tool_name == "connect_to_output":
                expression = self._ref(args["expression"])
                self.material_outputs[material_ref][str(args["material_property"])] = (
                    expression,
                    str(args["output_name"]),
                )
                return None
            if tool_name == "get_property_input":
                expression, output_name = self.material_outputs[material_ref].get(
                    str(args["material_property"]), (None, "")
                )
                return {
                    "expression": {"refPath": expression} if expression is not None else "None",
                    "output_name": output_name,
                    "input_name": "",
                }
            if tool_name in {"layout_expressions", "recompile"}:
                return None
        if toolset_name == b.STATIC_MESH_TOOLS:
            mesh_ref = self._ref(args["mesh"])
            if tool_name == "get_material_slots":
                return list(self.mesh_slots[mesh_ref])
            if tool_name == "set_material":
                self.mesh_materials[(mesh_ref, str(args["slot_name"]))] = self._ref(args["material"])
                return True
            if tool_name == "get_material":
                return {"refPath": self.mesh_materials[(mesh_ref, str(args["slot_name"]))]}
            if tool_name == "remove_collisions":
                if mesh_ref == b.VALIDATION_TERRAIN_MESH_REF:
                    self.collidable_actors.discard(self.terrain_actor)
                return True
        raise AssertionError(f"unhandled fake MCP call: {toolset_name}.{tool_name} {args}")


class LandscapeFakeTransport(FakeTransport):
    """Production Landscape variant for complete read-only terrain preflight."""

    def __init__(self, builder: Any) -> None:
        super().__init__(builder, terrain_class=builder.LANDSCAPE_CLASS)
        self.actor_labels[self.terrain_actor] = builder.PRODUCTION_TERRAIN_LABEL
        self.landscape_components = tuple(f"{self.terrain_actor}.LandscapeComponent{index}" for index in range(1024))
        self.landscape_collisions = tuple(
            f"{self.terrain_actor}.LandscapeHeightfieldCollisionComponent{index}" for index in range(1024)
        )
        for index, component in enumerate(self.landscape_components):
            x = index % 32
            y = index // 32
            texture_name = f"HM_{x // 8}_{y // 8}"
            texture_path = f"/Game/Forest/{texture_name}"
            texture_ref = f"{texture_path}.{texture_name}"
            self.assets.add(texture_path)
            self.asset_classes[texture_path] = "Texture2D"
            self.classes[texture_ref] = "/Script/Engine.Texture2D"
            self.classes[component] = builder.LANDSCAPE_COMPONENT_CLASS
            self.component_properties[component] = {
                **copy.deepcopy(self.component_properties[self.terrain_component]),
                "sectionBaseX": x * 126,
                "sectionBaseY": y * 126,
                "componentSizeQuads": 126,
                "subsectionSizeQuads": 63,
                "numSubsections": 2,
                "heightmapTexture": {"refPath": texture_ref},
            }
        for index, component in enumerate(self.landscape_collisions):
            x = index % 32
            y = index // 32
            self.classes[component] = builder.LANDSCAPE_COLLISION_COMPONENT_CLASS
            self.component_properties[component] = {
                **copy.deepcopy(self.component_properties[self.terrain_component]),
                "sectionBaseX": x * 126,
                "sectionBaseY": y * 126,
                "collisionSizeQuads": 126,
                "renderComponentRef": {"refPath": self.landscape_components[index]},
            }

    def call_tool(self, toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        b = self.builder
        if toolset_name == b.ACTOR_TOOLS and tool_name == "get_components":
            actor = self._ref(arguments["actor"])
            component_class = self._ref(arguments["component_type"])
            self.calls.append((toolset_name, tool_name, dict(arguments)))
            if actor == self.terrain_actor and component_class == b.LANDSCAPE_COMPONENT_CLASS:
                return [{"refPath": path} for path in self.landscape_components]
            if actor == self.terrain_actor and component_class == b.LANDSCAPE_COLLISION_COMPONENT_CLASS:
                return [{"refPath": path} for path in self.landscape_collisions]
        return super().call_tool(toolset_name, tool_name, arguments)


def test_plan_binds_exact_basecolor_normal_orm_and_instance_contract(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _validation_inputs(tmp_path, builder)

    plan = builder.build_pbr_plan(inputs)

    assert plan["target"]["terrain_state"] == "preview_only"
    assert plan["target"]["expected_actor_class"] == builder.STATIC_MESH_ACTOR_CLASS
    assert plan["inputs"]["forest_authoring"]["point_count"] == 3
    assert len(plan["inputs"]["forest_authoring"]["expected_partition_coordinates"]) == 64
    materials = {item["id"]: item for item in plan["materials"]}
    assert set(materials) == set(builder.MATERIAL_NAMES)
    assert materials["forest_ground"]["channel_routing"] == builder.EXPECTED_BINDINGS
    assert {texture["compression"] for texture in materials["pine_needles"]["textures"]} == {
        "TC_Default",
        "TC_Normalmap",
        "TC_Masks",
    }


def test_rejects_legacy_manifest_without_explicit_orm_bindings(tmp_path: Path) -> None:
    builder = _builder()
    manifest = _write_manifest(tmp_path, builder)
    plan = _write_forest_authoring_plan(tmp_path, builder)
    _rewrite_manifest(manifest, builder, lambda value: value["materials"][0].pop("bindings"))

    with pytest.raises(builder.PbrInputError, match="exact base/normal/ORM binding"):
        builder.load_pbr_inputs(manifest, builder.VALIDATION_PROFILE, forest_authoring_plan=plan)


def test_rejects_texture_bytes_that_do_not_match_manifest(tmp_path: Path) -> None:
    builder = _builder()
    manifest = _write_manifest(tmp_path, builder)
    plan = _write_forest_authoring_plan(tmp_path, builder)
    (tmp_path / "textures/forest/forest_ground_orm.png").write_bytes(b"tampered")

    with pytest.raises(builder.PbrInputError, match="artifact does not match"):
        builder.load_pbr_inputs(manifest, builder.VALIDATION_PROFILE, forest_authoring_plan=plan)


def test_rejects_tampered_forest_instance_provenance(tmp_path: Path) -> None:
    builder = _builder()
    manifest = _write_manifest(tmp_path, builder)
    plan = _write_forest_authoring_plan(tmp_path, builder)
    points = tmp_path / "forest-points.json"
    points.write_text(points.read_text() + " ", encoding="utf-8")

    with pytest.raises(builder.PbrInputError, match="does not match its bound SHA256"):
        builder.load_pbr_inputs(manifest, builder.VALIDATION_PROFILE, forest_authoring_plan=plan)


@pytest.mark.parametrize(
    ("width", "height", "bit_depth", "color_type"),
    [(257, 257, 16, 0), (4033, 4032, 16, 0), (4033, 4033, 8, 0), (4033, 4033, 16, 2)],
)
def test_production_requires_exact_4033_r16_grayscale_heightmap(
    tmp_path: Path,
    width: int,
    height: int,
    bit_depth: int,
    color_type: int,
) -> None:
    builder = _builder()
    manifest = _write_manifest(tmp_path, builder)
    plan = _write_forest_authoring_plan(tmp_path, builder)
    heightmap = _write_heightmap(tmp_path / "heightfield_r16.png", width, height, bit_depth, color_type)
    receipt = _write_landscape_receipt(tmp_path, builder, heightmap)

    with pytest.raises(builder.PbrInputError, match="4033x4033 16-bit grayscale"):
        builder.load_pbr_inputs(manifest, builder.PRODUCTION_PROFILE, heightmap, plan, receipt)


def test_production_rejects_truncated_png_even_with_valid_ihdr(tmp_path: Path) -> None:
    builder = _builder()
    manifest = _write_manifest(tmp_path, builder)
    plan = _write_forest_authoring_plan(tmp_path, builder)
    heightmap = tmp_path / "heightfield_r16.png"
    heightmap.write_bytes(
        b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", 4033, 4033, 16, 0, 0, 0, 0))
    )
    receipt = _write_landscape_receipt(tmp_path, builder, heightmap)

    with pytest.raises(builder.PbrInputError, match="complete IHDR/IDAT/IEND"):
        builder.load_pbr_inputs(manifest, builder.PRODUCTION_PROFILE, heightmap, plan, receipt)


def test_production_preflight_requires_receipt_and_exact_landscape_grid(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _production_inputs(tmp_path, builder)
    landscape = LandscapeFakeTransport(builder)
    landscape.actor_properties[landscape.terrain_actor]["tags"] = [
        "Production4033",
        builder._production_heightmap_tag(inputs.terrain.heightmap_sha256),
        builder._production_receipt_tag(inputs.terrain.import_receipt_digest),
    ]

    target = builder._preflight_terrain(builder._Session(landscape), inputs)

    assert target.actor_class == builder.LANDSCAPE_CLASS
    assert len(target.render_components) == 1024
    assert len(target.collision_components) == 1024
    landscape.component_properties[landscape.landscape_components[-1]]["sectionBaseX"] = 0
    with pytest.raises(builder.PbrMcpError, match=r"canonical 4033 topology|exact 32x32"):
        builder._preflight_terrain(builder._Session(landscape), inputs)

    wrong_type = FakeTransport(builder)
    wrong_type.actor_labels[wrong_type.terrain_actor] = builder.PRODUCTION_TERRAIN_LABEL
    with pytest.raises(builder.PbrMcpError, match="actual 4033 Landscape"):
        builder._preflight_terrain(builder._Session(wrong_type), inputs)
    assert not any(
        tool in {"set_properties", "import_file", "create_material", "set_material"}
        for _toolset, tool, _args in wrong_type.calls
    )


def test_existing_texture_same_source_is_repaired_without_reimport(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _validation_inputs(tmp_path, builder)
    transport = FakeTransport(builder)
    material = inputs.materials[0]
    texture = material.textures[0]
    transport.assets.add(texture.asset_path)
    transport.asset_classes[texture.asset_path] = "Texture2D"
    transport.texture_properties[texture.asset_ref] = {"sRGB": False, "compressionSettings": "TC_Masks"}
    transport.metadata[texture.asset_path] = {builder.SOURCE_SHA_TAG: texture.sha256}

    outcome = builder._ensure_texture(builder._Session(transport), inputs, material, texture)

    assert outcome == "repaired"
    assert not any(tool == "import_file" for _toolset, tool, _args in transport.calls)
    assert transport.texture_properties[texture.asset_ref]["sRGB"] is True
    assert transport.metadata[texture.asset_path][builder.MANIFEST_DIGEST_TAG] == inputs.manifest_digest


def test_existing_texture_changed_source_fails_closed_without_reimport(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _validation_inputs(tmp_path, builder)
    transport = FakeTransport(builder)
    material = inputs.materials[0]
    texture = material.textures[0]
    transport.assets.add(texture.asset_path)
    transport.asset_classes[texture.asset_path] = "Texture2D"
    transport.texture_properties[texture.asset_ref] = {"sRGB": True, "compressionSettings": "TC_Default"}
    transport.metadata[texture.asset_path] = {builder.SOURCE_SHA_TAG: "0" * 64}

    with pytest.raises(builder.PbrMcpError, match="explicit asset replacement is required"):
        builder._ensure_texture(builder._Session(transport), inputs, material, texture)
    assert not any(tool == "import_file" for _toolset, tool, _args in transport.calls)


def test_apply_builds_graphs_saves_external_actor_and_reaudits_reload(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _validation_inputs(tmp_path, builder)
    transport = FakeTransport(builder)

    result = builder.author_pbr(transport, inputs)

    assert transport.initialized and transport.notified
    assert result["terrain"]["state"] == "preview_only"
    assert result["hism_audit"]["partition_actor_count"] == 64
    assert result["hism_audit"]["instance_count"] == 3
    assert result["hism_audit"]["by_mesh"] == inputs.instances.expected_by_mesh
    assert any(tool == "save_actor" for _toolset, tool, _args in transport.calls)
    terrain = transport.component_properties[transport.terrain_component]
    assert terrain["bodyInstance"]["collisionProfileName"] == "NoCollision"
    assert terrain["bodyInstance"]["collisionEnabled"] == "QueryAndPhysics"
    assert terrain["bCanEverAffectNavigation"] is False
    assert transport.terrain_actor not in transport.collidable_actors
    for material in inputs.materials:
        builder._audit_material(builder._Session(transport), inputs, material)
        for texture in material.textures:
            builder._audit_texture(builder._Session(transport), inputs, material, texture)
    assert not any(
        toolset == builder.STATIC_MESH_TOOLS and tool == "import_file" for toolset, tool, _args in transport.calls
    )


def test_reload_audit_rejects_texture_metadata_that_did_not_persist(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _validation_inputs(tmp_path, builder)
    transport = FakeTransport(builder)
    transport.corrupt_texture_metadata_after_reload = True

    with pytest.raises(builder.PbrMcpError, match="texture import settings or provenance did not persist"):
        builder.author_pbr(transport, inputs)


def test_hism_audit_requires_all_cells_exact_counts_and_nocollision(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _validation_inputs(tmp_path, builder)
    transport = FakeTransport(builder)
    transport.partition_actors = transport.partition_actors[:-1]
    with pytest.raises(builder.PbrMcpError, match="all exact 64 loaded"):
        builder._audit_hism_authority(builder._Session(transport), inputs)

    transport = FakeTransport(builder)
    transport.component_properties[transport.pine_hism]["perInstanceSMData"].append({"transform": {}})
    with pytest.raises(builder.PbrMcpError, match="totals do not match"):
        builder._audit_hism_authority(builder._Session(transport), inputs)

    transport = FakeTransport(builder)
    transport.component_properties[transport.pine_hism]["bodyInstance"]["collisionProfileName"] = "BlockAll"
    with pytest.raises(builder.PbrMcpError, match="did not retain NoCollision"):
        builder._audit_hism_authority(builder._Session(transport), inputs)
