# ruff: noqa: S101

"""Deterministic, no-network tests for the Forest_HF UE MCP adapter."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any, Mapping, cast

import pytest

from sim.tools.worlds.forest_hf.generate import TerrainSpec, generate_forest_hf

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "sim/runtime/visual/RobotSimUE/Scripts/build_forest_hf_mcp.py"
OFFLINE_SCRIPT_PATH = REPO_ROOT / "sim/runtime/visual/RobotSimUE/Scripts/build_forest_hf.py"
PACKAGE_ROOT = REPO_ROOT / "sim/packages/worlds/forest_hf/2.0.0"


def _builder():
    spec = importlib.util.spec_from_file_location("lingtu_forest_hf_mcp_builder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _offline_builder():
    spec = importlib.util.spec_from_file_location("lingtu_forest_hf_offline_builder", OFFLINE_SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, value: object, builder: Any) -> Path:
    path.write_bytes(builder.canonical_json_bytes(value))
    return path


def _contract(builder: Any) -> dict[str, object]:
    group_base = {
        "component_class": "HierarchicalInstancedStaticMeshComponent",
        "classification": "VisualOnly",
        "collision_profile": "NoCollision",
        "collision_enabled": False,
        "generate_overlap_events": False,
        "simulate_physics": False,
        "can_ever_affect_navigation": False,
    }
    body: dict[str, object] = {
        "schema": builder.CONTRACT_SCHEMA,
        "world_package": builder.WORLD_PACKAGE,
        "seed": builder.FIXED_SEED,
        "generation": {
            "mode": "offline_baked_hism",
            "deterministic": True,
            "runtime_generation": False,
        },
        "tree_scale_contract": {"pcg_scale_factor": 4.5},
        "foliage_density_contract": {"accepted_point_count": 2},
        "target": {
            "map": builder.MAP_PATH,
            "asset_root": "/Game/RobotSim/Worlds/ForestHF2km",
            "world_partition": {"enabled": True},
        },
        "authority": {
            "physics": "mujoco",
            "raycast": "mujoco",
            "render_actors": "VisualOnly",
            "collision_profile": "NoCollision",
            "render_meshes_are_colliders": False,
        },
        "authoring_density_cells": [
            {
                "stable_id": "forest.cell.x00.y00",
                "hism_groups": [
                    {
                        **group_base,
                        "mesh_slot": "forest.asset.birch",
                        "instances": [
                            {
                                "source_stable_id": "forest.foliage.birch",
                                "stable_entity_id": "forest.ue.birch",
                                "unreal_transform": {
                                    "location_cm": [-100.0, 0.0, 0.0],
                                    "rotation_deg": [0.0, 0.0, 0.0],
                                    "scale_xyz": [1.0, 1.0, 1.0],
                                },
                            }
                        ],
                    },
                    {
                        **group_base,
                        "mesh_slot": "forest.asset.pine",
                        "instances": [
                            {
                                "source_stable_id": "forest.foliage.pine",
                                "stable_entity_id": "forest.ue.pine",
                                "unreal_transform": {
                                    "location_cm": [100.0, 0.0, 0.0],
                                    "rotation_deg": [0.0, 0.0, 90.0],
                                    "scale_xyz": [1.0, 1.0, 1.0],
                                },
                            }
                        ],
                    },
                ],
            }
        ],
    }
    return {
        **body,
        "content_digest": hashlib.sha256(builder.canonical_json_bytes(body)).hexdigest(),
    }


def _points(builder: Any) -> dict[str, object]:
    groups = []
    for slot, x, yaw in (
        ("forest.asset.birch", -100.0, 0.0),
        ("forest.asset.pine", 100.0, 90.0),
    ):
        stable_id = f"forest.foliage.{slot.rsplit('.', 1)[-1]}"
        groups.append(
            {
                "mesh_slot": slot,
                "point_count": 1,
                "source_bindings": [
                    {
                        "metadataEntry": 0,
                        "mesh_slot": slot,
                        "source_stable_id": stable_id,
                    }
                ],
                "json_params": {
                    "pointsToCreate": [
                        {
                            "transform": {
                                "location": {"x": x, "y": 0.0, "z": 10.0},
                                "rotation": {"x": 0.0, "y": 0.0, "z": yaw},
                                "scale": {"x": 4.5, "y": 4.5, "z": 4.5},
                            },
                            "density": 1.0,
                            "boundsMin": {"x": -50.0, "y": -50.0, "z": 0.0},
                            "boundsMax": {"x": 50.0, "y": 50.0, "z": 200.0},
                            "color": {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0},
                            "steepness": 0.0,
                            "seed": int.from_bytes(hashlib.sha256(stable_id.encode()).digest()[:4], "big")
                            & 0x7FFFFFFF,
                            "metadataEntry": 0,
                        }
                    ],
                    "coordinateSpace": "World",
                    "bCullPointsOutsideVolume": True,
                },
            }
        )
    body: dict[str, object] = {
        "schema": builder.CREATE_POINTS_SCHEMA,
        "world_package": builder.WORLD_PACKAGE,
        "seed": builder.FIXED_SEED,
        "contract_content_digest": _contract(builder)["content_digest"],
        "selection_policy": builder.EXACT_MESH_SLOT_SELECTION_POLICY,
        "point_count": 2,
        "slot_counts": {"forest.asset.birch": 1, "forest.asset.pine": 1},
        "groups": groups,
    }
    return {**body, "content_digest": hashlib.sha256(builder.canonical_json_bytes(body)).hexdigest()}


def _inputs(tmp_path: Path, builder: Any, *, include_boulder: bool = False):
    contract_path = _write_json(tmp_path / "forest-hf.bake.json", _contract(builder), builder)
    points_path = _write_json(tmp_path / "create-points.json", _points(builder), builder)
    pine = tmp_path / "forest_asset_pine.fbx"
    birch = tmp_path / "forest_asset_birch.fbx"
    pine.write_bytes(b"pine-fbx")
    birch.write_bytes(b"birch-fbx")
    bindings = [f"forest.asset.pine={pine}", f"forest.asset.birch={birch}"]
    if include_boulder:
        boulder = tmp_path / "forest_asset_boulder.fbx"
        boulder.write_bytes(b"boulder-fbx")
        bindings.append(f"forest.asset.boulder={boulder}")
    return builder.load_authoring_inputs(
        contract_path,
        points_path,
        bindings,
    )


class _FakeResponse:
    def __init__(self, payload: str, headers: Mapping[str, str] | None = None) -> None:
        self._payload = payload.encode()
        self.headers = dict(headers or {})

    def __enter__(self):
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def read(self) -> bytes:
        return self._payload


class _SequentialOpener:
    def __init__(self, responses: list[_FakeResponse]) -> None:
        self.responses = responses
        self.requests: list[tuple[Any, float]] = []

    def __call__(self, request: Any, timeout: float) -> _FakeResponse:
        self.requests.append((request, timeout))
        return self.responses.pop(0)


class _FakeTransport:
    def __init__(
        self,
        builder: Any,
        *,
        existing: bool,
        inputs: Any | None = None,
        fail_tool: str | None = None,
        false_tool: str | None = None,
        legacy_graph: bool = False,
        fail_import_paths: set[str] | None = None,
        fail_delete_paths: set[str] | None = None,
        fail_save_paths: set[str] | None = None,
        legacy_canonical_assets: bool = False,
        selector_drift: tuple[tuple[str | int, ...], object] | None = None,
    ) -> None:
        self.builder = builder
        self.existing = existing
        self.fail_tool = fail_tool
        self.false_tool = false_tool
        self.fail_import_paths = set(fail_import_paths or ())
        self.fail_delete_paths = set(fail_delete_paths or ())
        self.fail_save_paths = set(fail_save_paths or ())
        self.selector_drift = selector_drift
        self.initialized = False
        self.notified = False
        self.calls: list[tuple[str, str, dict[str, object]]] = []
        self.created_paths: set[str] = set()
        self.deleted_paths: set[str] = set()
        self.metadata: dict[str, dict[str, str]] = {}
        self.referencers: dict[str, list[str]] = {}
        self.properties: dict[str, dict[str, object]] = {}
        self.actor_transform = dict(builder.CANONICAL_VOLUME_TRANSFORM)
        self.hism_instance_counts = {"Birch": 1, "Pine": 1}
        self.current_level: str | None = None
        self.generated_available = False
        self.generated_persisted = False
        self.persist_generated_on_save = True
        self.existing_asset_paths: set[str] = set()
        self.hism_mesh_refs: dict[str, str] = {}
        if inputs is not None:
            self.hism_mesh_refs = {
                slot.slot.rsplit(".", 1)[-1].title(): slot.asset_ref
                for slot in inputs.slots
                if slot.slot in builder.REQUIRED_TREE_SLOTS
            }
        self.nodes: dict[str, str] = {
            builder.OUTPUT_NODE_NAME: f"{builder.GRAPH_OBJECT_PATH}:{builder.OUTPUT_NODE_NAME}"
        }
        self.edges: list[dict[str, object]] = []
        if existing:
            assert inputs is not None
            self.existing_asset_paths.update(
                {
                    builder.MAP_PATH,
                    builder.ASSET_FOLDER,
                    builder.GRAPH_FOLDER,
                    builder.GRAPH_ASSET_PATH,
                }
            )
            for slot in inputs.slots:
                if legacy_canonical_assets:
                    self.existing_asset_paths.add(
                        f"{builder.ASSET_FOLDER}/{builder.SLOT_ASSET_NAMES[slot.slot]}"
                    )
                else:
                    self.existing_asset_paths.add(slot.asset_path)
                    self.metadata[slot.asset_path] = {
                        builder.SOURCE_SHA_TAG: slot.sha256,
                        builder.SOURCE_SLOT_TAG: slot.slot,
                        builder.AUTHORITY_TAG: "VisualOnly;NoCollision;MuJoCoPhysicsAndRaycast",
                    }
                    self.referencers[slot.asset_path] = [builder.GRAPH_ASSET_PATH, builder.MAP_PATH]
            if legacy_graph:
                branch_names = [
                    (builder.LEGACY_CREATE_POINTS_NODE_NAME, builder.LEGACY_SPAWNER_NODE_NAME)
                ]
            else:
                branch_names = [
                    (builder.CREATE_POINTS_NODE_NAMES[slot], builder.SPAWNER_NODE_NAMES[slot])
                    for slot in sorted(builder.REQUIRED_TREE_SLOTS)
                ]
            for points_name, spawner_name in branch_names:
                self.nodes[points_name] = f"{builder.GRAPH_OBJECT_PATH}:{points_name}"
                self.nodes[spawner_name] = f"{builder.GRAPH_OBJECT_PATH}:{spawner_name}"
                self.edges.extend(
                    [
                        self._edge(self.nodes[points_name], "Out", self.nodes[spawner_name], "In"),
                        self._edge(
                            self.nodes[spawner_name],
                            "Out",
                            self.nodes[builder.OUTPUT_NODE_NAME],
                            "Out",
                        ),
                    ]
                )

    @staticmethod
    def _edge(source: str, source_pin: str, target: str, target_pin: str) -> dict[str, object]:
        return {
            "srcNode": {"refPath": source},
            "srcPin": source_pin,
            "destNode": {"refPath": target},
            "destPin": target_pin,
        }

    def initialize(self) -> None:
        self.initialized = True

    def notify_initialized(self) -> None:
        assert self.initialized
        self.notified = True

    def _structure(self) -> dict[str, object]:
        return {
            "nodes": [{"name": name, "path": {"refPath": path}} for name, path in sorted(self.nodes.items())],
            "edges": list(self.edges),
        }

    def _expanded_selector_readback(self, stored: Mapping[str, object]) -> dict[str, object]:
        expanded = cast(dict[str, Any], json.loads(json.dumps(stored)))
        entries = expanded.pop("MeshEntries")
        expanded["meshEntries"] = entries
        expanded["materialOverrideAttributes"] = expanded.pop("MaterialOverrideAttributes")
        for entry in entries:
            descriptor = entry["descriptor"]
            descriptor.update(
                {
                    "overrideMaterials": [],
                    "overlayMaterial": "None",
                    "runtimeVirtualTextures": [],
                    "hash": 0,
                    "lightmapType": "Default",
                    "bReceivesDecals": True,
                    "detailMode": "DM_Low",
                }
            )
            descriptor["bodyInstance"].update(
                {
                    "objectType": "ECC_WorldStatic",
                    "positionSolverIterationCount": 8,
                    "velocitySolverIterationCount": 2,
                    "bUseCCD": False,
                    "massScale": 1,
                }
            )
        if self.selector_drift is not None:
            path, value = self.selector_drift
            target: Any = expanded
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
        return expanded

    def _exists(self, path: str) -> bool:
        if path in self.deleted_paths:
            return False
        if path == self.builder.OPEN_WORLD_TEMPLATE:
            return True
        if path in self.created_paths:
            return True
        if self.existing:
            return path in self.existing_asset_paths
        return False

    def call_tool(self, toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        assert self.notified
        args = dict(arguments)
        self.calls.append((toolset_name, tool_name, args))
        if tool_name == self.fail_tool:
            raise RuntimeError(f"synthetic failure in {tool_name}")
        if tool_name == self.false_tool:
            return False
        if tool_name == "exists":
            return self._exists(str(args["path"]))
        if tool_name == "create_folder":
            self.created_paths.add(str(args["path"]))
            return True
        if tool_name == "load_level":
            level_path = str(args["level_path"])
            if level_path == self.builder.OPEN_WORLD_TEMPLATE:
                self.generated_available = False
            elif level_path == self.builder.MAP_PATH and self.current_level == self.builder.OPEN_WORLD_TEMPLATE:
                self.generated_available = self.generated_persisted
            self.current_level = level_path
            return True
        if tool_name == "get_current_level":
            assert self.current_level is not None
            return {"refPath": f"{self.current_level}.{self.current_level.rsplit('/', 1)[-1]}"}
        if tool_name == "duplicate":
            source = str(args["path"])
            destination = str(args["new_path"])
            self.created_paths.add(destination)
            self.deleted_paths.discard(destination)
            if source in self.metadata:
                self.metadata[destination] = dict(self.metadata[source])
            return True
        if tool_name == "import_file":
            asset_path = f"{args['folder_path']}/{args['asset_name']}"
            if asset_path in self.fail_import_paths:
                raise RuntimeError(f"synthetic import failure for {asset_path}")
            if self._exists(asset_path):
                raise RuntimeError(f"{args['asset_name']} already exists")
            self.created_paths.add(asset_path)
            self.deleted_paths.discard(asset_path)
            return [{"refPath": f"{asset_path}.{args['asset_name']}"}]
        if tool_name == "can_edit_asset":
            return True
        if tool_name == "get_referencers":
            return list(self.referencers.get(str(args["asset_path"]), []))
        if tool_name == "get_asset_class":
            return "StaticMesh"
        if tool_name == "delete":
            path = str(args["path"])
            if path in self.fail_delete_paths:
                return False
            if not self._exists(path):
                return False
            self.deleted_paths.add(path)
            self.created_paths.discard(path)
            self.metadata.pop(path, None)
            self.referencers.pop(path, None)
            return True
        if tool_name == "get_metadata_tags":
            return dict(self.metadata.get(str(args["asset_path"]), {}))
        if tool_name == "update_metadata_tags":
            set_tags = args["set_tags"]
            assert isinstance(set_tags, Mapping)
            self.metadata[str(args["asset_path"])] = {str(key): str(value) for key, value in set_tags.items()}
            return None
        if tool_name == "CreateGraph":
            self.created_paths.add(self.builder.GRAPH_ASSET_PATH)
            return {"refPath": self.builder.GRAPH_OBJECT_PATH}
        if tool_name == "GetGraphStructure":
            return self._structure()
        if tool_name == "ListNativeNodes":
            return ["创建点", "静态网格体生成器"]
        if tool_name == "AddNode":
            name = str(args["nodeName"])
            path = f"{self.builder.GRAPH_OBJECT_PATH}:{name}"
            self.nodes[name] = path
            return {"refPath": path}
        if tool_name == "RemoveNode":
            assert args == {
                "graph": {"refPath": self.builder.GRAPH_OBJECT_PATH},
                "node": args["node"],
            }
            path = str(args["node"]["refPath"])  # type: ignore[index]
            name = path.rsplit(":", 1)[-1]
            self.nodes.pop(name, None)
            self.edges = [
                edge
                for edge in self.edges
                if edge["srcNode"] != {"refPath": path} and edge["destNode"] != {"refPath": path}
            ]
            return True
        if tool_name == "ConnectNodePins":
            self.edges.append(
                self._edge(
                    str(args["fromNode"]["refPath"]),  # type: ignore[index]
                    str(args["fromPinLabel"]),
                    str(args["toNode"]["refPath"]),  # type: ignore[index]
                    str(args["toPinLabel"]),
                )
            )
            return []
        if tool_name == "GetNodeInfo":
            node_path = str(args["node"]["refPath"])  # type: ignore[index]
            return {
                "paramOverrides": {
                    "meshSelectorParameters": {"refPath": f"{node_path}MeshSelector"}
                }
            }
        if tool_name == "ListGraphInstances":
            if self.existing:
                return [
                    {
                        "actor": {"refPath": "/Game/RobotSim/Maps/Forest_HF_2km:PersistentLevel.PCGVolume_0"},
                        "graph": {"refPath": self.builder.GRAPH_OBJECT_PATH},
                    }
                ]
            return []
        if tool_name == "SpawnGraphInstance":
            return {"refPath": "/Game/RobotSim/Maps/Forest_HF_2km:PersistentLevel.PCGVolume_0"}
        if tool_name == "set_actor_transform":
            xform = args["xform"]
            assert isinstance(xform, Mapping)
            self.actor_transform = dict(xform)
            return True
        if tool_name == "get_actor_transform":
            return self.actor_transform
        if tool_name == "get_actor_bounds":
            return self.builder.CANONICAL_VOLUME_BOUNDS
        if tool_name == "find_actors":
            if not self.generated_available:
                return []
            return [{"refPath": ("/Game/RobotSim/Maps/Forest_HF_2km:PersistentLevel.PCGPartitionGridActor_25600_0_0")}]
        if tool_name == "get_components":
            return [
                {"refPath": "/FakeHISM/Birch"},
                {"refPath": "/FakeHISM/Pine"},
            ]
        if tool_name == "set_properties":
            self.properties[str(args["instance"]["refPath"])] = json.loads(str(args["values"]))  # type: ignore[index]
            return True
        if tool_name == "get_properties":
            instance_path = str(args["instance"]["refPath"])  # type: ignore[index]
            if instance_path.startswith("/FakeHISM/"):
                species = "Birch" if instance_path.endswith("Birch") else "Pine"
                return json.dumps(
                    {
                        "staticMesh": {"refPath": self.hism_mesh_refs[species]},
                        "perInstanceSMData": [
                            {"transform": {}} for _index in range(self.hism_instance_counts[species])
                        ],
                        "bodyInstance": {
                            "collisionProfileName": "NoCollision",
                            "collisionEnabled": "NoCollision",
                            "bSimulatePhysics": False,
                        },
                        "bGenerateOverlapEvents": False,
                        "bCanEverAffectNavigation": False,
                        "componentTags": ["LingTuForest", "VisualOnly", "NoCollision"],
                    }
                )
            stored = self.properties.get(instance_path, {})
            if "MeshEntries" in stored:
                return json.dumps(self._expanded_selector_readback(stored))
            property_names = args["properties"]
            assert isinstance(property_names, list)
            return json.dumps({str(name): stored.get(str(name)) for name in property_names})
        if tool_name == "ExecuteGraphInstance":
            self.generated_available = True
            return []
        if tool_name == "save_assets":
            asset_paths = args["asset_paths"]
            assert isinstance(asset_paths, list)
            if self.fail_save_paths.intersection(str(path) for path in asset_paths):
                return False
            if self.persist_generated_on_save:
                self.generated_persisted = self.generated_available
            return True
        if tool_name == "is_dirty":
            return False
        return True


def _tool_names(transport: _FakeTransport) -> list[str]:
    return [tool_name for _toolset, tool_name, _arguments in transport.calls]


def test_http_transport_negotiates_session_and_wraps_call_tool() -> None:
    builder = _builder()
    opener = _SequentialOpener(
        [
            _FakeResponse(
                json.dumps({"jsonrpc": "2.0", "id": 1, "result": {"protocolVersion": "2025-06-18"}}),
                {"Mcp-Session-Id": "session-forest"},
            ),
            _FakeResponse(""),
            _FakeResponse(
                json.dumps(
                    {
                        "jsonrpc": "2.0",
                        "id": 2,
                        "result": {"content": [{"type": "text", "text": json.dumps({"returnValue": True})}]},
                    }
                )
            ),
        ]
    )
    transport = builder.HttpMcpTransport(opener=opener)

    transport.initialize()
    transport.notify_initialized()
    assert transport.call_tool(builder.ASSET_TOOLS, "exists", {"path": builder.MAP_PATH}) is True
    assert transport.session_id == "session-forest"

    messages = [json.loads(request.data) for request, _timeout in opener.requests]
    assert [message["method"] for message in messages] == [
        "initialize",
        "notifications/initialized",
        "tools/call",
    ]
    call_arguments = messages[-1]["params"]["arguments"]
    assert call_arguments == {
        "toolset_name": builder.ASSET_TOOLS,
        "tool_name": "exists",
        "arguments": {"path": builder.MAP_PATH},
    }
    session_headers = [
        {key.lower(): value for key, value in request.header_items()} for request, _timeout in opener.requests[1:]
    ]
    assert all(headers["mcp-session-id"] == "session-forest" for headers in session_headers)


def test_default_cli_is_deterministic_dry_run_and_opens_no_transport(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)

    class _ForbiddenTransport:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            raise AssertionError("dry-run must not open MCP")

    monkeypatch.setattr(builder, "HttpMcpTransport", _ForbiddenTransport)
    argv = [
        "--contract",
        str(inputs.contract_path),
        "--create-points",
        str(inputs.points_path),
        *[argument for slot in inputs.slots for argument in ("--slot-fbx", f"{slot.slot}={slot.source_path}")],
    ]
    assert builder.main(argv) == 0
    first = capsys.readouterr().out
    assert builder.main(argv) == 0
    second = capsys.readouterr().out

    assert first == second
    assert "LINGTU_FOREST_HF_MCP_MODE=dry_run" in first
    assert builder.PLAN_SCHEMA in first
    assert '"apply_required": true' in first


def test_production_offline_builder_output_loads_with_exact_slot_counts(tmp_path: Path) -> None:
    builder = _builder()
    offline = _offline_builder()
    projection, terrain, routes = (
        json.loads(path.read_text(encoding="utf-8"))
        for path in (
            PACKAGE_ROOT / "visual/ue_projection.json",
            PACKAGE_ROOT / "terrain.recipe.json",
            PACKAGE_ROOT / "routes/forest.routes.json",
        )
    )
    contract = offline.build_offline_bake_contract(projection, terrain, routes)
    materialized = generate_forest_hf(tmp_path / "materialized", spec=TerrainSpec(resolution_px=17))
    points = offline.build_create_points_json_params(contract, terrain, materialized.package_root)
    contract_path = _write_json(tmp_path / "production-contract.json", contract, builder)
    points_path = _write_json(tmp_path / "production-points.json", points, builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")

    inputs = builder.load_authoring_inputs(
        contract_path,
        points_path,
        [f"pine={pine}", f"birch={birch}"],
    )

    assert inputs.point_count == 18707
    assert inputs.slot_point_counts == {
        "forest.asset.birch": 9469,
        "forest.asset.pine": 9238,
    }
    assert [group.slot for group in inputs.point_groups] == [
        "forest.asset.birch",
        "forest.asset.pine",
    ]


def test_short_slot_aliases_canonicalize_to_stable_asset_ids(tmp_path: Path) -> None:
    builder = _builder()
    contract_path = _write_json(tmp_path / "contract.json", _contract(builder), builder)
    points_path = _write_json(tmp_path / "points.json", _points(builder), builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    boulder = tmp_path / "boulder.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")
    boulder.write_bytes(b"boulder")

    inputs = builder.load_authoring_inputs(
        contract_path,
        points_path,
        [f"pine={pine}", f"birch={birch}", f"boulder={boulder}"],
    )

    assert [slot.slot for slot in inputs.slots] == [
        "forest.asset.birch",
        "forest.asset.boulder",
        "forest.asset.pine",
    ]
    assert all(
        slot.asset_name
        == f"{builder.SLOT_ASSET_NAMES[slot.slot]}_{slot.sha256[:builder.CONTENT_ADDRESS_DIGEST_LENGTH]}"
        for slot in inputs.slots
    )


def test_alias_and_full_slot_id_are_rejected_as_duplicate(tmp_path: Path) -> None:
    builder = _builder()
    contract_path = _write_json(tmp_path / "contract.json", _contract(builder), builder)
    points_path = _write_json(tmp_path / "points.json", _points(builder), builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")

    with pytest.raises(
        builder.AuthoringInputError,
        match=r"duplicate Forest_HF asset slot: forest\.asset\.pine",
    ):
        builder.load_authoring_inputs(
            contract_path,
            points_path,
            [f"pine={pine}", f"forest.asset.pine={pine}", f"birch={birch}"],
        )


def test_cli_help_documents_short_and_stable_slot_ids(capsys: pytest.CaptureFixture[str]) -> None:
    builder = _builder()

    with pytest.raises(SystemExit) as exit_info:
        builder._parse_args(["--help"])

    assert exit_info.value.code == 0
    output = capsys.readouterr().out
    assert "pine/birch" in output
    assert "forest.asset.<slot>" in output


def test_new_assets_follow_deterministic_authoring_sequence(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder, include_boulder=True)
    transport = _FakeTransport(builder, existing=False, inputs=inputs)

    result = builder.author_forest_hf(transport, inputs)

    names = _tool_names(transport)
    assert transport.initialized and transport.notified
    assert result["map"]["outcome"] == "created"
    assert result["graph"]["outcome"] == "created"
    assert result["volume"]["outcome"] == "created"
    assert names.count("import_file") == 3
    assert names.count("remove_collisions") == 3
    assert names.count("AddNode") == 4
    assert names.count("ConnectNodePins") == 4
    assert names.count("SpawnGraphInstance") == 1
    assert names.index("ExecuteGraphInstance") < names.index("save_assets") < names.index("is_dirty")
    assert names.index("duplicate") < names.index("load_level")
    assert names.index("CreateGraph") < names.index("AddNode")
    assert names.index("AddNode") < names.index("ConnectNodePins")
    assert names.index("ConnectNodePins") < names.index("ExecuteGraphInstance")

    add_calls = [arguments for _toolset, name, arguments in transport.calls if name == "AddNode"]
    assert [call["nativeNodeType"] for call in add_calls] == [
        "创建点",
        "静态网格体生成器",
        "创建点",
        "静态网格体生成器",
    ]
    point_group_params = {
        group.slot: group.json_params for group in inputs.point_groups
    }
    added_point_params = {
        slot: json.loads(
            str(
                next(
                    call["jsonParams"]
                    for call in add_calls
                    if call["nodeName"] == builder.CREATE_POINTS_NODE_NAMES[slot]
                )
            )
        )
        for slot in sorted(builder.REQUIRED_TREE_SLOTS)
    }
    assert added_point_params == point_group_params
    property_payloads = [
        json.loads(str(arguments["values"]))
        for _toolset, name, arguments in transport.calls
        if name == "set_properties"
    ]
    assert any(
        entry["descriptor"]["bodyInstance"]["collisionProfileName"] == "NoCollision"
        for payload in property_payloads
        for entry in payload.get("MeshEntries", [])
    )
    selector_payloads = [payload for payload in property_payloads if "MeshEntries" in payload]
    assert len(selector_payloads) == 2
    assert all(len(payload["MeshEntries"]) == 1 for payload in selector_payloads)
    assert all(payload["MeshEntries"][0]["weight"] == 1 for payload in selector_payloads)
    selector_refs = {
        payload["MeshEntries"][0]["descriptor"]["staticMesh"]["refPath"]
        for payload in selector_payloads
    }
    assert selector_refs == {
        slot.asset_ref for slot in inputs.slots if slot.slot in builder.REQUIRED_TREE_SLOTS
    }
    assert all("Boulder" not in ref for ref in selector_refs)
    assert result["graph"]["spawner_slots"] == ["forest.asset.birch", "forest.asset.pine"]
    assert result["graph"]["selection_policy"] == builder.EXACT_MESH_SLOT_SELECTION_POLICY
    assert result["import_only_slots"] == ["forest.asset.boulder"]
    assert any(payload.get("bActorEnableCollision") is False for payload in property_payloads)
    save_call = next(arguments for _toolset, name, arguments in transport.calls if name == "save_assets")
    saved_asset_paths = save_call["asset_paths"]
    assert isinstance(saved_asset_paths, list)
    assert saved_asset_paths == sorted(saved_asset_paths)
    assert save_call["asset_paths"] == result["saved_assets"]


def test_existing_assets_are_updated_without_duplicate_import_or_spawn(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    transport.actor_transform = {
        "location": {"x": 5000.0, "y": 0.0, "z": 0.0},
        "rotation": {"pitch": 0.0, "yaw": 45.0, "roll": 0.0},
        "scale": {"x": 1.0, "y": 1.0, "z": 1.0},
    }

    result = builder.author_forest_hf(transport, inputs)

    names = _tool_names(transport)
    assert result["map"]["outcome"] == "reused"
    assert result["graph"]["outcome"] == "reused"
    assert result["volume"]["outcome"] == "reused"
    assert not {
        "create_folder",
        "duplicate",
        "import_file",
        "CreateGraph",
        "AddNode",
        "ConnectNodePins",
        "SpawnGraphInstance",
    }.intersection(names)
    assert names.count("UpdateNode") == 4
    assert names.count("remove_collisions") == 2
    assert names.index("ExecuteGraphInstance") < names.index("save_assets") < names.index("is_dirty")
    assert transport.actor_transform == builder.CANONICAL_VOLUME_TRANSFORM


@pytest.mark.parametrize(
    ("path", "drifted_value"),
    [
        (("meshEntries",), []),
        (("meshEntries", 0, "weight"), 2),
        (("meshEntries", 0, "descriptor", "componentTags"), ["LingTuForest", "VisualOnly", "NoCollision", "Extra"]),
        (("meshEntries", 0, "descriptor", "additionalCommaSeparatedTags"), "LingTuForest,VisualOnly"),
        (("meshEntries", 0, "descriptor", "staticMesh", "refPath"), "/Game/Wrong.Wrong"),
        (("meshEntries", 0, "descriptor", "componentClass", "refPath"), "/Script/Engine.StaticMeshComponent"),
        (("meshEntries", 0, "descriptor", "mobility"), "Movable"),
        (("meshEntries", 0, "descriptor", "bodyInstance", "collisionProfileName"), "BlockAll"),
        (("meshEntries", 0, "descriptor", "bodyInstance", "collisionEnabled"), "QueryAndPhysics"),
        (("meshEntries", 0, "descriptor", "bodyInstance", "bSimulatePhysics"), True),
        (("meshEntries", 0, "descriptor", "bodyInstance", "bEnableGravity"), True),
        (("meshEntries", 0, "descriptor", "bodyInstance", "bNotifyRigidBodyCollision"), True),
        (("meshEntries", 0, "descriptor", "bHasCustomNavigableGeometry"), "EvenIfNotCollidable"),
        (("meshEntries", 0, "descriptor", "bUseDefaultCollision"), True),
        (("meshEntries", 0, "descriptor", "bGenerateOverlapEvents"), True),
        (("meshEntries", 0, "descriptor", "bCanEverAffectNavigation"), True),
        (("meshEntries", 0, "descriptor", "bForceNavigationObstacle"), True),
        (("meshEntries", 0, "descriptor", "bCastShadow"), False),
        (("meshEntries", 0, "descriptor", "bCastDynamicShadow"), False),
        (("meshEntries", 0, "descriptor", "bCastStaticShadow"), False),
        (("meshEntries", 0, "descriptor", "bVisible"), False),
        (("meshEntries", 0, "descriptor", "bHiddenInGame"), True),
        (("meshEntries", 0, "descriptor", "bIncludeInHLOD"), False),
        (("meshEntries", 0, "descriptor", "instanceStartCullDistance"), 0),
        (("meshEntries", 0, "descriptor", "instanceEndCullDistance"), 1000),
        (("bUseAttributeMaterialOverrides",), True),
        (("materialOverrideAttributes",), ["UnsafeMaterial"]),
    ],
    ids=[
        "entry-count",
        "weight",
        "tags",
        "comma-tags",
        "static-mesh",
        "component-class",
        "mobility",
        "collision-profile",
        "collision-enabled",
        "simulate-physics",
        "gravity",
        "collision-notify",
        "navigation-geometry",
        "default-collision",
        "overlap-events",
        "affect-navigation",
        "navigation-obstacle",
        "cast-shadow",
        "dynamic-shadow",
        "static-shadow",
        "visible",
        "hidden-in-game",
        "hlod",
        "start-cull",
        "end-cull",
        "attribute-material-overrides",
        "material-override-attributes",
    ],
)
def test_expanded_selector_readback_fails_each_safety_semantic_drift(
    tmp_path: Path,
    path: tuple[str | int, ...],
    drifted_value: object,
) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(
        builder,
        existing=False,
        inputs=inputs,
        selector_drift=(path, drifted_value),
    )

    with pytest.raises(builder.McpToolError, match="safety semantic drift"):
        builder.author_forest_hf(transport, inputs)

    assert "ExecuteGraphInstance" not in _tool_names(transport)
    assert "save_assets" not in _tool_names(transport)


def test_changed_fbx_content_selects_a_new_asset_path(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    old_by_slot = {slot.slot: slot for slot in inputs.slots}
    old_by_slot["forest.asset.pine"].source_path.write_bytes(b"pine-fbx-v2")

    changed = builder.load_authoring_inputs(
        inputs.contract_path,
        inputs.points_path,
        [f"{slot.slot}={slot.source_path}" for slot in inputs.slots],
    )
    changed_by_slot = {slot.slot: slot for slot in changed.slots}

    assert changed_by_slot["forest.asset.birch"].asset_path == old_by_slot["forest.asset.birch"].asset_path
    assert changed_by_slot["forest.asset.pine"].sha256 != old_by_slot["forest.asset.pine"].sha256
    assert changed_by_slot["forest.asset.pine"].asset_path != old_by_slot["forest.asset.pine"].asset_path


def test_unowned_legacy_canonical_assets_are_left_untouched(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(
        builder,
        existing=True,
        inputs=inputs,
        legacy_canonical_assets=True,
    )
    legacy_paths = {
        f"{builder.ASSET_FOLDER}/{builder.SLOT_ASSET_NAMES[slot.slot]}" for slot in inputs.slots
    }

    result = builder.author_forest_hf(transport, inputs)

    assert result["meshes"] == {
        "forest.asset.birch": "created",
        "forest.asset.pine": "created",
    }
    assert _tool_names(transport).count("import_file") == 2
    assert "delete" not in _tool_names(transport)
    assert all(transport._exists(path) for path in legacy_paths)
    assert set(result["generated_hism_audit"]["by_mesh"]) == {
        slot.asset_ref for slot in inputs.slots if slot.slot in builder.REQUIRED_TREE_SLOTS
    }


def test_content_addressed_asset_metadata_collision_fails_closed(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    pine = next(slot for slot in inputs.slots if slot.slot == "forest.asset.pine")
    transport.metadata[pine.asset_path] = {}

    with pytest.raises(builder.McpToolError, match="content-addressed asset metadata mismatch"):
        builder.author_forest_hf(transport, inputs)

    assert "delete" not in _tool_names(transport)
    assert "import_file" not in _tool_names(transport)
    assert transport._exists(pine.asset_path)


def test_existing_graph_with_unmanaged_branch_fails_closed(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    transport.nodes["UnexpectedCollisionSpawner"] = f"{builder.GRAPH_OBJECT_PATH}:UnexpectedCollisionSpawner"

    with pytest.raises(builder.McpProtocolError, match="unmanaged nodes"):
        builder.author_forest_hf(transport, inputs)

    assert "ExecuteGraphInstance" not in _tool_names(transport)
    assert "save_assets" not in _tool_names(transport)


def test_legacy_weighted_graph_is_replaced_by_exact_slot_branches(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs, legacy_graph=True)

    result = builder.author_forest_hf(transport, inputs)

    names = _tool_names(transport)
    assert names.count("RemoveNode") == 2
    remove_calls = [
        arguments for _toolset, name, arguments in transport.calls if name == "RemoveNode"
    ]
    assert all(
        set(arguments) == {"graph", "node"}
        and arguments["graph"] == {"refPath": builder.GRAPH_OBJECT_PATH}
        for arguments in remove_calls
    )
    assert names.count("AddNode") == 4
    assert names.count("ConnectNodePins") == 4
    assert result["graph"]["outcome"] == "updated"
    assert set(transport.nodes) == {
        builder.OUTPUT_NODE_NAME,
        *builder.CREATE_POINTS_NODE_NAMES.values(),
        *builder.SPAWNER_NODE_NAMES.values(),
    }


def test_real_graph_structure_bare_edge_node_names_resolve_to_full_refs() -> None:
    builder = _builder()
    node_paths = {
        name: f"{builder.GRAPH_OBJECT_PATH}:{name}"
        for name in (
            builder.INPUT_NODE_NAME,
            builder.OUTPUT_NODE_NAME,
            *builder.CREATE_POINTS_NODE_NAMES.values(),
            *builder.SPAWNER_NODE_NAMES.values(),
        )
    }
    structure = {
        "nodes": [
            {"name": name, "path": path}
            for name, path in node_paths.items()
        ],
        "edges": [
            {
                "srcNode": builder.SPAWNER_NODE_NAMES[slot],
                "srcPin": "Out",
                "destNode": builder.OUTPUT_NODE_NAME,
                "destPin": "Out",
            }
            for slot in sorted(builder.REQUIRED_TREE_SLOTS)
        ]
        + [
            {
                "srcNode": builder.CREATE_POINTS_NODE_NAMES[slot],
                "srcPin": "Out",
                "destNode": builder.SPAWNER_NODE_NAMES[slot],
                "destPin": "In",
            }
            for slot in sorted(builder.REQUIRED_TREE_SLOTS)
        ],
    }
    branch_paths = {
        slot: (
            node_paths[builder.CREATE_POINTS_NODE_NAMES[slot]],
            node_paths[builder.SPAWNER_NODE_NAMES[slot]],
        )
        for slot in sorted(builder.REQUIRED_TREE_SLOTS)
    }

    assert builder._edge_node_path(structure, builder.SPAWNER_NODE_NAMES["forest.asset.birch"]) == node_paths[
        builder.SPAWNER_NODE_NAMES["forest.asset.birch"]
    ]
    assert builder._edge_node_path(
        structure,
        {"refPath": node_paths[builder.SPAWNER_NODE_NAMES["forest.asset.birch"]]},
    ) == node_paths[builder.SPAWNER_NODE_NAMES["forest.asset.birch"]]
    assert builder._edge_exists(
        structure,
        branch_paths["forest.asset.birch"][0],
        "Out",
        branch_paths["forest.asset.birch"][1],
        "In",
    )
    builder._validate_canonical_topology(
        structure,
        branch_paths,
        node_paths[builder.OUTPUT_NODE_NAME],
    )


def test_false_collision_removal_result_fails_closed(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=False, inputs=inputs, false_tool="remove_collisions")

    with pytest.raises(builder.McpToolError, match="remove_collisions did not report success"):
        builder.author_forest_hf(transport, inputs)

    assert "CreateGraph" not in _tool_names(transport)
    assert "save_assets" not in _tool_names(transport)


def test_generated_hism_count_drift_fails_before_save(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    transport.hism_instance_counts["Pine"] = 0

    with pytest.raises(builder.McpToolError, match="instance total 1"):
        builder.author_forest_hf(transport, inputs)

    assert "ExecuteGraphInstance" in _tool_names(transport)
    assert "save_assets" not in _tool_names(transport)


def test_generated_hism_species_drift_fails_even_when_total_matches(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    transport.hism_instance_counts = {"Birch": 0, "Pine": 2}

    with pytest.raises(builder.McpToolError, match="per-mesh instance counts"):
        builder.author_forest_hf(transport, inputs)

    assert "ExecuteGraphInstance" in _tool_names(transport)
    assert "save_assets" not in _tool_names(transport)


def test_forced_map_switch_detects_unpersisted_world_partition_actors(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    transport.persist_generated_on_save = False

    with pytest.raises(builder.McpToolError, match="no 256 m PCG World Partition actors"):
        builder.author_forest_hf(transport, inputs)

    loaded_levels = [arguments["level_path"] for _toolset, name, arguments in transport.calls if name == "load_level"]
    assert loaded_levels[-2:] == [builder.OPEN_WORLD_TEMPLATE, builder.MAP_PATH]


def test_remote_error_fails_closed_before_downstream_mutation(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=False, inputs=inputs, fail_tool="CreateGraph")

    with pytest.raises(builder.McpToolError, match="CreateGraph failed closed"):
        builder.author_forest_hf(transport, inputs)

    names = _tool_names(transport)
    assert names[-1] == "CreateGraph"
    assert "AddNode" not in names
    assert "SpawnGraphInstance" not in names
    assert "save_assets" not in names


def test_input_drift_fails_before_session_or_mutation(tmp_path: Path) -> None:
    builder = _builder()
    contract = _contract(builder)
    contract["seed"] = 1
    contract_path = _write_json(tmp_path / "drifted-contract.json", contract, builder)
    points_path = _write_json(tmp_path / "points.json", _points(builder), builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")

    with pytest.raises(builder.AuthoringInputError, match="seed"):
        builder.load_authoring_inputs(
            contract_path,
            points_path,
            [f"forest.asset.pine={pine}", f"forest.asset.birch={birch}"],
        )


def test_create_points_placement_is_bound_to_contract_instances(tmp_path: Path) -> None:
    builder = _builder()
    contract_path = _write_json(tmp_path / "contract.json", _contract(builder), builder)
    points = _points(builder)
    points["groups"][0]["json_params"]["pointsToCreate"][0]["transform"]["location"]["x"] = -99.0  # type: ignore[index]
    points_body = dict(points)
    del points_body["content_digest"]
    points["content_digest"] = hashlib.sha256(builder.canonical_json_bytes(points_body)).hexdigest()
    points_path = _write_json(tmp_path / "drifted-points.json", points, builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")

    with pytest.raises(builder.AuthoringInputError, match="location does not match"):
        builder.load_authoring_inputs(
            contract_path,
            points_path,
            [f"forest.asset.pine={pine}", f"forest.asset.birch={birch}"],
        )


def test_create_points_source_binding_is_bound_to_contract_instance(tmp_path: Path) -> None:
    builder = _builder()
    contract_path = _write_json(tmp_path / "contract.json", _contract(builder), builder)
    points = _points(builder)
    points["groups"][0]["source_bindings"][0]["source_stable_id"] = "forest.foliage.wrong"  # type: ignore[index]
    points_body = dict(points)
    del points_body["content_digest"]
    points["content_digest"] = hashlib.sha256(builder.canonical_json_bytes(points_body)).hexdigest()
    points_path = _write_json(tmp_path / "drifted-bindings.json", points, builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")

    with pytest.raises(builder.AuthoringInputError, match="source binding does not match"):
        builder.load_authoring_inputs(
            contract_path,
            points_path,
            [f"forest.asset.pine={pine}", f"forest.asset.birch={birch}"],
        )


def test_legacy_ungrouped_create_points_file_fails_closed(tmp_path: Path) -> None:
    builder = _builder()
    contract_path = _write_json(tmp_path / "contract.json", _contract(builder), builder)
    grouped = _points(builder)
    groups = cast(list[dict[str, Any]], grouped["groups"])
    legacy_points = {
        "pointsToCreate": [
            point
            for group in groups
            for point in group["json_params"]["pointsToCreate"]
        ],
        "coordinateSpace": "World",
        "bCullPointsOutsideVolume": True,
    }
    points_path = _write_json(tmp_path / "legacy-points.json", legacy_points, builder)
    pine = tmp_path / "pine.fbx"
    birch = tmp_path / "birch.fbx"
    pine.write_bytes(b"pine")
    birch.write_bytes(b"birch")

    with pytest.raises(builder.AuthoringInputError, match="wrapper contains unsupported or missing fields"):
        builder.load_authoring_inputs(
            contract_path,
            points_path,
            [f"forest.asset.pine={pine}", f"forest.asset.birch={birch}"],
        )


def test_multiple_existing_graph_instances_fail_closed(tmp_path: Path) -> None:
    builder = _builder()
    inputs = _inputs(tmp_path, builder)
    transport = _FakeTransport(builder, existing=True, inputs=inputs)
    original_call = transport.call_tool

    def duplicate_instances(toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        result = original_call(toolset_name, tool_name, arguments)
        if tool_name == "ListGraphInstances":
            return [*result, result[0]]
        return result

    transport.call_tool = duplicate_instances  # type: ignore[method-assign]
    with pytest.raises(builder.McpProtocolError, match="multiple PCG volumes"):
        builder.author_forest_hf(transport, inputs)
    assert "ExecuteGraphInstance" not in _tool_names(transport)
    assert "save_assets" not in _tool_names(transport)
