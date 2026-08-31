"""Deterministic fail-closed importer for heightmap-backed simulation worlds."""

from __future__ import annotations

import json
import math
import os
import re
import secrets
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .contracts import (
    ImportCode,
    ImportDraft,
    ImportFailure,
    assert_no_reparse_components,
    canonical_json_bytes,
    file_records,
    require_identity,
    require_mapping,
    require_string,
    resolve_beneath,
    safe_relative_path,
    strict_keys,
    validate_provenance,
    write_json,
)
from .heightmap import build_heightmap_artifacts, read_u16_height_grid
from .intake import SourceIntake
from .promotion import CatalogPromoter, _PackageLock, _same_tree

_REQUEST_SCHEMA = "lingtu.sim.world-import-request.v1"
_PROJECTION_SCHEMA = "lingtu.sim.world-visual-projection.v1"
_PACKAGE_SCHEMA = "lingtu.sim.world-package.v1"
_PROVENANCE_PATH = "provenance/world.provenance.json"
_PROJECTION_PATH = "visual/world.visual-projection.json"
_MANIFEST_PATH = "world.package.yaml"
_HFIELD_BASE_THICKNESS_M = 1.0
_ENTITY_COLLISION_HALF_EXTENT_M = 0.5
_WORLD_ENTITY_ASSETS = {
    "box": "/Engine/BasicShapes/Cube.Cube",
    "cylinder": "/Engine/BasicShapes/Cylinder.Cylinder",
}
_DEFAULT_WORLD_ENTITY_PBR = {
    "base_color_rgba": [0.45, 0.48, 0.52, 1.0],
    "metallic": 0.0,
    "roughness": 0.65,
}
_DEFAULT_PHYSICS_POLICY = {
    "timestep_s": 0.002,
    "integrator": "rk4",
    "solver": "newton",
    "iterations": 100,
    "gravity_mps2": [0.0, 0.0, -9.81],
}
_MJCF_INTEGRATORS = {
    "euler": "Euler",
    "rk4": "RK4",
    "implicit": "implicit",
    "implicitfast": "implicitfast",
}
_MJCF_SOLVERS = {"pgs": "PGS", "cg": "CG", "newton": "Newton"}


class _SchemaValidationError(ValueError):
    """Internal fallback error when the optional jsonschema package is absent."""


class WorldImporter:
    """Import one same-source world package into a qualified draft."""

    def __init__(self, repo_root: Path, *, intake: SourceIntake | None = None) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.intake = intake or SourceIntake()

    def import_world(self, request: Mapping[str, Any], *, draft_root: Path | None = None) -> ImportDraft:
        """Create a qualified draft or quarantine the rejected request."""

        try:
            normalized = self._normalize_request(request)
        except ImportFailure as exc:
            root = self._lexical_absolute(
                draft_root or (self.repo_root / "sim" / "imports" / "world" / "invalid-request")
            )
            self._quarantine(root, dict(request), exc)
            raise
        import_id = f"{normalized['package']['id']}-{normalized['package']['version']}"
        root = self._lexical_absolute(draft_root or (self.repo_root / "sim" / "imports" / "world" / import_id))
        lock_path = self.repo_root / "sim" / "imports" / "world" / ".locks" / f"{import_id}.lock"
        with _PackageLock(lock_path):
            assert_no_reparse_components(root.parent, context=str(root.parent))
            root.parent.mkdir(parents=True, exist_ok=True)
            assert_no_reparse_components(root.parent, context=str(root.parent))
            staging = Path(tempfile.mkdtemp(prefix=f".{root.name}.staging-", dir=str(root.parent))).resolve()
            try:
                draft = self._import_normalized(normalized, root=staging, import_id=import_id)
                if os.path.lexists(root):
                    try:
                        assert_no_reparse_components(root, below=root.parent, context=str(root))
                    except ImportFailure as exc:
                        raise ImportFailure(
                            "draft target is a symbolic link or reparse point; refusing to follow it",
                            code=ImportCode.PROMOTION_CONFLICT,
                            context=str(root),
                        ) from exc
                    if self._existing_draft_is_identical(root, draft):
                        shutil.rmtree(staging)
                        return self._relocate_draft(draft, root, staging)
                    raise ImportFailure(
                        "draft target already exists with different content; refusing to overwrite it",
                        code=ImportCode.PROMOTION_CONFLICT,
                        context=str(root),
                    )
                self._publish_staging(staging, root)
                return self._relocate_draft(draft, root, staging)
            except ImportFailure as exc:
                shutil.rmtree(staging, ignore_errors=True)
                self._quarantine(root, normalized, exc)
                raise
            except (OSError, ValueError, yaml.YAMLError) as exc:
                shutil.rmtree(staging, ignore_errors=True)
                failure = ImportFailure(
                    f"world import failed: {exc}",
                    code=ImportCode.QUALIFICATION_FAILED,
                    context=import_id,
                )
                self._quarantine(root, normalized, failure)
                raise failure from exc

    def _import_normalized(self, request: dict[str, Any], *, root: Path, import_id: str) -> ImportDraft:
        package = request["package"]
        package_id = package["id"]
        version = package["version"]
        source = request["source"]
        source_path = self._source_path(source["path"])
        intake = self.intake.materialize(source_path, root / "source")
        provenance = validate_provenance(source["provenance"], source_root=intake.root)

        package_root = root / "package"
        artifact_root = package_root / "artifacts"
        visual_root = package_root / "visual"
        provenance_root = package_root / "provenance"
        artifact_root.mkdir(parents=True)
        visual_root.mkdir()
        provenance_root.mkdir()

        heightmap = request["heightmap"]
        heightmap_source = resolve_beneath(intake.root, heightmap["path"], "heightmap.path")
        samples = read_u16_height_grid(
            heightmap_source,
            width=heightmap["width"],
            height=heightmap["height"],
            endian=heightmap["endian"],
        )
        spawn = request["spawn"]
        artifacts = build_heightmap_artifacts(
            samples=samples,
            width=heightmap["width"],
            height=heightmap["height"],
            extent_m=heightmap["extent_m"],
            elevation_min_m=heightmap["elevation_min_m"],
            elevation_max_m=heightmap["elevation_max_m"],
            spawn_xy_m=spawn["position_m"][:2],
            artifact_root=artifact_root,
            mesh_name=f"{package_id}_terrain",
        )
        requested_z = float(spawn["position_m"][2])
        if abs(requested_z - artifacts.spawn_height_m) > float(spawn["height_tolerance_m"]):
            raise ImportFailure(
                "spawn z is not aligned with the heightmap",
                code=ImportCode.WORLD_ALIGNMENT_INVALID,
                context="spawn.position_m",
                details={"requested_z_m": requested_z, "heightmap_z_m": artifacts.spawn_height_m},
            )

        mesh_assets = self._copy_optional_meshes(request, intake.root, artifact_root)
        entities = request["entities"]
        alignment_path = write_json(artifact_root / "alignment-report.json", artifacts.alignment)
        provenance_path = write_json(
            package_root / _PROVENANCE_PATH,
            {
                "schema": "lingtu.sim.world-provenance.v1",
                "package": {
                    "id": package_id,
                    "version": version,
                    "manifest": _MANIFEST_PATH,
                    "projection": _PROJECTION_PATH,
                },
                "source_intake": intake.to_dict(),
                "provenance": provenance,
            },
        )
        mjcf_path = self._write_mjcf(
            package_root / "world.xml",
            package_id=package_id,
            extent_m=artifacts.extent_m,
            elevation_origin_m=artifacts.sampled_elevation_min_m,
            elevation_scale_m=artifacts.mujoco_elevation_scale_m,
            entities=entities,
            mesh_assets=mesh_assets,
            physics=request["physics"],
        )
        compile_report = self._compile_mjcf(mjcf_path)
        projection_path = write_json(
            package_root / _PROJECTION_PATH,
            self._world_projection(
                request,
                artifacts,
                mesh_assets,
                package_id=package_id,
                version=version,
            ),
        )
        content_records = self._content_records(package_root, excluded={_MANIFEST_PATH})
        manifest_path = self._write_manifest(
            package_root / "world.package.yaml",
            package_id=package_id,
            version=version,
            description=package.get("description"),
            visual=request["visual"],
            entities=entities,
            provenance_path=provenance_path.relative_to(package_root).as_posix(),
            projection_path=projection_path.relative_to(package_root).as_posix(),
            content_records=content_records,
            physics=request["physics"],
        )
        manifest = self._load_yaml_mapping(manifest_path, "world manifest")
        projection = self._load_json_mapping(projection_path, "world visual projection")
        self._schema_gate(manifest, "world.v1.json", "manifest", ImportCode.MODEL_INVALID)
        self._schema_gate(
            projection, "world-visual.v1.json", "projection", ImportCode.PROJECTION_INVALID
        )
        projection_gate = self._validate_package_content(
            package_root,
            manifest,
            projection,
            provenance_path,
            projection_path,
            content_records,
        )
        entity_gate = self._validate_entity_graph(package_root / "world.xml", entities)
        qualification_path = self._write_qualification(
            root,
            package_id=package_id,
            version=version,
            provenance_path=provenance_path,
            evidence_sources=(alignment_path, projection_path, provenance_path, mjcf_path),
            compile_report=compile_report,
            projection_gate=projection_gate,
            entity_gate=entity_gate,
        )
        draft = ImportDraft(
            import_id=import_id,
            kind="world",
            package_id=package_id,
            version=version,
            state="qualified",
            root=root,
            package_root=package_root,
            manifest_path=manifest_path,
            provenance_path=provenance_path,
            qualification_path=qualification_path,
        )
        write_json(root / "draft.json", draft.to_dict())
        return draft

    def _normalize_request(self, value: Mapping[str, Any]) -> dict[str, Any]:
        request = require_mapping(value, "request")
        strict_keys(
            request,
            required={"schema", "package", "source", "units", "heightmap", "visual", "spawn"},
            optional={"mesh", "entities", "bounds", "physics"},
            context="request",
        )
        if request["schema"] != _REQUEST_SCHEMA:
            raise ImportFailure("request.schema is unsupported", context="request.schema")
        package = require_mapping(request["package"], "package")
        strict_keys(package, required={"id", "version"}, optional={"description"}, context="package")
        source = require_mapping(request["source"], "source")
        strict_keys(source, required={"path", "provenance"}, optional=set(), context="source")
        units = require_mapping(request["units"], "units")
        strict_keys(units, required={"length", "up_axis", "handedness"}, optional=set(), context="units")
        if units != {"length": "m", "up_axis": "Z", "handedness": "RH"}:
            raise ImportFailure(
                "world import units must be explicit metres/RH Z-up",
                code=ImportCode.UNIT_AMBIGUOUS,
                context="units",
            )
        heightmap = require_mapping(request["heightmap"], "heightmap")
        strict_keys(
            heightmap,
            required={"path", "width", "height", "extent_m", "elevation_min_m", "elevation_max_m"},
            optional={"endian"},
            context="heightmap",
        )
        visual = require_mapping(request["visual"], "visual")
        strict_keys(visual, required={"binding", "level"}, optional=set(), context="visual")
        spawn = require_mapping(request["spawn"], "spawn")
        strict_keys(
            spawn,
            required={"position_m", "quaternion_wxyz"},
            optional={"height_tolerance_m"},
            context="spawn",
        )
        package_normalized: dict[str, Any] = {
            "id": require_identity(package["id"], "package.id"),
            "version": require_identity(package["version"], "package.version", version=True),
        }
        normalized = {
            "schema": _REQUEST_SCHEMA,
            "package": package_normalized,
            "source": {
                "path": require_string(source["path"], "source.path"),
                "provenance": require_mapping(source["provenance"], "source.provenance"),
            },
            "units": units,
            "heightmap": {
                "path": safe_relative_path(heightmap["path"], "heightmap.path"),
                "width": self._positive_int(heightmap["width"], "heightmap.width"),
                "height": self._positive_int(heightmap["height"], "heightmap.height"),
                "extent_m": self._float_vector(heightmap["extent_m"], 2, "heightmap.extent_m"),
                "elevation_min_m": self._finite_float(heightmap["elevation_min_m"], "heightmap.elevation_min_m"),
                "elevation_max_m": self._finite_float(heightmap["elevation_max_m"], "heightmap.elevation_max_m"),
                "endian": require_string(heightmap.get("endian", "little"), "heightmap.endian"),
            },
            "visual": {
                "binding": require_string(visual["binding"], "visual.binding"),
                "level": require_string(visual["level"], "visual.level"),
            },
            "spawn": {
                "position_m": self._float_vector(spawn["position_m"], 3, "spawn.position_m"),
                "quaternion_wxyz": self._float_vector(spawn["quaternion_wxyz"], 4, "spawn.quaternion_wxyz"),
                "height_tolerance_m": self._finite_float(
                    spawn.get("height_tolerance_m", 1e-6), "spawn.height_tolerance_m"
                ),
            },
            "physics": self._normalize_physics(request.get("physics")),
            "mesh": self._normalize_mesh(request.get("mesh")),
            "entities": self._normalized_entities(request.get("entities", [])),
        }
        if "description" in package:
            package_normalized["description"] = require_string(package["description"], "package.description")
        self._validate_bounds_request(normalized, request.get("bounds"))
        self._schema_gate(request, "world-import.v1.json", "request", ImportCode.INVALID_REQUEST)
        return normalized

    def _normalize_physics(self, value: Any) -> dict[str, Any]:
        if value is None:
            return dict(_DEFAULT_PHYSICS_POLICY)
        physics = require_mapping(value, "physics")
        strict_keys(
            physics,
            required=set(),
            optional={"timestep_s", "integrator", "solver", "iterations", "gravity"},
            context="physics",
        )
        policy = dict(_DEFAULT_PHYSICS_POLICY)
        if "timestep_s" in physics:
            timestep_s = self._finite_float(physics["timestep_s"], "physics.timestep_s")
            if timestep_s <= 0.0:
                raise ImportFailure("physics.timestep_s must be positive", context="physics.timestep_s")
            policy["timestep_s"] = timestep_s
        if "integrator" in physics:
            integrator = require_string(physics["integrator"], "physics.integrator")
            if integrator not in _MJCF_INTEGRATORS:
                raise ImportFailure("physics.integrator is unsupported", context="physics.integrator")
            policy["integrator"] = integrator
        if "solver" in physics:
            solver = require_string(physics["solver"], "physics.solver")
            if solver not in _MJCF_SOLVERS:
                raise ImportFailure("physics.solver is unsupported", context="physics.solver")
            policy["solver"] = solver
        if "iterations" in physics:
            policy["iterations"] = self._positive_int(physics["iterations"], "physics.iterations")
        if "gravity" in physics:
            policy["gravity_mps2"] = self._float_vector(physics["gravity"], 3, "physics.gravity")
        return policy

    def _normalize_mesh(self, value: Any) -> dict[str, Any] | None:
        if value is None:
            return None
        mesh = require_mapping(value, "mesh")
        strict_keys(mesh, required={"path", "collision"}, optional=set(), context="mesh")
        if mesh["collision"] is not True:
            raise ImportFailure(
                "optional mesh must be declared collision-capable or omitted",
                code=ImportCode.MODEL_INVALID,
                context="mesh.collision",
            )
        return {"path": safe_relative_path(mesh["path"], "mesh.path"), "collision": True}

    def _normalized_entities(self, value: Any) -> list[dict[str, Any]]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ImportFailure("entities must be an array", context="entities")
        entities: list[dict[str, Any]] = []
        seen: set[str] = set()
        for index, item in enumerate(value):
            context = f"entities[{index}]"
            entity = require_mapping(item, context)
            strict_keys(
                entity,
                required={
                    "entity_id",
                    "entity_type",
                    "authority",
                    "initial_transform",
                    "physics_proxy",
                    "semantic_class",
                },
                optional={"collision", "geometry", "visual"},
                context=context,
            )
            entity_id = require_identity(entity["entity_id"], f"{context}.entity_id")
            if entity_id in seen:
                raise ImportFailure(
                    "entity graph contains duplicate entity_id", code=ImportCode.MODEL_INVALID, context=context
                )
            seen.add(entity_id)
            physics_proxy = require_string(entity["physics_proxy"], f"{context}.physics_proxy")
            if physics_proxy not in {"mujoco", "kinematic", "none"}:
                raise ImportFailure(
                    "entity.physics_proxy is unsupported",
                    code=ImportCode.MODEL_INVALID,
                    context=f"{context}.physics_proxy",
                )
            collision = entity.get("collision", False)
            if not isinstance(collision, bool):
                raise ImportFailure("entity.collision must be boolean", code=ImportCode.MODEL_INVALID, context=context)
            if physics_proxy == "mujoco" and collision is not True:
                raise ImportFailure(
                    "colliding entity is missing an explicit collision declaration",
                    code=ImportCode.MODEL_INVALID,
                    context=context,
                )
            if collision is True and physics_proxy != "mujoco":
                raise ImportFailure(
                    "entity collision claims require a materialized mujoco physics proxy",
                    code=ImportCode.MODEL_INVALID,
                    context=context,
                )
            transform = require_mapping(entity["initial_transform"], f"{context}.initial_transform")
            strict_keys(
                transform,
                required={"position_m", "quaternion_wxyz"},
                optional=set(),
                context=f"{context}.initial_transform",
            )
            quaternion = self._float_vector(
                transform["quaternion_wxyz"],
                4,
                f"{context}.initial_transform.quaternion_wxyz",
            )
            norm = math.sqrt(sum(component * component for component in quaternion))
            if abs(norm - 1.0) > 1e-6:
                raise ImportFailure(
                    "entity initial quaternion must be normalized",
                    code=ImportCode.WORLD_ALIGNMENT_INVALID,
                    context=f"{context}.initial_transform.quaternion_wxyz",
                )
            semantic_class = require_string(entity["semantic_class"], f"{context}.semantic_class")
            normalized_entity = {
                    "entity_id": entity_id,
                    "entity_type": require_string(entity["entity_type"], f"{context}.entity_type"),
                    "authority": require_string(entity["authority"], f"{context}.authority"),
                    "initial_transform": {
                        "position_m": self._float_vector(
                            transform["position_m"], 3, f"{context}.initial_transform.position_m"
                        ),
                        "quaternion_wxyz": quaternion,
                    },
                    "physics_proxy": physics_proxy,
                    "semantic_class": semantic_class,
                    "collision": collision,
                }
            if "geometry" in entity:
                normalized_entity["geometry"] = self._normalize_entity_geometry(
                    entity["geometry"],
                    f"{context}.geometry",
                )
            normalized_entity["visual"] = self._normalize_entity_visual(
                entity.get("visual"),
                semantic_class=semantic_class,
                context=f"{context}.visual",
            )
            entities.append(normalized_entity)
        return entities

    def _normalize_entity_visual(
        self,
        value: Any,
        *,
        semantic_class: str,
        context: str,
    ) -> dict[str, Any]:
        if value is None:
            return {
                "mode": "runtime",
                "material": {
                    "key": semantic_class,
                    **_DEFAULT_WORLD_ENTITY_PBR,
                },
            }
        visual = require_mapping(value, context)
        strict_keys(
            visual,
            required={"mode"},
            optional={"material"},
            context=context,
        )
        mode = require_string(visual["mode"], f"{context}.mode")
        if mode not in {"level", "runtime"}:
            raise ImportFailure(
                "entity visual mode must be 'level' or 'runtime'",
                code=ImportCode.MODEL_INVALID,
                context=f"{context}.mode",
            )
        if mode == "level":
            if "material" in visual:
                raise ImportFailure(
                    "level-baked entities must not declare runtime material data",
                    code=ImportCode.MODEL_INVALID,
                    context=context,
                )
            return {"mode": "level"}

        material_value = visual.get("material")
        if material_value is None:
            return {
                "mode": "runtime",
                "material": {
                    "key": semantic_class,
                    **_DEFAULT_WORLD_ENTITY_PBR,
                },
            }
        material = require_mapping(material_value, f"{context}.material")
        strict_keys(
            material,
            required={"key", "base_color_rgba", "metallic", "roughness"},
            optional=set(),
            context=f"{context}.material",
        )
        base_color = self._float_vector(
            material["base_color_rgba"],
            4,
            f"{context}.material.base_color_rgba",
        )
        metallic = self._finite_float(material["metallic"], f"{context}.material.metallic")
        roughness = self._finite_float(material["roughness"], f"{context}.material.roughness")
        if any(component < 0.0 or component > 1.0 for component in base_color):
            raise ImportFailure(
                "entity material base_color_rgba must stay in [0, 1]",
                code=ImportCode.MODEL_INVALID,
                context=f"{context}.material.base_color_rgba",
            )
        if not 0.0 <= metallic <= 1.0 or not 0.0 <= roughness <= 1.0:
            raise ImportFailure(
                "entity material metallic and roughness must stay in [0, 1]",
                code=ImportCode.MODEL_INVALID,
                context=f"{context}.material",
            )
        return {
            "mode": "runtime",
            "material": {
                "key": require_string(material["key"], f"{context}.material.key"),
                "base_color_rgba": base_color,
                "metallic": metallic,
                "roughness": roughness,
            },
        }

    def _normalize_entity_geometry(self, value: Any, context: str) -> dict[str, Any]:
        geometry = require_mapping(value, context)
        shape = require_string(geometry.get("shape"), f"{context}.shape")
        if shape == "box":
            strict_keys(
                geometry,
                required={"shape", "size_m"},
                optional=set(),
                context=context,
            )
            size = self._float_vector(geometry["size_m"], 3, f"{context}.size_m")
            if any(component <= 0.0 for component in size):
                raise ImportFailure(
                    "box geometry size_m must contain positive values",
                    code=ImportCode.MODEL_INVALID,
                    context=f"{context}.size_m",
                )
            return {"shape": "box", "size_m": size}
        if shape == "cylinder":
            strict_keys(
                geometry,
                required={"shape", "radius_m", "half_height_m"},
                optional=set(),
                context=context,
            )
            radius = self._finite_float(geometry["radius_m"], f"{context}.radius_m")
            half_height = self._finite_float(
                geometry["half_height_m"],
                f"{context}.half_height_m",
            )
            if radius <= 0.0 or half_height <= 0.0:
                raise ImportFailure(
                    "cylinder geometry dimensions must be positive",
                    code=ImportCode.MODEL_INVALID,
                    context=context,
                )
            return {
                "shape": "cylinder",
                "radius_m": radius,
                "half_height_m": half_height,
            }
        raise ImportFailure(
            f"unsupported entity geometry shape {shape!r}",
            code=ImportCode.MODEL_INVALID,
            context=f"{context}.shape",
        )

    def _validate_bounds_request(self, normalized: dict[str, Any], bounds: Any) -> None:
        if bounds is None:
            return
        requested = require_mapping(bounds, "bounds")
        strict_keys(requested, required={"min_m", "max_m"}, optional=set(), context="bounds")
        expected = {
            "min_m": [
                -normalized["heightmap"]["extent_m"][0] / 2.0,
                -normalized["heightmap"]["extent_m"][1] / 2.0,
                normalized["heightmap"]["elevation_min_m"],
            ],
            "max_m": [
                normalized["heightmap"]["extent_m"][0] / 2.0,
                normalized["heightmap"]["extent_m"][1] / 2.0,
                normalized["heightmap"]["elevation_max_m"],
            ],
        }
        actual = {
            "min_m": self._float_vector(requested["min_m"], 3, "bounds.min_m"),
            "max_m": self._float_vector(requested["max_m"], 3, "bounds.max_m"),
        }
        if actual != expected:
            raise ImportFailure(
                "declared bounds do not match heightmap-derived bounds",
                code=ImportCode.PROJECTION_INVALID,
                context="bounds",
                details={"expected": expected, "actual": actual},
            )

    def _copy_optional_meshes(
        self, request: Mapping[str, Any], source_root: Path, artifact_root: Path
    ) -> tuple[dict[str, Any], ...]:
        mesh = request.get("mesh")
        if mesh is None:
            return ()
        source = resolve_beneath(source_root, mesh["path"], "mesh.path")
        if not source.is_file():
            raise ImportFailure(
                "mesh.path does not identify a file", code=ImportCode.ASSET_MISSING, context="mesh.path"
            )
        target = artifact_root / "source_mesh.obj"
        shutil.copyfile(source, target)
        return (
            {
                "role": "source_mesh",
                "path": target.relative_to(artifact_root.parent).as_posix(),
                "bytes": target.stat().st_size,
                "collision": True,
            },
        )

    def _world_projection(
        self,
        request: Mapping[str, Any],
        artifacts: Any,
        mesh_assets: Sequence[Mapping[str, Any]],
        *,
        package_id: str,
        version: str,
    ) -> dict[str, Any]:
        body = {
            "schema": _PROJECTION_SCHEMA,
            "package": {
                "id": package_id,
                "version": version,
                "manifest": _MANIFEST_PATH,
                "provenance": _PROVENANCE_PATH,
            },
            "binding": request["visual"]["binding"],
            "level": request["visual"]["level"],
            "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
            "terrain": {
                "grid_px": [artifacts.width, artifacts.height],
                "extent_m": list(artifacts.extent_m),
                "sample_spacing_m": list(artifacts.sample_spacing_m),
                "physics_bounds_m": artifacts.physics_bounds_m,
                "visual_bounds_m": artifacts.visual_bounds_m,
                "assets": [*artifacts.assets, *mesh_assets],
            },
            "entities": self._world_entity_projections(request["entities"]),
            "spawn_alignment": artifacts.alignment["spawn"],
        }
        return body

    @staticmethod
    def _world_entity_projections(
        entities: Sequence[Mapping[str, Any]],
    ) -> list[dict[str, Any]]:
        projections: list[dict[str, Any]] = []
        for entity in entities:
            visual = entity["visual"]
            if visual["mode"] != "runtime":
                continue
            geometry = entity.get("geometry") or {
                "shape": "box",
                "size_m": [
                    2.0 * _ENTITY_COLLISION_HALF_EXTENT_M,
                    2.0 * _ENTITY_COLLISION_HALF_EXTENT_M,
                    2.0 * _ENTITY_COLLISION_HALF_EXTENT_M,
                ],
            }
            if geometry["shape"] == "box":
                dimensions = [float(value) for value in geometry["size_m"]]
            else:
                diameter = 2.0 * float(geometry["radius_m"])
                dimensions = [diameter, diameter, 2.0 * float(geometry["half_height_m"])]
            material = visual["material"]
            projections.append(
                {
                    "entity_id": entity["entity_id"],
                    "semantic_class": entity["semantic_class"],
                    "authority": entity["authority"],
                    "transform": entity["initial_transform"],
                    "geometry": geometry,
                    "unreal": {
                        "representation": "static_mesh",
                        "component_class": "/Script/Engine.StaticMeshComponent",
                        "asset_path": _WORLD_ENTITY_ASSETS[geometry["shape"]],
                        "dimensions_m": dimensions,
                    },
                    "material": {
                        "source": "world_package",
                        "key": material["key"],
                        "pbr": {
                            "base_color_rgba": material["base_color_rgba"],
                            "metallic": material["metallic"],
                            "roughness": material["roughness"],
                        },
                    },
                }
            )
        return sorted(projections, key=lambda item: item["entity_id"])

    def _write_mjcf(
        self,
        path: Path,
        *,
        package_id: str,
        extent_m: tuple[float, float],
        elevation_origin_m: float,
        elevation_scale_m: float,
        entities: Sequence[Mapping[str, Any]],
        mesh_assets: Sequence[Mapping[str, Any]],
        physics: Mapping[str, Any],
    ) -> Path:
        half_x = extent_m[0] / 2.0
        half_y = extent_m[1] / 2.0
        base = _HFIELD_BASE_THICKNESS_M
        entity_lines: list[str] = []
        for entity in entities:
            if entity["physics_proxy"] != "mujoco":
                continue
            transform = entity["initial_transform"]
            entity_id = entity["entity_id"]
            position = " ".join(f"{value:.9g}" for value in transform["position_m"])
            quaternion = " ".join(f"{value:.9g}" for value in transform["quaternion_wxyz"])
            geometry = entity.get("geometry")
            if geometry is None:
                collision_type = "box"
                collision_size = " ".join(f"{_ENTITY_COLLISION_HALF_EXTENT_M:g}" for _ in range(3))
            elif geometry["shape"] == "box":
                collision_type = "box"
                collision_size = " ".join(f"{float(value) / 2.0:.9g}" for value in geometry["size_m"])
            else:
                collision_type = "cylinder"
                collision_size = (
                    f"{float(geometry['radius_m']):.9g} "
                    f"{float(geometry['half_height_m']):.9g}"
                )
            collision_line = (
                f'      <geom name="entity_{entity_id}_collision" '
                f'type="{collision_type}" size="{collision_size}" contype="1" conaffinity="1"/>'
            )
            entity_lines.extend(
                [
                    f'    <body name="entity_{entity_id}" pos="{position}" quat="{quaternion}">',
                    collision_line,
                    "    </body>",
                ]
            )
        mesh_lines: list[str] = []
        mesh_collision_lines: list[str] = []
        if mesh_assets:
            mesh_path = mesh_assets[0]["path"]
            mesh_name = f"{package_id}_source_mesh"
            mesh_lines.append(f'    <mesh name="{mesh_name}" file="{mesh_path}"/>')
            mesh_collision_lines.append(
                f'    <geom name="source_mesh_collision" type="mesh" mesh="{mesh_name}" contype="1" conaffinity="1"/>'
            )
        hfield_line = (
            f'    <hfield name="{package_id}_heightfield" '
            f'file="artifacts/heightfield_f32.bin" '
            f'content_type="image/vnd.mujoco.hfield" '
            f'size="{half_x:.9g} {half_y:.9g} {elevation_scale_m:.9g} {base:.9g}"/>'
        )
        terrain_line = (
            f'    <geom name="terrain_collision" type="hfield" '
            f'hfield="{package_id}_heightfield" pos="0 0 {elevation_origin_m:.9g}" '
            f'rgba="0.35 0.42 0.32 1" contype="1" conaffinity="1"/>'
        )
        path.write_text(
            "\n".join(
                [
                    f"<!-- Generated by WorldImporter for {package_id}; units are metres, RH Z-up. -->",
                    '<mujoco model="imported_world">',
                    '  <compiler angle="radian"/>',
                    (
                        '  <option '
                        f'gravity="{" ".join(f"{value:.9g}" for value in physics["gravity_mps2"])}" '
                        f'timestep="{float(physics["timestep_s"]):.17g}" '
                        f'integrator="{_MJCF_INTEGRATORS[physics["integrator"]]}" '
                        f'solver="{_MJCF_SOLVERS[physics["solver"]]}" '
                        f'iterations="{physics["iterations"]}"/>'
                    ),
                    "  <asset>",
                    hfield_line,
                    *mesh_lines,
                    "  </asset>",
                    "  <worldbody>",
                    terrain_line,
                    *mesh_collision_lines,
                    *entity_lines,
                    "  </worldbody>",
                    "</mujoco>",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        return path

    def _write_manifest(
        self,
        path: Path,
        *,
        package_id: str,
        version: str,
        description: str | None,
        visual: Mapping[str, Any],
        entities: Sequence[Mapping[str, Any]],
        provenance_path: str,
        projection_path: str,
        content_records: Sequence[Any],
        physics: Mapping[str, Any],
    ) -> Path:
        manifest: dict[str, Any] = {
            "schema": _PACKAGE_SCHEMA,
            "id": package_id,
            "version": version,
            "kind": "world",
            "physics": {
                "mjcf": "world.xml",
                "global_policy": {
                    "timestep_s": physics["timestep_s"],
                    "integrator": physics["integrator"],
                    "solver": physics["solver"],
                    "iterations": physics["iterations"],
                    "gravity_mps2": physics["gravity_mps2"],
                },
            },
            "visual": {
                "binding": visual["binding"],
                "level": visual["level"],
                "projection": projection_path,
            },
            "entities": list(entities),
            "provenance": {"path": provenance_path},
            "content": {
                "files": [{"path": item.path, "size": item.size} for item in content_records],
                "provenance": {"path": provenance_path},
                "visual_projection": {"path": projection_path},
            },
        }
        if description is not None:
            manifest["description"] = description
        path.write_bytes(yaml.safe_dump(manifest, sort_keys=False).encode("utf-8"))
        return path

    def _write_qualification(
        self,
        root: Path,
        *,
        package_id: str,
        version: str,
        provenance_path: Path,
        evidence_sources: Sequence[Path],
        compile_report: Mapping[str, Any],
        projection_gate: Mapping[str, Any],
        entity_gate: Mapping[str, Any],
    ) -> Path:
        qualification_dir = root / "qualification" / "world" / package_id
        evidence_dir = qualification_dir / "evidence" / version
        evidence_dir.mkdir(parents=True)
        evidence: list[dict[str, str]] = []
        for source in evidence_sources:
            target = evidence_dir / source.name
            shutil.copyfile(source, target)
            evidence.append({"path": f"evidence/{version}/{target.name}"})
        gate_documents = {
            "entity-graph-gate.json": entity_gate,
            "mujoco-compile-gate.json": compile_report,
            "projection-gate.json": projection_gate,
            "schema-gate.json": {
                "request": _REQUEST_SCHEMA,
                "package": _PACKAGE_SCHEMA,
                "projection": _PROJECTION_SCHEMA,
            },
        }
        for name, document in sorted(gate_documents.items()):
            target = write_json(evidence_dir / name, document)
            evidence.append({"path": f"evidence/{version}/{target.name}"})
        evidence_by_name = {Path(item["path"]).name: item for item in evidence}
        report = {
            "schema": "lingtu.sim.qualification-record.v1",
            "package": {
                "kind": "world",
                "id": package_id,
                "version": version,
            },
            "qualified_capabilities": {},
            "provenance": {
                "path": provenance_path.relative_to(root / "package").as_posix(),
            },
            "checks": [
                {
                    "id": "schema",
                    "status": "passed",
                    "evidence": [evidence_by_name["alignment-report.json"], evidence_by_name["schema-gate.json"]],
                },
                {
                    "id": "mujoco_compile",
                    "status": "passed",
                    "evidence": [evidence_by_name["world.xml"], evidence_by_name["mujoco-compile-gate.json"]],
                },
                {
                    "id": "visual_projection",
                    "status": "passed",
                    "evidence": [
                        evidence_by_name["world.visual-projection.json"],
                        evidence_by_name["projection-gate.json"],
                    ],
                },
                {"id": "entity_graph", "status": "passed", "evidence": [evidence_by_name["entity-graph-gate.json"]]},
                {
                    "id": "provenance",
                    "status": "passed",
                    "evidence": [evidence_by_name["world.provenance.json"]],
                },
            ],
        }
        return write_json(qualification_dir / f"{version}.qualification.json", report)

    def _quarantine(self, root: Path, request: Mapping[str, Any], failure: ImportFailure) -> None:
        root = self._lexical_absolute(root)
        token = secrets.token_hex(8)
        owned_root = self.repo_root / "sim" / "imports" / "world"
        staging_parent = owned_root / ".quarantine-staging"
        publication_parent = owned_root / "quarantine"
        for directory in (staging_parent, publication_parent):
            assert_no_reparse_components(directory, context=str(directory))
            directory.mkdir(parents=True, exist_ok=True)
            assert_no_reparse_components(directory, context=str(directory))
        staging = Path(tempfile.mkdtemp(prefix=f".{token}.staging-", dir=str(staging_parent)))
        quarantine = staging / "quarantine"
        quarantine.mkdir()
        try:
            self._write_exclusive_json(quarantine, "request.json", request, token)
            self._write_exclusive_json(quarantine, "failure.json", failure.to_diagnostic().to_dict(), token)
            for _ in range(8):
                target = publication_parent / f"{root.name}-{token}-{secrets.token_hex(8)}"
                try:
                    self._publish_staging(staging, target)
                    return
                except ImportFailure as exc:
                    if exc.code != ImportCode.PROMOTION_CONFLICT:
                        raise
            raise ImportFailure(
                "cannot publish quarantine after repeated target conflicts",
                code=ImportCode.PROMOTION_CONFLICT,
                context=str(publication_parent),
            )
        finally:
            shutil.rmtree(staging, ignore_errors=True)

    @staticmethod
    def _lexical_absolute(path: Path | str) -> Path:
        """Return an absolute path without following its final directory entry."""

        return Path(os.path.abspath(os.fspath(path)))

    def _source_path(self, value: str) -> Path:
        path = Path(value)
        if not path.is_absolute():
            path = self.repo_root / path
        return path.resolve()

    def _publish_staging(self, staging: Path, target: Path) -> None:
        """Atomically publish one owned staging directory without replacing a target."""

        try:
            CatalogPromoter(self.repo_root)._rename_noreplace(staging, target)
        except FileExistsError as exc:
            raise ImportFailure(
                "draft target appeared during import; refusing to overwrite it",
                code=ImportCode.PROMOTION_CONFLICT,
                context=str(target),
            ) from exc

    @staticmethod
    def _existing_draft_is_identical(target: Path, incoming: ImportDraft) -> bool:
        """Accept a complete prior publication only when its bound bytes match."""

        draft_path = target / "draft.json"
        package_root = target / "package"
        if not draft_path.is_file() or not package_root.is_dir() or incoming.package_root is None:
            return False
        try:
            document = json.loads(draft_path.read_text(encoding="utf-8"))
        except (ImportFailure, OSError, json.JSONDecodeError):
            return False
        return (
            isinstance(document, Mapping)
            and document.get("import_id") == incoming.import_id
            and _same_tree(package_root, incoming.package_root)
        )

    def _relocate_draft(self, draft: ImportDraft, target: Path, staging: Path) -> ImportDraft:
        def relocate(path: Path | None) -> Path | None:
            if path is None:
                return None
            return target / path.relative_to(staging)

        relocated = ImportDraft(
            import_id=draft.import_id,
            kind=draft.kind,
            package_id=draft.package_id,
            version=draft.version,
            state=draft.state,
            root=target,
            package_root=relocate(draft.package_root),
            manifest_path=relocate(draft.manifest_path),
            provenance_path=relocate(draft.provenance_path),
            qualification_path=relocate(draft.qualification_path),
            diagnostics=draft.diagnostics,
        )
        write_json(target / "draft.json", relocated.to_dict())
        return relocated

    def _content_records(self, package_root: Path, *, excluded: set[str]) -> tuple[Any, ...]:
        return tuple(item for item in file_records(package_root) if item.path not in excluded)

    def _schema_root(self) -> Path:
        local = self.repo_root / "schemas" / "simulation"
        if local.is_dir():
            return local
        return Path(__file__).resolve().parents[2] / "schemas" / "simulation"

    def _schema_gate(self, document: Any, filename: str, context: str, code: ImportCode) -> None:
        schema_path = self._schema_root() / filename
        try:
            schema = json.loads(schema_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ImportFailure(
                f"cannot load {context} schema: {exc}",
                code=ImportCode.QUALIFICATION_FAILED if context != "request" else code,
                context=str(schema_path),
            ) from exc
        try:
            import jsonschema  # type: ignore[import-not-found]
        except ImportError:
            try:
                _validate_json_schema(document, schema, schema, f"{context}")
            except _SchemaValidationError as exc:
                raise ImportFailure(str(exc), code=code, context=context) from exc
            return
        try:
            jsonschema.Draft202012Validator(schema).validate(document)
        except jsonschema.exceptions.ValidationError as exc:
            path = ".".join(str(item) for item in exc.absolute_path)
            detail = f"{context}.{path}" if path else context
            raise ImportFailure(
                f"{detail} failed schema validation: {exc.message}",
                code=code,
                context=detail,
            ) from exc

    def _load_yaml_mapping(self, path: Path, context: str) -> dict[str, Any]:
        try:
            value = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ImportFailure(
                f"cannot read {context}: {exc}", code=ImportCode.MODEL_INVALID, context=str(path)
            ) from exc
        return require_mapping(value, context)

    def _load_json_mapping(self, path: Path, context: str) -> dict[str, Any]:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ImportFailure(
                f"cannot read {context}: {exc}", code=ImportCode.PROJECTION_INVALID, context=str(path)
            ) from exc
        return require_mapping(value, context)

    def _validate_package_content(
        self,
        package_root: Path,
        manifest: Mapping[str, Any],
        projection: Mapping[str, Any],
        provenance_path: Path,
        projection_path: Path,
        content_records: Sequence[Any],
    ) -> dict[str, Any]:
        visual = require_mapping(manifest["visual"], "manifest.visual")
        if visual["projection"] != _PROJECTION_PATH:
            raise ImportFailure(
                "world manifest visual.projection must reference the package-local projection",
                code=ImportCode.PROJECTION_INVALID,
                context="manifest.visual.projection",
            )
        declared_provenance = require_mapping(manifest["provenance"], "manifest.provenance")
        if declared_provenance["path"] != _PROVENANCE_PATH:
            raise ImportFailure(
                "world manifest provenance.path must reference the package-local provenance",
                code=ImportCode.PROJECTION_INVALID,
                context="manifest.provenance.path",
            )
        content = require_mapping(manifest["content"], "manifest.content")
        declared_files = content["files"]
        actual_files = [{"path": item.path, "size": item.size} for item in content_records]
        if declared_files != actual_files:
            raise ImportFailure(
                "world manifest content identity does not match package bytes",
                code=ImportCode.PROJECTION_INVALID,
                context="manifest.content",
            )
        for key, expected_path in (("provenance", _PROVENANCE_PATH), ("visual_projection", _PROJECTION_PATH)):
            reference = require_mapping(content[key], f"manifest.content.{key}")
            if reference["path"] != expected_path:
                raise ImportFailure(
                    f"manifest.content.{key} does not match package bytes",
                    code=ImportCode.PROJECTION_INVALID,
                    context=f"manifest.content.{key}",
                )
        try:
            provenance_path.resolve().relative_to(package_root.resolve())
        except ValueError as exc:
            raise ImportFailure(
                "world package provenance escaped its package root",
                code=ImportCode.UNSAFE_SOURCE,
                context=str(provenance_path),
            ) from exc
        if provenance_path.relative_to(package_root).as_posix() != _PROVENANCE_PATH:
            raise ImportFailure(
                "world package provenance must use the canonical package-local path",
                code=ImportCode.PROJECTION_INVALID,
                context="provenance.path",
            )
        if not provenance_path.is_file():
            raise ImportFailure(
                "world package provenance is missing or unreadable",
                code=ImportCode.PROJECTION_INVALID,
                context=str(provenance_path),
            )
        projection_package = require_mapping(projection["package"], "projection.package")
        if projection_package != {
            "id": manifest["id"],
            "version": manifest["version"],
            "manifest": _MANIFEST_PATH,
            "provenance": _PROVENANCE_PATH,
        }:
            raise ImportFailure(
                "visual projection package provenance does not match the world manifest",
                code=ImportCode.PROJECTION_INVALID,
                context="projection.package",
            )
        if projection_path.relative_to(package_root).as_posix() != _PROJECTION_PATH:
            raise ImportFailure("visual projection escaped its package", code=ImportCode.UNSAFE_SOURCE)
        return {
            "status": "passed",
            "manifest_projection": _PROJECTION_PATH,
            "manifest_provenance": _PROVENANCE_PATH,
        }

    def _validate_entity_graph(self, mjcf_path: Path, entities: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
        try:
            root = ET.parse(mjcf_path).getroot()  # noqa: S314 - the importer generated this MJCF.
        except (OSError, ET.ParseError) as exc:
            raise ImportFailure(f"cannot parse generated world XML: {exc}", code=ImportCode.MODEL_INVALID) from exc
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ImportFailure("generated world XML has no worldbody", code=ImportCode.MODEL_INVALID)
        expected = {
            str(entity["entity_id"]): f"entity_{entity['entity_id']}"
            for entity in entities
            if entity["physics_proxy"] == "mujoco"
        }
        actual: dict[str, str] = {}
        for body in worldbody.findall("body"):
            name = body.attrib.get("name", "")
            if not name.startswith("entity_"):
                continue
            entity_id = name.removeprefix("entity_")
            if entity_id in actual:
                raise ImportFailure(
                    "entity graph emitted duplicate physics bodies", code=ImportCode.MODEL_INVALID, context=entity_id
                )
            geoms = body.findall("geom")
            if len(geoms) != 1 or geoms[0].attrib.get("name") != f"entity_{entity_id}_collision":
                raise ImportFailure(
                    "each mujoco physics entity must emit exactly one static collision geom",
                    code=ImportCode.MODEL_INVALID,
                    context=entity_id,
                )
            actual[entity_id] = name
        if actual != expected:
            raise ImportFailure(
                "entity graph physics proxies do not match generated MuJoCo collision bodies",
                code=ImportCode.MODEL_INVALID,
                details={"expected": sorted(expected), "actual": sorted(actual)},
            )
        return {"status": "passed", "expected_mujoco_entities": sorted(expected), "emitted_bodies": sorted(actual)}

    def _compile_mjcf(self, mjcf_path: Path) -> dict[str, Any]:
        try:
            import mujoco  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportFailure(
                "MuJoCo is required to compile imported world XML",
                code=ImportCode.CONVERTER_UNAVAILABLE,
                context="mujoco",
            ) from exc
        try:
            model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        except Exception as exc:  # MuJoCo exposes compile errors as several exception types.
            raise ImportFailure(
                f"generated world XML failed MuJoCo compilation: {exc}",
                code=ImportCode.MODEL_INVALID,
                context=str(mjcf_path),
            ) from exc
        return {
            "status": "passed",
            "backend": "mujoco",
            "nhfield": int(model.nhfield),
            "ngeom": int(model.ngeom),
            "nbody": int(model.nbody),
        }

    def _write_exclusive_json(self, directory: Path, name: str, value: Any, token: str) -> Path:
        candidate = directory / name
        if candidate.exists():
            candidate = directory / f"{candidate.stem}-{token}{candidate.suffix}"
        with candidate.open("xb") as stream:
            stream.write(canonical_json_bytes(value))
        return candidate

    def _positive_int(self, value: Any, context: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ImportFailure(f"{context} must be a positive integer", context=context)
        return value

    def _finite_float(self, value: Any, context: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ImportFailure(f"{context} must be finite numeric data", context=context)
        result = float(value)
        if result != result or result in {float("inf"), float("-inf")}:
            raise ImportFailure(f"{context} must be finite numeric data", context=context)
        return result

    def _float_vector(self, value: Any, size: int, context: str) -> list[float]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
            raise ImportFailure(f"{context} must contain exactly {size} values", context=context)
        return [self._finite_float(item, f"{context}[{index}]") for index, item in enumerate(value)]


def _validate_json_schema(instance: Any, schema: Any, root: Mapping[str, Any], path: str) -> None:
    """Validate the subset of JSON Schema used by the simulation package schemas."""

    if schema is True:
        return
    if schema is False:
        raise _SchemaValidationError(f"{path}: schema rejects the value")
    if not isinstance(schema, Mapping):
        raise _SchemaValidationError(f"{path}: schema must be an object")
    reference = schema.get("$ref")
    if reference is not None:
        if not isinstance(reference, str) or not reference.startswith("#/"):
            raise _SchemaValidationError(f"{path}: unsupported schema reference {reference!r}")
        target: Any = root
        for part in reference[2:].split("/"):
            if not isinstance(target, Mapping) or part.replace("~1", "/").replace("~0", "~") not in target:
                raise _SchemaValidationError(f"{path}: unresolved schema reference {reference!r}")
            target = target[part.replace("~1", "/").replace("~0", "~")]
        _validate_json_schema(instance, target, root, path)
        return

    if "const" in schema and instance != schema["const"]:
        raise _SchemaValidationError(f"{path}: expected {schema['const']!r}")
    if "enum" in schema and instance not in schema["enum"]:
        raise _SchemaValidationError(f"{path}: value is not in the declared enum")

    if "type" in schema:
        expected = schema["type"]
        expected_types = expected if isinstance(expected, list) else [expected]
        if not any(_json_type_matches(instance, item) for item in expected_types):
            raise _SchemaValidationError(f"{path}: expected type {expected!r}")

    if isinstance(instance, str):
        if len(instance) < int(schema.get("minLength", 0)):
            raise _SchemaValidationError(f"{path}: string is shorter than minLength")
        pattern = schema.get("pattern")
        if pattern is not None and re.search(str(pattern), instance) is None:
            raise _SchemaValidationError(f"{path}: string does not match pattern")

    if isinstance(instance, (int, float)) and not isinstance(instance, bool):
        if "minimum" in schema and instance < schema["minimum"]:
            raise _SchemaValidationError(f"{path}: number is below minimum")
        if "exclusiveMinimum" in schema:
            minimum = schema["exclusiveMinimum"]
            if isinstance(minimum, bool):
                if minimum and instance <= 0:
                    raise _SchemaValidationError(f"{path}: number is not above zero")
            elif instance <= minimum:
                raise _SchemaValidationError(f"{path}: number is not above exclusiveMinimum")

    if isinstance(instance, list):
        if len(instance) < int(schema.get("minItems", 0)):
            raise _SchemaValidationError(f"{path}: array is shorter than minItems")
        if "maxItems" in schema and len(instance) > int(schema["maxItems"]):
            raise _SchemaValidationError(f"{path}: array is longer than maxItems")
        prefix_items = schema.get("prefixItems", [])
        if isinstance(prefix_items, list):
            for index, item_schema in enumerate(prefix_items):
                if index < len(instance):
                    _validate_json_schema(instance[index], item_schema, root, f"{path}[{index}]")
        items = schema.get("items")
        if items is False and len(instance) > len(prefix_items):
            raise _SchemaValidationError(f"{path}: array has items beyond prefixItems")
        if isinstance(items, Mapping) or items is True or items is False:
            start = len(prefix_items) if isinstance(prefix_items, list) else 0
            for index, item in enumerate(instance[start:], start=start):
                _validate_json_schema(item, items, root, f"{path}[{index}]")

    if isinstance(instance, Mapping):
        required = schema.get("required", [])
        for key in required:
            if key not in instance:
                raise _SchemaValidationError(f"{path}: missing required property {key!r}")
        properties = schema.get("properties", {})
        if not isinstance(properties, Mapping):
            raise _SchemaValidationError(f"{path}: properties must be an object")
        if schema.get("additionalProperties") is False:
            unknown = sorted(set(instance) - set(properties))
            if unknown:
                raise _SchemaValidationError(f"{path}: unknown properties {unknown!r}")
        for key, item_schema in properties.items():
            if key in instance:
                _validate_json_schema(instance[key], item_schema, root, f"{path}.{key}")

    for item_schema in schema.get("allOf", []):
        _validate_json_schema(instance, item_schema, root, path)
    if "oneOf" in schema:
        matches = 0
        for item_schema in schema["oneOf"]:
            try:
                _validate_json_schema(instance, item_schema, root, path)
            except _SchemaValidationError:
                continue
            matches += 1
        if matches != 1:
            raise _SchemaValidationError(f"{path}: expected exactly one oneOf schema, got {matches}")
    if "anyOf" in schema:
        for item_schema in schema["anyOf"]:
            try:
                _validate_json_schema(instance, item_schema, root, path)
            except _SchemaValidationError:
                continue
            break
        else:
            raise _SchemaValidationError(f"{path}: expected at least one anyOf schema")
    if "if" in schema:
        try:
            _validate_json_schema(instance, schema["if"], root, path)
        except _SchemaValidationError:
            branch = schema.get("else")
        else:
            branch = schema.get("then")
        if branch is not None:
            _validate_json_schema(instance, branch, root, path)
    if "not" in schema:
        try:
            _validate_json_schema(instance, schema["not"], root, path)
        except _SchemaValidationError:
            pass
        else:
            raise _SchemaValidationError(f"{path}: not schema matched")


def _json_type_matches(value: Any, expected: Any) -> bool:
    if expected == "object":
        return isinstance(value, Mapping)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "null":
        return value is None
    return False


__all__ = ["WorldImporter"]
