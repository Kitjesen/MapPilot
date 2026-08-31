"""Publish compiler-validated SceneDrafts as immutable WorldPackages."""

from __future__ import annotations

import copy
import math
import struct
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from sim.importers import CatalogPromoter, WorldImporter
from sim.importers.contracts import (
    ImportDraft,
    ImportFailure,
    canonical_json_bytes,
    require_identity,
    require_mapping,
    require_string,
    strict_keys,
)

from .models import IdempotencyConflict, RevisionConflict, canonical_digest
from .scene_tools import FACTORY_PARK_WORLD_PACKAGE, FactoryParkSceneTool
from .store import StudioStore

_BASE_HFIELD = Path("sim/worlds/factory_park_hf/generated/heightfield_f32.bin")
_LICENSE_TEXT = "Copyright LingTu project contributors. Project-owned simulation content.\n"
_FACTORY_PARK_RUNTIME_MATERIALS = {
    "concrete": ([0.42, 0.43, 0.41, 1.0], 0.0, 0.88),
    "container_blue": ([0.04, 0.21, 0.48, 1.0], 0.12, 0.42),
    "container_orange": ([0.82, 0.32, 0.06, 1.0], 0.0, 0.4),
    "container_red": ([0.62, 0.055, 0.035, 1.0], 0.08, 0.4),
    "painted_steel": ([0.92, 0.74, 0.08, 1.0], 0.36, 0.3),
    "pallet_wood": ([0.48, 0.31, 0.15, 1.0], 0.0, 0.83),
    "road_marking": ([0.96, 0.86, 0.18, 0.9], 0.0, 0.55),
    "warehouse_cladding": ([0.54, 0.58, 0.58, 1.0], 0.4, 0.38),
}


class FactoryParkWorldVariantBuilder:
    """Derive one qualified world draft from trusted FactoryPark geometry.

    This is deliberately a world-specific compiler adapter.  The generic
    WorldImporter remains unaware of SceneDrafts and receives only canonical
    heightfield, entity, provenance, and package declarations.
    """

    def __init__(
        self,
        *,
        repo_root: Path,
        scene_tool: FactoryParkSceneTool,
        importer: WorldImporter | None = None,
    ) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.scene_tool = scene_tool
        self.importer = importer or WorldImporter(self.repo_root)

    def build(
        self,
        *,
        scene_draft_id: str,
        scene_draft_revision: int,
        batch: Mapping[str, Any],
        package: Mapping[str, Any],
        work_root: Path,
    ) -> tuple[ImportDraft, dict[str, Any]]:
        """Create a qualified draft and immutable source identity."""

        package_document = self._package(package)
        layout, compiled = self.scene_tool.compile_variant_source(batch)
        base_layout_digest = require_string(layout.get("layout_digest"), "layout.layout_digest")
        objects = self._objects(layout, compiled.objects)
        source_root = Path(work_root) / "source"
        source_root.mkdir(parents=True, exist_ok=False)

        grid = self._write_height_source(source_root, layout)
        source = {
            "schema": "lingtu.sim.studio.scene-publication-source.v1",
            "scene_draft_id": scene_draft_id,
            "scene_draft_revision": scene_draft_revision,
            "base_world_package": FACTORY_PARK_WORLD_PACKAGE,
            "base_layout_digest": base_layout_digest,
            "batch_digest": compiled.digest,
            "batch": copy.deepcopy(dict(batch)),
            "package": copy.deepcopy(package_document),
        }
        (source_root / "scene-draft.json").write_bytes(canonical_json_bytes(source))
        (source_root / "LICENSE.txt").write_text(_LICENSE_TEXT, encoding="utf-8")

        request = self._import_request(
            source_root=source_root,
            package=package_document,
            layout=layout,
            objects=objects,
            grid=grid,
            scene_draft_id=scene_draft_id,
            scene_draft_revision=scene_draft_revision,
        )
        draft = self.importer.import_world(request, draft_root=Path(work_root) / "draft")
        return draft, {
            "scene_draft_id": scene_draft_id,
            "scene_draft_revision": scene_draft_revision,
            "batch_digest": compiled.digest,
            "base_layout_digest": base_layout_digest,
        }

    @staticmethod
    def _package(value: Mapping[str, Any]) -> dict[str, str]:
        package = require_mapping(value, "package")
        strict_keys(
            package,
            required={"id", "version", "description"},
            optional=set(),
            context="package",
        )
        return {
            "id": require_identity(package["id"], "package.id"),
            "version": require_identity(package["version"], "package.version", version=True),
            "description": require_string(package["description"], "package.description"),
        }

    @staticmethod
    def _objects(
        layout: Mapping[str, Any],
        additions: Sequence[Mapping[str, object]],
    ) -> tuple[dict[str, Any], ...]:
        raw_objects = layout.get("objects")
        if isinstance(raw_objects, (str, bytes)) or not isinstance(raw_objects, Sequence):
            raise ImportFailure("FactoryPark layout objects are invalid", context="layout.objects")
        objects: list[dict[str, Any]] = []
        for raw, visual_mode in (
            *((raw, "level") for raw in raw_objects),
            *((raw, "runtime") for raw in additions),
        ):
            if not isinstance(raw, Mapping):
                raise ImportFailure("FactoryPark layout object is invalid", context="layout.objects")
            item = copy.deepcopy(dict(raw))
            item["__visual_mode"] = visual_mode
            objects.append(item)
        return tuple(objects)

    def _write_height_source(
        self,
        source_root: Path,
        layout: Mapping[str, Any],
    ) -> dict[str, Any]:
        path = (self.repo_root / _BASE_HFIELD).resolve()
        try:
            path.relative_to(self.repo_root)
        except ValueError as exc:
            raise ImportFailure("FactoryPark heightfield escapes repository", context=str(path)) from exc
        payload = path.read_bytes()
        if len(payload) < 8:
            raise ImportFailure("FactoryPark heightfield header is truncated", context=str(path))
        rows, columns = struct.unpack_from("<ii", payload)
        if rows < 2 or columns < 2 or len(payload) != 8 + rows * columns * 4:
            raise ImportFailure("FactoryPark heightfield dimensions are invalid", context=str(path))
        normalized = struct.unpack_from(f"<{rows * columns}f", payload, 8)
        if any(not math.isfinite(value) or value < 0.0 or value > 1.0 for value in normalized):
            raise ImportFailure("FactoryPark heightfield samples are invalid", context=str(path))

        samples: list[int] = []
        for row in reversed(range(rows)):
            start = row * columns
            samples.extend(round(value * 65_535.0) for value in normalized[start : start + columns])
        (source_root / "height.r16").write_bytes(struct.pack(f"<{len(samples)}H", *samples))

        coordinate = require_mapping(layout.get("coordinate_contract"), "layout.coordinate_contract")
        encoded = require_mapping(coordinate.get("height_encoding"), "layout.coordinate_contract.height_encoding")
        extent = coordinate.get("extent_m")
        grid = coordinate.get("grid_px")
        if (
            isinstance(extent, (str, bytes))
            or not isinstance(extent, Sequence)
            or len(extent) != 2
            or isinstance(grid, (str, bytes))
            or not isinstance(grid, Sequence)
            or list(grid) != [columns, rows]
        ):
            raise ImportFailure("FactoryPark coordinate contract does not match heightfield", context="layout")
        vertical_origin = float(coordinate.get("vertical_origin_m"))
        elevation_scale = float(encoded.get("elevation_scale_m"))
        if not math.isfinite(vertical_origin) or not math.isfinite(elevation_scale) or elevation_scale <= 0.0:
            raise ImportFailure("FactoryPark elevation contract is invalid", context="layout.coordinate_contract")
        return {
            "width": columns,
            "height": rows,
            "extent_m": [float(extent[0]), float(extent[1])],
            "elevation_min_m": vertical_origin,
            "elevation_max_m": vertical_origin + elevation_scale,
        }

    def _import_request(
        self,
        *,
        source_root: Path,
        package: Mapping[str, str],
        layout: Mapping[str, Any],
        objects: Sequence[Mapping[str, Any]],
        grid: Mapping[str, Any],
        scene_draft_id: str,
        scene_draft_revision: int,
    ) -> dict[str, Any]:
        spawn = require_mapping(layout.get("spawn"), "layout.spawn")
        spawn_position = spawn.get("position_m")
        if (
            isinstance(spawn_position, (str, bytes))
            or not isinstance(spawn_position, Sequence)
            or len(spawn_position) != 3
        ):
            raise ImportFailure("FactoryPark spawn is invalid", context="layout.spawn.position_m")
        return {
            "schema": "lingtu.sim.world-import-request.v1",
            "package": dict(package),
            "source": {
                "path": str(source_root),
                "provenance": {
                    "owner": "LingTu project",
                    "license": "LicenseRef-LingTu-Project-Owned",
                    "license_file": "LICENSE.txt",
                    "source_uri": (
                        f"simstudio://scene-drafts/{scene_draft_id}/revisions/"
                        f"{scene_draft_revision}"
                    ),
                    "third_party_assets": [],
                },
            },
            "units": {"length": "m", "up_axis": "Z", "handedness": "RH"},
            "heightmap": {"path": "height.r16", **dict(grid), "endian": "little"},
            "visual": {
                "binding": "WorldVisual:FactoryParkHF",
                "level": "/Game/RobotSim/Maps/FactoryPark_HF",
            },
            "spawn": {
                "position_m": [float(value) for value in spawn_position],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "height_tolerance_m": 1e-5,
            },
            "entities": [self._entity(item) for item in objects],
        }

    @staticmethod
    def _entity(item: Mapping[str, Any]) -> dict[str, Any]:
        shape = require_string(item.get("shape"), "layout.object.shape")
        collision = item.get("collision")
        if not isinstance(collision, bool):
            raise ImportFailure("layout object collision must be boolean", context="layout.object.collision")
        geometry: dict[str, Any]
        if shape == "box":
            size = item.get("size_m")
            if isinstance(size, (str, bytes)) or not isinstance(size, Sequence) or len(size) != 3:
                raise ImportFailure("layout box size is invalid", context="layout.object.size_m")
            geometry = {"shape": "box", "size_m": [float(value) for value in size]}
        elif shape == "cylinder":
            geometry = {
                "shape": "cylinder",
                "radius_m": float(item.get("radius_m")),
                "half_height_m": float(item.get("half_height_m")),
            }
        else:
            raise ImportFailure(f"unsupported FactoryPark shape {shape!r}", context="layout.object.shape")
        position = item.get("position_m")
        if isinstance(position, (str, bytes)) or not isinstance(position, Sequence) or len(position) != 3:
            raise ImportFailure("layout object position is invalid", context="layout.object.position_m")
        semantic_class = require_string(item.get("semantic_class"), "layout.object.semantic_class")
        visual_mode = require_string(item.get("__visual_mode"), "layout.object.__visual_mode")
        result = {
            "entity_id": require_identity(item.get("id"), "layout.object.id"),
            "entity_type": semantic_class,
            "authority": "mujoco" if collision else "ue_animation",
            "initial_transform": {
                "position_m": [float(value) for value in position],
                "quaternion_wxyz": FactoryParkWorldVariantBuilder._quaternion(
                    pitch_deg=float(item.get("pitch_deg", 0.0)),
                    yaw_deg=float(item.get("yaw_deg", 0.0)),
                ),
            },
            "physics_proxy": "mujoco" if collision else "none",
            "semantic_class": semantic_class,
            "collision": collision,
            "geometry": geometry,
        }
        if visual_mode == "level":
            result["visual"] = {"mode": "level"}
            return result
        if visual_mode != "runtime":
            raise ImportFailure("layout object visual mode is invalid", context="layout.object.__visual_mode")
        material_key = require_string(item.get("material"), "layout.object.material")
        rgba, metallic, roughness = _FACTORY_PARK_RUNTIME_MATERIALS.get(
            material_key,
            ([0.45, 0.48, 0.52, 1.0], 0.0, 0.65),
        )
        result["visual"] = {
            "mode": "runtime",
            "material": {
                "key": material_key,
                "base_color_rgba": list(rgba),
                "metallic": metallic,
                "roughness": roughness,
            },
        }
        return result

    @staticmethod
    def _quaternion(*, pitch_deg: float, yaw_deg: float) -> list[float]:
        """Match MuJoCo's default intrinsic ``xyz`` Euler sequence."""

        pitch = math.radians(pitch_deg) / 2.0
        yaw = math.radians(yaw_deg) / 2.0
        sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
        sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
        return [
            cos_pitch * cos_yaw,
            sin_pitch * sin_yaw,
            sin_pitch * cos_yaw,
            cos_pitch * sin_yaw,
        ]


class ScenePublicationService:
    """Coordinate immutable package promotion with SceneDraft revision history."""

    def __init__(
        self,
        *,
        store: StudioStore,
        scene_tool: FactoryParkSceneTool,
        repo_root: Path,
        builder: FactoryParkWorldVariantBuilder | None = None,
    ) -> None:
        self.store = store
        self.repo_root = Path(repo_root).resolve()
        self.builder = builder or FactoryParkWorldVariantBuilder(
            repo_root=self.repo_root,
            scene_tool=scene_tool,
        )

    def publish(
        self,
        scene_draft_id: str,
        *,
        revision: int,
        package: Mapping[str, Any],
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Publish the exact requested revision without mutating an old version."""

        if idempotency_key is not None:
            replay = self._idempotent_replay(
                scene_draft_id,
                revision=revision,
                package=package,
                idempotency_key=idempotency_key,
            )
            if replay is not None:
                return replay
        current = self.store.get_scene_draft(scene_draft_id)
        if current.revision != revision:
            raise RevisionConflict(
                f"stale scene_draft revision for {scene_draft_id}: expected {revision}, current {current.revision}",
                expected=revision,
                actual=current.revision,
            )
        payload = copy.deepcopy(dict(current.payload))
        batch = payload.get("batch")
        if not isinstance(batch, Mapping):
            raise ImportFailure("SceneDraft batch is invalid", context=scene_draft_id)

        # Keep the importer-owned evidence tree short on Windows.  A Studio
        # root can already sit below a long user profile or pytest path, and
        # qualification adds several fixed nested components.
        with tempfile.TemporaryDirectory(prefix="ltsp-") as temporary:
            draft, source = self.builder.build(
                scene_draft_id=scene_draft_id,
                scene_draft_revision=revision,
                batch=batch,
                package=package,
                work_root=Path(temporary),
            )
            promoted = CatalogPromoter(self.repo_root).promote(draft)

        publication = self._publication(promoted, source)
        publications = payload.get("publications", [])
        if not isinstance(publications, list):
            raise ImportFailure("SceneDraft publications are invalid", context=scene_draft_id)
        payload["publications"] = [*publications, copy.deepcopy(publication)]
        updated = self.store.update_scene_draft(
            scene_draft_id,
            expected_revision=revision,
            payload=payload,
            status="published",
            idempotency_key=idempotency_key,
        )
        return self._result(updated.to_dict(), publication)

    def _idempotent_replay(
        self,
        scene_draft_id: str,
        *,
        revision: int,
        package: Mapping[str, Any],
        idempotency_key: str,
    ) -> dict[str, Any] | None:
        existing = self.store.get_idempotency("update:scene_draft", idempotency_key)
        if existing is None:
            return None
        response = copy.deepcopy(dict(existing.response))
        payload = response.get("payload")
        if not isinstance(payload, Mapping):
            raise IdempotencyConflict("scene publication idempotency response is invalid")
        publications = payload.get("publications")
        if not isinstance(publications, list) or not publications:
            raise IdempotencyConflict("idempotency key does not belong to a scene publication")
        publication = publications[-1]
        if not isinstance(publication, Mapping):
            raise IdempotencyConflict("scene publication idempotency response is invalid")
        expected = {
            "kind": "scene_draft",
            "record_id": scene_draft_id,
            "expected_revision": revision,
            "payload": dict(payload),
            "status": "published",
        }
        if existing.request_digest != canonical_digest(expected):
            raise IdempotencyConflict("idempotency key already belongs to a different scene publication")
        package_identity = FactoryParkWorldVariantBuilder._package(package)
        promoted_package = publication.get("package")
        if not isinstance(promoted_package, Mapping) or promoted_package.get("ref") != (
            f"{package_identity['id']}@{package_identity['version']}"
        ):
            raise IdempotencyConflict("idempotency key already belongs to a different package identity")
        return self._result(response, dict(publication))

    def _publication(self, promoted: Any, source: Mapping[str, Any]) -> dict[str, Any]:
        value = promoted.to_dict()
        result = {
            "schema": "lingtu.sim.studio.world-publication.v1",
            "package": copy.deepcopy(value["package"]),
            "source": copy.deepcopy(dict(source)),
        }
        for field in ("package_root", "qualification_path"):
            path = Path(value[field]).resolve()
            try:
                result[field] = path.relative_to(self.repo_root).as_posix()
            except ValueError as exc:
                raise ImportFailure(f"promotion {field} escapes repository", context=field) from exc
        return result

    @staticmethod
    def _result(scene_draft: Mapping[str, Any], publication: Mapping[str, Any]) -> dict[str, Any]:
        source = publication.get("source")
        if not isinstance(source, Mapping):
            raise ImportFailure("scene publication source is invalid", context="publication.source")
        return {
            "schema": "lingtu.sim.studio.scene-publication.v1",
            "scene_draft": copy.deepcopy(dict(scene_draft)),
            "publication": copy.deepcopy({key: value for key, value in publication.items() if key != "source"}),
            "source": copy.deepcopy(dict(source)),
        }


__all__ = ["FactoryParkWorldVariantBuilder", "ScenePublicationService"]
