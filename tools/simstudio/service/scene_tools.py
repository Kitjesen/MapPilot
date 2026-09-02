"""Read-only SceneTool projections for server-authoritative simulation worlds."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

from sim.tools.worlds.factory_park_hf.elements import (
    ELEMENT_BATCH_SCHEMA,
    CompiledElementBatch,
    compile_element_batch,
    element_catalog,
)

FACTORY_PARK_SCENE_TOOL = "factory-park-hf"
FACTORY_PARK_WORLD_PACKAGE = "factory_park_hf@1.0.0"
_EXPANDED_LAYOUT = Path("sim/packages/worlds/factory_park_hf/generated/expanded-layout.json")


def _canonical_json_bytes(value: object) -> bytes:
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


def _finite_number(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _finite_vector(value: object, length: int, *, positive: bool = False) -> bool:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        return False
    return len(value) == length and all(
        _finite_number(item) and (not positive or float(item) > 0.0)
        for item in value
    )


class SceneToolValidationError(ValueError):
    """Report a rejected declarative batch through the stable HTTP envelope."""

    code = "SIMSTUDIO_SCENE_ELEMENT_BATCH_INVALID"

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.details = {"scene_tool": FACTORY_PARK_SCENE_TOOL}


class SceneToolUnavailableError(RuntimeError):
    """Hide server filesystem details when the trusted layout is unavailable."""


class FactoryParkSceneTool:
    """Expose safe FactoryPark_HF authoring declarations without execution."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = Path(repo_root).resolve()

    def catalog(self) -> dict[str, object]:
        """Return the fixed element catalog and its authority declarations."""

        layout = self._load_layout()
        templates = element_catalog()
        allowed_surface_classes = {
            str(surface_class)
            for template in templates.values()
            for surface_class in template["allowed_surface_classes"]
        }
        surfaces = []
        for item in layout["objects"]:
            if (
                isinstance(item, Mapping)
                and item.get("shape") == "box"
                and item.get("semantic_class") in allowed_surface_classes
            ):
                position = item["position_m"]
                size = item["size_m"]
                surfaces.append(
                    {
                        "surface_id": str(item["id"]),
                        "semantic_class": str(item["semantic_class"]),
                        "position_xy_m": [float(position[0]), float(position[1])],
                        "size_xy_m": [float(size[0]), float(size[1])],
                        "yaw_deg": float(item.get("yaw_deg", 0.0)),
                    }
                )
        spawn = layout["spawn"]
        return {
            "schema": "lingtu.sim.studio.scene-tool-catalog.v1",
            "scene_tool": FACTORY_PARK_SCENE_TOOL,
            "world_package": FACTORY_PARK_WORLD_PACKAGE,
            "element_batch_schema": ELEMENT_BATCH_SCHEMA,
            "read_only": True,
            "executes_runtime": False,
            "element_types": templates,
            "layout_digest": layout["layout_digest"],
            "surfaces": sorted(surfaces, key=lambda item: item["surface_id"]),
            "spawn": {
                "position_xy_m": [
                    float(spawn["position_m"][0]),
                    float(spawn["position_m"][1]),
                ],
                "clearance_radius_m": float(spawn["clearance_radius_m"]),
            },
        }

    def validate_element_batch(self, batch: Mapping[str, object]) -> dict[str, object]:
        """Compile a batch in memory and return its deterministic public summary."""

        layout, compiled = self.compile_variant_source(batch)

        elements: list[dict[str, str]] = []
        for item in compiled.objects:
            provenance = item.get("element_provenance")
            if not isinstance(provenance, Mapping):
                raise SceneToolUnavailableError(
                    "factory-park-hf compiler returned invalid provenance"
                )
            visual_only = item.get("visual_only")
            collision = item.get("collision")
            if visual_only is True and collision is False:
                authority = "VisualOnly"
            elif visual_only is False and collision is True:
                authority = "PhysicsShared"
            else:
                raise SceneToolUnavailableError(
                    "factory-park-hf compiler returned invalid authority"
                )
            elements.append(
                {
                    "stable_id": str(item["id"]),
                    "element_type": str(provenance["element_type"]),
                    "authority": authority,
                }
            )

        return {
            "schema": "lingtu.sim.studio.scene-tool-validation.v1",
            "scene_tool": FACTORY_PARK_SCENE_TOOL,
            "world_package": FACTORY_PARK_WORLD_PACKAGE,
            "valid": True,
            "batch_id": compiled.batch_id,
            "digest": compiled.digest,
            "layout_digest": layout["layout_digest"],
            "stable_ids": [item["stable_id"] for item in elements],
            "elements": elements,
            "diagnostics": [],
        }

    def compile_variant_source(
        self,
        batch: Mapping[str, object],
    ) -> tuple[dict[str, object], CompiledElementBatch]:
        """Return detached authoritative geometry for package publication.

        HTTP callers only receive ``validate_element_batch`` summaries.  This
        deeper seam is reserved for trusted publication code that must derive
        MuJoCo collision geometry from the exact same compiler decision.
        """

        layout = self._load_layout()
        try:
            compiled = compile_element_batch(batch, layout)
        except ValueError as exc:
            raise SceneToolValidationError(str(exc)) from exc
        except (KeyError, TypeError) as exc:
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout is incompatible with the compiler"
            ) from exc
        return copy.deepcopy(dict(layout)), compiled

    def _load_layout(self) -> Mapping[str, object]:
        requested = self._repo_root / _EXPANDED_LAYOUT
        try:
            layout_path = requested.resolve(strict=True)
            layout_path.relative_to(self._repo_root)
            raw = json.loads(layout_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout is unavailable"
            ) from exc
        if not isinstance(raw, Mapping):
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout must be an object"
            )
        if raw.get("schema") != "lingtu.sim.expanded-world-layout.v1":
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout schema is invalid"
            )
        if raw.get("world_package") != FACTORY_PARK_WORLD_PACKAGE:
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout package is invalid"
            )
        digest = raw.get("layout_digest")
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout digest is invalid"
            )
        layout_without_digest = dict(raw)
        layout_without_digest.pop("layout_digest", None)
        try:
            actual_digest = hashlib.sha256(
                _canonical_json_bytes(layout_without_digest)
            ).hexdigest()
        except (TypeError, ValueError) as exc:
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout content is invalid"
            ) from exc
        if actual_digest != digest:
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout digest is stale"
            )
        objects = raw.get("objects")
        if (
            isinstance(objects, (str, bytes))
            or not isinstance(objects, Sequence)
            or not objects
        ):
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout objects are invalid"
            )
        object_ids: set[str] = set()
        for item in objects:
            if not isinstance(item, Mapping):
                raise SceneToolUnavailableError(
                    "factory-park-hf authoritative layout objects are invalid"
                )
            stable_id = item.get("id")
            shape = item.get("shape")
            if (
                not isinstance(stable_id, str)
                or not stable_id
                or stable_id in object_ids
                or not isinstance(item.get("semantic_class"), str)
                or not _finite_vector(item.get("position_m"), 3)
                or not _finite_number(item.get("yaw_deg", 0.0))
                or shape not in {"box", "cylinder"}
            ):
                raise SceneToolUnavailableError(
                    "factory-park-hf authoritative layout objects are invalid"
                )
            object_ids.add(stable_id)
            geometry_valid = (
                _finite_vector(item.get("size_m"), 3, positive=True)
                if shape == "box"
                else _finite_number(item.get("radius_m"))
                and float(item["radius_m"]) > 0.0
                and _finite_number(item.get("half_height_m"))
                and float(item["half_height_m"]) > 0.0
            )
            if not geometry_valid:
                raise SceneToolUnavailableError(
                    "factory-park-hf authoritative layout objects are invalid"
                )
        spawn = raw.get("spawn")
        if (
            not isinstance(spawn, Mapping)
            or not _finite_vector(spawn.get("position_m"), 3)
            or not _finite_number(spawn.get("clearance_radius_m"))
            or float(spawn["clearance_radius_m"]) < 0.0
        ):
            raise SceneToolUnavailableError(
                "factory-park-hf authoritative layout spawn is invalid"
            )
        return raw


__all__ = [
    "FACTORY_PARK_SCENE_TOOL",
    "FACTORY_PARK_WORLD_PACKAGE",
    "FactoryParkSceneTool",
    "SceneToolUnavailableError",
    "SceneToolValidationError",
]
