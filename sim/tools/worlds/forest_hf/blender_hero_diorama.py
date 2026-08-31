"""Author a standalone, review-only Forest_HF hero diorama in Blender.

This module deliberately does not modify or export the canonical simulation
world.  It composes conditioned visual assets around a local S-curved trail and
renders one art-review image.  MuJoCo remains the sole physics and collision
authority; every scene item is marked ``VisualOnly`` / ``NoCollision``.

The planning API imports without Blender so its deterministic composition can
be regression-tested with ordinary Python.  Scene authoring is available only
when this file is executed by Blender's Python runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import re
import sys
from copy import deepcopy
from itertools import pairwise
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

_REPO_IMPORT_ROOT = Path(__file__).resolve().parents[4]
if str(_REPO_IMPORT_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_IMPORT_ROOT))

from sim.tools.worlds.forest_hf import blender_author as author
from sim.tools.worlds.forest_hf.blender_visuals import (
    build_procedural_materials,
    build_visual_templates,
    fern_clump_mesh_data,
    grass_clump_mesh_data,
    setup_morning_fog,
)

try:  # Blender is intentionally optional for contract tests.
    import bpy  # type: ignore[import-not-found]
except ModuleNotFoundError:  # pragma: no cover - ordinary Python path.
    bpy = None


DEFAULT_SEED = 5808
DEFAULT_OUTPUT_DIR = Path("build/forest-hf/review/hero-diorama-v001")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_RUNTIME_CONTRACT = {
    "classification": "VisualOnly",
    "collision_profile": "NoCollision",
    "physics_authority": "MuJoCo",
}
_UNDERSTORY_SLOT_IDS = frozenset(
    {"grass_clump", "fern_clump", "forest_floor_debris"}
)
_UNDERSTORY_TEMPLATE_KEYS = {
    "grass_clump": "grass",
    "fern_clump": "fern",
    "forest_floor_debris": "forest_floor_debris",
}
_UNDERSTORY_TEMPLATE_NAMES = {
    semantic_class: f"LT_HeroDiorama_Conditioned_{semantic_class.title().replace('_', '')}"
    for semantic_class in _UNDERSTORY_SLOT_IDS
}
_CONDITIONED_UNDERSTORY_HEIGHT_M = {
    "grass": 0.22,
    "fern": 0.43,
    "forest_floor_debris": 0.10,
}
_OUTPUTS = [
    {
        "artifact_role": "hero_diorama_preview",
        "filename": "forest_hf.hero-diorama.png",
    },
    {
        "artifact_role": "review_manifest",
        "filename": "forest_hf.hero-diorama.review.json",
    },
]

_TRAIL_CONTROL_POINTS = (
    (-48.0, -6.0, 0.15),
    (-37.0, -3.0, 0.40),
    (-25.0, 2.5, 0.85),
    (-13.0, 6.0, 1.20),
    (0.0, 6.5, 1.05),
    (13.0, 2.0, 1.40),
    (26.0, -4.0, 2.05),
    (38.0, -5.0, 2.55),
    (48.0, -1.0, 2.80),
)
_HERO_CAMERA_XY = (-32.0, -4.0)
_HERO_ROBOT_XY = (-21.5, 3.9)
_FERN_TEMPLATE_HEIGHT_M = max(vertex[2] for vertex in fern_clump_mesh_data()["vertices"])
_GRASS_TEMPLATE_HEIGHT_M = max(vertex[2] for vertex in grass_clump_mesh_data()["vertices"])


def hero_ground_surface_contract() -> dict[str, Any]:
    """Describe the review ground's deterministic, multi-scale surface language."""

    moss_coverage = 0.12
    moss_threshold = 1.0 - moss_coverage
    moss_transition = 0.035
    return {
        "schema": "lingtu.sim.forest-hero-ground-surface.v1",
        "surface_classes": {
            "moist_soil": {"color": "#3A2A1B", "coverage": 0.58},
            "forest_humus": {"color": "#21170F", "coverage": 0.30},
            "moss": {"color": "#4B5334", "coverage": moss_coverage},
        },
        "moss_mask": {
            "low_threshold": moss_threshold - moss_transition,
            "high_threshold": moss_threshold + moss_transition,
        },
        "coordinate_space": "object_local_metres",
        "noise_coordinate_input": "Object",
        "detail_layers": [
            {
                "layer_id": "micro_grain",
                "scale_m": 0.045,
                "noise_scale_per_m": 1.0 / 0.045,
                "role": "roughness_and_bump",
            },
            {
                "layer_id": "humus_breakup",
                "scale_m": 0.55,
                "noise_scale_per_m": 1.0 / 0.55,
                "role": "organic_colour",
            },
            {
                "layer_id": "moss_moisture",
                "scale_m": 7.5,
                "noise_scale_per_m": 1.0 / 7.5,
                "role": "broad_patch",
            },
        ],
        "roughness": 0.87,
        "roughness_range": [0.78, 0.94],
        "normal_strength": 0.16,
    }


def hero_ground_detail_counts() -> dict[str, int]:
    """Return the combined ground-detail budget used by the review scene."""

    return {
        "wet_leaves": 8400,
        "fallen_twigs": 320,
        "tree_base_moss": 210,
    }


def hero_trail_surface_contract() -> dict[str, Any]:
    """Describe the trail's metre-scaled wet-mud material independently of Blender."""

    return {
        "schema": "lingtu.sim.forest-hero-trail-surface.v1",
        "coordinate_space": "object_local_metres",
        "noise_coordinate_input": "Object",
        "detail_layers": [
            {"layer_id": "micro_grit", "scale_m": 0.045, "role": "roughness_and_bump"},
            {"layer_id": "mud_breakup", "scale_m": 0.52, "role": "organic_colour"},
            {"layer_id": "broad_dampness", "scale_m": 5.8, "role": "broad_patch"},
        ],
        "surface_colors": {
            "packed_humus": "#24170F",
            "warm_wet_soil": "#57331D",
            "deep_damp_patch": "#150E0A",
        },
        "roughness_range": [0.68, 0.88],
        "normal_strength": 0.13,
    }


def hero_lighting_contract() -> dict[str, Any]:
    """Return the softened neutral-lighting contract for the hero review."""

    return {
        "schema": "lingtu.sim.forest-hero-lighting.v1",
        "sun": {"energy": 2.8, "angle_deg": 6.0},
        "world_strength": 0.18,
        "fill": {
            "energy": 145.0,
            "color": [0.78, 0.76, 0.72],
            "size_m": 20.0,
        },
        "exposure": 0.14,
        "fog_density": 0.00082,
        "fog_anisotropy": 0.14,
    }


def hero_ground_noise_frequencies(contract: Mapping[str, Any]) -> dict[str, float]:
    """Resolve texture frequencies in cycles per object-local metre."""

    if contract.get("noise_coordinate_input") != "Object":
        raise ValueError("hero ground noise must use object-local metre coordinates")
    return {
        str(layer["layer_id"]): 1.0 / float(layer["scale_m"])
        for layer in contract["detail_layers"]
    }


def trail_shoulder_surface_contract() -> dict[str, Any]:
    """Describe the feathered shoulder material independently of Blender."""

    return {
        "schema": "lingtu.sim.forest-hero-trail-shoulder-surface.v1",
        "coordinate_space": "object_local_metres",
        "noise_coordinate_input": "Object",
        "blend_attribute": "trail_blend",
        "outer_alpha": 0.0,
        "inner_alpha": 1.0,
        "noise_scale_per_m": 1.0 / 0.28,
        "render_method_preference": ["DITHERED", "BLENDED"],
    }


def _visual_item(**values: Any) -> dict[str, Any]:
    return {
        **values,
        "classification": "VisualOnly",
        "collision_profile": "NoCollision",
        "collision": False,
        "simulate_physics": False,
        "can_ever_affect_navigation": False,
        "review_only": True,
    }


def _path_sample_at_x(x: float) -> tuple[float, float]:
    """Return the trail centre Y and elevation at one local X coordinate."""

    if x <= _TRAIL_CONTROL_POINTS[0][0]:
        return _TRAIL_CONTROL_POINTS[0][1], _TRAIL_CONTROL_POINTS[0][2]
    for first, second in pairwise(_TRAIL_CONTROL_POINTS):
        if x <= second[0]:
            amount = (x - first[0]) / max(second[0] - first[0], 1e-9)
            return (
                first[1] + (second[1] - first[1]) * amount,
                first[2] + (second[2] - first[2]) * amount,
            )
    return _TRAIL_CONTROL_POINTS[-1][1], _TRAIL_CONTROL_POINTS[-1][2]


def _terrain_height(x: float, y: float) -> float:
    """Return shared trail-aware relief for terrain, props, and robot grounding."""

    path_y, path_z = _path_sample_at_x(x)
    lateral = y - path_y
    bank = 0.095 * max(0.0, abs(lateral) - 2.2) ** 1.22
    if lateral < 0.0:
        bank *= 0.82
    attenuation = min(1.0, max(0.10, (abs(lateral) - 1.6) / 10.0))
    low_frequency = (
        0.72 * math.sin((x + 18.0) / 20.0)
        + 0.36 * math.cos((y - 9.0) / 11.0)
        + 0.15 * math.sin((x + y) / 4.8)
    )
    return float(path_z + bank + low_frequency * attenuation)


def _catmull_rom_centerline(subdivisions: int = 10) -> list[list[float]]:
    """Resample the art-directed trail into a smooth deterministic S curve."""

    controls = [(point[0], point[1]) for point in _TRAIL_CONTROL_POINTS]
    padded = [controls[0], *controls, controls[-1]]
    points: list[list[float]] = []
    for index in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[index - 1 : index + 3]
        for step in range(subdivisions):
            t = step / subdivisions
            t2, t3 = t * t, t * t * t
            x = 0.5 * (
                2.0 * p1[0]
                + (-p0[0] + p2[0]) * t
                + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
            )
            y = 0.5 * (
                2.0 * p1[1]
                + (-p0[1] + p2[1]) * t
                + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
            )
            points.append([round(x, 5), round(y, 5)])
    points.append([controls[-1][0], controls[-1][1]])
    return points


def _distance_to_segment(
    point: tuple[float, float],
    start: Sequence[float],
    end: Sequence[float],
) -> float:
    dx, dy = float(end[0]) - float(start[0]), float(end[1]) - float(start[1])
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.dist(point, (float(start[0]), float(start[1])))
    amount = max(
        0.0,
        min(
            1.0,
            ((point[0] - float(start[0])) * dx + (point[1] - float(start[1])) * dy)
            / length_squared,
        ),
    )
    closest = (float(start[0]) + dx * amount, float(start[1]) + dy * amount)
    return math.dist(point, closest)


def _distance_to_centerline(point: tuple[float, float], centerline: Sequence[Sequence[float]]) -> float:
    return min(
        _distance_to_segment(point, start, end)
        for start, end in pairwise(centerline)
    )


def _clear_of_hero_view(point: tuple[float, float]) -> bool:
    """Reserve an unobstructed camera-to-robot corridor for the review subject."""

    if math.dist(point, _HERO_CAMERA_XY) < 7.0:
        return False
    if math.dist(point, _HERO_ROBOT_XY) < 4.0:
        return False
    return _distance_to_segment(point, _HERO_CAMERA_XY, (-1.0, 18.0)) >= 1.45


def _tree_instances(rng: random.Random, centerline: Sequence[Sequence[float]]) -> list[dict[str, Any]]:
    bands = {
        "near": (-50.0, -16.0, 90),
        "mid": (-18.0, 23.0, 100),
        "far": (18.0, 56.0, 100),
    }
    instances: list[dict[str, Any]] = []
    index = 0
    for depth_band, (low_x, high_x, count) in bands.items():
        birch_count = 27 if depth_band == "near" else 30
        species_order = ["birch"] * birch_count + ["pine"] * (count - birch_count)
        rng.shuffle(species_order)
        cluster_fractions = (0.04, 0.13, 0.24, 0.37, 0.49, 0.63, 0.76, 0.88, 0.96)
        cluster_centres = [
            (
                low_x + (high_x - low_x) * fraction + rng.uniform(-2.2, 2.2),
                side * rng.uniform(4.5, 16.0),
            )
            for fraction, side in zip(
                cluster_fractions,
                (-1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0),
            )
        ]
        for local_index in range(count):
            accepted: tuple[float, float] | None = None
            for _attempt in range(600):
                centre_x, lateral = rng.choices(
                    cluster_centres,
                    weights=(7, 13, 9, 15, 8, 14, 6, 11, 5),
                    k=1,
                )[0]
                x = max(-57.0, min(57.0, centre_x + rng.gauss(0.0, 3.3)))
                path_y, _path_z = _path_sample_at_x(x)
                y = max(-46.0, min(46.0, path_y + lateral + rng.gauss(0.0, 2.5)))
                point = (x, y)
                if _distance_to_centerline(point, centerline) < 3.5:
                    continue
                if not _clear_of_hero_view(point):
                    continue
                if any(math.dist(point, tuple(tree["position_m"][:2])) < 1.25 for tree in instances):
                    continue
                accepted = point
                break
            if accepted is None:
                raise RuntimeError("could not place non-overlapping hero forest tree")
            x, y = accepted
            species = species_order[local_index]
            base_scale = rng.uniform(0.78, 1.42) if species == "pine" else rng.uniform(0.74, 1.30)
            xy_variation = rng.uniform(0.92, 1.08)
            instances.append(
                _visual_item(
                    instance_id=f"hero.tree.{index:03d}",
                    species_id=species,
                    depth_band=depth_band,
                    position_m=[round(x, 5), round(y, 5), 0.0],
                    yaw_deg=round(rng.uniform(0.0, 360.0), 4),
                    tilt_x_deg=round(rng.uniform(-3.2, 3.2), 4),
                    tilt_y_deg=round(rng.uniform(-3.2, 3.2), 4),
                    scale=[
                        round(base_scale * xy_variation, 4),
                        round(base_scale / xy_variation, 4),
                        round(base_scale * rng.uniform(0.92, 1.08), 4),
                    ],
                )
            )
            index += 1
    return instances


def _dressing_instances(
    rng: random.Random,
    centerline: Sequence[Sequence[float]],
    *,
    include_conditioned_debris: bool = False,
) -> list[dict[str, Any]]:
    counts = {"boulder": 18, "fern": 220, "grass": 960}
    dressing: list[dict[str, Any]] = []
    patch_centres: dict[str, list[tuple[str, str, float, float]]] = {}
    for kind, patch_count in (("fern", 26), ("grass", 82)):
        patches: list[tuple[str, str, float, float]] = []
        bands = (
            ("near", -39.0, -13.0, 0.44),
            ("mid", -13.0, 20.0, 0.41),
            ("far", 20.0, 54.0, 0.15),
        )
        allocations = [round(patch_count * band[3]) for band in bands]
        allocations[1] += patch_count - sum(allocations)
        for (depth_band, low_x, high_x, _weight), allocation in zip(bands, allocations):
            for local_index in range(allocation):
                x = rng.uniform(low_x, high_x)
                path_y, _path_z = _path_sample_at_x(x)
                side = -1.0 if rng.random() < 0.48 else 1.0
                lateral = rng.uniform(3.2, 18.0 if depth_band != "far" else 25.0)
                patches.append(
                    (
                        f"{kind}.{depth_band}.{local_index:02d}",
                        depth_band,
                        x,
                        path_y + side * lateral,
                    )
                )
        patch_centres[kind] = patches
    for kind, count in counts.items():
        for index in range(count):
            minimum_clearance = {"boulder": 3.0, "fern": 2.8, "grass": 2.6}[kind]
            patch_id: str | None = None
            depth_band: str | None = None
            for _attempt in range(400):
                if kind in patch_centres:
                    patch_id, depth_band, centre_x, centre_y = rng.choice(patch_centres[kind])
                    spread = 1.65 if kind == "fern" else 1.35
                    x = max(-54.0, min(55.0, centre_x + rng.gauss(0.0, spread * 1.25)))
                    y = max(-47.0, min(47.0, centre_y + rng.gauss(0.0, spread)))
                else:
                    x = rng.uniform(-54.0, 55.0)
                    path_y, _path_z = _path_sample_at_x(x)
                    side = -1.0 if rng.random() < 0.47 else 1.0
                    y = max(
                        -47.0,
                        min(47.0, path_y + side * rng.uniform(3.2, 31.0) + rng.gauss(0.0, 1.2)),
                    )
                if _distance_to_centerline((x, y), centerline) >= minimum_clearance:
                    if math.dist((x, y), _HERO_CAMERA_XY) < (4.5 if kind != "boulder" else 7.0):
                        continue
                    if _distance_to_segment((x, y), _HERO_CAMERA_XY, (-1.0, 18.0)) < 1.35:
                        continue
                    if kind == "boulder" and math.dist((x, y), _HERO_ROBOT_XY) < 4.0:
                        continue
                    break
            else:
                raise RuntimeError(f"could not place hero dressing item: {kind}")
            if kind == "boulder":
                scale = rng.uniform(0.42, 0.88)
                target_height = None
            elif kind == "fern":
                target_height = rng.triangular(0.22, 0.59, 0.38)
                scale = target_height / _FERN_TEMPLATE_HEIGHT_M
            else:
                target_height = rng.triangular(0.12, 0.345, 0.21)
                scale = target_height / _GRASS_TEMPLATE_HEIGHT_M
            dressing.append(
                _visual_item(
                    instance_id=f"hero.{kind}.{index:03d}",
                    kind=kind,
                    patch_id=patch_id,
                    depth_band=depth_band,
                    position_m=[round(x, 5), round(y, 5), 0.0],
                    yaw_deg=round(rng.uniform(0.0, 360.0), 4),
                    scale=[round(scale, 4)] * 3,
                    target_height_m=None if target_height is None else round(target_height, 4),
                )
            )
    if include_conditioned_debris:
        for index in range(44):
            for _attempt in range(400):
                x = rng.uniform(-48.0, 34.0)
                path_y, _path_z = _path_sample_at_x(x)
                side = -1.0 if rng.random() < 0.5 else 1.0
                y = path_y + side * rng.uniform(3.0, 18.0) + rng.gauss(0.0, 1.1)
                point = (x, y)
                if _distance_to_centerline(point, centerline) < 2.75:
                    continue
                if math.dist(point, _HERO_CAMERA_XY) < 5.0:
                    continue
                if _distance_to_segment(point, _HERO_CAMERA_XY, (-1.0, 18.0)) < 1.35:
                    continue
                break
            else:
                raise RuntimeError("could not place conditioned forest-floor debris")
            target_height = rng.uniform(0.065, 0.12)
            scale = target_height / _CONDITIONED_UNDERSTORY_HEIGHT_M["forest_floor_debris"]
            dressing.append(
                _visual_item(
                    instance_id=f"hero.forest_floor_debris.{index:03d}",
                    kind="forest_floor_debris",
                    patch_id=f"debris.sparse.{index // 4:02d}",
                    depth_band="near" if x < -13.0 else "mid",
                    position_m=[round(x, 5), round(y, 5), 0.0],
                    yaw_deg=round(rng.uniform(0.0, 360.0), 4),
                    scale=[round(scale, 4)] * 3,
                    target_height_m=round(target_height, 4),
                )
            )
    return dressing


def _validate_asset_slot_identities(
    external_asset_slots: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    required = {"pine", "birch", "boulder"}
    if set(external_asset_slots) != required:
        raise ValueError("external_asset_slots must contain pine, birch, and boulder")
    result: dict[str, dict[str, Any]] = {}
    for semantic_class in sorted(required):
        raw = external_asset_slots[semantic_class]
        if not isinstance(raw, Mapping):
            raise TypeError(f"external_asset_slots.{semantic_class} must be an object")
        path = raw.get("path")
        normalized_path = path.replace("\\", "/") if isinstance(path, str) else ""
        if (
            not isinstance(path, str)
            or not path
            or PurePosixPath(normalized_path).is_absolute()
            or ":" in normalized_path
            or PurePosixPath(normalized_path).suffix.casefold() != ".glb"
        ):
            raise ValueError(f"external_asset_slots.{semantic_class}.path must be relative")
        if any(part in {"", ".", ".."} for part in PurePosixPath(normalized_path).parts):
            raise ValueError(f"external_asset_slots.{semantic_class}.path must be portable")
        declared_bytes = raw.get("bytes")
        if isinstance(declared_bytes, bool) or not isinstance(declared_bytes, int) or declared_bytes < 1:
            raise ValueError(f"external_asset_slots.{semantic_class}.bytes must be positive")
        sha256 = raw.get("sha256")
        if not isinstance(sha256, str) or _SHA256.fullmatch(sha256) is None:
            raise ValueError(f"external_asset_slots.{semantic_class}.sha256 must be lowercase sha256")
        embed_depth = raw.get("embed_depth_m")
        if (
            isinstance(embed_depth, bool)
            or not isinstance(embed_depth, (int, float))
            or not math.isfinite(float(embed_depth))
            or not 0.0 <= float(embed_depth) <= 0.5
        ):
            raise ValueError(f"external_asset_slots.{semantic_class}.embed_depth_m is invalid")
        result[semantic_class] = dict(raw)
    return result


def _validate_understory_asset_slot_identities(
    understory_asset_slots: Mapping[str, Mapping[str, Any]] | None,
) -> dict[str, dict[str, Any]]:
    if understory_asset_slots is None:
        return {}
    if not isinstance(understory_asset_slots, Mapping):
        raise TypeError("understory_asset_slots must be an object")
    if set(understory_asset_slots) != _UNDERSTORY_SLOT_IDS:
        raise ValueError(
            "understory_asset_slots must contain grass_clump, fern_clump, "
            "and forest_floor_debris"
        )
    result: dict[str, dict[str, Any]] = {}
    for semantic_class in sorted(_UNDERSTORY_SLOT_IDS):
        raw = understory_asset_slots[semantic_class]
        if not isinstance(raw, Mapping):
            raise TypeError(f"understory_asset_slots.{semantic_class} must be an object")
        path = raw.get("path")
        normalized_path = path.replace("\\", "/") if isinstance(path, str) else ""
        if (
            not isinstance(path, str)
            or not path
            or PurePosixPath(normalized_path).is_absolute()
            or ":" in normalized_path
            or PurePosixPath(normalized_path).suffix.casefold() != ".glb"
        ):
            raise ValueError(f"understory_asset_slots.{semantic_class}.path must be relative")
        if any(part in {"", ".", ".."} for part in PurePosixPath(normalized_path).parts):
            raise ValueError(f"understory_asset_slots.{semantic_class}.path must be portable")
        declared_bytes = raw.get("bytes")
        if isinstance(declared_bytes, bool) or not isinstance(declared_bytes, int) or declared_bytes < 1:
            raise ValueError(f"understory_asset_slots.{semantic_class}.bytes must be positive")
        sha256 = raw.get("sha256")
        if not isinstance(sha256, str) or _SHA256.fullmatch(sha256) is None:
            raise ValueError(f"understory_asset_slots.{semantic_class}.sha256 must be lowercase sha256")
        result[semantic_class] = {
            "path": PurePosixPath(normalized_path).as_posix(),
            "bytes": declared_bytes,
            "sha256": sha256,
        }
    return result


def select_hero_understory_templates(
    procedural: Mapping[str, Any],
    conditioned: Mapping[str, Any],
) -> dict[str, Any]:
    """Overlay complete conditioned understory templates onto the review fallback set."""

    selected = dict(procedural)
    if not conditioned:
        return selected
    if set(conditioned) != _UNDERSTORY_SLOT_IDS:
        raise ValueError("conditioned understory templates must form one complete slot set")
    for semantic_class, template_key in _UNDERSTORY_TEMPLATE_KEYS.items():
        selected[template_key] = conditioned[semantic_class]
    return selected


def build_hero_diorama_plan(
    external_asset_slots: Mapping[str, Mapping[str, Any]],
    *,
    understory_asset_slots: Mapping[str, Mapping[str, Any]] | None = None,
    seed: int = DEFAULT_SEED,
    include_thunder: bool = False,
) -> dict[str, Any]:
    """Build the deterministic, non-authoritative hero composition contract."""

    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer")
    slots = _validate_asset_slot_identities(external_asset_slots)
    understory_slots = _validate_understory_asset_slot_identities(understory_asset_slots)
    rng = random.Random(seed)  # noqa: S311 - deterministic art layout, not security.
    centerline = _catmull_rom_centerline()
    review_actor = None
    if include_thunder:
        review_actor = _visual_item(
            asset_id="ThunderV4_v1.0.3",
            position_m=[_HERO_ROBOT_XY[0], _HERO_ROBOT_XY[1], 0.0],
            yaw_deg=-145.0,
            grounding_policy="lowest_vertex_to_terrain",
            ground_clearance_m=0.01,
            review_only=True,
        )
    return {
        "schema": "lingtu.sim.forest-hero-diorama-plan.v1",
        "seed": seed,
        "review_only": True,
        "modifies_production_world": False,
        "modifies_physics": False,
        "extent_m": [120.0, 100.0],
        "trail": {
            "centerline_m": centerline,
            "surface_width_m": 3.0,
            "closed_loop": False,
        },
        "external_asset_slots": deepcopy(slots),
        "understory_asset_slots": deepcopy(understory_slots),
        "trees": _tree_instances(rng, centerline),
        "dressing": _dressing_instances(
            rng,
            centerline,
            include_conditioned_debris=bool(understory_slots),
        ),
        "review_actor": review_actor,
        "cameras": [
            {
                "camera_id": "hero_diorama",
                "projection": "perspective",
                "lens_mm": 52.0,
                "location_m": [-32.0, -4.0, 2.1],
                "target_m": [-4.0, 19.0, 3.15],
                "clip_start_m": 0.2,
                "clip_end_m": 400.0,
                "focus_m": [_HERO_ROBOT_XY[0], _HERO_ROBOT_XY[1], 1.45],
                "f_stop": 6.3,
            }
        ],
        "runtime_contract": dict(_RUNTIME_CONTRACT),
        "outputs": deepcopy(_OUTPUTS),
    }


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        + "\n"
    ).encode("utf-8")


def _digest(value: object) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _load_external_assets(
    asset_root: Path,
    slots: Mapping[str, Mapping[str, Any]],
) -> dict[str, author._ExternalAssetRecord]:
    records: dict[str, author._ExternalAssetRecord] = {}
    root = asset_root.resolve(strict=True)
    for semantic_class, identity in slots.items():
        relative = str(identity["path"]).replace("\\", "/")
        source = author._validated_regular_asset_path(
            root,
            relative,
            f"external_asset_slots.{semantic_class}.path",
        )
        payload = source.read_bytes()
        if len(payload) != int(identity["bytes"]):
            raise ValueError(f"external_asset_slots.{semantic_class}.bytes does not match")
        if hashlib.sha256(payload).hexdigest() != identity["sha256"]:
            raise ValueError(f"external_asset_slots.{semantic_class}.sha256 does not match")
        records[semantic_class] = author._ExternalAssetRecord(identity, source)
    return records


def _load_understory_assets(
    asset_root: Path,
    slots: Mapping[str, Mapping[str, Any]],
) -> dict[str, author._ExternalAssetRecord]:
    """Load one complete conditioned understory set behind content identities."""

    normalized = _validate_understory_asset_slot_identities(slots)
    if not normalized:
        return {}
    records: dict[str, author._ExternalAssetRecord] = {}
    root = asset_root.resolve(strict=True)
    for semantic_class, identity in normalized.items():
        relative = str(identity["path"])
        source = author._validated_regular_asset_path(
            root,
            relative,
            f"understory_asset_slots.{semantic_class}.path",
        )
        payload = source.read_bytes()
        if len(payload) != int(identity["bytes"]):
            raise ValueError(f"understory_asset_slots.{semantic_class}.bytes does not match")
        if hashlib.sha256(payload).hexdigest() != identity["sha256"]:
            raise ValueError(f"understory_asset_slots.{semantic_class}.sha256 does not match")
        records[semantic_class] = author._ExternalAssetRecord(identity, source)
    return records


def _import_understory_asset_templates(
    records: Mapping[str, author._ExternalAssetRecord],
    template_collection: Any,
    procedural_templates: Mapping[str, Any],
    *,
    blender_override: Any | None = None,
) -> dict[str, Any]:
    """Import conditioned review understory without exposing it to world export."""

    if not records:
        return select_hero_understory_templates(procedural_templates, {})
    if set(records) != _UNDERSTORY_SLOT_IDS:
        raise ValueError("conditioned understory records must form one complete slot set")
    blender = blender_override if blender_override is not None else bpy
    if blender is None:
        raise RuntimeError("Blender bpy is required to import conditioned understory")
    conditioned: dict[str, Any] = {}
    for semantic_class in sorted(_UNDERSTORY_SLOT_IDS):
        record = records[semantic_class]
        before = {id(obj) for obj in blender.data.objects}
        result = blender.ops.import_scene.gltf(filepath=str(record.source_path))
        if result != {"FINISHED"}:
            raise RuntimeError(f"conditioned understory import failed: {semantic_class}")
        imported = [obj for obj in blender.data.objects if id(obj) not in before]
        if not imported:
            raise RuntimeError(f"conditioned understory import created no objects: {semantic_class}")
        imported_set = set(imported)
        top_level = [obj for obj in imported if obj.parent is None or obj.parent not in imported_set]
        object_name = _UNDERSTORY_TEMPLATE_NAMES[semantic_class]
        if len(top_level) == 1:
            root = top_level[0]
            root.name = object_name
        else:
            root = blender.data.objects.new(object_name, None)
            template_collection.objects.link(root)
            for obj in top_level:
                matrix_world = obj.matrix_world.copy()
                obj.parent = root
                obj.matrix_world = matrix_world
        for obj in imported:
            author._move_to_collection(obj, template_collection)
        for index, obj in enumerate((root, *root.children_recursive)):
            author._set_visual_only(
                obj,
                f"template.{semantic_class}.{index:02d}",
                f"review.template.{semantic_class}",
            )
            obj["review_only"] = True
            obj["asset_source_kind"] = "conditioned_glb"
            obj["asset_source_path"] = str(record["path"])
            obj["asset_source_bytes"] = int(record["bytes"])
            obj["asset_source_sha256"] = str(record["sha256"])
        author._hide_visual_template(root, f"review.template.{semantic_class}")
        conditioned[semantic_class] = root
    for template_key in ("grass", "fern"):
        fallback = procedural_templates.get(template_key)
        if fallback is not None:
            fallback.name = f"Template_Procedural_{template_key.title()}"
            author._hide_visual_template(fallback, f"review.template.procedural.{template_key}")
    return select_hero_understory_templates(procedural_templates, conditioned)


def resolve_review_output_dir(repo_root: Path | str, output_dir: Path | str) -> Path:
    """Resolve one new link-free output below ``build/forest-hf/review``."""

    root = Path(repo_root).resolve(strict=True)
    requested = Path(output_dir)
    if ".." in requested.parts:
        raise ValueError("review output must not contain .. traversal")
    candidate = requested if requested.is_absolute() else root / requested
    candidate = candidate.absolute()
    review_root = root / "build" / "forest-hf" / "review"
    try:
        relative = candidate.relative_to(review_root)
    except ValueError as exc:
        raise ValueError("review output must remain contained in build/forest-hf/review") from exc
    if not relative.parts:
        raise ValueError("review output must name a run directory below the review root")

    current = root
    for part in candidate.relative_to(root).parts:
        current = current / part
        if current.exists() or current.is_symlink():
            if current.is_symlink() or author._is_reparse_point(current):
                raise ValueError("review output must not contain a link or reparse point")
    if candidate.exists():
        raise FileExistsError("review output must be a new non-existing directory")
    return candidate


def _create_terrain(blender: Any, collection: Any, material: Any) -> Any:
    steps_x, steps_y = 120, 100
    vertices = []
    faces = []
    for iy in range(steps_y + 1):
        y = -50.0 + 100.0 * iy / steps_y
        for ix in range(steps_x + 1):
            x = -60.0 + 120.0 * ix / steps_x
            vertices.append((x, y, _terrain_height(x, y)))
    stride = steps_x + 1
    for iy in range(steps_y):
        for ix in range(steps_x):
            first = iy * stride + ix
            faces.append((first, first + 1, first + stride + 1, first + stride))
    mesh = blender.data.meshes.new("LT_HeroDiorama_Terrain_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    obj = blender.data.objects.new("LT_HeroDiorama_Terrain", mesh)
    collection.objects.link(obj)
    obj.data.materials.append(material)
    author._assign_planar_uv(mesh, 60.0, 50.0, tile_m=9.0)
    author._set_visual_only(obj, "hero.terrain", "review.terrain")
    obj["review_only"] = True
    return obj


def _build_hero_ground_material(blender: Any) -> Any:
    """Build a dedicated three-scale wet soil, humus, and moss material."""

    contract = hero_ground_surface_contract()
    noise_frequencies = hero_ground_noise_frequencies(contract)
    material = blender.data.materials.new("M_HeroDiorama_MultiscaleForestGround")
    material.diffuse_color = _rgba(contract["surface_classes"]["moist_soil"]["color"])
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (980.0, 40.0)
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    shader.location = (720.0, 40.0)
    shader.inputs["Roughness"].default_value = float(contract["roughness"])
    shader.inputs["Specular IOR Level"].default_value = 0.12
    coordinates = nodes.new("ShaderNodeTexCoord")
    coordinates.location = (-980.0, 20.0)

    noises: dict[str, Any] = {}
    node_y = {"moss_moisture": 300.0, "humus_breakup": 20.0, "micro_grain": -280.0}
    for layer in contract["detail_layers"]:
        layer_id = str(layer["layer_id"])
        noise = nodes.new("ShaderNodeTexNoise")
        noise.name = f"HeroGround_{layer_id}"
        noise.label = (
            f"{layer_id}: {layer['scale_m']}m / "
            f"{noise_frequencies[layer_id]:.6f}m^-1"
        )
        noise.location = (-720.0, node_y[layer_id])
        noise.inputs["Scale"].default_value = noise_frequencies[layer_id]
        noise.inputs["Detail"].default_value = 3.5 if layer_id != "micro_grain" else 2.0
        noise.inputs["Roughness"].default_value = 0.68
        links.new(coordinates.outputs[contract["noise_coordinate_input"]], noise.inputs["Vector"])
        noises[layer_id] = noise

    soil_ramp = nodes.new("ShaderNodeValToRGB")
    soil_ramp.name = "HeroGround_HumusSoilRamp"
    soil_ramp.location = (-390.0, 60.0)
    soil_ramp.color_ramp.elements[0].color = _rgba(
        contract["surface_classes"]["forest_humus"]["color"]
    )
    soil_ramp.color_ramp.elements[1].color = _rgba(
        contract["surface_classes"]["moist_soil"]["color"]
    )
    links.new(noises["humus_breakup"].outputs["Fac"], soil_ramp.inputs["Fac"])

    moss_mask = nodes.new("ShaderNodeValToRGB")
    moss_mask.name = "HeroGround_MossPatchMask"
    moss_mask.location = (-390.0, 330.0)
    moss_mask_contract = contract["moss_mask"]
    moss_mask.color_ramp.elements[0].position = float(moss_mask_contract["low_threshold"])
    moss_mask.color_ramp.elements[0].color = (0.0, 0.0, 0.0, 1.0)
    moss_mask.color_ramp.elements[1].position = float(moss_mask_contract["high_threshold"])
    moss_mask.color_ramp.elements[1].color = (1.0, 1.0, 1.0, 1.0)
    links.new(noises["moss_moisture"].outputs["Fac"], moss_mask.inputs["Fac"])

    colour_mix = nodes.new("ShaderNodeMixRGB")
    colour_mix.name = "HeroGround_SoilMossMix"
    colour_mix.blend_type = "MIX"
    colour_mix.location = (40.0, 110.0)
    colour_mix.inputs[2].default_value = _rgba(contract["surface_classes"]["moss"]["color"])
    links.new(moss_mask.outputs["Color"], colour_mix.inputs[0])
    links.new(soil_ramp.outputs["Color"], colour_mix.inputs[1])
    links.new(colour_mix.outputs["Color"], shader.inputs["Base Color"])

    roughness_ramp = nodes.new("ShaderNodeValToRGB")
    roughness_ramp.name = "HeroGround_RoughnessRange"
    roughness_ramp.location = (50.0, -180.0)
    roughness_ramp.color_ramp.elements[0].color = (0.78, 0.78, 0.78, 1.0)
    roughness_ramp.color_ramp.elements[1].color = (0.94, 0.94, 0.94, 1.0)
    links.new(noises["micro_grain"].outputs["Fac"], roughness_ramp.inputs["Fac"])
    links.new(roughness_ramp.outputs["Color"], shader.inputs["Roughness"])

    bump = nodes.new("ShaderNodeBump")
    bump.name = "HeroGround_CentimetreBump"
    bump.location = (450.0, -180.0)
    bump.inputs["Strength"].default_value = float(contract["normal_strength"])
    bump.inputs["Distance"].default_value = 0.035
    links.new(noises["micro_grain"].outputs["Fac"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], shader.inputs["Normal"])
    links.new(shader.outputs["BSDF"], output.inputs["Surface"])
    return material


def _build_hero_trail_material(blender: Any) -> Any:
    """Build the dedicated object-local, three-scale wet trail material."""

    contract = hero_trail_surface_contract()
    material = blender.data.materials.new("M_HeroDiorama_MultiscaleWetTrail")
    material.diffuse_color = _rgba(contract["surface_colors"]["warm_wet_soil"])
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (1050.0, 30.0)
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    shader.location = (790.0, 30.0)
    shader.inputs["Specular IOR Level"].default_value = 0.11
    coordinates = nodes.new("ShaderNodeTexCoord")
    coordinates.location = (-1040.0, 20.0)

    noises: dict[str, Any] = {}
    layer_y = {"broad_dampness": 310.0, "mud_breakup": 20.0, "micro_grit": -280.0}
    for layer in contract["detail_layers"]:
        layer_id = str(layer["layer_id"])
        noise = nodes.new("ShaderNodeTexNoise")
        noise.name = f"HeroTrail_{layer_id}"
        noise.label = f"{layer_id}: {float(layer['scale_m']):.3f}m"
        noise.location = (-780.0, layer_y[layer_id])
        noise.inputs["Scale"].default_value = 1.0 / float(layer["scale_m"])
        noise.inputs["Detail"].default_value = 4.0 if layer_id != "micro_grit" else 2.0
        noise.inputs["Roughness"].default_value = 0.64
        links.new(coordinates.outputs[contract["noise_coordinate_input"]], noise.inputs["Vector"])
        noises[layer_id] = noise

    soil_ramp = nodes.new("ShaderNodeValToRGB")
    soil_ramp.name = "HeroTrail_WarmSoilRamp"
    soil_ramp.location = (-430.0, 30.0)
    soil_ramp.color_ramp.elements[0].color = _rgba(
        contract["surface_colors"]["packed_humus"]
    )
    soil_ramp.color_ramp.elements[1].color = _rgba(
        contract["surface_colors"]["warm_wet_soil"]
    )
    links.new(noises["mud_breakup"].outputs["Fac"], soil_ramp.inputs["Fac"])

    damp_mix = nodes.new("ShaderNodeMixRGB")
    damp_mix.name = "HeroTrail_BroadDampnessMix"
    damp_mix.location = (0.0, 100.0)
    damp_mix.inputs[2].default_value = _rgba(contract["surface_colors"]["deep_damp_patch"])
    links.new(noises["broad_dampness"].outputs["Fac"], damp_mix.inputs[0])
    links.new(soil_ramp.outputs["Color"], damp_mix.inputs[1])
    links.new(damp_mix.outputs["Color"], shader.inputs["Base Color"])

    roughness_ramp = nodes.new("ShaderNodeValToRGB")
    roughness_ramp.name = "HeroTrail_RoughnessRange"
    roughness_ramp.location = (80.0, -180.0)
    roughness_low, roughness_high = contract["roughness_range"]
    roughness_ramp.color_ramp.elements[0].color = (roughness_low,) * 3 + (1.0,)
    roughness_ramp.color_ramp.elements[1].color = (roughness_high,) * 3 + (1.0,)
    links.new(noises["micro_grit"].outputs["Fac"], roughness_ramp.inputs["Fac"])
    links.new(roughness_ramp.outputs["Color"], shader.inputs["Roughness"])

    bump = nodes.new("ShaderNodeBump")
    bump.name = "HeroTrail_CentimetreBump"
    bump.location = (500.0, -170.0)
    bump.inputs["Strength"].default_value = float(contract["normal_strength"])
    bump.inputs["Distance"].default_value = 0.028
    links.new(noises["micro_grit"].outputs["Fac"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], shader.inputs["Normal"])
    links.new(shader.outputs["BSDF"], output.inputs["Surface"])
    return material


def trail_mesh_data(plan: Mapping[str, Any]) -> dict[str, Any]:
    """Build the review trail mesh with an explicit upward-facing winding."""

    points = plan["trail"]["centerline_m"]
    base_half_width = float(plan["trail"]["surface_width_m"]) * 0.5
    left: list[tuple[float, float, float]] = []
    right: list[tuple[float, float, float]] = []
    for index, point in enumerate(points):
        previous = points[max(0, index - 1)]
        following = points[min(len(points) - 1, index + 1)]
        dx, dy = following[0] - previous[0], following[1] - previous[1]
        length = max(math.hypot(dx, dy), 1e-9)
        nx, ny = -dy / length, dx / length
        width_variation = 0.94 + 0.065 * math.sin(index * 0.61) + 0.035 * math.sin(index * 1.37)
        half_width = base_half_width * width_variation
        for target, sign in ((left, 1.0), (right, -1.0)):
            x = point[0] + nx * half_width * sign
            y = point[1] + ny * half_width * sign
            target.append((x, y, _terrain_height(x, y) + 0.004))
    vertices = [value for pair in zip(left, right) for value in pair]
    faces = [
        (2 * index, 2 * index + 1, 2 * index + 3, 2 * index + 2)
        for index in range(len(points) - 1)
    ]
    return {"vertices": vertices, "faces": faces}


def trail_shoulder_mesh_data(plan: Mapping[str, Any]) -> dict[str, Any]:
    """Build two continuous shoulder strips fading from trail to forest floor."""

    points = plan["trail"]["centerline_m"]
    base_half_width = float(plan["trail"]["surface_width_m"]) * 0.5
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int, int]] = []
    blend_weights: list[float] = []
    side_widths: list[float] = []
    for side_index, sign in enumerate((1.0, -1.0)):
        side_offset = len(vertices)
        for point_index, point in enumerate(points):
            previous = points[max(0, point_index - 1)]
            following = points[min(len(points) - 1, point_index + 1)]
            dx, dy = following[0] - previous[0], following[1] - previous[1]
            length = max(math.hypot(dx, dy), 1e-9)
            nx, ny = -dy / length, dx / length
            trail_variation = 0.94 + 0.065 * math.sin(point_index * 0.61) + 0.035 * math.sin(
                point_index * 1.37
            )
            half_width = base_half_width * trail_variation
            shoulder_width = 1.0 + 0.22 * math.sin(
                point_index * 0.47 + side_index * 1.7
            ) + 0.08 * math.sin(
                point_index * 1.19 + side_index * 0.6
            )
            shoulder_width = max(0.7, min(1.3, shoulder_width))
            side_widths.append(round(shoulder_width, 6))
            inner_x = point[0] + nx * half_width * sign
            inner_y = point[1] + ny * half_width * sign
            outer_x = point[0] + nx * (half_width + shoulder_width) * sign
            outer_y = point[1] + ny * (half_width + shoulder_width) * sign
            vertices.extend(
                (
                    (outer_x, outer_y, _terrain_height(outer_x, outer_y) + 0.003),
                    (inner_x, inner_y, _terrain_height(inner_x, inner_y) + 0.0035),
                )
            )
            blend_weights.extend((0.0, 1.0))
        for index in range(len(points) - 1):
            current_outer = side_offset + 2 * index
            current_inner = current_outer + 1
            next_outer = current_outer + 2
            next_inner = current_outer + 3
            if sign > 0.0:
                face = (current_outer, current_inner, next_inner, next_outer)
            else:
                face = (next_outer, next_inner, current_inner, current_outer)
            faces.append(face)
    return {
        "vertices": vertices,
        "faces": faces,
        "blend_weights": blend_weights,
        "side_widths_m": side_widths,
        "visual_height_m": 0.0035,
    }


def _create_trail(blender: Any, collection: Any, plan: Mapping[str, Any], material: Any) -> Any:
    mesh_data = trail_mesh_data(plan)
    mesh = blender.data.meshes.new("LT_HeroDiorama_Trail_Mesh")
    mesh.from_pydata(mesh_data["vertices"], [], mesh_data["faces"])
    mesh.update()
    obj = blender.data.objects.new("LT_HeroDiorama_Trail", mesh)
    collection.objects.link(obj)
    obj.data.materials.append(material)
    author._assign_planar_uv(mesh, 60.0, 50.0, tile_m=4.0)
    author._set_visual_only(obj, "hero.trail", "review.trail")
    obj["review_only"] = True
    return obj


def _create_trail_shoulders(blender: Any, collection: Any, plan: Mapping[str, Any]) -> Any:
    """Create one combined shoulder mesh that feathers mud into forest floor."""

    surface_contract = trail_shoulder_surface_contract()
    mesh_data = trail_shoulder_mesh_data(plan)
    mesh = blender.data.meshes.new("LT_HeroDiorama_TrailShoulders_Mesh")
    mesh.from_pydata(mesh_data["vertices"], [], mesh_data["faces"])
    mesh.update()
    blend = mesh.attributes.new(
        name=surface_contract["blend_attribute"],
        type="FLOAT",
        domain="POINT",
    )
    for value, weight in zip(blend.data, mesh_data["blend_weights"]):
        value.value = float(weight)

    material = blender.data.materials.new("M_HeroDiorama_TrailShoulderBlend")
    material.diffuse_color = _rgba("#21170F")
    material.use_nodes = True
    applied_render_method: str | None = None
    if hasattr(material, "surface_render_method"):
        for candidate in surface_contract["render_method_preference"]:
            try:
                material.surface_render_method = candidate
            except (TypeError, ValueError):
                continue
            applied_render_method = candidate
            break
    elif hasattr(material, "blend_method"):
        material.blend_method = "BLEND"
        applied_render_method = "BLEND"
    if applied_render_method is None:
        raise RuntimeError("review shoulder material cannot enable alpha rendering")
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (580.0, 20.0)
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    shader.location = (330.0, 20.0)
    shader.inputs["Roughness"].default_value = 0.86
    shader.inputs["Specular IOR Level"].default_value = 0.09
    shader.inputs["Alpha"].default_value = float(surface_contract["inner_alpha"])
    attribute = nodes.new("ShaderNodeAttribute")
    attribute.attribute_name = surface_contract["blend_attribute"]
    attribute.location = (-520.0, 120.0)
    coordinates = nodes.new("ShaderNodeTexCoord")
    coordinates.location = (-780.0, -150.0)
    noise = nodes.new("ShaderNodeTexNoise")
    noise.name = "HeroTrailShoulder_Breakup"
    noise.label = (
        "shoulder breakup: "
        f"{float(surface_contract['noise_scale_per_m']):.6f}m^-1"
    )
    noise.location = (-520.0, -150.0)
    noise.inputs["Scale"].default_value = float(surface_contract["noise_scale_per_m"])
    noise.inputs["Detail"].default_value = 3.0
    mix = nodes.new("ShaderNodeMixRGB")
    mix.location = (-80.0, 30.0)
    mix.inputs[1].default_value = _rgba("#28301F")
    mix.inputs[2].default_value = _rgba("#2D1B10")
    links.new(attribute.outputs["Fac"], mix.inputs[0])
    links.new(mix.outputs["Color"], shader.inputs["Base Color"])
    links.new(attribute.outputs["Fac"], shader.inputs["Alpha"])
    links.new(
        coordinates.outputs[surface_contract["noise_coordinate_input"]],
        noise.inputs["Vector"],
    )
    bump = nodes.new("ShaderNodeBump")
    bump.location = (80.0, -160.0)
    bump.inputs["Strength"].default_value = 0.10
    bump.inputs["Distance"].default_value = 0.018
    links.new(noise.outputs["Fac"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], shader.inputs["Normal"])
    links.new(shader.outputs["BSDF"], output.inputs["Surface"])

    obj = blender.data.objects.new("LT_HeroDiorama_TrailShoulders", mesh)
    collection.objects.link(obj)
    obj.data.materials.append(material)
    author._set_visual_only(obj, "hero.trail.shoulders", "review.trail.shoulders")
    obj["review_only"] = True
    obj["blend_attribute"] = surface_contract["blend_attribute"]
    obj["alpha_render_method"] = applied_render_method
    obj["visual_height_m"] = float(mesh_data["visual_height_m"])
    return obj


def _create_trail_details(blender: Any, collection: Any, plan: Mapping[str, Any]) -> Any:
    """Add sparse wet ruts and puddled mud without changing the route geometry."""

    rng = random.Random(int(plan["seed"]) ^ 0x7A411)  # noqa: S311 - deterministic art.
    points = plan["trail"]["centerline_m"]
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    material_indices: list[int] = []
    for index in range(8, len(points) - 7, 3):
        point = points[index]
        previous = points[index - 1]
        following = points[index + 1]
        dx, dy = following[0] - previous[0], following[1] - previous[1]
        length = max(math.hypot(dx, dy), 1e-9)
        tx, ty = dx / length, dy / length
        nx, ny = -ty, tx
        for lane_index, lane in enumerate((-0.48, 0.48)):
            if rng.random() < 0.38:
                continue
            center_x = point[0] + nx * (lane + rng.uniform(-0.12, 0.12))
            center_y = point[1] + ny * (lane + rng.uniform(-0.12, 0.12))
            half_length = rng.uniform(0.28, 0.72)
            half_width = rng.uniform(0.055, 0.14)
            first = len(vertices)
            vertex_count = 10
            for vertex_index in range(vertex_count):
                angle = math.tau * vertex_index / vertex_count
                radial_jitter = rng.uniform(0.82, 1.12)
                longitudinal = math.cos(angle) * half_length * radial_jitter
                lateral = math.sin(angle) * half_width * radial_jitter
                x = center_x + tx * longitudinal + nx * lateral
                y = center_y + ty * longitudinal + ny * lateral
                vertices.append((x, y, _terrain_height(x, y) + 0.0048))
            faces.append(tuple(range(first, first + vertex_count)))
            material_indices.append((index + lane_index) % 2)

    mesh = blender.data.meshes.new("LT_HeroDiorama_TrailDetails_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = blender.data.objects.new("LT_HeroDiorama_TrailDetails", mesh)
    collection.objects.link(obj)
    for name, color, roughness in (
        ("M_HeroTrail_WetRuts", (0.024, 0.014, 0.008, 1.0), 0.46),
        ("M_HeroTrail_DampMud", (0.070, 0.034, 0.014, 1.0), 0.66),
    ):
        material = blender.data.materials.new(name)
        material.diffuse_color = color
        material.use_nodes = True
        shader = material.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = color
        shader.inputs["Roughness"].default_value = roughness
        shader.inputs["Specular IOR Level"].default_value = 0.10
        obj.data.materials.append(material)
    for polygon, material_index in zip(mesh.polygons, material_indices):
        polygon.material_index = material_index
    author._set_visual_only(obj, "hero.trail.details", "review.trail.details")
    obj["review_only"] = True
    return obj


def _rgba(hex_color: str) -> tuple[float, float, float, float]:
    value = hex_color.removeprefix("#")
    red = int(value[0:2], 16) / 255.0
    green = int(value[2:4], 16) / 255.0
    blue = int(value[4:6], 16) / 255.0
    return red, green, blue, 1.0


def _tune_review_material(
    material: Any,
    dark: str,
    light: str,
    *,
    roughness: float,
    mapping_scale: tuple[float, float, float],
) -> None:
    """Replace generic preview colors with restrained wet-forest values."""

    found = {"ramp": False, "shader": False, "mapping": False}
    for node in material.node_tree.nodes:
        if getattr(node, "bl_idname", "") == "ShaderNodeValToRGB":
            node.color_ramp.elements[0].color = _rgba(dark)
            node.color_ramp.elements[1].color = _rgba(light)
            found["ramp"] = True
        if getattr(node, "bl_idname", "") == "ShaderNodeBsdfPrincipled":
            node.inputs["Roughness"].default_value = roughness
            found["shader"] = True
        if getattr(node, "bl_idname", "") == "ShaderNodeMapping":
            node.inputs["Scale"].default_value = mapping_scale
            found["mapping"] = True
    if not all(found.values()):
        missing = sorted(name for name, present in found.items() if not present)
        raise RuntimeError(f"review material is missing required nodes: {missing}")


def _instantiate_plan(
    plan: Mapping[str, Any],
    templates: Mapping[str, Any],
    collections: Mapping[str, Any],
) -> None:
    conditioned_understory = bool(plan.get("understory_asset_slots"))

    def mark_review_only(root: Any) -> None:
        for obj in (root, *root.children_recursive):
            obj["review_only"] = True

    for item in plan["trees"]:
        obj = author._link_visual_hierarchy(
            templates[item["species_id"]],
            item["instance_id"],
            collections["Trees"],
            item["instance_id"],
            f"review.tree.{item['species_id']}",
        )
        x, y, _ = item["position_m"]
        embed = float(plan["external_asset_slots"][item["species_id"]]["embed_depth_m"])
        obj.location = (x, y, _terrain_height(x, y) - embed)
        obj.rotation_euler[0] = math.radians(float(item["tilt_x_deg"]))
        obj.rotation_euler[1] = math.radians(float(item["tilt_y_deg"]))
        obj.rotation_euler[2] = math.radians(float(item["yaw_deg"]))
        obj.scale = tuple(item["scale"])
        mark_review_only(obj)
    for item in plan["dressing"]:
        kind = item["kind"]
        obj = author._link_visual_hierarchy(
            templates[kind],
            item["instance_id"],
            collections["Dressing"],
            item["instance_id"],
            f"review.dressing.{kind}",
        )
        x, y, _ = item["position_m"]
        embed = (
            float(plan["external_asset_slots"]["boulder"]["embed_depth_m"])
            if kind == "boulder"
            else 0.015
        )
        obj.location = (x, y, _terrain_height(x, y) - embed)
        obj.rotation_euler[2] = math.radians(float(item["yaw_deg"]))
        if conditioned_understory and kind in _CONDITIONED_UNDERSTORY_HEIGHT_M:
            target_height = float(item["target_height_m"])
            scale = target_height / _CONDITIONED_UNDERSTORY_HEIGHT_M[kind]
            obj.scale = (scale, scale, scale)
        else:
            obj.scale = tuple(item["scale"])
        mark_review_only(obj)


def _create_leaf_litter(blender: Any, collection: Any, plan: Mapping[str, Any]) -> Any:
    """Create one combined mesh of wet leaves, fallen twigs, and tree-base moss."""

    detail_counts = hero_ground_detail_counts()
    rng = random.Random(int(plan["seed"]) ^ 0x51A77E)  # noqa: S311 - deterministic art.
    centerline = plan["trail"]["centerline_m"]
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    material_indices: list[int] = []
    clusters = [
        (rng.uniform(-54.0, 54.0), rng.uniform(-44.0, 44.0))
        for _ in range(260)
    ]
    for leaf_index in range(detail_counts["wet_leaves"]):
        for _attempt in range(100):
            centre_x, centre_y = clusters[leaf_index % len(clusters)]
            x = max(-57.0, min(57.0, centre_x + rng.gauss(0.0, 1.25)))
            y = max(-47.0, min(47.0, centre_y + rng.gauss(0.0, 1.05)))
            if _distance_to_centerline((x, y), centerline) >= 1.85:
                break
        length = rng.uniform(0.08, 0.22)
        width = rng.uniform(0.025, 0.075)
        angle = rng.uniform(0.0, math.tau)
        tangent = (math.cos(angle) * length * 0.5, math.sin(angle) * length * 0.5)
        normal = (-math.sin(angle) * width * 0.5, math.cos(angle) * width * 0.5)
        z = _terrain_height(x, y) + rng.uniform(0.0045, 0.0060)
        first = len(vertices)
        vertices.extend(
            (
                (x - tangent[0] - normal[0], y - tangent[1] - normal[1], z),
                (x + tangent[0] - normal[0], y + tangent[1] - normal[1], z),
                (x + tangent[0] + normal[0], y + tangent[1] + normal[1], z),
                (x - tangent[0] + normal[0], y - tangent[1] + normal[1], z),
            )
        )
        faces.append((first, first + 1, first + 2, first + 3))
        material_indices.append(rng.randrange(3))

    for _twig_index in range(detail_counts["fallen_twigs"]):
        for _attempt in range(100):
            x = rng.triangular(-48.0, 38.0, -12.0)
            y = rng.uniform(-38.0, 38.0)
            if _distance_to_centerline((x, y), centerline) >= 2.05:
                break
        length = rng.uniform(0.18, 0.62)
        width = rng.uniform(0.012, 0.026)
        angle = rng.uniform(0.0, math.tau)
        tangent = (math.cos(angle) * length * 0.5, math.sin(angle) * length * 0.5)
        normal = (-math.sin(angle) * width * 0.5, math.cos(angle) * width * 0.5)
        z = _terrain_height(x, y) + 0.013
        first = len(vertices)
        vertices.extend(
            (
                (x - tangent[0] - normal[0], y - tangent[1] - normal[1], z),
                (x + tangent[0] - normal[0], y + tangent[1] - normal[1], z),
                (x + tangent[0] + normal[0], y + tangent[1] + normal[1], z),
                (x - tangent[0] + normal[0], y - tangent[1] + normal[1], z),
            )
        )
        faces.append((first, first + 1, first + 2, first + 3))
        material_indices.append(3)

    moss_trees = list(plan["trees"])
    rng.shuffle(moss_trees)
    applied_moss_count = min(detail_counts["tree_base_moss"], len(moss_trees))
    for tree in moss_trees[:applied_moss_count]:
        tree_x, tree_y, _tree_z = tree["position_m"]
        angle = rng.uniform(0.0, math.tau)
        radius = rng.uniform(0.20, 0.52)
        center_x = tree_x + math.cos(angle) * rng.uniform(0.10, 0.42)
        center_y = tree_y + math.sin(angle) * rng.uniform(0.10, 0.42)
        first = len(vertices)
        vertex_count = 10
        for vertex_index in range(vertex_count):
            local_angle = math.tau * vertex_index / vertex_count
            local_radius = radius * rng.uniform(0.68, 1.12)
            x = center_x + math.cos(local_angle) * local_radius
            y = center_y + math.sin(local_angle) * local_radius * rng.uniform(0.62, 0.92)
            vertices.append((x, y, _terrain_height(x, y) + 0.009))
        faces.append(tuple(range(first, first + vertex_count)))
        material_indices.append(4)

    mesh = blender.data.meshes.new("LT_HeroDiorama_LeafLitter_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = blender.data.objects.new("LT_HeroDiorama_LeafLitter", mesh)
    collection.objects.link(obj)
    for name, color, roughness in (
        ("M_HeroLeafLitter_Dark", (0.11, 0.055, 0.025, 1.0), 0.88),
        ("M_HeroLeafLitter_Wet", (0.19, 0.095, 0.035, 1.0), 0.76),
        ("M_HeroLeafLitter_Ochre", (0.29, 0.16, 0.055, 1.0), 0.84),
        ("M_HeroLeafLitter_Twigs", (0.075, 0.041, 0.021, 1.0), 0.91),
        ("M_HeroLeafLitter_Moss", (0.105, 0.155, 0.070, 1.0), 0.93),
    ):
        material = blender.data.materials.new(name)
        material.diffuse_color = color
        material.use_nodes = True
        shader = material.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = color
        shader.inputs["Roughness"].default_value = roughness
        obj.data.materials.append(material)
    for polygon, material_index in zip(mesh.polygons, material_indices):
        polygon.material_index = material_index
    author._set_visual_only(obj, "hero.leaf_litter", "review.dressing.leaf_litter")
    obj["review_only"] = True
    obj["combined_detail_counts"] = json.dumps(
        {
            **detail_counts,
            "tree_base_moss": applied_moss_count,
        },
        sort_keys=True,
    )
    return obj


def _create_camera(blender: Any, collection: Any, spec: Mapping[str, Any]) -> Any:
    from mathutils import Vector  # type: ignore[import-not-found]

    data = blender.data.cameras.new("Camera_Forest_HeroDiorama")
    data.lens = float(spec["lens_mm"])
    data.sensor_width = 36.0
    data.clip_start = float(spec["clip_start_m"])
    data.clip_end = float(spec["clip_end_m"])
    camera = blender.data.objects.new("Camera_Forest_HeroDiorama", data)
    camera.location = tuple(spec["location_m"])
    direction = Vector(spec["target_m"]) - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    collection.objects.link(camera)
    author._set_visual_only(camera, "hero.camera", "review.camera")
    camera["review_only"] = True
    focus_point = spec.get("focus_m")
    if isinstance(focus_point, Sequence) and len(focus_point) == 3:
        focus = blender.data.objects.new("LT_HeroDiorama_CameraFocus", None)
        focus.location = tuple(float(value) for value in focus_point)
        collection.objects.link(focus)
        author._set_visual_only(focus, "hero.camera.focus", "review.camera.focus")
        focus["review_only"] = True
        data.dof.use_dof = True
        data.dof.focus_object = focus
        data.dof.aperture_fstop = float(spec.get("f_stop", 6.3))
    return camera


def _relocate_review_robot(robot_root: Any, actor: Mapping[str, Any]) -> None:
    from mathutils import Vector  # type: ignore[import-not-found]

    x, y, _ = actor["position_m"]
    robot_root.location = (x, y, _terrain_height(x, y))
    robot_root.rotation_euler[2] = math.radians(float(actor["yaw_deg"]))
    bpy.context.view_layer.update()
    meshes = [obj for obj in robot_root.children_recursive if getattr(obj, "type", None) == "MESH"]
    lowest = min((obj.matrix_world @ Vector(corner)).z for obj in meshes for corner in obj.bound_box)
    robot_root.location.z += _terrain_height(x, y) + float(actor["ground_clearance_m"]) - lowest
    robot_root["review_only"] = True
    bpy.context.view_layer.update()


def _make_review_robot_readable(robot_root: Any) -> None:
    """Apply restrained review-only materials so the robot does not collapse to black."""

    palettes = {
        "body": (0.14, 0.18, 0.19, 1.0),
        "limb": (0.075, 0.095, 0.105, 1.0),
        "sensor": (0.26, 0.34, 0.37, 1.0),
    }
    materials: dict[str, Any] = {}
    for role, color in palettes.items():
        material = bpy.data.materials.new(f"M_HeroDiorama_Thunder_{role.title()}")
        material.diffuse_color = color
        material.use_nodes = True
        shader = material.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = color
        shader.inputs["Metallic"].default_value = 0.18 if role == "body" else 0.08
        shader.inputs["Roughness"].default_value = 0.54 if role == "sensor" else 0.62
        materials[role] = material

    for obj in robot_root.children_recursive:
        if getattr(obj, "type", None) != "MESH":
            continue
        name = obj.name.casefold()
        if any(token in name for token in ("camera", "lidar", "imu", "sensor")):
            role = "sensor"
        elif any(token in name for token in ("hip", "thigh", "calf", "foot", "leg")):
            role = "limb"
        else:
            role = "body"
        obj.data.materials.clear()
        obj.data.materials.append(materials[role])
        obj["review_material_role"] = role


def _create_robot_contact_shadows(blender: Any, collection: Any, robot_root: Any) -> Any:
    """Place four small review-only ground marks below the imported foot meshes."""

    from mathutils import Vector  # type: ignore[import-not-found]

    foot_corners: dict[str, list[Any]] = {foot: [] for foot in ("fl", "fr", "rl", "rr")}
    for obj in robot_root.children_recursive:
        if getattr(obj, "type", None) != "MESH":
            continue
        name = obj.name.casefold()
        matching_feet = [foot for foot in foot_corners if f"{foot}_foot" in name]
        if len(matching_feet) > 1:
            raise RuntimeError(f"review robot mesh matches multiple feet: {obj.name}")
        if matching_feet:
            foot_corners[matching_feet[0]].extend(
                obj.matrix_world @ Vector(corner) for corner in obj.bound_box
            )
    missing_feet = [foot.upper() for foot, corners in foot_corners.items() if not corners]
    if missing_feet:
        raise RuntimeError(f"review robot is missing unique foot meshes: {missing_feet}")
    foot_centres = [
        (
            sum(corner.x for corner in corners) / len(corners),
            sum(corner.y for corner in corners) / len(corners),
        )
        for corners in foot_corners.values()
    ]

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    vertex_count = 18
    for center_x, center_y in foot_centres:
        first = len(vertices)
        for index in range(vertex_count):
            angle = math.tau * index / vertex_count
            x = center_x + math.cos(angle) * 0.14
            y = center_y + math.sin(angle) * 0.08
            vertices.append((x, y, _terrain_height(x, y) + 0.026))
        faces.append(tuple(range(first, first + vertex_count)))

    mesh = blender.data.meshes.new("LT_HeroDiorama_RobotContact_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = blender.data.objects.new("LT_HeroDiorama_RobotContact", mesh)
    collection.objects.link(obj)
    material = blender.data.materials.new("M_HeroDiorama_RobotContact")
    material.diffuse_color = (0.038, 0.026, 0.017, 1.0)
    material.use_nodes = True
    shader = material.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = material.diffuse_color
    shader.inputs["Roughness"].default_value = 0.92
    shader.inputs["Specular IOR Level"].default_value = 0.04
    obj.data.materials.append(material)
    author._set_visual_only(obj, "hero.robot.contact", "review.robot.contact")
    obj["review_only"] = True
    return obj


def _add_robot_rim_light(collection: Any, actor: Mapping[str, Any]) -> Any:
    """Add a restrained warm edge light so the dark robot reads in foliage."""

    from mathutils import Vector  # type: ignore[import-not-found]

    x, y, _ = actor["position_m"]
    data = bpy.data.lights.new("LT_HeroDiorama_RobotRim", "AREA")
    data.energy = 420.0
    data.color = (1.0, 0.55, 0.28)
    data.shape = "DISK"
    data.size = 2.8
    light = bpy.data.objects.new("LT_HeroDiorama_RobotRim", data)
    light.location = (_HERO_CAMERA_XY[0] + 4.0, _HERO_CAMERA_XY[1] + 2.0, _terrain_height(x, y) + 4.2)
    target = Vector((x, y, _terrain_height(x, y) + 0.55))
    light.rotation_euler = (target - light.location).to_track_quat("-Z", "Y").to_euler()
    collection.objects.link(light)
    author._set_visual_only(light, "hero.lighting.robot_rim", "review.lighting.robot_rim")
    light["review_only"] = True
    return light


def build_hero_diorama(
    plan: Mapping[str, Any],
    asset_root: Path,
    output_dir: Path,
    *,
    repo_root: Path,
    width: int = 1280,
    height: int = 720,
    samples: int = 48,
    review_robot_asset_index: Path | None = None,
    understory_asset_root: Path | None = None,
) -> dict[str, Any]:
    """Render the plan to exactly one PNG and one review manifest."""

    output_dir = resolve_review_output_dir(repo_root, output_dir)
    if bpy is None:
        raise RuntimeError("Blender bpy is required; run with blender --background --python")
    records = _load_external_assets(asset_root, plan["external_asset_slots"])
    understory_slots = plan.get("understory_asset_slots", {})
    if understory_slots and understory_asset_root is None:
        raise ValueError("conditioned understory plan requires --understory-asset-root")
    understory_records = (
        _load_understory_assets(understory_asset_root, understory_slots)
        if understory_asset_root is not None and understory_slots
        else {}
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    collections = author._setup_scene(width, height, samples)
    materials = build_procedural_materials(bpy)
    hero_ground_material = _build_hero_ground_material(bpy)
    hero_trail_material = _build_hero_trail_material(bpy)
    lighting_contract = hero_lighting_contract()
    procedural_templates = build_visual_templates(bpy, collections["Templates"], materials)
    templates = author._import_external_asset_templates(
        records,
        collections["Templates"],
        procedural_templates,
        blender_override=bpy,
    )
    templates = _import_understory_asset_templates(
        understory_records,
        collections["Templates"],
        templates,
        blender_override=bpy,
    )
    _create_terrain(bpy, collections["Terrain"], hero_ground_material)
    _create_trail(bpy, collections["Trail"], plan, hero_trail_material)
    _create_trail_shoulders(bpy, collections["Trail"], plan)
    _create_trail_details(bpy, collections["Trail"], plan)
    _instantiate_plan(plan, templates, collections)
    _create_leaf_litter(bpy, collections["Dressing"], plan)
    lighting = setup_morning_fog(bpy, collections["Lighting"])
    lighting["fog"].scale = (130.0, 110.0, 24.0)
    lighting["fog"].location = (0.0, 0.0, 4.0)
    fog_nodes = [
        node
        for node in lighting["fog"].data.materials[0].node_tree.nodes
        if getattr(node, "bl_idname", "") == "ShaderNodeVolumePrincipled"
    ]
    if len(fog_nodes) != 1:
        raise RuntimeError("review fog must contain exactly one Principled Volume node")
    fog_nodes[0].inputs["Density"].default_value = float(lighting_contract["fog_density"])
    anisotropy = fog_nodes[0].inputs.get("Anisotropy")
    if anisotropy is None:
        raise RuntimeError("review fog is missing the Anisotropy input")
    anisotropy.default_value = float(lighting_contract["fog_anisotropy"])
    lighting["sun"].data.energy = float(lighting_contract["sun"]["energy"])
    lighting["sun"].data.angle = math.radians(
        float(lighting_contract["sun"]["angle_deg"])
    )
    lighting["fill"].data.energy = float(lighting_contract["fill"]["energy"])
    lighting["fill"].data.color = tuple(lighting_contract["fill"]["color"])
    lighting["fill"].data.size = float(lighting_contract["fill"]["size_m"])
    for light_id, light in lighting.items():
        author._set_visual_only(light, f"hero.lighting.{light_id}", f"review.lighting.{light_id}")
        light["review_only"] = True
    background = bpy.context.scene.world.node_tree.nodes.get("Background")
    if background is None:
        raise RuntimeError("review world is missing the Background node")
    background.inputs["Strength"].default_value = float(lighting_contract["world_strength"])
    bpy.context.scene.view_settings.exposure = float(lighting_contract["exposure"])
    applied_look: str | None = None
    for look in ("AgX - Medium High Contrast", "Medium High Contrast"):
        try:
            bpy.context.scene.view_settings.look = look
        except (TypeError, ValueError):
            continue
        applied_look = look
        break
    if applied_look is None:
        raise RuntimeError("review scene could not apply an approved AgX color look")
    camera = _create_camera(bpy, collections["Cameras"], plan["cameras"][0])
    bpy.context.scene.camera = camera

    robot_contract: dict[str, Any] | None = None
    if plan["review_actor"] is not None:
        if review_robot_asset_index is None:
            raise ValueError("include_thunder plan requires --review-robot-asset-index")
        robot_contract = author.load_review_robot_asset_index(review_robot_asset_index)
        package = robot_contract["visual_projection"].get("package", {})
        if package.get("id") != "thunderv4" or package.get("version") != "1.0.3":
            raise ValueError("review robot must identify thunderv4@1.0.3")
        robot = author._import_review_robot(
            review_robot_asset_index,
            {},
            plan,
            collections["ReviewActors"],
        )
        _relocate_review_robot(robot, plan["review_actor"])
        _make_review_robot_readable(robot)
        _create_robot_contact_shadows(bpy, collections["ReviewActors"], robot)
        _add_robot_rim_light(collections["Lighting"], plan["review_actor"])

    image_path = output_dir / _OUTPUTS[0]["filename"]
    bpy.context.scene.render.filepath = str(image_path)
    bpy.ops.render.render(write_still=True)
    image_payload = image_path.read_bytes()
    robot_evidence = None
    if robot_contract is not None:
        package_root = Path(robot_contract["package_root"])
        identity_paths = {
            "asset_index": Path(robot_contract["source_index"]),
            "runtime_recipe": package_root / "thunderv4-runtime.recipe.json",
            "visual_projection": package_root / "robot.visual-projection.json",
        }
        robot_evidence = {
            "package": {"id": "thunderv4", "version": "1.0.3"},
            "component_count": len(robot_contract["assets"]),
            "documents": {
                name: {
                    "bytes": path.stat().st_size,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                }
                for name, path in identity_paths.items()
            },
        }
    manifest = {
        "schema": "lingtu.sim.forest-hero-diorama-review.v1",
        "review_only": True,
        "runtime_contract": dict(_RUNTIME_CONTRACT),
        "plan_digest": _digest(plan),
        "external_asset_slots": deepcopy(plan["external_asset_slots"]),
        "understory_asset_slots": deepcopy(understory_slots),
        "review_robot": robot_evidence,
        "art_direction_contracts": {
            "ground_surface": hero_ground_surface_contract(),
            "ground_detail_counts": hero_ground_detail_counts(),
            "trail_surface": hero_trail_surface_contract(),
            "lighting": deepcopy(lighting_contract),
        },
        "render_settings": {
            "color_look": applied_look,
            "exposure": lighting_contract["exposure"],
            "fog_density": lighting_contract["fog_density"],
            "fog_anisotropy": lighting_contract["fog_anisotropy"],
            "world_strength": lighting_contract["world_strength"],
            "sun_energy": lighting_contract["sun"]["energy"],
            "sun_angle_deg": lighting_contract["sun"]["angle_deg"],
            "fill_energy": lighting_contract["fill"]["energy"],
            "fill_color": lighting_contract["fill"]["color"],
        },
        "artifacts": [
            {
                "artifact_role": "hero_diorama_preview",
                "filename": image_path.name,
                "bytes": len(image_payload),
                "sha256": hashlib.sha256(image_payload).hexdigest(),
            }
        ],
        "forbidden_outputs": ["blend", "fbx", "glb", "gltf", "production_manifest"],
    }
    manifest_path = output_dir / _OUTPUTS[1]["filename"]
    manifest_path.write_bytes(_canonical_json(manifest))
    allowed_names = {image_path.name, manifest_path.name}
    observed_names = {path.name for path in output_dir.iterdir()}
    if observed_names != allowed_names:
        raise RuntimeError("review output contains unexpected or forbidden artifacts")
    return {
        "schema": manifest["schema"],
        "preview": str(image_path),
        "review_manifest": str(manifest_path),
        "plan_digest": manifest["plan_digest"],
    }


def parse_cli_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse ordinary Python validation or Blender ``--`` arguments."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--external-asset-root", type=Path, required=True)
    parser.add_argument("--external-assets-json", type=Path, required=True)
    parser.add_argument("--understory-asset-root", type=Path)
    parser.add_argument("--understory-assets-json", type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--samples", type=int, default=48)
    parser.add_argument("--review-robot-asset-index", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    raw_args = list(sys.argv[1:] if argv is None else argv)
    if "--" in raw_args:
        raw_args = raw_args[raw_args.index("--") + 1 :]
    args = parser.parse_args(raw_args)
    if args.width < 64 or args.height < 64 or args.samples < 1:
        parser.error("width/height must be >= 64 and samples must be >= 1")
    args.repo_root = args.repo_root.resolve()
    if (args.understory_asset_root is None) != (args.understory_assets_json is None):
        parser.error(
            "--understory-asset-root and --understory-assets-json must be provided together"
        )
    for field in (
        "external_asset_root",
        "external_assets_json",
        "understory_asset_root",
        "understory_assets_json",
        "output_dir",
        "review_robot_asset_index",
    ):
        value = getattr(args, field)
        if value is not None and not value.is_absolute():
            setattr(args, field, args.repo_root / value)
    return args


def main(argv: Sequence[str] | None = None) -> int:
    """Validate identities or render the standalone review diorama."""

    args = parse_cli_args(argv)
    slots_document = json.loads(args.external_assets_json.read_text(encoding="utf-8"))
    if not isinstance(slots_document, Mapping):
        raise TypeError("--external-assets-json must contain an object")
    understory_document = None
    if args.understory_assets_json is not None:
        understory_document = json.loads(
            args.understory_assets_json.read_text(encoding="utf-8")
        )
        if not isinstance(understory_document, Mapping):
            raise TypeError("--understory-assets-json must contain an object")
    include_thunder = args.review_robot_asset_index is not None
    plan = build_hero_diorama_plan(
        slots_document,
        understory_asset_slots=understory_document,
        seed=args.seed,
        include_thunder=include_thunder,
    )
    if args.validate_only:
        _load_external_assets(args.external_asset_root, plan["external_asset_slots"])
        if plan["understory_asset_slots"]:
            _load_understory_assets(
                args.understory_asset_root,
                plan["understory_asset_slots"],
            )
        print(json.dumps({"valid": True, "plan_digest": _digest(plan)}, sort_keys=True))
        return 0
    result = build_hero_diorama(
        plan,
        args.external_asset_root,
        args.output_dir,
        repo_root=args.repo_root,
        width=args.width,
        height=args.height,
        samples=args.samples,
        review_robot_asset_index=args.review_robot_asset_index,
        understory_asset_root=args.understory_asset_root,
    )
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover - Blender/Python CLI path.
    raise SystemExit(main())
