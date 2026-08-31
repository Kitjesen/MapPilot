"""Generate the deterministic FactoryPark_HF physics and authoring assets.

The expanded layout is the single static-site authority.  MuJoCo geometry,
semantic entities, the Unreal import recipe, and the SVG site plan are derived
from that layout.  One unsigned 16-bit sample grid is likewise shared by the
MuJoCo heightfield, Unreal heightmap, and OBJ terrain mesh.
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import html
import json
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from sim.tools.worlds.factory_park_hf.elements import CompiledElementBatch, compile_element_batch

WORLD_ID = "factory_park_hf"
WORLD_VERSION = "1.0.0"
WORLD_PACKAGE = f"{WORLD_ID}@{WORLD_VERSION}"
DEFAULT_SEED = 20260808
GENERATOR_ALGORITHM = "lingtu_factory_park_layout.v2"
CANONICAL_ELEMENT_BATCHES = (
    Path(__file__).with_name("element_batches") / "operations_safety.v1.json",
)
_U16_MAX = 65_535
_U64_MASK = (1 << 64) - 1


@dataclass(frozen=True)
class TerrainSpec:
    """Physical extent and same-source sampling contract."""

    width_px: int = 221
    height_px: int = 181
    size_x_m: float = 220.0
    size_y_m: float = 180.0
    elevation_scale_m: float = 0.75
    base_depth_m: float = 1.0
    spawn_position_m: tuple[float, float, float] = (0.0, -76.0, 0.0)

    @property
    def spacing_x_m(self) -> float:
        """Return horizontal sample spacing in metres."""

        return self.size_x_m / (self.width_px - 1)

    @property
    def spacing_y_m(self) -> float:
        """Return vertical sample spacing in metres."""

        return self.size_y_m / (self.height_px - 1)


@dataclass(frozen=True)
class GeneratedWorld:
    """Paths and deterministic identity emitted by one generation."""

    repo_root: Path
    package_root: Path
    world_root: Path
    layout_digest: str
    asset_set_digest: str
    element_batch_ids: tuple[str, ...] = ()


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


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _splitmix64(value: int) -> int:
    value = (value + 0x9E3779B97F4A7C15) & _U64_MASK
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & _U64_MASK
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & _U64_MASK
    return value ^ (value >> 31)


def _hash_u16(seed: int, x: int, y: int) -> int:
    mixed = (
        (seed & _U64_MASK)
        ^ ((x * 0xD6E8FEB86659FD93) & _U64_MASK)
        ^ ((y * 0xA5A3564E27F8862B) & _U64_MASK)
    )
    return (_splitmix64(mixed) >> 48) & _U16_MAX


def _contains_xy(item: Mapping[str, object], x_m: float, y_m: float) -> bool:
    """Return whether a source-frame point lies in an object's footprint."""

    position = item["position_m"]
    center_x = float(position[0])  # type: ignore[index]
    center_y = float(position[1])  # type: ignore[index]
    offset_x = x_m - center_x
    offset_y = y_m - center_y
    if item["shape"] == "cylinder":
        return offset_x * offset_x + offset_y * offset_y <= float(item["radius_m"]) ** 2
    yaw_rad = math.radians(float(item["yaw_deg"]))
    local_x = math.cos(yaw_rad) * offset_x + math.sin(yaw_rad) * offset_y
    local_y = -math.sin(yaw_rad) * offset_x + math.cos(yaw_rad) * offset_y
    size = item["size_m"]
    return abs(local_x) <= float(size[0]) / 2.0 and abs(local_y) <= float(size[1]) / 2.0  # type: ignore[index]


def _height_samples(
    spec: TerrainSpec,
    seed: int,
    objects: Sequence[Mapping[str, object]],
) -> tuple[int, ...]:
    if spec.width_px < 2 or spec.height_px < 2:
        raise ValueError("heightfield must contain at least 2x2 samples")
    if spec.size_x_m <= 0 or spec.size_y_m <= 0:
        raise ValueError("terrain extent must be positive")
    if spec.elevation_scale_m <= 0 or spec.base_depth_m <= 0:
        raise ValueError("height scales must be positive")

    spawn_x, spawn_y, _ = spec.spawn_position_m
    datum = 36_000
    engineered_footprints = tuple(item for item in objects if bool(item["collision"]))
    raw: list[int] = []
    for row in range(spec.height_px):
        y_m = spec.size_y_m / 2.0 - row * spec.spacing_y_m
        for column in range(spec.width_px):
            x_m = -spec.size_x_m / 2.0 + column * spec.spacing_x_m
            noise = (_hash_u16(seed, column // 3, row // 3) - 32_768) * 450 // 32_768
            height = datum + int(y_m * 4.0) + noise

            # Static physical assets define engineered, level foundations.
            # This keeps road and pad tops above the terrain instead of merely
            # relying on small noise amplitude or coincident geometry.
            if any(_contains_xy(item, x_m, y_m) for item in engineered_footprints):
                height = datum

            # A concrete-lined drainage channel is cut into the eastern yard.
            if -56.0 <= y_m <= 56.0:
                ditch_distance = abs(x_m - 85.0)
                if ditch_distance < 2.0:
                    height -= int((2.0 - ditch_distance) * 8_500)

            # The northern gravel acceptance pad has deterministic roughness.
            if -90.0 <= x_m <= -62.0 and 69.0 <= y_m <= 83.0:
                gravel = _hash_u16(seed ^ 0x47524156, column, row) - 32_768
                height += gravel * 2_200 // 32_768

            # The robot spawn apron is exactly level and therefore replayable.
            if (x_m - spawn_x) ** 2 + (y_m - spawn_y) ** 2 <= 6.0**2:
                height = datum
            raw.append(height)

    minimum = min(raw)
    maximum = max(raw)
    if minimum == maximum:
        raise ValueError("heightfield synthesis produced no elevation range")
    source_range = maximum - minimum
    return tuple((sample - minimum) * _U16_MAX // source_range for sample in raw)


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = binascii.crc32(kind)
    checksum = binascii.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def _stored_zlib(payload: bytes) -> bytes:
    blocks = bytearray(b"\x78\x01")
    offset = 0
    while offset < len(payload):
        chunk = payload[offset : offset + 65_535]
        offset += len(chunk)
        blocks.append(1 if offset == len(payload) else 0)
        blocks.extend(struct.pack("<H", len(chunk)))
        blocks.extend(struct.pack("<H", 0xFFFF ^ len(chunk)))
        blocks.extend(chunk)
    blocks.extend(struct.pack(">I", zlib.adler32(payload) & 0xFFFFFFFF))
    return bytes(blocks)


def _height_png(spec: TerrainSpec, samples: Sequence[int]) -> bytes:
    rows = bytearray()
    for row in range(spec.height_px):
        rows.append(0)
        start = row * spec.width_px
        for sample in samples[start : start + spec.width_px]:
            rows.extend(struct.pack(">H", sample))
    header = struct.pack(">IIBBBBB", spec.width_px, spec.height_px, 16, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", _stored_zlib(bytes(rows)))
        + _png_chunk(b"IEND", b"")
    )


def _mujoco_hfield(spec: TerrainSpec, samples: Sequence[int]) -> bytes:
    payload = bytearray(struct.pack("<ii", spec.height_px, spec.width_px))
    for row in reversed(range(spec.height_px)):
        start = row * spec.width_px
        normalized = (sample / _U16_MAX for sample in samples[start : start + spec.width_px])
        payload.extend(struct.pack(f"<{spec.width_px}f", *normalized))
    return bytes(payload)


def _height_m(sample: int, spec: TerrainSpec, vertical_origin_m: float = 0.0) -> float:
    return sample * spec.elevation_scale_m / _U16_MAX + vertical_origin_m


def _terrain_obj(
    spec: TerrainSpec,
    samples: Sequence[int],
    seed: int,
    vertical_origin_m: float,
) -> bytes:
    lines = [
        "# LingTu FactoryPark_HF deterministic terrain mesh",
        f"# seed={seed} source_grid={spec.width_px}x{spec.height_px}",
        "# coordinates=unreal_lh_z_up_cm; one vertex per heightfield sample",
        "o FactoryPark_HF_Terrain",
    ]
    for row in range(spec.height_px):
        source_y_m = spec.size_y_m / 2.0 - row * spec.spacing_y_m
        ue_y_cm = -source_y_m * 100.0
        for column in range(spec.width_px):
            source_x_m = -spec.size_x_m / 2.0 + column * spec.spacing_x_m
            sample = samples[row * spec.width_px + column]
            lines.append(
                f"v {source_x_m * 100.0:.6f} {ue_y_cm:.6f} "
                f"{_height_m(sample, spec, vertical_origin_m) * 100.0:.6f}"
            )
    for row in range(spec.height_px - 1):
        for column in range(spec.width_px - 1):
            north_west = row * spec.width_px + column + 1
            north_east = north_west + 1
            south_west = north_west + spec.width_px
            south_east = south_west + 1
            lines.append(f"f {north_west} {north_east} {south_east}")
            lines.append(f"f {north_west} {south_east} {south_west}")
    return ("\n".join(lines) + "\n").encode("ascii")


def _coordinate_contract(spec: TerrainSpec, samples: Sequence[int]) -> dict[str, object]:
    spawn_x, spawn_y, spawn_z = spec.spawn_position_m
    spawn_column = round((spawn_x + spec.size_x_m / 2.0) / spec.spacing_x_m)
    spawn_row = round((spec.size_y_m / 2.0 - spawn_y) / spec.spacing_y_m)
    spawn_sample = samples[spawn_row * spec.width_px + spawn_column]
    vertical_origin_m = spawn_z - _height_m(spawn_sample, spec)
    return {
        "source_frame": "mujoco_rh_z_up_m",
        "unreal_frame": "unreal_lh_z_up_cm",
        "axis_mapping": ["x", "-y", "z"],
        "origin": "site_center",
        "vertical_origin_m": vertical_origin_m,
        "spawn_reference": {
            "sample_pixel": [spawn_column, spawn_row],
            "sample_u16": spawn_sample,
            "world_position_m": [spawn_x, spawn_y, spawn_z],
        },
        "extent_m": [spec.size_x_m, spec.size_y_m],
        "grid_px": [spec.width_px, spec.height_px],
        "sample_spacing_m": [spec.spacing_x_m, spec.spacing_y_m],
        "height_encoding": {
            "type": "uint16_normalized",
            "bit_depth": 16,
            "range": [0, _U16_MAX],
            "elevation_scale_m": spec.elevation_scale_m,
            "formula": "world_height_m = sample_u16 / 65535 * elevation_scale_m + vertical_origin_m",
        },
        "image_mapping": {
            "column_0_x_m": -spec.size_x_m / 2.0,
            "column_last_x_m": spec.size_x_m / 2.0,
            "row_0_y_m": spec.size_y_m / 2.0,
            "row_last_y_m": -spec.size_y_m / 2.0,
        },
        "mujoco_hfield": {
            "size": [
                spec.size_x_m / 2.0,
                spec.size_y_m / 2.0,
                spec.elevation_scale_m,
                spec.base_depth_m,
            ],
            "geom_position_m": [0.0, 0.0, vertical_origin_m],
        },
        "unreal_landscape": {
            "location_cm": [
                0.0,
                0.0,
                spec.elevation_scale_m * 50.0 + vertical_origin_m * 100.0,
            ],
            "scale_xyz": [
                spec.spacing_x_m * 100.0,
                spec.spacing_y_m * 100.0,
                spec.elevation_scale_m * 100.0 / 512.0,
            ],
        },
    }


def _box(
    object_id: str,
    semantic_class: str,
    position_m: Sequence[float],
    size_m: Sequence[float],
    material: str,
    *,
    collision: bool,
    visual_only: bool = False,
    yaw_deg: float = 0.0,
    **properties: object,
) -> dict[str, object]:
    item: dict[str, object] = {
        "id": object_id,
        "semantic_class": semantic_class,
        "shape": "box",
        "position_m": [float(value) for value in position_m],
        "size_m": [float(value) for value in size_m],
        "yaw_deg": float(yaw_deg),
        "material": material,
        "collision": collision,
        "visual_only": visual_only,
    }
    item.update(properties)
    return item


def _cylinder(
    object_id: str,
    semantic_class: str,
    position_m: Sequence[float],
    radius_m: float,
    half_height_m: float,
    material: str,
    *,
    collision: bool,
    visual_only: bool = False,
    yaw_deg: float = 0.0,
    **properties: object,
) -> dict[str, object]:
    item: dict[str, object] = {
        "id": object_id,
        "semantic_class": semantic_class,
        "shape": "cylinder",
        "position_m": [float(value) for value in position_m],
        "radius_m": float(radius_m),
        "half_height_m": float(half_height_m),
        "yaw_deg": float(yaw_deg),
        "material": material,
        "collision": collision,
        "visual_only": visual_only,
    }
    item.update(properties)
    return item


def _factory_objects() -> list[dict[str, object]]:
    objects: list[dict[str, object]] = []

    roads = (
        ("road_ring_north", [0, 62, 0.025], [186, 10, 0.05], 10.0),
        ("road_ring_south", [0, -62, 0.025], [186, 10, 0.05], 10.0),
        ("road_ring_west", [-93, 0, 0.025], [10, 124, 0.05], 10.0),
        ("road_ring_east", [93, 0, 0.025], [10, 124, 0.05], 10.0),
        ("road_central_cross", [0, 0, 0.026], [186, 12, 0.052], 12.0),
        ("road_entry_boulevard", [0, -44, 0.027], [12, 88, 0.054], 12.0),
        ("road_central_spine", [0, 31, 0.027], [12, 62, 0.054], 12.0),
        ("road_loading_apron", [50, 9, 0.026], [70, 8, 0.052], 8.0),
        ("road_gravel_access", [-76, 68, 0.026], [8, 12, 0.052], 8.0),
    )
    for object_id, position, size, width in roads:
        objects.append(
            _box(
                object_id,
                "road",
                position,
                size,
                "asphalt",
                collision=True,
                clear_width_m=width,
                surface="vehicle",
                friction=[1.0, 0.02, 0.001],
            )
        )

    objects.extend(
        [
            _box(
                "parking_apron",
                "parking_area",
                [25, -34, 0.022],
                [38, 34, 0.044],
                "asphalt",
                collision=True,
                clear_width_m=6.0,
            ),
            _box(
                "container_yard_apron",
                "container_yard",
                [-52, -33, 0.025],
                [58, 42, 0.05],
                "concrete",
                collision=True,
                clear_aisle_width_m=6.0,
            ),
            _box(
                "tank_bund_floor",
                "containment_bund",
                [65, -36, 0.02],
                [35, 35, 0.04],
                "bund_concrete",
                collision=True,
            ),
            _box(
                "gravel_acceptance_pad",
                "gravel_test_area",
                [-76, 76, 0.025],
                [28, 14, 0.05],
                "gravel",
                collision=True,
                terrain_modifier="roughen",
                friction=[0.78, 0.03, 0.002],
            ),
        ]
    )

    objects.extend(
        [
            _box("fence_north", "boundary_fence", [0, 89.4, 1.2], [219, 0.2, 2.4], "fence_steel", collision=True),
            _box("fence_west", "boundary_fence", [-109.4, 0, 1.2], [0.2, 179, 2.4], "fence_steel", collision=True),
            _box("fence_east", "boundary_fence", [109.4, 0, 1.2], [0.2, 179, 2.4], "fence_steel", collision=True),
            _box("fence_south_west", "boundary_fence", [-59.25, -89.4, 1.2], [100.5, 0.2, 2.4], "fence_steel", collision=True),
            _box("fence_south_east", "boundary_fence", [59.25, -89.4, 1.2], [100.5, 0.2, 2.4], "fence_steel", collision=True),
            _box("gate_leaf_west_open", "gate", [-8.7, -86.2, 1.1], [0.25, 6.0, 2.2], "painted_steel", collision=True, state="open"),
            _box("gate_leaf_east_open", "gate", [8.7, -86.2, 1.1], [0.25, 6.0, 2.2], "painted_steel", collision=True, state="open"),
            _cylinder("gate_post_west", "gate_post", [-9, -89, 1.5], 0.22, 1.5, "painted_steel", collision=True),
            _cylinder("gate_post_east", "gate_post", [9, -89, 1.5], 0.22, 1.5, "painted_steel", collision=True),
            _box("gate_canopy", "gate_canopy", [0, -84.5, 3.6], [20, 1.2, 0.5], "painted_steel", collision=True, clear_height_m=3.35),
            _box("south_guardhouse", "guardhouse", [-14, -80, 1.8], [7, 6, 3.6], "guardhouse", collision=True),
        ]
    )

    objects.extend(
        [
            _box("main_factory", "main_factory_building", [-47, 32, 6], [70, 40, 12], "factory_cladding", collision=True),
            _box("main_factory_roof_monitor", "roof_structure", [-47, 32, 12.8], [34, 5, 1.6], "glass", collision=True),
            _box("warehouse", "warehouse_building", [50, 34, 5], [60, 32, 10], "warehouse_cladding", collision=True),
            _box("administration_office", "office_building", [22, 77, 4], [24, 10, 8], "guardhouse", collision=True),
        ]
    )

    for index, x_m in enumerate((30.0, 50.0, 70.0), start=1):
        objects.append(
            _box(
                f"loading_dock_{index:02d}",
                "loading_dock",
                [x_m, 15.7, 0.65],
                [8, 4.4, 1.3],
                "concrete",
                collision=True,
                dock_height_m=1.3,
            )
        )
        objects.append(
            _box(
                f"loading_ramp_{index:02d}",
                "loading_ramp",
                [x_m, 11.2, 0.34],
                [5.2, 5.0, 0.24],
                "steel",
                collision=True,
                yaw_deg=90.0,
                pitch_deg=-6.0,
                slope_percent=10.5,
            )
        )

    container_specs = (
        ("container_blue_01", -68.0, -45.0, 1.295, "container_blue"),
        ("container_orange_01", -50.0, -45.0, 1.295, "container_orange"),
        ("container_red_01", -32.0, -45.0, 1.295, "container_red"),
        ("container_orange_02", -68.0, -27.0, 1.295, "container_orange"),
        ("container_blue_02", -50.0, -27.0, 1.295, "container_blue"),
        ("container_red_02", -32.0, -27.0, 1.295, "container_red"),
        ("container_blue_03_stacked", -68.0, -45.0, 3.885, "container_blue"),
    )
    for object_id, x_m, y_m, z_m, material in container_specs:
        objects.append(
            _box(object_id, "shipping_container", [x_m, y_m, z_m], [12.2, 2.44, 2.59], material, collision=True)
        )

    objects.extend(
        [
            _box("tank_bund_west", "containment_bund", [47.5, -36, 0.6], [0.5, 35, 1.2], "bund_concrete", collision=True),
            _box("tank_bund_east", "containment_bund", [82.5, -36, 0.6], [0.5, 35, 1.2], "bund_concrete", collision=True),
            _box("tank_bund_north", "containment_bund", [65, -18.5, 0.6], [35, 0.5, 1.2], "bund_concrete", collision=True),
            _box("tank_bund_south", "containment_bund", [65, -53.5, 0.6], [35, 0.5, 1.2], "bund_concrete", collision=True),
            _cylinder("storage_tank_01", "storage_tank", [57, -43, 4], 4, 4, "tank_white", collision=True),
            _cylinder("storage_tank_02", "storage_tank", [73, -43, 4], 4, 4, "tank_white", collision=True),
            _cylinder("storage_tank_03", "storage_tank", [65, -28, 4], 4, 4, "tank_white", collision=True),
            _box("tank_pipe_rack", "pipe_rack", [65, -36, 3.0], [24, 0.35, 0.35], "steel", collision=True),
        ]
    )

    for row_index, y_m in enumerate((-44.0, -24.0), start=1):
        for column_index, x_m in enumerate((13.0, 21.0, 29.0, 37.0), start=1):
            objects.append(
                _box(
                    f"parking_space_{row_index}_{column_index}",
                    "parking_space",
                    [x_m, y_m, 0.051],
                    [2.7, 5.4, 0.012],
                    "parking_marking",
                    collision=False,
                    visual_only=True,
                    accessible=(row_index == 2 and column_index == 1),
                )
            )
            stop_y = y_m + (2.45 if row_index == 1 else -2.45)
            objects.append(
                _box(
                    f"parking_stop_{row_index}_{column_index}",
                    "parking_stop",
                    [x_m, stop_y, 0.09],
                    [2.35, 0.22, 0.18],
                    "curb_concrete",
                    collision=True,
                )
            )

    objects.extend(
        [
            _box("parking_curb_west", "curb", [6.4, -34, 0.09], [0.3, 34, 0.18], "curb_concrete", collision=True),
            _box("parking_curb_east", "curb", [43.6, -34, 0.09], [0.3, 34, 0.18], "curb_concrete", collision=True),
            _box("parking_curb_north", "curb", [25, -16.9, 0.09], [37.5, 0.3, 0.18], "curb_concrete", collision=True),
            _box("speed_bump_south", "speed_bump", [0, -68, 0.065], [11.5, 0.5, 0.13], "rubber", collision=True),
            _box("speed_bump_gate", "speed_bump", [0, -58, 0.065], [11.5, 0.5, 0.13], "rubber", collision=True),
            _box("speed_bump_north", "speed_bump", [0, 55, 0.065], [11.5, 0.5, 0.13], "rubber", collision=True),
            _box(
                "east_drainage_channel",
                "drainage_ditch",
                [85, 0, -0.18],
                [2.0, 112, 0.36],
                "drainage_concrete",
                collision=False,
                terrain_modifier="subtract",
                depth_m=0.45,
            ),
            _box("east_drainage_grate", "drainage_grate", [85, 0, 0.025], [2.2, 12, 0.05], "steel", collision=True),
        ]
    )

    for index, (x_m, y_m) in enumerate(
        ((-88, -58), (-88, 58), (-5, 58), (88, 58), (88, -58), (5, -58), (5, 5), (-88, 5)),
        start=1,
    ):
        objects.append(
            _cylinder(
                f"streetlight_{index:02d}_pole",
                "streetlight",
                [x_m, y_m, 3.0],
                0.12,
                3.0,
                "painted_steel",
                collision=True,
            )
        )
        objects.append(
            _box(
                f"streetlight_{index:02d}_head",
                "streetlight_luminaire",
                [x_m + 0.45, y_m, 6.05],
                [1.0, 0.35, 0.15],
                "lamp",
                collision=False,
                visual_only=True,
            )
        )

    for index, y_m in enumerate(range(-74, 56, 12), start=1):
        objects.append(
            _box(
                f"boulevard_lane_mark_{index:02d}",
                "lane_marking",
                [0, float(y_m), 0.06],
                [0.14, 4.0, 0.01],
                "road_marking",
                collision=False,
                visual_only=True,
            )
        )
    for index, x_m in enumerate(range(-84, 85, 12), start=1):
        objects.append(
            _box(
                f"cross_street_lane_mark_{index:02d}",
                "lane_marking",
                [float(x_m), 0, 0.06],
                [4.0, 0.14, 0.01],
                "road_marking",
                collision=False,
                visual_only=True,
            )
        )
    for index, x_m in enumerate((-4.5, -2.7, -0.9, 0.9, 2.7, 4.5), start=1):
        objects.append(
            _box(
                f"pedestrian_crossing_stripe_{index:02d}",
                "pedestrian_crossing",
                [x_m, -10, 0.061],
                [1.0, 4.0, 0.012],
                "road_marking",
                collision=False,
                visual_only=True,
            )
        )

    checkpoint_positions = (
        ("checkpoint_gate", 0.0, -76.0),
        ("checkpoint_container_yard", -16.0, -33.0),
        ("checkpoint_factory", -6.0, 32.0),
        ("checkpoint_gravel", -76.0, 69.5),
        ("checkpoint_warehouse", 50.0, 7.0),
        ("checkpoint_tank_farm", 88.0, -36.0),
    )
    for object_id, x_m, y_m in checkpoint_positions:
        objects.append(
            _cylinder(
                object_id,
                "semantic_checkpoint",
                [x_m, y_m, 0.065],
                0.8,
                0.012,
                "checkpoint_green",
                collision=False,
                visual_only=True,
            )
        )

    ids = [str(item["id"]) for item in objects]
    if len(ids) != len(set(ids)):
        raise ValueError("factory layout contains duplicate object ids")
    return objects


def _routes() -> list[dict[str, object]]:
    return [
        {
            "id": "robot_acceptance_loop",
            "semantic_class": "robot_route",
            "width_m": 4.0,
            "closed": True,
            "waypoints_m": [
                [0.0, -76.0, 0.0],
                [0.0, -62.0, 0.0],
                [-93.0, -62.0, 0.0],
                [-93.0, 62.0, 0.0],
                [-76.0, 68.0, 0.0],
                [-76.0, 76.0, 0.0],
                [-76.0, 68.0, 0.0],
                [0.0, 62.0, 0.0],
                [93.0, 62.0, 0.0],
                [93.0, -62.0, 0.0],
                [0.0, -62.0, 0.0],
                [0.0, -76.0, 0.0],
            ],
        },
        {
            "id": "forklift_logistics_route",
            "semantic_class": "forklift_route",
            "width_m": 3.5,
            "closed": True,
            "waypoints_m": [
                [-18.0, -33.0, 0.0],
                [-10.0, -33.0, 0.0],
                [-10.0, 0.0, 0.0],
                [15.0, 0.0, 0.0],
                [15.0, 9.0, 0.0],
                [74.0, 9.0, 0.0],
                [74.0, 5.0, 0.0],
                [15.0, 5.0, 0.0],
                [15.0, -8.0, 0.0],
                [-18.0, -8.0, 0.0],
                [-18.0, -33.0, 0.0],
            ],
        },
        {
            "id": "pedestrian_safe_route",
            "semantic_class": "pedestrian_route",
            "width_m": 1.8,
            "closed": True,
            "waypoints_m": [
                [-9.0, -78.0, 0.0],
                [-7.0, -70.0, 0.0],
                [-7.0, -8.0, 0.0],
                [-10.0, 7.0, 0.0],
                [-10.0, 56.0, 0.0],
                [10.0, 56.0, 0.0],
                [10.0, 7.0, 0.0],
                [7.0, -8.0, 0.0],
                [7.0, -70.0, 0.0],
                [9.0, -78.0, 0.0],
                [-9.0, -78.0, 0.0],
            ],
        },
    ]


def _checkpoints() -> list[dict[str, object]]:
    return [
        {"id": "checkpoint_gate", "label": "南门与门岗", "position_m": [0.0, -76.0, 0.0], "radius_m": 0.8, "route_id": "robot_acceptance_loop"},
        {"id": "checkpoint_container_yard", "label": "集装箱堆场", "position_m": [-16.0, -33.0, 0.0], "radius_m": 0.8, "route_id": "forklift_logistics_route"},
        {"id": "checkpoint_factory", "label": "主厂房", "position_m": [-6.0, 32.0, 0.0], "radius_m": 0.8, "route_id": "pedestrian_safe_route"},
        {"id": "checkpoint_gravel", "label": "碎石测试区", "position_m": [-76.0, 69.5, 0.0], "radius_m": 0.8, "route_id": "robot_acceptance_loop"},
        {"id": "checkpoint_warehouse", "label": "仓库装卸区", "position_m": [50.0, 7.0, 0.0], "radius_m": 0.8, "route_id": "forklift_logistics_route"},
        {"id": "checkpoint_tank_farm", "label": "罐区围堰", "position_m": [88.0, -36.0, 0.0], "radius_m": 0.8, "route_id": "robot_acceptance_loop"},
    ]


def _expanded_layout(
    spec: TerrainSpec,
    seed: int,
    coordinate_contract: Mapping[str, object],
    objects: Sequence[Mapping[str, object]],
    element_batches: Sequence[CompiledElementBatch] = (),
) -> tuple[dict[str, object], str]:
    layout_without_digest: dict[str, object] = {
        "schema": "lingtu.sim.expanded-world-layout.v1",
        "world_package": WORLD_PACKAGE,
        "seed": seed,
        "generator": {
            "module": "sim.tools.worlds.factory_park_hf.generate",
            "algorithm": GENERATOR_ALGORITHM,
        },
        "coordinate_system": {
            "frame": "mujoco_rh_z_up_m",
            "handedness": "right",
            "up_axis": "z",
            "linear_unit": "metre",
        },
        "coordinate_contract": dict(coordinate_contract),
        "extent_m": [spec.size_x_m, spec.size_y_m],
        "site_boundary_m": {
            "minimum": [-spec.size_x_m / 2.0, -spec.size_y_m / 2.0, -spec.base_depth_m],
            "maximum": [spec.size_x_m / 2.0, spec.size_y_m / 2.0, 16.0],
        },
        "spawn": {
            "id": "thunder_01_spawn",
            "position_m": list(spec.spawn_position_m),
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "clearance_radius_m": 4.0,
            "surface_id": "road_entry_boulevard",
        },
        "terrain": {
            "heightfield_r16_png": "sim/worlds/factory_park_hf/generated/heightfield_r16.png",
            "mujoco_heightfield": "sim/worlds/factory_park_hf/generated/heightfield_f32.bin",
            "terrain_obj": "sim/worlds/factory_park_hf/generated/terrain.obj",
            "grid_px": [spec.width_px, spec.height_px],
            "extent_m": [spec.size_x_m, spec.size_y_m],
            "elevation_scale_m": spec.elevation_scale_m,
            "source_frame": "mujoco_rh_z_up_m",
            "obj_frame": "unreal_lh_z_up_cm",
        },
        "objects": [dict(item) for item in objects],
        "routes": _routes(),
        "checkpoints": _checkpoints(),
        "validation": {
            "minimum_vehicle_road_width_m": 8.0,
            "minimum_forklift_route_width_m": 3.5,
            "minimum_pedestrian_route_width_m": 1.8,
            "south_gate_clear_width_m": 18.0,
            "spawn_clearance_radius_m": 4.0,
            "required_semantic_classes": [
                "road",
                "main_factory_building",
                "warehouse_building",
                "loading_dock",
                "loading_ramp",
                "shipping_container",
                "storage_tank",
                "containment_bund",
                "parking_area",
                "guardhouse",
                "drainage_ditch",
                "speed_bump",
                "curb",
                "gravel_test_area",
                "boundary_fence",
                "gate",
            ],
        },
    }
    if element_batches:
        layout_without_digest["element_batches"] = [
            {
                "batch_id": batch.batch_id,
                "digest": batch.digest,
                "object_count": len(batch.objects),
            }
            for batch in element_batches
        ]
    layout_digest = _sha256(_canonical_json(layout_without_digest))
    layout = dict(layout_without_digest)
    layout["layout_digest"] = layout_digest
    return layout, layout_digest


def _semantic_entities(layout: Mapping[str, object]) -> bytes:
    entities = []
    for item in layout["objects"]:  # type: ignore[index]
        object_item = dict(item)
        object_item["physics"] = {
            "authority": "mujoco",
            "collision": object_item["collision"],
            "visual_only": object_item["visual_only"],
        }
        entities.append(object_item)
    return _canonical_json(
        {
            "schema": "lingtu.sim.semantic-entities.v1",
            "world_package": WORLD_PACKAGE,
            "coordinate_frame": "mujoco_rh_z_up_m",
            "layout_digest": layout["layout_digest"],
            "entities": entities,
            "routes": layout["routes"],
            "checkpoints": layout["checkpoints"],
        }
    )


_MATERIAL_RGBA: dict[str, tuple[float, float, float, float]] = {
    "asphalt": (0.16, 0.18, 0.19, 1.0),
    "bund_concrete": (0.54, 0.55, 0.52, 1.0),
    "checkpoint_green": (0.10, 0.85, 0.30, 0.80),
    "concrete": (0.55, 0.57, 0.56, 1.0),
    "container_blue": (0.08, 0.28, 0.55, 1.0),
    "container_orange": (0.82, 0.32, 0.06, 1.0),
    "container_red": (0.62, 0.08, 0.06, 1.0),
    "curb_concrete": (0.76, 0.76, 0.70, 1.0),
    "drainage_concrete": (0.35, 0.38, 0.39, 1.0),
    "factory_cladding": (0.28, 0.38, 0.43, 1.0),
    "fence_steel": (0.31, 0.34, 0.34, 1.0),
    "glass": (0.25, 0.48, 0.60, 0.72),
    "gravel": (0.36, 0.32, 0.26, 1.0),
    "guardhouse": (0.72, 0.72, 0.67, 1.0),
    "lamp": (1.0, 0.88, 0.52, 1.0),
    "painted_steel": (0.92, 0.74, 0.08, 1.0),
    "pallet_wood": (0.48, 0.31, 0.15, 1.0),
    "parking_marking": (0.88, 0.88, 0.82, 0.85),
    "road_marking": (0.96, 0.86, 0.18, 0.90),
    "rubber": (0.08, 0.08, 0.07, 1.0),
    "steel": (0.32, 0.34, 0.36, 1.0),
    "tank_white": (0.82, 0.84, 0.80, 1.0),
    "warehouse_cladding": (0.54, 0.58, 0.58, 1.0),
}


def _format_values(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def _mjcf(
    spec: TerrainSpec,
    vertical_origin_m: float,
    layout: Mapping[str, object],
) -> bytes:
    lines = [
        f"<!-- Generated FactoryPark_HF world; layout_sha256={layout['layout_digest']} -->",
        '<mujoco model="factory_park_hf">',
        '  <compiler angle="radian" autolimits="true"/>',
        '  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4" solver="Newton" iterations="100"/>',
        '  <visual><headlight diffuse=".55 .55 .52" ambient=".28 .30 .32" specular=".05 .05 .05"/></visual>',
        "  <asset>",
        '    <hfield name="factory_park_terrain" file="generated/heightfield_f32.bin" '
        'content_type="image/vnd.mujoco.hfield" '
        f'size="{spec.size_x_m / 2.0:g} {spec.size_y_m / 2.0:g} {spec.elevation_scale_m:g} {spec.base_depth_m:g}"/>',
        '    <texture name="terrain_grid" type="2d" builtin="checker" rgb1=".23 .25 .20" rgb2=".27 .28 .23" width="512" height="512"/>',
        '    <material name="terrain_ground" texture="terrain_grid" texrepeat="44 36" reflectance="0.01" roughness="0.9"/>',
    ]
    for material, rgba in sorted(_MATERIAL_RGBA.items()):
        lines.append(
            f'    <material name="{material}" rgba="{_format_values(rgba)}" '
            'specular="0.05" shininess="0.04"/>'
        )
    lines.extend(
        [
            "  </asset>",
            "  <worldbody>",
            '    <geom name="terrain" type="hfield" hfield="factory_park_terrain" '
            f'pos="0 0 {vertical_origin_m:.9g}" material="terrain_ground" '
            'contype="1" conaffinity="1" condim="3" friction="0.92 0.02 0.001"/>',
        ]
    )
    for item in layout["objects"]:  # type: ignore[index]
        object_id = html.escape(str(item["id"]), quote=True)
        shape = str(item["shape"])
        position = _format_values(item["position_m"])  # type: ignore[arg-type]
        material = html.escape(str(item["material"]), quote=True)
        if shape == "box":
            size = _format_values(float(value) / 2.0 for value in item["size_m"])  # type: ignore[index]
        elif shape == "cylinder":
            size = _format_values((float(item["radius_m"]), float(item["half_height_m"])))
        else:
            raise ValueError(f"unsupported layout shape {shape!r}")
        attributes = [
            f'name="{object_id}"',
            f'type="{shape}"',
            f'pos="{position}"',
            f'size="{size}"',
            f'material="{material}"',
        ]
        pitch_deg = float(item.get("pitch_deg", 0.0))
        yaw_deg = float(item["yaw_deg"])
        if pitch_deg or yaw_deg:
            attributes.append(
                f'euler="0 {math.radians(pitch_deg):.9g} {math.radians(yaw_deg):.9g}"'
            )
        if bool(item["collision"]):
            friction = item.get("friction", [0.9, 0.02, 0.001])
            attributes.extend(
                [
                    'contype="1"',
                    'conaffinity="1"',
                    'condim="3"',
                    f'friction="{_format_values(friction)}"',  # type: ignore[arg-type]
                ]
            )
        else:
            attributes.extend(['contype="0"', 'conaffinity="0"'])
        lines.append("    <geom " + " ".join(attributes) + "/>")

    for checkpoint in layout["checkpoints"]:  # type: ignore[index]
        position = list(checkpoint["position_m"])  # type: ignore[index]
        position[2] = 0.12
        lines.append(
            f'    <site name="site_{html.escape(str(checkpoint["id"]), quote=True)}" '
            f'type="cylinder" pos="{_format_values(position)}" size="{float(checkpoint["radius_m"]):.9g} 0.01" '
            'rgba=".1 .9 .3 .35"/>'
        )
    lines.extend(
        [
            '    <light name="sun" pos="20 -40 70" dir="-.25 .35 -1" directional="true" diffuse=".9 .87 .78" specular=".08 .08 .08"/>',
            '    <light name="yard_fill" pos="-70 -50 18" dir=".4 .2 -1" diffuse=".28 .31 .34"/>',
            "  </worldbody>",
            "</mujoco>",
        ]
    )
    return ("\n".join(lines) + "\n").encode("utf-8")


_SVG_COLORS = {
    material: "#{:02x}{:02x}{:02x}".format(*(round(channel * 255) for channel in rgba[:3]))
    for material, rgba in _MATERIAL_RGBA.items()
}


def _site_plan_svg(layout: Mapping[str, object]) -> bytes:
    scale = 5.0
    half_x = float(layout["extent_m"][0]) / 2.0  # type: ignore[index]
    half_y = float(layout["extent_m"][1]) / 2.0  # type: ignore[index]

    def project(position: Sequence[float]) -> tuple[float, float]:
        return ((float(position[0]) + half_x) * scale, (half_y - float(position[1])) * scale)

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {half_x * 2 * scale:g} {half_y * 2 * scale:g}" role="img" aria-labelledby="title desc">',
        "  <title id=" + '"title">FactoryPark_HF site plan</title>',
        f'  <desc id="desc">Deterministic 220 by 180 metre LingTu robot simulation factory park. Layout {layout["layout_digest"]}.</desc>',
        '  <rect width="100%" height="100%" fill="#59634d"/>',
        '  <g id="objects" stroke="#111820" stroke-width="1">',
    ]
    for item in layout["objects"]:  # type: ignore[index]
        x_px, y_px = project(item["position_m"])  # type: ignore[arg-type]
        color = _SVG_COLORS[str(item["material"])]
        opacity = "0.58" if bool(item["visual_only"]) else "0.92"
        common = (
            f'id="{html.escape(str(item["id"]), quote=True)}" '
            f'data-semantic-class="{html.escape(str(item["semantic_class"]), quote=True)}" '
            f'fill="{color}" fill-opacity="{opacity}"'
        )
        if item["shape"] == "box":
            width = float(item["size_m"][0]) * scale  # type: ignore[index]
            height = float(item["size_m"][1]) * scale  # type: ignore[index]
            rotation = -float(item["yaw_deg"])
            lines.append(
                f'    <rect {common} x="{x_px - width / 2:.3f}" y="{y_px - height / 2:.3f}" '
                f'width="{width:.3f}" height="{height:.3f}" transform="rotate({rotation:.3f} {x_px:.3f} {y_px:.3f})"/>'
            )
        else:
            radius = float(item["radius_m"]) * scale
            lines.append(f'    <circle {common} cx="{x_px:.3f}" cy="{y_px:.3f}" r="{radius:.3f}"/>')
    lines.append("  </g>")
    route_colors = ("#48d7ff", "#ff9e3d", "#f9f4b7")
    lines.append('  <g id="routes" fill="none">')
    for route, color in zip(layout["routes"], route_colors, strict=True):  # type: ignore[index]
        points = " ".join(f"{x:.3f},{y:.3f}" for x, y in (project(point) for point in route["waypoints_m"]))  # type: ignore[index]
        lines.append(
            f'    <polyline id="{route["id"]}" data-semantic-class="{route["semantic_class"]}" '
            f'points="{points}" stroke="{color}" stroke-width="{float(route["width_m"]) * scale:.3f}" '
            'stroke-opacity="0.45" stroke-linejoin="round" stroke-linecap="round"/>'
        )
    lines.append("  </g>")
    lines.append('  <g id="checkpoint-labels" font-family="sans-serif" font-size="11" fill="#ffffff">')
    for checkpoint in layout["checkpoints"]:  # type: ignore[index]
        x_px, y_px = project(checkpoint["position_m"])  # type: ignore[arg-type]
        label = html.escape(str(checkpoint["label"]))
        lines.append(f'    <text x="{x_px + 7:.3f}" y="{y_px - 7:.3f}">{label}</text>')
    lines.extend(
        [
            "  </g>",
            f'  <text x="14" y="24" font-family="sans-serif" font-size="18" fill="#ffffff">FactoryPark_HF · {WORLD_PACKAGE}</text>',
            "</svg>",
        ]
    )
    return ("\n".join(lines) + "\n").encode("utf-8")


def _realism_recipe(layout_digest: str, seed: int) -> bytes:
    """Return the deterministic visual-dressing contract for the factory park."""

    material_profiles: dict[str, object] = {
        "asphalt_worn": {
            "applies_to": ["asphalt"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.12, 0.13, 0.14],
            "metallic": 0.0,
            "roughness_range": [0.72, 0.92],
            "channels": ["base_color", "normal", "roughness", "ambient_occlusion"],
            "texel_density_px_per_m": 512,
            "variation": ["aggregate", "patches", "tire_wear", "oil_stains", "sealed_cracks"],
        },
        "concrete_weathered": {
            "applies_to": ["concrete", "bund_concrete", "curb_concrete", "drainage_concrete"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.48, 0.49, 0.47],
            "metallic": 0.0,
            "roughness_range": [0.68, 0.9],
            "channels": ["base_color", "normal", "roughness", "ambient_occlusion"],
            "texel_density_px_per_m": 512,
            "variation": ["formwork", "water_streaks", "edge_darkening", "efflorescence", "repairs"],
        },
        "factory_cladding_blue": {
            "applies_to": ["factory_cladding"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.17, 0.29, 0.34],
            "metallic": 0.45,
            "roughness_range": [0.42, 0.68],
            "channels": ["base_color", "normal", "roughness", "metallic", "ambient_occlusion"],
            "texel_density_px_per_m": 512,
            "variation": ["corrugation", "panel_seams", "fasteners", "sun_fade", "rain_streaks"],
        },
        "warehouse_galvanized": {
            "applies_to": ["warehouse_cladding", "fence_steel", "steel"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.47, 0.5, 0.5],
            "metallic": 0.82,
            "roughness_range": [0.34, 0.62],
            "channels": ["base_color", "normal", "roughness", "metallic", "ambient_occlusion"],
            "texel_density_px_per_m": 512,
            "variation": ["zinc_spangle", "panel_seams", "fasteners", "oxidation", "handling_marks"],
        },
        "painted_safety_steel": {
            "applies_to": ["painted_steel"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.86, 0.61, 0.03],
            "metallic": 0.55,
            "roughness_range": [0.38, 0.66],
            "channels": ["base_color", "normal", "roughness", "metallic", "ambient_occlusion"],
            "texel_density_px_per_m": 768,
            "variation": ["paint_chips", "edge_wear", "primer", "localized_rust"],
        },
        "tank_enamel_white": {
            "applies_to": ["tank_white"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.77, 0.79, 0.76],
            "metallic": 0.32,
            "roughness_range": [0.4, 0.7],
            "channels": ["base_color", "normal", "roughness", "metallic", "ambient_occlusion"],
            "texel_density_px_per_m": 512,
            "variation": ["weld_seams", "inspection_hatches", "rain_streaks", "dust_band", "minor_rust"],
        },
        "shipping_container_weathered": {
            "applies_to": ["container_blue", "container_orange", "container_red"],
            "workflow": "metallic_roughness",
            "base_color_source": "retain_layout_material_hue",
            "metallic": 0.55,
            "roughness_range": [0.45, 0.78],
            "channels": ["base_color", "normal", "roughness", "metallic", "ambient_occlusion"],
            "texel_density_px_per_m": 768,
            "variation": ["corrugation", "door_hardware", "scratches", "paint_fade", "surface_rust"],
        },
        "industrial_glass": {
            "applies_to": ["glass"],
            "workflow": "translucent_dielectric",
            "base_color_srgb": [0.18, 0.34, 0.4],
            "metallic": 0.0,
            "roughness_range": [0.12, 0.28],
            "transmission": 0.72,
            "ior": 1.52,
            "channels": ["base_color", "normal", "roughness", "opacity"],
            "texel_density_px_per_m": 512,
            "variation": ["dust", "water_spots", "frame_occlusion"],
        },
        "compacted_gravel": {
            "applies_to": ["gravel"],
            "workflow": "metallic_roughness",
            "base_color_srgb": [0.31, 0.27, 0.21],
            "metallic": 0.0,
            "roughness_range": [0.82, 0.98],
            "channels": ["base_color", "normal", "roughness", "ambient_occlusion", "height"],
            "texel_density_px_per_m": 512,
            "variation": ["aggregate_size", "wheel_ruts", "dust", "edge_scatter"],
        },
        "road_and_parking_paint": {
            "applies_to": ["road_marking", "parking_marking"],
            "workflow": "metallic_roughness",
            "base_color_source": "retain_layout_material_hue",
            "metallic": 0.0,
            "roughness_range": [0.55, 0.78],
            "channels": ["base_color", "normal", "roughness", "ambient_occlusion", "opacity"],
            "texel_density_px_per_m": 1024,
            "variation": ["traffic_wear", "edge_chipping", "rubber_transfer", "dirt_mask"],
        },
    }
    detail_targets: dict[str, object] = {
        "facades": {
            "scope": ["main_factory", "warehouse", "administration_office", "south_guardhouse"],
            "features": [
                "corrugated_panel_courses",
                "corner_flashings",
                "roof_parapets_and_gutters",
                "downpipes",
                "window_and_door_frames",
                "service_doors",
                "ventilation_louvres",
                "weathering_masks",
            ],
            "hero_distance_m": 35.0,
            "repeat_module_m": 1.0,
        },
        "loading": {
            "scope_semantic_classes": ["loading_dock", "loading_ramp"],
            "features": [
                "roller_shutter_doors",
                "dock_levellers",
                "rubber_bumpers",
                "wheel_guides",
                "bollards",
                "canopies",
                "safety_stripes",
                "pallet_clusters",
            ],
            "hero_distance_m": 22.0,
            "clear_route_margin_m": 1.2,
        },
        "gate": {
            "scope": ["gate_leaf_west_open", "gate_leaf_east_open", "south_guardhouse", "gate_canopy"],
            "features": [
                "sliding_gate_rails",
                "access_control_pedestals",
                "barrier_arms",
                "security_cameras",
                "intercom",
                "canopy_luminaires",
                "lane_delineators",
            ],
            "hero_distance_m": 18.0,
            "preserve_clear_width_m": 18.0,
        },
        "tanks": {
            "scope_semantic_classes": ["storage_tank", "containment_bund", "pipe_rack"],
            "features": [
                "weld_courses",
                "roof_rails",
                "access_ladders",
                "inspection_hatches",
                "valve_clusters",
                "pipe_elbows_and_flanges",
                "hazard_placards_without_branding",
                "localized_weathering",
            ],
            "hero_distance_m": 28.0,
            "bund_access_clearance_m": 1.4,
        },
        "utilities": {
            "scope": ["administration_office", "tank_pipe_rack"],
            "features": [
                "transformer_yard",
                "cable_trays",
                "electrical_cabinets",
                "hvac_units",
                "roof_vents",
                "hydrant_and_fire_hose_points",
                "utility_conduits",
            ],
            "hero_distance_m": 25.0,
        },
        "roads": {
            "scope_semantic_classes": ["road", "parking_area", "parking_space", "curb", "speed_bump"],
            "features": [
                "lane_edge_wear",
                "sealed_cracks",
                "patch_repairs",
                "oil_and_tire_marks",
                "drain_grates",
                "manhole_covers",
                "curb_staining",
                "low_profile_puddles_visual_only",
            ],
            "decal_density_per_100_square_m": [1.5, 3.0],
            "keep_lane_markings_readable": True,
        },
        "clutter": {
            "features": [
                "pallets",
                "stillages",
                "IBC_totes",
                "traffic_cones",
                "waste_bins",
                "cable_reels",
                "empty_drums",
                "wheel_chocks",
            ],
            "clusters": 34,
            "minimum_route_clearance_m": 1.2,
            "exclude_spawn_clearance_radius_m": 4.0,
        },
        "vegetation": {
            "features": ["mown_grass", "weedy_edges", "drainage_reeds", "sparse_temperate_trees"],
            "tree_count_range": [26, 38],
            "shrub_cluster_count_range": [18, 30],
            "grass_density_per_square_m": [0.35, 0.9],
            "exclude_engineered_surfaces": True,
            "minimum_road_edge_clearance_m": 0.6,
        },
        "lighting_support": {
            "features": ["street_lamp_heads", "wall_packs", "dock_lights", "gate_canopy_lights"],
            "fixture_instance_limit": 96,
            "emissive_only_beyond_m": 45.0,
        },
    }
    recipe = {
        "schema": "lingtu.sim.factory-park-realism-recipe.v1",
        "world_package": WORLD_PACKAGE,
        "profile": "industrial_realism_v2",
        "profile_description": "Contemporary maintained Chinese industrial campus with operational wear, restrained clutter, and robot-scale readability.",
        "seed": seed,
        "layout_digest": layout_digest,
        "inputs": {
            "expanded_layout": "sim/worlds/factory_park_hf/generated/expanded-layout.json",
            "semantic_entities": "sim/worlds/factory_park_hf/generated/semantic-entities.json",
            "terrain_obj": "sim/worlds/factory_park_hf/generated/terrain.obj",
            "mujoco_world": "sim/worlds/factory_park_hf/factory_park_hf.xml",
        },
        "authoring_contract": {
            "coordinate_frame": "mujoco_rh_z_up_m",
            "placement_source": "expanded_layout",
            "dressing_id_namespace": "factory_park_hf.realism",
            "randomness": "seeded_only",
            "stable_sort_key": "dressing_id",
            "dressing_defaults": {
                "classification": "VisualOnly",
                "collision": False,
                "physics_representation": "none",
                "cast_shadow": True,
                "receive_decals": True,
            },
            "preview_output_contract": {
                "recipe_field": "output_basename",
                "consumer_output_root": "consumer_owned_evidence_root",
                "rule": "Each Blender or Unreal consumer writes beneath its own output root and records the actual repository-relative or absolute artifact path in its evidence manifest.",
            },
        },
        "hard_rules": {
            "layout_unchanged": True,
            "mujoco_world_unchanged": True,
            "added_dressing_classification": "VisualOnly",
            "added_dressing_collision": False,
            "physics_authority": "mujoco",
            "allow_new_physics_authority": False,
        },
        "pbr_material_profiles": material_profiles,
        "detail_targets": detail_targets,
        "lighting": {
            "intent": "neutral_late_morning_operational_daylight",
            "sun": {
                "azimuth_deg": 138.0,
                "elevation_deg": 42.0,
                "illuminance_lux": 78_000,
                "angular_diameter_deg": 0.535,
                "color_temperature_k": 5_600,
            },
            "sky": {
                "cloud_cover": 0.28,
                "turbidity": 2.8,
                "aerial_haze": 0.08,
                "ground_albedo": 0.18,
            },
            "exposure": {
                "mode": "manual",
                "ev100": 14.0,
                "white_balance_k": 5_600,
                "highlight_rolloff": "filmic_medium_high",
            },
            "unreal": {
                "global_illumination": "Lumen",
                "reflections": "Lumen",
                "shadow_method": "VirtualShadowMaps",
                "skylight_realtime_capture": True,
                "fog": "ExponentialHeightFog",
            },
            "blender": {
                "renderer": "BLENDER_EEVEE_NEXT",
                "world_strength": 0.35,
                "sun_angle_deg": 0.535,
                "ambient_occlusion": True,
            },
        },
        "preview_targets": [
            {
                "id": "site_aerial",
                "purpose": "whole_site_composition_and_zone_readability",
                "camera_frame": "mujoco_rh_z_up_m",
                "position_m": [155.0, -185.0, 145.0],
                "look_at_m": [0.0, 0.0, 0.0],
                "lens_mm": 48.0,
                "resolution_px": [1920, 1080],
                "output_basename": "site-aerial-v2.png",
            },
            {
                "id": "south_gate_robot_eye",
                "purpose": "robot_scale_gate_and_road_readability",
                "camera_frame": "mujoco_rh_z_up_m",
                "position_m": [0.0, -79.0, 0.78],
                "look_at_m": [0.0, -50.0, 0.9],
                "lens_mm": 24.0,
                "resolution_px": [1920, 1080],
                "output_basename": "south-gate-robot-eye-v2.png",
            },
            {
                "id": "loading_dock_robot_eye",
                "purpose": "loading_detail_and_route_clearance",
                "camera_frame": "mujoco_rh_z_up_m",
                "position_m": [49.0, 4.0, 0.78],
                "look_at_m": [50.0, 19.0, 2.2],
                "lens_mm": 28.0,
                "resolution_px": [1920, 1080],
                "output_basename": "loading-dock-robot-eye-v2.png",
            },
            {
                "id": "tank_farm_inspection",
                "purpose": "tank_weathering_utilities_and_bund_access",
                "camera_frame": "mujoco_rh_z_up_m",
                "position_m": [89.0, -36.0, 1.55],
                "look_at_m": [65.0, -36.0, 3.2],
                "lens_mm": 35.0,
                "resolution_px": [1920, 1080],
                "output_basename": "tank-farm-inspection-v2.png",
            },
        ],
        "performance_budgets": {
            "reference_resolution_px": [1920, 1080],
            "target_frame_rate_fps": 60,
            "target_frame_time_ms": 16.67,
            "max_draw_calls": 1_800,
            "max_visible_triangles": 8_000_000,
            "max_non_nanite_triangles": 2_000_000,
            "max_texture_memory_mb": 2_048,
            "max_unique_4k_texture_sets": 18,
            "max_visual_only_instances": 5_000,
            "max_shadow_casting_local_lights": 24,
            "minimum_foliage_instance_batch": 32,
            "lod_policy": {
                "hero_assets": "LOD0_to_35m_then_automatic",
                "repeated_props": "instanced_static_mesh_or_hierarchical_instancing",
                "micro_detail_cull_distance_m": 55.0,
                "foliage_cull_distance_m": 120.0,
            },
        },
    }
    return _canonical_json(recipe)


def _manifest() -> bytes:
    return (
        b"schema: lingtu.sim.world-package.v1\n"
        b"id: factory_park_hf\n"
        b"version: 1.0.0\n"
        b"kind: world\n"
        b"description: Deterministic 220x180 m modern factory park for MuJoCo and Unreal.\n"
        b"physics:\n"
        b"  mjcf: ../../../worlds/factory_park_hf/factory_park_hf.xml\n"
        b"  global_policy:\n"
        b"    timestep_s: 0.002\n"
        b"    integrator: rk4\n"
        b"    solver: newton\n"
        b"    iterations: 100\n"
        b"    gravity_mps2: [0.0, 0.0, -9.81]\n"
        b"visual:\n"
        b"  binding: WorldVisual:FactoryParkHF\n"
        b"  level: /Game/RobotSim/Maps/FactoryPark_HF\n"
        b"entities:\n"
        b"  - source: ../../../worlds/factory_park_hf/generated/semantic-entities.json\n"
        b"    role: static_semantics\n"
    )


def _output_record(relative_path: Path, payload: bytes) -> dict[str, object]:
    return {
        "path": relative_path.as_posix(),
        "bytes": len(payload),
        "sha256": _sha256(payload),
    }


def _asset_set_digest(records: Sequence[Mapping[str, object]]) -> str:
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: str(item["path"]))
    ]
    return _sha256(_canonical_json(identity))


def _element_batch_records(
    batches: Sequence[CompiledElementBatch],
) -> list[dict[str, object]]:
    return [
        {
            "batch_id": batch.batch_id,
            "digest": batch.digest,
            "object_count": len(batch.objects),
        }
        for batch in batches
    ]


def _load_element_batch_input(
    source: Mapping[str, object] | Path | str,
    repo_root: Path,
) -> Mapping[str, object]:
    if isinstance(source, Mapping):
        return source
    source_path = Path(source)
    if not source_path.is_absolute():
        source_path = repo_root / source_path
    raw = json.loads(source_path.resolve().read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise TypeError(f"element batch {source_path} root must be an object")
    return raw


def _write_generated_outputs(
    repo_root: Path,
    generated: Mapping[Path, bytes],
    *,
    overwrite_existing: bool,
) -> None:
    conflicts: list[Path] = []
    ordered_outputs = sorted(generated.items(), key=lambda item: item[0].as_posix())
    for relative_path, payload in ordered_outputs:
        target = repo_root / relative_path
        try:
            target.resolve(strict=False).relative_to(repo_root)
        except ValueError as exc:
            raise ValueError(f"generated output escapes repo root: {relative_path}") from exc
        if target.is_symlink():
            raise FileExistsError(f"refusing to write generated output through symlink: {target}")
        if target.exists() and not target.is_file():
            raise FileExistsError(f"generated output target is not a file: {target}")
        if target.is_file() and target.read_bytes() != payload and not overwrite_existing:
            conflicts.append(relative_path)

    if conflicts:
        paths = ", ".join(path.as_posix() for path in conflicts[:5])
        suffix = "" if len(conflicts) <= 5 else f" (+{len(conflicts) - 5} more)"
        raise FileExistsError(
            "refusing to overwrite changed generated outputs; inspect the files and rerun with "
            f"--force-overwrite if replacement is intentional: {paths}{suffix}"
        )

    for relative_path, payload in ordered_outputs:
        target = repo_root / relative_path
        if target.is_file() and target.read_bytes() == payload:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)


def generate_factory_park_hf(
    repo_root: Path | str,
    *,
    seed: int = DEFAULT_SEED,
    spec: TerrainSpec = TerrainSpec(),
    element_batches: Sequence[Mapping[str, object] | Path | str] | None = None,
    overwrite_existing: bool = False,
) -> GeneratedWorld:
    """Generate every canonical FactoryPark_HF artifact below ``repo_root``."""

    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("seed must be an integer")
    if not isinstance(overwrite_existing, bool):
        raise TypeError("overwrite_existing must be a boolean")
    repo_root = Path(repo_root).resolve()
    package_root = repo_root / "sim" / "packages" / "worlds" / WORLD_ID
    world_root = repo_root / "sim" / "worlds" / WORLD_ID
    objects = _factory_objects()
    compiled_batches: list[CompiledElementBatch] = []
    element_layout: dict[str, object] = {
        "coordinate_system": {
            "frame": "mujoco_rh_z_up_m",
            "handedness": "right",
            "up_axis": "z",
            "linear_unit": "metre",
        },
        "site_boundary_m": {
            "minimum": [-spec.size_x_m / 2.0, -spec.size_y_m / 2.0, -spec.base_depth_m],
            "maximum": [spec.size_x_m / 2.0, spec.size_y_m / 2.0, 16.0],
        },
        "spawn": {
            "position_m": list(spec.spawn_position_m),
            "clearance_radius_m": 4.0,
        },
        "objects": objects,
    }
    batch_sources = CANONICAL_ELEMENT_BATCHES if element_batches is None else element_batches
    for batch_source in batch_sources:
        raw_batch = _load_element_batch_input(batch_source, repo_root)
        compiled = compile_element_batch(raw_batch, element_layout)
        compiled_batches.append(compiled)
        objects.extend(compiled.objects)
    samples = _height_samples(spec, seed, objects)
    coordinate_contract = _coordinate_contract(spec, samples)
    vertical_origin_m = float(coordinate_contract["vertical_origin_m"])
    layout, layout_digest = _expanded_layout(
        spec,
        seed,
        coordinate_contract,
        objects,
        compiled_batches,
    )

    png_path = Path("sim/worlds/factory_park_hf/generated/heightfield_r16.png")
    binary_path = Path("sim/worlds/factory_park_hf/generated/heightfield_f32.bin")
    obj_path = Path("sim/worlds/factory_park_hf/generated/terrain.obj")
    layout_path = Path("sim/worlds/factory_park_hf/generated/expanded-layout.json")
    semantics_path = Path("sim/worlds/factory_park_hf/generated/semantic-entities.json")
    site_plan_path = Path("sim/worlds/factory_park_hf/generated/site-plan.svg")
    asset_manifest_path = Path("sim/worlds/factory_park_hf/generated/asset-manifest.json")
    package_manifest_path = Path("sim/packages/worlds/factory_park_hf/world.package.yaml")
    mjcf_path = Path("sim/worlds/factory_park_hf/factory_park_hf.xml")
    recipe_path = Path("sim/packages/worlds/factory_park_hf/visual/ue_import.recipe.json")
    realism_recipe_path = Path("sim/packages/worlds/factory_park_hf/visual/realism.recipe.json")
    provenance_path = Path("sim/packages/worlds/factory_park_hf/provenance/factory-park.provenance.json")

    generated: dict[Path, bytes] = {
        package_manifest_path: _manifest(),
        mjcf_path: _mjcf(spec, vertical_origin_m, layout),
        png_path: _height_png(spec, samples),
        binary_path: _mujoco_hfield(spec, samples),
        obj_path: _terrain_obj(spec, samples, seed, vertical_origin_m),
        layout_path: _canonical_json(layout),
        semantics_path: _semantic_entities(layout),
        site_plan_path: _site_plan_svg(layout),
        realism_recipe_path: _realism_recipe(layout_digest, seed),
    }

    roles = {
        png_path: ("ue_heightfield", "16-bit grayscale PNG"),
        binary_path: ("mujoco_heightfield", "image/vnd.mujoco.hfield"),
        obj_path: ("unreal_terrain_mesh", "Wavefront OBJ in Unreal centimetres"),
        layout_path: ("expanded_layout", "canonical JSON"),
        semantics_path: ("semantic_entities", "canonical JSON"),
        site_plan_path: ("site_plan", "SVG"),
        realism_recipe_path: ("visual_realism_recipe", "canonical JSON"),
        mjcf_path: ("mujoco_world", "MJCF XML"),
    }
    source_records = []
    for path in sorted(roles, key=lambda item: item.as_posix()):
        role, file_format = roles[path]
        record = {"role": role, "format": file_format, **_output_record(path, generated[path])}
        if path == binary_path:
            record["row_order"] = "south_to_north_y_ascending"
        if path == obj_path:
            record.update(
                {
                    "sample_grid": [spec.width_px, spec.height_px],
                    "vertex_count": spec.width_px * spec.height_px,
                    "triangle_count": 2 * (spec.width_px - 1) * (spec.height_px - 1),
                }
            )
        source_records.append(record)

    element_batch_records = _element_batch_records(compiled_batches)
    asset_manifest = {
        "schema": "lingtu.sim.world-asset-manifest.v1",
        "world_package": WORLD_PACKAGE,
        "generator": {
            "module": "sim.tools.worlds.factory_park_hf.generate",
            "algorithm": GENERATOR_ALGORITHM,
            "seed": seed,
        },
        "coordinate_contract": coordinate_contract,
        "layout_digest": layout_digest,
        "assets": source_records,
        "asset_set_digest": _asset_set_digest(source_records),
    }
    if element_batch_records:
        asset_manifest["element_batches"] = element_batch_records
    generated[asset_manifest_path] = _canonical_json(asset_manifest)
    manifest_record = _output_record(asset_manifest_path, generated[asset_manifest_path])

    record_by_path = {str(record["path"]): record for record in source_records}
    recipe = {
        "schema": "lingtu.sim.unreal-world-import-recipe.v1",
        "status": "source_assets_only",
        "world_package": WORLD_PACKAGE,
        "binding": "WorldVisual:FactoryParkHF",
        "target_level": "/Game/RobotSim/Maps/FactoryPark_HF",
        "coordinate_contract": coordinate_contract,
        "layout_digest": layout_digest,
        "sources": {
            "asset_manifest": manifest_record,
            "expanded_layout": record_by_path[layout_path.as_posix()],
            "semantic_entities": record_by_path[semantics_path.as_posix()],
            "site_plan_svg": record_by_path[site_plan_path.as_posix()],
            "heightfield_png": record_by_path[png_path.as_posix()],
            "mujoco_heightfield": record_by_path[binary_path.as_posix()],
            "terrain_obj": record_by_path[obj_path.as_posix()],
            "mujoco_world": record_by_path[mjcf_path.as_posix()],
            "realism_recipe": record_by_path[realism_recipe_path.as_posix()],
        },
        "unreal_landscape_import": {
            "recommended_source": png_path.as_posix(),
            "dimensions_px": [spec.width_px, spec.height_px],
            "format": "16-bit grayscale PNG",
            "location_cm": coordinate_contract["unreal_landscape"]["location_cm"],  # type: ignore[index]
            "scale_xyz": coordinate_contract["unreal_landscape"]["scale_xyz"],  # type: ignore[index]
            "material_requirement": "Project-owned asphalt, concrete, gravel, soil, and drainage layers.",
        },
        "blender_authoring": {
            "expanded_layout": layout_path.as_posix(),
            "semantic_entities": semantics_path.as_posix(),
            "terrain_obj": obj_path.as_posix(),
            "layout_coordinates": "mujoco_rh_z_up_m",
            "terrain_coordinates": "unreal_lh_z_up_cm",
            "object_shapes": ["box", "cylinder"],
            "collision_authority_field": "collision",
            "visual_only_authority_field": "visual_only",
        },
        "unreal_layout_import": {
            "source": layout_path.as_posix(),
            "axis_mapping": ["x", "-y", "z"],
            "linear_scale_m_to_cm": 100.0,
            "object_count": len(layout["objects"]),
            "supported_shapes": ["box", "cylinder"],
            "semantic_metadata_fields": ["id", "semantic_class", "material", "collision", "visual_only"],
        },
        "physics_authority": {
            "backend": "mujoco",
            "mjcf": mjcf_path.as_posix(),
            "layout": layout_path.as_posix(),
            "rule": "Every collision=true layout object is emitted as a MuJoCo geom; terrain modifiers are baked into the shared heightfield.",
        },
        "limitations": [
            "No Unreal UMAP, production PBR texture set, foliage, lighting bake, Cook, or packaged build is generated.",
            "Dynamic pedestrians and forklifts are route intent only; a scenario executor must own their runtime motion.",
        ],
    }
    if element_batch_records:
        recipe["element_batches"] = element_batch_records
    generated[recipe_path] = _canonical_json(recipe)

    records = [
        _output_record(path, payload)
        for path, payload in sorted(generated.items(), key=lambda item: item[0].as_posix())
    ]
    asset_digest = _asset_set_digest(records)
    provenance = {
        "schema": "lingtu.sim.generated-world-provenance.v1",
        "asset_id": "factory_park_hf.world",
        "version": WORLD_VERSION,
        "generator": {
            "module": "sim.tools.worlds.factory_park_hf.generate",
            "algorithm": GENERATOR_ALGORITHM,
            "seed": seed,
        },
        "source": {
            "type": "procedural",
            "owner": "LingTu project",
            "license": "LicenseRef-LingTu-Project-Owned",
            "third_party_assets": [],
        },
        "coordinate_contract": coordinate_contract,
        "layout_digest": layout_digest,
        "object_count": len(layout["objects"]),
        "route_count": len(layout["routes"]),
        "checkpoint_count": len(layout["checkpoints"]),
        "outputs": records,
        "asset_set_digest": asset_digest,
        "reproducibility": {
            "deterministic": True,
            "standard_library_only": True,
            "command": f"python -m sim.tools.worlds.factory_park_hf.generate --repo-root . --seed {seed}",
        },
    }
    if element_batch_records:
        provenance["element_batches"] = element_batch_records
    generated[provenance_path] = _canonical_json(provenance)

    _write_generated_outputs(
        repo_root,
        generated,
        overwrite_existing=overwrite_existing,
    )

    return GeneratedWorld(
        repo_root=repo_root,
        package_root=package_root,
        world_root=world_root,
        layout_digest=layout_digest,
        asset_set_digest=asset_digest,
        element_batch_ids=tuple(batch.batch_id for batch in compiled_batches),
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate deterministic FactoryPark_HF assets.")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--element-batch",
        action="append",
        default=[],
        type=Path,
        help="Repeatable JSON element batch path, relative to --repo-root unless absolute.",
    )
    parser.add_argument(
        "--no-default-elements",
        action="store_true",
        help="Omit the canonical FactoryPark_HF element batch before applying --element-batch files.",
    )
    parser.add_argument(
        "--force-overwrite",
        action="store_true",
        help="Replace existing generated files whose bytes differ after explicit review.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Generate assets and print their deterministic identities."""

    args = _build_parser().parse_args(argv)
    requested_batches: Sequence[Path] | None
    if args.no_default_elements:
        requested_batches = args.element_batch
    elif args.element_batch:
        requested_batches = (*CANONICAL_ELEMENT_BATCHES, *args.element_batch)
    else:
        requested_batches = None
    generated = generate_factory_park_hf(
        args.repo_root,
        seed=args.seed,
        element_batches=requested_batches,
        overwrite_existing=args.force_overwrite,
    )
    print(
        json.dumps(
            {
                "asset_set_digest": generated.asset_set_digest,
                "element_batches": list(generated.element_batch_ids),
                "layout_digest": generated.layout_digest,
                "package_root": str(generated.package_root),
                "world_root": str(generated.world_root),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
