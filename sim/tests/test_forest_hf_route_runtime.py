# ruff: noqa: S101

"""Bounded MuJoCo route-runtime smoke for the deterministic Forest_HF world.

This is deliberately not a controller, gait, or sensor end-to-end test.  It
qualifies the static WorldPackage geometry and a constrained vertical contact
probe against the declared primary route.
"""

from __future__ import annotations

import array
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass
from itertools import combinations, pairwise
from pathlib import Path
from typing import Any, Mapping, Sequence
from xml.etree import ElementTree

import numpy as np
import pytest
import yaml

from sim.catalog.resolver import CatalogResolver
from sim.tools.worlds.forest_hf.generate import TerrainSpec, generate_forest_hf
from sim.tools.worlds.forest_hf.materialize_proxies import (
    materialize_worldpackage_collision_proxies,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_PACKAGE_ENV = "LINGTU_FOREST_HF_PRODUCTION_PACKAGE"
PRIMARY_ROUTE_ID = "forest.route.primary_loop.v1"
ROUTE_SAMPLE_SPACING_M = 10.0
VALIDATION_RESOLUTION_PX = 513
VALIDATION_HEIGHT_TOLERANCE_M = 0.10
PRODUCTION_HEIGHT_TOLERANCE_M = 0.01
PROBE_HALF_HEIGHT_M = 0.10
PROBE_DROP_M = 0.35
PROBE_SETTLE_STEPS = 3_000


@dataclass(frozen=True)
class _RawHeightfield:
    samples: array.array[int]
    width: int
    height: int
    extent_x_m: float
    extent_y_m: float
    elevation_min_m: float
    elevation_max_m: float

    def sample(self, x_m: float, y_m: float) -> float:
        column = min(
            self.width - 1.0,
            max(
                0.0,
                (x_m + self.extent_x_m)
                * (self.width - 1)
                / (2.0 * self.extent_x_m),
            ),
        )
        row = min(
            self.height - 1.0,
            max(
                0.0,
                (self.extent_y_m - y_m)
                * (self.height - 1)
                / (2.0 * self.extent_y_m),
            ),
        )
        x0, y0 = int(math.floor(column)), int(math.floor(row))
        x1, y1 = min(self.width - 1, x0 + 1), min(self.height - 1, y0 + 1)
        tx, ty = column - x0, row - y0

        def elevation(ix: int, iy: int) -> float:
            normalized = self.samples[iy * self.width + ix] / 65_535.0
            return self.elevation_min_m + normalized * (
                self.elevation_max_m - self.elevation_min_m
            )

        north = elevation(x0, y0) * (1.0 - tx) + elevation(x1, y0) * tx
        south = elevation(x0, y1) * (1.0 - tx) + elevation(x1, y1) * tx
        return north * (1.0 - ty) + south * ty


@dataclass(frozen=True)
class _RuntimePackage:
    root: Path
    routes: dict[str, Any]
    proxies: tuple[dict[str, Any], ...]
    heightfield: _RawHeightfield
    validation_resolution: bool


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_runtime_package(root: Path) -> _RuntimePackage:
    manifest = _load_json(root / "generated" / "asset-manifest.json")
    canonical = manifest["canonical_source"]
    recipe = _load_json(root / "terrain.recipe.json")
    dimensions = canonical["dimensions_px"]
    width, height = int(dimensions[0]), int(dimensions[1])
    payload = (root / canonical["path"]).read_bytes()
    assert len(payload) == width * height * 2
    samples: array.array[int] = array.array("H")
    samples.frombytes(payload)
    if sys.byteorder != "little":  # pragma: no cover - supported hosts are little-endian.
        samples.byteswap()
    extent_x_m, extent_y_m = (float(value) / 2.0 for value in canonical["extent_m"])
    elevation_min_m, elevation_max_m = (
        float(value) for value in recipe["elevation_range_m"]
    )
    proxy_manifest = _load_json(
        root / "generated" / "forest_collision_proxies.manifest.json"
    )
    return _RuntimePackage(
        root=root,
        routes=_load_json(root / "routes" / "forest.routes.json"),
        proxies=tuple(proxy_manifest["proxies"]),
        heightfield=_RawHeightfield(
            samples=samples,
            width=width,
            height=height,
            extent_x_m=extent_x_m,
            extent_y_m=extent_y_m,
            elevation_min_m=elevation_min_m,
            elevation_max_m=elevation_max_m,
        ),
        validation_resolution=manifest["validation_resolution"] is True,
    )


def _robot_footprint_radius_m() -> float:
    config = yaml.safe_load(
        (
            REPO_ROOT
            / "config"
            / "robots"
            / "unitree"
            / "go2"
            / "robot.yaml"
        ).read_text()
    )
    geometry = config["geometry"]
    return math.hypot(
        float(geometry["vehicle_length"]) / 2.0,
        float(geometry["vehicle_width"]) / 2.0,
    )


def _primary_route(routes: Mapping[str, Any]) -> dict[str, Any]:
    matches = [route for route in routes["routes"] if route["stable_id"] == PRIMARY_ROUTE_ID]
    assert len(matches) == 1
    return matches[0]


def _sample_polyline(
    points: Sequence[Sequence[float]], spacing_m: float
) -> tuple[tuple[float, float], ...]:
    samples: list[tuple[float, float]] = []
    for start, end in pairwise(points):
        segment_length = math.dist(start, end)
        divisions = max(1, math.ceil(segment_length / spacing_m))
        for index in range(divisions):
            ratio = index / divisions
            samples.append(
                (
                    float(start[0]) + (float(end[0]) - float(start[0])) * ratio,
                    float(start[1]) + (float(end[1]) - float(start[1])) * ratio,
                )
            )
    samples.append((float(points[-1][0]), float(points[-1][1])))
    return tuple(samples)


def _route_samples(runtime: _RuntimePackage) -> tuple[tuple[float, float], ...]:
    route = _primary_route(runtime.routes)
    points = route["centerline_xy_m"]
    assert points[0] == points[-1]
    route_length_m = sum(math.dist(start, end) for start, end in pairwise(points))
    assert route_length_m >= 2_000.0
    native_spacing_m = min(
        2.0 * runtime.heightfield.extent_x_m / (runtime.heightfield.width - 1),
        2.0 * runtime.heightfield.extent_y_m / (runtime.heightfield.height - 1),
    )
    return _sample_polyline(points, min(ROUTE_SAMPLE_SPACING_M, native_spacing_m))


def _geom_name(mujoco: Any, model: Any, geom_id: int) -> str | None:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def _raycast_terrain_height(
    mujoco: Any, model: Any, data: Any, x_m: float, y_m: float
) -> tuple[float, str | None]:
    geom_id = np.array([-1], dtype=np.int32)
    terrain_group_only = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    origin_z_m = 120.0
    distance_m = mujoco.mj_ray(
        model,
        data,
        np.array([x_m, y_m, origin_z_m], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
        terrain_group_only,
        1,
        -1,
        geom_id,
    )
    assert math.isfinite(distance_m) and distance_m > 0.0
    assert int(geom_id[0]) >= 0
    return origin_z_m - float(distance_m), _geom_name(
        mujoco, model, int(geom_id[0])
    )


def _distance_to_segment_m(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> float:
    dx, dy = float(end[0]) - float(start[0]), float(end[1]) - float(start[1])
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.dist(point, start)
    ratio = (
        (float(point[0]) - float(start[0])) * dx
        + (float(point[1]) - float(start[1])) * dy
    ) / denominator
    ratio = min(1.0, max(0.0, ratio))
    return math.dist(
        point,
        (float(start[0]) + ratio * dx, float(start[1]) + ratio * dy),
    )


def _minimum_route_proxy_margin_m(
    centerline: Sequence[Sequence[float]],
    proxies: Sequence[Mapping[str, Any]],
    robot_radius_m: float,
) -> float:
    return min(
        min(
            _distance_to_segment_m(proxy["position_m"][:2], start, end)
            for start, end in pairwise(centerline)
        )
        - robot_radius_m
        - float(proxy["footprint_radius_m"])
        for proxy in proxies
    )


def _minimum_proxy_pair_margin_m(proxies: Sequence[Mapping[str, Any]]) -> float:
    return min(
        math.dist(first["position_m"][:2], second["position_m"][:2])
        - float(first["footprint_radius_m"])
        - float(second["footprint_radius_m"])
        for first, second in combinations(proxies, 2)
    )


def _compiled_proxy_contract(
    mujoco: Any,
    model: Any,
    data: Any,
    proxies: Sequence[Mapping[str, Any]],
) -> tuple[dict[str, Any], ...]:
    compiled: list[dict[str, Any]] = []
    expected_names = {str(proxy["stable_id"]) for proxy in proxies}
    actual_names = {
        _geom_name(mujoco, model, geom_id)
        for geom_id in range(model.ngeom)
        if _geom_name(mujoco, model, geom_id) != "forest_terrain"
    }
    assert actual_names == expected_names
    for proxy in proxies:
        stable_id = str(proxy["stable_id"])
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, stable_id)
        assert geom_id >= 0
        expected_geom_type = (
            mujoco.mjtGeom.mjGEOM_CYLINDER
            if proxy["kind"] == "tree"
            else mujoco.mjtGeom.mjGEOM_ELLIPSOID
        )
        assert int(model.geom_type[geom_id]) == int(expected_geom_type)
        compiled_position = tuple(float(value) for value in data.geom_xpos[geom_id])
        compiled_rotation = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
        assert compiled_rotation == pytest.approx(np.eye(3), abs=1e-9)
        expected_size = tuple(float(value) for value in proxy["size_m"])
        compiled_size = tuple(
            float(value) for value in model.geom_size[geom_id][: len(expected_size)]
        )
        expected_position = tuple(float(value) for value in proxy["position_m"])
        assert compiled_position == pytest.approx(expected_position, abs=1e-6)
        assert compiled_size == pytest.approx(expected_size, abs=1e-6)
        assert int(model.geom_contype[geom_id]) == 1
        assert int(model.geom_conaffinity[geom_id]) == 1
        footprint_radius_m = (
            compiled_size[0]
            if proxy["kind"] == "tree"
            else max(compiled_size[:2])
        )
        compiled.append(
            {
                "stable_id": stable_id,
                "kind": proxy["kind"],
                "position_m": compiled_position,
                "size_m": compiled_size,
                "footprint_radius_m": footprint_radius_m,
            }
        )
    return tuple(compiled)


def _assert_production_artifact_bindings(runtime: _RuntimePackage) -> None:
    evidence_path = (
        runtime.root / "generated" / "forest_collision_promotion.evidence.json"
    )
    evidence = _load_json(evidence_path)
    content_digest = evidence.pop("content_digest")
    canonical = (
        json.dumps(
            evidence,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode()
    assert content_digest == hashlib.sha256(canonical).hexdigest()
    assert evidence["qualification"]["qualified_for_worldpackage_promotion"] is True
    for artifact in evidence["artifacts"].values():
        path = runtime.root / artifact["path"]
        assert path.is_file()
        assert _sha256_file(path) == artifact["sha256"]
    assert evidence["promotion_candidate"]["physics_mjcf"] == (
        "physics/forest_hf.with-collision-proxies.xml"
    )
    resolver = CatalogResolver(runtime.root, (runtime.root,))
    record = resolver.find_package("forest_hf@2.0.0", kind="world")
    resolver._validate_world_package_content(record)
    assert resolver._package_lock(record)["package_artifact_sha256"]
    assert record.data["physics"]["mjcf"] == (
        "physics/forest_hf.with-collision-proxies.xml"
    )


def _assert_static_route_runtime(
    runtime: _RuntimePackage,
    *,
    height_tolerance_m: float,
    require_proxy_ground_alignment: bool,
) -> tuple[tuple[float, float], ...]:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(
        str(runtime.root / "physics" / "forest_hf.with-collision-proxies.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    assert model.ngeom == len(runtime.proxies) + 1
    assert _geom_name(mujoco, model, 0) == "forest_terrain"
    route = _primary_route(runtime.routes)
    route_samples = _route_samples(runtime)
    robot_radius_m = _robot_footprint_radius_m()
    compiled_proxies = _compiled_proxy_contract(mujoco, model, data, runtime.proxies)
    assert _minimum_route_proxy_margin_m(
        route["centerline_xy_m"], compiled_proxies, robot_radius_m
    ) > 0.0
    assert _minimum_proxy_pair_margin_m(compiled_proxies) > 0.0

    marker_points = {
        tuple(float(value) for value in runtime.routes[name]["position_xy_m"])
        for name in ("spawn", "goal")
    }
    assert marker_points <= set(route_samples)
    assert len(marker_points) == 2
    assert float(route["navigation_limits"]["minimum_drivable_width_m"]) > (
        2.0 * robot_radius_m
    )
    for marker_name, radius_field in (
        ("spawn", "minimum_clear_radius_m"),
        ("goal", "acceptance_radius_m"),
    ):
        marker = runtime.routes[marker_name]
        marker_xy = tuple(float(value) for value in marker["position_xy_m"])
        assert marker["route_id"] == PRIMARY_ROUTE_ID
        assert abs(marker_xy[0]) <= runtime.heightfield.extent_x_m
        assert abs(marker_xy[1]) <= runtime.heightfield.extent_y_m
        assert math.isfinite(runtime.heightfield.sample(*marker_xy))
        marker_margin_m = min(
            math.dist(marker_xy, proxy["position_m"][:2])
            - float(marker[radius_field])
            - robot_radius_m
            - float(proxy["footprint_radius_m"])
            for proxy in compiled_proxies
        )
        assert marker_margin_m > 0.0

    first_pass: list[float] = []
    for x_m, y_m in route_samples:
        ground_z_m, hit_name = _raycast_terrain_height(
            mujoco, model, data, x_m, y_m
        )
        assert hit_name == "forest_terrain"
        assert ground_z_m == pytest.approx(
            runtime.heightfield.sample(x_m, y_m), abs=height_tolerance_m
        )
        first_pass.append(ground_z_m)

    for _ in range(10):
        mujoco.mj_step(model, data)
    second_pass = [
        _raycast_terrain_height(mujoco, model, data, x_m, y_m)[0]
        for x_m, y_m in route_samples
    ]
    assert second_pass == pytest.approx(first_pass, abs=1e-9)

    limits = route["navigation_limits"]
    maximum_grade = max(
        abs(end_z - start_z) / math.dist(start_xy, end_xy)
        for start_xy, end_xy, start_z, end_z in zip(
            route_samples,
            route_samples[1:],
            first_pass,
            first_pass[1:],
        )
    )
    maximum_step_m = max(
        abs(end_z - start_z) for start_z, end_z in pairwise(first_pass)
    )
    cross_slopes: list[float] = []
    unique_route_samples = route_samples[:-1]
    for index, (x_m, y_m) in enumerate(unique_route_samples):
        previous = unique_route_samples[index - 1]
        following = unique_route_samples[(index + 1) % len(unique_route_samples)]
        tangent_x = following[0] - previous[0]
        tangent_y = following[1] - previous[1]
        tangent_norm = math.hypot(tangent_x, tangent_y)
        assert tangent_norm > 0.0
        normal_x = -tangent_y / tangent_norm
        normal_y = tangent_x / tangent_norm
        left = (
            x_m + normal_x * robot_radius_m,
            y_m + normal_y * robot_radius_m,
        )
        right = (
            x_m - normal_x * robot_radius_m,
            y_m - normal_y * robot_radius_m,
        )
        left_z, left_name = _raycast_terrain_height(mujoco, model, data, *left)
        right_z, right_name = _raycast_terrain_height(mujoco, model, data, *right)
        assert left_name == right_name == "forest_terrain"
        assert left_z == pytest.approx(
            runtime.heightfield.sample(*left), abs=height_tolerance_m
        )
        assert right_z == pytest.approx(
            runtime.heightfield.sample(*right), abs=height_tolerance_m
        )
        cross_slopes.append(abs(left_z - right_z) / (2.0 * robot_radius_m))
    assert maximum_grade <= float(limits["maximum_grade"])
    assert maximum_step_m <= float(limits["maximum_step_m"])
    assert max(cross_slopes) <= float(limits["maximum_cross_slope"])

    if require_proxy_ground_alignment:
        for proxy in compiled_proxies:
            x_m, y_m, center_z_m = (
                float(value) for value in proxy["position_m"]
            )
            half_height_m = float(
                proxy["size_m"][1] if proxy["kind"] == "tree" else proxy["size_m"][2]
            )
            terrain_z_m, hit_name = _raycast_terrain_height(
                mujoco, model, data, x_m, y_m
            )
            assert hit_name == "forest_terrain"
            assert center_z_m - half_height_m == pytest.approx(
                terrain_z_m, abs=height_tolerance_m
            )
    return route_samples


def _write_contact_probe_world(
    runtime: _RuntimePackage,
    probe_points: Sequence[tuple[float, float]],
    ground_heights_m: Sequence[float],
    output: Path,
) -> Path:
    source = runtime.root / "physics" / "forest_hf.with-collision-proxies.xml"
    root = ElementTree.parse(source).getroot()  # noqa: S314 - generated local MJCF.
    hfield = root.find("./asset/hfield[@name='forest_terrain']")
    assert hfield is not None
    hfield.set(
        "file",
        str((runtime.root / "generated" / "heightfield_f32.bin").resolve()),
    )
    worldbody = root.find("./worldbody")
    assert worldbody is not None
    footprint_radius_m = _robot_footprint_radius_m()
    for index, ((x_m, y_m), ground_z_m) in enumerate(
        zip(probe_points, ground_heights_m, strict=True)
    ):
        body = ElementTree.SubElement(
            worldbody,
            "body",
            {
                "name": f"forest_route_probe_{index}",
                "pos": (
                    f"{x_m:.9f} {y_m:.9f} "
                    f"{ground_z_m + PROBE_HALF_HEIGHT_M + PROBE_DROP_M:.9f}"
                ),
            },
        )
        ElementTree.SubElement(
            body,
            "joint",
            {
                "name": f"forest_route_probe_slide_{index}",
                "type": "slide",
                "axis": "0 0 1",
                "damping": "4",
                "armature": "0.01",
            },
        )
        ElementTree.SubElement(
            body,
            "geom",
            {
                "name": f"forest_route_probe_geom_{index}",
                "type": "cylinder",
                "size": f"{footprint_radius_m:.9f} {PROBE_HALF_HEIGHT_M:.9f}",
                "mass": "30",
                "contype": "1",
                "conaffinity": "1",
                "condim": "3",
                "friction": "0.9 0.02 0.001",
            },
        )
    ElementTree.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return output


def _assert_contact_probe_runtime(
    runtime: _RuntimePackage,
    route_samples: Sequence[tuple[float, float]],
    output_root: Path,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    base_model = mujoco.MjModel.from_xml_path(
        str(runtime.root / "physics" / "forest_hf.with-collision-proxies.xml")
    )
    base_data = mujoco.MjData(base_model)
    mujoco.mj_forward(base_model, base_data)
    probe_points = (
        tuple(float(value) for value in runtime.routes["spawn"]["position_xy_m"]),
        route_samples[len(route_samples) // 2],
        tuple(float(value) for value in runtime.routes["goal"]["position_xy_m"]),
    )
    ground_heights_m = tuple(
        _raycast_terrain_height(mujoco, base_model, base_data, x_m, y_m)[0]
        for x_m, y_m in probe_points
    )
    probe_world = _write_contact_probe_world(
        runtime,
        probe_points,
        ground_heights_m,
        output_root / "forest_hf.route-probe.xml",
    )
    model = mujoco.MjModel.from_xml_path(str(probe_world))
    data = mujoco.MjData(model)
    contact_steps = {index: 0 for index in range(len(probe_points))}
    maximum_normal_force_n = {index: 0.0 for index in range(len(probe_points))}
    maximum_vertical_speed_mps = {index: 0.0 for index in range(len(probe_points))}
    maximum_z_error_m = {index: 0.0 for index in range(len(probe_points))}

    for step in range(PROBE_SETTLE_STEPS):
        mujoco.mj_step(model, data)
        if step < PROBE_SETTLE_STEPS - 250:
            continue
        for probe_index, expected_ground_z_m in enumerate(ground_heights_m):
            body_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                f"forest_route_probe_{probe_index}",
            )
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"forest_route_probe_slide_{probe_index}",
            )
            dof_id = int(model.jnt_dofadr[joint_id])
            maximum_vertical_speed_mps[probe_index] = max(
                maximum_vertical_speed_mps[probe_index],
                abs(float(data.qvel[dof_id])),
            )
            maximum_z_error_m[probe_index] = max(
                maximum_z_error_m[probe_index],
                abs(
                    float(data.xpos[body_id][2])
                    - expected_ground_z_m
                    - PROBE_HALF_HEIGHT_M
                ),
            )
        contacted_this_step: set[int] = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            names = {
                _geom_name(mujoco, model, int(contact.geom1)),
                _geom_name(mujoco, model, int(contact.geom2)),
            }
            for probe_index in range(len(probe_points)):
                probe_name = f"forest_route_probe_geom_{probe_index}"
                if probe_name not in names:
                    continue
                assert names == {"forest_terrain", probe_name}
                contacted_this_step.add(probe_index)
                force = np.zeros(6, dtype=np.float64)
                mujoco.mj_contactForce(model, data, contact_index, force)
                assert np.isfinite(force).all()
                maximum_normal_force_n[probe_index] = max(
                    maximum_normal_force_n[probe_index], abs(float(force[0]))
                )
        for probe_index in contacted_this_step:
            contact_steps[probe_index] += 1

    for index, expected_ground_z_m in enumerate(ground_heights_m):
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"forest_route_probe_{index}"
        )
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"forest_route_probe_slide_{index}"
        )
        dof_id = int(model.jnt_dofadr[joint_id])
        assert float(data.xpos[body_id][2]) == pytest.approx(
            expected_ground_z_m + PROBE_HALF_HEIGHT_M, abs=0.02
        )
        assert abs(float(data.qvel[dof_id])) < 0.02
        assert contact_steps[index] == 250
        assert maximum_normal_force_n[index] > 0.0
        assert maximum_vertical_speed_mps[index] < 0.02
        assert maximum_z_error_m[index] < 0.02


@pytest.fixture(scope="module")
def validation_runtime_package(tmp_path_factory: pytest.TempPathFactory) -> _RuntimePackage:
    root = tmp_path_factory.mktemp("forest-hf-route-runtime")
    generated = generate_forest_hf(
        root, spec=TerrainSpec(resolution_px=VALIDATION_RESOLUTION_PX)
    )
    result = materialize_worldpackage_collision_proxies(generated.package_root)
    assert result.evidence["mujoco_compile"]["qualified"] is True
    runtime = _load_runtime_package(generated.package_root)
    assert runtime.validation_resolution is True
    return runtime


def test_validation_world_traces_the_full_route_without_proxy_intersection(
    validation_runtime_package: _RuntimePackage,
) -> None:
    _assert_static_route_runtime(
        validation_runtime_package,
        height_tolerance_m=VALIDATION_HEIGHT_TOLERANCE_M,
        require_proxy_ground_alignment=False,
    )


def test_validation_world_has_stable_ground_contacts_at_route_markers(
    validation_runtime_package: _RuntimePackage,
    tmp_path: Path,
) -> None:
    route_samples = _route_samples(validation_runtime_package)
    _assert_contact_probe_runtime(validation_runtime_package, route_samples, tmp_path)


def test_production_world_route_runtime_when_explicitly_selected(
    tmp_path: Path,
) -> None:
    package_value = os.environ.get(PRODUCTION_PACKAGE_ENV)
    if not package_value:
        pytest.skip(f"set {PRODUCTION_PACKAGE_ENV} to run the production 4033 route gate")
    runtime = _load_runtime_package(Path(package_value).resolve())
    assert runtime.validation_resolution is False
    assert (runtime.heightfield.width, runtime.heightfield.height) == (4033, 4033)
    _assert_production_artifact_bindings(runtime)
    route_samples = _assert_static_route_runtime(
        runtime,
        height_tolerance_m=PRODUCTION_HEIGHT_TOLERANCE_M,
        require_proxy_ground_alignment=True,
    )
    _assert_contact_probe_runtime(runtime, route_samples, tmp_path)
