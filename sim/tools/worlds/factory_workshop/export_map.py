"""Sample the compiled workshop's solid geometry into a same-source PCD.

This is an offline synthetic geometry map, not SLAM or live sensor evidence.
Only contact-enabled geometry is sampled, regardless of its display group.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[4] / "sim/packages/worlds/factory_workshop"
DEFAULT_WORLD_PACKAGE = PACKAGE / "2.0.0/world.package.yaml"


def axis_samples(half_size: float, spacing: float):
    return np.linspace(-half_size, half_size, max(2, math.ceil(2 * half_size / spacing) + 1))


def sample_solids(model, data, spacing=0.05):
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be positive and finite")
    for gid in range(model.ngeom):
        if not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
            continue
        size = model.geom_size[gid]
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        faces = []
        if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
            for fixed in range(3):
                if name == "fw_floor" and fixed != 2:
                    continue
                u, v = [i for i in range(3) if i != fixed]
                us, vs = axis_samples(size[u], spacing), axis_samples(size[v], spacing)
                rows = max(1, 65536 // len(us))
                for begin in range(0, len(vs), rows):
                    uu, vv = np.meshgrid(us, vs[begin:begin + rows])
                    for sign in (1,) if name == "fw_floor" else (-1, 1):
                        p = np.zeros((uu.size, 3))
                        p[:, fixed], p[:, u], p[:, v] = sign * size[fixed], uu.ravel(), vv.ravel()
                        yield (p @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]).astype("<f4")
            continue
        elif model.geom_type[gid] in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
            radius, half_height = size[:2]
            angle = np.linspace(0, 2 * np.pi, max(12, math.ceil(2 * np.pi * radius / spacing)), endpoint=False)
            aa, zz = np.meshgrid(angle, axis_samples(half_height, spacing))
            faces.append(np.column_stack((radius * np.cos(aa.ravel()), radius * np.sin(aa.ravel()), zz.ravel())))
            if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_CAPSULE:
                latitude = np.linspace(0, np.pi / 2, max(3, math.ceil(np.pi * radius / (2 * spacing)) + 1))
                aa, pp = np.meshgrid(angle, latitude)
                for sign in (-1, 1):
                    faces.append(
                        np.column_stack(
                            (
                                radius * np.cos(pp.ravel()) * np.cos(aa.ravel()),
                                radius * np.cos(pp.ravel()) * np.sin(aa.ravel()),
                                sign * (half_height + radius * np.sin(pp.ravel())),
                            )
                        )
                    )
            else:
                xx, yy = np.meshgrid(axis_samples(radius, spacing), axis_samples(radius, spacing))
                keep = xx * xx + yy * yy <= radius * radius
                for sign in (-1, 1):
                    faces.append(np.column_stack((xx[keep], yy[keep], np.full(int(keep.sum()), sign * half_height))))
        else:
            raise ValueError(f"unsupported solid in workshop map export: {name}")
        for face in faces:
            if len(face):
                yield (face @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]).astype("<f4")


def _world_package_source(world_package: Path | None) -> tuple[str, Path]:
    manifest_path = (world_package or DEFAULT_WORLD_PACKAGE).resolve()
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    world = f"{manifest['id']}@{manifest['version']}"
    scene = (manifest_path.parent / manifest["physics"]["mjcf"]).resolve()
    return world, scene


def export_map(output_dir: Path, spacing=0.05, world_package: Path | None = None, resolution=0.1):
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be positive and finite")
    world, scene = _world_package_source(world_package)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    output_dir.mkdir(parents=True, exist_ok=True)
    def header(count):
        # Fixed-width counts allow streaming the payload before its size is known.
        return (
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            f"COUNT 1 1 1\nWIDTH {count:20d}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {count:20d}\nDATA binary\n"
        ).encode("ascii")

    count = 0
    minimum, maximum = np.full(3, np.inf), np.full(3, -np.inf)
    pending = output_dir / "map.pcd.partial"
    with pending.open("wb") as stream:
        stream.write(header(0))
        for points in sample_solids(model, data, spacing):
            # Keep an original sample for every occupied OctoMap voxel. This
            # removes dense duplicate hits without enlarging stair fronts.
            keys = np.floor(points.astype(np.float64) / resolution).astype(np.int64)
            _, selected = np.unique(keys, axis=0, return_index=True)
            points = points[np.sort(selected)]
            stream.write(points.tobytes())
            count += len(points)
            minimum, maximum = np.minimum(minimum, points.min(axis=0)), np.maximum(maximum, points.max(axis=0))
        if count == 0:
            raise ValueError("world has no sampled contact geometry")
        stream.seek(0)
        stream.write(header(count))
    pending.replace(output_dir / "map.pcd")
    provenance = {
        "world": world,
        "source_scene": str(scene),
        "frame": "map",
        "data_source": "synthetic_mujoco_collision_geometry",
        "slam": False,
        "spacing_m": spacing,
        "voxel_resolution_m": resolution,
        "point_count": count,
        "bounds_min_m": minimum.tolist(),
        "bounds_max_m": maximum.tolist(),
    }
    (output_dir / "source.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--spacing", type=float, default=0.05)
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--world-package", type=Path, default=None)
    args = parser.parse_args()
    print(json.dumps(export_map(args.output_dir, args.spacing, args.world_package, args.resolution), indent=2))


if __name__ == "__main__":
    main()
