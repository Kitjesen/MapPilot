#!/usr/bin/env python3
"""Build a geometry-semantic scene view from a binary PCD.

The output keeps several useful layers instead of collapsing the cloud into a
single height map: vertical structure groups, connected support-surface
regions, dominant height levels, collision geometry, and unknown space. It
does not invent object names such as ``desk`` or ``box`` without RGB/RGB-D
evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def read_pcd(path: Path) -> np.ndarray:
    """Read x/y/z from the binary PCD layout used by the fixture."""
    raw = path.read_bytes()
    marker = b"DATA binary\n"
    try:
        offset = raw.index(marker) + len(marker)
    except ValueError as exc:
        raise ValueError(f"{path} is not a binary PCD") from exc

    header = raw[:offset].decode("ascii", errors="ignore")
    points_line = next((line for line in header.splitlines() if line.startswith("POINTS ")), "")
    if not points_line:
        raise ValueError(f"{path} has no POINTS declaration")
    count = int(points_line.split()[1])
    # x, y, z are float32; the fixture's fourth field is a packed 4-byte value.
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("extra", "u1", 4)])
    payload = raw[offset : offset + count * dtype.itemsize]
    if len(payload) != count * dtype.itemsize:
        raise ValueError(f"{path} ended before the declared point count")
    records = np.frombuffer(payload, dtype=dtype, count=count)
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float32, copy=False)
    return xyz[np.isfinite(xyz).all(axis=1)]


def dominant_levels(z: np.ndarray, *, bin_size: float = 0.1, min_separation: float = 0.8) -> list[float]:
    """Find broad horizontal surface levels from the height histogram."""
    lower = float(np.floor(float(z.min()) / bin_size) * bin_size)
    upper = float(np.ceil(float(z.max()) / bin_size) * bin_size + bin_size)
    edges = np.arange(lower, upper + bin_size * 0.5, bin_size)
    counts, _ = np.histogram(z, bins=edges)
    smoothed = np.convolve(counts.astype(np.float64), np.ones(5) / 5.0, mode="same")
    threshold = max(float(np.percentile(smoothed, 92.0)), float(smoothed.max()) * 0.18)
    candidates: list[tuple[float, float]] = []
    for index in range(1, len(smoothed) - 1):
        if smoothed[index] >= threshold and smoothed[index] >= smoothed[index - 1] and smoothed[index] >= smoothed[index + 1]:
            candidates.append(((edges[index] + edges[index + 1]) / 2.0, float(smoothed[index])))

    candidates.sort(key=lambda item: item[1], reverse=True)
    selected: list[float] = []
    for level, _ in candidates:
        if all(abs(level - other) >= min_separation for other in selected):
            selected.append(level)
    return sorted(selected)


def connected_components(mask: np.ndarray, *, min_cells: int = 4) -> list[list[tuple[int, int]]]:
    """Return 8-connected components from a small boolean grid."""
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: list[list[tuple[int, int]]] = []
    for y, x in zip(*np.where(mask)):
        if visited[y, x]:
            continue
        queue = [(int(y), int(x))]
        visited[y, x] = True
        component: list[tuple[int, int]] = []
        while queue:
            current_y, current_x = queue.pop()
            component.append((current_y, current_x))
            for delta_y in (-1, 0, 1):
                for delta_x in (-1, 0, 1):
                    next_y = current_y + delta_y
                    next_x = current_x + delta_x
                    if (
                        0 <= next_y < height
                        and 0 <= next_x < width
                        and mask[next_y, next_x]
                        and not visited[next_y, next_x]
                    ):
                        visited[next_y, next_x] = True
                        queue.append((next_y, next_x))
        if len(component) >= min_cells:
            components.append(component)
    components.sort(key=len, reverse=True)
    return components


def component_grid(component: list[tuple[int, int]], shape: tuple[int, int]) -> np.ndarray:
    """Convert component cells to a boolean grid."""
    grid = np.zeros(shape, dtype=bool)
    if component:
        rows, columns = zip(*component)
        grid[np.asarray(rows), np.asarray(columns)] = True
    return grid


def build_scene(points: np.ndarray) -> dict[str, Any]:
    """Build geometry-semantic labels and a compact semantic graph."""
    levels = dominant_levels(points[:, 2])
    if not levels:
        raise ValueError("no dominant horizontal surface levels found")
    level_array = np.asarray(levels)
    distance = np.min(np.abs(points[:, 2, None] - level_array[None, :]), axis=1)
    nearest = np.argmin(np.abs(points[:, 2, None] - level_array[None, :]), axis=1)
    surface_mask = distance <= 0.13

    voxel_size = 0.25
    min_xy = points[:, :2].min(axis=0)
    cell_xy = np.floor((points[:, :2] - min_xy[None, :]) / voxel_size).astype(np.int32)
    width = int(cell_xy[:, 0].max()) + 1
    height = int(cell_xy[:, 1].max()) + 1
    count_grid = np.zeros((height, width), dtype=np.int32)
    z_min_grid = np.full((height, width), np.inf, dtype=np.float32)
    z_max_grid = np.full((height, width), -np.inf, dtype=np.float32)
    rows, columns = cell_xy[:, 1], cell_xy[:, 0]
    np.add.at(count_grid, (rows, columns), 1)
    np.minimum.at(z_min_grid, (rows, columns), points[:, 2])
    np.maximum.at(z_max_grid, (rows, columns), points[:, 2])

    structural_components = connected_components(
        (count_grid >= 4) & ((z_max_grid - z_min_grid) > 0.8),
    )
    structural_grids = [component_grid(component, count_grid.shape) for component in structural_components]
    building_nodes: list[dict[str, Any]] = []
    for index, (component, grid) in enumerate(zip(structural_components, structural_grids)):
        component_rows, component_columns = zip(*component)
        x_values = min_xy[0] + np.asarray(component_columns) * voxel_size
        y_values = min_xy[1] + np.asarray(component_rows) * voxel_size
        building_nodes.append(
            {
                "id": f"building_{index}",
                "kind": "vertical_structure_group",
                "label": "building structure candidate",
                "center": {"x": round(float(x_values.mean()), 3), "y": round(float(y_values.mean()), 3)},
                "bbox": {
                    "min_x": round(float(x_values.min()), 3),
                    "max_x": round(float(x_values.max() + voxel_size), 3),
                    "min_y": round(float(y_values.min()), 3),
                    "max_y": round(float(y_values.max() + voxel_size), 3),
                },
                "cell_count": len(component),
                "point_count": int(count_grid[grid].sum()),
            }
        )

    floor_regions: list[dict[str, Any]] = []
    floor_region_grids: list[np.ndarray] = []
    for level_index, level in enumerate(levels):
        level_points = np.abs(points[:, 2] - level) <= 0.13
        level_grid = np.zeros((height, width), dtype=bool)
        np.logical_or.at(level_grid, (rows[level_points], columns[level_points]), True)
        for region_index, component in enumerate(connected_components(level_grid)):
            region_grid = component_grid(component, level_grid.shape)
            component_rows, component_columns = zip(*component)
            x_values = min_xy[0] + np.asarray(component_columns) * voxel_size
            y_values = min_xy[1] + np.asarray(component_rows) * voxel_size
            center_x = float(x_values.mean())
            center_y = float(y_values.mean())
            if building_nodes:
                building_index = min(
                    range(len(building_nodes)),
                    key=lambda index: (
                        (center_x - building_nodes[index]["center"]["x"]) ** 2
                        + (center_y - building_nodes[index]["center"]["y"]) ** 2
                    ),
                )
            else:
                building_index = -1
            region_id = f"floor_region_{level_index}_{region_index}"
            floor_regions.append(
                {
                    "id": region_id,
                    "kind": "support_surface_region",
                    "label": "support surface candidate",
                    "level_id": f"surface_{level_index}",
                    "building_id": f"building_{building_index}" if building_index >= 0 else None,
                    "building_assignment": "nearest structure-group center" if building_index >= 0 else "unassigned",
                    "z": round(float(level), 3),
                    "center": {"x": round(center_x, 3), "y": round(center_y, 3), "z": round(float(level), 3)},
                    "area_m2": round(len(component) * voxel_size * voxel_size, 3),
                    "cell_count": len(component),
                }
            )
            floor_region_grids.append(region_grid)

    level_nodes = [
        {
            "id": f"surface_{index}",
            "kind": "surface_level",
            "label": "horizontal surface candidate",
            "z": round(float(level), 3),
            "evidence": "dominant point-cloud height band",
            "point_count": int(np.count_nonzero(surface_mask & (nearest == index))),
        }
        for index, level in enumerate(levels)
    ]
    nodes = [{"id": "site", "kind": "site", "label": "point-cloud site"}, *building_nodes, *floor_regions, *level_nodes]
    relations: list[dict[str, Any]] = []
    relations.extend(
        {"subject_id": node["id"], "predicate": "contains", "object_id": region["id"], "evidence": "2D component overlap"}
        for node in building_nodes
        for region in floor_regions
        if region.get("building_id") == node["id"]
    )
    relations.extend(
        {"subject_id": "site", "predicate": "contains", "object_id": node["id"], "evidence": "scene root"}
        for node in building_nodes
    )
    relations.extend(
        {"subject_id": region["id"], "predicate": "at_level", "object_id": region["level_id"], "evidence": "height band"}
        for region in floor_regions
    )
    relations.extend(
        {
            "subject_id": level_nodes[index]["id"],
            "predicate": "below",
            "object_id": level_nodes[index + 1]["id"],
            "evidence": "ordered height levels",
        }
        for index in range(len(level_nodes) - 1)
    )

    return {
        "schema_version": "geometry_semantic_scene.v1",
        "source": {
            "kind": "point_cloud",
            "path": "sim/evaluation/data/octoplanner3d/building2_9.pcd",
            "point_count": int(len(points)),
            "frame_id": "source_pcd",
        },
        "semantic_layers": [
            {"id": "surface_level", "meaning": "dominant horizontal surface candidate", "distance_tolerance_m": 0.13},
            {"id": "structure", "meaning": "remaining observed geometry; no object class inferred"},
        ],
        "voxel_size_m": voxel_size,
        "nodes": nodes,
        "relations": relations,
        "navigation_layers": [
            {"id": "support_candidate", "source": "floor_region", "use": "candidate ground/support"},
            {"id": "collision_geometry", "source": "all observed points", "use": "native collision query input"},
            {"id": "unknown_space", "source": "not inferred from missing points", "use": "requires ray/sensor evidence"},
        ],
        "limitations": [
            "PCD has geometry but no RGB/RGB-D appearance or camera poses.",
            "Object labels and room names are not inferred from geometry alone.",
            "This is a planning-support semantic layer, not an activation-ready saved map.",
        ],
        "labels": {
            "surface_level": surface_mask,
            "surface_index": nearest,
            "cell_xy": cell_xy,
            "structural_grids": structural_grids,
            "floor_region_grids": floor_region_grids,
        },
    }


def render(scene: dict[str, Any], points: np.ndarray, output: Path) -> None:
    """Render top-down and 3D geometry-semantic views."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    labels = scene["labels"]
    surface = labels["surface_level"]
    surface_index = labels["surface_index"]
    level_nodes = [node for node in scene["nodes"] if node["kind"] == "surface_level"]
    building_nodes = [node for node in scene["nodes"] if node["kind"] == "vertical_structure_group"]
    floor_nodes = [node for node in scene["nodes"] if node["kind"] == "support_surface_region"]
    levels = [node["z"] for node in level_nodes]
    plot_stride = max(1, len(points) // 150000)
    sample = points[::plot_stride]
    sample_surface = surface[::plot_stride]
    sample_index = surface_index[::plot_stride]
    sample_cells = labels["cell_xy"][::plot_stride]
    sample_x = np.clip(sample_cells[:, 0], 0, labels["structural_grids"][0].shape[1] - 1)
    sample_y = np.clip(sample_cells[:, 1], 0, labels["structural_grids"][0].shape[0] - 1)

    structure_index = np.full(len(sample), -1, dtype=np.int32)
    for index, grid in enumerate(labels["structural_grids"]):
        selected = grid[sample_y, sample_x] & (structure_index < 0)
        structure_index[selected] = index
    region_index = np.full(len(sample), -1, dtype=np.int32)
    for index, grid in enumerate(labels["floor_region_grids"]):
        selected = grid[sample_y, sample_x] & sample_surface & (region_index < 0)
        region_index[selected] = index

    level_palette = plt.cm.tab10(np.linspace(0.08, 0.88, max(1, len(levels))))
    structure_palette = plt.cm.Set2(np.linspace(0.12, 0.88, max(1, len(building_nodes))))
    colors = np.tile(np.array([0.45, 0.49, 0.55, 0.65]), (len(sample), 1))
    structural_points = structure_index >= 0
    colors[structural_points] = structure_palette[structure_index[structural_points] % len(structure_palette)]
    colors[sample_surface] = level_palette[sample_index[sample_surface] % len(level_palette)]

    plt.style.use("dark_background")
    figure = plt.figure(figsize=(14, 8.5), dpi=170, facecolor="#0e141b")
    figure.suptitle("Geometry-semantic scene from building2_9.pcd", color="#e8edf3", fontsize=16, y=0.97)
    figure.text(
        0.5,
        0.935,
        f"{len(points):,} points · structure groups + support regions + surface levels · object names require RGB/RGB-D",
        ha="center",
        color="#a6b1c2",
        fontsize=9,
    )

    top = figure.add_subplot(2, 2, 1)
    top.scatter(sample[:, 0], sample[:, 1], s=0.18, c=colors, linewidths=0, rasterized=True)
    top.set_title("Semantic top view", color="#e8edf3")
    top.set_xlabel("x (m)"); top.set_ylabel("y (m)"); top.set_aspect("equal", adjustable="box"); top.grid(alpha=0.12)

    view = figure.add_subplot(2, 2, 2, projection="3d")
    view_sample = sample[::max(1, len(sample) // 65000)]
    view_surface = sample_surface[::max(1, len(sample) // 65000)]
    view_index = sample_index[::max(1, len(sample) // 65000)]
    view_structure = structure_index[::max(1, len(sample) // 65000)]
    view_colors = np.tile(np.array([0.45, 0.49, 0.55, 0.62]), (len(view_sample), 1))
    view_colors[view_structure >= 0] = structure_palette[view_structure[view_structure >= 0] % len(structure_palette)]
    view_colors[view_surface] = level_palette[view_index[view_surface] % len(level_palette)]
    view.scatter(view_sample[:, 0], view_sample[:, 1], view_sample[:, 2], s=0.24, c=view_colors, linewidths=0, depthshade=False, rasterized=True)
    view.set_title("3D semantic geometry", color="#e8edf3")
    view.set_xlabel("x (m)"); view.set_ylabel("y (m)"); view.set_zlabel("z (m)"); view.view_init(elev=33, azim=-57); view.grid(False)
    view.xaxis.pane.set_alpha(0); view.yaxis.pane.set_alpha(0); view.zaxis.pane.set_alpha(0)

    xz = figure.add_subplot(2, 2, 3)
    xz.scatter(sample[:, 0], sample[:, 2], s=0.18, c=colors, linewidths=0, rasterized=True)
    for level in levels:
        xz.axhline(level, color="#ffb86b", linewidth=0.8, alpha=0.55)
    xz.set_title("Surface candidates in x–z", color="#e8edf3"); xz.set_xlabel("x (m)"); xz.set_ylabel("z (m)"); xz.grid(alpha=0.12)

    legend = figure.add_subplot(2, 2, 4)
    legend.axis("off")
    legend.text(0.02, 0.91, "Current semantic evidence", color="#e8edf3", fontsize=13, weight="medium")
    legend.text(0.02, 0.84, "Observed geometry is grouped into a scene hierarchy.", color="#a6b1c2", fontsize=9)
    legend.text(0.02, 0.78, f"site → {len(building_nodes)} structure groups → {len(floor_nodes)} support regions → {len(level_nodes)} surface levels", color="#e8edf3", fontsize=10)
    cursor = 0.69
    for index, node in enumerate(building_nodes):
        legend.add_line(Line2D([0.03, 0.10], [cursor, cursor], color=structure_palette[index], linewidth=5))
        legend.text(0.14, cursor, f"{node['id']} · {node['cell_count']:,} cells · {node['point_count']:,} pts", va="center", color="#e8edf3", fontsize=9)
        cursor -= 0.055
    cursor -= 0.015
    for index, level in enumerate(levels):
        y = cursor - index * min(0.06, 0.34 / max(1, len(levels)))
        legend.add_line(Line2D([0.03, 0.10], [y, y], color=level_palette[index], linewidth=5))
        legend.text(0.14, y, f"surface_{index} · z={level:.2f} m · {level_nodes[index]['point_count']:,} pts", va="center", color="#e8edf3", fontsize=9)
    legend.text(0.02, 0.09, "support_candidate / collision_geometry / unknown_space are separate navigation layers", color="#ffb86b", fontsize=8.5)
    legend.text(0.02, 0.045, "Next layer: RGB/RGB-D + poses + LiDAR↔camera extrinsics → desk/box/room labels", color="#ffb86b", fontsize=8.5)

    figure.text(0.01, 0.012, "Geometry-only semantic scene · no fabricated object labels", color="#a6b1c2", fontsize=8)
    figure.tight_layout(rect=[0.02, 0.035, 0.98, 0.91])
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches="tight", facecolor=figure.get_facecolor())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path, help="binary PCD input")
    parser.add_argument("--output-dir", type=Path, required=True, help="directory for scene JSON and PNG")
    args = parser.parse_args()

    points = read_pcd(args.pcd)
    scene = build_scene(points)
    scene["source"]["path"] = args.pcd.as_posix()
    scene_for_json = {key: value for key, value in scene.items() if key != "labels"}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "scene.json").write_text(json.dumps(scene_for_json, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    render(scene, points, args.output_dir / "scene.png")
    print(json.dumps({"points": len(points), "surface_levels_m": [node["z"] for node in scene["nodes"] if node["kind"] == "surface_level"], "structures": len([node for node in scene["nodes"] if node["kind"] == "vertical_structure_group"]), "support_regions": len([node for node in scene["nodes"] if node["kind"] == "support_surface_region"]), "output_dir": str(args.output_dir)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
