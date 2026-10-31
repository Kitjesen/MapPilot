#!/usr/bin/env python3
"""Render saved native mapping evidence without loading the whole PCD into RAM."""

from __future__ import annotations

import argparse
import json
from itertools import product
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import findfont
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d import proj3d


def _pcd_header(stream: Any) -> tuple[dict[str, list[str]], int]:
    header: dict[str, list[str]] = {}
    while line := stream.readline():
        text = line.decode("ascii").strip()
        if not text or text.startswith("#"):
            continue
        key, *values = text.split()
        header[key.upper()] = values
        if key.upper() == "DATA":
            return header, stream.tell()
    raise ValueError("PCD is missing its DATA header")


def load_cloud_sample(path: Path, max_points: int = 150_000) -> tuple[np.ndarray, dict[str, Any]]:
    """Read finite bounds in chunks and sample deterministically by source index."""
    if max_points <= 0:
        raise ValueError("max_points must be positive")
    with path.open("rb") as stream:
        header, offset = _pcd_header(stream)
    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD must contain x/y/z")
    count = int(header.get("POINTS", ["0"])[0])
    if count <= 0:
        raise ValueError("PCD contains no points")
    encoding = header["DATA"][0].lower()
    indices = np.linspace(0, count - 1, min(count, max_points), dtype=np.int64)
    low = np.full(3, np.inf)
    high = np.full(3, -np.inf)
    finite_count = 0

    def measure(xyz: np.ndarray) -> None:
        nonlocal low, high, finite_count
        finite = xyz[np.isfinite(xyz).all(axis=1)]
        if len(finite):
            low = np.minimum(low, finite.min(axis=0))
            high = np.maximum(high, finite.max(axis=0))
            finite_count += len(finite)

    if encoding == "binary":
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        if not len(fields) == len(sizes) == len(types) == len(counts):
            raise ValueError("PCD field declarations disagree")
        formats, offsets, stride = [], [], 0
        for field, size, kind, multiplicity in zip(fields, sizes, types, counts):
            if field in ("x", "y", "z"):
                if kind != "F" or size not in (4, 8) or multiplicity != 1:
                    raise ValueError("PCD XYZ fields must be scalar floating-point values")
                formats.append((field, f"<f{size}"))
                offsets.append((field, stride))
            stride += size * multiplicity
        if path.stat().st_size < offset + count * stride:
            raise ValueError("PCD binary payload is shorter than POINTS declares")
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [dict(formats)[axis] for axis in ("x", "y", "z")],
            "offsets": [dict(offsets)[axis] for axis in ("x", "y", "z")],
            "itemsize": stride,
        })
        records = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,))
        for begin in range(0, count, 262_144):
            block = records[begin:begin + 262_144]
            measure(np.column_stack([block[axis] for axis in ("x", "y", "z")]))
        sample = np.column_stack([records[axis][indices] for axis in ("x", "y", "z")])
        del records
    elif encoding == "ascii":
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        columns, cursor = {}, 0
        for field, multiplicity in zip(fields, counts):
            columns[field] = cursor
            cursor += multiplicity
        selected = set(indices.tolist())
        samples = []
        rows = 0
        with path.open("rb") as stream:
            stream.seek(offset)
            for line in stream:
                if not line.strip():
                    continue
                values = line.split()
                xyz = np.asarray([float(values[columns[axis]]) for axis in ("x", "y", "z")])
                measure(xyz.reshape(1, 3))
                if rows in selected:
                    samples.append(xyz)
                rows += 1
        if rows != count:
            raise ValueError("PCD ASCII row count disagrees with POINTS")
        sample = np.asarray(samples)
    else:
        raise ValueError(f"Unsupported PCD DATA encoding: {encoding}; native binary/ASCII required")
    sample = sample[np.isfinite(sample).all(axis=1)]
    if not finite_count or not len(sample):
        raise ValueError("PCD has no finite points available for rendering")
    return sample, {
        "source_points": count,
        "finite_points": finite_count,
        "rendered_points": len(sample),
        "max_rendered_points": max_points,
        "sampling": "deterministic_linear_source_indices; nonfinite XYZ excluded",
        "encoding": encoding,
        "bounds_min_m": low.tolist(),
        "bounds_max_m": high.tolist(),
    }


def load_trajectory(path: Path) -> np.ndarray:
    table = np.loadtxt(path, comments="#", ndmin=2)
    if table.shape[1] != 8 or not len(table) or not np.isfinite(table).all():
        raise ValueError("Trajectory must contain finite TUM rows: timestamp XYZ qx qy qz qw")
    return table


def load_plan(path: Path | None) -> tuple[np.ndarray, dict[str, Any]]:
    if path is None:
        return np.empty((0, 3)), {"status": "not_run", "error": None, "displayed_path_points": 0}
    result = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(result, dict):
        raise ValueError("Native planning result must be a JSON object")
    returned = result.get("path", [])
    points = np.asarray(returned, dtype=np.float64)
    if points.size and (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()):
        raise ValueError("Native planning path must contain finite XYZ triples")
    reached = result.get("ok") is True and result.get("reached_goal") is True and bool(points.size)
    info = {
        "status": "returned_route" if reached else "not_reached",
        "ok": result.get("ok"),
        "reached_goal": result.get("reached_goal"),
        "error": result.get("error"),
        "returned_path_points": len(points),
        "displayed_path_points": len(points) if reached else 0,
        "execution_validated": False,
    }
    return (points if reached else np.empty((0, 3))), info


def _style() -> None:
    findfont("Arial", fallback_to_default=False)
    plt.rcParams.update({
        "font.family": "Arial", "font.size": 10,
        "axes.titlesize": 10, "axes.labelsize": 10,
        "xtick.labelsize": 10, "ytick.labelsize": 10,
        "legend.fontsize": 10, "figure.titlesize": 10,
        "pdf.fonttype": 42, "ps.fonttype": 42,
        "axes.edgecolor": "#7a828b", "axes.labelcolor": "#303943",
        "xtick.color": "#4c5661", "ytick.color": "#4c5661",
        "grid.color": "#dbe0e5", "grid.linewidth": 0.5,
        "savefig.facecolor": "white",
    })


def trajectory_focus_bounds(trajectory: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return a display-only XYZ crop with the declared fixed five-metre margin."""
    return trajectory[:, 1:4].min(axis=0) - 5.0, trajectory[:, 1:4].max(axis=0) + 5.0


def _position_3d_heading(fig: Any, ax: Any, colorbar: Any, handles: list, title: str) -> None:
    """Place the heading against the projected map box, not the empty axes corner."""
    fig.canvas.draw()
    corners = np.asarray(list(product(ax.get_xlim(), ax.get_ylim(), ax.get_zlim())))
    projected = proj3d.proj_transform(*corners.T, ax.get_proj())
    display = ax.transData.transform(np.column_stack(projected[:2]))
    positions = fig.transFigure.inverted().transform(display)
    bottom, top = positions[:, 1].min(), positions[:, 1].max()
    colorbar.ax.set_position([0.88, max(0.16, bottom), 0.022, max(0.25, top - bottom)])
    legend = ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.47, min(0.87, top + 0.02)),
                       bbox_transform=fig.transFigure, borderaxespad=0, frameon=False,
                       ncols=len(handles), columnspacing=1.1, handletextpad=0.45)
    fig.canvas.draw()
    legend_top = fig.transFigure.inverted().transform(legend.get_window_extent().get_points())[1, 1]
    fig.suptitle(title, y=min(0.99, legend_top + 0.035), va="bottom")


def render(
    map_dir: Path, trajectory_path: Path, output_dir: Path, title: str,
    planning_result: Path | None = None, max_points: int = 150_000,
    focus_trajectory: bool = False,
) -> dict[str, Any]:
    cloud_path = map_dir / "map.pcd"
    cloud, cloud_info = load_cloud_sample(cloud_path, max_points)
    trajectory = load_trajectory(trajectory_path)
    route, plan_info = load_plan(planning_result)
    _style()
    output_dir.mkdir(parents=True, exist_ok=True)
    display = {"mode": "full_map", "source_data_changed": False}
    if focus_trajectory:
        focus_min, focus_max = trajectory_focus_bounds(trajectory)
        sampled_before_focus = len(cloud)
        cloud = cloud[((cloud >= focus_min) & (cloud <= focus_max)).all(axis=1)]
        if not len(cloud):
            raise ValueError("No sampled map returns intersect the trajectory focus bounds")
        cloud_info = {**cloud_info, "rendered_points": len(cloud),
                      "sampled_points_before_focus": sampled_before_focus,
                      "focus_excluded_sample_points": sampled_before_focus - len(cloud)}
        display = {"mode": "trajectory_bounds_plus_margin", "margin_m": 5.0,
                   "bounds_min_m": focus_min.tolist(), "bounds_max_m": focus_max.tolist(),
                   "source_data_changed": False}
    color = "#bb723b"
    route_color = "#008da2"
    limits_min = np.asarray(cloud_info["bounds_min_m"])
    limits_max = np.asarray(cloud_info["bounds_max_m"])
    if focus_trajectory:
        limits_min, limits_max = focus_min, focus_max
    else:
        for values in (trajectory[:, 1:4], route):
            if len(values):
                limits_min = np.minimum(limits_min, values.min(axis=0))
                limits_max = np.maximum(limits_max, values.max(axis=0))
    span = np.maximum(limits_max - limits_min, 0.2)
    margin = span * 0.035
    if not focus_trajectory:
        limits_min -= margin
        limits_max += margin
    norm = matplotlib.colors.Normalize(
        vmin=cloud_info["bounds_min_m"][2],
        vmax=max(cloud_info["bounds_max_m"][2], cloud_info["bounds_min_m"][2] + 0.01),
    )
    legend = [
        Line2D([], [], marker=".", linestyle="none", color="#607b88", label="Saved surface returns"),
        Line2D([], [], color=color, linewidth=1.2, label="Estimated IMU trajectory"),
    ]
    if len(route):
        legend.append(Line2D([], [], color=route_color, linewidth=1.6, label="Global planned route"))
    outputs = []
    for name, axes in (("map3d", None), ("topdown", (0, 1)), ("side", (0, 2))):
        fig = plt.figure(figsize=(7.4, 5.5 if axes is None else 4.8))
        ax = fig.add_subplot(111, projection="3d" if axes is None else None)
        fig.subplots_adjust(left=0.10, right=0.79 if axes is None else 0.86, bottom=0.15, top=0.84)
        fig.suptitle(title, y=0.98)
        if axes is None:
            scatter = ax.scatter(*cloud.T, c=cloud[:, 2], cmap="cividis", norm=norm,
                                 s=0.45, alpha=0.58, linewidths=0, depthshade=False)
            ax.plot(*trajectory[:, 1:4].T, color=color, linewidth=1.0)
            if len(route):
                ax.plot(*route.T, color=route_color, linewidth=1.7)
            ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
                   xlim=(limits_min[0], limits_max[0]), ylim=(limits_min[1], limits_max[1]),
                   zlim=(limits_min[2], limits_max[2]))
            ax.set_box_aspect(span)
            ax.view_init(elev=26, azim=-55)
            for axis, extent in zip((ax.xaxis, ax.yaxis, ax.zaxis), span):
                axis.set_major_locator(MaxNLocator(nbins=2 if extent < span.max() * 0.18 else 4))
            ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        else:
            first, second = axes
            scatter = ax.scatter(cloud[:, first], cloud[:, second], c=cloud[:, 2],
                                 cmap="cividis", norm=norm, s=0.4, alpha=0.58, linewidths=0)
            ax.plot(trajectory[:, first + 1], trajectory[:, second + 1], color=color, linewidth=1.0)
            if len(route):
                ax.plot(route[:, first], route[:, second], color=route_color, linewidth=1.6)
            ax.set(xlabel=f"{'XYZ'[first]} (m)", ylabel=f"{'XYZ'[second]} (m)",
                   xlim=(limits_min[first], limits_max[first]), ylim=(limits_min[second], limits_max[second]))
            if name == "topdown":
                ax.set_aspect("equal", adjustable="box" if focus_trajectory else "datalim")
            ax.spines[["top", "right"]].set_visible(False)
            ax.set_axisbelow(True)
            ax.grid(True)
        if axes is None:
            colorbar = fig.colorbar(scatter, cax=fig.add_axes([0.88, 0.25, 0.022, 0.5]))
            _position_3d_heading(fig, ax, colorbar, legend, title)
        else:
            ax.legend(handles=legend, loc="lower center", bbox_to_anchor=(0.5, 1.015),
                      borderaxespad=0, frameon=False, ncols=2 if len(legend) < 3 else 3,
                      columnspacing=1.2, handletextpad=0.45)
            colorbar = fig.colorbar(scatter, ax=ax, fraction=0.035, pad=0.045, shrink=0.8)
        colorbar.set_label("Map Z (m)", fontsize=10)
        colorbar.ax.tick_params(labelsize=10)
        if plan_info["status"] == "not_reached":
            fig.text(0.12, 0.035, f"Planning: {plan_info['error'] or 'goal not reached'}", fontsize=10)
        for suffix in ("png", "pdf"):
            target = output_dir / f"{name}{'_focus' if focus_trajectory else ''}.{suffix}"
            fig.savefig(target, dpi=220, bbox_inches="tight", pad_inches=0.08)
            outputs.append(str(target.resolve()))
        plt.close(fig)
    report = {
        "scope": "offline_native_saved_map_and_global_plan_visualization",
        "map_pcd": str(cloud_path.resolve()),
        "trajectory": str(trajectory_path.resolve()),
        "planning_result": str(planning_result.resolve()) if planning_result else None,
        "title": title,
        "font": {"family": "Arial", "size_pt": 10},
        "cloud": cloud_info,
        "display_view": display,
        "trajectory_reference": "dataset IMU pose; not robot motion execution",
        "trajectory_samples": len(trajectory),
        "trajectory_duration_s": float(trajectory[-1, 0] - trajectory[0, 0]),
        "planning": plan_info,
        "execution_validated": False,
        "files": outputs,
    }
    report_name = "figures_focus.json" if focus_trajectory else "figures.json"
    (output_dir / report_name).write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--planning-result", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--title", default="Native LiDAR-inertial map")
    parser.add_argument("--max-points", type=int, default=150_000)
    parser.add_argument("--focus-trajectory", action="store_true",
                        help="Display only the estimated-trajectory XYZ bounds plus a fixed 5 m margin; source PCD is unchanged")
    args = parser.parse_args()
    report = render(args.map_dir, args.trajectory, args.output_dir, args.title,
                    args.planning_result, args.max_points, args.focus_trajectory)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
