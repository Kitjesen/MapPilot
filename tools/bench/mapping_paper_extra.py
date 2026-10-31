"""Draw supplemental manuscript figures from retained measurements only."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import findfont
from matplotlib.lines import Line2D
from matplotlib.ticker import FixedLocator, MaxNLocator
from render_mapping import load_cloud_sample, load_trajectory, trajectory_focus_bounds

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build/paper-manuscript-20261021-rev2/figures"
WIDTH = 6.85
BLUE = "#23679b"
ORANGE = "#b75b35"
GREY = "#77828a"


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def style():
    font = findfont("Arial", fallback_to_default=False)
    plt.rcParams.update({
        "font.family": "Arial", "font.size": 10, "axes.labelsize": 10,
        "axes.titlesize": 10, "xtick.labelsize": 10, "ytick.labelsize": 10,
        "legend.fontsize": 10, "figure.titlesize": 10,
        "pdf.fonttype": 42, "ps.fonttype": 42, "axes.linewidth": .6,
        "axes.edgecolor": "#333333", "grid.color": "#d7dde0",
        "grid.linewidth": .45, "axes.axisbelow": True,
    })
    return font


def save(figure, stem, data):
    figure.canvas.draw()
    for suffix in ("pdf", "png"):
        figure.savefig(OUT / f"{stem}.{suffix}", dpi=300, facecolor="white")
    data.update({"font": {"family": "Arial", "size_pt": 10},
                 "size_inches": list(figure.get_size_inches()), "png_dpi": 300,
                 "figure_pdf_scale_for_manuscript": 1.0})
    (OUT / f"{stem}.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    plt.close(figure)
    return data


def clean_axes(axes):
    axes.spines[["top", "right"]].set_visible(False)
    axes.grid(axis="y")


def garage_partial():
    folder = ROOT / "build/garage-validation-20261021"
    cloud_path = folder / "maps/no_loop/map.pcd"
    trajectory_path = folder / "runs/no_loop/replay.map.tum"
    cloud, info = load_cloud_sample(cloud_path, 100_000)
    trajectory = load_trajectory(trajectory_path)
    low, high = trajectory_focus_bounds(trajectory)
    before = len(cloud)
    cloud = cloud[((cloud >= low) & (cloud <= high)).all(axis=1)]
    routes, sources = [], []
    for case in ("local_20_25", "longer_40_70"):
        source = folder / f"planning/no_loop/{case}_native/result.json"
        sources.append(str(source))
        routes.append(np.asarray(read(source)["path"], dtype=float))
    if [len(route) for route in routes] != [34, 145]:
        raise ValueError("The retained no_loop routes must have 34 and 145 vertices")
    summary_path = folder / "planning/no_loop/summary.json"
    summary = read(summary_path)
    figure = plt.figure(figsize=(WIDTH, 4.9))
    ax3 = figure.add_axes([.05, .46, .80, .44], projection="3d")
    norm = matplotlib.colors.Normalize(vmin=low[2], vmax=high[2])
    points = ax3.scatter(*cloud.T, c=cloud[:, 2], cmap="cividis", norm=norm,
                         s=.35, alpha=.45, linewidths=0, depthshade=False, rasterized=True)
    ax3.plot(*trajectory[:, 1:4].T, color=GREY, linewidth=.8)
    for route, color in zip(routes, (BLUE, ORANGE)):
        ax3.plot(*route.T, color=color, linewidth=1.5)
    ax3.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
            xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
    ax3.set_box_aspect(high - low)
    ax3.view_init(elev=28, azim=-58)
    ax3.xaxis.pane.fill = ax3.yaxis.pane.fill = ax3.zaxis.pane.fill = False
    for axis in (ax3.xaxis, ax3.yaxis):
        axis.set_major_locator(MaxNLocator(nbins=3))
        axis.labelpad = 3
    ax3.zaxis.set_major_locator(FixedLocator([0]))
    ax3.zaxis.labelpad = 2
    ax3.tick_params(pad=1)
    colorbar = figure.colorbar(points, cax=figure.add_axes([.90, .55, .015, .28]))
    colorbar.set_label("Map Z (m)")
    colorbar.locator = MaxNLocator(nbins=3)
    colorbar.update_ticks()
    figure.text(.07, .88, "(a) Historical partial map (~134 s)")
    for index, (route, color) in enumerate(zip(routes, (BLUE, ORANGE))):
        ax = figure.add_axes([.11 + index*.48, .15, .34, .24])
        limits_low = route.min(axis=0) - .8
        limits_high = route.max(axis=0) + .8
        visible = cloud[((cloud[:, :2] >= limits_low[:2]) &
                         (cloud[:, :2] <= limits_high[:2])).all(axis=1)]
        ax.scatter(visible[:, 0], visible[:, 1], color="#8e969c", s=.4,
                   alpha=.25, linewidths=0, rasterized=True)
        ax.plot(route[:, 0], route[:, 1], color=color, linewidth=1,
                marker=".", markersize=2.5)
        ax.scatter(*route[0, :2], marker="s", s=22, color=color, zorder=4)
        ax.scatter(*route[-1, :2], marker="D", s=22, color=color, zorder=4)
        ax.set(xlabel="X (m)", ylabel="Y (m)",
               xlim=(limits_low[0], limits_high[0]), ylim=(limits_low[1], limits_high[1]))
        ax.set_aspect("equal", adjustable="box")
        ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
        clean_axes(ax)
        figure.text(.11 + index*.48, .415,
                    f"({'bc'[index]}) {len(route)} vertices / {('20-25', '40-70')[index]} s")
    figure.legend(handles=[
        Line2D([], [], color=GREY, label="Estimated IMU path"),
        Line2D([], [], color=BLUE, label="34-vertex route"),
        Line2D([], [], color=ORANGE, label="145-vertex route")],
        loc="upper center", bbox_to_anchor=(.5, 1), frameon=False,
        ncols=3, columnspacing=.9, handlelength=1.3)
    figure.text(.5, .025, "Saved no_loop evidence; offline geometry, no robot execution.", ha="center")
    return save(figure, "garage-partial-routes", {
        "source_map": str(cloud_path), "source_trajectory": str(trajectory_path),
        "source_routes": sources, "source_summary": str(summary_path),
        "cloud": info, "display_crop": {"rule": "same-run trajectory bounds plus 5 m",
            "min_m": low.tolist(), "max_m": high.tolist(),
            "sampled_before_crop": before, "sampled_after_crop": len(cloud)},
        "route_xyz_m": [route.tolist() for route in routes],
        "route_summary": summary["cases"][:2],
        "trajectory_duration_s": float(trajectory[-1, 0] - trajectory[0, 0]),
        "qualification": "Historical partial no_loop map and same-run routes only; not the latest complete first-stage replay; offline geometry, no SCAN or robot execution.",
        "source_changed": False,
    })


def corridor_components():
    source = ROOT / "build/prob-plane-20261021/isotropic/evaluation.json"
    scores = read(source)["cases"]["corridor"]["scores"]["imu_frame"]["common"]
    prior = np.array([scores["prior"]["rmse_m"], *scores["prior"]["axis_rmse_m"]])
    fresh = np.array([scores["fresh"]["rmse_m"], *scores["fresh"]["axis_rmse_m"]])
    changes = (fresh / prior - 1) * 100
    figure, axes = plt.subplots(figsize=(WIDTH, 3.35))
    figure.subplots_adjust(left=.12, right=.97, top=.79, bottom=.21)
    x = np.arange(4)
    for shift, values, color, label in ((-.19, prior, GREY, "Time-consistent baseline"),
                                        (.19, fresh, BLUE, "Probability-plane stage 1")):
        axes.bar(x + shift, values, width=.34, color=color, label=label)
        for at, value in zip(x + shift, values):
            axes.text(at, value + .035, f"{value:.3f}", ha="center", va="bottom")
    axes.set(xticks=x, xticklabels=["3D position", "X", "Y", "Z"],
             ylabel="RMSE (m)", ylim=(0, 1.9))
    axes.get_xticklabels()[-1].set_color(ORANGE)
    for at, change in zip(x, changes):
        axes.text(at, 1.76, f"{change:+.2f}%", ha="center",
                  color=ORANGE if at == 3 else "#333333")
    clean_axes(axes)
    figure.legend(loc="upper center", bbox_to_anchor=(.52, .99), frameon=False,
                  ncols=2, columnspacing=1.1, handlelength=1.3)
    figure.text(.5, .055, "Corridor02: 8,038 common samples; scale-1 SE(3) alignment.", ha="center")
    return save(figure, "corridor-rmse-components", {
        "source": str(source), "sample_count": scores["prior"]["samples"],
        "metric_labels": ["3D position", "X", "Y", "Z"],
        "prior_rmse_m": prior.tolist(), "first_stage_rmse_m": fresh.tolist(),
        "change_percent": changes.tolist(), "exact_source_scores": scores,
        "qualification": "Same frozen reference times; conditional reference accuracy, not robot tracking; lower total RMSE coexists with worse Z RMSE.",
    })


def garage_gcps():
    source = ROOT / "build/genz-fusion-20261021/garage/comparison-first-stage-v1-all-first-stage-full.json"
    run = read(source)["runs"]["first-stage-full"]
    evaluation = run["own_sparse_gcp_evaluation"]
    scores = evaluation["runs"]["first-stage-full"]
    rows = scores["point_residuals"]
    values = [row["residual_norm_m"] for row in rows]
    indices = [row["gcp_index"] for row in rows]
    figure, axes = plt.subplots(figsize=(WIDTH, 3.3))
    figure.subplots_adjust(left=.12, right=.97, top=.79, bottom=.31)
    axes.bar(indices, values, color=BLUE, width=.57)
    for at, value in zip(indices, values):
        axes.text(at, value+.02, f"{value:.3f}", ha="center", va="bottom")
    rmse = scores["sparse_gcp_rmse_m"]
    axes.axhline(rmse, color=ORANGE, linestyle="--", linewidth=1,
                 label=f"Sparse GCP RMSE = {rmse:.3f} m")
    axes.set(xlabel="Surveyed GCP index", ylabel="3D residual norm (m)",
             xticks=indices, ylim=(0, .78))
    clean_axes(axes)
    figure.legend(loc="upper center", bbox_to_anchor=(.52, .99), frameon=False)
    figure.text(.5, .10, "Complete first-stage replay: 7 / 7 checkpoints.", ha="center")
    figure.text(.5, .025, "Same-point SE(3) fit, scale 1; not dense ATE or held-out accuracy.", ha="center")
    return save(figure, "garage-full-gcp-errors", {
        "source": str(source), "run": "first-stage-full", "point_residuals": rows,
        "sparse_gcp_rmse_m": rmse, "sparse_gcp_max_error_m": scores["sparse_gcp_max_error_m"],
        "pairwise_distance_rmse_m": scores["pairwise_distance_rmse_m"],
        "association_rule": evaluation["gcp_rule"], "associations": evaluation["associations"],
        "limits": evaluation["limitations"], "trajectory": scores["trajectory"],
        "pose_duration_s": run["pose_duration_s"], "summary": run["summary"],
        "qualification": "Complete replay sparse surveyed GCP fit residuals. Same seven points fit and evaluated, scale 1; not dense trajectory ATE, not independent validation, not a map rendering or robot execution.",
    })


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    font = style()
    figures = {"garage-partial-routes": garage_partial(),
               "corridor-rmse-components": corridor_components(),
               "garage-full-gcp-errors": garage_gcps()}
    (OUT / "extra-figures-manifest.json").write_text(json.dumps(
        {"generator": str(Path(__file__).resolve()), "font_file": font,
         "figures": {name: data["size_inches"] for name, data in figures.items()}}, indent=2),
        encoding="utf-8")
    print(json.dumps({"output": str(OUT), "figures": list(figures)}, indent=2))


if __name__ == "__main__":
    main()
