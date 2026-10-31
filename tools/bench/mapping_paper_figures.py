"""Export dated manuscript figures from retained native experiment data."""
from __future__ import annotations

import csv
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
OUT = ROOT / "build/paper-manuscript-20261021/figures"
WIDTH = 6.85
BLUE = "#23679b"
ORANGE = "#b75b35"
GREY = "#66747e"


def style():
    font = findfont("Arial", fallback_to_default=False)
    plt.rcParams.update({
        "font.family": "Arial", "font.size": 10, "axes.labelsize": 10,
        "axes.titlesize": 10, "xtick.labelsize": 10, "ytick.labelsize": 10,
        "legend.fontsize": 10, "figure.titlesize": 10,
        "pdf.fonttype": 42, "ps.fonttype": 42,
        "axes.linewidth": .6, "axes.edgecolor": "#333333",
        "grid.color": "#d7dde0", "grid.linewidth": .45,
        "savefig.facecolor": "white", "axes.axisbelow": True,
    })
    return font


def save(fig, stem, data):
    fig.canvas.draw()
    for suffix in ("pdf", "png"):
        fig.savefig(OUT / f"{stem}.{suffix}", dpi=300, facecolor="white")
    data.update({"font": {"family": "Arial", "size_pt": 10},
                 "size_inches": list(fig.get_size_inches()),
                 "png_dpi": 300, "figure_pdf_scale_for_manuscript": 1.0})
    (OUT / f"{stem}.json").write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    plt.close(fig)
    return data


def map_plate(name):
    folder = ROOT / "build/dataset-mapping-20261021"
    cloud, information = load_cloud_sample(folder / "maps" / name / "map.pcd", 100_000)
    trajectory_file = folder / "runs" / name / "replay.map.tum"
    trajectory = load_trajectory(trajectory_file)
    low, high = trajectory_focus_bounds(trajectory)
    before = len(cloud)
    cloud = cloud[((cloud >= low) & (cloud <= high)).all(axis=1)]
    xyz = trajectory[:, 1:4]
    figure = plt.figure(figsize=(WIDTH, 5.3))
    ax3 = figure.add_axes([.07, .48, .80, .43], projection="3d")
    norm = matplotlib.colors.Normalize(vmin=low[2], vmax=high[2])
    scatter = ax3.scatter(*cloud.T, c=cloud[:, 2], cmap="cividis", norm=norm,
                          s=.35, alpha=.58, linewidths=0, depthshade=False, rasterized=True)
    ax3.plot(*xyz.T, color=ORANGE, linewidth=.8)
    ax3.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
            xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
    ax3.set_box_aspect(high-low)
    ax3.view_init(elev=26, azim=-55)
    ax3.xaxis.pane.fill = ax3.yaxis.pane.fill = ax3.zaxis.pane.fill = False
    for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        axis.set_major_locator(MaxNLocator(nbins=3))
    if (high[2]-low[2]) / np.max(high-low) < .2:
        ax3.zaxis.set_major_locator(FixedLocator([0.0]))
    ax3.tick_params(pad=1)
    ax3.xaxis.labelpad = 3
    ax3.yaxis.labelpad = 3
    ax3.zaxis.labelpad = 2
    colorbar = figure.colorbar(scatter, cax=figure.add_axes([.91, .56, .015, .27]))
    colorbar.set_label("Map Z (m)", labelpad=5)
    colorbar.locator = MaxNLocator(nbins=3)
    colorbar.update_ticks()
    figure.text(.065, .89, "(a) 3D", ha="left")
    for axes, xy, label, equal in [([.10,.13,.34,.26], (0,1), "(b) Top view", True),
                                   ([.60,.13,.34,.26], (0,2), "(c) Side view", False)]:
        ax = figure.add_axes(axes)
        first, second = xy
        ax.scatter(cloud[:, first], cloud[:, second], c=cloud[:, 2], cmap="cividis", norm=norm,
                   s=.30, alpha=.58, linewidths=0, rasterized=True)
        ax.plot(xyz[:, first], xyz[:, second], color=ORANGE, linewidth=.8)
        ax.set(xlabel=f"{'XYZ'[first]} (m)", ylabel=f"{'XYZ'[second]} (m)",
               xlim=(low[first],high[first]), ylim=(low[second],high[second]))
        if equal:
            ax.set_aspect("equal", adjustable="box")
        ax.set_title(label, loc="left", pad=5)
        ax.spines[["top","right"]].set_visible(False)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.grid(True)
    handles = [Line2D([],[],marker=".",linestyle="none",color=GREY,label="Measured map returns"),
               Line2D([],[],color=ORANGE,linewidth=1,label="Estimated IMU trajectory")]
    figure.legend(handles=handles, loc="upper center", bbox_to_anchor=(.49,.993),
                  frameon=False, ncols=2, columnspacing=1.2, handletextpad=.5)
    return save(figure, f"map-{name}", {
        "source_map": str(folder / "maps" / name / "map.pcd"),
        "source_trajectory": str(trajectory_file), "cloud": information,
        "display_crop": {"rule": "original estimated trajectory XYZ bounds plus 5m",
                         "min_m": low.tolist(), "max_m": high.tolist(),
                         "sampled_before_crop": before, "sampled_after_crop": len(cloud)},
        "axes": "Native XYZ source frame; 3D box and XY use physical aspect. XZ is a coordinate plot with independently labelled axis ranges.",
        "qualification": "Historical Windows baseline map; not the latest probability-plane trajectory, GT or robot execution.",
        "source_changed": False,
    })


def route_figure():
    source = ROOT / "build/dataset-planning-fix-20261021/planning/geode/local_geometry_control/summary.json"
    data = json.loads(source.read_text())
    route = np.asarray(data["result"]["path"], dtype=float)
    difference = np.diff(route, axis=0)
    spacing = np.linalg.norm(difference, axis=1)
    arc = np.r_[0, np.cumsum(spacing)]
    with (OUT / "geode-waypoints.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["index", "x_m", "y_m", "z_m", "arc_length_m", "previous_step_m"])
        for index, point in enumerate(route):
            writer.writerow([index, *point, arc[index], "" if index == 0 else spacing[index-1]])
    figure, axes = plt.subplots(1,2,figsize=(WIDTH,2.85))
    figure.subplots_adjust(left=.105,right=.975,bottom=.22,top=.82,wspace=.40)
    xy, height = axes
    xy.plot(route[:,0], route[:,1], color=BLUE, linewidth=.9, marker="o", markersize=3.3)
    for index in (0,5,10,15,20,25):
        xy.annotate(str(index), route[index,:2], xytext=(0,8 if index%10==0 else -15),
                    textcoords="offset points", ha="center", fontsize=10)
    xy.scatter(*route[0,:2], marker="s", color=ORANGE, s=22,zorder=4)
    xy.scatter(*route[-1,:2], marker="D", color=ORANGE,s=22,zorder=4)
    xpad=.17
    xy.set(xlabel="X (m)", ylabel="Y (m)",
           xlim=(route[:,0].min()-xpad,route[:,0].max()+xpad),
           ylim=(route[:,1].min()-.58,route[:,1].max()+.58))
    xy.set_aspect("equal",adjustable="box")
    figure.text(.105,.90,"(a) Global route vertices",ha="left")
    height.plot(arc, route[:,2], color=BLUE, linewidth=.9, marker="o",markersize=3.1)
    height.set(xlabel="Arc length (m)",ylabel="Z (m)",xlim=(-.04,arc[-1]+.04))
    figure.text(.615,.90,"(b) Height profile",ha="left")
    for ax in axes:
        ax.spines[["top","right"]].set_visible(False)
        ax.grid(True)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=4))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
    return save(figure,"geode-waypoints",{
        "source":str(source),"path":route.tolist(),"arc_length_m":arc.tolist(),
        "point_count":len(route),"edge_count":len(spacing),"length_m":float(arc[-1]),
        "spacing_min_m":float(spacing.min()),"spacing_median_m":float(np.median(spacing)),
        "spacing_max_m":float(spacing.max()),
        "max_slope":float(np.max(np.abs(difference[:,2])/np.linalg.norm(difference[:,:2],axis=1))),
        "goal_error_m":data["result"]["diagnostics"]["goal_error_m"],
        "verification":data["verification"],
        "qualification":"26 graph-route vertices, not SCAN guide count, spline control points or executed commands. Exact plotted data, no smoothing.",
        "execution_validated":False,
    })


def accuracy_figure():
    source = ROOT / "build/prob-plane-20261021/isotropic/evaluation.json"
    experiment = json.loads(source.read_text())
    figure, axes = plt.subplots(2,2,figsize=(WIDTH,3.8))
    figure.subplots_adjust(left=.11,right=.97,bottom=.19,top=.84,wspace=.30,hspace=.77)
    cases = [("geode","imu_frame_assumption","(a) GEODE / IMU"),
             ("geode","lidar_frame_assumption","(b) GEODE / LiDAR"),
             ("slippage","imu_frame","(c) Slippage"),
             ("corridor","imu_frame","(d) Corridor02")]
    measured=[]
    for ax,(case,hypothesis,title) in zip(axes.flat,cases):
        record=experiment["cases"][case]
        scores=record["scores"][hypothesis]["common"]
        values=[scores[key]["rmse_m"] for key in ("prior","fresh")]
        ax.barh([1,0],values,height=.45,color=[GREY,BLUE])
        maximum=max(values)
        for y,value in zip([1,0],values):
            label=f"{value:.5f}" if case=="slippage" else f"{value:.3f}"
            ax.text(value+maximum*.035,y,label,va="center",fontsize=10)
        ax.set_xlim(0,maximum*1.29)
        ax.set_yticks([])
        ax.set_ylim(-.6,1.6)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.grid(axis="x")
        ax.spines[["top","right","left"]].set_visible(False)
        ax.set_xlabel("Position RMSE (m)",labelpad=3)
        ax.set_title(title+f"  (N={record['common_reference_samples']})",loc="left",pad=7)
        measured.append({"case":case,"hypothesis":hypothesis,"samples":record["common_reference_samples"],
                         "baseline":scores["prior"],"accepted":scores["fresh"]})
    figure.legend(handles=[Line2D([],[],color=GREY,linewidth=5,label="Time-consistent baseline"),
                           Line2D([],[],color=BLUE,linewidth=5,label="Accepted probability planes")],
                  loc="upper center",bbox_to_anchor=(.5,.99),frameon=False,ncols=2,
                  columnspacing=1,handlelength=1.5,handletextpad=.5)
    corridor=experiment["cases"]["corridor"]["scores"]["imu_frame"]["common"]
    z=[corridor[key]["height_rmse_m"] for key in ("prior","fresh")]
    figure.text(.11,.045,f"Corridor02 Z RMSE: {z[0]:.3f} to {z[1]:.3f} m (higher).",fontsize=10)
    return save(figure,"precision-comparison",{
        "source":str(source),"cases":measured,
        "corridor_height_rmse_m":{"baseline":z[0],"accepted":z[1]},
        "qualification":"Fixed common reference times, scale=1 SE3; independent linear x-ranges from zero. GEODE IMU/LiDAR are separate reference-point hypotheses, not independent experiments. Position improvement does not imply every axis improves.",
        "missing_metrics":"No p95 field in this source; no synthetic metric added.",
        "execution_validated":False,
    })


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    font=style()
    outputs={"font_file":font,"maps":{},"route":{},"accuracy":{}}
    for name in ("corridor","geode"):
        outputs["maps"][name]=map_plate(name)
    outputs["route"]=route_figure()
    outputs["accuracy"]=accuracy_figure()
    (OUT/"manifest.json").write_text(json.dumps(outputs,ensure_ascii=False,indent=2),encoding="utf-8")
    (OUT/"README.md").write_text(
        "# Manuscript figures\n\nAll four plates use Arial 10 pt at native 6.85 inch width. "
        "PNG is 300 dpi; PDF labels remain vector, while map scatter layers are rasterized at 300 dpi. "
        "Source points are deterministically sampled then cropped to the baseline estimated trajectory XYZ bounds plus 5 m. "
        "The crop and sampled counts are recorded in each JSON. Full maps and earlier uncombined views remain in the original experiment.\n\n"
        "Map panels preserve source coordinates. 3D uses physical XYZ aspect, XY uses equal aspect; "
        "XZ is a coordinate plot with independent labeled ranges. These are historical baseline maps, "
        "not maps generated with the newer probability-plane estimator.\n\n"
        "The route has exactly 26 global graph vertices; its point count is not a deployed SCAN reference/control count. "
        "The accuracy plate uses the accepted paired probability-plane evaluation only; GEODE reference hypotheses are separate. "
        "Corridor vertical RMSE increases despite lower total position RMSE. No execution qualification is inferred.\n\n"
        "Reproduce from repository root: `.venv/Scripts/python.exe tools/bench/mapping_paper_figures.py`. "
        "No production source, input data or planning result is modified.\n",encoding="utf-8")
    print(json.dumps({"output":str(OUT),"plates":4,"font":font},ensure_ascii=False))


if __name__=="__main__":
    main()
