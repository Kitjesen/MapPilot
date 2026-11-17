"""Regenerate publication figures from the frozen scene and route data beside this file."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

ROOT = Path(__file__).resolve().parent
DATA = json.loads((ROOT / "data.json").read_text(encoding="utf-8"))
SCENES = {scene["id"]: scene for scene in DATA["scenes"]}
BLUE, ORANGE, GREEN, GREY = "#0072B2", "#D55E00", "#009E73", "#AAB7C4"
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 8,
    "axes.labelsize": 8, "axes.titlesize": 9, "axes.linewidth": .6,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "lines.linewidth": 1.25,
    "pdf.fonttype": 42, "ps.fonttype": 42, "svg.fonttype": "none",
    "savefig.facecolor": "white", "axes.unicode_minus": True,
})
TITLES = {
    "edge_00": "Observed floor begins at x = 0 m",
    "edge_20": "Observed floor begins at x = 0.20 m",
    "blind_patch": "Unobserved patch under the initial body",
    "open_gap": "Mid-route ground gap (0.50 m)",
    "wall_detour": "Box-obstacle detour",
    "door_080": "Doorway width: 0.80 m", "door_060": "Doorway width: 0.60 m",
    "l_120": "L-shaped stairs (1.20 m wide)", "l_200": "L-shaped stairs (2.00 m wide)",
    "u_120": "U-shaped stairs (1.20 m wide)", "u_200": "U-shaped stairs (2.00 m wide)",
    "three_levels": "Three levels (4.80 m rise)",
    "overlap_linked": "Overlapping floors",
    "approach_20m": "20 m approach followed by stairs",
    "dual_stairs": "Two stair connections", "missing_treads": "Missing stair treads",
    "no_connection": "Disconnected floors", "sealed_entry": "Sealed stair entrance",
    "low_headroom": "Insufficient overhead clearance",
}
for key in SCENES:
    if key.startswith("step_"): TITLES[key] = f"Single step: {int(key[5:])/100:.2f} m"
    elif key.startswith("ramp_"): TITLES[key] = f"Nominal ramp angle: {key[5:]}°"
    elif key.startswith("straight_"):
        _, rise, width = key.split("_")
        TITLES[key] = f"Stairs: {int(rise)/100:.2f} m rise, {int(width)/100:.2f} m width"


def faces(box):
    a, b, c = box["min"]; d, e, f = box["max"]
    vertices = np.array([[a,b,c],[d,b,c],[d,e,c],[a,e,c],
                         [a,b,f],[d,b,f],[d,e,f],[a,e,f]])
    return [vertices[index] for index in
            [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]]


def draw(ax, scene, requests, title, *, omit_walls=False):
    terrain = [b for b in scene["geometry"] if not (omit_walls and b.get("kind") == "wall")]
    for box in terrain:
        wall = box.get("kind") == "wall"
        ceiling = box.get("kind") == "ceiling"
        ax.add_collection3d(Poly3DCollection(
            faces(box), facecolor="#D8A68E" if ceiling else GREY,
            edgecolor="none" if wall else "#6F8396", linewidth=.22,
            alpha=.07 if wall else .28, zorder=1))
    bounds = [b["min"] for b in terrain] + [b["max"] for b in terrain]
    for i, request in enumerate(requests):
        path = request.get("result", {}).get("path", [])
        color = BLUE if i == 0 else ORANGE
        if path:
            p = np.asarray(path)
            ax.plot(*p.T, color=color, linestyle="-" if i == 0 else "--", zorder=5)
            bounds.extend(path)
        if i == 0:
            ax.scatter(*request["start"], color=GREEN, s=13, depthshade=False, zorder=6)
            ax.scatter(*request["goal"], color=ORANGE, s=25, marker="*", depthshade=False, zorder=6)
        bounds.extend([request["start"], request["goal"]])
    lo, hi = np.min(bounds, axis=0), np.max(bounds, axis=0)
    span = hi - lo
    pad = np.maximum(span * .025, .04)
    ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]), ylim=(lo[1]-pad[1],hi[1]+pad[1]),
           zlim=(lo[2]-pad[2],hi[2]+pad[2]), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    ax.set_box_aspect(span + 2*pad)
    ax.set_proj_type("ortho")
    ax.view_init(28, -58)
    ax.set_title(title, pad=2)
    ax.grid(False)
    ax.tick_params(pad=0)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_major_locator(MaxNLocator(3))
        axis.pane.set_alpha(0)
        axis.labelpad = 1
    if span[0] > 5 * max(span[1], .1):
        ax.set_yticks([lo[1], hi[1]])


def legend(fig, reverse=False):
    handles = [Line2D([],[],color=BLUE,label="Body-reference route"),
               Line2D([],[],marker="o",linestyle="none",color=GREEN,markersize=4,label="Forward start" if reverse else "Start"),
               Line2D([],[],marker="*",linestyle="none",color=ORANGE,markersize=6,label="Forward goal" if reverse else "Requested goal")]
    if reverse:
        handles.insert(1,Line2D([],[],color=ORANGE,linestyle="--",label="Reverse route"))
    fig.legend(handles=handles,loc="lower center",bbox_to_anchor=(.5,.005),
               ncol=len(handles),frameon=False,fontsize=7,handlelength=1.5,columnspacing=1.2)


def main():
    for fmt in ("png", "pdf", "svg"):
        (ROOT / fmt).mkdir(exist_ok=True)
    manifest = []
    with PdfPages(ROOT / "figure-atlas.pdf", metadata={"Title":"LingTu static global-planning figures", "Author":"LingTu"}) as atlas:
        def save(fig, name, caption, source_ids):
            labels = [axis.label for ax in fig.axes for axis in
                      ([ax.xaxis, ax.yaxis, ax.zaxis] if hasattr(ax, "zaxis") else [ax.xaxis, ax.yaxis])]
            labels.extend(fig.legends)
            for fmt in ("png", "pdf", "svg"):
                fig.savefig(ROOT/fmt/f"{name}.{fmt}", dpi=600 if fmt == "png" else 150,
                            bbox_inches="tight", bbox_extra_artists=labels, pad_inches=.10)
            atlas.savefig(fig, bbox_inches="tight", bbox_extra_artists=labels, pad_inches=.10)
            plt.close(fig)
            manifest.append(dict(name=name,caption=caption,requests=source_ids))
            print(name, flush=True)

        for name, ids, caption in [
            ("fig01_multilevel",["l_120","u_120","three_levels","overlap_linked"],
             "Static body-reference routes on four multi-level OctoMap scenes. Outer walls are omitted for visibility; axes have equal metric scale. Green circles and orange stars denote the actual starts and requested goals. These results establish geometric planning feasibility, not foothold or physical locomotion performance."),
            ("fig03_steps",["step_35","step_40","step_45","step_50"],
             "Bidirectional single-step probes with the same declared body and 0.45 m step limit. Routes and independent geometry checks pass at 0.35, 0.40 and 0.45 m; no route is returned at 0.50 m. No physical climbing experiment is represented."),
            ("fig04_ramps",["ramp_20","ramp_25","ramp_30","ramp_35"],
             "Bidirectional routes on nominal 20, 25, 30 and 35 degree ramp maps. Map inclination is distinct from individual route-segment slope. The 35 degree scene uses winding routes and took 101.503 s forward and 22.194 s reverse in this run; it is not a real-time or physical-climbing result."),
            ("fig05_obstacles",["wall_detour","door_080","door_060","open_gap"],
             "Obstacle and clearance cases. The box detour and 0.80 m doorway return checked routes. The 0.60 m doorway and mid-route 0.50 m ground gap return no route. Empty results are shown with endpoints only, without invented connecting trajectories."),
        ]:
            fig=plt.figure(figsize=(7.2,5.8))
            for i,key in enumerate(ids):
                scene=SCENES[key]
                requests=scene["requests"][:2] if name in ("fig03_steps","fig04_ramps") else scene["requests"][:1]
                passed=all(r.get("result",{}).get("ok") for r in requests)
                ax=fig.add_subplot(2,2,i+1,projection="3d",computed_zorder=False)
                draw(ax,scene,requests,f"({chr(97+i)}) {TITLES.get(key,key)}\n"+("Route found" if passed else "No route"),omit_walls=name=="fig01_multilevel")
            fig.subplots_adjust(left=.015,right=.98,top=.96,bottom=.10,hspace=.1,wspace=.06)
            legend(fig, name in ("fig03_steps","fig04_ramps"))
            save(fig,name,caption,[f"{key}/{r['id']}" for key in ids for r in SCENES[key]["requests"][:(2 if name in ("fig03_steps","fig04_ramps") else 1)]])

        before=json.loads((ROOT/"startup-before.json").read_text(encoding="utf-8"))
        fig,axes=plt.subplots(2,2,figsize=(7.2,4.6),sharex=True,sharey=True)
        for col,(key,case,gap) in enumerate([("edge_00","01-000",.02),("edge_20","06-000",.22)]):
            old=next(r for r in before if r["scene"]==key and r["id"]==case)
            new=next(r for r in SCENES[key]["requests"] if r["id"]==case)
            for row,path in enumerate([old["path"],new["result"]["path"]]):
                ax=axes[row,col];edge=0 if key=="edge_00" else .2
                ax.add_patch(Rectangle((edge,-.05),2,.05,facecolor=GREY))
                ax.add_patch(Rectangle((-.57,.385),1.10,.4,facecolor="#EEF1F4",edgecolor="#8193A3",linestyle="--",linewidth=.7))
                if path:
                    p=np.asarray(path);ax.plot(p[:,0],p[:,2],color=BLUE)
                ax.scatter(-.02,.435,s=16,color=GREEN,zorder=5)
                ax.annotate("",(-.02,.12),(edge,.12),arrowprops=dict(arrowstyle="|-|",color=ORANGE,lw=.8))
                ax.text((edge-.02)/2,.17,f"{gap*100:.0f} cm",ha="center",color=ORANGE,fontsize=8)
                ax.set_title(f"({chr(97+row*2+col)}) {'Before' if row==0 else 'After'}: {'route found' if path else 'no route'}")
                ax.set(xlim=(-.7,1.2),ylim=(-.12,.9),xlabel="x (m)",ylabel="z (m)")
                ax.set_aspect("equal");ax.spines[["top","right"]].set_visible(False)
        fig.tight_layout(pad=.7)
        save(fig,"fig02_departure","Departure comparison using identical starts and body envelopes. Distances are horizontal offsets from the body reference point to the observed floor edge, not step heights or measured foot gaps. Missing observations inside the initial body footprint are tolerated only on the initial connector; occupancy, no-go and measured lower ground remain checked. The map itself is unchanged.",["edge_00/01-000","edge_20/06-000"])

        for scene in DATA["scenes"]:
            for request in scene["requests"]:
                name=f"scene_{scene['id']}__{request['id']}"
                fig=plt.figure(figsize=(3.6,3.2))
                ax=fig.add_subplot(111,projection="3d",computed_zorder=False)
                found=request.get("result",{}).get("ok",False)
                draw(ax,scene,[request],TITLES.get(scene["id"],scene["id"])+f"\n{request['id']} | "+("Route found" if found else "No route"))
                fig.subplots_adjust(left=.01,right=.97,top=.90,bottom=.12)
                legend(fig)
                save(fig,name,
                     f"{TITLES.get(scene['id'],scene['id'])}; request {request['id']}. Start {request['start']} m; requested goal {request['goal']} m. Recorded status: {request['status']}. Map resolution: {scene['resolution']} m. All coordinates are original stored values. Static planning and geometry only.",
                     [f"{scene['id']}/{request['id']}"])

    assert len(manifest)==122
    with (ROOT/"index.csv").open("w",encoding="utf-8-sig",newline="") as stream:
        writer=csv.writer(stream);writer.writerow(["figure","caption_en","source_requests"])
        for entry in manifest:writer.writerow([entry["name"],entry["caption"],";".join(entry["requests"])])
    (ROOT/"figures.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    html='<!doctype html><meta charset="utf-8"><title>LingTu figure catalogue</title><style>body{font:15px system-ui;max-width:1100px;margin:40px auto;color:#263442}section{margin:35px 0}img{max-width:100%;max-height:650px}a{margin-right:16px}small{color:#586879}</style><h1>LingTu · 论文图目录</h1><p>31 个场景，117 组请求；5 张组合图及全部独立图。PNG 为 600 dpi，PDF/SVG 为矢量。仅表示静态规划与几何结果。</p>'
    for entry in manifest:
        name=entry["name"]
        html+=f'<section><h2>{name}</h2><a href="pdf/{name}.pdf">PDF</a><a href="svg/{name}.svg">SVG</a><a href="png/{name}.png">PNG</a><p><img loading="lazy" src="png/{name}.png" alt="{name}"></p><small>{entry["caption"]}</small></section>'
    (ROOT/"index.html").write_text(html,encoding="utf-8")
    print(f"Exported {len(manifest)} figures in PNG, PDF and SVG.")


if __name__=="__main__":main()
