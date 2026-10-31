"""Render retained native XYZ / URDF-pose routes without inventing motion.

Inputs are the native demo's JSON outputs. The GIF interpolates geometric
poses only; no gait, controller, elapsed-time comparison or hardware is shown.
"""
from __future__ import annotations

import argparse
import html
import json
import math
from itertools import pairwise, product
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon, Rectangle
from scipy.spatial import ConvexHull

SCENES = ("detour", "align", "corner", "too_narrow")
LABELS = {
    "detour": "Two doorways: detour versus direct route",
    "align": "0.60 m doorway: align before entry",
    "corner": "L corridor: translation and body heading",
    "too_narrow": "0.28 m corridor: fixed posture rejected",
}
COLORS = {"before": "#60798a", "after": "#2f8067"}


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def load(out: Path) -> tuple[dict, dict]:
    model = read_json(out / "model.json")
    cases = {}
    for name in SCENES:
        folder = out / "runs" / name
        case = {key: read_json(folder / f"{key}.json") for key in ("scene", "before", "after")}
        for variant in ("before", "after"):
            result = case[variant]
            if result["ok"] and not result["path"]:
                raise ValueError(f"{name}/{variant}: successful native result contains no path")
            if not result["ok"] and result["path"]:
                raise ValueError(f"{name}/{variant}: rejected native result contains a path")
        if any(point[3] is None for point in case["after"]["path"]):
            raise ValueError(f"{name}: native pose route is missing a body heading")
        cases[name] = case
    return model, cases


def paper_style() -> None:
    plt.rcParams.update({
        "font.family": "Arial", "font.size": 10, "axes.titlesize": 10,
        "axes.labelsize": 10, "xtick.labelsize": 10, "ytick.labelsize": 10,
        "legend.fontsize": 10, "pdf.fonttype": 42,
    })


def part_polygons(model: dict) -> list[tuple[str, np.ndarray]]:
    polygons = []
    corners = np.asarray(list(product((-1., 1.), repeat=3)))
    for part in model["parts"]:
        points = ((corners * np.asarray(part["half_size"])) @ np.asarray(part["rotation"]).T
                  + np.asarray(part["center"]))[:, :2]
        polygons.append((part["link"], points[ConvexHull(points).vertices]))
    return polygons


def body(ax, polygons: list[tuple[str, np.ndarray]], pose, alpha=.26) -> None:
    yaw = float(pose[3])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    for link, polygon in polygons:
        points = polygon @ rotation.T + np.asarray(pose[:2])
        color = "#ae7b3b" if link.startswith(("livox", "mid360")) else COLORS["after"]
        ax.add_patch(Polygon(points, facecolor=color, edgecolor=color, alpha=alpha, lw=.55))


def world(ax, scene: dict) -> None:
    for obstacle in scene["obstacles"]:
        low, high = np.asarray(obstacle["low"]), np.asarray(obstacle["high"])
        ax.add_patch(Rectangle(low[:2], high[0]-low[0], high[1]-low[1],
                               facecolor="#dedfda", edgecolor="#9a9e96", lw=.65))
    low_x, high_x, low_y, high_y = scene["bounds"]
    ax.set(xlim=(low_x, high_x), ylim=(low_y, high_y), aspect="equal",
           xlabel="X (m)", ylabel="Y (m)")
    ax.grid(alpha=.13, linewidth=.6)
    ax.scatter([scene["start"][0]], [scene["start"][1]], s=22, color="#314e61", zorder=6)
    ax.scatter([scene["goal"][0]], [scene["goal"][1]], s=30, color="#b27c35", marker="*", zorder=6)


def metrics(result: dict) -> str:
    if not result["ok"]:
        return f"No route / {result['reason']}"
    text = f"{result['length_m']:.3f} m / {result['points']} points / {result['elapsed_ms']:.1f} ms"
    if result.get("body_sweep_checked") and not result.get("body_sweep_valid"):
        text += " / sweep FAILED"
    return text


def draw_path(ax, result: dict, variant: str, polygons, scene: dict, snapshots=True) -> None:
    if not result["ok"]:
        if variant == "before":
            ax.add_patch(Circle(scene["start"][:2], .44, facecolor=COLORS[variant],
                                edgecolor=COLORS[variant], alpha=.18, lw=.9))
        else:
            body(ax, polygons, scene["start"])
        return
    path = np.asarray(result["path"], dtype=float)
    ax.plot(path[:, 0], path[:, 1], color=COLORS[variant], lw=1.7,
            label="Historical XYZ circle" if variant == "before" else "URDF OBB + yaw", zorder=4)
    if variant == "before":
        if snapshots:
            point = path[len(path)//2]
            ax.add_patch(Circle(point[:2], .44, facecolor=COLORS[variant],
                                edgecolor=COLORS[variant], alpha=.18, lw=.9))
        return
    # Arrows always use the native heading, including lateral translations.
    for point in path[::max(1, len(path)//7)]:
        ax.arrow(point[0], point[1], .12*math.cos(point[3]), .12*math.sin(point[3]),
                 width=.005, head_width=.045, length_includes_head=True,
                 color=COLORS[variant], zorder=5)
    if snapshots:
        body(ax, polygons, path[len(path)//2])


def case_figures(out: Path, cases: dict, polygons) -> None:
    for name, case in cases.items():
        height = 3.6 if name == "align" else 4.6
        fig, axes = plt.subplots(1, 2, figsize=(8.5, height), layout="constrained")
        for ax, variant in zip(axes, ("before", "after")):
            world(ax, case["scene"])
            draw_path(ax, case[variant], variant, polygons, case["scene"])
            label = "Before: historical XYZ (r = 0.44 m)" if variant == "before" else "After: URDF OBB + body yaw"
            ax.set_title(f"{label}\n{metrics(case[variant])}", pad=8)
        fig.suptitle(LABELS[name], fontsize=10)
        for suffix in ("png", "pdf"):
            fig.savefig(out / f"{name}_comparison.{suffix}", dpi=220, bbox_inches="tight")
        plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(8.5, 8.4), layout="constrained")
    for ax, (name, case) in zip(axes.flat, cases.items()):
        world(ax, case["scene"])
        for variant in ("before", "after"):
            result = case[variant]
            if result["ok"]:
                draw_path(ax, result, variant, polygons, case["scene"], snapshots=False)
        before = f"{case['before']['length_m']:.3f} m" if case["before"]["ok"] else "no route"
        after = f"{case['after']['length_m']:.3f} m" if case["after"]["ok"] else "no route"
        ax.set_title(f"{LABELS[name]}\nBefore: {before} / after: {after}", pad=8)
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, loc="upper center", frameon=True, framealpha=.92,
                      borderpad=.3, handlelength=1.4)
    for suffix in ("png", "pdf"):
        fig.savefig(out / f"summary_overview.{suffix}", dpi=220, bbox_inches="tight")
    plt.close(fig)


def interpolate(path: list, fraction: float, headed: bool) -> list:
    if len(path) == 1:
        return path[0]
    weights = []
    for first, second in pairwise(path):
        translation = math.dist(first[:3], second[:3])
        yaw = abs(math.remainder(second[3]-first[3], 2*math.pi)) if headed else 0.
        weights.append(translation + .4*yaw)
    cumulative = np.concatenate(([0.], np.cumsum(weights)))
    distance = fraction*cumulative[-1]
    index = min(max(0, int(np.searchsorted(cumulative, distance, side="right")-1)), len(path)-2)
    ratio = (distance-cumulative[index])/weights[index] if weights[index] > 0 else 0.
    first, second = path[index], path[index+1]
    xyz = [first[i]+ratio*(second[i]-first[i]) for i in range(3)]
    yaw = first[3]+ratio*math.remainder(second[3]-first[3], 2*math.pi) if headed else None
    return [*xyz, yaw]


def animation(out: Path, cases: dict, polygons) -> None:
    chosen = ("detour", "align", "corner")
    frames_per_scene = 55
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 4.5), layout="constrained")

    def frame(number):
        name = chosen[number//frames_per_scene]
        case = cases[name]
        fraction = (number % frames_per_scene)/(frames_per_scene-1)
        for ax, variant in zip(axes, ("before", "after")):
            ax.clear()
            world(ax, case["scene"])
            result = case[variant]
            if result["ok"]:
                path = np.asarray(result["path"], dtype=float)
                ax.plot(path[:, 0], path[:, 1], color=COLORS[variant], lw=1.1, alpha=.65)
                pose = interpolate(result["path"], fraction, variant == "after")
                if variant == "before":
                    ax.add_patch(Circle(pose[:2], .44, facecolor=COLORS[variant],
                                        edgecolor=COLORS[variant], alpha=.25))
                else:
                    body(ax, polygons, pose, alpha=.45)
                    ax.arrow(pose[0], pose[1], .18*math.cos(pose[3]), .18*math.sin(pose[3]),
                             width=.008, head_width=.05, color=COLORS[variant], length_includes_head=True)
                status = "historical XYZ circle" if variant == "before" else "URDF OBB + body yaw"
            else:
                status = "No route"
                ax.text(.5, .5, result["reason"], transform=ax.transAxes, ha="center", va="center",
                        bbox={"facecolor":"white", "alpha":.9, "edgecolor":"none"})
            ax.set_title(f"{'Before' if variant == 'before' else 'After'} / {status}")
        fig.suptitle(f"{LABELS[name]}\nGeometry playback: fixed joints; no controller or gait", fontsize=10)
        return []

    movie = FuncAnimation(fig, frame, frames=frames_per_scene*len(chosen), interval=100)
    movie.save(out / "native_routes.gif", writer=PillowWriter(fps=10), dpi=100)
    plt.close(fig)


def page(out: Path, model: dict, cases: dict) -> None:
    rows, sections = [], []
    for name, case in cases.items():
        cells = []
        for variant in ("before", "after"):
            result = case[variant]
            cell = f"{result['length_m']:.3f} m · {result['points']} 点" if result["ok"] else "未返回路径"
            if variant == "after" and result["ok"] and not result.get("body_sweep_valid"):
                cell += " · 扫掠未通过"
            cells.append(cell)
        rows.append(f"<tr><td>{html.escape(LABELS[name])}</td><td>{cells[0]}</td><td>{cells[1]}</td></tr>")
        sections.append(f'<figure><img src="{name}_comparison.png" alt="{name} before and after">'
                        f'<figcaption>{html.escape(LABELS[name])} · <a href="{name}_comparison.pdf">PDF</a> · '
                        f'<a href="runs/{name}/scene.json">场景</a> · <a href="runs/{name}/before.json">修改前</a> · '
                        f'<a href="runs/{name}/after.json">修改后</a></figcaption></figure>')
    text = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LingTu · 原生全局规划姿态路线对比</title><style>
body{margin:0;background:#f5f5f2;color:#263034;font:16px/1.7 Arial,"Microsoft YaHei",sans-serif}
main{max-width:1080px;margin:auto;padding:44px 26px}h1{font-size:29px;font-weight:550;margin:8px 0}h2{font-size:20px;margin:34px 0 12px}
p{max-width:960px}.muted,small{color:#6c7473}figure{margin:22px 0;padding:12px;background:white;border:1px solid #e0e3dd}
img{display:block;width:100%;height:auto}figcaption{font-size:14px;color:#677270;margin:10px 8px 2px}a{color:#3e6877}
table{border-collapse:collapse;width:100%;font-size:15px}th,td{text-align:left;padding:12px 8px;border-bottom:1px solid #d9ded7}
nav{display:flex;gap:22px;flex-wrap:wrap;margin:22px 0}footer{border-top:1px solid #d9ded7;margin-top:36px;padding-top:16px;color:#677270;font-size:14px}
@media(max-width:650px){main{padding:22px 12px}figure{padding:3px}th,td{padding:9px 4px;font-size:12px}}</style>
<main><small>LINGTU / NATIVE GLOBAL PLANNING / 2026.10.22</small><h1>同一场景，修改前后的全局路线</h1>
<p class="muted">合成三维 OctoMap 场景。左侧为本实验保留的历史 XYZ＋0.44 m 外接圆基线，不代表当前正式配置；右侧使用本轮原生 XYZ＋yaw 搜索与 PARTS 个 URDF 分部保守盒体。C++ 执行几何查询和搜索。</p>
<nav><a href="#overview">路线总览</a><a href="#playback">几何回放</a><a href="#cases">逐例对比</a><a href="model.json">模型输入</a><a href="README.md">过程记录</a></nav>
<table><thead><tr><th>场景</th><th>修改前</th><th>修改后</th></tr></thead><tbody>ROWS</tbody></table>
<h2 id="overview">路线总览</h2><figure><img src="summary_overview.png"><figcaption>线条只来自实际返回路径；未返回路径的案例明确标注。箭头使用原生结果中的身体朝向，允许移动方向与身体朝向不同。返回路线扫掠复查使用同一 BodyQuery、重新查询且不复用搜索缓存，不是独立数值算法。<a href="summary_overview.pdf">PDF</a></figcaption></figure>
<h2 id="playback">几何姿态回放</h2>PLAYBACK
<h2 id="cases">逐例保留输入与结果</h2>SECTIONS
<footer>图表 Arial 10 pt，PNG / PDF 同源。实验验证全局几何姿态路径；未验证 SCAN 轨迹执行、物理行走或实机导航。当前模型仍有附件与运动范围待补齐。</footer></main></html>'''
    text = text.replace("PARTS", str(model["primitive_count"])).replace("ROWS", "".join(rows)).replace("SECTIONS", "".join(sections))
    playback = ('<figure><img src="native_routes.gif"><figcaption>左侧圆表示旧包络；右侧分部形状按原生 yaw 旋转。'
                '各自按路径进度播放，不能用于速度对比。关节固定，没有步态或运动控制器。</figcaption></figure>'
                if (out / "native_routes.gif").exists() else '<p class="muted">当前仅生成静态图，几何回放尚未生成。</p>')
    text = text.replace("PLAYBACK", playback)
    (out / "index.html").write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-animation", action="store_true", help="Regenerate static figures and page only")
    args = parser.parse_args()
    model, cases = load(args.out)
    paper_style()
    polygons = part_polygons(model)
    case_figures(args.out, cases, polygons)
    if not args.no_animation:
        animation(args.out, cases, polygons)
    page(args.out, model, cases)
    print(json.dumps({"page": str(args.out / "index.html"), "scenes": list(cases),
                      "font": "Arial 10 pt", "physical_execution": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
