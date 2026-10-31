"""Render native global -> Executor -> SCAN -> Follower execution evidence.

CLI: python tools/bench/body_execution_view.py --out build/body-execution-20261022
Use --runs PATH to render a retained trial without overwriting its raw data.
Kinematic closed-loop traces must not be labelled physical simulation.
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from pose_view import LABELS, SCENES, paper_style, part_polygons, world

COLORS = {"legacy": "#657e91", "body": "#28795e"}


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def array(rows: list[dict], keys: tuple[str, ...]) -> np.ndarray:
    return np.asarray([[float(row[key]) for key in keys] for row in rows], dtype=float).reshape(-1, len(keys))


def load(root: Path, runs: Path, reference: Path) -> tuple[dict, dict]:
    model = json.loads((reference / "model.json").read_text(encoding="utf-8"))
    cases = {}
    for name in SCENES:
        scene = json.loads((reference / "runs" / name / "scene.json").read_text(encoding="utf-8"))
        variants = {}
        for variant in COLORS:
            folder = runs / name / variant
            summary_path = folder / "summary.json"
            result = (json.loads(summary_path.read_text(encoding="utf-8"))
                      if summary_path.exists() else {"reason": "not_run", "incomplete": True})
            variants[variant] = {
                "summary": result,
                "states": read_csv(folder / "states.csv"),
                "global": read_csv(folder / "global.csv"),
                "folder": folder,
            }
        cases[name] = {"scene": scene, "variants": variants}
    return model, cases


def status(summary: dict) -> str:
    if summary.get("incomplete"):
        return "Not run"
    if not summary.get("global_ok"):
        return f"Global rejected: {summary.get('reason', 'unknown')}"
    if summary.get("reached"):
        return "Reached"
    return f"Execution failed: {summary.get('reason', 'unknown')}"


def stats(summary: dict) -> str:
    if not summary.get("execution_started"):
        return status(summary)
    return (f"{status(summary)} | travelled {summary.get('travel_m', 0):.3f} m\n"
            f"XY error {summary.get('goal_error_m', 0):.3f} m | "
            f"max control tick {summary.get('max_tick_ms', 0):.1f} ms")


def outline(ax, polygons, pose, color, alpha=.27) -> None:
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    for _, vertices in polygons:
        ax.add_patch(Polygon(vertices @ rotation.T + pose[:2], facecolor=color,
                             edgecolor=color, lw=.4, alpha=alpha, zorder=5))
    ax.arrow(pose[0], pose[1], .23*cosine, .23*sine, color=color,
             width=.008, head_width=.055, length_includes_head=True, zorder=7)


def figures(out: Path, model: dict, cases: dict) -> None:
    polygons = part_polygons(model)
    fig, axes = plt.subplots(4, 2, figsize=(11.6, 16.0), constrained_layout=True)
    for row, (name, case) in enumerate(cases.items()):
        for column, (variant, data) in enumerate(case["variants"].items()):
            ax = axes[row, column]
            world(ax, case["scene"])
            color = COLORS[variant]
            planned = array(data["global"], ("x", "y"))
            actual = array(data["states"], ("x", "y", "yaw"))
            if len(planned):
                ax.plot(planned[:, 0], planned[:, 1], "--", color=color, lw=1.2)
            if len(actual):
                ax.plot(actual[:, 0], actual[:, 1], color=color, lw=1.8)
                for index in np.unique(np.linspace(0, len(actual)-1, min(4, len(actual))).astype(int)):
                    outline(ax, polygons, actual[index], color, .14)
                ax.scatter(actual[-1, 0], actual[-1, 1], marker="s", s=18, color=color, zorder=8)
            elif data["summary"].get("execution_started"):
                ax.text(.5, .5, "No recorded execution states", ha="center", transform=ax.transAxes)
            else:
                ax.text(.5, .5, status(data["summary"]), ha="center", wrap=True,
                        bbox={"facecolor": "white", "alpha": .92, "edgecolor": "none"},
                        transform=ax.transAxes)
            label = "Current XYZ + SCAN" if variant == "legacy" else "URDF pose route + SCAN"
            ax.set_title(f"{LABELS[name]}\n{label}\n{stats(data['summary'])}", pad=8)
    legend = fig.legend(handles=[Line2D([], [], color="#435363", linestyle="--", label="Global planned route"),
                        Line2D([], [], color="#435363", label="Recorded actual route"),
                        Line2D([], [], marker="s", color="#435363", linestyle="none", label="Last recorded pose")],
               loc="upper center", bbox_to_anchor=(.5, .963), ncol=3, frameon=False)
    title = fig.suptitle("Native closed-loop component comparison — kinematic state integration\n"
                        "Executor + SCAN + Follower; no gait, physics or field validation", fontsize=10, y=.994)
    legend.set_in_layout(False)
    title.set_in_layout(False)
    fig.get_layout_engine().set(rect=(0, 0, 1, .941))
    fig.savefig(out / "comparison.png", dpi=180)
    fig.savefig(out / "comparison.pdf")
    plt.close(fig)
    for name, case in cases.items():
        fig, axes = plt.subplots(1, 2, figsize=(10, 4.8), constrained_layout=True)
        for ax, (variant, data) in zip(axes, case["variants"].items(), strict=True):
            world(ax, case["scene"])
            color = COLORS[variant]
            planned = array(data["global"], ("x", "y"))
            actual = array(data["states"], ("x", "y", "yaw"))
            if len(planned):
                ax.plot(planned[:, 0], planned[:, 1], "--", color=color, lw=1,
                        label="Global reference")
            if len(actual):
                ax.plot(actual[:, 0], actual[:, 1], color=color, lw=1.8,
                        label="Closed-loop trace")
                outline(ax, polygons, actual[-1], color)
            ax.set_title(("Current XYZ + SCAN" if variant == "legacy" else
                          "URDF pose route + SCAN") + "\n" + stats(data["summary"]))
            if len(actual):
                ax.legend(loc="upper left", frameon=False)
        fig.suptitle(LABELS[name] + " — kinematic feedback, no gait physics", fontsize=10)
        fig.savefig(out / f"{name}.png", dpi=180)
        fig.savefig(out / f"{name}.pdf")
        plt.close(fig)


def animation(out: Path, model: dict, cases: dict) -> None:
    polygons = part_polygons(model)
    fig, axes = plt.subplots(2, 2, figsize=(10.0, 9.0), constrained_layout=True)
    tracks = []
    duration = 0.0
    for ax, (name, case) in zip(axes.flat, cases.items(), strict=True):
        world(ax, case["scene"])
        ax.set_title(LABELS[name])
        for variant, data in case["variants"].items():
            color = COLORS[variant]
            planned = array(data["global"], ("x", "y"))
            if len(planned):
                ax.plot(planned[:, 0], planned[:, 1], "--", color=color, lw=.8, alpha=.6)
            states = array(data["states"], ("t", "x", "y", "yaw"))
            if not len(states):
                continue
            states[:, 0] -= states[0, 0]
            duration = max(duration, states[-1, 0])
            line, = ax.plot([], [], color=color, lw=1.5)
            patches = [ax.add_patch(Polygon(np.zeros_like(vertices), closed=True, facecolor=color,
                                           edgecolor=color, lw=.4, alpha=.28)) for _, vertices in polygons]
            arrow, = ax.plot([], [], color=color, lw=2)
            tracks.append((states, line, patches, arrow))
        summaries = case["variants"]
        ax.text(.02, .02, "XYZ: " + status(summaries["legacy"]["summary"]) + "\nPose: " +
                status(summaries["body"]["summary"]), transform=ax.transAxes, fontsize=10,
                bbox={"facecolor": "white", "alpha": .88, "edgecolor": "none"})
    label = fig.suptitle("Recorded closed-loop execution — kinematic, no physics", fontsize=10)

    def update(elapsed):
        for states, line, patches, arrow in tracks:
            index = max(0, int(np.searchsorted(states[:, 0], elapsed, side="right")-1))
            pose = states[index, 1:]
            line.set_data(states[:index+1, 1], states[:index+1, 2])
            cosine, sine = math.cos(pose[2]), math.sin(pose[2])
            rotation = np.array([[cosine, -sine], [sine, cosine]])
            for patch, (_, vertices) in zip(patches, polygons, strict=True):
                patch.set_xy(vertices @ rotation.T + pose[:2])
            arrow.set_data([pose[0], pose[0]+.23*cosine], [pose[1], pose[1]+.23*sine])
        label.set_text(f"Recorded closed-loop execution — t = {elapsed:.1f} s\n"
                       "Blue: current XYZ; green: URDF pose route. Kinematic, no physics.")

    anim = FuncAnimation(fig, update, frames=np.linspace(0, max(.1, duration), 120), interval=100)
    anim.save(out / "closed_loop.gif", writer=PillowWriter(fps=10), dpi=95)
    plt.close(fig)


def repair_figure(out: Path, model: dict, cases: dict) -> None:
    initial = out / "runs-initial/corner/body"
    if not (initial / "summary.json").exists():
        return
    before = {"summary": json.loads((initial / "summary.json").read_text()),
              "states": read_csv(initial / "states.csv"),
              "global": read_csv(initial / "global.csv")}
    after = cases["corner"]["variants"]["body"]
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.5), constrained_layout=True)
    for ax, data, title, color in zip(axes, (before, after),
                                     ("Before: fitted endpoint stops short", "After: exact stage endpoint"),
                                     ("#a56449", COLORS["body"]), strict=True):
        world(ax, cases["corner"]["scene"])
        planned = array(data["global"], ("x", "y"))
        actual = array(data["states"], ("x", "y", "yaw"))
        ax.plot(planned[:, 0], planned[:, 1], "--", color="#78868c", lw=1, label="Global reference")
        ax.plot(actual[:, 0], actual[:, 1], color=color, lw=1.8, label="Closed-loop trace")
        outline(ax, part_polygons(model), actual[-1], color)
        ax.set_title(title + "\n" + status(data["summary"]) +
                     f" | XY error {data['summary']['goal_error_m']:.3f} m")
        ax.legend(loc="upper left", frameon=False)
    fig.suptitle("Same URDF model, map and turn tolerance — kinematic closed-loop feedback", fontsize=10)
    fig.savefig(out / "corner_fix.png", dpi=200)
    fig.savefig(out / "corner_fix.pdf")
    plt.close(fig)


def report(out: Path, runs: Path, cases: dict) -> None:
    rows = []
    for name, case in cases.items():
        for variant, data in case["variants"].items():
            s = data["summary"]
            cells = [name, variant, status(s)]
            cells += [f"{s[key]:.3f}" if key in s else "—" for key in
                      ("travel_m", "goal_error_m", "elapsed_sim_s", "global_ms", "max_tick_ms")]
            rows.append("<tr>" + "".join(f"<td>{html.escape(cell)}</td>" for cell in cells) + "</tr>")
    page = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Native closed-loop execution</title>
<style>body{font:16px Arial,sans-serif;color:#26382f;background:#f6f7f4;margin:32px auto;max-width:1120px;padding:0 22px}
h1{font-size:28px}p{line-height:1.7}img{max-width:100%;background:white}table{border-collapse:collapse;width:100%;background:white}
td,th{text-align:left;padding:11px 9px;border-bottom:1px solid #dce2dc;font-size:14px}a{color:#28795e}</style>
<h1>原生全局规划 → SCAN → 跟踪闭环</h1>
<p>本页为运动学闭环：每一帧以反馈状态调用真实 Executor、SCAN 和 Follower，再积分经过机体与支撑检查的速度。
它不同于全局路径插值动画，但没有步态与物理接触，不能作为 MuJoCo 或实机导航成功证据。失败与未执行案例完整保留。</p>
<p>蓝色：当前 XYZ＋SCAN。绿色：URDF 分部模型＋姿态路线＋SCAN。虚线：规划路线；实线：闭环实际轨迹。</p>
<p><a href="comparison.pdf">论文图 PDF</a> · <a href="comparison.png">PNG</a> · <a href="closed_loop.gif">闭环回放 GIF</a></p>
<img src="closed_loop.gif" alt="Recorded kinematic closed-loop execution">
<table><thead><tr><th>Case</th><th>Variant</th><th>Result</th><th>Travel (m)</th><th>XY error (m)</th><th>Execution (s)</th><th>Global search (ms)</th><th>Max control tick (ms)</th></tr></thead><tbody>ROWS</tbody></table>
<p>最大 control tick 是单次 Executor 调用与当前基准最终检查的总墙钟时间；它不等同于 SCAN 优化耗时，也不能用作其 p99。
所有科研图 Arial 10 pt。数据目录：RUNS。</p>
<p><a href="detour.pdf">直穿门</a> · <a href="align.pdf">先转向再穿门</a> · <a href="corner.pdf">L 形转角</a> · <a href="too_narrow.pdf">过窄负例</a></p>
EVIDENCE<img src="comparison.png" alt="Planned and actual routes"></html>"""
    evidence = ""
    if (out / "physics/README.md").exists():
        evidence = ('<p><a href="README.md">完整记录和复现命令</a> · '
                    '<a href="initial-view/index.html">修复前失败结果</a> · '
                    '<a href="physics/README.md">Thunder 物理失败记录</a> · '
                    '<a href="physics/route-only/physics.mp4">实际物理视频</a></p>'
                    '<p>初轮 L 形转角因样条终点偏离转向点约 18 mm 停住，修复精确边界后复测到达。'
                    '完整 Thunder 物理模型仍因轮柱外接盒与地面交叠而拒绝运动。'
                    '固定关节与 yaw 模型、动态障碍组合、正式端点部署和实时性能仍有限制；详情见完整记录。</p>')
    page = page.replace("EVIDENCE", evidence)
    (out / "index.html").write_text(page.replace("ROWS", "".join(rows)).replace("RUNS", html.escape(str(runs))), encoding="utf-8")
    (out / "view.md").write_text(
        "# Closed-loop evidence renderer\n\n"
        "Run `.venv/Scripts/python.exe tools/bench/body_execution_view.py --out build/body-execution-20261022`.\n"
        "Use `--runs PATH --out OTHER_PATH` for retained trial comparisons. `--no-animation` skips GIF rendering.\n\n"
        "Input: each `runs/CASE/{legacy,body}/summary.json`, `states.csv`, `global.csv`; original scenes/model from "
        "`build/native-body-routes-20261022`. Missing trials are marked not run. Failed traces remain visible.\n\n"
        "Output: comparison.png/pdf, closed_loop.gif, index.html. Plots use Arial 10 pt. "
        "Animation samples recorded feedback states at common elapsed times and holds completed trials at their last state. "
        "This report labels the current runner's internal mode as kinematic; physical runs require separate rendering.\n",
        encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("build/body-execution-20261022"))
    parser.add_argument("--runs", type=Path)
    parser.add_argument("--reference", type=Path, default=Path("build/native-body-routes-20261022"))
    parser.add_argument("--no-animation", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    runs = args.runs or args.out / "runs"
    model, cases = load(args.out, runs, args.reference)
    paper_style()
    figures(args.out, model, cases)
    repair_figure(args.out, model, cases)
    if not args.no_animation:
        animation(args.out, model, cases)
    report(args.out, runs, cases)


if __name__ == "__main__":
    main()
