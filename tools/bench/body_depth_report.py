"""Summarize paired native Body planning runs without treating timeouts as no-path proofs."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path


def plot_routes(root: Path) -> None:
    import math

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    plt.rcParams.update({"font.family": "Arial", "font.size": 10})
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), constrained_layout=True)
    for row, scene in enumerate(("align", "corner")):
        meta = json.loads((root / "updated" / scene / "scene.json").read_text())
        for col, model in enumerate(("obb", "parts")):
            ax = axes[row, col]
            for obstacle in meta["obstacles"]:
                low, high = obstacle["low"], obstacle["high"]
                ax.add_patch(Rectangle(low[:2], high[0]-low[0], high[1]-low[1],
                                       color="#d9dde1", linewidth=0))
            result = json.loads((root / "updated" / scene / f"{model}_0.json").read_text())
            path = result["path"]
            if path:
                ax.plot([p[0] for p in path], [p[1] for p in path], color="#246b91", lw=1.8)
                for point in path[::max(1, len(path)//10)]:
                    ax.arrow(point[0], point[1], .14*math.cos(point[3]), .14*math.sin(point[3]),
                             width=.006, head_width=.055, color="#246b91", length_includes_head=True)
            else:
                ax.text(.5, .5, "No route returned", transform=ax.transAxes,
                        ha="center", bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
            for label, marker in (("start", "o"), ("goal", "*")):
                point = meta[label]
                ax.scatter(point[0], point[1], s=45, marker=marker, color="#222222", zorder=5)
            xmin, xmax, ymin, ymax = meta["bounds"]
            ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel="X (m)", ylabel="Y (m)")
            ax.set_aspect("equal")
            ax.set_title(f'{scene.replace("align", "Door alignment").replace("corner", "L-shaped corridor")} | '
                         f'{"Single OBB" if model=="obb" else "34 parts"}', fontsize=10)
            ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Updated global routes: XY projection, arrows show body yaw\n"
                 "Fixed-pose synthetic geometry; circle = start, star = goal", fontsize=10)
    fig.savefig(root / "routes.png", dpi=180)
    fig.savefig(root / "routes.pdf")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    rows = []
    for stage in ("baseline", "updated"):
        for path in sorted((args.root / stage).glob("*/*.json")):
            if not path.stem.startswith(("obb_", "parts_")):
                continue
            result = json.loads(path.read_text(encoding="utf-8"))
            model, repeat = path.stem.split("_")
            rows.append({"stage": stage, "scene": path.parent.name, "model": model,
                         "repeat": int(repeat), **{key: result[key] for key in
                         ("ok", "reason", "elapsed_ms", "points", "length_m",
                          "turn_rad", "body_sweep_valid")}})
    with (args.root / "results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = []
    for scene in sorted({row["scene"] for row in rows}):
        for model in ("obb", "parts"):
            entry = {"scene": scene, "model": model}
            for stage in ("baseline", "updated"):
                group = [r for r in rows if (r["scene"], r["model"], r["stage"]) ==
                         (scene, model, stage)]
                if len(group) != 3:
                    raise ValueError(f"Expected three repeats for {scene}/{model}/{stage}")
                if any(r["ok"] and not r["body_sweep_valid"] for r in group):
                    raise ValueError(f"Unchecked successful route: {scene}/{model}/{stage}")
                entry[stage] = {"successes": sum(r["ok"] for r in group),
                                "median_ms": statistics.median(r["elapsed_ms"] for r in group),
                                "min_ms": min(r["elapsed_ms"] for r in group),
                                "max_ms": max(r["elapsed_ms"] for r in group),
                                "reasons": sorted({r["reason"] for r in group if r["reason"]})}
            summary.append(entry)
    (args.root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    lines = ["# 机体表示与直接姿态路线对照 · 2026-10-25", "",
             "Windows Release 原生组件；三次交替模型顺序；每次独立 PlannerSession，地图准备不计时。",
             "同一场景内共用地图、起终点、搜索参数；每次 10 秒合作截止。",
             "单 OBB 使用 Go2 既有尺寸夹具（0.96 × 0.51 × 0.34 m，中心 Z=0.07 m，零额外余量）；",
             "多部件使用现有 34 部件 BODY2，加载器默认余量 0.02 m。二者占据范围和支撑语义不同，",
             "这是两种既有配置的对照，不能解释为完全相同几何下只比较数据结构。",
             "前后改动只有全局规划前的直接候选尝试；不改变 Body、碰撞、支撑、制动参数。",
             "首次 baseline 测量与后续编译有部分重叠，机器非隔离；毫秒值只作本机诊断，不能声称严格提速倍数。",
             "", "| 场景 | 模型 | 修改前中位数 ms | 成功 | 修改后中位数 ms | 成功 | 修改后失败原因 |",
             "|---|---|---:|---:|---:|---:|---|"]
    for entry in summary:
        before, after = entry["baseline"], entry["updated"]
        lines.append(f'| {entry["scene"]} | {entry["model"]} | {before["median_ms"]:.2f} | '
                     f'{before["successes"]}/3 | {after["median_ms"]:.2f} | '
                     f'{after["successes"]}/3 | {", ".join(after["reasons"]) or "—"} |')
    lines += ["", "## 判读", "",
              "- align 从超时变为可返回的已检查路线，说明候选组织是这个案例的关键问题。",
              "- corner 仍需完整搜索，单 OBB 拒绝而固定多部件通过；不据此推断真实步态可通过。",
              "- support_gap 仍触发截止，没有新增错误放行，也尚未解决快速无路判断。",
              "- too_narrow 仍拒绝真实起点；upper_floor 在上层地表规划，没有用下层地板代替支撑。",
              "- 所有成功路线用同一个 C++ BodyQuery 实现再次扫掠检查，属于复查，不是独立碰撞算法验证。",
              "- 高频 SAT、支撑与体素缓存的独立时间分解尚未完成，不能仅由缓存命中率给出耗时占比。",
              "- 没有改变 Go2 默认模型，没有部署或启动实机。", "",
              "## 复现", "",
              "```powershell",
              "# 两个目录保存各自可执行文件、core.cpp、日志、地图和逐轮路线 JSON。",
              "& <baseline-or-updated>/demo.exe config/robots/unitree/go2/body/go2_mid360_flatpack.body2 <new-output-dir> --matched",
              "uv run python tools/bench/body_depth_report.py build/body-depth-20261025",
              "```", ""]
    physics = args.root / "physics" / "summary.json"
    if physics.exists():
        result = json.loads(physics.read_text(encoding="utf-8"))
        lines += ["## 物理执行", "", f'结果：`{result["result"]}`。',
                  "Thunder V4 既有策略 + MuJoCo + 原生 SCAN/Executor；不是 Go2 或完整 Product 验收。",
                  "完整指标见 physics/summary.json，调用与场景边界见 physics/invocation.json。", ""]
    (args.root / "README.md").write_text("\n".join(lines), encoding="utf-8")
    plot_routes(args.root)
    print(f"Wrote {len(rows)} trials and {len(summary)} paired summaries to {args.root}")


if __name__ == "__main__":
    main()
