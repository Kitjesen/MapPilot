"""Render retained support-contact evidence without substituting planned motion.

Run after the native/physical probes have written their final summaries.
"""

from __future__ import annotations

import argparse
import html
import json
import os
from itertools import product
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
COLORS = {"old": "#8b9299", "new": "#236b80", "source": "#467d64", "intermediate": "#b88650"}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def save(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path.with_suffix(".png"), dpi=240)
    fig.savefig(path.with_suffix(".pdf"))
    plt.close(fig)


def polygon(axis, points: np.ndarray, color: str, label: str) -> None:
    projection = points[:, [0, 2]]
    hull = ConvexHull(projection)
    axis.add_patch(Polygon(projection[hull.vertices], facecolor=color, alpha=.14,
                           edgecolor=color, linewidth=1.5, label=label))


def contact_figure(out: Path) -> dict:
    initial = read(out / "physics/first/initialization.json")
    part = next(p for p in initial["parts"] if p["link"] == "FL_foot_34")
    rotation = np.asarray(part["rotation"])
    yaw = initial["rpy"][2]
    cosine, sine = np.cos(yaw), np.sin(yaw)
    yaw_rotation = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
    axes = yaw_rotation @ rotation
    center = yaw_rotation @ part["center"] + initial["base_position"]
    radius, half_length = part["radius_m"], part["half_length_m"]
    theta = np.linspace(0, 2 * np.pi, 1441)
    source = np.concatenate([
        np.column_stack([radius * np.cos(theta), radius * np.sin(theta),
                         np.full_like(theta, height)]) @ axes.T + center
        for height in (-half_length, half_length)
    ])
    corners = np.array(list(product((-1, 1), repeat=3)))
    old = (corners * part["half_size"]) @ axes.T + center
    cylinder_axis = axes[:, 2]
    radial = np.cross(cylinder_axis, np.cross([0, 0, 1], cylinder_axis))
    radial /= np.linalg.norm(radial)
    canonical_axes = np.column_stack([radial, np.cross(cylinder_axis, radial), cylinder_axis])
    corrected = (corners * [radius, radius, half_length]) @ canonical_axes.T + center
    # The exact vertical cylinder support function avoids sampling error.
    exact_min = float(center[2] - abs(cylinder_axis[2]) * half_length
                      - radius * np.hypot(cylinder_axis[0], cylinder_axis[1]))
    report = {"source": "physics/first/initialization.json", "link": part["link"],
              "source_min_z_m": exact_min, "old_obb_min_z_m": float(old[:, 2].min()),
              "canonical_obb_min_z_m": float(corrected[:, 2].min()),
              "margin_m": 0, "projection": "World X-Z; unchanged primitive, center and axis"}
    fig, panels = plt.subplots(1, 3, figsize=(10.8, 3.6), layout="constrained", sharex=True, sharey=True)
    for panel, points, title, color, bottom in zip(
        panels, (source, old, corrected),
        ("Source cylinder", "Previous collision OBB", "Gravity-aligned collision OBB"),
        (COLORS["source"], COLORS["old"], COLORS["new"]),
        (exact_min, old[:, 2].min(), corrected[:, 2].min()), strict=True,
    ):
        panel.axhspan(-.06, 0, color="#e6e4df", zorder=-2)
        panel.axhline(0, color="#343a3d", linewidth=.8)
        polygon(panel, points, color, title)
        if title != "Source cylinder":
            outline = ConvexHull(source[:, [0, 2]]).vertices
            outline = np.append(outline, outline[0])
            panel.plot(source[outline, 0], source[outline, 2],
                       color=COLORS["source"], linewidth=.7, alpha=.7)
        panel.set(title=f"{title}\nLowest Z = {bottom * 1000:.2f} mm",
                  xlabel="World X (m)", xlim=(center[0]-.155, center[0]+.155),
                  ylim=(-.055, .25), aspect="equal")
        panel.spines[["top", "right"]].set_visible(False)
        panel.grid(alpha=.16, linewidth=.5)
    panels[0].set_ylabel("World Z (m)")
    save(fig, out / "figure/contact_geometry")
    return report


def states(directory: Path) -> tuple[dict, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    summary = read(directory / "summary.json")
    rows = [json.loads(line) for line in (directory / "states.jsonl").read_text().splitlines()]
    time = np.array([row["t"] for row in rows]) - summary["execution_start_time_s"]
    position = np.array([row["position"] for row in rows])
    yaw = np.unwrap([row["rpy"][2] for row in rows])
    speed = np.array([np.hypot(row["command"]["vx"], row["command"]["vy"]) for row in rows])
    return summary, time, position, yaw, speed


def physics_figure(out: Path, physical_run: Path) -> dict:
    old = ROOT / "build/body-execution-20261022/physics/route-only-timing-fix"
    initial = read(physical_run / "initialization.json")
    goal = initial["requested_route"][-1]
    fig, axes = plt.subplots(2, 2, figsize=(9.8, 6.1), layout="constrained")
    panels = axes.ravel()
    all_positions = []
    runs = [(old, "old", "Previous: blocked at start")]
    if physical_run != out / "physics/final":
        runs.append((out / "physics/final", "intermediate", "Support fix: first completed run"))
    runs.append((physical_run, "new", "Corrected: measured physics"))
    for directory, key, label in runs:
        summary, time, position, yaw, speed = states(directory)
        color = COLORS[key]
        all_positions.append(position)
        panels[0].plot(position[:, 0], position[:, 1], color=color, linewidth=1.5, label=label)
        panels[0].scatter(*position[-1, :2], color=color, s=22, zorder=4)
        panels[1].plot(time, np.degrees(yaw), color=color, linewidth=1.2)
        panels[2].plot(time, position[:, 2], color=color, linewidth=1.1)
        panels[3].plot(time, speed, color=color, linewidth=1.1)
    reference = np.asarray(initial["requested_route"])
    panels[0].plot(reference[:, 0], reference[:, 1], color="#42535c", linewidth=.8,
                   linestyle="--", zorder=0)
    panels[0].scatter(goal[0], goal[1], marker="*", color="#22272b", s=95, label="Requested goal", zorder=5)
    combined = np.concatenate(all_positions)
    panels[0].set(xlabel="World X (m)", ylabel="World Y (m)", title="Measured base trajectory",
                  ylim=(min(-.20, combined[:, 1].min()-.04), max(.20, combined[:, 1].max()+.04)),
                  xlim=(combined[:, 0].min()-.07, combined[:, 0].max()+.07))
    panels[1].axhline(np.degrees(goal[3]), color="#22272b", linewidth=.8, linestyle="--")
    panels[1].set(xlabel="Execution time (s)", ylabel="Measured yaw (deg)", title="Heading feedback and requested final yaw")
    panels[2].axhline(goal[2], color="#22272b", linewidth=.8, linestyle="--")
    panels[2].set(xlabel="Execution time (s)", ylabel="Measured base Z (m)", title="Physical base height; dashed: route height")
    panels[3].set(xlabel="Execution time (s)", ylabel="Commanded XY speed (m/s)", title="Commands released by native execution checks")
    for panel in panels:
        panel.spines[["top", "right"]].set_visible(False)
        panel.grid(alpha=.18, linewidth=.5)
    handles, labels = panels[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=2 if len(handles) > 3 else 3, frameon=False)
    save(fig, out / "figure/physics_execution")
    return summary


def poster(out: Path, physical_run: Path) -> None:
    import cv2

    reader = cv2.VideoCapture(str(physical_run / "physics.mp4"))
    try:
        count = int(reader.get(cv2.CAP_PROP_FRAME_COUNT))
        reader.set(cv2.CAP_PROP_POS_FRAMES, max(0, count - 1))
        ok, frame = reader.read()
        if not ok:
            raise RuntimeError("Could not decode final physical video frame")
        cv2.imwrite(str(out / "figure/physics_poster.jpg"), frame)
    finally:
        reader.release()


def result_table(out: Path, physical_run: Path) -> str:
    directories = [(ROOT / "build/body-execution-20261022/physics/route-only-timing-fix", "修复前"),
                   (out / "physics/motion", "支撑修复，尚未修正高度参考"),
                   (out / "physics/motion-fixed-root", "高度修复，尚未修正最终静止判定"),
                   (out / "physics/final", "首次完成：最终朝向仍频繁切换")]
    if physical_run != out / "physics/final":
        directories.append((physical_run, "本轮最终复测"))
    repeat = out / "physics/final-yaw-repeat"
    if (repeat / "summary.json").exists() and repeat != physical_run:
        directories.append((repeat, "同版本、同任务重复验证"))
    rows = []
    for directory, label in directories:
        data = read(directory / "summary.json")
        link = Path(os.path.relpath(directory / "summary.json", out)).as_posix()
        rows.append(f'<tr><td>{html.escape(label)}</td><td><a href="{link}">{html.escape(data["result"])}</a></td>'
                    f'<td>{data["actual_xy_displacement_m"]:.4f}</td><td>{data["goal_xy_error_m"]:.4f}</td>'
                    f'<td>{data["goal_yaw_error_rad"]:.4f}</td><td>{data["execution_duration_s"]:.2f}</td></tr>')
    return "".join(rows)


def go2_table(out: Path) -> str:
    cases = (("go2-other/detour/body", "直穿门洞"),
             ("go2-other/align/body", "调整朝向后穿门"),
             ("go2-final/corner/body", "L 形通道"),
             ("go2-final/too_narrow/body", "0.28 m 过窄通道"))
    rows = []
    for relative, label in cases:
        data = read(out / relative / "summary.json")
        elapsed = f'{data["elapsed_sim_s"]:.2f}' if data["execution_started"] else "—"
        blocked = str(data["final_blocked_ticks"]) if data["execution_started"] else "—"
        error = f'{data["goal_error_m"]:.4f}' if data["execution_started"] else "—"
        result = html.escape(data["reason"])
        rows.append(f'<tr><td>{label}</td><td><a href="{relative}/summary.json">{result}</a></td>'
                    f'<td>{data["global_ms"]:.2f}</td><td>{elapsed}</td><td>{error}</td><td>{blocked}</td></tr>')
    return "".join(rows)


def page(out: Path, final: dict, geometry: dict, physical_run: Path) -> None:
    repeat_path = out / "physics/final-yaw-repeat/summary.json"
    repeat_note = ""
    if repeat_path.exists():
        repeat = read(repeat_path)
        repeat_note = (f'重复运行反馈 {repeat["result"]}，执行 {repeat["execution_duration_s"]:.2f} 秒，'
                       f'XY 误差 {repeat["goal_xy_error_m"]*100:.2f} cm，'
                       f'yaw 误差 {np.degrees(repeat["goal_yaw_error_rad"]):.2f}°；'
                       f'{repeat["physics_substeps_checked"]} 个物理子步中，'
                       f'非法世界接触 {repeat["non_foot_contact_substeps"]} 次，'
                       f'无承载支撑命令帧 {repeat["no_loaded_support_frames"]} 次。')
    document = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>支撑接触修复 · LingTu</title>
<style>*{box-sizing:border-box}body{margin:0;background:#f5f5f2;color:#263237;font:15px/1.65 Arial,"Microsoft YaHei",sans-serif}main{max-width:1100px;margin:auto;padding:46px 28px}h1{font-size:30px;letter-spacing:-1px;margin:4px 0 14px}h2{font-size:20px;margin:36px 0 12px}p{max-width:850px;color:#5b666b}a{color:#236b80}header small{color:#6e797e;letter-spacing:1px}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;margin:28px 0}.metric{border-top:1px solid #cbd3d5;padding:13px 0}.metric strong{display:block;font-size:25px;font-weight:500}.metric span{font-size:13px;color:#6c777b}figure{margin:20px 0;background:#fff;border:1px solid #e2e6e5;padding:12px}figure img{width:100%;display:block}figcaption{font-size:13px;color:#667378;padding:8px}video{width:100%;display:block;background:#151b1e}table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:12px 8px;border-bottom:1px solid #dce1e0}th{color:#68757b;font-weight:400}.scope{border-left:3px solid #79939a;padding-left:14px}footer{margin-top:38px;font-size:13px;color:#6c777b}@media(max-width:650px){.metrics{grid-template-columns:repeat(2,1fr)}main{padding:24px 16px}.table{overflow:auto}}</style>
<main><header><small>LINGTU · NATIVE NAVIGATION · 2026.10.23</small><h1>正常落地，通过检查，再实际移动</h1>
<p>轮足几何与支撑接触修复。保留首轮失败和中间失败，依据实际测量反馈展示最终结果。</p></header>
<div class="metrics"><div class="metric"><strong>__RESULT__</strong><span>最终原生执行反馈</span></div><div class="metric"><strong>__XY__ cm</strong><span>终点 XY 误差</span></div><div class="metric"><strong>__YAW__°</strong><span>终点朝向误差</span></div><div class="metric"><strong>__TIME__ s</strong><span>实际物理执行时间</span></div></div>
<p class="scope">Thunder 模型＋现有 ONNX 策略＋MuJoCo 动力学＋原生 C++ 导航组件。输入使用物理真值位姿和固定地图；本页不代表 Go2 实机、完整 Product 或在线建图验收。</p>
<p><strong>录像复核：</strong>旧画面的黑线和彩色点是场景参照，位于实际路线侧面约0.85m，不是导航目标。已按同一次测得状态制作<a href="motion-review/index.html">真实路线＋实走轨迹的双视图回放</a>。这次只验证短直线前进与终点转向，终点到达不代表样条跟踪精度和停转效率已达标；全过程指标见<a href="motion-review/tracking-metrics.json">跟踪记录</a>。</p>
<h2>01 / 为什么站稳却被拒绝</h2><p>同一个 FL 轮柱，保持原始尺寸、中心和轴向。旧外接盒将不存在的角部伸进地面；新径向轴对齐重力投影，外接盒的最低点与真实圆柱一致。下图未加膨胀余量；合法地面接触仍须通过独立支撑面规则。</p>
<figure><img src="figure/contact_geometry.png" alt="源圆柱、旧外接盒与重力对齐外接盒对比"><figcaption>数据：physics/first/initialization.json · FL_foot_34 · <a href="figure/contact_geometry.pdf">PDF</a> · <a href="figure/geometry_measurements.json">计算记录</a> · Arial 10 pt</figcaption></figure>
<h2>02 / 原生检查放行后的真实物理反馈</h2><p>最终反馈：__SUBSTEPS__ 个物理子步中，非法世界接触 __CONTACTS__ 次；无承载轮足的命令帧 __UNSUPPORTED__ 次。旧实验仅运行 5 秒并始终被阻止；曲线未扩展或补齐旧实验。最终执行仍耗时较长，速度优化单独推进。</p>
<figure><img src="figure/physics_execution.png" alt="实际物理轨迹、朝向、高度与速度命令"><figcaption><a href="figure/physics_execution.pdf">PDF</a> · <a href="physics/final/summary.json">最终原始结果</a> · <a href="physics/final/states.jsonl">逐帧状态</a> · Arial 10 pt</figcaption></figure>
<figure><video controls preload="metadata" poster="figure/physics_poster.jpg" src="physics/final/physics.mp4"></video><figcaption>保留的原始 MuJoCo＋策略录影；画中黑线不是本次导航路线。<a href="motion-review/index.html">查看叠加真实路线的记录回放</a> · <a href="physics/final/physics.mp4">原视频</a>。</figcaption></figure>
<h2>03 / 保留的失败与最终复测</h2><p>表中保留各阶段原始记录，以及同版本、同任务的重复验证。样本数量仍不足以支持统计性能结论。最终图保留首次完成的 46.78 秒运行作为对照；重复验证未叠加到图中，主视频对应 24.42 秒运行。</p><p>__REPEAT_NOTE__</p><div class="table"><table><thead><tr><th>阶段</th><th>结果／原始记录</th><th>位移 m</th><th>XY 误差 m</th><th>yaw 误差 rad</th><th>执行 s</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<h2>04 / Go2 固定姿态回放另行验证</h2><p>以下四项是 Go2 固定姿态原生组件回放，与上方 Thunder 动力学证据分开。拦停帧表示最终运动检查阻止了该帧命令；最终到达与过程中被阻止可以同时成立。过窄通道在全局阶段拒绝，未启动执行。</p><div class="table"><table><thead><tr><th>场景</th><th>结果／原始记录</th><th>全局计算 ms</th><th>执行 s</th><th>XY 误差 m</th><th>最终检查拦停帧</th></tr></thead><tbody>__GO2_ROWS__</tbody></table></div>
<footer><a href="README.md">完整实验说明</a> · <a href="physics/README.md">物理复现实验说明</a> · <a href="../../tools/bench/body_support_view.py">本页与论文图生成程序</a><br>三维占据、地面支撑、机体碰撞与最终运动检查分别保留；过窄通道继续拒绝。</footer></main></html>"""
    fields = {"RESULT": "已到达" if final["result"] == "native_goal_reached" else "未完成",
              "XY": f'{final["goal_xy_error_m"]*100:.2f}',
              "YAW": f'{np.degrees(final["goal_yaw_error_rad"]):.2f}', "TIME": f'{final["execution_duration_s"]:.2f}',
              "SUBSTEPS": str(final["physics_substeps_checked"]), "CONTACTS": str(final["non_foot_contact_substeps"]),
              "UNSUPPORTED": str(final["no_loaded_support_frames"]), "ROWS": result_table(out, physical_run),
              "GO2_ROWS": go2_table(out), "REPEAT_NOTE": repeat_note}
    run_link = Path(os.path.relpath(physical_run, out)).as_posix()
    document = document.replace("physics/final/", run_link + "/")
    for key, value in fields.items():
        document = document.replace(f"__{key}__", value if key.endswith("ROWS") else html.escape(value))
    (out / "index.html").write_text(document, encoding="utf-8")
    (out / "figure/geometry_measurements.json").write_text(json.dumps(geometry, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "build/body-support-20261023")
    parser.add_argument("--physics", type=Path, help="Completed physical run, default: OUT/physics/final")
    args = parser.parse_args()
    out = args.out.resolve()
    physical_run = args.physics.resolve() if args.physics else out / "physics/final"
    (out / "figure").mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "Arial", "font.size": 10, "axes.titlesize": 10,
                         "axes.labelsize": 10, "legend.fontsize": 10, "xtick.labelsize": 10,
                         "ytick.labelsize": 10, "pdf.fonttype": 42})
    geometry = contact_figure(out)
    final = physics_figure(out, physical_run)
    poster(out, physical_run)
    page(out, final, geometry, physical_run)
    print(json.dumps({"index": str(out / "index.html"), "geometry": geometry,
                      "physical_result": final["result"]}, indent=2))


if __name__ == "__main__":
    main()
