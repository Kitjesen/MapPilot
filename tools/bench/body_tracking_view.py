"""Explain retained yaw/trajectory errors and show all long-route results."""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OLD_RUN = ROOT / "build/body-support-20261023/physics/final-yaw"
CASES = ("straight6-before", "straight6-current", "straight6-initial-clock-fixed")


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def explain(out: Path) -> None:
    initial, summary = read(OLD_RUN / "initialization.json"), read(OLD_RUN / "summary.json")
    diagnosis = read(out / "alignment-diagnosis.json")
    with (OLD_RUN / "native/states.csv").open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    time = np.array([float(row["t"]) for row in rows])
    yaw = np.array([float(row["yaw"]) for row in rows])
    goal = initial["requested_route"][-1][3]
    error = np.abs(np.arctan2(np.sin(goal-yaw), np.cos(goal-yaw)))
    turn_start = diagnosis["native_turn_stage_start_s"]
    full_time = diagnosis["first_full_tolerance_s"] - turn_start
    half_time = diagnosis["first_interior_tolerance_s"] - turn_start
    yaw85 = diagnosis["visible_turn_thresholds_deg"]["85"]["seconds_from_turn_start"]
    turn_mask = (time >= turn_start-1e-9) & (time <= diagnosis["first_interior_tolerance_s"]+1e-9)
    fig, axes = plt.subplots(1, 2, figsize=(12.3, 4.7), layout="constrained")
    axis = axes[0]
    axis.plot(time[turn_mask]-turn_start, np.degrees(yaw[turn_mask]), color="#286f88", label="Measured world yaw")
    axis.plot(time[turn_mask]-turn_start, np.degrees(error[turn_mask]), color="#ac7543",
              linestyle="--", label="Absolute final-yaw error")
    axis.axhline(np.degrees(goal), color="#565f64", linestyle=":", linewidth=.8)
    for x, color in ((yaw85, "#71928c"), (full_time, "#71928c"), (half_time, "#71928c")):
        axis.axvline(x, color=color, linestyle=":", linewidth=.8)
    axis.annotate(f"85 deg at {yaw85:.2f} s", xy=(yaw85, 85), xytext=(4.0, 62),
                  arrowprops={"arrowstyle": "-", "color": "#7d888b"}, fontsize=10)
    axis.annotate(f"Task tolerance {diagnosis['full_task_tolerance_deg']:.3f} deg\nfirst met at {full_time:.2f} s",
                  xy=(full_time, diagnosis["full_task_tolerance_deg"]), xytext=(5.7, 33),
                  arrowprops={"arrowstyle": "-", "color": "#7d888b"}, fontsize=10)
    axis.annotate(f"Half tolerance {diagnosis['interior_acquisition_tolerance_deg']:.3f} deg\nfirst met at {half_time:.2f} s",
                  xy=(half_time, diagnosis["interior_acquisition_tolerance_deg"]), xytext=(10.1, 63),
                  arrowprops={"arrowstyle": "-", "color": "#7d888b"}, fontsize=10)
    axis.set(title="Terminal turn: large rotation versus tolerance tail", xlabel="Time since terminal turn stage began (s)",
             ylabel="Angle (deg)", xlim=(-.3, 16.2), ylim=(-3, 97))
    origin = summary["execution_start_time_s"]
    tracking = np.array([float(row["tracking_error"]) if row["reason"] == "spline_control_ready" else np.nan for row in rows])
    first = next(i for i, row in enumerate(rows) if row["trajectory"] == "2" and row["reason"] == "spline_control_ready")
    switch_time, switch_error = time[first]-origin, tracking[first]
    mask = time-origin <= 7.5
    axes[1].plot(time[mask]-origin, tracking[mask]*100, color="#286f88", linewidth=1.4, label="Timed-spline XY reference error")
    axes[1].axvline(switch_time, color="#ac7543", linewidth=1, linestyle="--")
    axes[1].scatter(switch_time, switch_error*100, color="#ac7543", zorder=4, s=25)
    axes[1].annotate(f"Trajectory 2 adopted at {switch_time:.2f} s\nreference error {switch_error*100:.2f} cm",
                     xy=(switch_time, switch_error*100), xytext=(.3, 22),
                     arrowprops={"arrowstyle": "-", "color": "#7d888b"}, fontsize=10)
    axes[1].text(.50, .80, "Timed-spline reference error;\nnot global cross-track distance.",
                 transform=axes[1].transAxes, color="#4e5e65", fontsize=10)
    axes[1].set(title="1.2 m run: discontinuity when changing trajectory", xlabel="Execution time (s)",
                ylabel="XY distance to timed spline reference (cm)", xlim=(0, 7.5), ylim=(-.5, 27))
    for panel in axes:
        panel.spines[["top", "right"]].set_visible(False)
        panel.grid(alpha=.18, linewidth=.5)
    handles, labels = [], []
    for panel in axes:
        panel_handles, panel_labels = panel.get_legend_handles_labels()
        handles.extend(panel_handles)
        labels.extend(panel_labels)
    fig.legend(handles, labels, loc="outside upper center", ncol=3, frameon=False, fontsize=10)
    fig.savefig(out / "figure/yaw_and_reference_error.png", dpi=240)
    fig.savefig(out / "figure/yaw_and_reference_error.pdf")
    plt.close(fig)
    values = {"source": str(OLD_RUN), "turn_start_native_s": turn_start,
              "yaw_85_seconds_after_turn_start": yaw85, "task_tolerance_first_s": full_time,
              "half_tolerance_first_s": half_time, "trajectory_2_execution_s": float(switch_time),
              "trajectory_2_initial_timed_reference_error_m": float(switch_error),
              "meaning": "Timed reference error is not global cross-track distance; align status includes tolerance acquisition."}
    (out / "figure/diagnostic-values.json").write_text(json.dumps(values, indent=2) + "\n", encoding="utf-8")


def table(out: Path) -> str:
    metrics = read(out / "tracking-metrics.json")
    labels = {"straight6-before": "旧构建，首次长线", "straight6-current": "重新编译，首帧时钟仍错误",
              "straight6-initial-clock-fixed": "首次轨迹时钟修复"}
    rows = []
    for case in CASES:
        path = out / case / "summary.json"
        if not path.exists():
            rows.append(f'<tr><td>{labels[case]}</td><td colspan="8">尚未产生完成记录</td></tr>')
            continue
        summary, metric = read(path), metrics.get(case, {})
        numbers = {key: f'{metric[key]:.4f}' if key in metric else "—" for key in (
            "nearest_global_xy_distance_max_m", "reported_follower_position_error_rmse_m",
            "reported_follower_position_error_max_m")}
        rows.append(f'<tr><td>{labels[case]}</td><td><a href="{case}/summary.json">{html.escape(summary["result"])}</a></td>'
                    f'<td>{summary["execution_duration_s"]:.2f}</td><td>{summary["wall_time_s"]:.2f}</td>'
                    f'<td>{summary["goal_xy_error_m"]:.4f}</td><td>{numbers["nearest_global_xy_distance_max_m"]}</td>'
                    f'<td>{numbers["reported_follower_position_error_rmse_m"]}</td><td>{numbers["reported_follower_position_error_max_m"]}</td>'
                    f'<td>{summary["non_foot_contact_substeps"]} / {summary["physics_substeps_checked"]}</td></tr>')
    return "".join(rows)


def page(out: Path) -> None:
    latest = read(out / "straight6-initial-clock-fixed/summary.json")
    metrics = read(out / "tracking-metrics.json")["straight6-initial-clock-fixed"]
    document = """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>LingTu · 轨迹时钟与长线物理验证</title><style>*{box-sizing:border-box}body{margin:0;background:#f4f5f2;color:#26383e;font:15px/1.7 Arial,'Microsoft YaHei',sans-serif}main{max-width:1280px;margin:auto;padding:42px 24px}h1{font-size:29px;margin:5px 0 12px;letter-spacing:-.6px}h2{font-size:20px;margin-top:35px}p{color:#617078;max-width:1000px}a{color:#236f86}small{color:#73858b;letter-spacing:1px}figure{margin:20px 0;background:white;padding:12px;border:1px solid #e0e6e4}figure img,video{width:100%;display:block}figcaption{font-size:13px;color:#6d7a80;margin:8px 4px}.table{overflow:auto}table{border-collapse:collapse;white-space:nowrap;width:100%;font-size:13px}td,th{padding:11px 9px;border-bottom:1px solid #d6dfdc;text-align:left}th{font-weight:400;color:#687c84}.note{border-left:3px solid #7c979b;padding-left:15px}.link{display:inline-block;margin:8px 15px 8px 0}</style><main><small>LINGTU · NATIVE NAVIGATION · 2026.10.23</small><h1>走到终点，与精确跟踪之间还有距离</h1><p>在已有短直线实验中拆清楚慢转向和轨迹参考跳变，再用 6 m 原生全局路线进入真实 Thunder＋MuJoCo＋策略物理验证。所有运行与失败性能保留。</p>
<h2>01 / 先把两个现象讲清楚</h2><figure><img src="figure/yaw_and_reference_error.png" alt="真实末端转向的容差尾段，以及切换样条后的时间参考误差"><figcaption>已有 1.2 m 实验 final-yaw 的原生 CSV。<a href="figure/yaw_and_reference_error.pdf">论文图 PDF</a> · <a href="figure/diagnostic-values.json">逐项计算记录</a> · Arial 10 pt</figcaption></figure><p>左图：转到 85° 只用了 3.04 秒，进入任务容差首次用了 11.08 秒，进入内部半容差用了 15.70 秒。align 状态累计 15.72 秒包含末端收敛与等待，不能理解为一直在做有效的大角度旋转。右图：执行 3.56 秒切到 trajectory 2，控制器对当前样条时间参考的 XY 距离跳至约 18 cm；它不是离全局路线的横向距离。</p>
<h2>02 / 6 m 长直线：保留每次结果</h2><p>本轮修复首次轨迹激活时的本地时钟起点。所有行都使用真实原生全局＋Executor＋SCAN＋控制器＋物理反馈；没有跳过全局，也没有用位姿积分替代动力学。</p><div class="table"><table><thead><tr><th>阶段</th><th>结果／原始 summary</th><th>仿真执行 s</th><th>墙钟 s</th><th>终点 XY m</th><th>全局线最近距 max m</th><th>时间样条误差 RMSE m</th><th>时间样条误差 max m</th><th>非法接触／物理子步</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<p class="note">最新运行仿真执行 __SIM__ 秒，但实验墙钟为 __WALL__ 秒。机器人在仿真中更早完成任务，不代表整套程序跑得更快或达到实时。首次等待修复也不代表所有跟踪误差已经解决。每行只是一次运行，不能形成统计性能结论。</p>
<h2>03 / 看全程实测状态，保留慢转向</h2><video controls preload="metadata" src="straight6-replay/measured-route-replay.mp4"></video><p>这是原始记录的双视图回放，不是重新运行的试验。视频时钟为仿真时间，不代表墙钟；恢复全部 16 个关节和实测 base 姿态，不插值、不平滑。路线与测得轨迹投影到地面以便观察。<a href="straight6-replay/index.html">打开双视图、帧图和恢复校验</a>。</p>
<h2>04 / 当前结论的范围</h2><p>最新运行时间样条 XY 误差 RMSE 为 __RMSE__ m、最大 __MAX__ m；到有限全局路线的最近距离最大 __GLOBAL__ m。前者衡量当前时刻应到的位置，后者衡量几何路线偏离，不能互换。末端转向仍约 __ALIGN__ 秒；本轮是直线加终点转向，不是复杂连续折线、多层或 Go2 实机验收。</p><a class="link" href="README.md">完整实验说明与已知问题</a><a class="link" href="tracking-metrics.json">全部跟踪指标</a><a class="link" href="alignment-diagnosis.json">原始末端对齐诊断</a><a class="link" href="../body-support-20261023/index.html">支撑接触修复记录</a></main></html>"""
    fields = {"ROWS": table(out), "SIM": f'{latest["execution_duration_s"]:.2f}', "WALL": f'{latest["wall_time_s"]:.2f}',
              "RMSE": f'{metrics["reported_follower_position_error_rmse_m"]:.4f}',
              "MAX": f'{metrics["reported_follower_position_error_max_m"]:.4f}',
              "GLOBAL": f'{metrics["nearest_global_xy_distance_max_m"]:.4f}',
              "ALIGN": f'{metrics["phase_duration_s"]["aligning_goal_yaw"]:.2f}'}
    for key, value in fields.items():
        document = document.replace(f"__{key}__", value if key == "ROWS" else html.escape(value))
    (out / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "build/body-tracking-20261023")
    parser.add_argument("--figures-only", action="store_true")
    args = parser.parse_args()
    out = args.out.resolve()
    (out / "figure").mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "Arial", "font.size": 10, "axes.titlesize": 10,
                         "axes.labelsize": 10, "legend.fontsize": 10, "xtick.labelsize": 10,
                         "ytick.labelsize": 10, "pdf.fonttype": 42})
    explain(out)
    if not args.figures_only:
        page(out)
    print(out / "index.html")


if __name__ == "__main__":
    main()
