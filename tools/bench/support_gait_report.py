"""Render saved physical states and report support probe disagreements.

Replay never steps physics. Original results and recordings are retained.
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import subprocess
from collections import Counter
from pathlib import Path

from support_gait_probe import POLICY, build_engine, mujoco, np


LABELS = {
    "flat_forward": "平地直行", "flat_turn": "平地转向",
    "slot_stand_010": "纵向槽 10 cm · 站立",
    "slot_stand_030": "纵向槽 30 cm · 站立",
    "slot_stand_050": "纵向槽 50 cm · 初始化站立失败",
    "slot_walk_010": "纵向槽 10 cm · 前进",
    "gap_walk_010": "横向断口 10 cm",
    "gap_walk_030": "横向断口 30 cm",
    "corridor_060": "通道 60 cm", "corridor_080": "通道 80 cm",
}


def figures(root: Path, records: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "Arial", "font.size": 10,
                         "axes.titlesize": 10, "axes.labelsize": 10,
                         "xtick.labelsize": 10, "ytick.labelsize": 10,
                         "legend.fontsize": 10, "pdf.fonttype": 42})
    completed = [r for r in records if r["result"] in ("distance_completed", "hold_completed", "turn_completed")]
    if not completed:
        return
    labels = [r["name"].replace("_", " ") for r in completed]
    x = np.arange(len(completed))
    fig, ax = plt.subplots(figsize=(9, 3.8), layout="constrained")
    for offset, key, label, color in [(-.27,"strict_support","Center support","#9a5338"),
                                      (-.09,"nearby_support","Nearby support","#d1ab73"),
                                      (.09,"nominal_free","Fixed body","#6a8b9d"),
                                      (.27,"observed_free","Measured body","#254b64")]:
        ax.bar(x+offset, [100*r[key+"_accepted"]/r["native_samples"] for r in completed], .18, label=label, color=color)
    ax.set_xticks(x, labels, rotation=24, ha="right")
    ax.set_ylabel("Accepted recorded poses (%)")
    ax.set_ylim(0, 110)
    ax.legend(loc="lower center", bbox_to_anchor=(.5, 1.01), ncol=4, frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    for ext in ("png", "pdf", "svg"):
        fig.savefig(root / f"support-acceptance.{ext}", dpi=220)
    plt.close(fig)


def replay(folder: Path, states: list[dict]) -> None:
    import cv2
    from PIL import Image, ImageDraw, ImageFont
    engine = build_engine(world=folder / "world.xml", drive_mode="policy", start=[0, 0, .58],
                          mujoco_memory="64M", policy_path=POLICY, policy_cpu_threads=1,
                          lidar_backend="mujoco_lidar", lidar_body_name="lidar1_link",
                          lidar_site_name="lidar1_link_site", mid360_samples_per_frame=32)
    renderer = writer = None
    try:
        renderer = mujoco.Renderer(engine.model, height=540, width=960)
        writer = cv2.VideoWriter(str(folder / "motion-source.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (960, 540))
        if not writer.isOpened():
            raise RuntimeError("video writer failed")
        options = mujoco.MjvOption()
        options.geomgroup[5] = 1
        camera = mujoco.MjvCamera()
        camera.distance, camera.azimuth, camera.elevation = 2.8, 125, -28
        if folder.name.startswith("corridor"):
            camera.distance, camera.azimuth, camera.elevation = 3.2, 90, -85
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 16)
        indices = list(range(0, len(states), 5))
        if indices[-1] != len(states)-1:
            indices.append(len(states)-1)
        # A failed settling run may already be below the fixture at the first
        # saved sample. Render that state honestly; never relocate the robot.
        for index in indices:
            row = states[index]
            engine.data.qpos[:] = row["qpos"]
            engine.data.qvel[:] = row["qvel"]
            engine.data.time = row["t"] + 2
            mujoco.mj_forward(engine.model, engine.data)
            camera.lookat[:] = [max(.3, row["position"][0]), 0, .2]
            renderer.update_scene(engine.data, camera=camera, scene_option=options)
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 960, 52), fill=(246, 247, 249))
            draw.text((14, 6), f"{folder.name} | recorded physical states | t={row['t']:.2f} s", font=font, fill=(25, 30, 36))
            draw.text((14, 29), f"Thunder V4 / MuJoCo | loaded feet: {len(row['loaded_feet'])} | base z: {row['position'][2]:.3f} m", font=font, fill=(25, 30, 36))
            array = np.asarray(frame)
            writer.write(cv2.cvtColor(array, cv2.COLOR_RGB2BGR))
            if index == indices[0]:
                frame.save(folder / "start.png")
            if index == indices[len(indices)//2]:
                frame.save(folder / "preview.png")
            if index == indices[-1]:
                frame.save(folder / "last.png")
    finally:
        if writer is not None:
            writer.release()
        if renderer is not None:
            renderer.close()
        engine.close()
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                    "-i", str(folder / "motion-source.mp4"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                    str(folder / "motion.mp4")], check=True)


def report(root: Path, render: bool, render_case: str | None = None) -> None:
    records, cards = [], []
    for folder in sorted(root.iterdir()):
        if not (folder / "summary.json").exists():
            continue
        raw = json.loads((folder / "summary.json").read_text())
        states = [json.loads(line) for line in (folder / "states.jsonl").read_text().splitlines()]
        with (folder / "native.csv").open(newline="") as stream:
            native = list(csv.DictReader(stream))
        result = raw["result"]
        if min(s["position"][2] for s in states) < -.3:
            result = "fell_during_settle" if states[0]["position"][2] < -.3 else "fell_below_fixture"
        row = {**raw, "raw_result": raw["result"], "result": result,
               "initial_base_z_m": states[0]["position"][2],
               "final_base_z_m": states[-1]["position"][2],
               "native_samples": len(native)}
        for key in ("strict_support", "nearby_support", "nominal_free", "observed_free"):
            row[key+"_accepted"] = sum(int(n[key]) for n in native)
        row["nominal_reasons"] = dict(Counter(n["nominal_reason"] for n in native if n["nominal_reason"]))
        row["observed_reasons"] = dict(Counter(n["observed_reason"] for n in native if n["observed_reason"]))
        row["loaded_feet_counts"] = dict(Counter(len(s["loaded_feet"]) for s in states))
        records.append(row)
        if render and (render_case is None or folder.name.startswith(render_case)):
            settle_path = folder / "settle-states.jsonl"
            replay_states = states
            if result == "fell_during_settle" and settle_path.exists():
                replay_states = [json.loads(line) for line in settle_path.read_text().splitlines()] + states
            replay(folder, replay_states)
        note = ""
        if result == "fell_during_settle":
            note = "预站立阶段已掉落，不能计为成功。" + ("录像包含预站立过程（负时间）。" if (folder / "settle-states.jsonl").exists() else "原始 hold_completed 标签错误；原记录不含最初掉落过程。")
        elif result == "contact_violation":
            note = "因前轮边缘接触触发既有规则而中止；未完成跨越，也未证明物理上无法跨越。"
        elif folder.name == "corridor_060":
            note = "实际完成，但原生实测构型检查有 7/38 次左前轮支撑拒绝。6 s 时真实轮缘 y=0.2769 m，膨胀包络 y=0.3014 m，进入 y=0.30 m 的墙列；支撑检查要求膨胀区域每一列上方自由。"
        else:
            note = "预设速度指令的物理能力测试；不代表自主导航闭环通过。"
        name = folder.name
        video_path, poster_path = f"{name}/motion.mp4", f"{name}/preview.png"
        if result == "fell_during_settle" and root.name == "support-gait-20261025-r1":
            video_path = f"../support-gait-20261025-settle/{name}/motion.mp4"
            poster_path = f"../support-gait-20261025-settle/{name}/start.png"
            note += " 下方视频使用同场景补跑，包含最初掉落；负时间为预站立。"
        cards.append(f'''<article><h2>{LABELS.get(name,name)}</h2>
<video controls preload="none" poster="{poster_path}" src="{video_path}"></video>
<p><b>{html.escape(result)}</b> · {row['duration_s']:.2f} s · 前进 {row['net_x_m']:.3f} m</p>
<p>{note}</p><p>支撑接受：中心 {row['strict_support_accepted']}/{len(native)}；附近 {row['nearby_support_accepted']}/{len(native)}。机体接受：固定 {row['nominal_free_accepted']}/{len(native)}；实测 {row['observed_free_accepted']}/{len(native)}。</p>
<p><a href="{name}/states.jsonl">实际状态</a> · <a href="{name}/native.csv">原生判定</a> · <a href="{name}/case.json">配置</a> · <a href="{name}/world.xml">场景</a> · <a href="{name}/summary.json">原始摘要</a></p></article>''')
        print(name, result, flush=True)
    (root / "evaluation.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    with (root / "evaluation.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    figures(root, records)
    page = '''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>支撑与实际运动 · 10 组实验</title>
<style>body{margin:40px auto;max-width:1240px;padding:0 24px;background:#f2f3f5;color:#212730;font:16px Arial,"Microsoft YaHei",sans-serif}h1{font-size:30px}h2{font-size:19px;margin:0 0 16px}p{line-height:1.7}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:24px}article{background:white;padding:24px;border-radius:12px}video{width:100%;border-radius:6px}a{color:#226294}header{margin-bottom:30px}small{color:#55616c}</style>
<header><h1>支撑判断，是否挡住了实际能走的地方？</h1>
<p>Thunder V4 · 原生 MuJoCo + 现有 ONNX 策略 · 10 组独立场景 · 2026-10-25</p>
<p>纵向槽 10/30 cm 暴露中心支撑的误拒；50 cm 槽预站立掉落；横向断口触发轮缘接触规则。直行主要为四轮承重滚动，不能代表 Go2 迈腿。</p>
<p>“50 cm 纵向槽”是沿前进方向缺失的一条地板，50 cm 指横向宽度。机器人从槽上方直接初始化，场景没有槽底地板；这不是从远处接近并跨沟的导航测试，也不能据此断言 50 cm 是跨越极限。</p>
<p>视频为已保存实际 qpos/qvel 的状态重放，正常场景约 1 倍速、10 fps；不是重新运行物理。原录像遗漏机器人显示组，已保留但不作为展示。所有结果含失败均在下方。</p>
<p>本实验未执行 Product、SCAN 或导航仲裁。原生检查离线使用相同盒体场景、5 cm 体素、固定与实测构型作对照。<a href="evaluation.csv">汇总 CSV</a> · <a href="evaluation.json">完整评估</a> · <a href="support-acceptance.pdf">Arial 10 号统计图</a></p></header><main>'''+"\n".join(cards)+"</main></html>"
    if root.name == "support-gait-20261025-r1":
        page = page.replace("<main>", '<p><a href="../support-gait-20261025-settle/slot_stand_050/motion.mp4">50 cm 纵向槽：补录预站立掉落全过程</a>（同场景重新运行；负时间为预站立）</p><main>')
    (root / "index.html").write_text(page, encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--render-case")
    args = parser.parse_args()
    report(args.root, args.render, args.render_case)
