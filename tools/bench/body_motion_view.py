"""Replay recorded MuJoCo states with the actual native route visible.

This renderer never steps physics or re-executes a controller. Base pose and
all 16 joints are restored from retained measurements without interpolation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
from itertools import pairwise
from pathlib import Path
from typing import Any

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

DISPLAY_ONLY_GEOMS = ("x_reference", "start_marker", "end_marker")
ROUTE_COLOR = np.array([.12, .42, .92, 1.0], dtype=np.float32)
TRAIL_COLOR = np.array([.08, .72, .45, 1.0], dtype=np.float32)


def hide_showcase_markers(model: Any) -> list[str]:
    """Hide only the three named nonphysical decorations in flat_showcase."""
    hidden = []
    for name in DISPLAY_ONLY_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            if model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]:
                raise ValueError(f"Refusing to hide physical geometry {name}")
            model.geom_rgba[geom_id, 3] = 0
            hidden.append(name)
    return hidden


def read_route(path: Path) -> np.ndarray | None:
    """Only a completed native CSV is presented as a global route."""
    if not path.exists():
        return None
    with path.open(encoding="utf-8", newline="") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        return None
    return np.array([[float(row[key]) for key in ("x", "y", "z", "yaw")] for row in rows])


def connector(scene: Any, a: np.ndarray, b: np.ndarray, color: np.ndarray, width: float) -> None:
    if np.linalg.norm(b - a) < 1e-9:
        return
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Replay overlay exhausted MuJoCo scene capacity")
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                       np.zeros(3), np.zeros(3), np.eye(3).ravel(), color)
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
    geom.emission, geom.specular, geom.reflectance, geom.shininess = 1.0, 0.0, 0.0, 0.0
    scene.ngeom += 1


def draw_route_overlay(renderer: Any, route: np.ndarray | None,
                       measured: np.ndarray, requested: np.ndarray) -> None:
    """Project actual route/trail XY to the floor for legible spatial comparison."""
    if route is not None:
        floor_route = np.array(route[:, :3], copy=True)
        floor_route[:, 2] = .018
        for first, second in pairwise(floor_route):
            connector(renderer.scene, first, second, ROUTE_COLOR, .012)
        goal = floor_route[-1]
        heading = float(route[-1, 3])
        connector(renderer.scene, goal, np.add(goal, [.20 * math.cos(heading), .20 * math.sin(heading), 0]),
                  np.array([.56, .30, .76, 1], dtype=np.float32), .018)
    else:
        # Requested goals are deliberately points, never a fabricated route.
        for pose in requested:
            point = np.array([pose[0], pose[1], .022])
            connector(renderer.scene, point - [.035, 0, 0], np.add(point, [.035, 0, 0]),
                      np.array([.7, .5, .2, 1], dtype=np.float32), .012)
    if len(measured) > 1:
        floor_trail = np.array(measured[:, :3], copy=True)
        floor_trail[:, 2] = .045
        for first, second in pairwise(floor_trail):
            connector(renderer.scene, first, second, TRAIL_COLOR, .006)


def pose_quaternion(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy) / 2
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def camera(route: np.ndarray, *, top: bool) -> Any:
    view = mujoco.MjvCamera()
    view.type = mujoco.mjtCamera.mjCAMERA_FREE
    low, high = route[:, :2].min(axis=0), route[:, :2].max(axis=0)
    center = (low + high) / 2
    span = high - low
    # The view is fixed for the entire recording, with room for the robot at
    # both endpoints. The 640x480 viewport is wider than its vertical FOV.
    horizontal_distance = (span[0] + 1.6) / (2 * math.tan(math.radians(45) / 2) * (640 / 480))
    vertical_distance = (span[1] + 1.6) / (2 * math.tan(math.radians(45) / 2))
    extent_distance = max(horizontal_distance, vertical_distance if top else vertical_distance / math.sin(math.radians(32)))
    view.lookat[:] = [*center, .20]
    view.distance, view.azimuth, view.elevation = (max(3.0, extent_distance), 90, -89.5) if top else (
        max(3.2, extent_distance * 1.08), 90, -32)
    return view


def restore_obstacles(model: Any, data: Any, sample: dict[str, Any]) -> list[dict[str, Any]]:
    """Restore retained physical mocap positions; never infer actor movement."""
    checks = []
    for box in sample.get("dynamic_obstacles", []):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, box["name"])
        if body < 0 or int(model.body_mocapid[body]) < 0:
            raise ValueError(f"Recorded obstacle is absent from physical world: {box['name']}")
        mocap = int(model.body_mocapid[body])
        data.mocap_pos[mocap] = box["position"]
        checks.append({"name": box["name"], "recorded_position": box["position"],
                       "restored_position": data.mocap_pos[mocap].tolist()})
    return checks


def route_description(invocation: dict[str, Any]) -> str:
    return ("Native checked supplied pose route" if "--use-route" in invocation.get("command", [])
            else "Native global search route")


def captions(frame: np.ndarray, sample: dict, elapsed: float, route_ready: bool,
             route_kind: str = "Native global search route") -> None:
    frame[:100, :] = [28, 34, 38]
    frame[580:, :] = [28, 34, 38]
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "RECORDED STATE REPLAY | Thunder + MuJoCo policy experiment", (24, 30),
                font, .66, (236, 238, 240), 1, cv2.LINE_AA)
    x, y, z = sample["position"]
    info = f"t={elapsed:5.2f} s   base X={x:.3f}  Y={y:.3f}  Z={z:.3f} m   yaw={math.degrees(sample['rpy'][2]):.1f} deg"
    cv2.putText(frame, info, (24, 61), font, .58, (207, 218, 224), 1, cv2.LINE_AA)
    cv2.putText(frame, "TOP VIEW", (24, 88), font, .46, (166, 183, 191), 1, cv2.LINE_AA)
    cv2.putText(frame, "FIXED OBLIQUE VIEW", (665, 88), font, .46, (166, 183, 191), 1, cv2.LINE_AA)
    legend = "Blue: " + route_kind if route_ready else "Orange points: requested poses; native route pending"
    cv2.putText(frame, legend + " | Green: measured base trail | Floor projection", (24, 606),
                font, .52, (216, 224, 230), 1, cv2.LINE_AA)
    cv2.putText(frame, "10 fps recorded samples, no pose interpolation or new physics | " + sample["command"]["status"],
                (24, 631), font, .47, (166, 183, 191), 1, cv2.LINE_AA)


def render_review(run: Path, out: Path, *, fps: int = 10) -> None:
    from sim.compat.engine.core.robot import THUNDER_V4_JOINT_NAMES

    from drivers.sim.mujoco.runtime import build_engine

    out.mkdir(parents=True, exist_ok=True)
    initial = json.loads((run / "initialization.json").read_text())
    summary = json.loads((run / "summary.json").read_text())
    invocation = json.loads((run / "invocation.json").read_text())
    route_kind = route_description(invocation)
    samples = [json.loads(line) for line in (run / "states.jsonl").read_text().splitlines()]
    route = read_route(run / "native/global.csv")
    if route is None:
        raise ValueError("Recorded review requires the actual native global.csv")
    engine = build_engine(world=Path(initial["world"]), robot_xml=Path(initial["robot_xml"]),
                          drive_mode="kinematic", start=initial["base_position"],
                          mujoco_memory="64M", lidar_backend="mujoco_lidar",
                          lidar_body_name="lidar1_link", lidar_site_name="lidar1_link_site",
                          mid360_samples_per_frame=32)
    model, data = engine.model, engine.data
    hidden = hide_showcase_markers(model)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    free_joint = int(model.body_jntadr[base_id])
    if model.jnt_type[free_joint] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("Recorded robot root is not a free joint")
    root_address = int(model.jnt_qposadr[free_joint])
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in THUNDER_V4_JOINT_NAMES]
    if min(joint_ids) < 0 or len(joint_ids) != 16:
        raise ValueError("Recorded Thunder joint mapping is incomplete")
    addresses = np.array([int(model.jnt_qposadr[joint]) for joint in joint_ids])
    data.qpos[:] = initial["qpos"]
    initial_quat_error = float(np.max(np.abs(pose_quaternion(initial["rpy"]) - data.qpos[root_address+3:root_address+7])))
    if initial_quat_error > 1e-9:
        raise ValueError("RPY-to-quaternion does not reproduce the recorded root convention")
    renderer = mujoco.Renderer(model, height=480, width=640)
    options = mujoco.MjvOption()
    options.geomgroup[5] = 1
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("Existing ffmpeg executable is required for browser-compatible H264")
    times = np.array([sample["t"] for sample in samples])
    origin = summary["execution_start_time_s"]
    targets = np.arange(times[0], times[-1] + 1e-8, 1/fps)
    indices = np.searchsorted(times, targets)
    indices = np.minimum(indices, len(samples)-1)
    if indices[-1] != len(samples)-1:
        indices = np.append(indices, len(samples)-1)
    positions = np.array([sample["position"] for sample in samples])
    request = np.asarray(initial["requested_route"])
    framing = [*route[:, :3], *positions, *request[:, :3]]
    for sample in samples:
        for box in sample.get("dynamic_obstacles", []):
            center, half = np.asarray(box["position"]), np.asarray(box["half_size"])
            framing.extend([center - half, center + half])
    views = (camera(np.asarray(framing), top=True), camera(np.asarray(framing), top=False))
    frame_records = []
    state_checks = []
    snapshots = {0, len(indices)//4, len(indices)//2, 3*len(indices)//4, len(indices)-1}
    output = out / "measured-route-replay.mp4"
    command = [ffmpeg, "-y", "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", "1280x640",
               "-framerate", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "fast",
               "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]
    try:
        with (out / "ffmpeg.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
            try:
                for frame_index, state_index in enumerate(indices):
                    sample = samples[int(state_index)]
                    data.qpos[root_address:root_address+3] = sample["position"]
                    data.qpos[root_address+3:root_address+7] = pose_quaternion(sample["rpy"])
                    data.qpos[addresses] = sample["joint_positions"]
                    data.time = sample["t"]
                    obstacle_checks = restore_obstacles(model, data, sample)
                    mujoco.mj_forward(model, data)
                    if np.max(np.abs(data.xpos[base_id] - sample["position"])) > 1e-10:
                        raise ValueError("Rendered base differs from recorded measured base")
                    restored = engine.get_robot_state()
                    restored_quat = np.asarray(restored.orientation)[[3, 0, 1, 2]]
                    expected_quat = pose_quaternion(sample["rpy"])
                    errors = {"position_max_abs_m": float(np.max(np.abs(restored.position - sample["position"]))),
                              "joints_max_abs_rad": float(np.max(np.abs(restored.joint_positions - sample["joint_positions"]))),
                              "orientation_quaternion_error": float(min(np.max(np.abs(restored_quat - expected_quat)),
                                                                         np.max(np.abs(restored_quat + expected_quat))))}
                    if max(errors.values()) > 1e-10:
                        raise ValueError(f"Public engine state does not reproduce recording: {errors}")
                    if frame_index in snapshots:
                        state_checks.append({"source_state_index": int(state_index), "recorded_position": sample["position"],
                                             "restored_position": restored.position.tolist(), "recorded_rpy": sample["rpy"],
                                             "recorded_joint_positions": sample["joint_positions"],
                                             "restored_joint_positions": restored.joint_positions.tolist(),
                                             "obstacles": obstacle_checks, **errors})
                    images = []
                    for view in views:
                        renderer.update_scene(data, camera=view, scene_option=options)
                        draw_route_overlay(renderer, route, positions[:state_index+1], request)
                        images.append(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
                    frame = np.zeros((640, 1280, 3), dtype=np.uint8)
                    frame[100:580] = np.concatenate(images, axis=1)
                    elapsed = sample["t"] - origin
                    captions(frame, sample, elapsed, True, route_kind)
                    process.stdin.write(frame.tobytes())
                    frame_records.append({"video_frame": frame_index, "source_state_index": int(state_index),
                                          "recorded_time_s": sample["t"], "elapsed_s": elapsed})
                    if frame_index in snapshots:
                        cv2.imwrite(str(out / f"frame-{frame_index:04d}.jpg"), frame)
            finally:
                process.stdin.close()
                result = process.wait(timeout=60)
            if result:
                raise RuntimeError(f"ffmpeg returned {result}; see ffmpeg.log")
    finally:
        renderer.close()
        engine.close()
    report = {"source_run": str(run), "source_summary": summary, "scope": "recorded-state visualization; no new physics",
              "sample_selection": "first recorded sample at or after each 0.1 s target, plus final sample; no interpolation",
              "joint_mapping": dict(zip(THUNDER_V4_JOINT_NAMES, addresses.tolist(), strict=True)),
              "initial_quaternion_error": initial_quat_error, "hidden_nonphysical_geoms": hidden,
              "public_engine_state_checks": state_checks,
              "route_kind": route_kind,
              "obstacle_scope": invocation.get("obstacle_input_scope", "no recorded dynamic fixture input"),
              "route_source": str(run / "native/global.csv"), "overlay": "world XY projected to floor for legibility",
              "frames": frame_records, "video": output.name}
    (out / "replay-record.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    document = """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>实际状态与路线回放</title><style>body{margin:0;background:#f4f5f3;color:#26343a;font:16px/1.7 Arial,'Microsoft YaHei',sans-serif}main{max-width:1280px;margin:auto;padding:32px 20px}h1{font-size:26px;margin:0}p{max-width:1000px;color:#626f76}video{display:block;width:100%;background:#1c2226}a{color:#236b80}.frames{display:grid;grid-template-columns:repeat(2,1fr);gap:14px}.frames img{width:100%}figcaption{font-size:13px}figure{margin:10px 0}</style><main><h1>实际测得状态，与真实全局路线</h1><p>这是已有 Thunder＋MuJoCo＋策略实验的记录回放，没有重新运行物理或导航。每帧恢复原始机身位置、roll/pitch/yaw 和全部 16 个关节；10 fps 抽取记录，不插值、不平滑、不移动路线。主视频为固定俯视与斜视双画面。</p><video controls preload="metadata" poster="__POSTER__" src="measured-route-replay.mp4"></video><p>蓝色为 native/global.csv 的真实全局路线，绿色为截至该时刻的测得机身轨迹，紫色为目标朝向；两条线投影到地面以便观察，机身与关节保留原始三维实测姿态。原画面的黑线和彩色点是场景展示道具，位于 y=-0.85，与导航路线无关，这次仅在渲染中隐藏。</p><p>本实验任务是直线目标距离约 __DISTANCE__ m，终点转向约 __TURN__°；非连续折线路径验收。转向状态累计 __ALIGN__ 秒。贴合全局几何路线不代表按样条时间精确跟踪，误差定义和统计分别见跟踪指标。不代表多层或 Go2 实机验收。<a href="replay-record.json">逐帧来源和关节地址</a> · <a href="__METRICS__">跟踪指标</a> · <a href="__RUN__/native/global.csv">原始全局路线</a> · <a href="__RUN__/states.jsonl">全部实测状态</a> · <a href="../index.html">完整结果页</a></p><div class="frames">__FRAMES__</div></main></html>"""
    figures = "".join(f'<figure><img src="frame-{i:04d}.jpg"><figcaption>记录时刻 {frame_records[i]["elapsed_s"]:.2f} s</figcaption></figure>' for i in sorted(snapshots))
    document = document.replace("主视频为固定俯视与斜视双画面。", "主视频为固定俯视与斜视双画面。"
                                f"视频按仿真时间播放；本次仿真执行 {summary['execution_duration_s']:.2f} 秒，"
                                f"实验墙钟耗时 {summary['wall_time_s']:.2f} 秒，两者不同。")
    document = document.replace("__POSTER__", f"frame-{len(indices)-1:04d}.jpg").replace("__FRAMES__", figures)
    run_link = Path(os.path.relpath(run, out)).as_posix()
    alignment_s = sum(sample["command"]["status"] == "aligning_goal_yaw" for sample in samples) * invocation["dt_s"]
    requested_length = sum(float(np.linalg.norm(b[:3] - a[:3])) for a, b in pairwise(request))
    if invocation.get("waypoints"):
        task_text = (f"本实验使用显式指定的分段位姿路线，总平移长度 {requested_length:.2f} m，"
                     f"包含 {len(request)} 个请求姿态。蓝线由原生执行器检查后接收，"
                     "不能据此声称转角由全局搜索自动生成。")
    else:
        task_text = (f"本实验从实测起点向目标请求全局搜索，起终点平移距离 "
                     f"{np.linalg.norm(request[-1,:2] - request[0,:2]):.2f} m，"
                     f"终点朝向相对起点 {math.degrees(math.remainder(request[-1,3] - request[0,3], 2*math.pi)):.1f}°。")
    document = document.replace("本实验任务是直线目标距离约 __DISTANCE__ m，终点转向约 __TURN__°；非连续折线路径验收。", task_text)
    document = document.replace("每帧恢复原始机身位置、roll/pitch/yaw 和全部 16 个关节；",
                                "每帧恢复原始机身位置、roll/pitch/yaw、全部 16 个关节，以及记录中的动态障碍物位置；")
    document = document.replace("蓝色为 native/global.csv 的真实全局路线", f"蓝色为 native/global.csv 中的 {route_kind}")
    if summary.get("dynamic_obstacle_count", 0):
        document = document.replace("不代表多层或 Go2 实机验收。",
                                    "障碍输入来自确定性测试脚本的真实物理代理位置，不代表 LiDAR 检测、Product 或 Go2 实机验收。")
    metrics = out / "tracking-metrics.json"
    if not metrics.exists():
        metrics = out.parent / "tracking-metrics.json"
    document = (document.replace("__DISTANCE__", f"{np.linalg.norm(request[-1,:2] - request[0,:2]):.2f}")
                .replace("__TURN__", f"{math.degrees(math.remainder(request[-1,3] - request[0,3], 2*math.pi)):.1f}")
                .replace("__ALIGN__", f"{alignment_s:.2f}")
                .replace("__RUN__", run_link)
                .replace("__METRICS__", Path(os.path.relpath(metrics, out)).as_posix()))
    (out / "index.html").write_text(document, encoding="utf-8")
    print(json.dumps({"video": str(output), "frames": len(indices), "initial_quaternion_error": initial_quat_error}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=ROOT / "build/body-support-20261023/physics/final-yaw")
    parser.add_argument("--out", type=Path, default=ROOT / "build/body-support-20261023/motion-review")
    args = parser.parse_args()
    render_review(args.run.resolve(), args.out.resolve())


if __name__ == "__main__":
    main()
