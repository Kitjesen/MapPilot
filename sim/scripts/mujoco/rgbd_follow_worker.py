"""Controlled-shirt RGB-D following diagnostic, not a learned person detector.

Render the latest physical snapshot in a separate process so graphics cannot
stall IMU/driver stepping. Actor poses only restore the rendered scene; the
perception and servo receive pixels, depth and the synchronized camera pose.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from sim.compat.engine.core.robot import RobotConfig
from sim.compat.engine.core.sensor import CameraConfig
from sim.compat.engine.mujoco.engine import MuJoCoEngine
from sim.scripts.mujoco.rgbd_goal_acceptance import camera_optical_transform

import mujoco
from decision.modules.visual_servo import VisualServoModule
from perception.backends import RgbdObservationSource
from perception.detection.detector_base import Detection2D
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


class ShirtPixelDetector:
    """Single distinctive shirt fixture; it has no world/actor access."""

    def detect(self, bgr, prompt):
        b, g, r = cv2.split(bgr.astype(np.int16))
        mask = ((r > g * 1.6 + 20) & (b > g * 1.4 + 20) & (r > 65)).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if count < 2:
            return []
        index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        x, y, w, h, area = stats[index]
        if area < 18:
            return []
        return [
            Detection2D(
                bbox=np.array([x, y, x + w, y + h], dtype=np.float32),
                score=1.0,
                label="person",
                mask=labels == index,
            )
        ]


def restore_snapshot(engine, row):
    """Restore physics for rendering only; no actor coordinates leave this call."""
    engine._data.qpos[:] = row["qpos"]
    engine._data.qvel[:] = 0
    actor = row.get("mocap_pose")
    if actor:
        body = engine._model.body(actor["body_name"]).id
        engine._data.mocap_pos[engine._model.body_mocapid[body]] = actor["position_m"]
    engine._data.time = row["sim_time_s"]
    mujoco.mj_forward(engine._model, engine._data)


def observe_frame(source, frame, transform, stamp):
    h, w = frame.depth.shape
    fx, fy, cx, cy = frame.intrinsics
    color = Image(data=frame.rgb, format=ImageFormat.RGB, ts=stamp)
    observation = SimpleNamespace(
        color=color,
        depth=Image(data=frame.depth, format=ImageFormat.DEPTH_F32, ts=stamp),
        intrinsics=CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h),
        map_from_camera=transform,
    )
    return color, source.observe(observation, "person")


class FollowController:
    def __init__(self, binary, domain_id, log, follow_distance=1.8, goal_deadband_m=0.25):
        self.binary, self.domain_id, self.log = binary, domain_id, log
        self.goals = 0
        self.accepted = 0
        self.cancels = 0
        self.task = None
        self.pending = None
        self.cancel_reason = None
        self.servo = VisualServoModule(follow_distance=follow_distance, goal_deadband_m=goal_deadband_m)
        self.servo.setup()
        self.servo.goal_pose._add_callback(self.queue_goal)
        self.servo.goal_cancel._add_callback(self.queue_cancel)
        self.servo.servo_target._deliver("follow_id:shirt_fixture")

    def queue_goal(self, goal):
        self.pending = goal

    def queue_cancel(self, reason):
        self.pending = None
        self.cancel_reason = reason

    def command(self, arguments):
        started = time.monotonic()
        proc = subprocess.run(
            [self.binary, *arguments, "--domain-id", str(self.domain_id), "--timeout-ms", "2000"],
            capture_output=True,
            text=True,
            timeout=7,
        )
        result = {
            "command": arguments,
            "ok": proc.returncode == 0,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "latency_s": time.monotonic() - started,
        }
        self.log.write(json.dumps({"event": "command", **result}) + "\n")
        self.log.flush()
        return result

    def dispatch(self):
        if self.cancel_reason is not None:
            if self.task:
                self.command(["cancel", self.task, self.cancel_reason])
                self.cancels += 1
            self.cancel_reason = None
            self.task = None
        if self.pending is None:
            return
        goal, self.pending = self.pending, None
        self.goals += 1
        task = f"rgbd-follow-{self.goals}"
        result = self.command(
            [
                "goal",
                str(goal.x),
                str(goal.y),
                str(goal.z),
                str(goal.yaw),
                "--task-id",
                task,
            ]
        )
        if result["ok"]:
            self.task = task
            self.accepted += 1
        self.servo.goal_status._deliver(
            {
                "action": "visual_servo",
                "task_id": task,
                "accepted": result["ok"],
                "state": "accepted" if result["ok"] else "rejected",
                "reason": result["stderr"],
            }
        )

    def update(self, row, color, detections):
        self.servo.robot_pose._deliver(
            PoseStamped(
                pose=Pose(position=Vector3(row["x"], row["y"], row["z"]), orientation=Quaternion.from_yaw(row["yaw"])),
                frame_id="map",
                ts=row["t"],
            )
        )
        self.servo.color_image._deliver(color)
        self.servo.detections_3d._deliver(
            [
                Detection3D(
                    id="shirt_fixture",
                    label="person",
                    confidence=item.score,
                    position=Vector3(*map(float, item.position)),
                    bbox_2d=item.bbox_2d.tolist(),
                    ts=row["t"],
                )
                for item in detections
            ]
        )
        self.dispatch()

    def close(self):
        self.servo.stop()
        self.dispatch()


def run(args):
    root = Path(args.phase_dir)
    source = RgbdObservationSource(ShirtPixelDetector(), min_depth=0.3, max_depth=8, u16_depth_scale=0.001)
    source.load()
    engine = MuJoCoEngine(
        robot_config=RobotConfig.default_thunder_v4(),
        camera_configs=[CameraConfig(width=320, height=240)],
        drive_mode="kinematic",
    )
    log = (root / "rgbd-follow.jsonl").open("w", encoding="utf-8", buffering=1)
    controller = FollowController(args.control_binary, args.domain_id, log,
                                  goal_deadband_m=args.goal_deadband_m)
    video = cv2.VideoWriter(str(root / "rgbd-camera.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 5, (640, 240))
    samples = visible = 0
    last_seen = time.monotonic()
    max_gap = 0.0
    error = ""
    reader = None
    pending = ""
    try:
        engine.load(args.world)
        engine.reset()
        camera_id = engine._cameras["front_camera"].cam_id
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            if (root / "rgbd-follow.stop").exists():
                break
            if not (root / "rgbd-follow.ready").exists():
                time.sleep(0.05)
                continue
            if reader is None:
                if not (root / "motion.jsonl").exists():
                    time.sleep(0.05)
                    continue
                reader = (root / "motion.jsonl").open(encoding="utf-8")
            pending += reader.read()
            lines = pending.split("\n")
            pending = lines.pop()
            if not lines:
                if time.monotonic() - last_seen > 1:
                    controller.queue_cancel("rgbd_snapshot_stale")
                    controller.dispatch()
                time.sleep(0.04)
                continue
            row = json.loads(lines[-1])
            gap = time.monotonic() - last_seen
            max_gap = max(max_gap, gap) if samples else max_gap
            last_seen = time.monotonic()
            restore_snapshot(engine, row)
            frame = engine.get_camera_data()
            color, detections = observe_frame(
                source, frame, camera_optical_transform(engine._data, camera_id), row["t"]
            )
            samples += 1
            visible += bool(detections)
            annotated = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
            for item in detections:
                x1, y1, x2, y2 = map(int, item.bbox_2d)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
            depth = cv2.applyColorMap((np.clip(frame.depth / 8, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            panel = np.hstack([annotated, depth])
            cv2.putText(
                panel,
                f"RGB-D | t={row['sim_time_s']:.1f}s | fixture shirt",
                (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
            )
            video.write(panel)
            if samples == 1:
                cv2.imwrite(str(root / "rgbd-first.png"), panel)
                np.save(root / "rgbd-first-depth.npy", frame.depth)
            cv2.imwrite(str(root / "rgbd-latest.png"), panel)
            projected = [item.position.tolist() for item in detections]
            log.write(
                json.dumps(
                    {
                        "event": "observation",
                        "sim_time_s": row["sim_time_s"],
                        "t": row["t"],
                        "robot": [row["x"], row["y"], row["z"]],
                        "projected_targets": projected,
                    }
                )
                + "\n"
            )
            controller.update(row, color, detections)
            time.sleep(0.10)
        else:
            error = "follow_worker_timeout"
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            controller.close()
        except Exception as exc:
            error = error or f"follow_shutdown:{type(exc).__name__}:{exc}"
            raise
        finally:
            if reader:
                reader.close()
            engine.close()
            source.close()
            video.release()
            log.close()
            report = {
                "samples": samples,
                "visible_samples": visible,
                "visible_fraction": visible / max(1, samples),
                "goals": controller.goals,
                "accepted_goals": controller.accepted,
                "cancels": controller.cancels,
                "max_snapshot_gap_wall_s": max_gap,
                "error": error,
                "detector": "RGB shirt color fixture, not learned person recognition",
                "goal_deadband_m": args.goal_deadband_m,
            }
            (root / "rgbd-follow-report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--phase-dir", required=True)
    parser.add_argument("--control-binary", required=True)
    parser.add_argument("--domain-id", type=int, required=True)
    parser.add_argument("--timeout-s", type=float, default=450)
    parser.add_argument("--goal-deadband-m", type=float, default=.25)
    run(parser.parse_args())
