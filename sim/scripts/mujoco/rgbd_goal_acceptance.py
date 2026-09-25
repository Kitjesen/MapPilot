"""Image-selected RGB-D goal through production projection and native navigation.

This component diagnostic deliberately uses truth localization and an operator
ROI, not a learned detector or scene-metadata target. Native processes, robot
physics, contact checks and stop acknowledgement use the existing runner.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from sim.compat.engine.core.robot import RobotConfig
from sim.compat.engine.core.sensor import CameraConfig
from sim.compat.engine.mujoco.engine import MuJoCoEngine, resolve_scene_asset_paths
from sim.scripts.mujoco import native_navigation_acceptance as native

from decision.modules.visual_servo import VisualServoModule
from lingtu.assembly.compiler import compile_run_plan
from perception.backends import RgbdObservationSource
from perception.detection.detector_base import Detection2D
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat

ROOT = Path(__file__).resolve().parents[3]
MANIFEST = ROOT / "config/acceptance/mujoco/rgbd_goal.json"


class SelectedImageRegion:
    """Represent an operator-selected pixel region; never read scene metadata."""

    def __init__(self, bbox):
        self.bbox = np.asarray(bbox, dtype=np.float32)

    def detect(self, image, prompt):
        return [Detection2D(bbox=self.bbox, label="selected_target", score=1.0)]


def camera_optical_transform(data, camera_id):
    """Convert the simulator camera pose to the RGB-D optical convention."""
    transform = np.eye(4)
    transform[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
    transform[:3, 3] = data.cam_xpos[camera_id]
    return transform


def project_selected_goal(frame, map_from_camera, robot_pose, bbox):
    """Use the same RGB-D projection and standoff calculation as the Host."""
    height, width = frame.depth.shape
    fx, fy, cx, cy = frame.intrinsics
    stamp = 1.0
    observation = SimpleNamespace(
        color=Image(data=frame.rgb, format=ImageFormat.RGB, ts=stamp),
        depth=Image(data=frame.depth, format=ImageFormat.DEPTH_F32, ts=stamp),
        intrinsics=CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height),
        map_from_camera=map_from_camera,
    )
    source = RgbdObservationSource(
        SelectedImageRegion(bbox),
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
    )
    source.load()
    try:
        detections = source.observe(observation, "selected_target")
    finally:
        source.close()
    if len(detections) != 1:
        raise ValueError("selected_region_has_no_valid_depth")
    target = detections[0]
    servo = VisualServoModule()
    servo.setup()
    goals = []
    servo.goal_pose._add_callback(goals.append)
    try:
        servo.robot_pose._deliver(robot_pose)
        servo._on_servo_target("find:selected_target")
        servo.detections_3d._deliver(
            [
                Detection3D(
                    label=target.label,
                    confidence=target.score,
                    position=Vector3(*map(float, target.position)),
                    bbox_2d=list(map(float, bbox)),
                    ts=stamp,
                )
            ]
        )
        if len(goals) != 1:
            raise ValueError("visual_servo_did_not_publish_one_goal")
        return target, goals[0]
    finally:
        servo.stop()


def prepare(out: Path, map_dir: Path, case: str, bbox):
    manifest = native._load_manifest(MANIFEST)
    world_path = ROOT / manifest["world"]
    tree = ET.parse(world_path)
    resolve_scene_asset_paths(tree.getroot(), world_path)
    if case == "obstacle":
        ET.SubElement(
            tree.getroot().find("worldbody"),
            "geom",
            {
                "name": "rgbd_new_obstacle",
                "type": "box",
                "pos": "41.5 6 0.25",
                "size": "0.35 0.55 0.25",
                "rgba": "0.9 0.25 0.05 1",
                "conaffinity": "1",
                "condim": "3",
            },
        )
    world = out / "world.xml"
    tree.write(world, encoding="unicode")
    robot = RobotConfig.default_thunder_v4()
    robot.init_position = manifest["start"][:3]
    engine = MuJoCoEngine(robot_config=robot, camera_configs=[CameraConfig()], drive_mode="kinematic")
    try:
        engine.load(str(world))
        engine.reset()
        frame = engine.get_camera_data()
        camera_id = engine._cameras["front_camera"].cam_id
        transform = camera_optical_transform(engine._data, camera_id)
        state = engine.get_robot_state()
        pose = PoseStamped(
            pose=Pose(position=Vector3(*map(float, state.position)), orientation=Quaternion.from_yaw(0)),
            frame_id="map",
            ts=1.0,
        )
        target, goal = project_selected_goal(frame, transform, pose, bbox)
        cv2.imwrite(str(out / "camera-rgb.png"), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
        annotated = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(annotated, "Selected image region", (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imwrite(str(out / "camera-selection.png"), annotated)
        np.save(out / "camera-depth-m.npy", frame.depth)
    finally:
        engine.close()
    # Independent geometry is only an evaluator; it never creates the goal.
    target_xy_error = math.hypot(float(target.position[0]) - 44.55, float(target.position[1]) - 6.0)
    evidence = {
        "case": case,
        "selection": "operator_roi",
        "bbox": bbox,
        "target_from_rgbd_m": target.position.tolist(),
        "goal_from_visual_servo_m": [goal.x, goal.y, goal.z, goal.yaw],
        "depth_m": target.depth,
        "map_from_camera": transform.tolist(),
        "reference_target_xy_m": [44.55, 6.0],
        "target_xy_error_m": target_xy_error,
        "projection_passed": target_xy_error <= 0.05,
        "localization": "simulation_truth_fixture",
        "robot": "ThunderV4",
        "automatic_recognition_tested": False,
    }
    native._write_json(out / "vision.json", evidence)
    if not evidence["projection_passed"]:
        raise ValueError("rendered_target_projection_failed")
    manifest["world"] = str(world)
    manifest["map_dir"] = str(map_dir)
    manifest["goal"] = [goal.x, goal.y, goal.z, goal.yaw]
    manifest.pop("asset_builder", None)
    native._write_json(out / "navigation.json", manifest)
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--map-dir", type=Path, default=Path.home() / "data/lingtu/maps/industrial_park_tracking")
    parser.add_argument("--case", choices=("clear", "obstacle"), default="clear")
    parser.add_argument("--bbox", nargs=4, type=int, default=[312, 232, 328, 248])
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    evidence = prepare(out, args.map_dir.resolve(), args.case, args.bbox)
    print(json.dumps(evidence, indent=2))
    if args.prepare_only:
        return 0
    # Resolve current robot geometry and process artifacts once. This component
    # runner does not start a Product and must not claim a verified Product run.
    plan = compile_run_plan(
        "nav", "sim", robot="doso/thunder_v4", local_planner="scan", env_config={"backend": "mujoco"}
    )
    report = native.run(
        argparse.Namespace(
            manifest=str(out / "navigation.json"),
            out_dir=str(out / "native"),
            mode="motion",
            build_helper=False,
            prepare_assets=False,
            preflight_only=False,
            domain_id=225,
            validated_run_plan=plan,
            run_plan_verified=False,
            record_video=True,
            video_width=1280,
            video_height=720,
            video_fps=12,
            video_lidar_points=640,
        )
    )
    phase = report.get("phases", {}).get("motion", {})
    evidence.update(
        {
            "navigation_ok": report["ok"],
            "blockers": report.get("blockers", []),
            "goal_metrics": phase.get("goal_metrics"),
            "stop": phase.get("terminal_driver_stop"),
            "native_report": str(out / "native/report.json"),
        }
    )
    native._write_json(out / "summary.json", evidence)
    print(json.dumps(evidence, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
