"""Run the real sim_mujoco semantic full stack in one process.

Usage:
    cd D:\\inovxio\\brain\\lingtu
    set PYTHONPATH=src;.
    python sim\\scripts\\run_semantic_full_stack.py --world open_field --duration 20
    python sim\\scripts\\run_semantic_full_stack.py --instruction "find the door"
    python sim\\scripts\\run_semantic_full_stack.py --goal 4 2 --render
"""

from __future__ import annotations

import argparse
import importlib
import logging
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
SRC_ROOT = REPO_ROOT / "src"

for path in (SRC_ROOT, REPO_ROOT):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
logger = logging.getLogger("run_semantic_full_stack")


def _import_status(module_name: str) -> str:
    try:
        importlib.import_module(module_name)
        return "ok"
    except Exception as exc:  # pragma: no cover - diagnostic path
        return f"missing ({exc})"


def _log_dependency_status(detector: str, encoder: str) -> None:
    detector_modules = {
        "yoloe": "perception.detection.yoloe_detector",
        "yolo_world": "perception.detection.yolo_world_detector",
        "bpu": "perception.detection.bpu_detector",
    }
    encoder_modules = {
        "mobileclip": "perception.encoding.mobileclip_encoder",
        "clip": "perception.encoding.clip_encoder",
    }

    checks = [
        ("runtime_utils", "runtime.utils.sanitize"),
        ("decision", "decision.llm.client"),
        ("tracker", "perception.tracking.instance_tracker"),
    ]
    if detector in detector_modules:
        checks.append((f"detector:{detector}", detector_modules[detector]))
    if encoder in encoder_modules:
        checks.append((f"encoder:{encoder}", encoder_modules[encoder]))

    for label, module_name in checks:
        logger.info("Dependency %-18s %s", label, _import_status(module_name))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the real semantic full stack on sim_mujoco.")
    parser.add_argument("--world", default="open_field")
    parser.add_argument("--slam-profile", default="none")
    parser.add_argument("--detector", default="yoloe")
    parser.add_argument("--encoder", default="mobileclip")
    parser.add_argument("--llm", default="mock")
    parser.add_argument("--instruction", default="")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--enable-gateway", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    _log_dependency_status(args.detector, args.encoder)

    from lingtu.assembly.profile_builder import build_system_for_profile
    from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

    system = build_system_for_profile(
        "sim",
        dict(
            robot="sim_mujoco",
            world=args.world,
            slam_profile=args.slam_profile,
            detector=args.detector,
            encoder=args.encoder,
            llm=args.llm,
            enable_native=False,
            enable_semantic=True,
            enable_gateway=args.enable_gateway,
            render=args.render,
        ),
    )

    driver = system.get_module("MujocoDriverModule")
    nav = system.get_module("nav.mission")
    perception = system.get_module("PerceptionModule")
    semantic = system.get_module("SemanticPlannerModule")

    stats = {
        "odom": 0,
        "scene_graph": 0,
        "detections": 0,
        "semantic_goals": 0,
        "waypoints": 0,
    }

    driver.odometry._add_callback(lambda _: stats.__setitem__("odom", stats["odom"] + 1))
    perception.scene_graph._add_callback(lambda _: stats.__setitem__("scene_graph", stats["scene_graph"] + 1))
    perception.detections_3d._add_callback(
        lambda dets: stats.__setitem__("detections", stats["detections"] + len(dets))
    )
    semantic.goal_pose._add_callback(lambda _: stats.__setitem__("semantic_goals", stats["semantic_goals"] + 1))
    nav.waypoint._add_callback(lambda _: stats.__setitem__("waypoints", stats["waypoints"] + 1))

    logger.info("Starting semantic full stack: world=%s render=%s", args.world, args.render)
    system.start()

    try:
        deadline = time.time() + max(args.warmup, 1.0)
        while time.time() < deadline and driver._engine is None:
            time.sleep(0.1)

        if driver._engine is None:
            logger.error("MuJoCo driver failed to initialize")
            return 2

        logger.info("Driver ready; warming up for %.1fs", args.warmup)
        time.sleep(max(args.warmup, 0.0))

        if args.goal:
            x, y = args.goal
            pose = PoseStamped(
                pose=Pose(
                    position=Vector3(float(x), float(y), 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
            logger.info("Injecting goal: (%.2f, %.2f)", x, y)
            nav.goal_pose._deliver(pose)

        if args.instruction.strip():
            logger.info("Injecting instruction: %s", args.instruction)
            semantic.instruction._deliver(args.instruction.strip())

        start = time.time()
        next_report = start
        end_time = start + max(args.duration, 0.0)
        while time.time() < end_time:
            now = time.time()
            if now >= next_report:
                logger.info(
                    "stats odom=%d scene_graph=%d detections=%d semantic_goals=%d waypoints=%d",
                    stats["odom"],
                    stats["scene_graph"],
                    stats["detections"],
                    stats["semantic_goals"],
                    stats["waypoints"],
                )
                next_report = now + 2.0
            time.sleep(0.1)
    finally:
        system.stop()

    logger.info("Run finished: %s", stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
