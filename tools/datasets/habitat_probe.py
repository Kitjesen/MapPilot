#!/usr/bin/env python3
"""Record synchronized RGB-D views of an official Habitat scene; no navigation."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def record(dataset: Path, scene: str, output: Path, seed: int = 7, navmesh: Path | None = None) -> dict:
    import habitat_sim
    from habitat_sim.utils.common import quat_from_angle_axis, quat_to_coeffs
    from PIL import Image

    width, height, hfov, sensor_height = 640, 480, 90.0, 1.2
    sim_config = habitat_sim.SimulatorConfiguration()
    sim_config.scene_dataset_config_file = str(dataset.resolve())
    sim_config.scene_id = scene
    sim_config.enable_physics = True
    sensors = []
    for name, kind in (("rgb", habitat_sim.SensorType.COLOR), ("depth", habitat_sim.SensorType.DEPTH)):
        sensor = habitat_sim.CameraSensorSpec()
        sensor.uuid, sensor.sensor_type = name, kind
        sensor.resolution = [height, width]
        sensor.position = [0.0, sensor_height, 0.0]
        sensor.hfov = hfov
        sensors.append(sensor)
    agent_config = habitat_sim.agent.AgentConfiguration()
    agent_config.sensor_specifications = sensors
    output.mkdir(parents=True, exist_ok=True)
    frames = []
    sheet = Image.new("RGB", (width * 4, height * 2))
    with habitat_sim.Simulator(habitat_sim.Configuration(sim_config, [agent_config])) as sim:
        if navmesh is not None and not sim.pathfinder.load_nav_mesh(str(navmesh.resolve())):
            raise RuntimeError(f"could not load the supplied scene navmesh: {navmesh}")
        if not sim.pathfinder.is_loaded:
            raise RuntimeError("official scene navmesh is not loaded")
        sim.pathfinder.seed(seed)
        position = sim.pathfinder.get_random_navigable_point()
        if not np.isfinite(position).all():
            raise RuntimeError("no navigable sample in scene navmesh")
        agent = sim.initialize_agent(0)
        for index in range(8):
            state = habitat_sim.AgentState()
            state.position = position
            state.rotation = quat_from_angle_axis(index * math.pi / 4, np.array([0, 1, 0]))
            agent.set_state(state)
            observation = sim.get_sensor_observations()
            rgb = Image.fromarray(observation["rgb"][..., :3])
            rgb_path, depth_path = f"{index:02d}.png", f"{index:02d}-depth.npy"
            rgb.save(output / rgb_path)
            sheet.paste(rgb, ((index % 4) * width, (index // 4) * height))
            depth = observation["depth"]
            np.save(output / depth_path, depth)
            camera = agent.get_state().sensor_states["rgb"]
            frames.append({
                "index": index, "rgb": rgb_path, "depth": depth_path,
                "camera_position": camera.position.tolist(),
                "camera_rotation_xyzw": quat_to_coeffs(camera.rotation).tolist(),
                "valid_depth_fraction": float(np.mean(np.isfinite(depth) & (depth > 0))),
            })
    sheet.save(output / "views.jpg", quality=90)
    focal = width / (2 * math.tan(math.radians(hfov) / 2))
    report = {
        "evidence": "habitat_rgbd_render_only", "dataset": str(dataset), "scene": scene, "seed": seed,
        "habitat_sim_version": habitat_sim.__version__,
        "navmesh": str(navmesh) if navmesh else "scene_default",
        "agent_position": np.asarray(position).tolist(), "frames": frames,
        "camera": {"width": width, "height": height, "hfov_degrees": hfov,
                   "fx": focal, "fy": focal, "cx": width / 2, "cy": height / 2,
                   "depth_units": "meters", "world_frame": "Habitat Y-up",
                   "camera_axes": "OpenGL: +X right, +Y up, -Z forward"},
        "sensor_pairing": "rgb and depth returned by the same get_sensor_observations call",
        "robot_motion_executed": False, "detector_executed": False, "motion_authorization": False,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--scene", default="apt_0")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--navmesh", type=Path, help="explicit official navmesh for the selected scene")
    args = parser.parse_args()
    report = record(args.dataset, args.scene, args.output, args.seed, args.navmesh)
    print(json.dumps({"evidence": report["evidence"], "scene": args.scene, "frames": len(report["frames"])}))


if __name__ == "__main__":
    main()
