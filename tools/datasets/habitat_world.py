#!/usr/bin/env python3
"""Small JSON-line Habitat process for the RGB-D navigation experiment."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def main() -> None:
    import habitat_sim
    from habitat_sim.utils.common import quat_from_angle_axis, quat_to_coeffs
    from PIL import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--navmesh", required=True)
    parser.add_argument("--scene", default="apt_0")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "rgb").mkdir(exist_ok=True)
    (args.output / "depth").mkdir(exist_ok=True)
    (args.output / "identity").mkdir(exist_ok=True)
    config = habitat_sim.SimulatorConfiguration()
    config.scene_dataset_config_file = args.dataset
    config.scene_id = args.scene
    config.enable_physics = True
    sensors = []
    for name, kind in (
        ("rgb", habitat_sim.SensorType.COLOR),
        ("depth", habitat_sim.SensorType.DEPTH),
        ("identity", habitat_sim.SensorType.SEMANTIC),
    ):
        sensor = habitat_sim.CameraSensorSpec()
        sensor.uuid, sensor.sensor_type = name, kind
        sensor.resolution = [480, 640]
        sensor.position = [0.0, 1.2, 0.0]
        sensor.hfov = 90.0
        sensors.append(sensor)
    agent_config = habitat_sim.agent.AgentConfiguration()
    agent_config.sensor_specifications = sensors
    agent_config.action_space = {
        "forward": habitat_sim.agent.ActionSpec("move_forward", habitat_sim.agent.ActuationSpec(amount=0.05)),
        "left": habitat_sim.agent.ActionSpec("turn_left", habitat_sim.agent.ActuationSpec(amount=3.0)),
        "right": habitat_sim.agent.ActionSpec("turn_right", habitat_sim.agent.ActuationSpec(amount=3.0)),
    }
    index = 0
    with habitat_sim.Simulator(habitat_sim.Configuration(config, [agent_config])) as sim:
        object_map = {}
        rigid_objects = sim.get_rigid_object_manager().get_objects_by_handle_substring("")
        for semantic_id, obj in enumerate(sorted(rigid_objects.values(), key=lambda item: item.handle), 1_000_000):
            obj.semantic_id = semantic_id
            object_map[str(semantic_id)] = {
                "handle": obj.handle,
                "object_id": int(obj.object_id),
                "semantic_id": int(obj.semantic_id),
            }
        (args.output / "identity" / "object-map.json").write_text(
            json.dumps(object_map, indent=2) + "\n",
        )
        if not sim.pathfinder.load_nav_mesh(args.navmesh):
            raise RuntimeError("official scene navmesh could not be loaded")
        sim.pathfinder.seed(args.seed)
        island = max(range(sim.pathfinder.num_islands), key=sim.pathfinder.island_area)
        position = sim.pathfinder.get_random_navigable_point(island_index=island)
        agent = sim.initialize_agent(0)
        state = habitat_sim.AgentState()
        state.position = position
        state.rotation = quat_from_angle_axis(0, np.array([0, 1, 0]))
        agent.set_state(state)

        def snapshot(save_depth=True):
            nonlocal index
            observations = sim.get_sensor_observations()
            name = f"rgb/{index:05d}.jpg"
            Image.fromarray(observations["rgb"][..., :3]).save(args.output / name, quality=92)
            camera = agent.get_state().sensor_states["rgb"]
            depth_name = f"depth/{index:05d}.npy"
            if save_depth:
                np.save(args.output / depth_name, observations["depth"])
            response = {"index": index, "rgb": name, "depth": depth_name,
                        "navmesh_start_island": island, "navmesh_start_island_area": sim.pathfinder.island_area(island),
                        "agent_position": agent.get_state().position.tolist(),
                        "camera_position": camera.position.tolist(),
                        "camera_rotation_xyzw": quat_to_coeffs(camera.rotation).tolist()}
            index += 1
            return response

        for line in sys.stdin:
            request = json.loads(line)
            command = request["command"]
            if command == "close":
                break
            if command == "observe":
                response = snapshot()
            elif command == "geometry":
                import magnum as mn

                observations = sim.get_sensor_observations()
                camera = sim._sensors["depth"]._sensor_object.render_camera
                samples = []
                for u, v in ((40, 40), (320, 240), (600, 40), (40, 440), (600, 440)):
                    ray = camera.unproject(mn.Vector2i(u, v), normalized=False)
                    depth = float(observations["depth"][v, u])
                    ndc = mn.Vector3(2*(u+0.5)/640-1, 1-2*(v+0.5)/480, -1)
                    point = camera.projection_matrix.inverted().transform_point(ndc)
                    point *= depth / -point.z
                    world_point = camera.camera_matrix.inverted().transform_point(point)
                    samples.append({"u": u, "v": v, "depth": float(observations["depth"][v, u]),
                                    "origin": list(ray.origin), "direction": list(ray.direction),
                                    "renderer_world_point": list(world_point)})
                response = {"samples": samples}
            elif command == "identity":
                observations = sim.get_sensor_observations()
                identity = np.asarray(observations["identity"], dtype=np.uint32)
                frame_index = int(request.get("index", index - 1))
                name = f"identity/{frame_index:05d}.npy"
                np.save(args.output / name, identity)
                ids, counts = np.unique(identity, return_counts=True)
                visible = {
                    str(int(identity_id)): int(count)
                    for identity_id, count in zip(ids, counts)
                    if str(int(identity_id)) in object_map
                }
                response = {
                    "index": frame_index,
                    "identity": name,
                    "object_map": "identity/object-map.json",
                    "visible_object_pixels": visible,
                }
            elif command == "step":
                old = agent.get_state().position.copy()
                video_frames = []
                collisions = 0
                for micro in range(5 if request["action"] != "forward" else 4):
                    collisions += int(sim.step(request["action"])["collided"])
                    video_frames.append(snapshot(save_depth=micro == (4 if request["action"] != "forward" else 3)))
                response = {**video_frames[-1], "video_frames": video_frames,
                            "collisions": collisions,
                            "translation_m": float(np.linalg.norm(agent.get_state().position-old))}
            elif command == "paths":
                response = {"paths": []}
                for goal in request["goals"]:
                    raw = np.asarray(goal, dtype=np.float32)
                    # Experiment Z-up -> Habitat Y-up.
                    habitat = raw[[0, 2, 1]] * np.array([1, 1, -1], dtype=np.float32)
                    snapped = sim.pathfinder.snap_point(habitat, island_index=island)
                    if not np.isfinite(snapped).all() or np.linalg.norm(snapped-habitat) > 0.35:
                        response["paths"].append(None)
                        continue
                    path = habitat_sim.ShortestPath()
                    path.requested_start = agent.get_state().position
                    path.requested_end = snapped
                    if sim.pathfinder.find_path(path):
                        points = np.asarray(path.points)
                        points = points[:, [0, 2, 1]] * [1, -1, 1]
                        response["paths"].append({"points": points.tolist(), "length": path.geodesic_distance,
                                                  "goal": points[-1].tolist()})
                    else:
                        response["paths"].append(None)
            else:
                raise ValueError(command)
            print(json.dumps(response), flush=True)


if __name__ == "__main__":
    main()
