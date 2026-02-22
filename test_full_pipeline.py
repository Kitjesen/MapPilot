#!/usr/bin/env python3
"""
MapPilot Full Pipeline Test with Video Recording
完整测试: YOLO检测 → 场景图构建 → Goal Resolver → 路径规划 → 导航
"""
import sys
import os
import json
import time
import asyncio
import numpy as np
import cv2
from datetime import datetime

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")
sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_perception")

# API configuration
os.environ["KIMI_API_KEY"] = "sk-tpUJJs4F9dlNsPadiWXraVnN1x1PQhHnkbbqo9uMwvLkUtb8"

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

from semantic_planner.goal_resolver import GoalResolver
from semantic_planner.llm_client import LLMConfig

SCENE_PATH = "/home/bsrl/hongsenpang/habitat/data/hm3d_hf/minival/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"
OUTPUT_DIR = "/home/bsrl/hongsenpang/habitat/pipeline_output"

class PipelineTest:
    def __init__(self):
        self.sim = None
        self.resolver = None
        self.scene_graph = {"objects": [], "relations": [], "regions": []}
        self.video_writer = None
        self.frame_count = 0

    def setup_simulator(self):
        """Initialize Habitat simulator with RGB-D sensors."""
        print("\n[1] Setting up Habitat Simulator...")
        settings = default_sim_settings.copy()
        settings["width"] = 640
        settings["height"] = 480
        settings["scene"] = SCENE_PATH
        settings["sensor_height"] = 0.88
        settings["color_sensor"] = True
        settings["depth_sensor"] = True
        settings["seed"] = 42
        settings["enable_physics"] = True

        cfg = make_cfg(settings)
        self.sim = habitat_sim.Simulator(cfg)

        # Setup navmesh
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        navmesh_settings.agent_radius = 0.25
        navmesh_settings.agent_height = 0.88
        self.sim.recompute_navmesh(self.sim.pathfinder, navmesh_settings)

        print(f"    Scene: {SCENE_PATH.split('/')[-1]}")
        print(f"    Navigable area: {self.sim.pathfinder.navigable_area:.2f} m²")

    def setup_yolo_detector(self):
        """Initialize YOLO-World detector."""
        print("\n[2] Setting up YOLO-World Detector...")
        try:
            from ultralytics import YOLO
            self.yolo = YOLO("yolov8l-world.pt")
            self.detection_classes = [
                "chair", "table", "sofa", "couch", "bed", "desk",
                "tv", "television", "lamp", "door", "window",
                "refrigerator", "sink", "toilet", "cabinet", "shelf",
                "plant", "picture", "mirror", "rug", "curtain"
            ]
            self.yolo.set_classes(self.detection_classes)
            print(f"    YOLO-World loaded with {len(self.detection_classes)} classes")
            return True
        except Exception as e:
            print(f"    [WARNING] YOLO not available: {e}")
            print("    Using mock detections instead")
            self.yolo = None
            return False

    def setup_goal_resolver(self):
        """Initialize Goal Resolver with LLM."""
        print("\n[3] Setting up Goal Resolver...")
        config = LLMConfig(
            backend="openai",
            model="kimi-k2.5",
            api_key_env="KIMI_API_KEY",
            base_url="https://api.xiaocaseai.com/v1",
            timeout_sec=60.0,
            temperature=0.2,
        )
        self.resolver = GoalResolver(primary_config=config)
        print(f"    LLM: {config.model}")
        print("    Fast-Slow dual process ready")

    def detect_objects(self, rgb_image, depth_image, camera_pos):
        """Detect objects using YOLO-World and estimate 3D positions."""
        detections = []

        if self.yolo is not None:
            # Real YOLO detection
            results = self.yolo.predict(rgb_image, conf=0.3, verbose=False)
            boxes = results[0].boxes

            for i, box in enumerate(boxes):
                cls_id = int(box.cls.cpu().numpy()[0])
                conf = float(box.conf.cpu().numpy()[0])
                xyxy = box.xyxy.cpu().numpy()[0]

                # Get center of bounding box
                cx = int((xyxy[0] + xyxy[2]) / 2)
                cy = int((xyxy[1] + xyxy[3]) / 2)

                # Get depth at center (with bounds check)
                cy = min(cy, depth_image.shape[0] - 1)
                cx = min(cx, depth_image.shape[1] - 1)
                depth = depth_image[cy, cx]

                if depth > 0 and depth < 10:  # Valid depth
                    # Simple 3D position estimation
                    # (In real system, use camera intrinsics)
                    fx, fy = 320, 240  # Approximate focal length
                    x = (cx - 320) * depth / fx
                    y = (cy - 240) * depth / fy
                    z = depth

                    # Transform to world coordinates (simplified)
                    world_pos = [
                        float(camera_pos[0] + z),
                        float(camera_pos[1]),
                        float(camera_pos[2] + x)
                    ]

                    detections.append({
                        "id": len(self.scene_graph["objects"]) + len(detections) + 1,
                        "label": self.detection_classes[cls_id],
                        "position": world_pos,
                        "confidence": conf,
                        "bbox": xyxy.tolist(),
                        "depth": float(depth)
                    })
        else:
            # Mock detections based on position
            mock_objects = [
                ("chair", [2.0, 0.0, 1.0]),
                ("table", [3.0, 0.0, 2.0]),
                ("sofa", [-1.0, 0.0, 0.5]),
                ("bed", [0.0, 0.0, -3.0]),
                ("tv", [-2.0, 0.0, 0.0]),
            ]
            for i, (label, offset) in enumerate(mock_objects):
                detections.append({
                    "id": i + 1,
                    "label": label,
                    "position": [
                        float(camera_pos[0] + offset[0]),
                        float(camera_pos[1] + offset[1]),
                        float(camera_pos[2] + offset[2])
                    ],
                    "confidence": 0.85 + np.random.random() * 0.1
                })

        return detections

    def update_scene_graph(self, detections):
        """Update scene graph with new detections (simplified ConceptGraphs)."""
        for det in detections:
            # Check if object already exists (by label and proximity)
            exists = False
            for obj in self.scene_graph["objects"]:
                if obj["label"] == det["label"]:
                    dist = np.linalg.norm(
                        np.array(obj["position"]) - np.array(det["position"])
                    )
                    if dist < 1.0:  # Same object
                        # Update with higher confidence
                        if det["confidence"] > obj["confidence"]:
                            obj["position"] = det["position"]
                            obj["confidence"] = det["confidence"]
                        exists = True
                        break

            if not exists:
                self.scene_graph["objects"].append(det)

        # Update relations (simplified)
        self.scene_graph["relations"] = []
        objects = self.scene_graph["objects"]
        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i >= j:
                    continue
                dist = np.linalg.norm(
                    np.array(obj1["position"]) - np.array(obj2["position"])
                )
                if dist < 2.0:
                    self.scene_graph["relations"].append({
                        "subject_id": obj1["id"],
                        "predicate": "near",
                        "object_id": obj2["id"]
                    })

    def start_video(self, filename):
        """Start video recording."""
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        filepath = f"{OUTPUT_DIR}/{filename}"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(filepath, fourcc, 10, (640, 480))
        self.frame_count = 0
        print(f"    Recording to: {filepath}")

    def add_frame(self, rgb, info_text=""):
        """Add frame to video with overlay."""
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Add info overlay
        if info_text:
            lines = info_text.split('\n')
            y = 30
            for line in lines:
                cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                           0.6, (0, 255, 0), 2)
                y += 25

        self.video_writer.write(frame)
        self.frame_count += 1

    def stop_video(self):
        """Stop video recording."""
        if self.video_writer:
            self.video_writer.release()
            print(f"    Video saved: {self.frame_count} frames")

    def explore_and_build_scene_graph(self):
        """Explore the environment and build scene graph."""
        print("\n[4] Exploring environment and building scene graph...")

        agent = self.sim.get_agent(0)

        # Explore multiple viewpoints
        num_viewpoints = 8
        for i in range(num_viewpoints):
            # Get random navigable point
            pos = self.sim.pathfinder.get_random_navigable_point()

            # Move agent
            agent_state = agent.get_state()
            agent_state.position = pos
            agent.set_state(agent_state)

            # Get observation
            obs = self.sim.get_sensor_observations()
            rgb = obs["color_sensor"][:, :, :3]
            depth = obs["depth_sensor"]

            # Detect objects
            detections = self.detect_objects(rgb, depth, pos)

            # Update scene graph
            self.update_scene_graph(detections)

            print(f"    Viewpoint {i+1}/{num_viewpoints}: {len(detections)} detections")

        print(f"\n    Scene graph built:")
        print(f"      Objects: {len(self.scene_graph['objects'])}")
        for obj in self.scene_graph["objects"]:
            pos = obj["position"]
            print(f"        - {obj['label']}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}] (conf: {obj['confidence']:.2f})")
        print(f"      Relations: {len(self.scene_graph['relations'])}")

    async def navigate_to_goal(self, instruction):
        """Navigate to goal using Goal Resolver."""
        print(f'\n[5] Navigation Task: "{instruction}"')

        agent = self.sim.get_agent(0)

        # Start from random position
        start_pos = self.sim.pathfinder.get_random_navigable_point()
        agent_state = agent.get_state()
        agent_state.position = start_pos
        agent.set_state(agent_state)

        print(f"    Start position: [{start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f}]")

        # Start video
        timestamp = datetime.now().strftime("%H%M%S")
        video_name = f"nav_{timestamp}_{instruction[:20].replace(' ', '_')}.mp4"
        self.start_video(video_name)

        # Get initial observation
        obs = self.sim.get_sensor_observations()
        rgb = obs["color_sensor"][:, :, :3]
        self.add_frame(rgb, f"Instruction: {instruction}\nSearching for goal...")

        # Resolve goal
        scene_graph_json = json.dumps(self.scene_graph)

        print("    Resolving goal...")
        t0 = time.time()

        # Try Fast Path
        fast_result = self.resolver.fast_resolve(instruction, scene_graph_json)

        if fast_result and fast_result.is_valid:
            resolve_time = (time.time() - t0) * 1000
            print(f"    Fast Path [{resolve_time:.0f}ms]: {fast_result.target_label}")
            goal = np.array([fast_result.target_x, fast_result.target_y, fast_result.target_z])
            target_label = fast_result.target_label
            path_type = "Fast Path"
        else:
            print("    Fast Path: No match, trying Slow Path (LLM)...")

            # Try Slow Path
            result = await self.resolver.resolve(instruction, scene_graph_json)
            resolve_time = (time.time() - t0) * 1000

            if result and result.is_valid:
                print(f"    Slow Path [{resolve_time:.0f}ms]: {result.target_label}")
                print(f"    Reasoning: {result.reasoning[:80]}...")
                goal = np.array([result.target_x, result.target_y, result.target_z])
                target_label = result.target_label
                path_type = "Slow Path (LLM)"
            else:
                print(f"    Failed to resolve goal: {result.error if result else 'No result'}")
                self.stop_video()
                return False

        # Plan path
        print(f"    Goal: {target_label} at [{goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}]")

        path = habitat_sim.ShortestPath()
        path.requested_start = start_pos
        path.requested_end = goal

        if not self.sim.pathfinder.find_path(path):
            print("    No path found!")
            self.stop_video()
            return False

        print(f"    Path planned: {len(path.points)} waypoints, {path.geodesic_distance:.2f}m")

        # Navigate
        print("    Navigating...")
        for i, waypoint in enumerate(path.points):
            # Move agent
            agent_state = agent.get_state()
            agent_state.position = waypoint
            agent.set_state(agent_state)

            # Get observation
            obs = self.sim.get_sensor_observations()
            rgb = obs["color_sensor"][:, :, :3]

            # Calculate progress
            dist_to_goal = np.linalg.norm(np.array(waypoint) - goal)
            progress = (1 - dist_to_goal / path.geodesic_distance) * 100

            # Add frame with overlay
            info = f"Target: {target_label} ({path_type})\n"
            info += f"Step: {i+1}/{len(path.points)}\n"
            info += f"Distance: {dist_to_goal:.2f}m\n"
            info += f"Progress: {progress:.0f}%"
            self.add_frame(rgb, info)

        # Final frame
        obs = self.sim.get_sensor_observations()
        rgb = obs["color_sensor"][:, :, :3]
        final_dist = np.linalg.norm(agent.get_state().position - goal)
        success = final_dist < 1.0

        info = f"Target: {target_label}\n"
        info += f"Status: {'SUCCESS!' if success else 'FAILED'}\n"
        info += f"Final distance: {final_dist:.2f}m"

        # Add multiple final frames
        for _ in range(20):
            self.add_frame(rgb, info)

        self.stop_video()

        print(f"    Navigation {'SUCCESS' if success else 'FAILED'} (final dist: {final_dist:.2f}m)")
        return success

    async def run_full_pipeline(self):
        """Run the complete pipeline test."""
        print("="*70)
        print("  MapPilot Full Pipeline Test")
        print("  YOLO Detection → Scene Graph → Goal Resolver → Navigation")
        print("="*70)

        # Setup
        self.setup_simulator()
        self.setup_yolo_detector()
        self.setup_goal_resolver()

        # Build scene graph
        self.explore_and_build_scene_graph()

        # Test navigation with different instructions
        instructions = [
            "Go to the chair",
            "Find a place to sit and relax",
            "Navigate to the table",
        ]

        results = []
        for instr in instructions:
            try:
                success = await self.navigate_to_goal(instr)
                results.append({"instruction": instr, "success": success})
            except Exception as e:
                print(f"    Error: {e}")
                results.append({"instruction": instr, "success": False})

        # Summary
        print("\n" + "="*70)
        print("  PIPELINE TEST SUMMARY")
        print("="*70)

        success_count = sum(1 for r in results if r["success"])
        print(f"\n  Results: {success_count}/{len(results)} tasks completed")

        for r in results:
            status = "✓" if r["success"] else "✗"
            print(f"    [{status}] {r['instruction']}")

        print(f"\n  Videos saved to: {OUTPUT_DIR}")
        print("="*70)

        # Cleanup
        self.sim.close()

        return results

async def main():
    test = PipelineTest()
    await test.run_full_pipeline()

if __name__ == "__main__":
    asyncio.run(main())
