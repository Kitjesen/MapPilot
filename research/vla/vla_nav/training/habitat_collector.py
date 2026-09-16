"""
Habitat Data Collector — Collect expert navigation trajectories for VLA training.

Reference: VLingNav (arXiv 2601.08665) Section 3.4
  - Uses Habitat simulator with HM3D/MP3D scenes
  - Expert policy: shortest-path follower (oracle)
  - Records: RGB observation, position, heading, action, instruction

Data collection pipeline:
  1. Load scene from HM3D/MP3D dataset
  2. Generate navigation episodes (start → goal with instruction)
  3. Run shortest-path expert policy
  4. Save trajectories as a structured dataset

Target: 50K-100K trajectories (VLingNav used 2.9M, we're limited by compute)
"""

import json
import logging
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class TrajectoryStep:
    """A single step in an expert navigation trajectory."""
    step_idx: int
    rgb_path: str               # Path to saved RGB image
    position: List[float]       # [x, y, z] in world frame
    heading: float              # yaw in radians
    action: List[float]         # [dx, dy, dtheta] relative to robot frame
    target_position: List[float]  # [x, y, z] of the goal
    distance_to_goal: float     # meters
    is_terminal: bool           # True if this is the last step


@dataclass
class TrajectoryRecord:
    """A complete navigation trajectory with metadata."""
    episode_id: str
    scene_id: str
    instruction: str            # e.g. "Navigate to the kitchen chair"
    object_category: str        # e.g. "chair"
    start_position: List[float]
    goal_position: List[float]
    steps: List[TrajectoryStep] = field(default_factory=list)
    success: bool = False
    total_distance: float = 0.0
    num_steps: int = 0


# ---------------------------------------------------------------------------
# Instruction templates for ObjectNav
# ---------------------------------------------------------------------------
INSTRUCTION_TEMPLATES = [
    "Go to the {object}.",
    "Navigate to the {object}.",
    "Find the {object} and go to it.",
    "Walk towards the {object}.",
    "I need you to go to the {object}.",
    "Please find a {object} in this room.",
    "Can you navigate to where the {object} is?",
    "Move to the {object}.",
    "去找到{object}。",
    "导航到{object}的位置。",
    "请走到{object}那里。",
    "帮我找到{object}。",
]


def generate_instruction(object_category: str, template_idx: Optional[int] = None) -> str:
    """Generate a natural language instruction for ObjectNav."""
    if template_idx is None:
        template_idx = hash(object_category) % len(INSTRUCTION_TEMPLATES)
    template = INSTRUCTION_TEMPLATES[template_idx % len(INSTRUCTION_TEMPLATES)]
    return template.format(object=object_category)


class HabitatCollector:
    """
    Collect expert navigation trajectories from Habitat simulator.

    Usage:
        collector = HabitatCollector(
            scene_dataset="data/scene_datasets/hm3d",
            output_dir="data/trajectories",
        )
        collector.collect(num_episodes=50000)
    """

    def __init__(
        self,
        scene_dataset: str = "data/scene_datasets/hm3d/hm3d_annotated_basis.scene_dataset_config.json",
        output_dir: str = "data/trajectories",
        image_size: Tuple[int, int] = (480, 640),
        max_steps_per_episode: int = 500,
        success_distance: float = 1.0,
        seed: int = 42,
    ):
        self.scene_dataset = scene_dataset
        self.output_dir = Path(output_dir)
        self.image_size = image_size
        self.max_steps = max_steps_per_episode
        self.success_distance = success_distance
        self.seed = seed

        self._sim = None
        self._rng = np.random.RandomState(seed)

    def _init_habitat(self):
        """Initialize Habitat simulator with ObjectNav configuration."""
        try:
            import habitat
            import habitat_sim
            from habitat.config.default import get_config
        except ImportError:
            raise ImportError(
                "Habitat not installed. Run:\n"
                "  conda install habitat-sim headless -c conda-forge -c aihabitat\n"
                "  pip install habitat-lab"
            )

        logger.info("Initializing Habitat with scene dataset: %s", self.scene_dataset)

        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_dataset_config_file = self.scene_dataset
        sim_cfg.gpu_device_id = 0

        agent_cfg = habitat_sim.agent.AgentConfiguration()
        agent_cfg.height = 0.88  # Quadruped robot height
        agent_cfg.radius = 0.25

        # RGB sensor
        rgb_sensor = habitat_sim.CameraSensorSpec()
        rgb_sensor.uuid = "rgb"
        rgb_sensor.sensor_type = habitat_sim.SensorType.COLOR
        rgb_sensor.resolution = list(self.image_size)
        rgb_sensor.position = [0.0, agent_cfg.height, 0.0]
        rgb_sensor.hfov = 79  # Typical RealSense D435 horizontal FOV
        agent_cfg.sensor_specifications = [rgb_sensor]

        cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])
        self._sim = habitat_sim.Simulator(cfg)

        logger.info("Habitat simulator initialized")

    def _get_shortest_path(
        self,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> Optional[List[np.ndarray]]:
        """Compute shortest path between two navigable points."""
        path = habitat_sim.ShortestPath()
        path.requested_start = start.tolist()
        path.requested_end = goal.tolist()

        if self._sim.pathfinder.find_path(path):
            return [np.array(p) for p in path.points]
        return None

    def _compute_action(
        self,
        current_pos: np.ndarray,
        current_heading: float,
        next_pos: np.ndarray,
    ) -> List[float]:
        """Compute (dx, dy, dtheta) action in the robot's local frame."""
        # World-frame displacement
        world_dx = next_pos[0] - current_pos[0]
        world_dy = next_pos[2] - current_pos[2]  # Habitat uses x-z plane

        # Rotate to robot frame
        cos_h = np.cos(-current_heading)
        sin_h = np.sin(-current_heading)
        local_dx = world_dx * cos_h - world_dy * sin_h
        local_dy = world_dx * sin_h + world_dy * cos_h

        # Compute heading change
        target_heading = np.arctan2(world_dx, -world_dy)
        dtheta = target_heading - current_heading
        # Normalize to [-pi, pi]
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

        return [float(local_dx), float(local_dy), float(dtheta)]

    def collect_episode(
        self,
        scene_id: str,
        start_pos: np.ndarray,
        goal_pos: np.ndarray,
        object_category: str,
        episode_id: str,
    ) -> Optional[TrajectoryRecord]:
        """
        Collect one expert trajectory for a given start/goal pair.

        Returns None if the episode fails (no path, etc.).
        """
        path_points = self._get_shortest_path(start_pos, goal_pos)
        if path_points is None or len(path_points) < 2:
            return None

        instruction = generate_instruction(
            object_category,
            template_idx=self._rng.randint(len(INSTRUCTION_TEMPLATES)),
        )

        record = TrajectoryRecord(
            episode_id=episode_id,
            scene_id=scene_id,
            instruction=instruction,
            object_category=object_category,
            start_position=start_pos.tolist(),
            goal_position=goal_pos.tolist(),
        )

        # Episode output directory
        ep_dir = self.output_dir / episode_id
        ep_dir.mkdir(parents=True, exist_ok=True)

        # Interpolate path to get dense waypoints
        dense_points = self._interpolate_path(path_points, step_size=0.25)

        agent = self._sim.get_agent(0)
        state = habitat_sim.AgentState()
        state.position = start_pos
        agent.set_state(state)

        heading = 0.0
        total_dist = 0.0

        for i, point in enumerate(dense_points):
            is_last = (i == len(dense_points) - 1)

            # Get RGB observation
            obs = self._sim.get_sensor_observations()
            rgb = obs["rgb"][:, :, :3]  # Drop alpha channel

            # Save image
            img_path = str(ep_dir / f"rgb_{i:04d}.jpg")
            self._save_image(rgb, img_path)

            # Compute action to next point
            if not is_last:
                next_point = dense_points[i + 1]
                action = self._compute_action(point, heading, next_point)
                step_dist = float(np.linalg.norm(next_point[:2] - point[:2]))
                total_dist += step_dist
            else:
                action = [0.0, 0.0, 0.0]

            dist_to_goal = float(np.linalg.norm(point - goal_pos))

            step = TrajectoryStep(
                step_idx=i,
                rgb_path=img_path,
                position=point.tolist(),
                heading=float(heading),
                action=action,
                target_position=goal_pos.tolist(),
                distance_to_goal=dist_to_goal,
                is_terminal=is_last,
            )
            record.steps.append(step)

            # Move agent for next observation
            if not is_last:
                state = habitat_sim.AgentState()
                state.position = dense_points[i + 1]
                heading += action[2]
                agent.set_state(state)

            if i >= self.max_steps:
                break

        record.success = record.steps[-1].distance_to_goal <= self.success_distance
        record.total_distance = total_dist
        record.num_steps = len(record.steps)

        # Save metadata
        meta_path = ep_dir / "trajectory.json"
        self._save_metadata(record, meta_path)

        return record

    def collect(
        self,
        num_episodes: int = 50000,
        scenes: Optional[List[str]] = None,
        categories: Optional[List[str]] = None,
    ):
        """
        Run the full data collection pipeline.

        Args:
            num_episodes: Target number of episodes
            scenes:       List of scene IDs (None = use all available)
            categories:   Object categories to navigate to
        """
        if self._sim is None:
            self._init_habitat()

        if categories is None:
            categories = [
                "chair", "couch", "bed", "toilet", "tv_monitor",
                "dining_table", "refrigerator", "sink", "oven",
                "potted_plant", "book", "clock", "vase",
            ]

        self.output_dir.mkdir(parents=True, exist_ok=True)

        collected = 0
        failed = 0
        t_start = time.time()

        logger.info("Starting data collection: target=%d episodes", num_episodes)

        while collected < num_episodes:
            # Sample random navigable points
            start = self._sim.pathfinder.get_random_navigable_point()
            goal = self._sim.pathfinder.get_random_navigable_point()

            if np.linalg.norm(start - goal) < 2.0:
                continue  # Too close

            category = categories[self._rng.randint(len(categories))]
            episode_id = f"ep_{collected:06d}"

            record = self.collect_episode(
                scene_id="default",
                start_pos=start,
                goal_pos=goal,
                object_category=category,
                episode_id=episode_id,
            )

            if record is not None and record.success:
                collected += 1
                if collected % 1000 == 0:
                    elapsed = time.time() - t_start
                    rate = collected / elapsed
                    logger.info(
                        "Collected %d/%d (%.1f eps/sec, %d failed)",
                        collected, num_episodes, rate, failed,
                    )
            else:
                failed += 1

        elapsed = time.time() - t_start
        logger.info(
            "Collection complete: %d episodes in %.1f min (%.1f eps/sec)",
            collected, elapsed / 60, collected / elapsed,
        )

        # Save manifest
        manifest = {
            "num_episodes": collected,
            "categories": categories,
            "output_dir": str(self.output_dir),
            "seed": self.seed,
            "collection_time_min": elapsed / 60,
        }
        with open(self.output_dir / "manifest.json", "w") as f:
            json.dump(manifest, f, indent=2)

    # ---- Utilities -----------------------------------------------------------

    @staticmethod
    def _interpolate_path(
        points: List[np.ndarray],
        step_size: float = 0.25,
    ) -> List[np.ndarray]:
        """Interpolate between path points to get dense waypoints."""
        dense = [points[0]]
        for i in range(1, len(points)):
            start = points[i - 1]
            end = points[i]
            dist = float(np.linalg.norm(end - start))
            if dist < 1e-6:
                continue
            n_interp = max(1, int(dist / step_size))
            for j in range(1, n_interp + 1):
                t = j / n_interp
                interp = start + t * (end - start)
                dense.append(interp)
        return dense

    @staticmethod
    def _save_image(rgb: np.ndarray, path: str):
        """Save RGB image to disk."""
        import cv2
        cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    @staticmethod
    def _save_metadata(record: TrajectoryRecord, path: Path):
        """Save trajectory metadata as JSON."""
        data = {
            "episode_id": record.episode_id,
            "scene_id": record.scene_id,
            "instruction": record.instruction,
            "object_category": record.object_category,
            "start_position": record.start_position,
            "goal_position": record.goal_position,
            "success": record.success,
            "total_distance": record.total_distance,
            "num_steps": record.num_steps,
            "steps": [
                {
                    "step_idx": s.step_idx,
                    "rgb_path": s.rgb_path,
                    "position": s.position,
                    "heading": s.heading,
                    "action": s.action,
                    "distance_to_goal": s.distance_to_goal,
                    "is_terminal": s.is_terminal,
                }
                for s in record.steps
            ],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Collect Habitat navigation trajectories")
    parser.add_argument("--scene-dataset", type=str, required=True,
                        help="Path to scene_dataset_config.json")
    parser.add_argument("--output-dir", type=str, default="data/trajectories")
    parser.add_argument("--num-episodes", type=int, default=50000)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    collector = HabitatCollector(
        scene_dataset=args.scene_dataset,
        output_dir=args.output_dir,
        seed=args.seed,
    )
    collector.collect(num_episodes=args.num_episodes)
