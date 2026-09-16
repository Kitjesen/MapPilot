"""
Navigation Dataset — PyTorch Dataset for VLA training.

Loads annotated trajectories and produces training samples:
  - Input:  instruction text + RGB images (multi-frame)
  - Labels: AdaCoT trigger (THINK/NO_THINK), CoT text, waypoint actions

Reference: VLingNav (arXiv 2601.08665) training pipeline
"""

import json
import logging
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset

logger = logging.getLogger(__name__)


class NavTrajectoryDataset(Dataset):
    """
    PyTorch Dataset for VLA navigation training.

    Each sample is a single trajectory step, with context from surrounding steps.

    Sample structure:
        {
            "instruction":        str,
            "images":             List[np.ndarray],  # context_window RGB images
            "adacot_label":       int,               # 0 = NO_THINK, 1 = THINK
            "cot_text":           str,               # CoT reasoning (empty if NO_THINK)
            "action":             np.ndarray,         # (3,) = (dx, dy, dtheta)
            "future_actions":     np.ndarray,         # (horizon, 3) = future waypoints
            "position":           np.ndarray,         # (3,) current position
            "heading":            float,
            "distance_to_goal":   float,
        }
    """

    def __init__(
        self,
        data_dir: str,
        context_window: int = 5,
        action_horizon: int = 5,
        max_episodes: Optional[int] = None,
        image_size: Tuple[int, int] = (224, 224),
        split: str = "train",
        train_ratio: float = 0.95,
    ):
        self.data_dir = Path(data_dir)
        self.context_window = context_window
        self.action_horizon = action_horizon
        self.image_size = image_size

        # Load all annotated episodes
        self.samples: List[dict] = []
        self._load_episodes(max_episodes, split, train_ratio)

        logger.info(
            "NavTrajectoryDataset: %d samples from %s (split=%s, ctx=%d, horizon=%d)",
            len(self.samples), data_dir, split, context_window, action_horizon,
        )

    def _load_episodes(
        self,
        max_episodes: Optional[int],
        split: str,
        train_ratio: float,
    ):
        """Load annotated trajectory files and build sample index."""
        episode_dirs = sorted([
            d for d in self.data_dir.iterdir()
            if d.is_dir() and (d / "trajectory_annotated.json").exists()
        ])

        if max_episodes:
            episode_dirs = episode_dirs[:max_episodes]

        # Split
        n_train = int(len(episode_dirs) * train_ratio)
        if split == "train":
            episode_dirs = episode_dirs[:n_train]
        elif split == "val":
            episode_dirs = episode_dirs[n_train:]

        for ep_dir in episode_dirs:
            try:
                self._load_episode(ep_dir)
            except Exception as e:
                logger.warning("Failed to load %s: %s", ep_dir.name, e)

    def _load_episode(self, ep_dir: Path):
        """Load one annotated episode and create per-step samples."""
        with open(ep_dir / "trajectory_annotated.json") as f:
            traj = json.load(f)

        instruction = traj["instruction"]
        steps = traj["steps"]

        for i, step in enumerate(steps):
            # Gather context window of image paths
            start_ctx = max(0, i - self.context_window + 1)
            context_steps = steps[start_ctx: i + 1]
            image_paths = [s["rgb_path"] for s in context_steps]

            # Gather future actions for action horizon
            future_actions = []
            for j in range(i, min(i + self.action_horizon, len(steps))):
                future_actions.append(steps[j]["action"])
            # Pad if at end of trajectory
            while len(future_actions) < self.action_horizon:
                future_actions.append([0.0, 0.0, 0.0])

            sample = {
                "episode_dir": str(ep_dir),
                "instruction": instruction,
                "image_paths": image_paths,
                "adacot_label": 1 if step.get("adacot_label") == "THINK" else 0,
                "cot_text": step.get("cot_text", ""),
                "action": step["action"],
                "future_actions": future_actions,
                "position": step["position"],
                "heading": step["heading"],
                "distance_to_goal": step["distance_to_goal"],
                "step_idx": step["step_idx"],
            }
            self.samples.append(sample)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict:
        sample = self.samples[idx]

        # Load images (lazy, to save memory)
        images = []
        for path in sample["image_paths"]:
            img = self._load_image(path)
            images.append(img)

        return {
            "instruction": sample["instruction"],
            "images": images,
            "adacot_label": torch.tensor(sample["adacot_label"], dtype=torch.long),
            "cot_text": sample["cot_text"],
            "action": torch.tensor(sample["action"], dtype=torch.float32),
            "future_actions": torch.tensor(
                sample["future_actions"], dtype=torch.float32,
            ),
            "position": torch.tensor(sample["position"], dtype=torch.float32),
            "heading": torch.tensor(sample["heading"], dtype=torch.float32),
            "distance_to_goal": torch.tensor(
                sample["distance_to_goal"], dtype=torch.float32,
            ),
        }

    def _load_image(self, path: str) -> np.ndarray:
        """Load and resize an RGB image."""
        import cv2

        if os.path.exists(path):
            img = cv2.imread(path)
            if img is not None:
                img = cv2.resize(img, (self.image_size[1], self.image_size[0]))
                return img

        # Fallback: black image
        return np.zeros(
            (self.image_size[0], self.image_size[1], 3), dtype=np.uint8,
        )

    def get_statistics(self) -> dict:
        """Compute dataset statistics."""
        think_count = sum(1 for s in self.samples if s["adacot_label"] == 1)
        return {
            "total_samples": len(self.samples),
            "think_samples": think_count,
            "no_think_samples": len(self.samples) - think_count,
            "think_ratio": think_count / max(len(self.samples), 1),
            "cot_text_available": sum(
                1 for s in self.samples if s.get("cot_text")
            ),
        }


def collate_nav_batch(batch: List[dict]) -> dict:
    """Custom collate function for NavTrajectoryDataset."""
    return {
        "instructions": [b["instruction"] for b in batch],
        "images": [b["images"] for b in batch],
        "adacot_labels": torch.stack([b["adacot_label"] for b in batch]),
        "cot_texts": [b["cot_text"] for b in batch],
        "actions": torch.stack([b["action"] for b in batch]),
        "future_actions": torch.stack([b["future_actions"] for b in batch]),
        "positions": torch.stack([b["position"] for b in batch]),
        "headings": torch.stack([b["heading"] for b in batch]),
        "distances_to_goal": torch.stack([b["distance_to_goal"] for b in batch]),
    }
