"""
AdaCoT Annotation Pipeline — Label each trajectory step as THINK or NO_THINK.

Reference: VLingNav (arXiv 2601.08665) Section 3.4
  - Nav-AdaCoT-2.9M dataset has adaptive CoT annotations
  - Annotations encode both "when to think" and "what to think about"
  - THINK steps get full CoT reasoning text; NO_THINK steps get direct action

Annotation strategy (rule-based + VLM refinement):
  Rule-based triggers for THINK:
    1. Heading change > 30 degrees (turning point)
    2. First step and last 3 steps of episode
    3. Distance to goal crosses thresholds (5m, 2m, 1m)
    4. Long straight segment ending (> 5 steps straight → THINK at end)
    5. Significant change in surroundings (feature delta)

  VLM refinement (optional, uses GPT-4o):
    For each THINK step, generate structured CoT reasoning text:
      [Observation] What the agent sees
      [Memory]      Relevant past observations
      [Instruction] What the goal asks
      [Reasoning]   Why to go in this direction
"""

import json
import logging
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Rule-based THINK triggers
# ---------------------------------------------------------------------------
class RuleBasedTrigger:
    """
    Determine which trajectory steps should be labelled THINK vs NO_THINK.

    The rules are designed to approximate the VLingNav finding that ~30% of
    steps benefit from explicit reasoning (the rest can use System 1).
    """

    def __init__(
        self,
        heading_change_threshold: float = 0.52,   # ~30 degrees
        goal_distance_thresholds: List[float] = None,
        straight_segment_length: int = 5,
        first_n_think: int = 1,
        last_n_think: int = 3,
        target_think_ratio: float = 0.30,
    ):
        self.heading_change_threshold = heading_change_threshold
        self.goal_thresholds = goal_distance_thresholds or [5.0, 2.0, 1.0]
        self.straight_segment_length = straight_segment_length
        self.first_n_think = first_n_think
        self.last_n_think = last_n_think
        self.target_think_ratio = target_think_ratio

    def annotate(self, steps: List[dict]) -> List[bool]:
        """
        Label each step as THINK (True) or NO_THINK (False).

        Args:
            steps: List of step dicts from trajectory JSON

        Returns:
            List of bool, same length as steps
        """
        n = len(steps)
        if n == 0:
            return []

        labels = [False] * n

        # Rule 1: First step(s)
        for i in range(min(self.first_n_think, n)):
            labels[i] = True

        # Rule 2: Last steps
        for i in range(max(0, n - self.last_n_think), n):
            labels[i] = True

        # Rule 3: Heading change > threshold
        for i in range(1, n):
            prev_heading = steps[i - 1]["heading"]
            curr_heading = steps[i]["heading"]
            delta = abs(_normalize_angle(curr_heading - prev_heading))
            if delta > self.heading_change_threshold:
                labels[i] = True

        # Rule 4: Goal distance crosses thresholds
        crossed = set()
        for i in range(n):
            d = steps[i]["distance_to_goal"]
            for thresh in self.goal_thresholds:
                if thresh not in crossed and d <= thresh:
                    crossed.add(thresh)
                    labels[i] = True

        # Rule 5: End of long straight segments
        straight_count = 0
        for i in range(1, n):
            prev_heading = steps[i - 1]["heading"]
            curr_heading = steps[i]["heading"]
            delta = abs(_normalize_angle(curr_heading - prev_heading))
            if delta < 0.1:  # Nearly straight
                straight_count += 1
            else:
                if straight_count >= self.straight_segment_length:
                    labels[i] = True  # End of straight segment
                straight_count = 0

        # Ensure target ratio isn't too far off
        think_count = sum(labels)
        current_ratio = think_count / n if n > 0 else 0

        # If too few THINK labels, add some at regular intervals
        if current_ratio < self.target_think_ratio * 0.5:
            interval = max(1, int(1.0 / self.target_think_ratio))
            for i in range(0, n, interval):
                if not labels[i]:
                    labels[i] = True

        return labels


# ---------------------------------------------------------------------------
# CoT Text Generator (via VLM API)
# ---------------------------------------------------------------------------
class CoTTextGenerator:
    """
    Generate structured Chain-of-Thought reasoning text for THINK steps.

    Uses an external VLM API (GPT-4o / Qwen) to produce reasoning.
    For steps labelled NO_THINK, an empty string is returned.

    The CoT follows VLingNav's format:
      [Observation] ...
      [Memory]      ...
      [Instruction] ...
      [Reasoning]   ...
    """

    COT_SYSTEM_PROMPT = (
        "You are an embodied navigation agent reasoning about your next move. "
        "Given the current visual observation, your goal instruction, and your "
        "navigation memory, provide a concise chain-of-thought reasoning about "
        "what direction to go and why. Keep it to 2-3 sentences."
    )

    COT_USER_TEMPLATE = (
        "[Observation] I am at position ({x:.1f}, {y:.1f}), facing {heading_dir}. "
        "Distance to goal: {dist:.1f}m.\n"
        "[Instruction] {instruction}\n"
        "[Step] {step_idx}/{total_steps} in this trajectory.\n"
        "[Reasoning] "
    )

    def __init__(
        self,
        api_backend: str = "openai",
        api_key: Optional[str] = None,
        model: str = "gpt-4o-mini",
        max_tokens: int = 100,
    ):
        self.api_backend = api_backend
        self.api_key = api_key or os.getenv("OPENAI_API_KEY", "")
        self.model = model
        self.max_tokens = max_tokens

    def generate(
        self,
        step: dict,
        instruction: str,
        total_steps: int,
    ) -> str:
        """Generate CoT reasoning text for a single THINK step."""
        pos = step["position"]
        heading_dir = _heading_to_cardinal(step["heading"])

        prompt = self.COT_USER_TEMPLATE.format(
            x=pos[0], y=pos[1] if len(pos) > 1 else pos[0],
            heading_dir=heading_dir,
            dist=step["distance_to_goal"],
            instruction=instruction,
            step_idx=step["step_idx"],
            total_steps=total_steps,
        )

        if self.api_backend == "openai":
            return self._call_openai(prompt)
        elif self.api_backend == "qwen":
            return self._call_qwen(prompt)
        else:
            return self._fallback_cot(step, instruction)

    def _call_openai(self, prompt: str) -> str:
        try:
            import openai
            client = openai.OpenAI(api_key=self.api_key)
            response = client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.COT_SYSTEM_PROMPT},
                    {"role": "user", "content": prompt},
                ],
                max_tokens=self.max_tokens,
                temperature=0.3,
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            logger.warning("OpenAI API call failed: %s", e)
            return ""

    def _call_qwen(self, prompt: str) -> str:
        try:
            import dashscope
            from dashscope import Generation
            response = Generation.call(
                model="qwen-turbo",
                messages=[
                    {"role": "system", "content": self.COT_SYSTEM_PROMPT},
                    {"role": "user", "content": prompt},
                ],
                max_tokens=self.max_tokens,
                temperature=0.3,
            )
            return response.output.text.strip()
        except Exception as e:
            logger.warning("Qwen API call failed: %s", e)
            return ""

    @staticmethod
    def _fallback_cot(step: dict, instruction: str) -> str:
        """Template-based fallback when API is unavailable."""
        dist = step["distance_to_goal"]
        heading = _heading_to_cardinal(step["heading"])
        if dist > 5.0:
            return f"I'm still far from the goal ({dist:.1f}m). Moving {heading} to explore."
        elif dist > 2.0:
            return f"Getting closer ({dist:.1f}m). Continuing {heading} toward the target."
        else:
            return f"Very close to the goal ({dist:.1f}m). Approaching carefully."


# ---------------------------------------------------------------------------
# Full annotation pipeline
# ---------------------------------------------------------------------------
class AdaCoTAnnotator:
    """
    Annotate collected trajectories with THINK/NO_THINK labels and CoT text.

    Pipeline:
      1. Load trajectory JSON
      2. Apply rule-based THINK triggers
      3. Generate CoT text for THINK steps (via VLM API or template)
      4. Save annotated trajectory

    Usage:
        annotator = AdaCoTAnnotator(data_dir="data/trajectories")
        annotator.annotate_all()
    """

    def __init__(
        self,
        data_dir: str = "data/trajectories",
        api_backend: str = "openai",
        api_key: Optional[str] = None,
        use_vlm_cot: bool = True,
        heading_change_threshold: float = 0.52,
        target_think_ratio: float = 0.30,
    ):
        self.data_dir = Path(data_dir)
        self.trigger = RuleBasedTrigger(
            heading_change_threshold=heading_change_threshold,
            target_think_ratio=target_think_ratio,
        )
        self.use_vlm_cot = use_vlm_cot
        self.cot_gen = CoTTextGenerator(
            api_backend=api_backend,
            api_key=api_key,
        ) if use_vlm_cot else None

    def annotate_episode(self, episode_dir: Path) -> Optional[dict]:
        """Annotate a single episode's trajectory."""
        traj_path = episode_dir / "trajectory.json"
        if not traj_path.exists():
            return None

        with open(traj_path) as f:
            traj = json.load(f)

        steps = traj["steps"]
        instruction = traj["instruction"]

        # Step 1: Rule-based THINK labels
        think_labels = self.trigger.annotate(steps)

        # Step 2: Generate CoT text for THINK steps
        for i, (step, should_think) in enumerate(zip(steps, think_labels)):
            step["adacot_label"] = "THINK" if should_think else "NO_THINK"
            if should_think and self.cot_gen is not None:
                step["cot_text"] = self.cot_gen.generate(
                    step, instruction, len(steps)
                )
            elif should_think:
                step["cot_text"] = CoTTextGenerator._fallback_cot(step, instruction)
            else:
                step["cot_text"] = ""

        # Save annotated version
        traj["steps"] = steps
        traj["annotation_stats"] = {
            "total_steps": len(steps),
            "think_steps": sum(think_labels),
            "think_ratio": sum(think_labels) / max(len(think_labels), 1),
        }

        annotated_path = episode_dir / "trajectory_annotated.json"
        with open(annotated_path, "w") as f:
            json.dump(traj, f, indent=2)

        return traj["annotation_stats"]

    def annotate_all(self) -> dict:
        """Annotate all episodes in the data directory."""
        episode_dirs = sorted([
            d for d in self.data_dir.iterdir()
            if d.is_dir() and (d / "trajectory.json").exists()
        ])

        logger.info("Annotating %d episodes in %s", len(episode_dirs), self.data_dir)

        stats = {"total": 0, "think_steps": 0, "total_steps": 0}

        for i, ep_dir in enumerate(episode_dirs):
            result = self.annotate_episode(ep_dir)
            if result:
                stats["total"] += 1
                stats["think_steps"] += result["think_steps"]
                stats["total_steps"] += result["total_steps"]

            if (i + 1) % 1000 == 0:
                ratio = stats["think_steps"] / max(stats["total_steps"], 1)
                logger.info(
                    "Annotated %d/%d episodes (think ratio: %.1f%%)",
                    i + 1, len(episode_dirs), ratio * 100,
                )

        stats["think_ratio"] = stats["think_steps"] / max(stats["total_steps"], 1)
        logger.info(
            "Annotation complete: %d episodes, %d steps, %.1f%% THINK",
            stats["total"], stats["total_steps"], stats["think_ratio"] * 100,
        )

        # Save summary
        with open(self.data_dir / "annotation_summary.json", "w") as f:
            json.dump(stats, f, indent=2)

        return stats


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


_CARDINAL = [
    (0, "N"), (45, "NE"), (90, "E"), (135, "SE"),
    (180, "S"), (225, "SW"), (270, "W"), (315, "NW"), (360, "N"),
]


def _heading_to_cardinal(heading_rad: float) -> str:
    deg = (math.degrees(heading_rad) % 360 + 360) % 360
    closest = min(_CARDINAL, key=lambda x: abs(x[0] - deg))
    return closest[1]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Annotate trajectories with AdaCoT labels")
    parser.add_argument("--data-dir", type=str, default="data/trajectories")
    parser.add_argument("--api-backend", type=str, default="openai",
                        choices=["openai", "qwen", "none"])
    parser.add_argument("--no-vlm", action="store_true",
                        help="Use template-based CoT instead of VLM API")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    annotator = AdaCoTAnnotator(
        data_dir=args.data_dir,
        api_backend=args.api_backend,
        use_vlm_cot=not args.no_vlm,
    )
    annotator.annotate_all()
