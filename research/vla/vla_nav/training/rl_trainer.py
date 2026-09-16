"""
RL Post-Training — Online Expert-Guided Reinforcement Learning.

Reference: VLingNav (arXiv 2601.08665) Section 3.5
  - After SFT, use online RL to surpass pure imitation learning
  - Agent interacts with Habitat simulator to collect on-policy trajectories
  - Hybrid objective: outcome-driven optimization + expert-guided supervision
  - Prevents catastrophic forgetting of the expert policy

Also references:
  - AdaCoT (Pareto-optimal): PPO for adaptive CoT triggering
  - CompassNav (ICLR 2026): Gap-Aware reward function
  - GRPO (Group Relative Policy Optimization) from DeepSeek-R1

Reward function:
  R = R_success + R_progress + R_efficiency + R_cot_quality

  R_success:    +10 if reach goal within threshold, else 0
  R_progress:   Δ(distance_to_goal) — closer = positive reward
  R_efficiency: -0.01 per step (encourage shorter paths)
  R_cot_quality: Penalize unnecessary THINK steps (waste compute)
"""

import json
import logging
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Reward computation
# ---------------------------------------------------------------------------
class NavigationReward:
    """
    Compute shaped reward for VLA navigation.

    Reward components:
      1. Success reward:    Large positive for reaching the goal
      2. Progress reward:   Positive for moving closer to goal
      3. Efficiency penalty: Small negative per step
      4. CoT quality:       Penalise unnecessary thinking
      5. Collision penalty:  Negative for hitting obstacles
    """

    def __init__(
        self,
        success_reward: float = 10.0,
        progress_scale: float = 2.0,
        step_penalty: float = -0.01,
        unnecessary_think_penalty: float = -0.1,
        collision_penalty: float = -0.5,
        success_distance: float = 1.0,
    ):
        self.success_reward = success_reward
        self.progress_scale = progress_scale
        self.step_penalty = step_penalty
        self.unnecessary_think_penalty = unnecessary_think_penalty
        self.collision_penalty = collision_penalty
        self.success_distance = success_distance

    def compute(
        self,
        prev_distance: float,
        curr_distance: float,
        did_think: bool,
        is_terminal: bool,
        collided: bool,
        step_idx: int,
    ) -> Tuple[float, Dict[str, float]]:
        """
        Compute reward for a single step.

        Returns:
            (total_reward, reward_breakdown_dict)
        """
        components = {}

        # Success
        if is_terminal and curr_distance <= self.success_distance:
            components["success"] = self.success_reward
        else:
            components["success"] = 0.0

        # Progress (positive when getting closer)
        progress = prev_distance - curr_distance
        components["progress"] = progress * self.progress_scale

        # Step penalty
        components["efficiency"] = self.step_penalty

        # CoT quality: penalise THINK when progress is good (unnecessary)
        if did_think and progress > 0.3:
            components["cot_quality"] = self.unnecessary_think_penalty
        else:
            components["cot_quality"] = 0.0

        # Collision
        if collided:
            components["collision"] = self.collision_penalty
        else:
            components["collision"] = 0.0

        total = sum(components.values())
        return total, components


# ---------------------------------------------------------------------------
# Experience buffer
# ---------------------------------------------------------------------------
class RolloutBuffer:
    """Buffer for storing on-policy rollout data."""

    def __init__(self):
        self.observations: List[dict] = []
        self.actions: List[torch.Tensor] = []
        self.rewards: List[float] = []
        self.values: List[float] = []
        self.log_probs: List[float] = []
        self.dones: List[bool] = []
        self.adacot_labels: List[int] = []
        self.hidden_states: List[torch.Tensor] = []

    def add(
        self,
        observation: dict,
        action: torch.Tensor,
        reward: float,
        value: float,
        log_prob: float,
        done: bool,
        adacot_label: int,
        hidden_state: torch.Tensor,
    ):
        self.observations.append(observation)
        self.actions.append(action)
        self.rewards.append(reward)
        self.values.append(value)
        self.log_probs.append(log_prob)
        self.dones.append(done)
        self.adacot_labels.append(adacot_label)
        self.hidden_states.append(hidden_state)

    def compute_returns(self, gamma: float = 0.99, lam: float = 0.95) -> Tuple[list, list]:
        """Compute GAE (Generalized Advantage Estimation) returns and advantages."""
        n = len(self.rewards)
        returns = [0.0] * n
        advantages = [0.0] * n

        last_gae = 0.0
        last_value = 0.0

        for t in reversed(range(n)):
            if self.dones[t]:
                next_value = 0.0
                last_gae = 0.0
            else:
                next_value = self.values[t + 1] if t + 1 < n else last_value

            delta = self.rewards[t] + gamma * next_value - self.values[t]
            last_gae = delta + gamma * lam * last_gae
            advantages[t] = last_gae
            returns[t] = advantages[t] + self.values[t]

        return returns, advantages

    def clear(self):
        self.observations.clear()
        self.actions.clear()
        self.rewards.clear()
        self.values.clear()
        self.log_probs.clear()
        self.dones.clear()
        self.adacot_labels.clear()
        self.hidden_states.clear()

    def __len__(self):
        return len(self.rewards)


# ---------------------------------------------------------------------------
# Value Head (critic)
# ---------------------------------------------------------------------------
class ValueHead(nn.Module):
    """Critic network for estimating state value."""

    def __init__(self, hidden_dim: int = 2048):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(hidden_dim, 256),
            nn.GELU(),
            nn.Linear(256, 64),
            nn.GELU(),
            nn.Linear(64, 1),
        )

    def forward(self, hidden_state: torch.Tensor) -> torch.Tensor:
        return self.mlp(hidden_state).squeeze(-1)


# ---------------------------------------------------------------------------
# PPO Trainer
# ---------------------------------------------------------------------------
class VLANavRLTrainer:
    """
    Online RL post-training for VLA Navigation.

    Uses PPO (Proximal Policy Optimization) to fine-tune:
      - AdaCoT trigger head (when to THINK)
      - Action head (waypoint quality)
      - LoRA parameters (navigation strategy)

    The VLM backbone is mostly frozen; only LoRA + heads receive gradients.

    Training loop:
      1. Collect rollout in Habitat (N steps)
      2. Compute rewards + GAE advantages
      3. PPO update (K epochs over the buffer)
      4. Repeat
    """

    def __init__(
        self,
        sft_checkpoint: str = "checkpoints/sft/best",
        output_dir: str = "checkpoints/rl",
        # Habitat
        scene_dataset: str = "data/scene_datasets/hm3d/hm3d_annotated_basis.scene_dataset_config.json",
        # PPO config
        rollout_steps: int = 256,
        ppo_epochs: int = 4,
        mini_batch_size: int = 32,
        clip_epsilon: float = 0.2,
        value_loss_coef: float = 0.5,
        entropy_coef: float = 0.01,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        learning_rate: float = 5e-5,
        max_grad_norm: float = 0.5,
        # Training
        total_timesteps: int = 100_000,
        eval_interval: int = 10_000,
        save_interval: int = 25_000,
        # Reward
        success_reward: float = 10.0,
        progress_scale: float = 2.0,
    ):
        self.sft_checkpoint = Path(sft_checkpoint)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.scene_dataset = scene_dataset
        self.rollout_steps = rollout_steps
        self.ppo_epochs = ppo_epochs
        self.mini_batch_size = mini_batch_size
        self.clip_epsilon = clip_epsilon
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.lr = learning_rate
        self.max_grad_norm = max_grad_norm
        self.total_timesteps = total_timesteps
        self.eval_interval = eval_interval
        self.save_interval = save_interval

        self.reward_fn = NavigationReward(
            success_reward=success_reward,
            progress_scale=progress_scale,
        )
        self.buffer = RolloutBuffer()

        # Will be initialized in setup()
        self._vla_model = None
        self._value_head = None
        self._optimizer = None
        self._sim = None

    def setup(self):
        """Load SFT checkpoint and initialize RL components."""
        logger.info("Setting up RL trainer from SFT checkpoint: %s", self.sft_checkpoint)

        from vla_nav.model.vla_model import VLANavModel

        # Load VLA model with SFT weights
        self._vla_model = VLANavModel(backbone_name="Qwen/Qwen2.5-VL-3B-Instruct")
        self._vla_model.load()
        self._vla_model.load_heads(str(self.sft_checkpoint))

        device = self._vla_model.backbone.device_name
        hidden_dim = self._vla_model.backbone.hidden_dim

        # Value head (critic) — new, trained from scratch during RL
        self._value_head = ValueHead(hidden_dim).to(device)

        # Optimizer: only heads + value head + LoRA
        self._optimizer = torch.optim.AdamW(
            list(self._vla_model.adacot.parameters())
            + list(self._vla_model.action_head.parameters())
            + list(self._value_head.parameters()),
            lr=self.lr,
        )

        logger.info("RL trainer setup complete")

    def collect_rollout(self) -> Dict[str, float]:
        """
        Collect one rollout of experience from Habitat.

        Returns:
            Episode statistics (reward, success, steps, etc.)
        """
        self.buffer.clear()
        self._vla_model.eval()

        # Placeholder for Habitat interaction
        # In production, this creates a new episode in the simulator
        # and runs the VLA model to collect steps

        episode_stats = {
            "total_reward": 0.0,
            "num_steps": 0,
            "success": False,
        }

        prev_distance = 10.0  # Starting distance to goal

        for step in range(self.rollout_steps):
            # Get VLA model output (in production: from actual Habitat obs)
            device = self._vla_model.backbone.device_name
            hidden_dim = self._vla_model.backbone.hidden_dim

            # Simulated hidden state (in production: from backbone.encode())
            hidden_state = torch.randn(1, hidden_dim, device=device)

            # AdaCoT decision
            trigger_prob = self._vla_model.adacot.trigger_head.predict_prob(hidden_state)
            did_think = trigger_prob > self._vla_model.adacot.trigger_threshold

            # Action prediction
            waypoints = self._vla_model.action_head(hidden_state.float())
            action = waypoints[:, 0, :]  # First waypoint

            # Value estimate
            value = self._value_head(hidden_state.float()).item()

            # Compute log probability (simplified)
            log_prob = -torch.abs(action).sum().item()

            # Simulated environment step
            curr_distance = max(0.0, prev_distance - 0.1 + np.random.randn() * 0.05)
            is_terminal = curr_distance < 1.0 or step == self.rollout_steps - 1

            # Reward
            reward, _ = self.reward_fn.compute(
                prev_distance=prev_distance,
                curr_distance=curr_distance,
                did_think=did_think,
                is_terminal=is_terminal,
                collided=False,
                step_idx=step,
            )

            # Store
            self.buffer.add(
                observation={},
                action=action.detach(),
                reward=reward,
                value=value,
                log_prob=log_prob,
                done=is_terminal,
                adacot_label=1 if did_think else 0,
                hidden_state=hidden_state.detach(),
            )

            episode_stats["total_reward"] += reward
            episode_stats["num_steps"] += 1
            prev_distance = curr_distance

            if is_terminal:
                episode_stats["success"] = curr_distance < 1.0
                break

        return episode_stats

    def ppo_update(self) -> Dict[str, float]:
        """Run PPO update on the collected rollout buffer."""
        self._vla_model.train()
        self._value_head.train()

        returns, advantages = self.buffer.compute_returns(
            self.gamma, self.gae_lambda,
        )

        returns_t = torch.tensor(returns, dtype=torch.float32)
        advantages_t = torch.tensor(advantages, dtype=torch.float32)
        # Normalize advantages
        advantages_t = (advantages_t - advantages_t.mean()) / (advantages_t.std() + 1e-8)

        old_log_probs = torch.tensor(self.buffer.log_probs, dtype=torch.float32)

        n = len(self.buffer)
        update_stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}

        for _ in range(self.ppo_epochs):
            indices = np.random.permutation(n)

            for start in range(0, n, self.mini_batch_size):
                end = min(start + self.mini_batch_size, n)
                mb_idx = indices[start:end]

                # Gather mini-batch
                mb_hidden = torch.stack([self.buffer.hidden_states[i] for i in mb_idx])
                mb_returns = returns_t[mb_idx].to(mb_hidden.device)
                mb_advantages = advantages_t[mb_idx].to(mb_hidden.device)
                mb_old_lp = old_log_probs[mb_idx].to(mb_hidden.device)

                # Forward pass
                mb_hidden_f = mb_hidden.squeeze(1).float()
                new_waypoints = self._vla_model.action_head(mb_hidden_f)
                new_values = self._value_head(mb_hidden_f)

                # Simplified log prob (in production: proper distribution)
                new_log_probs = -torch.abs(new_waypoints[:, 0, :]).sum(dim=-1)

                # PPO clipped loss
                ratio = torch.exp(new_log_probs - mb_old_lp)
                surr1 = ratio * mb_advantages
                surr2 = torch.clamp(ratio, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * mb_advantages
                policy_loss = -torch.min(surr1, surr2).mean()

                # Value loss
                value_loss = F.mse_loss(new_values, mb_returns)

                # Total loss
                loss = (
                    policy_loss
                    + self.value_loss_coef * value_loss
                )

                self._optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    list(self._vla_model.action_head.parameters())
                    + list(self._value_head.parameters()),
                    self.max_grad_norm,
                )
                self._optimizer.step()

                update_stats["policy_loss"] += policy_loss.item()
                update_stats["value_loss"] += value_loss.item()

        n_updates = max(1, self.ppo_epochs * (n // self.mini_batch_size))
        for k in update_stats:
            update_stats[k] /= n_updates

        return update_stats

    def train(self):
        """Run the full RL training loop."""
        if self._vla_model is None:
            self.setup()

        logger.info("Starting RL training for %d timesteps", self.total_timesteps)

        total_steps = 0
        iteration = 0
        best_reward = -float("inf")

        while total_steps < self.total_timesteps:
            iteration += 1

            # Collect rollout
            episode_stats = self.collect_rollout()
            total_steps += episode_stats["num_steps"]

            # PPO update
            update_stats = self.ppo_update()

            # Logging
            logger.info(
                "Iter %d | steps=%d/%d | reward=%.2f | success=%s | "
                "policy_loss=%.4f | value_loss=%.4f",
                iteration, total_steps, self.total_timesteps,
                episode_stats["total_reward"],
                episode_stats["success"],
                update_stats["policy_loss"],
                update_stats["value_loss"],
            )

            # Save
            if total_steps % self.save_interval < self.rollout_steps:
                self._save(total_steps)

            if episode_stats["total_reward"] > best_reward:
                best_reward = episode_stats["total_reward"]
                self._save(total_steps, is_best=True)

        self._save(total_steps, is_final=True)
        logger.info("RL training complete. Best reward: %.2f", best_reward)

    def _save(self, step: int, is_best: bool = False, is_final: bool = False):
        if is_best:
            save_dir = self.output_dir / "best"
        elif is_final:
            save_dir = self.output_dir / "final"
        else:
            save_dir = self.output_dir / f"step_{step}"

        save_dir.mkdir(parents=True, exist_ok=True)
        self._vla_model.save_heads(str(save_dir))
        torch.save(self._value_head.state_dict(), save_dir / "value_head.pt")

        meta = {"step": step, "is_best": is_best}
        with open(save_dir / "rl_meta.json", "w") as f:
            json.dump(meta, f, indent=2)

        logger.info("RL checkpoint saved to %s", save_dir)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RL post-training for VLA navigation")
    parser.add_argument("--sft-checkpoint", type=str, default="checkpoints/sft/best")
    parser.add_argument("--output-dir", type=str, default="checkpoints/rl")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--rollout-steps", type=int, default=256)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    trainer = VLANavRLTrainer(
        sft_checkpoint=args.sft_checkpoint,
        output_dir=args.output_dir,
        total_timesteps=args.total_timesteps,
        rollout_steps=args.rollout_steps,
    )
    trainer.setup()
    trainer.train()
