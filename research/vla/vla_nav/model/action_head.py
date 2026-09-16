"""
Action Head — MLP Waypoint Prediction.

Reference: VLingNav (arXiv 2601.08665) Section 3.2, 3.3
  - The action model conditions on the VLM backbone's hidden states to predict
    a motion trajectory τ = {a_1, a_2, ..., a_n}
  - Each a ∈ R^3 = (x, y, θ) denotes a waypoint with position and orientation
  - n is the trajectory horizon (default 5)

Design choices:
  - MLP with residual connections for stable training
  - tanh activation for bounded output:
      x, y ∈ [-max_linear_step, +max_linear_step]  (default ±0.5 m)
      θ   ∈ [-max_angular_step, +max_angular_step]  (default ±0.785 rad ≈ 45°)
  - The first waypoint is executed; the rest provide a look-ahead trajectory
    that can be published as a nav_msgs/Path for visualization
"""

import math
import logging
from typing import Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F

logger = logging.getLogger(__name__)


class ResidualBlock(nn.Module):
    """Pre-norm residual MLP block."""

    def __init__(self, dim: int, dropout: float = 0.1):
        super().__init__()
        self.norm = nn.LayerNorm(dim)
        self.mlp = nn.Sequential(
            nn.Linear(dim, dim * 2),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(dim * 2, dim),
            nn.Dropout(dropout),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + self.mlp(self.norm(x))


class ActionHead(nn.Module):
    """
    MLP action head for continuous waypoint prediction.

    Architecture:
        hidden_state (hidden_dim)
          → Linear(hidden_dim, 512)
          → ResidualBlock(512)
          → ResidualBlock(512)
          → Linear(512, action_dim * horizon)
          → reshape to (batch, horizon, action_dim)
          → tanh scaling

    The residual blocks improve gradient flow during RL post-training
    where the action head receives gradients from the reward signal.
    """

    def __init__(
        self,
        hidden_dim: int = 2048,
        action_dim: int = 3,
        horizon: int = 5,
        intermediate_dim: int = 512,
        max_linear_step: float = 0.5,
        max_angular_step: float = 0.785,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.action_dim = action_dim
        self.horizon = horizon
        self.max_linear_step = max_linear_step
        self.max_angular_step = max_angular_step

        self.input_proj = nn.Sequential(
            nn.Linear(hidden_dim, intermediate_dim),
            nn.GELU(),
            nn.Dropout(dropout),
        )
        self.blocks = nn.Sequential(
            ResidualBlock(intermediate_dim, dropout),
            ResidualBlock(intermediate_dim, dropout),
        )
        self.output_proj = nn.Linear(intermediate_dim, action_dim * horizon)

        self._init_weights()

    def _init_weights(self):
        """Small initialization for the output layer → start with near-zero actions."""
        nn.init.uniform_(self.output_proj.weight, -0.01, 0.01)
        nn.init.zeros_(self.output_proj.bias)

    def forward(
        self,
        hidden_state: torch.Tensor,
        return_raw: bool = False,
    ) -> torch.Tensor:
        """
        Predict a trajectory of waypoints.

        Args:
            hidden_state: (batch, hidden_dim) from VLM backbone
            return_raw:   If True, return pre-tanh logits (for training)

        Returns:
            waypoints: (batch, horizon, action_dim)
                       Each waypoint = (delta_x, delta_y, delta_theta)
                       in the robot's local frame.
        """
        x = self.input_proj(hidden_state)   # (B, intermediate_dim)
        x = self.blocks(x)                  # (B, intermediate_dim)
        raw = self.output_proj(x)           # (B, action_dim * horizon)

        if return_raw:
            return raw.view(-1, self.horizon, self.action_dim)

        # Bounded output via tanh
        waypoints = raw.view(-1, self.horizon, self.action_dim)
        # x, y displacement
        waypoints[..., :2] = torch.tanh(waypoints[..., :2]) * self.max_linear_step
        # theta rotation
        waypoints[..., 2] = torch.tanh(waypoints[..., 2]) * self.max_angular_step

        return waypoints

    def compute_loss(
        self,
        predicted: torch.Tensor,
        target: torch.Tensor,
        mask: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """
        Compute L1 loss between predicted and target waypoints.

        Args:
            predicted: (batch, horizon, action_dim) — model output
            target:    (batch, horizon, action_dim) — expert waypoints
            mask:      (batch, horizon) — optional mask for variable-length trajectories

        Returns:
            Scalar loss
        """
        diff = torch.abs(predicted - target)

        if mask is not None:
            mask = mask.unsqueeze(-1).expand_as(diff)
            diff = diff * mask
            loss = diff.sum() / mask.sum().clamp(min=1.0)
        else:
            loss = diff.mean()

        return loss

    def get_first_waypoint(self, waypoints: torch.Tensor) -> torch.Tensor:
        """Extract the first waypoint (the one to execute now)."""
        return waypoints[:, 0, :]  # (batch, action_dim)

    def waypoints_to_world(
        self,
        waypoints: torch.Tensor,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
    ) -> list:
        """
        Convert local-frame waypoint deltas to world-frame coordinates.

        Args:
            waypoints:   (1, horizon, 3) — (dx, dy, dtheta) in robot frame
            robot_x/y:   Current world position
            robot_theta: Current world heading (radians)

        Returns:
            List of (world_x, world_y, world_theta) tuples
        """
        wp = waypoints.squeeze(0).cpu().numpy()  # (horizon, 3)
        world_points = []
        cx, cy, ct = robot_x, robot_y, robot_theta

        for i in range(wp.shape[0]):
            dx, dy, dtheta = wp[i]
            # Rotate local delta to world frame
            wx = cx + dx * math.cos(ct) - dy * math.sin(ct)
            wy = cy + dx * math.sin(ct) + dy * math.cos(ct)
            wt = ct + dtheta
            world_points.append((wx, wy, wt))
            cx, cy, ct = wx, wy, wt

        return world_points
