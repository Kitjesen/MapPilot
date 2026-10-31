"""Depth-camera point-token student policy for Thunder.

This module is deliberately independent from the paused PPO runner.  The
sensor adapter must convert depth pixels to base-frame XYZ points, append a
normalized frame-age channel, and provide a validity mask.  Keeping that
conversion outside the policy makes camera calibration and augmentation
testable without changing the policy graph.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import torch
from torch import Tensor, nn


@dataclass
class DepthStudentConfig:
    point_dim: int = 4  # x, y, z, normalized frame age
    point_width: int = 64
    state_width: int = 128
    fusion_heads: int = 8
    action_dim: int = 16
    history_dim: int = 54
    command_dim: int = 3
    history_steps: int = 1
    max_points_per_frame: int = 80
    frames: int = 5


class DepthPointEncoder(nn.Module):
    """Encode masked point tokens and retain per-token features for fusion."""

    def __init__(self, cfg: DepthStudentConfig) -> None:
        super().__init__()
        self.cfg = cfg
        self.lift = nn.Sequential(
            nn.Linear(cfg.point_dim, cfg.point_width),
            nn.LayerNorm(cfg.point_width),
            nn.SiLU(),
            nn.Linear(cfg.point_width, cfg.point_width),
            nn.LayerNorm(cfg.point_width),
            nn.SiLU(),
        )
        self.self_attention = nn.MultiheadAttention(
            cfg.point_width,
            cfg.fusion_heads,
            batch_first=True,
        )
        self.norm = nn.LayerNorm(cfg.point_width)

    def forward(self, points: Tensor, valid: Tensor) -> Tuple[Tensor, Tensor]:
        if points.ndim != 4:
            raise ValueError("points must have shape [B, frames, K, point_dim]")
        if valid.shape != points.shape[:3]:
            raise ValueError("valid must have shape [B, frames, K]")
        if points.shape[-1] != self.cfg.point_dim:
            raise ValueError(f"expected point_dim={self.cfg.point_dim}")

        batch = points.shape[0]
        tokens = self.lift(points).flatten(1, 2)
        mask = ~valid.flatten(1, 2).bool()
        attended, _ = self.self_attention(
            tokens,
            tokens,
            tokens,
            key_padding_mask=mask,
            need_weights=False,
        )
        tokens = self.norm(tokens + attended)
        weights = (~mask).unsqueeze(-1).to(tokens.dtype)
        pooled = (tokens * weights).sum(dim=1) / weights.sum(dim=1).clamp_min(1.0)
        # A fully invalid frame is legal during camera dropout.  Returning a
        # zero token keeps the policy finite and lets training learn recovery.
        pooled = torch.where((weights.sum(dim=1) > 0), pooled, torch.zeros_like(pooled))
        return tokens, pooled


class DepthCameraStudentPolicy(nn.Module):
    """UniPoint-style depth student with proprioception-queried cross-attention.

    Inputs:
      points: [B, 5, 80, 4] base-frame XYZ + frame age.
      valid: [B, 5, 80] boolean token validity mask.
      history: [B, 54] or [B, history_steps, 54] Thunder student history.
      command: [B, 3] velocity/yaw command.

    The deployment action is the Gaussian mean.  ``log_std`` is kept as a
    parameter for PPO and distillation wrappers that need a distribution.
    """

    def __init__(self, cfg: Optional[DepthStudentConfig] = None) -> None:
        super().__init__()
        self.cfg = cfg or DepthStudentConfig()
        self.point_encoder = DepthPointEncoder(self.cfg)
        state_input = self.cfg.history_dim * self.cfg.history_steps + self.cfg.command_dim
        self.state_encoder = nn.Sequential(
            nn.Linear(state_input, self.cfg.state_width),
            nn.LayerNorm(self.cfg.state_width),
            nn.SiLU(),
            nn.Linear(self.cfg.state_width, self.cfg.state_width),
            nn.LayerNorm(self.cfg.state_width),
            nn.SiLU(),
        )
        self.query = nn.Linear(self.cfg.state_width, self.cfg.point_width)
        self.cross_attention = nn.MultiheadAttention(
            self.cfg.point_width,
            self.cfg.fusion_heads,
            batch_first=True,
        )
        self.policy = nn.Sequential(
            nn.Linear(self.cfg.state_width + self.cfg.point_width, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 256),
            nn.SiLU(),
        )
        self.action_mean = nn.Linear(256, self.cfg.action_dim)
        self.log_std = nn.Parameter(torch.full((self.cfg.action_dim,), -1.0))

    def _encode_state(self, history: Tensor, command: Tensor) -> Tensor:
        if history.ndim == 2:
            history = history.unsqueeze(1)
        if history.ndim != 3:
            raise ValueError("history must have shape [B, 54] or [B, steps, 54]")
        if history.shape[1:] != (self.cfg.history_steps, self.cfg.history_dim):
            raise ValueError(
                f"expected history shape [B, {self.cfg.history_steps}, {self.cfg.history_dim}]"
            )
        if command.ndim != 2 or command.shape[-1] != self.cfg.command_dim:
            raise ValueError(f"command must have shape [B, {self.cfg.command_dim}]")
        return self.state_encoder(torch.cat((history.flatten(1), command), dim=-1))

    def forward(
        self,
        points: Tensor,
        valid: Tensor,
        history: Tensor,
        command: Tensor,
    ) -> Tuple[Tensor, Tensor]:
        tokens, _ = self.point_encoder(points, valid)
        state = self._encode_state(history, command)
        query = self.query(state).unsqueeze(1)
        point_mask = ~valid.flatten(1, 2).bool()
        fused_point, _ = self.cross_attention(
            query,
            tokens,
            tokens,
            key_padding_mask=point_mask,
            need_weights=False,
        )
        fused = self.policy(torch.cat((state, fused_point.squeeze(1)), dim=-1))
        return self.action_mean(fused), self.log_std.expand(points.shape[0], -1)

    @torch.no_grad()
    def act_inference(
        self,
        points: Tensor,
        valid: Tensor,
        history: Tensor,
        command: Tensor,
    ) -> Tensor:
        return self(points, valid, history, command)[0]
