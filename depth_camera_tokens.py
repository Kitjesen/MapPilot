"""Calibration-aware depth image to UniPoint-style point tokens.

The adapter intentionally owns only geometry.  Camera intrinsics/extrinsics
must come from the robot calibration, and callers should pass one calibration
per camera.  It returns fixed-size tokens suitable for
``DepthCameraStudentPolicy``.
"""

from typing import Tuple

import torch
from torch import Tensor, nn


class DepthCameraTokeniser(nn.Module):
    def __init__(
        self,
        image_height: int,
        image_width: int,
        voxel_size: float = 0.1,
        max_points_per_frame: int = 80,
        min_depth: float = 0.15,
        max_depth: float = 5.0,
    ) -> None:
        super().__init__()
        self.voxel_size = voxel_size
        self.max_points_per_frame = max_points_per_frame
        self.min_depth = min_depth
        self.max_depth = max_depth
        v, u = torch.meshgrid(
            torch.arange(image_height, dtype=torch.float32),
            torch.arange(image_width, dtype=torch.float32),
            indexing="ij",
        )
        self.register_buffer("u", u.reshape(-1), persistent=False)
        self.register_buffer("v", v.reshape(-1), persistent=False)

    @torch.no_grad()
    def forward(
        self,
        depth: Tensor,
        intrinsics: Tensor,
        camera_to_base: Tensor,
        frame_age: float = 0.0,
    ) -> Tuple[Tensor, Tensor]:
        """Return ``points [B, 1, K, 4]`` and ``valid [B, 1, K]``.

        Depth is in metres.  ``camera_to_base`` is a homogeneous transform
        from the camera optical frame to the robot base frame.  Invalid or
        out-of-range pixels are removed before voxel aggregation.
        """
        if depth.ndim != 3:
            raise ValueError("depth must have shape [B, H, W]")
        if intrinsics.shape != (depth.shape[0], 3, 3):
            raise ValueError("intrinsics must have shape [B, 3, 3]")
        if camera_to_base.shape != (depth.shape[0], 4, 4):
            raise ValueError("camera_to_base must have shape [B, 4, 4]")
        b, h, w = depth.shape
        if self.u.numel() != h * w:
            raise ValueError("depth resolution differs from tokeniser calibration")

        d = depth.reshape(b, -1)
        fx, fy = intrinsics[:, 0, 0:1], intrinsics[:, 1, 1:2]
        cx, cy = intrinsics[:, 0, 2:3], intrinsics[:, 1, 2:3]
        u = self.u.to(depth).unsqueeze(0)
        v = self.v.to(depth).unsqueeze(0)
        valid = torch.isfinite(d) & (d >= self.min_depth) & (d <= self.max_depth)
        x = (u - cx) * d / fx.clamp_min(1e-6)
        y = (v - cy) * d / fy.clamp_min(1e-6)
        cam = torch.stack((x, y, d, torch.ones_like(d)), dim=-1)
        base = torch.bmm(cam, camera_to_base.transpose(1, 2))[..., :3]
        tokens = depth.new_zeros((b, self.max_points_per_frame, 4))
        mask = torch.zeros((b, self.max_points_per_frame), dtype=torch.bool, device=depth.device)

        # Voxel means are computed per batch element.  This path is intended
        # for the low-rate observation adapter, not the control hot loop.
        for i in range(b):
            xyz = base[i][valid[i]]
            if xyz.numel() == 0:
                continue
            voxel = torch.floor(xyz / self.voxel_size).to(torch.int64)
            unique, inverse = torch.unique(voxel, dim=0, return_inverse=True)
            sums = torch.zeros((unique.shape[0], 3), device=depth.device, dtype=xyz.dtype)
            sums.index_add_(0, inverse, xyz)
            counts = torch.bincount(inverse, minlength=unique.shape[0]).to(xyz.dtype).unsqueeze(1)
            xyz = sums / counts.clamp_min(1.0)
            # Stable deterministic token selection keeps rollout/replay aligned.
            order = torch.argsort(xyz[:, 0] * 10000 + xyz[:, 1] * 100 + xyz[:, 2])
            xyz = xyz[order][: self.max_points_per_frame]
            n = xyz.shape[0]
            tokens[i, :n, :3] = xyz
            tokens[i, :n, 3] = frame_age
            mask[i, :n] = True
        return tokens.unsqueeze(1), mask.unsqueeze(1)
