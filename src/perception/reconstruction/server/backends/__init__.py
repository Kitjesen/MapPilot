"""Pluggable reconstruction backend registry.

Each backend implements ReconBackendBase and is registered under a name
that can be selected via the server's --backend argument.

Available backends:
    tsdf         — TSDF volumetric fusion using Open3D (CPU/GPU)
    open3d       — Open3D full reconstruction pipeline (RGBD odometry + TSDF)
    nerfstudio   — NeRF-based reconstruction via nerfstudio CLI
    gsfusion     — GSFusion online Gaussian Splatting + TSDF (requires CUDA)
    gaussian     — Offline 3D Gaussian Splatting via gsplat/nerfstudio
"""

from .base import ReconBackendBase
from .registry import BackendRegistry, get_backend, list_backends, register_backend

__all__ = [
    "ReconBackendBase",
    "BackendRegistry",
    "get_backend",
    "list_backends",
    "register_backend",
]
