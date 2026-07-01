"""LingTu reconstruction server package.

Provides:
    ReconServer  — FastAPI application that receives keyframe batches
                   from the robot and manages reconstruction jobs.

Quick start:
    python -m perception.reconstruction.server.recon_server --backend tsdf
    python -m perception.reconstruction.server.recon_server --backend open3d
    python -m perception.reconstruction.server.recon_server --backend nerfstudio
"""

from .recon_server import ReconServer, create_app

__all__ = ["ReconServer", "create_app"]
