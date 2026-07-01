"""ReconBackendBase — interface every reconstruction backend must implement.

A backend receives an ordered sequence of Keyframe objects (each with an
RGB image, depth image, camera intrinsics, and camera-to-world pose) and
produces a reconstruction artifact (PLY point cloud, mesh, NeRF checkpoint,
or Gaussian splat checkpoint) saved to `output_dir`.

The server calls backends asynchronously in a subprocess / thread pool,
so each backend must be safe to run in its own process.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class Keyframe:
    """One RGB-D keyframe with full pose and intrinsics.

    Attributes
    ----------
    frame_idx : int
        Monotonic index within the session.
    timestamp : float
        Unix epoch seconds.
    pose : np.ndarray  (4, 4) float64
        Camera-to-world homogeneous transform.
    fx, fy, cx, cy : float
        Pinhole intrinsics in pixels.
    width, height : int
        Image dimensions.
    color_path : Path
        Path to JPEG colour image on disk.
    depth_path : Path
        Path to 16-bit PNG depth image on disk (values in mm).
    """

    frame_idx: int = 0
    timestamp: float = 0.0
    pose: Any = None        # np.ndarray (4,4); 'Any' to avoid numpy import here
    fx: float = 615.0
    fy: float = 615.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    color_path: Path = field(default_factory=lambda: Path("/dev/null"))
    depth_path: Path = field(default_factory=lambda: Path("/dev/null"))


@dataclass
class ReconResult:
    """Result from a reconstruction job.

    Attributes
    ----------
    success : bool
    output_path : Path | None
        Path to the primary output artifact (PLY, mesh, checkpoint, etc.)
    output_format : str
        "ply", "mesh_obj", "nerf_checkpoint", "gaussian_ckpt", …
    num_frames : int
        Number of frames actually used.
    elapsed_sec : float
        Wall-clock time taken.
    message : str
        Human-readable summary or error message.
    extra : dict
        Backend-specific metrics (PSNR, voxel count, etc.)
    """

    success: bool = False
    output_path: Path | None = None
    output_format: str = "ply"
    num_frames: int = 0
    elapsed_sec: float = 0.0
    message: str = ""
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "output_path": str(self.output_path) if self.output_path else None,
            "output_format": self.output_format,
            "num_frames": self.num_frames,
            "elapsed_sec": self.elapsed_sec,
            "message": self.message,
            "extra": self.extra,
        }


class ReconBackendBase(abc.ABC):
    """Abstract base class for all reconstruction backends.

    Subclasses must implement `reconstruct()`.  The server instantiates
    one backend instance per job and calls `reconstruct()` synchronously
    inside a thread/process pool worker.
    """

    name: str = "base"

    @abc.abstractmethod
    def reconstruct(
        self,
        keyframes: list[Keyframe],
        output_dir: Path,
        **options: Any,
    ) -> ReconResult:
        """Run reconstruction on the given keyframes.

        Parameters
        ----------
        keyframes : list[Keyframe]
            Ordered list of RGB-D keyframes with poses.
        output_dir : Path
            Directory where outputs should be written.  Created by caller.
        **options
            Backend-specific keyword arguments (passed from HTTP request).

        Returns
        -------
        ReconResult
        """
        ...

    def check_dependencies(self) -> tuple[bool, str]:
        """Return (available, reason) — checks if required libraries are installed."""
        return True, "ok"
