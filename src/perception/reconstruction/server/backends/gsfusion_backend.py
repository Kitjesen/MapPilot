"""GSFusion backend — Online Gaussian Splatting + TSDF fusion.

GSFusion (ETH Zurich) combines TSDF volumetric fusion with 3D Gaussian
primitives for real-time RGB-D mapping with novel-view rendering quality.

GitHub: https://github.com/ethz-mrl/GSFusion (MIT License)
Paper:  GSFusion: Online RGB-D Mapping Where Gaussian Splatting Meets TSDF Fusion
        IEEE RA-L 2024 — https://arxiv.org/abs/2408.12677

Requirements:
    - CUDA GPU (NVIDIA RTX 3060+ recommended)
    - GSFusion built from source:
        git clone https://github.com/ethz-mrl/GSFusion.git
        cd GSFusion && mkdir build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8
    - Set env var GSFUSION_BIN=/path/to/GSFusion/build/GSFusion

The backend writes keyframes to a temporary TUM-format dataset directory,
then runs GSFusion as a subprocess.

Output: Point cloud (.ply) extracted from the Gaussian splat model.

If GSFusion is not installed, this backend falls back gracefully to the
TSDF backend with a warning.
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from runtime.msgs.numpy_compat import np

from .base import Keyframe, ReconBackendBase, ReconResult
from .registry import register_backend

logger = logging.getLogger(__name__)

_GSFUSION_BIN_ENV = "GSFUSION_BIN"


@register_backend("gsfusion")
class GSFusionBackend(ReconBackendBase):
    """GSFusion online Gaussian Splatting + TSDF reconstruction.

    Writes a TUM-format RGB-D dataset, runs GSFusion, and exports the
    resulting Gaussian model as a PLY point cloud.

    Options:
        voxel_size   float  0.05   — TSDF voxel size (m)
        sdf_trunc    float  0.10   — TSDF truncation (m)
        max_depth_m  float  5.0
        gsfusion_bin str    env(GSFUSION_BIN) — path to GSFusion executable
    """

    name = "gsfusion"

    def check_dependencies(self) -> tuple[bool, str]:
        bin_path = os.environ.get(_GSFUSION_BIN_ENV, "")
        if not bin_path or not Path(bin_path).exists():
            return False, (
                f"GSFusion binary not found. "
                f"Set {_GSFUSION_BIN_ENV}=/path/to/GSFusion/build/GSFusion\n"
                f"Build from source: https://github.com/ethz-mrl/GSFusion"
            )
        return True, f"GSFusion at {bin_path}"

    def reconstruct(
        self,
        keyframes: list[Keyframe],
        output_dir: Path,
        **options: Any,
    ) -> ReconResult:
        t0 = time.time()

        gsfusion_bin = str(options.get(
            "gsfusion_bin", os.environ.get(_GSFUSION_BIN_ENV, "")
        ))

        ok, reason = self.check_dependencies()
        if not ok:
            logger.warning("GSFusion unavailable (%s), falling back to TSDF", reason)
            from .tsdf_backend import TSDFBackend
            return TSDFBackend().reconstruct(keyframes, output_dir, **options)

        voxel_size  = float(options.get("voxel_size", 0.05))
        sdf_trunc   = float(options.get("sdf_trunc", voxel_size * 2))
        max_depth   = float(options.get("max_depth_m", 5.0))

        output_dir.mkdir(parents=True, exist_ok=True)

        # ── Write TUM-format dataset ─────────────────────────────────────
        # TUM format: rgb/ depth/ associations.txt camera_info.txt
        dataset_dir = output_dir / "gsfusion_dataset"
        dataset_dir.mkdir(exist_ok=True)
        (dataset_dir / "rgb").mkdir(exist_ok=True)
        (dataset_dir / "depth").mkdir(exist_ok=True)

        assoc_lines = []
        for kf in keyframes:
            ts = f"{kf.timestamp:.6f}"
            rgb_dst   = dataset_dir / "rgb"   / f"{ts}.jpg"
            depth_dst = dataset_dir / "depth" / f"{ts}.png"
            shutil.copy2(kf.color_path, rgb_dst)
            shutil.copy2(kf.depth_path, depth_dst)
            assoc_lines.append(f"{ts} rgb/{ts}.jpg {ts} depth/{ts}.png")

        (dataset_dir / "associations.txt").write_text("\n".join(assoc_lines))

        # Camera info for GSFusion
        kf0 = keyframes[0]
        cam_info = {
            "fx": kf0.fx, "fy": kf0.fy,
            "cx": kf0.cx, "cy": kf0.cy,
            "width": kf0.width, "height": kf0.height,
            "depth_scale": 1000.0,   # mm → m conversion factor
        }
        (dataset_dir / "camera_info.json").write_text(json.dumps(cam_info, indent=2))

        # Write pose file (custom format: ts tx ty tz qx qy qz qw)
        pose_lines = []
        for kf in keyframes:
            # Extract translation + quaternion from 4×4 pose
            R = kf.pose[:3, :3]
            t = kf.pose[:3, 3]
            q = _rotation_matrix_to_quaternion(R)
            pose_lines.append(
                f"{kf.timestamp:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
            )
        (dataset_dir / "poses.txt").write_text("\n".join(pose_lines))

        # ── Run GSFusion ─────────────────────────────────────────────────
        gs_output = output_dir / "gs_output"
        gs_output.mkdir(exist_ok=True)

        cmd = [
            gsfusion_bin,
            "--dataset", str(dataset_dir),
            "--output", str(gs_output),
            "--voxel_size", str(voxel_size),
            "--sdf_trunc", str(sdf_trunc),
            "--max_depth", str(max_depth),
        ]
        logger.info("GSFusion: running %s", " ".join(cmd))

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=3600,
            )
            if result.returncode != 0:
                logger.warning("GSFusion stderr: %s", result.stderr[-1000:])
                return ReconResult(
                    success=False,
                    message=f"GSFusion failed (rc={result.returncode})",
                    elapsed_sec=time.time() - t0,
                )
        except subprocess.TimeoutExpired:
            return ReconResult(success=False, message="GSFusion timed out (1h limit)",
                               elapsed_sec=time.time() - t0)

        # Find output PLY
        plys = sorted(gs_output.rglob("*.ply"))
        output_path = plys[-1] if plys else gs_output

        elapsed = time.time() - t0
        logger.info("GSFusion: %d frames in %.1fs → %s", len(keyframes), elapsed, output_path)

        return ReconResult(
            success=True,
            output_path=output_path,
            output_format="ply",
            num_frames=len(keyframes),
            elapsed_sec=elapsed,
            message=f"GSFusion complete: {len(keyframes)} frames in {elapsed:.1f}s",
        )


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple:
    """Convert 3×3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)
