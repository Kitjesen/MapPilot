"""Nerfstudio backend — NeRF or 3D Gaussian Splatting via nerfstudio CLI.

Converts robot keyframes into a nerfstudio-compatible `transforms.json`
dataset, then launches `ns-train <method>` as a subprocess.

Supported methods (passed as option `method`):
    instant-ngp      — hash-encoded NeRF, ~5 min on RTX 3060, good quality
    nerfacto         — nerfstudio default, best quality, ~15-30 min
    splatfacto       — 3D Gaussian Splatting in nerfstudio, ~10 min
    depth-nerfacto   — depth-supervised NeRF (uses our depth images)

Output: nerfstudio checkpoint directory + exported PLY/splat.

Requirements:
    pip install nerfstudio          # see https://docs.nerf.studio/quickstart/installation.html
    ns-train --help                 # verify installation

GitHub: nerfstudio-project/nerfstudio (Apache 2.0)
Note: GPU with CUDA ≥11.7 strongly recommended (RTX 3060+).
"""

from __future__ import annotations

import json
import logging
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from runtime.msgs.numpy_compat import np

from .base import Keyframe, ReconBackendBase, ReconResult
from .registry import register_backend

logger = logging.getLogger(__name__)


@register_backend("nerfstudio")
class NerfstudioBackend(ReconBackendBase):
    """NeRF / 3DGS reconstruction via nerfstudio CLI.

    Options:
        method       str   "instant-ngp"  — nerfstudio trainer method name
        max_num_iterations int 5000        — training iterations
        export_ply   bool  True            — export point cloud after training
        camera_model str   "OPENCV"        — nerfstudio camera model
    """

    name = "nerfstudio"

    def check_dependencies(self) -> tuple[bool, str]:
        if shutil.which("ns-train") is None:
            return False, (
                "nerfstudio not found — install: pip install nerfstudio\n"
                "See: https://docs.nerf.studio/quickstart/installation.html"
            )
        return True, "nerfstudio available"

    def reconstruct(
        self,
        keyframes: list[Keyframe],
        output_dir: Path,
        **options: Any,
    ) -> ReconResult:
        t0 = time.time()

        ok, reason = self.check_dependencies()
        if not ok:
            return ReconResult(success=False, message=reason)

        method       = str(options.get("method", "instant-ngp"))
        max_iters    = int(options.get("max_num_iterations", 5000))
        export_ply   = bool(options.get("export_ply", True))
        camera_model = str(options.get("camera_model", "OPENCV"))

        output_dir.mkdir(parents=True, exist_ok=True)
        data_dir = output_dir / "nerfstudio_data"
        data_dir.mkdir(exist_ok=True)
        images_dir = data_dir / "images"
        images_dir.mkdir(exist_ok=True)
        depths_dir = data_dir / "depths"
        depths_dir.mkdir(exist_ok=True)

        # ── Build transforms.json ───────────────────────────────────────
        # Copy images + depths and build nerfstudio dataset format
        frames_json = []
        for kf in keyframes:
            dst_color = images_dir / f"frame_{kf.frame_idx:06d}.jpg"
            dst_depth = depths_dir / f"depth_{kf.frame_idx:06d}.png"

            shutil.copy2(kf.color_path, dst_color)
            shutil.copy2(kf.depth_path, dst_depth)

            # nerfstudio uses OpenCV convention: transform_matrix is camera-to-world
            # but in nerfstudio's coordinate system (Y-down, Z-forward converted to
            # Y-up, Z-backward OpenGL).  We apply the convention flip.
            T_cv  = kf.pose  # camera-to-world, OpenCV (x right, y down, z forward)
            T_gl  = _opencv_to_opengl(T_cv)

            frames_json.append({
                "file_path": f"images/frame_{kf.frame_idx:06d}.jpg",
                "depth_file_path": f"depths/depth_{kf.frame_idx:06d}.png",
                "transform_matrix": T_gl.tolist(),
            })

        if not frames_json:
            return ReconResult(success=False, message="No frames available",
                               elapsed_sec=time.time() - t0)

        kf0 = keyframes[0]
        transforms = {
            "camera_model": camera_model,
            "fl_x": kf0.fx,
            "fl_y": kf0.fy,
            "cx":   kf0.cx,
            "cy":   kf0.cy,
            "w":    kf0.width,
            "h":    kf0.height,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
            "depth_unit_scale_factor": 0.001,   # mm → m
            "frames": frames_json,
        }
        transforms_path = data_dir / "transforms.json"
        with open(transforms_path, "w", encoding="utf-8") as f:
            json.dump(transforms, f, indent=2)

        # ── Run ns-train ────────────────────────────────────────────────
        train_output = output_dir / "ns_output"
        cmd = [
            "ns-train", method,
            "--data", str(data_dir),
            "--output-dir", str(train_output),
            "--max-num-iterations", str(max_iters),
            "--pipeline.model.predict-normals", "True",
        ]
        if "depth" in method:
            cmd += ["--pipeline.datamanager.depth-unit-scale-factor", "0.001"]

        logger.info("nerfstudio: running %s", " ".join(cmd))
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=7200,
            )
            if result.returncode != 0:
                return ReconResult(
                    success=False,
                    message=f"ns-train failed (rc={result.returncode}): {result.stderr[-500:]}",
                    elapsed_sec=time.time() - t0,
                )
        except subprocess.TimeoutExpired:
            return ReconResult(success=False,
                               message="ns-train timed out (2h limit)",
                               elapsed_sec=time.time() - t0)

        # ── Export PLY ──────────────────────────────────────────────────
        output_path = train_output
        output_format = "nerf_checkpoint"

        if export_ply:
            ply_path = output_dir / "reconstruction.ply"
            export_cmd = [
                "ns-export", "pointcloud",
                "--load-config", str(_find_config(train_output)),
                "--output-dir", str(output_dir),
                "--num-points", "1000000",
            ]
            try:
                subprocess.run(
                    export_cmd,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=300,
                )
                if ply_path.exists():
                    output_path   = ply_path
                    output_format = "ply"
            except Exception:
                logger.warning("nerfstudio: PLY export failed, returning checkpoint")

        elapsed = time.time() - t0
        logger.info("nerfstudio (%s): %d frames in %.1fs", method, len(keyframes), elapsed)

        return ReconResult(
            success=True,
            output_path=output_path,
            output_format=output_format,
            num_frames=len(keyframes),
            elapsed_sec=elapsed,
            message=f"nerfstudio {method} complete: {len(keyframes)} frames",
        )


# ── Coordinate-frame helpers ────────────────────────────────────────────────

def _opencv_to_opengl(T_cv: np.ndarray) -> np.ndarray:
    """Convert camera-to-world from OpenCV to OpenGL/nerfstudio convention.

    OpenCV: x-right, y-down, z-forward
    OpenGL: x-right, y-up,   z-backward

    The flip matrix is: diag(1, -1, -1, 1)
    """
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    return T_cv @ flip


def _find_config(train_output: Path) -> Path:
    """Find the nerfstudio config.yml inside the output directory."""
    configs = sorted(train_output.rglob("config.yml"))
    if configs:
        return configs[-1]
    return train_output / "config.yml"
