"""TSDF volumetric fusion backend — uses Open3D ScalableTSDFVolume.

This is the lightest backend: pure CPU, no CUDA required, runs on any server.
Output: coloured point cloud (.ply) and optionally a mesh (.obj via marching cubes).

Requirements:
    pip install open3d

GitHub reference: isl-org/Open3D (Apache 2.0)
Docs: http://www.open3d.org/docs/release/tutorial/t_reconstruction_system/integration.html
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

from runtime.msgs.numpy_compat import np

from .base import Keyframe, ReconBackendBase, ReconResult
from .registry import register_backend

logger = logging.getLogger(__name__)


@register_backend("tsdf")
class TSDFBackend(ReconBackendBase):
    """TSDF volumetric fusion using Open3D ScalableTSDFVolume.

    Options (passed as keyword args to reconstruct):
        voxel_length  float  0.04  — voxel size in metres
        sdf_trunc     float  0.08  — TSDF truncation distance
        extract_mesh  bool   False — also extract triangle mesh (slow)
        min_depth_m   float  0.1   — skip depth values below this (m)
        max_depth_m   float  5.0   — skip depth values above this (m)
    """

    name = "tsdf"

    def check_dependencies(self) -> tuple[bool, str]:
        try:
            import open3d  # noqa
            return True, "open3d available"
        except ImportError:
            return False, "open3d not installed — run: pip install open3d"

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

        import open3d as o3d

        voxel_length = float(options.get("voxel_length", 0.04))
        sdf_trunc    = float(options.get("sdf_trunc", voxel_length * 2))
        extract_mesh = bool(options.get("extract_mesh", False))
        float(options.get("min_depth_m", 0.1))
        max_depth    = float(options.get("max_depth_m", 5.0))

        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=voxel_length,
            sdf_trunc=sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        )

        used = 0
        for kf in keyframes:
            try:
                color_o3d = o3d.io.read_image(str(kf.color_path))
                depth_o3d = o3d.io.read_image(str(kf.depth_path))
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_o3d, depth_o3d,
                    depth_scale=1000.0,     # depth PNG is in mm
                    depth_trunc=max_depth,
                    convert_rgb_to_intensity=False,
                )
                intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    width=kf.width, height=kf.height,
                    fx=kf.fx, fy=kf.fy, cx=kf.cx, cy=kf.cy,
                )
                # Open3D integrate expects extrinsic = world-to-camera (inverse of pose)
                extrinsic = np.linalg.inv(kf.pose)
                volume.integrate(rgbd, intrinsic, extrinsic)
                used += 1
            except Exception:
                logger.warning("TSDF: skipping frame %d", kf.frame_idx, exc_info=True)

        if used == 0:
            return ReconResult(success=False, message="No frames integrated",
                               elapsed_sec=time.time() - t0)

        # Extract point cloud
        pcd = volume.extract_point_cloud()
        output_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = output_dir / "reconstruction.ply"
        o3d.io.write_point_cloud(str(pcd_path), pcd)

        result_path = pcd_path
        output_format = "ply"

        if extract_mesh:
            try:
                mesh = volume.extract_triangle_mesh()
                mesh.compute_vertex_normals()
                mesh_path = output_dir / "mesh.obj"
                o3d.io.write_triangle_mesh(str(mesh_path), mesh)
                result_path = mesh_path
                output_format = "mesh_obj"
            except Exception:
                logger.warning("TSDF: mesh extraction failed", exc_info=True)

        elapsed = time.time() - t0
        n_pts = len(np.asarray(pcd.points))
        logger.info("TSDF: %d frames → %d points in %.1fs", used, n_pts, elapsed)

        return ReconResult(
            success=True,
            output_path=result_path,
            output_format=output_format,
            num_frames=used,
            elapsed_sec=elapsed,
            message=f"TSDF fusion complete: {n_pts} points from {used} frames",
            extra={"n_points": n_pts, "voxel_length": voxel_length},
        )
