"""Startup calibration self-check — validates sensor calibration before system start.

Called automatically by full_stack_blueprint() when native SLAM or camera is enabled.
Returns structured results so callers can block startup on FAIL-level issues.

Usage::

    from core.utils.calibration_check import run_calibration_check

    errors, warnings = run_calibration_check()
    if errors:
        raise RuntimeError(f"Calibration check failed: {errors}")
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
FASTLIO2_CONFIG = REPO_ROOT / "src" / "slam" / "fastlio2" / "config" / "lio.yaml"
POINTLIO_CONFIG = REPO_ROOT / "config" / "pointlio.yaml"

# Plausible LiDAR↔IMU clock offset range (seconds). Hardware-synchronised systems
# typically calibrate to <10 ms; values beyond this likely indicate a parsing bug
# or hardware sync failure.
TIME_OFFSET_MAX_ABS_S = 0.1


@dataclass
class CalibrationReport:
    """Structured calibration check results."""
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    info: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return len(self.errors) == 0

    def summary(self) -> str:
        parts = []
        if self.errors:
            parts.append(f"{len(self.errors)} ERROR(s)")
        if self.warnings:
            parts.append(f"{len(self.warnings)} warning(s)")
        if not parts:
            return "All calibration checks passed"
        return "Calibration: " + ", ".join(parts)


def run_calibration_check(
    config=None,
    require_camera: bool = False,
    require_slam: bool = False,
) -> CalibrationReport:
    """Run all calibration sanity checks against robot_config.yaml.

    Parameters
    ----------
    config : RobotConfig, optional
        If None, loads via get_config().
    require_camera : bool
        If True, camera extrinsics all-zero is an ERROR (not warning).
    require_slam : bool
        If True, LiDAR-IMU extrinsic mismatch is an ERROR.

    Returns
    -------
    CalibrationReport with errors, warnings, info lists.
    """
    if config is None:
        from core.config import get_config
        config = get_config()

    report = CalibrationReport()

    _check_camera_intrinsics(config, report, require_camera)
    _check_camera_extrinsics(config, report, require_camera)
    _check_lidar_extrinsics(config, report)
    _check_depth_scale(config, report)
    _check_lidar_imu_consistency(config, report, require_slam)
    _check_rotation_validity(report)
    _check_time_offset(report, require_slam)

    # Log results
    for msg in report.errors:
        logger.error("CALIB FAIL: %s", msg)
    for msg in report.warnings:
        logger.warning("CALIB WARN: %s", msg)
    for msg in report.info:
        logger.info("CALIB OK: %s", msg)

    return report


def _check_camera_intrinsics(config, report: CalibrationReport, required: bool) -> None:
    """Validate camera focal length and principal point."""
    cam = getattr(config, "camera", None)
    if cam is None:
        msg = "Camera calibration config missing"
        if required:
            report.errors.append(msg)
        else:
            report.warnings.append(msg)
        return
    fx, fy = cam.fx, cam.fy

    if fx <= 0 or fy <= 0:
        report.errors.append(
            f"Camera focal length non-positive: fx={fx}, fy={fy}"
        )
        return

    # Aspect ratio sanity: fx and fy should be within 10% of each other
    if abs(fx - fy) / max(fx, fy) > 0.15:
        report.warnings.append(
            f"Camera fx ({fx:.1f}) and fy ({fy:.1f}) differ by >{15}%"
        )

    w, h = cam.width, cam.height
    if w <= 0 or h <= 0:
        report.errors.append(f"Camera image size invalid: {w}x{h}")
        return

    # Principal point should be within 20% of image center
    cx_off = abs(cam.cx - w / 2) / w
    cy_off = abs(cam.cy - h / 2) / h
    if cx_off > 0.2 or cy_off > 0.2:
        report.warnings.append(
            f"Principal point ({cam.cx:.1f}, {cam.cy:.1f}) far from center "
            f"({w/2:.0f}, {h/2:.0f}) — may need intrinsic calibration"
        )

    # Focal length plausibility (for typical 60-90 deg FOV cameras)
    diag = math.hypot(w, h)
    fov_deg = 2 * math.degrees(math.atan2(diag / 2, fx))
    if fov_deg < 20 or fov_deg > 170:
        report.warnings.append(
            f"Implied FOV {fov_deg:.0f} deg seems unusual (expected 40-120)"
        )
    else:
        report.info.append(f"Camera intrinsics: {w}x{h}, fx={fx:.1f}, FOV~{fov_deg:.0f} deg")


def _check_camera_extrinsics(config, report: CalibrationReport, required: bool) -> None:
    """Validate camera mounting position in body frame."""
    cam = getattr(config, "camera", None)
    if cam is None:
        return
    pos = (cam.position_x, cam.position_y, cam.position_z)

    if all(v == 0.0 for v in pos):
        msg = "Camera extrinsics position is all zeros — 3D projections will be wrong"
        if required:
            report.errors.append(msg)
        else:
            report.warnings.append(msg)
        return

    mag = math.sqrt(sum(v**2 for v in pos))
    if mag > 3.0:
        report.warnings.append(
            f"Camera position magnitude {mag:.2f}m seems too large for quadruped"
        )
    else:
        report.info.append(
            f"Camera extrinsics: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m"
        )

    # Rotation sanity — identity rotation means camera Z = body Z (up)
    # For a forward-facing camera, we expect a ~90 deg rotation
    rot = (cam.roll, cam.pitch, cam.yaw)
    rot_mag = math.sqrt(sum(r**2 for r in rot))
    if rot_mag < 0.01 and not all(v == 0 for v in pos):
        report.warnings.append(
            "Camera rotation is identity — camera Z-axis aligns with body Z (up), "
            "not forward. A forward-facing camera needs pitch or yaw rotation. "
            "Run: python calibration/camera_lidar/calibrate.sh"
        )
    elif math.degrees(rot_mag) > 90:
        report.warnings.append(
            f"Camera rotation {math.degrees(rot_mag):.1f} deg is very large"
        )


def _check_lidar_extrinsics(config, report: CalibrationReport) -> None:
    """Validate LiDAR mounting offset."""
    lidar = getattr(config, "lidar", None)
    if lidar is None:
        report.warnings.append("LiDAR calibration config missing")
        return
    ox, oy, oz = lidar.offset_x, lidar.offset_y, lidar.offset_z
    mag = math.sqrt(ox**2 + oy**2 + oz**2)

    if mag < 0.001:
        report.warnings.append(
            "LiDAR offset near zero — may not be calibrated"
        )
    elif mag > 0.5:
        report.warnings.append(
            f"LiDAR offset {mag:.4f}m seems large for body-mounted sensor"
        )
    else:
        report.info.append(
            f"LiDAR offset: ({ox:.5f}, {oy:.5f}, {oz:.5f}) m = {mag:.4f}m"
        )


def _check_depth_scale(config, report: CalibrationReport) -> None:
    """Validate depth_scale is set correctly."""
    cam = getattr(config, "camera", None)
    if cam is None:
        return
    ds = cam.depth_scale
    if ds <= 0:
        report.errors.append(f"depth_scale is non-positive: {ds}")
    elif ds > 1.0:
        report.warnings.append(
            f"depth_scale={ds} > 1.0 — raw depth values would be amplified"
        )
    elif ds == 1.0:
        report.warnings.append(
            "depth_scale=1.0 (identity) — confirm depth sensor outputs meters"
        )
    else:
        report.info.append(f"depth_scale: {ds} (raw → meters)")


def _check_lidar_imu_consistency(
    config, report: CalibrationReport, required: bool
) -> None:
    """Cross-check LiDAR offsets between robot_config.yaml and SLAM configs."""
    if not FASTLIO2_CONFIG.exists():
        return

    try:
        import yaml
        with open(FASTLIO2_CONFIG, encoding="utf-8") as f:
            lio = yaml.safe_load(f) or {}
    except Exception:
        return

    t_il = lio.get("t_il")
    if not t_il:
        return

    lidar = getattr(config, "lidar", None)
    if lidar is None:
        msg = "LiDAR calibration config missing; cannot compare against lio.yaml"
        if required:
            report.errors.append(msg)
        else:
            report.warnings.append(msg)
        return
    t_cfg = [lidar.offset_x, lidar.offset_y, lidar.offset_z]

    if np.allclose(t_il, t_cfg, atol=0.002):
        report.info.append("LiDAR offsets match robot_config.yaml and lio.yaml")
    else:
        msg = (
            f"LiDAR offset mismatch: robot_config={t_cfg} vs lio.yaml={t_il}. "
            "Run: python calibration/apply_calibration.py to sync."
        )
        if required:
            report.errors.append(msg)
        else:
            report.warnings.append(msg)

    # Check IMU noise parameters are sensible
    na = lio.get("na", 0)
    ng = lio.get("ng", 0)
    if na <= 0 or ng <= 0:
        report.errors.append(f"SLAM IMU noise non-positive: na={na}, ng={ng}")
    elif na > 0.5 or ng > 0.5:
        report.warnings.append(
            f"High IMU noise: na={na}, ng={ng} — consider Allan variance calibration"
        )


def _extract_pointlio_time_offset() -> float | None:
    """Read time_diff_lidar_to_imu from pointlio.yaml.

    Supports both ROS2 parameter file layout (`/** -> ros__parameters -> common`)
    and flat-key layout. Returns None if missing/unreadable.
    """
    if not POINTLIO_CONFIG.exists():
        return None
    try:
        import yaml
        with open(POINTLIO_CONFIG, encoding="utf-8") as f:
            pl = yaml.safe_load(f) or {}
    except Exception:
        return None

    # Search common locations in priority order.
    for node in (
        pl,
        pl.get("/**", {}).get("ros__parameters", {}).get("common", {})
            if isinstance(pl.get("/**"), dict) else {},
        pl.get("common", {}) if isinstance(pl.get("common"), dict) else {},
        pl.get("ros__parameters", {}).get("common", {})
            if isinstance(pl.get("ros__parameters"), dict) else {},
    ):
        if isinstance(node, dict) and "time_diff_lidar_to_imu" in node:
            try:
                return float(node["time_diff_lidar_to_imu"])
            except (TypeError, ValueError):
                return None
    return None


def _check_time_offset(report: CalibrationReport, required: bool) -> None:
    """Validate LiDAR↔IMU time offset is within physical range and consistent
    between fastlio2 and pointlio configs."""
    import yaml

    lio_offset = None
    if FASTLIO2_CONFIG.exists():
        try:
            with open(FASTLIO2_CONFIG, encoding="utf-8") as f:
                lio = yaml.safe_load(f) or {}
            lio_offset = lio.get("time_diff_lidar_to_imu")
        except Exception:
            pass

    pointlio_offset = _extract_pointlio_time_offset()

    for name, val in (("lio.yaml", lio_offset), ("pointlio.yaml", pointlio_offset)):
        if val is None:
            continue
        if abs(val) > TIME_OFFSET_MAX_ABS_S:
            msg = (
                f"{name} time_diff_lidar_to_imu = {val:.6f}s exceeds plausible "
                f"±{TIME_OFFSET_MAX_ABS_S}s range — calibration likely wrong"
            )
            if required:
                report.errors.append(msg)
            else:
                report.warnings.append(msg)

    if lio_offset is not None and pointlio_offset is not None:
        if abs(lio_offset - pointlio_offset) > 1e-4:
            msg = (
                f"time_diff_lidar_to_imu mismatch: lio.yaml={lio_offset:.6f}s "
                f"vs pointlio.yaml={pointlio_offset:.6f}s. "
                "Run: python calibration/apply_calibration.py to sync."
            )
            if required:
                report.errors.append(msg)
            else:
                report.warnings.append(msg)
        else:
            report.info.append(
                f"time_diff_lidar_to_imu consistent across configs ({lio_offset:.6f}s)"
            )


def _check_rotation_validity(report: CalibrationReport) -> None:
    """Validate rotation matrices in SLAM config are proper rotations."""
    if not FASTLIO2_CONFIG.exists():
        return

    try:
        import yaml
        with open(FASTLIO2_CONFIG, encoding="utf-8") as f:
            lio = yaml.safe_load(f) or {}
    except Exception:
        return

    r_il = lio.get("r_il")
    if not r_il:
        return

    R = np.array(r_il, dtype=np.float64).reshape(3, 3)
    det = np.linalg.det(R)

    if abs(det - 1.0) > 0.01:
        report.errors.append(
            f"r_il is not a valid rotation: det(R) = {det:.6f} (expected 1.0)"
        )
    elif abs(det - 1.0) > 0.001:
        report.warnings.append(
            f"r_il determinant slightly off: {det:.6f}"
        )

    # Check orthogonality: R^T @ R should be close to I
    RtR = R.T @ R
    ortho_err = np.max(np.abs(RtR - np.eye(3)))
    if ortho_err > 0.01:
        report.errors.append(
            f"r_il not orthogonal: max |R^T R - I| = {ortho_err:.6f}"
        )
