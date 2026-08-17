#!/usr/bin/env python3
"""One-click calibration verification.

Checks all sensor calibration parameters in robot_config.yaml and SLAM configs
for sanity. Reports pass/warn/fail for each sensor.

Usage:
    python tools/calibration/verify.py
    python tools/calibration/verify.py --verbose
    python tools/calibration/verify.py --config /path/to/robot_config.yaml
"""

import argparse
import logging
import sys
from pathlib import Path

import numpy as np
import yaml

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_CONFIG = REPO_ROOT / "config" / "robot_config.yaml"
FASTLIO2_CONFIG = REPO_ROOT / "src" / "localization" / "fastlio2" / "config" / "mid360_s100p.yaml"
POINTLIO_CONFIG = REPO_ROOT / "config" / "pointlio.yaml"

# ANSI colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"
BOLD = "\033[1m"


class CalibrationConfigError(ValueError):
    """A calibration YAML file could not be decoded or parsed."""


def load_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except UnicodeDecodeError as exc:
        raise CalibrationConfigError(f"Failed to read YAML config {path}: file is not valid UTF-8") from exc
    except yaml.YAMLError as exc:
        detail = getattr(exc, "problem", None) or str(exc).splitlines()[0]
        raise CalibrationConfigError(f"Failed to parse YAML config {path}: {detail}") from exc


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the URDF fixed-axis ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` rotation."""

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


class CheckResult:
    def __init__(self):
        self.passed = 0
        self.warned = 0
        self.failed = 0

    def ok(self, msg: str) -> None:
        self.passed += 1
        logger.info(f"  {GREEN}PASS{RESET}  {msg}")

    def warn(self, msg: str) -> None:
        self.warned += 1
        logger.info(f"  {YELLOW}WARN{RESET}  {msg}")

    def fail(self, msg: str) -> None:
        self.failed += 1
        logger.info(f"  {RED}FAIL{RESET}  {msg}")

    @property
    def total(self) -> int:
        return self.passed + self.warned + self.failed

    @property
    def success(self) -> bool:
        return self.failed == 0


def check_camera(cfg: dict, result: CheckResult, verbose: bool) -> None:
    """Verify camera calibration parameters."""
    logger.info(f"\n{BOLD}== Camera =={RESET}")
    cam = cfg.get("camera", {})

    if not cam:
        result.fail("No camera section in robot_config.yaml")
        return

    # Intrinsics
    fx = cam.get("fx", 0)
    fy = cam.get("fy", 0)
    cx = cam.get("cx", 0)
    cy = cam.get("cy", 0)
    w = cam.get("width", 0)
    h = cam.get("height", 0)

    if fx <= 0 or fy <= 0:
        result.fail(f"Invalid focal length: fx={fx}, fy={fy}")
    elif abs(fx - fy) / max(fx, fy) > 0.1:
        result.warn(f"fx ({fx:.1f}) and fy ({fy:.1f}) differ by >10%")
    else:
        result.ok(f"Focal length: fx={fx:.1f}, fy={fy:.1f}")

    if w > 0 and h > 0:
        # Principal point should be near image center
        if abs(cx - w / 2) > w * 0.15 or abs(cy - h / 2) > h * 0.15:
            result.warn(f"Principal point ({cx:.1f}, {cy:.1f}) far from center ({w / 2}, {h / 2})")
        else:
            result.ok(f"Principal point: ({cx:.1f}, {cy:.1f}) in {w}x{h}")
    else:
        result.fail(f"Invalid image size: {w}x{h}")

    # Distortion
    dk1 = cam.get("dist_k1", 0)
    dk2 = cam.get("dist_k2", 0)
    if abs(dk1) > 2.0 or abs(dk2) > 2.0:
        result.warn(f"Large distortion coefficients: k1={dk1:.4f}, k2={dk2:.4f}")
    else:
        has_dist = any(abs(cam.get(k, 0)) > 1e-10 for k in ["dist_k1", "dist_k2", "dist_p1", "dist_p2", "dist_k3"])
        if has_dist:
            result.ok(f"Distortion calibrated: k1={dk1:.6f}, k2={dk2:.6f}")
        else:
            result.warn("No distortion coefficients (all zero) — may need calibration")

    # Extrinsics
    px = cam.get("position_x", 0)
    py = cam.get("position_y", 0)
    pz = cam.get("position_z", 0)

    if px == 0 and py == 0 and pz == 0:
        result.fail("Camera position is all zeros — extrinsics not calibrated")
    elif abs(px) > 2 or abs(py) > 2 or abs(pz) > 3:
        result.warn(f"Camera position ({px:.3f}, {py:.3f}, {pz:.3f}) seems too large")
    else:
        result.ok(f"Camera position: ({px:.3f}, {py:.3f}, {pz:.3f}) m")

    depth_scale = cam.get("depth_scale", 1.0)
    if depth_scale <= 0 or depth_scale > 1.0:
        result.warn(f"Unusual depth_scale: {depth_scale} (expected 0.001 for mm)")
    elif verbose:
        result.ok(f"Depth scale: {depth_scale}")


def check_lidar(cfg: dict, result: CheckResult, verbose: bool) -> None:
    """Verify LiDAR calibration parameters."""
    logger.info(f"\n{BOLD}== LiDAR =={RESET}")
    lidar = cfg.get("lidar", {})

    if not lidar:
        result.fail("No lidar section in robot_config.yaml")
        return

    ox = lidar.get("offset_x", 0)
    oy = lidar.get("offset_y", 0)
    oz = lidar.get("offset_z", 0)
    mag = (ox**2 + oy**2 + oz**2) ** 0.5

    if mag > 0.5:
        result.warn(f"LiDAR offset magnitude {mag:.4f}m seems large")
    elif mag < 0.001:
        result.warn(f"LiDAR offset near zero ({mag:.4f}m) — may not be calibrated")
    else:
        result.ok(f"LiDAR offset: ({ox:.5f}, {oy:.5f}, {oz:.5f}) m = {mag:.4f}m")

    roll = lidar.get("roll", 0)
    pitch = lidar.get("pitch", 0)
    yaw = lidar.get("yaw", 0)
    rot_mag = (roll**2 + pitch**2 + yaw**2) ** 0.5
    if np.degrees(rot_mag) > 30:
        result.warn(f"LiDAR rotation {np.degrees(rot_mag):.1f} deg seems large")
    elif verbose:
        result.ok(f"LiDAR rotation: [{roll:.4f}, {pitch:.4f}, {yaw:.4f}] rad")


def check_imu(result: CheckResult, verbose: bool) -> None:
    """Verify IMU noise parameters in SLAM configs."""
    logger.info(f"\n{BOLD}== IMU Noise (SLAM configs) =={RESET}")

    checked = False

    if FASTLIO2_CONFIG.exists():
        lio = load_yaml(FASTLIO2_CONFIG)
        na = lio.get("na", 0.01)
        ng = lio.get("ng", 0.01)
        nba = lio.get("nba", 0.0001)
        nbg = lio.get("nbg", 0.0001)

        # Typical ranges for MEMS IMU
        if na <= 0 or ng <= 0:
            result.fail(f"Non-positive noise: na={na}, ng={ng}")
        elif na > 0.1 or ng > 0.1:
            result.warn(f"High noise values: na={na}, ng={ng} — consider re-calibrating")
        else:
            result.ok(f"Fast-LIO2: na={na}, ng={ng}, nba={nba}, nbg={nbg}")

        # Check LiDAR-IMU extrinsics in SLAM config
        r_il = lio.get("r_il")
        t_il = lio.get("t_il")
        if t_il:
            result.ok(f"Fast-LIO2 t_il: {t_il}")
        if r_il and verbose:
            R = np.array(r_il).reshape(3, 3)
            det = np.linalg.det(R)
            if abs(det - 1.0) > 0.01:
                result.fail(f"r_il determinant = {det:.4f} (expected 1.0)")
            else:
                result.ok(f"Fast-LIO2 r_il determinant: {det:.6f}")

        # Gravity alignment
        gravity_align = lio.get("gravity_align", False)
        if gravity_align:
            result.ok("Gravity alignment: enabled")
        elif verbose:
            result.warn("Gravity alignment: disabled — IMU init may be inaccurate")

        checked = True

    if POINTLIO_CONFIG.exists():
        pio = load_yaml(POINTLIO_CONFIG)
        mapping = pio.get("/**", {}).get("ros__parameters", {}).get("mapping", {})
        acc_cov = mapping.get("imu_meas_acc_cov", 0.01)
        omg_cov = mapping.get("imu_meas_omg_cov", 0.01)
        if acc_cov > 0 and omg_cov > 0:
            result.ok(f"Point-LIO: acc_cov={acc_cov}, omg_cov={omg_cov}")
        checked = True

    if not checked:
        result.warn("No SLAM config files found — skipping IMU checks")


def check_consistency(cfg: dict, result: CheckResult) -> None:
    """Cross-check between different calibration parameters."""
    logger.info(f"\n{BOLD}== Cross-Checks =={RESET}")

    cam = cfg.get("camera", {})
    lidar = cfg.get("lidar", {})

    if cam and lidar:
        cam_z = cam.get("position_z", 0)
        lidar_z = lidar.get("offset_z", 0)
        # Camera should be above LiDAR typically
        if cam_z > 0 and lidar_z > 0:
            result.ok(f"Camera Z ({cam_z:.3f}m) and LiDAR Z ({lidar_z:.4f}m) both positive")
        elif cam_z == 0:
            result.warn("Camera Z is 0 — likely uncalibrated")

    # Check the composed SLAM transforms match the fixed mechanical mount:
    # T_body_lidar = T_body_imu * T_imu_lidar.
    if FASTLIO2_CONFIG.exists() and lidar:
        lio = load_yaml(FASTLIO2_CONFIG)
        required = (
            "r_il",
            "t_il",
            "navigation_body_from_imu_rotation",
            "navigation_body_from_imu_translation",
        )
        missing = [name for name in required if lio.get(name) is None]
        if missing:
            result.fail(f"Fast-LIO2 transform chain missing: {', '.join(missing)}")
            return

        try:
            r_il = np.asarray(lio["r_il"], dtype=np.float64).reshape(3, 3)
            t_il = np.asarray(lio["t_il"], dtype=np.float64).reshape(3)
            r_body_imu = np.asarray(
                lio["navigation_body_from_imu_rotation"],
                dtype=np.float64,
            ).reshape(3, 3)
            t_body_imu = np.asarray(
                lio["navigation_body_from_imu_translation"],
                dtype=np.float64,
            ).reshape(3)
        except (TypeError, ValueError) as exc:
            result.fail(f"Invalid Fast-LIO2 transform chain: {exc}")
            return

        r_composed = r_body_imu @ r_il
        t_composed = t_body_imu + r_body_imu @ t_il
        r_mount = _rotation_from_rpy(
            float(lidar.get("roll", 0.0)),
            float(lidar.get("pitch", 0.0)),
            float(lidar.get("yaw", 0.0)),
        )
        t_mount = np.asarray(
            [
                float(lidar.get("offset_x", 0.0)),
                float(lidar.get("offset_y", 0.0)),
                float(lidar.get("offset_z", 0.0)),
            ],
            dtype=np.float64,
        )
        translation_error = float(np.linalg.norm(t_composed - t_mount))
        rotation_cos = np.clip((np.trace(r_mount.T @ r_composed) - 1.0) / 2.0, -1.0, 1.0)
        rotation_error_deg = float(np.degrees(np.arccos(rotation_cos)))
        if translation_error <= 0.001 and rotation_error_deg <= 0.1:
            result.ok("Composed T_body_imu * T_imu_lidar matches robot_config T_body_lidar")
        else:
            result.fail(
                "LiDAR mount transform mismatch: "
                f"translation={translation_error:.6f}m, "
                f"rotation={rotation_error_deg:.4f}deg"
            )


def check_lidar_camera_projection(cfg: dict, result: CheckResult, verbose: bool) -> None:
    """Validate LiDAR→camera projection chain using synthetic test points.

    Computes T_body_lidar and T_body_camera from config, then projects
    synthetic body-frame points through the complete LiDAR→camera chain.
    Body-frame points avoid assuming that any particular LiDAR axis points
    toward the front of the robot.
    """
    logger.info(f"\n{BOLD}== LiDAR→Camera Projection =={RESET}")

    cam = cfg.get("camera", {})
    lidar = cfg.get("lidar", {})

    if not cam or not lidar:
        result.warn("Cannot check projection — missing camera or lidar config")
        return

    fx = cam.get("fx", 0)
    fy = cam.get("fy", 0)
    cx = cam.get("cx", 0)
    cy = cam.get("cy", 0)
    w = cam.get("width", 0)
    h = cam.get("height", 0)

    if fx <= 0 or fy <= 0 or w <= 0 or h <= 0:
        result.warn("Cannot check projection — invalid intrinsics")
        return

    try:
        T_body_lidar = np.eye(4)
        T_body_lidar[:3, :3] = _rotation_from_rpy(
            float(lidar.get("roll", 0)),
            float(lidar.get("pitch", 0)),
            float(lidar.get("yaw", 0)),
        )
        T_body_lidar[:3, 3] = [
            float(lidar.get("offset_x", 0)),
            float(lidar.get("offset_y", 0)),
            float(lidar.get("offset_z", 0)),
        ]

        T_body_camera = np.eye(4)
        T_body_camera[:3, :3] = _rotation_from_rpy(
            float(cam.get("roll", 0)),
            float(cam.get("pitch", 0)),
            float(cam.get("yaw", 0)),
        )
        T_body_camera[:3, 3] = [
            float(cam.get("position_x", 0)),
            float(cam.get("position_y", 0)),
            float(cam.get("position_z", 0)),
        ]
    except (TypeError, ValueError) as exc:
        result.fail(f"Invalid LiDAR/camera extrinsics: {exc}")
        return

    if not np.isfinite(T_body_lidar).all() or not np.isfinite(T_body_camera).all():
        result.fail("LiDAR/camera transform contains non-finite values")
        return

    try:
        T_camera_body = np.linalg.inv(T_body_camera)
        T_lidar_body = np.linalg.inv(T_body_lidar)
    except np.linalg.LinAlgError:
        result.fail("LiDAR/camera transform is not invertible")
        return
    T_camera_lidar = T_camera_body @ T_body_lidar

    # Synthetic points are defined in the body frame, where +X is the robot's
    # documented forward direction. Convert body→LiDAR first, then exercise
    # the configured LiDAR→camera chain. This remains valid for a rotated LiDAR.
    test_points_body = np.array(
        [
            [5.0, 0.0, 0.0],  # center, 5m ahead
            [5.0, -1.0, 0.0],  # 1m right
            [5.0, 1.0, 0.0],  # 1m left
            [5.0, 0.0, 0.5],  # 0.5m above
            [5.0, 0.0, -0.5],  # 0.5m below
            [-2.0, 0.0, 0.0],  # behind the robot
        ],
        dtype=np.float64,
    )

    forward_in_image = 0
    rear_is_behind = False
    proj_results = []

    for i, pt_body in enumerate(test_points_body):
        point_body_h = np.array([*pt_body, 1.0])
        point_lidar_h = T_lidar_body @ point_body_h
        point_camera = (T_camera_lidar @ point_lidar_h)[:3]
        point_camera_direct = (T_camera_body @ point_body_h)[:3]
        if not np.isfinite(point_camera).all() or not np.allclose(
            point_camera,
            point_camera_direct,
            atol=1e-9,
        ):
            result.fail("LiDAR→camera transform chain is non-finite or inconsistent")
            return

        # Camera convention: Z is depth (forward), X right, Y down
        # If Z <= 0, point is behind camera
        if point_camera[2] <= 0:
            if i == len(test_points_body) - 1:
                rear_is_behind = True
            proj_results.append(
                f"  pt[{i}] body=({pt_body[0]:.1f},{pt_body[1]:.1f},{pt_body[2]:.1f}) "
                f"→ behind camera (Z={point_camera[2]:.2f})"
            )
            continue

        # Project to pixels
        u = fx * point_camera[0] / point_camera[2] + cx
        v = fy * point_camera[1] / point_camera[2] + cy
        in_image = 0 <= u <= w and 0 <= v <= h
        if i < len(test_points_body) - 1 and in_image:
            forward_in_image += 1
        proj_results.append(
            f"  pt[{i}] body=({pt_body[0]:.1f},{pt_body[1]:.1f},{pt_body[2]:.1f}) → "
            f"cam=({point_camera[0]:.2f},{point_camera[1]:.2f},{point_camera[2]:.2f}) → "
            f"pixel=({u:.0f},{v:.0f}) {'IN' if in_image else 'OUT'}"
        )

    if verbose:
        for line in proj_results:
            logger.info(line)

    if forward_in_image >= 3:
        result.ok(f"LiDAR→camera projection: {forward_in_image}/5 body-forward points project into {w}x{h}")
    elif forward_in_image >= 1:
        result.warn(
            "LiDAR→camera projection: only "
            f"{forward_in_image}/5 body-forward points in image — "
            f"check camera-body or LiDAR-body extrinsics"
        )
    else:
        result.fail("LiDAR→camera projection: 0/5 body-forward points in image — extrinsic chain is likely wrong")

    if rear_is_behind:
        result.ok("Rear point correctly rejected (behind camera)")
    elif verbose:
        result.warn("Rear point not rejected — camera may face backward?")


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify sensor calibration")
    parser.add_argument("--config", default=str(ROBOT_CONFIG), help="Path to robot_config.yaml")
    parser.add_argument("--verbose", "-v", action="store_true", help="Show additional checks")
    args = parser.parse_args()

    try:
        cfg = load_yaml(Path(args.config))
        if not cfg:
            logger.error("Failed to load config from %s", args.config)
            return 1

        result = CheckResult()

        check_camera(cfg, result, args.verbose)
        check_lidar(cfg, result, args.verbose)
        check_imu(result, args.verbose)
        check_consistency(cfg, result)
        check_lidar_camera_projection(cfg, result, args.verbose)
    except CalibrationConfigError as exc:
        logger.error("%s", exc)
        return 1

    # Summary
    logger.info(f"\n{BOLD}== Summary =={RESET}")
    logger.info(
        f"  {GREEN}{result.passed} passed{RESET}, "
        f"{YELLOW}{result.warned} warnings{RESET}, "
        f"{RED}{result.failed} failed{RESET}"
    )

    if result.failed > 0:
        logger.info(f"\n{RED}Some checks failed. Run calibration for affected sensors.{RESET}")
        return 1
    if result.warned > 0:
        logger.info(f"\n{YELLOW}All critical checks passed, but some warnings need attention.{RESET}")
    else:
        logger.info(f"\n{GREEN}All calibration checks passed.{RESET}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
