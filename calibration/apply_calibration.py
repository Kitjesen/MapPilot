#!/usr/bin/env python3
"""Apply calibration results to robot_config.yaml and SLAM configs.

Reads output from each calibration tool and writes the parameters into
the appropriate config files. Backs up originals before overwriting.

Usage:
    # Apply all at once
    python calibration/apply_calibration.py \
        --camera calibration/camera/output/camera_calib.yaml \
        --imu calibration/imu/output/imu.yaml \
        --lidar-imu calibration/lidar_imu/output/lidar_imu_calib.yaml \
        --camera-lidar preprocessed_01/calib.json

    # Apply only camera intrinsics
    python calibration/apply_calibration.py \
        --camera calibration/camera/output/camera_calib.yaml

    # Dry run (show what would change, don't write)
    python calibration/apply_calibration.py --camera ... --dry-run
"""

import argparse
import importlib
import json
import logging
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path

import yaml

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent
ROBOT_CONFIG = REPO_ROOT / "config" / "robot_config.yaml"
FASTLIO2_CONFIG = REPO_ROOT / "src" / "slam" / "fastlio2" / "config" / "lio.yaml"
POINTLIO_CONFIG = REPO_ROOT / "config" / "pointlio.yaml"


class _LazyNumpy:
    def __getattr__(self, name: str):
        module = importlib.import_module("numpy")
        return getattr(module, name)


np = _LazyNumpy()


def backup_file(path: Path) -> None:
    """Create timestamped backup of a config file."""
    if not path.exists():
        return
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".{ts}.bak")
    shutil.copy2(path, backup)
    logger.info("  Backup: %s", backup.name)


def load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def save_yaml(path: Path, data: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False,
                  allow_unicode=True)


def pointlio_section(cfg: dict, section: str) -> dict:
    """Return the nested ROS2 parameter section dict for pointlio.yaml.

    pointlio.yaml uses the ROS2 parameter file layout
    `/** -> ros__parameters -> {common,mapping,preprocess,...}`.
    Writes into the dict returned here are reflected in the original tree.
    Creates missing intermediate nodes.
    """
    root = cfg.setdefault("/**", {})
    params = root.setdefault("ros__parameters", {})
    return params.setdefault(section, {})


def apply_camera_intrinsics(calib_path: str, dry_run: bool = False) -> None:
    """Apply camera intrinsic calibration to robot_config.yaml."""
    logger.info("\n== Camera Intrinsics ==")
    calib = load_yaml(Path(calib_path))

    K = calib.get("camera_matrix", {}).get("data", [])
    D = calib.get("distortion_coefficients", {}).get("data", [])
    width = calib.get("image_width", 0)
    height = calib.get("image_height", 0)

    if len(K) < 9:
        logger.error("  Invalid camera matrix in %s", calib_path)
        return

    fx, fy = K[0], K[4]
    cx, cy = K[2], K[5]

    logger.info("  fx=%.1f  fy=%.1f  cx=%.1f  cy=%.1f", fx, fy, cx, cy)
    logger.info("  Resolution: %dx%d", width, height)
    if D:
        logger.info("  Distortion: [%.6f, %.6f, %.6f, %.6f, %.6f]",
                     D[0], D[1], D[2] if len(D) > 2 else 0,
                     D[3] if len(D) > 3 else 0, D[4] if len(D) > 4 else 0)

    rms = calib.get("rms_reprojection_error", -1)
    if rms > 0:
        logger.info("  RMS reprojection error: %.4f px", rms)
        if rms > 1.0:
            logger.warning("  WARNING: High reprojection error, consider re-calibrating")

    if dry_run:
        logger.info("  [DRY RUN] Would update robot_config.yaml camera section")
        return

    backup_file(ROBOT_CONFIG)
    cfg = load_yaml(ROBOT_CONFIG)
    cam = cfg.setdefault("camera", {})
    cam["fx"] = round(fx, 1)
    cam["fy"] = round(fy, 1)
    cam["cx"] = round(cx, 1)
    cam["cy"] = round(cy, 1)
    cam["width"] = width
    cam["height"] = height
    if len(D) >= 5:
        cam["dist_k1"] = round(D[0], 8)
        cam["dist_k2"] = round(D[1], 8)
        cam["dist_p1"] = round(D[2], 8)
        cam["dist_p2"] = round(D[3], 8)
        cam["dist_k3"] = round(D[4], 8)

    save_yaml(ROBOT_CONFIG, cfg)
    logger.info("  Updated: %s", ROBOT_CONFIG.name)


# ICM-40609-D (Livox Mid-360 built-in IMU) datasheet typical values, used
# as a sanity-check reference. If a calibration result lies outside
# [low/5, high*5] we warn — the most common cause is recording on a
# vibrating surface or with other ROS nodes still pumping CPU heat into
# the IMU.
# Source: TDK InvenSense ICM-40609-D datasheet DS-000330 v1.2:
#   - Accelerometer Noise: 70 µg/√Hz typ → ~6.87e-4 m/s²/√Hz
#   - Gyroscope Noise:    3.8 mdps/√Hz typ → ~6.63e-5 rad/s/√Hz
# Note: ICM-40609 is ~10x quieter than BMI088, so the previous shipped
# default of na=ng=0.01 in lio.yaml is ~15x / ~150x too high respectively.
ICM40609_REFERENCE = {
    "na":  (3.0e-4, 3.0e-3),   # accel noise density (m/s²/√Hz), typ ~7e-4
    "ng":  (3.0e-5, 3.0e-4),   # gyro noise density (rad/s/√Hz),  typ ~6.6e-5
    "nba": (1.0e-5, 1.0e-3),   # accel random walk — datasheet does not list,
                               # range from typical Allan analyses on ICM4xxxx
    "nbg": (1.0e-7, 1.0e-5),   # gyro random walk — same caveat
}


def _sanity_check_imu_noise(name: str, value: float) -> None:
    """Compare measured IMU noise to ICM-40609 datasheet typical range and
    warn if it is wildly off. Tolerance is intentionally loose (±5x) —
    Allan Variance is sensitive to environment and sensor batch variance,
    but a 1-2 order-of-magnitude miss usually points at procedural error."""
    if name not in ICM40609_REFERENCE:
        return
    low, high = ICM40609_REFERENCE[name]
    if value < low / 5:
        logger.warning(
            "  WARNING: %s=%.6g is unusually LOW for ICM-40609 (datasheet typical %.2g–%.2g). "
            "Possible causes: integration window too short, IMU saturated, units mis-converted.",
            name, value, low, high)
    elif value > high * 5:
        logger.warning(
            "  WARNING: %s=%.6g is unusually HIGH for ICM-40609 (datasheet typical %.2g–%.2g). "
            "Possible causes: vibration during recording, thermal drift, AC vent draft.",
            name, value, low, high)
    else:
        logger.info("  %s=%.6g is within ICM-40609 expected range (%.2g–%.2g)",
                    name, value, low, high)


def apply_imu_noise(calib_path: str, dry_run: bool = False) -> None:
    """Apply IMU noise parameters to SLAM configs."""
    logger.info("\n== IMU Noise Parameters ==")
    calib = load_yaml(Path(calib_path))

    # Kalibr-format YAML from allan_variance_ros2
    na = calib.get("accelerometer_noise_density", None)
    ng = calib.get("gyroscope_noise_density", None)
    nba = calib.get("accelerometer_random_walk", None)
    nbg = calib.get("gyroscope_random_walk", None)

    if na is None or ng is None:
        logger.error("  Missing noise parameters in %s", calib_path)
        return

    logger.info("  Accel noise density (na): %.8f m/s^2/sqrt(Hz)", na)
    logger.info("  Gyro noise density  (ng): %.8f rad/s/sqrt(Hz)", ng)
    if nba:
        logger.info("  Accel random walk  (nba): %.8f", nba)
    if nbg:
        logger.info("  Gyro random walk   (nbg): %.8f", nbg)

    # Datasheet-grounded sanity warnings (ICM-40609-D = Mid-360 built-in IMU)
    _sanity_check_imu_noise("na", float(na))
    _sanity_check_imu_noise("ng", float(ng))
    if nba is not None:
        _sanity_check_imu_noise("nba", float(nba))
    if nbg is not None:
        _sanity_check_imu_noise("nbg", float(nbg))

    if dry_run:
        logger.info("  [DRY RUN] Would update lio.yaml and pointlio.yaml")
        return

    # Update Fast-LIO2 config
    if FASTLIO2_CONFIG.exists():
        backup_file(FASTLIO2_CONFIG)
        cfg = load_yaml(FASTLIO2_CONFIG)
        cfg["na"] = float(na)
        cfg["ng"] = float(ng)
        if nba:
            cfg["nba"] = float(nba)
        if nbg:
            cfg["nbg"] = float(nbg)
        save_yaml(FASTLIO2_CONFIG, cfg)
        logger.info("  Updated: %s", FASTLIO2_CONFIG.name)

    # Update Point-LIO config (ROS2 nested layout: /**.ros__parameters.mapping.*)
    if POINTLIO_CONFIG.exists():
        backup_file(POINTLIO_CONFIG)
        cfg = load_yaml(POINTLIO_CONFIG)
        mapping = pointlio_section(cfg, "mapping")
        mapping["imu_meas_acc_cov"] = float(na)
        mapping["imu_meas_omg_cov"] = float(ng)
        if nba:
            mapping["b_acc_cov"] = float(nba)
        if nbg:
            mapping["b_gyr_cov"] = float(nbg)
        save_yaml(POINTLIO_CONFIG, cfg)
        logger.info("  Updated: %s", POINTLIO_CONFIG.name)


def apply_lidar_imu(calib_path: str, dry_run: bool = False) -> None:
    """Apply LiDAR-IMU extrinsics to robot_config.yaml and SLAM configs."""
    logger.info("\n== LiDAR-IMU Extrinsics ==")
    calib = load_yaml(Path(calib_path))

    t_il = calib.get("t_il")
    r_il = calib.get("r_il")
    time_offset = calib.get("time_offset", 0.0)

    if t_il is None:
        logger.error("  Missing t_il in %s", calib_path)
        return

    logger.info("  Translation: [%.5f, %.5f, %.5f]", *t_il)
    if r_il:
        R = np.array(r_il).reshape(3, 3)
        angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        logger.info("  Rotation angle: %.2f deg", angle)
    logger.info("  Time offset: %.6f s", time_offset)

    if dry_run:
        logger.info("  [DRY RUN] Would update configs")
        return

    # Update robot_config.yaml lidar section
    backup_file(ROBOT_CONFIG)
    cfg = load_yaml(ROBOT_CONFIG)
    lidar = cfg.setdefault("lidar", {})
    lidar["offset_x"] = round(t_il[0], 6)
    lidar["offset_y"] = round(t_il[1], 6)
    lidar["offset_z"] = round(t_il[2], 6)
    if r_il:
        # Convert rotation to Rodrigues for config
        try:
            import cv2
            rvec, _ = cv2.Rodrigues(np.array(r_il).reshape(3, 3))
            lidar["roll"] = round(float(rvec[0]), 6)
            lidar["pitch"] = round(float(rvec[1]), 6)
            lidar["yaw"] = round(float(rvec[2]), 6)
        except ImportError:
            logger.warning("  cv2 not available, skipping rotation update")
    save_yaml(ROBOT_CONFIG, cfg)
    logger.info("  Updated: %s lidar section", ROBOT_CONFIG.name)

    if abs(time_offset) > 0.1:
        logger.warning(
            "  WARNING: time_offset %.6f s exceeds plausible ±0.1s — "
            "likely calibration error, not writing to configs", time_offset)
        write_time_offset = False
    else:
        write_time_offset = True

    # Update Fast-LIO2 config
    if FASTLIO2_CONFIG.exists():
        backup_file(FASTLIO2_CONFIG)
        cfg = load_yaml(FASTLIO2_CONFIG)
        if r_il:
            cfg["r_il"] = r_il
        cfg["t_il"] = t_il
        if write_time_offset:
            cfg["time_diff_lidar_to_imu"] = round(time_offset, 6)
        save_yaml(FASTLIO2_CONFIG, cfg)
        logger.info("  Updated: %s", FASTLIO2_CONFIG.name)

    # Update Point-LIO config (ROS2 nested layout)
    if POINTLIO_CONFIG.exists():
        backup_file(POINTLIO_CONFIG)
        cfg = load_yaml(POINTLIO_CONFIG)
        mapping = pointlio_section(cfg, "mapping")
        if r_il:
            mapping["extrinsic_R"] = r_il
        mapping["extrinsic_T"] = t_il
        if write_time_offset:
            common = pointlio_section(cfg, "common")
            common["time_diff_lidar_to_imu"] = round(time_offset, 6)
        save_yaml(POINTLIO_CONFIG, cfg)
        logger.info("  Updated: %s", POINTLIO_CONFIG.name)


def apply_camera_lidar(calib_path: str, dry_run: bool = False) -> None:
    """Apply camera-LiDAR extrinsics to robot_config.yaml."""
    logger.info("\n== Camera-LiDAR Extrinsics ==")

    with open(calib_path) as f:
        calib = json.load(f)

    # direct_visual_lidar_calibration outputs T_lidar_camera as [x, y, z, qx, qy, qz, qw]
    T_data = calib.get("T_lidar_camera") or calib.get("results", [{}])[0].get("T_lidar_camera")
    if T_data is None:
        logger.error("  No T_lidar_camera found in %s", calib_path)
        return

    if len(T_data) == 7:
        # [x, y, z, qx, qy, qz, qw] format
        x, y, z, qx, qy, qz, qw = T_data
        try:
            import cv2

            # Quaternion to rotation matrix
            from scipy.spatial.transform import Rotation
            R_lc = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            t_lc = np.array([x, y, z])
        except ImportError:
            logger.error("  scipy required for quaternion conversion")
            return
    elif len(T_data) == 16:
        # 4x4 matrix format
        T = np.array(T_data).reshape(4, 4)
        R_lc = T[:3, :3]
        t_lc = T[:3, 3]
    else:
        logger.error("  Unexpected T_lidar_camera format (len=%d)", len(T_data))
        return

    # T_lidar_camera gives camera pose in LiDAR frame.
    # We need camera pose in body frame for robot_config.yaml.
    # T_body_camera = T_body_lidar @ T_lidar_camera
    # We read T_body_lidar from current robot_config.yaml lidar section.
    cfg = load_yaml(ROBOT_CONFIG)
    lidar = cfg.get("lidar", {})
    t_bl = np.array([
        lidar.get("offset_x", -0.011),
        lidar.get("offset_y", -0.02329),
        lidar.get("offset_z", 0.04412),
    ])

    # Construct T_body_lidar (assume identity rotation if not specified)
    T_body_lidar = np.eye(4)
    T_body_lidar[:3, 3] = t_bl
    roll = lidar.get("roll", 0)
    pitch = lidar.get("pitch", 0)
    yaw = lidar.get("yaw", 0)
    rvec = np.array([roll, pitch, yaw], dtype=np.float64)
    if not np.allclose(rvec, 0):
        try:
            import cv2
            R_bl, _ = cv2.Rodrigues(rvec)
            T_body_lidar[:3, :3] = R_bl
        except ImportError:
            pass

    # T_lidar_camera
    T_lidar_camera = np.eye(4)
    T_lidar_camera[:3, :3] = R_lc
    T_lidar_camera[:3, 3] = t_lc

    # T_body_camera = T_body_lidar @ T_lidar_camera
    T_body_camera = T_body_lidar @ T_lidar_camera

    pos = T_body_camera[:3, 3]
    R_bc = T_body_camera[:3, :3]

    logger.info("  Camera in body frame:")
    logger.info("    Position: [%.4f, %.4f, %.4f]", *pos)

    try:
        import cv2
        rvec_bc, _ = cv2.Rodrigues(R_bc)
        logger.info("    Rodrigues: [%.6f, %.6f, %.6f]", *rvec_bc.flatten())
    except ImportError:
        rvec_bc = None

    if dry_run:
        logger.info("  [DRY RUN] Would update robot_config.yaml camera section")
        return

    backup_file(ROBOT_CONFIG)
    cfg = load_yaml(ROBOT_CONFIG)
    cam = cfg.setdefault("camera", {})
    cam["position_x"] = round(float(pos[0]), 5)
    cam["position_y"] = round(float(pos[1]), 5)
    cam["position_z"] = round(float(pos[2]), 5)
    if rvec_bc is not None:
        cam["roll"] = round(float(rvec_bc[0]), 6)
        cam["pitch"] = round(float(rvec_bc[1]), 6)
        cam["yaw"] = round(float(rvec_bc[2]), 6)

    save_yaml(ROBOT_CONFIG, cfg)
    logger.info("  Updated: %s camera extrinsics", ROBOT_CONFIG.name)


def main():
    parser = argparse.ArgumentParser(
        description="Apply calibration results to robot configuration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--camera", help="Camera intrinsic calibration YAML")
    parser.add_argument("--imu", help="IMU noise calibration YAML (Kalibr format)")
    parser.add_argument("--lidar-imu", help="LiDAR-IMU extrinsic calibration YAML")
    parser.add_argument("--camera-lidar", help="Camera-LiDAR extrinsic calibration JSON")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show what would change without writing")

    args = parser.parse_args()

    if not any([args.camera, args.imu, args.lidar_imu, args.camera_lidar]):
        parser.print_help()
        logger.error("\nNo calibration results specified. Use --camera, --imu, "
                     "--lidar-imu, or --camera-lidar.")
        sys.exit(1)

    logger.info("Applying calibration results to config files...")
    if args.dry_run:
        logger.info("[DRY RUN MODE — no files will be modified]\n")

    if args.camera:
        apply_camera_intrinsics(args.camera, args.dry_run)
    if args.imu:
        apply_imu_noise(args.imu, args.dry_run)
    if args.lidar_imu:
        apply_lidar_imu(args.lidar_imu, args.dry_run)
    if args.camera_lidar:
        apply_camera_lidar(args.camera_lidar, args.dry_run)

    logger.info("\nDone. Run 'python calibration/verify.py' to validate.")


if __name__ == "__main__":
    main()
