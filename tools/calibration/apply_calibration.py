#!/usr/bin/env python3
"""Apply calibration results to the Go2 RobotConfig and SLAM configs.

Reads output from each calibration tool and writes the parameters into
the appropriate config files. Backs up originals before overwriting.

Usage:
    # Apply all at once
    python tools/calibration/apply_calibration.py \
        --camera tools/calibration/camera/output/camera_calib.yaml \
        --imu tools/calibration/imu/output/imu.yaml \
        --lidar-imu tools/calibration/lidar_imu/output/lidar_imu_calib.yaml \
        --camera-lidar preprocessed_01/calib.json

    # Apply only camera intrinsics
    python tools/calibration/apply_calibration.py \
        --camera tools/calibration/camera/output/camera_calib.yaml

    # Dry run (show what would change, don't write)
    python tools/calibration/apply_calibration.py --camera ... --dry-run
"""

import argparse
import importlib
import json
import logging
import os
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import yaml

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml"
FASTLIO2_CONFIG = (
    REPO_ROOT
    / "config"
    / "robots"
    / "unitree"
    / "go2"
    / "sensors"
    / "mid360_fastlio2.yaml"
)


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
    """Atomically replace a YAML file using a temporary sibling file."""
    fd, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        text=True,
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            yaml.dump(
                data,
                f,
                default_flow_style=False,
                sort_keys=False,
                allow_unicode=True,
            )
            f.flush()
            os.fsync(f.fileno())
        if path.exists():
            shutil.copymode(path, temp_path)
        os.replace(temp_path, path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise


def derive_body_from_imu(
    lidar_mount: dict,
    r_il: list[float],
    t_il: list[float],
) -> tuple[list[float], list[float]]:
    """Derive ``T_body_imu`` while preserving the fixed ``T_body_lidar`` mount.

    Fast-LIO uses ``T_imu_lidar``:
    ``p_imu = R_il * p_lidar + t_il``. Robot configuration stores the separate
    mechanical mount ``T_body_lidar``. Therefore:

    ``T_body_imu = T_body_lidar * inverse(T_imu_lidar)``.
    """

    r_il_matrix = np.asarray(r_il, dtype=np.float64).reshape(3, 3)
    t_il_vector = np.asarray(t_il, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(r_il_matrix)) or not np.all(np.isfinite(t_il_vector)):
        raise ValueError("LiDAR-IMU extrinsics must be finite")
    if not np.allclose(r_il_matrix.T @ r_il_matrix, np.eye(3), atol=1e-6):
        raise ValueError("r_il must be an orthonormal rotation matrix")
    if not np.isclose(np.linalg.det(r_il_matrix), 1.0, atol=1e-6):
        raise ValueError("r_il determinant must be +1")

    roll = float(lidar_mount.get("roll", 0.0))
    pitch = float(lidar_mount.get("pitch", 0.0))
    yaw = float(lidar_mount.get("yaw", 0.0))
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    r_body_lidar = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    t_body_lidar = np.asarray(
        [
            float(lidar_mount.get("offset_x", 0.0)),
            float(lidar_mount.get("offset_y", 0.0)),
            float(lidar_mount.get("offset_z", 0.0)),
        ],
        dtype=np.float64,
    )

    r_body_imu = r_body_lidar @ r_il_matrix.T
    t_body_imu = t_body_lidar - r_body_imu @ t_il_vector
    return r_body_imu.reshape(-1).tolist(), t_body_imu.tolist()


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
        logger.info(
            "  Distortion: [%.6f, %.6f, %.6f, %.6f, %.6f]",
            D[0],
            D[1],
            D[2] if len(D) > 2 else 0,
            D[3] if len(D) > 3 else 0,
            D[4] if len(D) > 4 else 0,
        )

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
    "na": (3.0e-4, 3.0e-3),  # accel noise density (m/s²/√Hz), typ ~7e-4
    "ng": (3.0e-5, 3.0e-4),  # gyro noise density (rad/s/√Hz),  typ ~6.6e-5
    "nba": (1.0e-5, 1.0e-3),  # accel random walk — datasheet does not list,
    # range from typical Allan analyses on ICM4xxxx
    "nbg": (1.0e-7, 1.0e-5),  # gyro random walk — same caveat
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
            "  WARNING: %s=%.6g is unusually LOW for ICM-40609 (datasheet typical %.2g-%.2g). "
            "Possible causes: integration window too short, IMU saturated, units mis-converted.",
            name,
            value,
            low,
            high,
        )
    elif value > high * 5:
        logger.warning(
            "  WARNING: %s=%.6g is unusually HIGH for ICM-40609 (datasheet typical %.2g-%.2g). "
            "Possible causes: vibration during recording, thermal drift, AC vent draft.",
            name,
            value,
            low,
            high,
        )
    else:
        logger.info("  %s=%.6g is within ICM-40609 expected range (%.2g-%.2g)", name, value, low, high)


def apply_imu_noise(calib_path: str, dry_run: bool = False) -> None:
    """Apply IMU noise parameters to SLAM configs."""
    logger.info("\n== IMU Noise Parameters ==")
    try:
        calib = load_yaml(Path(calib_path))
        fastlio_exists = FASTLIO2_CONFIG.exists()
        fastlio_cfg = load_yaml(FASTLIO2_CONFIG) if fastlio_exists else {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        logger.error("  Cannot read calibration target configs: %s", exc)
        return

    if not isinstance(calib, dict):
        logger.error("  IMU calibration result in %s must contain a YAML mapping", calib_path)
        return
    if not isinstance(fastlio_cfg, dict):
        logger.error("  %s must contain a YAML mapping", FASTLIO2_CONFIG.name)
        return

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

    if fastlio_exists:
        fastlio_cfg["na"] = float(na)
        fastlio_cfg["ng"] = float(ng)
        if nba is not None:
            fastlio_cfg["nba"] = float(nba)
        if nbg is not None:
            fastlio_cfg["nbg"] = float(nbg)

    if dry_run:
        logger.info("  [DRY RUN] Would update %s", FASTLIO2_CONFIG.name)
        return

    if fastlio_exists:
        backup_file(FASTLIO2_CONFIG)
        save_yaml(FASTLIO2_CONFIG, fastlio_cfg)
        logger.info("  Updated: %s", FASTLIO2_CONFIG.name)


def apply_lidar_imu(calib_path: str, dry_run: bool = False) -> None:
    """Apply internal LiDAR-IMU extrinsics without changing the body mount."""
    logger.info("\n== LiDAR-IMU Extrinsics ==")
    try:
        calib = load_yaml(Path(calib_path))
        fastlio_exists = FASTLIO2_CONFIG.exists()
        fastlio_cfg = load_yaml(FASTLIO2_CONFIG) if fastlio_exists else {}
        robot_cfg = load_yaml(ROBOT_CONFIG)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        logger.error("  Cannot read calibration target configs: %s", exc)
        return

    configs = {
        "calibration result": calib,
        ROBOT_CONFIG.name: robot_cfg,
    }
    if fastlio_exists:
        configs[FASTLIO2_CONFIG.name] = fastlio_cfg
    for name, cfg in configs.items():
        if not isinstance(cfg, dict):
            logger.error("  %s must contain a YAML mapping", name)
            return

    t_il = calib.get("t_il")
    r_il = calib.get("r_il")
    time_offset = calib.get("time_offset", 0.0)

    if t_il is None:
        logger.error("  Missing t_il in %s", calib_path)
        return

    effective_r_il = r_il or fastlio_cfg.get("r_il")
    if effective_r_il is None:
        logger.error("  Missing r_il in %s and %s", calib_path, FASTLIO2_CONFIG)
        return

    lidar_mount = robot_cfg.get("lidar")
    if not isinstance(lidar_mount, dict) or not lidar_mount:
        logger.error("  Missing fixed body-to-LiDAR mount in %s", ROBOT_CONFIG)
        return

    try:
        r_body_imu, t_body_imu = derive_body_from_imu(
            lidar_mount,
            effective_r_il,
            t_il,
        )
    except (TypeError, ValueError) as exc:
        logger.error("  Invalid LiDAR-IMU extrinsics: %s", exc)
        return

    logger.info("  T_imu_lidar translation: [%.5f, %.5f, %.5f]", *t_il)
    if effective_r_il:
        R = np.array(effective_r_il).reshape(3, 3)
        angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        logger.info("  T_imu_lidar rotation angle: %.2f deg", angle)
    logger.info(
        "  Preserving T_body_lidar mount: [%.5f, %.5f, %.5f]",
        lidar_mount.get("offset_x", 0.0),
        lidar_mount.get("offset_y", 0.0),
        lidar_mount.get("offset_z", 0.0),
    )
    logger.info("  Derived T_body_imu translation: [%.5f, %.5f, %.5f]", *t_body_imu)
    logger.info("  Time offset: %.6f s", time_offset)

    if dry_run:
        logger.info(
            "  [DRY RUN] Would update internal SLAM extrinsics; robot_config.yaml LiDAR mount remains unchanged"
        )
        return

    if abs(time_offset) > 0.1:
        logger.warning(
            "  WARNING: time_offset %.6f s exceeds plausible ±0.1s — likely calibration error, not writing to configs",
            time_offset,
        )
        write_time_offset = False
    else:
        write_time_offset = True

    # Build every target update before creating backups or replacing files.
    if fastlio_exists:
        fastlio_cfg["r_il"] = effective_r_il
        fastlio_cfg["t_il"] = t_il
        fastlio_cfg["navigation_body_from_imu_rotation"] = r_body_imu
        fastlio_cfg["navigation_body_from_imu_translation"] = t_body_imu
        if write_time_offset:
            fastlio_cfg["time_diff_lidar_to_imu"] = round(time_offset, 6)

    if fastlio_exists:
        backup_file(FASTLIO2_CONFIG)
        save_yaml(FASTLIO2_CONFIG, fastlio_cfg)
        logger.info("  Updated: %s", FASTLIO2_CONFIG.name)


def _parse_camera_lidar_transform(calib: object, source: str):
    """Return validated camera pose in the LiDAR frame from a result document."""
    if not isinstance(calib, dict):
        logger.error("  Camera-LiDAR result in %s must be a JSON object", source)
        return None

    transform_data = calib.get("T_lidar_camera")
    if transform_data is None:
        results = calib.get("results")
        if results is not None and not isinstance(results, dict):
            logger.error("  Camera-LiDAR results in %s must be a JSON object", source)
            return None
        transform_data = results.get("T_lidar_camera") if results is not None else None
    if transform_data is None:
        logger.error("  No T_lidar_camera found in %s", source)
        return None

    try:
        values = np.asarray(transform_data, dtype=np.float64)
    except (TypeError, ValueError):
        logger.error("  T_lidar_camera in %s must contain only numeric values", source)
        return None
    if values.size not in (7, 16):
        logger.error(
            "  T_lidar_camera in %s must contain 7 quaternion-pose or 16 matrix values (got %d)",
            source,
            values.size,
        )
        return None
    values = values.reshape(-1)
    if not np.all(np.isfinite(values)):
        logger.error("  T_lidar_camera in %s contains non-finite values", source)
        return None

    try:
        from scipy.spatial.transform import Rotation
    except ImportError:
        logger.error("  scipy required for camera-LiDAR rotation conversion")
        return None

    if values.size == 7:
        # [x, y, z, qx, qy, qz, qw] format
        translation = values[:3]
        quaternion = values[3:]
        if np.linalg.norm(quaternion) <= 1e-12:
            logger.error("  T_lidar_camera quaternion in %s has zero length", source)
            return None
        try:
            rotation = Rotation.from_quat(quaternion).as_matrix()
        except ValueError as exc:
            logger.error("  Invalid T_lidar_camera quaternion in %s: %s", source, exc)
            return None
        return rotation, translation

    if values.size == 16:
        # 4x4 matrix format
        transform = values.reshape(4, 4)
        rotation = transform[:3, :3]
        if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
            logger.error("  T_lidar_camera in %s is not a valid homogeneous rigid transform", source)
            return None
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(
            np.linalg.det(rotation),
            1.0,
            atol=1e-5,
        ):
            logger.error("  T_lidar_camera in %s is not a valid rigid transform", source)
            return None
        return rotation, transform[:3, 3]

    return None


def apply_camera_lidar(calib_path: str, dry_run: bool = False) -> None:
    """Apply camera-LiDAR extrinsics to robot_config.yaml."""
    logger.info("\n== Camera-LiDAR Extrinsics ==")

    try:
        with open(calib_path, encoding="utf-8") as f:
            calib = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        logger.error("  Cannot read camera-LiDAR result %s: %s", calib_path, exc)
        return

    parsed = _parse_camera_lidar_transform(calib, calib_path)
    if parsed is None:
        return
    R_lc, t_lc = parsed

    from scipy.spatial.transform import Rotation

    # T_lidar_camera gives camera pose in LiDAR frame.
    # We need camera pose in body frame for robot_config.yaml.
    # T_body_camera = T_body_lidar @ T_lidar_camera
    # We read T_body_lidar from current robot_config.yaml lidar section.
    cfg = load_yaml(ROBOT_CONFIG)
    lidar = cfg.get("lidar", {})
    t_bl = np.array(
        [
            lidar.get("offset_x", -0.011),
            lidar.get("offset_y", -0.02329),
            lidar.get("offset_z", 0.04412),
        ]
    )

    # Construct T_body_lidar using the RobotConfig fixed-axis RPY convention.
    T_body_lidar = np.eye(4)
    T_body_lidar[:3, 3] = t_bl
    roll = lidar.get("roll", 0)
    pitch = lidar.get("pitch", 0)
    yaw = lidar.get("yaw", 0)
    T_body_lidar[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()

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

    rpy_bc = Rotation.from_matrix(R_bc).as_euler("xyz")
    logger.info("    Fixed-axis RPY: [%.6f, %.6f, %.6f]", *rpy_bc)

    if dry_run:
        logger.info("  [DRY RUN] Would update robot_config.yaml camera section")
        return

    backup_file(ROBOT_CONFIG)
    cfg = load_yaml(ROBOT_CONFIG)
    cam = cfg.setdefault("camera", {})
    cam["position_x"] = round(float(pos[0]), 5)
    cam["position_y"] = round(float(pos[1]), 5)
    cam["position_z"] = round(float(pos[2]), 5)
    cam["roll"] = round(float(rpy_bc[0]), 6)
    cam["pitch"] = round(float(rpy_bc[1]), 6)
    cam["yaw"] = round(float(rpy_bc[2]), 6)

    save_yaml(ROBOT_CONFIG, cfg)
    logger.info("  Updated: %s camera extrinsics", ROBOT_CONFIG.name)


def main():
    global ROBOT_CONFIG, FASTLIO2_CONFIG

    parser = argparse.ArgumentParser(
        description="Apply calibration results to robot configuration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--camera", help="Camera intrinsic calibration YAML")
    parser.add_argument("--imu", help="IMU noise calibration YAML (Kalibr format)")
    parser.add_argument("--lidar-imu", help="LiDAR-IMU extrinsic calibration YAML")
    parser.add_argument("--camera-lidar", help="Camera-LiDAR extrinsic calibration JSON")
    parser.add_argument(
        "--robot-config",
        default=str(ROBOT_CONFIG),
        help="RobotConfig to update",
    )
    parser.add_argument(
        "--slam-config",
        default=str(FASTLIO2_CONFIG),
        help="Robot-specific Fast-LIO2 MID-360 config to update",
    )
    parser.add_argument("--dry-run", action="store_true", help="Show what would change without writing")

    args = parser.parse_args()
    ROBOT_CONFIG = Path(args.robot_config)
    FASTLIO2_CONFIG = Path(args.slam_config)

    if not any([args.camera, args.imu, args.lidar_imu, args.camera_lidar]):
        parser.print_help()
        logger.error("\nNo calibration results specified. Use --camera, --imu, --lidar-imu, or --camera-lidar.")
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

    logger.info("\nDone. Run 'python tools/calibration/verify.py' to validate.")


if __name__ == "__main__":
    main()
