#!/usr/bin/env python3
"""Parse LiDAR_IMU_Init output (Initialization_result.txt) into structured YAML.

LI-Init writes a result file containing:
  - LiDAR-IMU rotation (3x3 matrix)
  - LiDAR-IMU translation (3x1 vector)
  - Time offset (lidar-to-imu)
  - IMU bias (accelerometer + gyroscope)
  - Gravity vector

This script parses that file and outputs a clean YAML that
apply_calibration.py can consume.

Usage:
    python tools/calibration/lidar_imu/ros2_adapter/parse_result.py \
        --input tools/calibration/lidar_imu/LiDAR_IMU_Init/result/Initialization_result.txt \
        --output tools/calibration/lidar_imu/output/lidar_imu_calib.yaml
"""

import argparse
import logging
import re
import sys

import numpy as np
import yaml

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

_NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"


def _last_values(pattern: str, content: str, count: int) -> list[float] | None:
    matches = list(re.finditer(pattern, content, re.IGNORECASE))
    if not matches:
        return None
    match = matches[-1]
    return [float(match.group(index)) for index in range(1, count + 1)]


def parse_initialization_result(filepath: str) -> dict:
    """Parse LI-Init Initialization_result.txt.

    Current LI-Init files contain single-line Euler angles plus a homogeneous
    transform, and may append a refined result. The last matching values are
    the final result. The legacy three-row rotation format remains supported:

        Rotation LiDAR to IMU =
        0.999  -0.001   0.002
        0.001   0.999  -0.003
        -0.002  0.003   0.999
        Translation LiDAR to IMU =
        -0.011  -0.023   0.044
        Time offset (lidar to imu) =    0.001
        Accelerometer bias =    0.01  0.02  -0.03
        Gyroscope bias =    0.001  -0.002  0.003
        Gravity =    0.00  0.00  -9.81
    """
    with open(filepath, encoding="utf-8") as f:
        content = f.read()

    result = {}

    # Parse rotation matrix
    rot_values = _last_values(
        r"Rotation\s+LiDAR\s+to\s+IMU\s*[=:]\s*\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s*\r?\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s*\r?\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})",
        content,
        9,
    )
    if rot_values:
        result["r_il"] = rot_values

    # Parse translation
    translation = _last_values(
        r"Translation\s+LiDAR\s+to\s+IMU(?:\s*\([^\r\n)]*\))?\s*[=:]\s*\r?\n?"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})",
        content,
        3,
    )
    if translation:
        result["t_il"] = translation

    # Parse time offset
    time_offset = _last_values(
        rf"(?:Time\s+offset[^\r\n]*?|Time\s+Lag\s+IMU\s+to\s+LiDAR(?:\s*\([^\r\n)]*\))?)"
        rf"\s*[=:]\s*({_NUMBER})",
        content,
        1,
    )
    if time_offset:
        result["time_offset"] = time_offset[0]

    # Parse accelerometer bias
    acc_bias = _last_values(
        rf"(?:Accelerometer\s+bias|Bias\s+of\s+Accelerometer)(?:\s*\([^\r\n)]*\))?"
        rf"\s*[=:]\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})",
        content,
        3,
    )
    if acc_bias:
        result["acc_bias"] = acc_bias

    # Parse gyroscope bias
    gyr_bias = _last_values(
        rf"(?:Gyroscope\s+bias|Bias\s+of\s+Gyroscope)(?:\s*\([^\r\n)]*\))?"
        rf"\s*[=:]\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})",
        content,
        3,
    )
    if gyr_bias:
        result["gyr_bias"] = gyr_bias

    # Parse gravity
    gravity = _last_values(
        rf"Gravity(?:\s+in\s+World\s+Frame)?(?:\s*\([^\r\n)]*\))?"
        rf"\s*[=:]\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})",
        content,
        3,
    )
    if gravity:
        result["gravity"] = gravity

    # Try to parse 4x4 transformation matrix if present
    transform_values = _last_values(
        r"(?:Homogeneous\s+Transformation\s+Matrix\s+from\s+LiDAR\s+to\s+IMU|"
        r"Transformation|T_LI|Extrinsic)\s*[=:]?\s*\r?\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s*\r?\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s*\r?\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s*\r?\n"
        rf"\s*({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})\s+({_NUMBER})",
        content,
        16,
    )
    if transform_values:
        T = np.array(transform_values).reshape(4, 4)
        result["T_lidar_imu"] = T.tolist()
        result["r_il"] = T[:3, :3].flatten().tolist()
        result["t_il"] = T[:3, 3].tolist()

    if "r_il" in result:
        R = np.array(result["r_il"]).reshape(3, 3)
        try:
            import cv2

            rvec, _ = cv2.Rodrigues(R)
            result["rotation_rodrigues"] = rvec.flatten().tolist()
        except ImportError:
            pass

    if "r_il" not in result:
        logger.warning("Could not parse rotation matrix")
    if "t_il" not in result:
        logger.warning("Could not parse translation")

    return result


def validate_result(result: dict) -> list:
    """Sanity-check calibration result. Returns list of warnings."""
    warnings = []

    if "t_il" in result:
        t = np.array(result["t_il"])
        mag = np.linalg.norm(t)
        if mag > 0.5:
            warnings.append(f"LiDAR-IMU translation magnitude {mag:.3f}m seems large for co-located sensors")

    if "r_il" in result:
        R = np.array(result["r_il"]).reshape(3, 3)
        det = np.linalg.det(R)
        if abs(det - 1.0) > 0.01:
            warnings.append(f"Rotation matrix determinant {det:.4f} != 1.0")
        # Check rotation angle
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        if np.degrees(angle) > 30:
            warnings.append(f"Rotation angle {np.degrees(angle):.1f} deg seems too large")

    if "gravity" in result:
        g = np.linalg.norm(result["gravity"])
        if abs(g - 9.81) > 0.5:
            warnings.append(f"Gravity magnitude {g:.3f} m/s^2 deviates from 9.81")

    if "time_offset" in result:
        dt = abs(result["time_offset"])
        if dt > 0.1:
            warnings.append(f"Time offset {dt:.4f}s seems large (>100ms)")

    return warnings


def save_yaml(result: dict, output_path: str) -> None:
    """Save parsed result as clean YAML."""
    with open(output_path, "w") as f:
        yaml.dump(result, f, default_flow_style=False, sort_keys=False)
    logger.info("Saved to %s", output_path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Parse LiDAR_IMU_Init result")
    parser.add_argument("--input", required=True, help="Path to Initialization_result.txt")
    parser.add_argument(
        "--output", default="tools/calibration/lidar_imu/output/lidar_imu_calib.yaml", help="Output YAML path"
    )
    args = parser.parse_args()

    logger.info("Parsing %s ...", args.input)
    result = parse_initialization_result(args.input)

    missing = [field for field in ("r_il", "t_il") if field not in result]
    if missing:
        logger.error(
            "Missing required calibration field(s): %s; not writing YAML",
            ", ".join(missing),
        )
        return 1

    # Validate
    warnings = validate_result(result)
    for w in warnings:
        logger.warning("  WARNING: %s", w)

    # Print summary
    if "t_il" in result:
        t = result["t_il"]
        logger.info("  Translation (LiDAR→IMU): [%.5f, %.5f, %.5f]", *t)
    if "r_il" in result:
        R = np.array(result["r_il"]).reshape(3, 3)
        angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        logger.info("  Rotation angle: %.2f deg", angle)
    if "time_offset" in result:
        logger.info("  Time offset: %.6f s", result["time_offset"])
    if "gravity" in result:
        g = result["gravity"]
        logger.info("  Gravity: [%.4f, %.4f, %.4f] (norm=%.4f)", g[0], g[1], g[2], np.linalg.norm(g))

    save_yaml(result, args.output)

    if not warnings:
        logger.info("  All checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
