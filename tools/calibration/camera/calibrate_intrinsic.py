#!/usr/bin/env python3
"""Camera intrinsic calibration using OpenCV checkerboard detection.

Usage:
    # Capture mode — save checkerboard images from live camera
    python tools/calibration/camera/calibrate_intrinsic.py capture --device 0 --out calibration_imgs/

    # Calibrate from saved images
    python tools/calibration/camera/calibrate_intrinsic.py calibrate --images calibration_imgs/ --out tools/calibration/camera/output/camera_calib.yaml

    # One-shot — capture + calibrate interactively
    python tools/calibration/camera/calibrate_intrinsic.py auto --device 0

    # Verify existing calibration
    python tools/calibration/camera/calibrate_intrinsic.py verify --calib tools/calibration/camera/output/camera_calib.yaml --device 0

    # From ROS2 bag (no GUI needed)
    python tools/calibration/camera/calibrate_intrinsic.py from-bag --bag camera_bag/ --topic /camera/color/image_raw --out tools/calibration/camera/output/camera_calib.yaml

Output format is OpenCV-compatible YAML, directly loadable by
CameraIntrinsics.from_yaml() and robot_config.yaml.

Factory calibration workflow:
    1. Print a checkerboard pattern (9x6 inner corners recommended)
    2. Run `calibrate_camera.py auto` on the robot
    3. Move the board to ~20 different poses covering the full FOV
    4. Script saves calibration to config/camera_calib.yaml
    5. Copy fx/fy/cx/cy/dist_* values into robot_config.yaml camera section
"""

import argparse
import glob
import logging
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

# Default checkerboard parameters (inner corners)
DEFAULT_BOARD_COLS = 9
DEFAULT_BOARD_ROWS = 6
DEFAULT_SQUARE_SIZE = 0.025  # 25mm squares


def find_checkerboard(
    image: np.ndarray,
    board_size: tuple,
    refine: bool = True,
) -> tuple:
    """Detect checkerboard corners in an image.

    Returns (found, corners) where corners is (N, 1, 2) float32.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)

    if found and refine:
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    return found, corners


def make_object_points(board_size: tuple, square_size: float) -> np.ndarray:
    """Generate 3D object points for a checkerboard on the Z=0 plane."""
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def calibrate_from_images(
    image_paths: list,
    board_size: tuple,
    square_size: float,
) -> dict:
    """Run OpenCV camera calibration on a set of checkerboard images.

    Returns dict with: K, D, width, height, rms, num_images, per_view_errors.
    """
    objp = make_object_points(board_size, square_size)
    obj_points = []
    img_points = []
    img_size = None
    used_images = []

    for path in sorted(image_paths):
        img = cv2.imread(str(path))
        if img is None:
            logger.warning("  Skipping unreadable: %s", path)
            continue

        if img_size is None:
            img_size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != img_size:
            logger.warning("  Skipping size mismatch: %s", path)
            continue

        found, corners = find_checkerboard(img, board_size)
        if found:
            obj_points.append(objp)
            img_points.append(corners)
            used_images.append(path)
            logger.info("  [OK] %s", os.path.basename(path))
        else:
            logger.info("  [--] %s (no board found)", os.path.basename(path))

    if len(obj_points) < 5:
        raise RuntimeError(
            f"Only {len(obj_points)} valid images found, need at least 5. "
            "Capture more images with the checkerboard visible."
        )

    logger.info("\nCalibrating with %d images...", len(obj_points))
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)

    # Per-view reprojection errors
    per_view = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, D)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
        per_view.append(float(err))

    return {
        "K": K,
        "D": D.flatten(),
        "width": img_size[0],
        "height": img_size[1],
        "rms": float(rms),
        "num_images": len(obj_points),
        "per_view_errors": per_view,
        "used_images": [str(p) for p in used_images],
    }


def save_calibration_yaml(result: dict, path: str) -> None:
    """Save calibration result as OpenCV-compatible YAML."""
    K = result["K"]
    D = result["D"]
    data = {
        "image_width": result["width"],
        "image_height": result["height"],
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": K.flatten().tolist(),
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(D),
            "data": D.tolist(),
        },
        "rms_reprojection_error": result["rms"],
        "num_calibration_images": result["num_images"],
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    logger.info("Calibration saved to %s", path)

    # Print robot_config.yaml snippet
    logger.info("\n── Copy into config/robots/unitree/go2/robot.yaml camera section ──")
    logger.info("  fx: %.1f", K[0, 0])
    logger.info("  fy: %.1f", K[1, 1])
    logger.info("  cx: %.1f", K[0, 2])
    logger.info("  cy: %.1f", K[1, 2])
    logger.info("  width: %d", result["width"])
    logger.info("  height: %d", result["height"])
    logger.info("  dist_k1: %.6f", D[0])
    logger.info("  dist_k2: %.6f", D[1])
    logger.info("  dist_p1: %.6f", D[2])
    logger.info("  dist_p2: %.6f", D[3])
    logger.info("  dist_k3: %.6f", D[4] if len(D) > 4 else 0.0)


def cmd_capture(args):
    """Interactive image capture from camera."""
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        logger.error("Cannot open camera device %s", args.device)
        sys.exit(1)

    os.makedirs(args.out, exist_ok=True)
    board_size = (args.cols, args.rows)
    count = 0

    logger.info("Press SPACE to capture, Q to quit. Need 15-25 images.")
    logger.info("Move the checkerboard to cover all corners of the image.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()
        found, corners = find_checkerboard(frame, board_size, refine=False)
        if found:
            cv2.drawChessboardCorners(display, board_size, corners, found)

        cv2.putText(
            display,
            f"Captured: {count}  ({'Board OK' if found else 'No board'})",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if found else (0, 0, 255),
            2,
        )
        cv2.imshow("Calibration Capture", display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" ") and found:
            path = os.path.join(args.out, f"calib_{count:03d}.png")
            cv2.imwrite(path, frame)
            count += 1
            logger.info("Saved %s", path)

    cap.release()
    cv2.destroyAllWindows()
    logger.info("Captured %d images to %s", count, args.out)


def cmd_calibrate(args):
    """Calibrate from a directory of images."""
    patterns = ["*.png", "*.jpg", "*.jpeg", "*.bmp"]
    paths = []
    for pat in patterns:
        paths.extend(glob.glob(os.path.join(args.images, pat)))

    if not paths:
        logger.error("No images found in %s", args.images)
        sys.exit(1)

    logger.info("Found %d images in %s", len(paths), args.images)
    board_size = (args.cols, args.rows)

    result = calibrate_from_images(paths, board_size, args.square_size)

    logger.info("\n── Calibration Result ──")
    logger.info("  RMS reprojection error: %.4f px", result["rms"])
    logger.info("  fx=%.1f  fy=%.1f", result["K"][0, 0], result["K"][1, 1])
    logger.info("  cx=%.1f  cy=%.1f", result["K"][0, 2], result["K"][1, 2])
    logger.info("  Distortion: %s", np.array2string(result["D"], precision=6))
    logger.info("  Images used: %d / %d", result["num_images"], len(paths))

    if result["rms"] > 1.0:
        logger.warning("  ⚠ RMS > 1.0px — consider re-capturing with better coverage")
    elif result["rms"] < 0.1:
        logger.warning("  ⚠ RMS suspiciously low — check board dimensions")
    else:
        logger.info("  ✓ Good calibration quality")

    save_calibration_yaml(result, args.out)


def cmd_auto(args):
    """One-shot: interactive capture then calibrate."""
    import tempfile

    img_dir = tempfile.mkdtemp(prefix="calib_")
    args.out = img_dir
    cmd_capture(args)

    # Check enough images
    images = glob.glob(os.path.join(img_dir, "*.png"))
    if len(images) < 5:
        logger.error("Not enough images captured (%d). Need at least 5.", len(images))
        sys.exit(1)

    args.images = img_dir
    args.out = args.calib_out
    cmd_calibrate(args)


def cmd_verify(args):
    """Verify calibration by showing undistorted live feed."""
    # Load calibration
    with open(args.calib) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(data["distortion_coefficients"]["data"])

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        logger.error("Cannot open camera device %s", args.device)
        sys.exit(1)

    ret, frame = cap.read()
    h, w = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

    logger.info("Press Q to quit. Left=original, Right=undistorted")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted = cv2.undistort(frame, K, D, None, new_K)

        # Crop to ROI
        x, y, rw, rh = roi
        if rw > 0 and rh > 0:
            undistorted = undistorted[y : y + rh, x : x + rw]

        # Side by side
        h1, _w1 = frame.shape[:2]
        h2, w2 = undistorted.shape[:2]
        scale = h1 / h2 if h2 > 0 else 1.0
        undist_resized = cv2.resize(undistorted, (int(w2 * scale), h1))

        combined = np.hstack([frame, undist_resized])
        cv2.imshow("Original | Undistorted", combined)

        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def cmd_from_bag(args):
    """Extract checkerboard images from a ROS2 bag and calibrate (headless)."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from sensor_msgs.msg import Image as RosImage
    except ImportError:
        logger.error(
            "ROS2 rosbag2_py not available. Install with:\n"
            "  sudo apt install ros-humble-rosbag2-default-plugins\n"
            "Or run inside a sourced ROS2 environment."
        )
        sys.exit(1)

    storage = StorageOptions(uri=args.bag, storage_id="mcap")
    converter = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage, converter)

    # Filter to image topic
    from rosbag2_py import StorageFilter

    reader.set_filter(StorageFilter(topics=[args.topic]))

    board_size = (args.cols, args.rows)
    img_dir = os.path.join(os.path.dirname(args.out), "extracted_frames")
    os.makedirs(img_dir, exist_ok=True)

    count = 0
    saved = 0
    sample_interval = max(1, args.skip)

    logger.info("Extracting frames from %s topic=%s ...", args.bag, args.topic)

    while reader.has_next():
        _topic, data, _ts = reader.read_next()
        count += 1
        if count % sample_interval != 0:
            continue

        msg = deserialize_message(data, RosImage)
        # Convert ROS Image to OpenCV
        if msg.encoding in ("rgb8", "bgr8"):
            dtype = np.uint8
            channels = 3
        elif msg.encoding == "mono8":
            dtype = np.uint8
            channels = 1
        else:
            continue

        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Check for checkerboard
        found, _corners = find_checkerboard(img, board_size)
        if found:
            path = os.path.join(img_dir, f"bag_{saved:03d}.png")
            cv2.imwrite(path, img)
            saved += 1
            if saved % 5 == 0:
                logger.info("  Found %d boards in %d frames...", saved, count)

    logger.info("Extracted %d checkerboard frames from %d total", saved, count)

    if saved < 5:
        logger.error("Not enough checkerboard frames (%d). Need at least 5.", saved)
        sys.exit(1)

    # Run calibration on extracted images
    patterns = ["*.png"]
    paths = []
    for pat in patterns:
        paths.extend(glob.glob(os.path.join(img_dir, pat)))

    result = calibrate_from_images(paths, board_size, args.square_size)

    logger.info("\n── Calibration Result ──")
    logger.info("  RMS reprojection error: %.4f px", result["rms"])
    logger.info("  fx=%.1f  fy=%.1f", result["K"][0, 0], result["K"][1, 1])
    logger.info("  cx=%.1f  cy=%.1f", result["K"][0, 2], result["K"][1, 2])
    logger.info("  Distortion: %s", np.array2string(result["D"], precision=6))

    save_calibration_yaml(result, args.out)


def main():
    parser = argparse.ArgumentParser(
        description="Camera calibration tool (OpenCV checkerboard)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--cols", type=int, default=DEFAULT_BOARD_COLS, help="Checkerboard inner corners (columns)")
    parser.add_argument("--rows", type=int, default=DEFAULT_BOARD_ROWS, help="Checkerboard inner corners (rows)")
    parser.add_argument(
        "--square-size", type=float, default=DEFAULT_SQUARE_SIZE, help="Checkerboard square size in meters"
    )

    sub = parser.add_subparsers(dest="command")

    # capture
    p_cap = sub.add_parser("capture", help="Capture checkerboard images from camera")
    p_cap.add_argument("--device", type=int, default=0, help="Camera device index")
    p_cap.add_argument("--out", default="calibration_imgs/", help="Output directory")

    # calibrate
    p_cal = sub.add_parser("calibrate", help="Calibrate from saved images")
    p_cal.add_argument("--images", required=True, help="Directory with checkerboard images")
    p_cal.add_argument("--out", default="tools/calibration/camera/output/camera_calib.yaml", help="Output YAML")

    # auto
    p_auto = sub.add_parser("auto", help="Interactive capture + calibrate")
    p_auto.add_argument("--device", type=int, default=0, help="Camera device index")
    p_auto.add_argument(
        "--calib-out", default="tools/calibration/camera/output/camera_calib.yaml", help="Output calibration YAML"
    )

    # from-bag (headless, for S100P)
    p_bag = sub.add_parser("from-bag", help="Extract from ROS2 bag + calibrate (headless)")
    p_bag.add_argument("--bag", required=True, help="Path to ROS2 bag directory")
    p_bag.add_argument("--topic", default="/camera/color/image_raw", help="Image topic in bag")
    p_bag.add_argument("--skip", type=int, default=10, help="Process every Nth frame (default: 10)")
    p_bag.add_argument("--out", default="tools/calibration/camera/output/camera_calib.yaml", help="Output YAML")

    # verify
    p_ver = sub.add_parser("verify", help="Verify calibration with live undistortion")
    p_ver.add_argument("--calib", required=True, help="Calibration YAML file")
    p_ver.add_argument("--device", type=int, default=0, help="Camera device index")

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        sys.exit(0)

    {
        "capture": cmd_capture,
        "calibrate": cmd_calibrate,
        "auto": cmd_auto,
        "from-bag": cmd_from_bag,
        "verify": cmd_verify,
    }[args.command](args)


if __name__ == "__main__":
    main()
