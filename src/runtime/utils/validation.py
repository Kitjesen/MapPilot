"""
validation.py — 图像 / 深度 / 内参 / 四元数校验器

Canonical location: runtime.utils.validation

用法:
    from runtime.utils.validation import validate_bgr, validate_depth, validate_intrinsics

    bgr = validate_bgr(raw_img)        # None if invalid, auto-converts BGRA→BGR
    depth = validate_depth(raw_depth)    # None if invalid
    ok = validate_intrinsics(K, w, h)   # False if bad intrinsics
"""

import logging
import math
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


# ──────────────────────────────────────────────
#  图像校验
# ──────────────────────────────────────────────

def validate_bgr(
    img: np.ndarray,
    *,
    allow_bgra: bool = True,
    caller: str = "",
) -> np.ndarray | None:
    """校验 BGR 图像，自动转换 BGRA→BGR。

    Returns:
        有效的 BGR (H,W,3) 图像，或 None。
    """
    tag = f"[{caller}] " if caller else ""

    if img is None:
        logger.error("%sBGR image is None", tag)
        return None

    if img.ndim != 3:
        logger.error("%sExpected 3D image, got ndim=%d", tag, img.ndim)
        return None

    channels = img.shape[2]
    if channels == 3:
        return img
    if channels == 4 and allow_bgra:
        try:
            import cv2

            logger.debug("%sConverting BGRA→BGR", tag)
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        except ImportError:
            # 手动去除 alpha 通道
            return img[:, :, :3].copy()
    logger.error("%sExpected 3 channels, got %d", tag, channels)
    return None


def validate_depth(
    img: np.ndarray,
    *,
    caller: str = "",
) -> np.ndarray | None:
    """校验深度图 (uint16 或 float32, 2D)。

    Returns:
        有效的 2D 深度图，或 None。
    """
    tag = f"[{caller}] " if caller else ""

    if img is None:
        logger.error("%sDepth image is None", tag)
        return None

    if img.ndim != 2:
        # 3D depth (如 RGB 编码 depth) 不处理
        logger.error("%sExpected 2D depth, got shape %s", tag, img.shape)
        return None

    if img.size == 0:
        logger.error("%sDepth image is empty", tag)
        return None

    return img


_depth_pair_warned: set = set()  # per-caller warn-once tracking


def validate_depth_pair(
    bgr: np.ndarray,
    depth: np.ndarray,
    *,
    resize_depth: bool = True,
    caller: str = "",
) -> np.ndarray | None:
    """校验 BGR 和 depth 尺寸对齐，必要时 resize depth。

    Returns:
        对齐后的 depth，或 None。
    """
    tag = f"[{caller}] " if caller else ""

    if bgr.shape[:2] == depth.shape[:2]:
        return depth

    # Warn once per caller (not once globally — avoids test interference)
    if caller not in _depth_pair_warned:
        logger.warning(
            "%sDepth %s != BGR %s, resizing depth (subsequent warnings suppressed)",
            tag,
            depth.shape[:2],
            bgr.shape[:2],
        )
        _depth_pair_warned.add(caller)

    if not resize_depth:
        return None

    try:
        import cv2

        return cv2.resize(
            depth,
            (bgr.shape[1], bgr.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
    except ImportError:
        logger.error("%scv2 not available for depth resize", tag)
        return None


# ──────────────────────────────────────────────
#  相机内参校验
# ──────────────────────────────────────────────

@dataclass
class IntrinsicsResult:
    """校验后的相机内参。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    valid: bool = True


def validate_intrinsics(
    K,
    width: int,
    height: int,
    *,
    caller: str = "",
) -> IntrinsicsResult | None:
    """校验相机内参矩阵 K (长度 ≥ 6)。

    Args:
        K: 内参数组 [fx, 0, cx, 0, fy, cy, ...]
        width: 图像宽度
        height: 图像高度

    Returns:
        IntrinsicsResult 或 None (无效时)。
    """
    tag = f"[{caller}] " if caller else ""

    if K is None or len(K) < 6:
        logger.error(
            "%sCameraInfo K too short: %d, expected >= 6",
            tag,
            0 if K is None else len(K),
        )
        return None

    fx, fy = float(K[0]), float(K[4])
    cx, cy = float(K[2]), float(K[5])

    if fx <= 0.0 or fy <= 0.0:
        logger.warning("%sInvalid focal length: fx=%.2f, fy=%.2f", tag, fx, fy)
        return None

    if cx < 0 or cy < 0 or cx >= width or cy >= height:
        logger.warning(
            "%sPrincipal point out of bounds: cx=%.1f, cy=%.1f (img %dx%d)",
            tag,
            cx,
            cy,
            width,
            height,
        )
        return None

    return IntrinsicsResult(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)


# ──────────────────────────────────────────────
#  四元数校验 + 归一化
# ──────────────────────────────────────────────

def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
    *,
    epsilon: float = 1e-6,
) -> tuple[float, float, float, float] | None:
    """归一化四元数，返回 (x, y, z, w) 或 None。

    Examples::

        >>> normalize_quaternion(0.0, 0.0, 0.0, 1.0)
        (0.0, 0.0, 0.0, 1.0)
        >>> q = normalize_quaternion(0.0, 0.0, 0.1, 0.995)
        >>> abs(sum(c*c for c in q) - 1.0) < 1e-12
        True
        >>> normalize_quaternion(0.0, 0.0, 0.0, 0.0) is None
        True
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < epsilon:
        logger.warning("Quaternion near-zero norm: %.2e", norm)
        return None
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)
