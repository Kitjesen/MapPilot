"""
Laplacian 方差模糊检测 — 源自 LOVON 项目。

四足机器人步态振动导致大量模糊帧, 检测这些帧并跳过,
只处理清晰帧以提高检测准确率。
"""

import cv2
import numpy as np


def compute_laplacian_variance(gray: np.ndarray) -> float:
    """
    计算灰度图的 Laplacian 方差 (模糊度度量)。

    Args:
        gray: HxW uint8 灰度图

    Returns:
        Laplacian 方差值。值越高图像越清晰。
    """
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    return float(laplacian.var())


def is_blurry(image: np.ndarray, threshold: float = 100.0) -> bool:
    """
    判断图像是否模糊。

    Args:
        image: HxWx3 BGR 或 HxW 灰度图
        threshold: Laplacian 方差阈值, 低于此值判定为模糊

    Returns:
        True = 模糊, False = 清晰
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    variance = compute_laplacian_variance(gray)
    return variance < threshold
