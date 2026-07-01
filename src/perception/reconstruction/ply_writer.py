"""
ply_writer.py — 零依赖 PLY 点云写入（binary little-endian）

不依赖 open3d，纯 numpy + struct，适合 aarch64 部署。
"""

import os

import numpy as np


def save_ply(
    points_xyz: np.ndarray,          # (N, 3) float32
    colors_rgb: np.ndarray | None = None,  # (N, 3) uint8，可选
    labels: list[str] | None = None,        # (N,) str，可选，写入 comment
    filepath: str = "output.ply",
) -> int:
    """
    保存点云为 PLY 文件（binary little-endian）。

    Returns:
        写入点数
    """
    n = len(points_xyz)
    if n == 0:
        return 0

    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)

    has_color = colors_rgb is not None and len(colors_rgb) == n

    # ── Header ──
    header_lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"comment LingTu 三维重建 — {n} points",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if has_color:
        header_lines += [
            "property uchar red",
            "property uchar green",
            "property uchar blue",
        ]
    header_lines.append("end_header")
    header = "\n".join(header_lines) + "\n"

    # ── Binary data ──
    xyz = points_xyz.astype(np.float32)

    if has_color:
        rgb = colors_rgb.astype(np.uint8)
        # 每点: 3×float32 (12字节) + 3×uint8 (3字节) = 15字节
        buf = np.zeros(n, dtype=[
            ("x", np.float32), ("y", np.float32), ("z", np.float32),
            ("r", np.uint8), ("g", np.uint8), ("b", np.uint8),
        ])
        buf["x"] = xyz[:, 0]
        buf["y"] = xyz[:, 1]
        buf["z"] = xyz[:, 2]
        buf["r"] = rgb[:, 0]
        buf["g"] = rgb[:, 1]
        buf["b"] = rgb[:, 2]
    else:
        buf = np.zeros(n, dtype=[
            ("x", np.float32), ("y", np.float32), ("z", np.float32),
        ])
        buf["x"] = xyz[:, 0]
        buf["y"] = xyz[:, 1]
        buf["z"] = xyz[:, 2]

    with open(filepath, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(buf.tobytes())

    return n


def save_ply_with_labels(
    xyzrgb: np.ndarray,        # (N, 6) float32: xyz + rgb
    labels: list[str],
    filepath: str,
) -> int:
    """保存带颜色的点云，同时在同目录输出 labels.txt。"""
    n = len(xyzrgb)
    if n == 0:
        return 0

    written = save_ply(
        points_xyz=xyzrgb[:, :3],
        colors_rgb=xyzrgb[:, 3:6].astype(np.uint8),
        filepath=filepath,
    )

    label_path = filepath.replace(".ply", "_labels.txt")
    with open(label_path, "w", encoding="utf-8") as f:
        f.write("\n".join(labels))

    return written
