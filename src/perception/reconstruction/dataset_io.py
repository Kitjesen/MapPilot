"""dataset_io.py — 读取 / 导出本地 RGB-D 关键帧数据集。

支持的输入格式:
    transforms_dir  含 transforms.json 的目录（DatasetRecorderModule 输出）
    tum_dir         TUM RGB-D 格式（rgb/ depth/ associations.txt）
    colmap_dir      COLMAP sparse 模型目录（cameras.txt / images.txt）

支持的输出格式（re-export）:
    nerfstudio      transforms.json（已是默认格式）
    tum             rgb/ depth/ associations.txt  （GSFusion / RTAB-Map 通用）
    colmap          cameras.txt / images.txt / points3D.txt

工具函数:
    load_dataset(path)          → list[Keyframe]
    export_nerfstudio(kfs, dir) → Path
    export_tum(kfs, dir)        → Path
    dataset_stats(kfs)          → dict
"""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path
from typing import Any

import numpy as np

from .server.backends.base import Keyframe

# ── 加载函数 ─────────────────────────────────────────────────────────────────


def load_dataset(path: str | Path) -> list[Keyframe]:
    """从磁盘加载关键帧数据集，自动检测格式。

    参数
    ----
    path : str | Path
        目录路径，含 transforms.json（recorder 格式）、
        或 rgb/ + depth/ + associations.txt（TUM 格式）。

    返回
    ----
    list[Keyframe]  按 frame_idx 排序
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Dataset path not found: {p}")

    if (p / "transforms.json").exists():
        return _load_transforms_json(p)
    elif (p / "associations.txt").exists():
        return _load_tum(p)
    elif (p / "images.txt").exists():
        return _load_colmap(p)
    else:
        raise ValueError(
            f"Cannot detect dataset format at {p}. "
            f"Expected transforms.json / associations.txt / images.txt"
        )


def _load_transforms_json(base: Path) -> list[Keyframe]:
    """加载 DatasetRecorderModule / nerfstudio 格式数据集。"""
    with open(base / "transforms.json", encoding="utf-8") as f:
        data = json.load(f)

    fx  = float(data.get("fl_x", 615.0))
    fy  = float(data.get("fl_y", 615.0))
    cx  = float(data.get("cx",   320.0))
    cy  = float(data.get("cy",   240.0))
    w   = int(data.get("w",   640))
    h   = int(data.get("h",   480))

    keyframes: list[Keyframe] = []
    for i, frame in enumerate(data.get("frames", [])):
        color_rel = frame.get("file_path", "")
        depth_rel = frame.get("depth_file_path", "")
        color_path = base / color_rel
        depth_path = base / depth_rel if depth_rel else Path("/dev/null")

        # transforms.json 存 OpenGL → 转回 OpenCV
        T_gl = np.array(frame["transform_matrix"], dtype=np.float64)
        T_cv = T_gl @ np.diag([1., -1., -1., 1.])

        kf = Keyframe(
            frame_idx=i,
            timestamp=float(frame.get("timestamp", float(i))),
            pose=T_cv,
            fx=fx, fy=fy, cx=cx, cy=cy,
            width=w, height=h,
            color_path=color_path,
            depth_path=depth_path,
        )

        # 每帧可能有独立内参
        if "fl_x" in frame:
            kf.fx = float(frame["fl_x"])
            kf.fy = float(frame.get("fl_y", frame["fl_x"]))
            kf.cx = float(frame.get("cx", cx))
            kf.cy = float(frame.get("cy", cy))
            kf.width  = int(frame.get("w", w))
            kf.height = int(frame.get("h", h))

        if color_path.exists():
            keyframes.append(kf)

    return keyframes


def _load_tum(base: Path) -> list[Keyframe]:
    """加载 TUM RGB-D 格式数据集。

    associations.txt 格式：
        <ts_rgb> rgb/<ts_rgb>.png <ts_depth> depth/<ts_depth>.png
    poses.txt 格式（可选）：
        <ts> tx ty tz qx qy qz qw
    camera_info.json 格式（可选）：
        {"fx":..., "fy":..., "cx":..., "cy":..., "width":..., "height":...}
    """
    # 读取内参
    fx, fy, cx, cy, w, h = 615.0, 615.0, 320.0, 240.0, 640, 480
    cam_info_path = base / "camera_info.json"
    if cam_info_path.exists():
        cam = json.loads(cam_info_path.read_text())
        fx, fy = float(cam.get("fx", fx)), float(cam.get("fy", fy))
        cx, cy = float(cam.get("cx", cx)), float(cam.get("cy", cy))
        w,  h  = int(cam.get("width", w)), int(cam.get("height", h))

    # 读取位姿（可选）
    poses: dict[str, np.ndarray] = {}
    poses_path = base / "poses.txt"
    if poses_path.exists():
        for line in poses_path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                ts = parts[0]
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = (float(parts[4]), float(parts[5]),
                                   float(parts[6]), float(parts[7]))
                poses[ts] = _quat_to_matrix(tx, ty, tz, qx, qy, qz, qw)

    # 读取 associations.txt
    keyframes: list[Keyframe] = []
    for i, line in enumerate(
        (base / "associations.txt").read_text().splitlines()
    ):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        ts_rgb   = parts[0]
        rgb_rel  = parts[1]
        parts[2]
        dep_rel  = parts[3]

        color_path = base / rgb_rel
        depth_path = base / dep_rel
        if not color_path.exists():
            continue

        pose = poses.get(ts_rgb, np.eye(4, dtype=np.float64))
        keyframes.append(Keyframe(
            frame_idx=i,
            timestamp=float(ts_rgb),
            pose=pose,
            fx=fx, fy=fy, cx=cx, cy=cy,
            width=w, height=h,
            color_path=color_path,
            depth_path=depth_path,
        ))

    return keyframes


def _load_colmap(base: Path) -> list[Keyframe]:
    """加载 COLMAP sparse 重建目录（images.txt + cameras.txt）。

    仅加载有 pose 的图像（已标定帧），深度图若无则跳过（返回 Keyframe 但
    depth_path=/dev/null），供 NeRF/GS 单目重建使用。
    """
    # 读取 cameras.txt
    cameras: dict[int, dict] = {}
    cam_txt = base / "cameras.txt"
    if cam_txt.exists():
        for line in cam_txt.read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            cam_id = int(parts[0])
            model  = parts[1]
            width  = int(parts[2])
            height = int(parts[3])
            params = [float(x) for x in parts[4:]]
            # SIMPLE_RADIAL: f, cx, cy, k
            # PINHOLE:       fx, fy, cx, cy
            if model == "SIMPLE_RADIAL" and len(params) >= 3:
                fx = fy = params[0]
                cx = params[1]
                cy = params[2]
            elif model == "PINHOLE" and len(params) >= 4:
                fx, fy, cx, cy = params[0], params[1], params[2], params[3]
            else:
                fx = fy = width * 0.9
                cx = width / 2
                cy = height / 2
            cameras[cam_id] = {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                                "w": width, "h": height}

    # 读取 images.txt（每帧两行：pose + 点列表）
    images_dir = base / "images"
    keyframes: list[Keyframe] = []
    idx = 0
    lines = (base / "images.txt").read_text().splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if line.startswith("#") or not line:
            continue
        parts = line.split()
        if len(parts) < 9:
            i += 1  # skip point line
            continue

        qw, qx, qy, qz = (float(parts[1]), float(parts[2]),
                           float(parts[3]), float(parts[4]))
        tx, ty, tz = float(parts[5]), float(parts[6]), float(parts[7])
        cam_id = int(parts[8])
        img_name = parts[9] if len(parts) > 9 else ""

        # COLMAP: T_world_cam = T_cam_world^{-1}
        R_cw = _quat_to_R(qx, qy, qz, qw)
        t_cw = np.array([tx, ty, tz])
        T_cw = np.eye(4)
        T_cw[:3, :3] = R_cw
        T_cw[:3, 3]  = t_cw
        T_wc = np.linalg.inv(T_cw)  # camera-to-world

        cam = cameras.get(cam_id, {"fx": 615, "fy": 615, "cx": 320, "cy": 240,
                                    "w": 640, "h": 480})
        color_path = images_dir / img_name
        depth_path = base / "depths" / img_name.replace(".jpg", ".png") \
                                               .replace(".JPG", ".png")

        if color_path.exists():
            keyframes.append(Keyframe(
                frame_idx=idx,
                timestamp=float(idx),
                pose=T_wc,
                fx=cam["fx"], fy=cam["fy"],
                cx=cam["cx"], cy=cam["cy"],
                width=cam["w"], height=cam["h"],
                color_path=color_path,
                depth_path=depth_path if depth_path.exists() else Path("/dev/null"),
            ))
            idx += 1

        i += 1  # skip 2D point list line

    return keyframes


# ── 导出函数 ─────────────────────────────────────────────────────────────────


def export_nerfstudio(keyframes: list[Keyframe], output_dir: Path) -> Path:
    """将 Keyframe 列表导出为 nerfstudio transforms.json 格式。

    图片和深度图会被复制到 output_dir/images/ 和 output_dir/depths/。
    """
    output_dir = Path(output_dir)
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "depths").mkdir(exist_ok=True)

    frames = []
    kf0 = keyframes[0] if keyframes else None

    for kf in keyframes:
        img_dst   = output_dir / "images" / f"{kf.frame_idx:06d}.jpg"
        depth_dst = output_dir / "depths" / f"{kf.frame_idx:06d}.png"
        if kf.color_path != img_dst and kf.color_path.exists():
            shutil.copy2(kf.color_path, img_dst)
        if kf.depth_path.exists() and kf.depth_path != depth_dst:
            shutil.copy2(kf.depth_path, depth_dst)

        T_gl = kf.pose @ np.diag([1., -1., -1., 1.])
        entry: dict = {
            "file_path":        f"images/{kf.frame_idx:06d}.jpg",
            "depth_file_path":  f"depths/{kf.frame_idx:06d}.png",
            "transform_matrix": T_gl.tolist(),
            "timestamp": kf.timestamp,
        }
        frames.append(entry)

    transforms = {
        "camera_model": "OPENCV",
        "fl_x": kf0.fx if kf0 else 615.0,
        "fl_y": kf0.fy if kf0 else 615.0,
        "cx":   kf0.cx if kf0 else 320.0,
        "cy":   kf0.cy if kf0 else 240.0,
        "w":    kf0.width  if kf0 else 640,
        "h":    kf0.height if kf0 else 480,
        "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
        "depth_unit_scale_factor": 0.001,
        "frames": frames,
    }
    dst = output_dir / "transforms.json"
    dst.write_text(json.dumps(transforms, indent=2))
    return dst


def export_tum(keyframes: list[Keyframe], output_dir: Path) -> Path:
    """将 Keyframe 列表导出为 TUM RGB-D 格式。

    适用于 RTAB-Map、GSFusion、MonST3R 等工具。
    """
    output_dir = Path(output_dir)
    (output_dir / "rgb").mkdir(parents=True, exist_ok=True)
    (output_dir / "depth").mkdir(exist_ok=True)

    assoc_lines: list[str] = []
    pose_lines:  list[str] = []

    for kf in keyframes:
        ts = f"{kf.timestamp:.6f}"
        rgb_dst   = output_dir / "rgb"   / f"{ts}.jpg"
        depth_dst = output_dir / "depth" / f"{ts}.png"

        if kf.color_path.exists():
            shutil.copy2(kf.color_path, rgb_dst)
        if kf.depth_path.exists() and str(kf.depth_path) != "/dev/null":
            shutil.copy2(kf.depth_path, depth_dst)
            assoc_lines.append(f"{ts} rgb/{ts}.jpg {ts} depth/{ts}.png")
        else:
            assoc_lines.append(f"{ts} rgb/{ts}.jpg {ts} depth/{ts}.png")

        R  = kf.pose[:3, :3]
        t  = kf.pose[:3, 3]
        q  = _rotation_matrix_to_quat(R)
        pose_lines.append(
            f"{ts} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
            f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
        )

    (output_dir / "associations.txt").write_text("\n".join(assoc_lines))
    (output_dir / "poses.txt").write_text("\n".join(pose_lines))

    if keyframes:
        kf0 = keyframes[0]
        cam = {"fx": kf0.fx, "fy": kf0.fy, "cx": kf0.cx, "cy": kf0.cy,
               "width": kf0.width, "height": kf0.height, "depth_scale": 1000.0}
        (output_dir / "camera_info.json").write_text(json.dumps(cam, indent=2))

    return output_dir / "associations.txt"


def dataset_stats(keyframes: list[Keyframe]) -> dict[str, Any]:
    """返回数据集基本统计信息。"""
    if not keyframes:
        return {"frames": 0}
    ts = [kf.timestamp for kf in keyframes]
    poses = [kf.pose[:3, 3] for kf in keyframes]
    traj_len = sum(
        float(np.linalg.norm(poses[i] - poses[i - 1]))
        for i in range(1, len(poses))
    )
    return {
        "frames":        len(keyframes),
        "duration_s":    ts[-1] - ts[0] if len(ts) > 1 else 0.0,
        "trajectory_m":  round(traj_len, 2),
        "first_ts":      ts[0],
        "last_ts":       ts[-1],
        "image_size":    f"{keyframes[0].width}×{keyframes[0].height}",
        "has_depth":     any(
            kf.depth_path.exists() and str(kf.depth_path) != "/dev/null"
            for kf in keyframes
        ),
    }


# ── 几何工具 ─────────────────────────────────────────────────────────────────

def _quat_to_matrix(tx, ty, tz, qx, qy, qz, qw) -> np.ndarray:
    R = _quat_to_R(qx, qy, qz, qw)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = [tx, ty, tz]
    return T


def _quat_to_R(qx, qy, qz, qw) -> np.ndarray:
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def _rotation_matrix_to_quat(R: np.ndarray) -> tuple:
    """3×3 旋转矩阵 → (qx, qy, qz, qw)。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return ((R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s,
                (R[1,0]-R[0,1])*s, 0.25/s)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return (0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s)
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return ((R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s)
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return ((R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s)
