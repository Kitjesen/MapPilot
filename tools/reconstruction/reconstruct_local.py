#!/usr/bin/env python3
"""reconstruct_local.py — 本地离线三维重建工具（无需服务器）。

从以下输入之一加载 RGB-D 数据并在本机运行重建：
  1. 录制数据集目录（DatasetRecorderModule 输出，含 transforms.json）
  2. ROS2 bag（.db3 文件或含 metadata.yaml 的目录）
  3. 点云 bag（直接合并 LiDAR 点云为 PLY）

用法示例:
  # 从 recorder 录制的数据集重建（TSDF，CPU）
  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000

  # 从 ROS2 bag 提取帧后重建
  python tools/reconstruction/reconstruct_local.py --bag ~/data/run1/ --backend tsdf

  # 直接从点云 bag 生成 PLY（最简单）
  python tools/reconstruction/reconstruct_local.py --bag ~/data/run1/ --lidar

  # 用 NeRF（需要 GPU + nerfstudio）
  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000 \\
      --backend nerfstudio --method instant-ngp

  # 用 3D Gaussian Splatting
  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000 \\
      --backend nerfstudio --method splatfacto

  # 列出所有可用后端及其依赖状态
  python tools/reconstruction/reconstruct_local.py --list-backends

  # 将数据集转换为 TUM 格式（用于 RTAB-Map / GSFusion）
  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000 \\
      --export-tum /tmp/my_tum_dataset

输出:
  outputs/reconstruction/<session>/
  ├── reconstruction.ply    (TSDF / Open3D)
  ├── mesh.obj              (Open3D，可选)
  └── ns_output/            (nerfstudio checkpoint)
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path

# ── sys.path 设置 ─────────────────────────────────────────────────────────────
def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


_REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)
_SRC_DIR   = _REPO_ROOT / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))


def setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s — %(message)s",
        datefmt="%H:%M:%S",
    )


def cmd_reconstruct(args) -> int:
    """从数据集目录运行重建。"""
    from semantic.reconstruction.dataset_io import (
        load_dataset, dataset_stats, export_tum
    )
    from semantic.reconstruction.server.backends.registry import get_backend, list_backends

    dataset_path = Path(args.dataset).expanduser()
    if not dataset_path.exists():
        print(f"[ERROR] Dataset not found: {dataset_path}")
        return 1

    print(f"Loading dataset: {dataset_path}")
    keyframes = load_dataset(dataset_path)
    if not keyframes:
        print("[ERROR] No keyframes found in dataset")
        return 1

    stats = dataset_stats(keyframes)
    print(f"  {stats['frames']} frames | "
          f"{stats['trajectory_m']} m trajectory | "
          f"{stats.get('duration_s', 0):.1f}s | "
          f"depth={'yes' if stats['has_depth'] else 'no'}")

    # 仅导出格式转换
    if args.export_tum:
        out = Path(args.export_tum).expanduser()
        export_tum(keyframes, out)
        print(f"Exported TUM dataset to: {out}")
        return 0

    # 重建
    backend_name = args.backend
    output_dir   = Path(args.output or
                        f"outputs/reconstruction/{dataset_path.name}").expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        backend_cls = get_backend(backend_name)
    except KeyError:
        avail = list_backends()
        print(f"[ERROR] Unknown backend '{backend_name}'. Available: {avail}")
        return 1

    backend = backend_cls()
    ok, reason = backend.check_dependencies()
    if not ok:
        print(f"[ERROR] Backend '{backend_name}' dependencies not met:\n  {reason}")
        return 1

    opts: dict = {}
    if args.method:
        opts["method"] = args.method
    if args.extract_mesh:
        opts["extract_mesh"] = True
    if args.voxel_size:
        opts["voxel_size"] = args.voxel_size

    print(f"\nRunning reconstruction: backend={backend_name} output={output_dir}")
    t0 = time.time()
    result = backend.reconstruct(keyframes, output_dir, **opts)
    elapsed = time.time() - t0

    if result.success:
        print(f"\n✓ Done in {elapsed:.1f}s")
        print(f"  Output:  {result.output_path}")
        print(f"  Format:  {result.output_format}")
        print(f"  Frames:  {result.num_frames}")
        if result.extra:
            for k, v in result.extra.items():
                print(f"  {k}: {v}")
        return 0
    else:
        print(f"\n✗ Reconstruction failed: {result.message}")
        return 1


def cmd_from_bag(args) -> int:
    """从 bag 提取帧后重建（或直接合并点云）。"""
    bag_path = Path(args.bag).expanduser()
    if not bag_path.exists():
        print(f"[ERROR] Bag not found: {bag_path}")
        return 1

    if args.lidar:
        # 直接从点云 bag 生成 PLY
        from compat.ros2.bag_reader import read_lidar_bag
        out_ply = Path(args.output or f"outputs/reconstruction/{bag_path.stem}_lidar.ply")
        out_ply.parent.mkdir(parents=True, exist_ok=True)
        print(f"Extracting LiDAR point cloud from {bag_path} → {out_ply}")
        try:
            result_path = read_lidar_bag(
                bag_path, out_ply,
                cloud_topic=args.cloud_topic,
                voxel_size=args.voxel_size or 0.05,
            )
            print(f"✓ Point cloud saved: {result_path}")
            return 0
        except Exception as exc:
            print(f"✗ Error: {exc}")
            return 1

    # RGB-D bag → 提取帧 → 重建
    from compat.ros2.bag_reader import read_rgb_d_bag

    frames_dir = Path(args.frames_dir or
                      f"outputs/reconstruction/{bag_path.stem}_frames").expanduser()
    frames_dir.mkdir(parents=True, exist_ok=True)

    print(f"Extracting RGB-D keyframes from {bag_path} → {frames_dir}")
    try:
        keyframes = read_rgb_d_bag(
            bag_path, frames_dir,
            color_topic=args.color_topic,
            depth_topic=args.depth_topic,
            odom_topic=args.odom_topic,
            keyframe_dist_m=args.kf_dist or 0.15,
            keyframe_rot_rad=args.kf_rot  or 0.17,
            keyframe_time_s=args.kf_time  or 1.0,
            max_depth_m=args.max_depth or 6.0,
            max_frames=args.max_frames or 0,
        )
    except Exception as exc:
        print(f"✗ Bag extraction failed: {exc}")
        return 1

    print(f"  Extracted {len(keyframes)} keyframes")

    if args.extract_only:
        print(f"✓ Frames saved to {frames_dir}")
        return 0

    # 委托 cmd_reconstruct
    args.dataset = str(frames_dir)
    return cmd_reconstruct(args)


def cmd_list_backends(args) -> int:
    from semantic.reconstruction.server.backends.registry import list_backends, get_backend
    print("Available reconstruction backends:\n")
    for name in list_backends():
        cls  = get_backend(name)
        inst = cls()
        ok, reason = inst.check_dependencies()
        mark = "✓" if ok else "✗"
        print(f"  {mark} {name:<20s}  {reason}")
    print()
    print("Install instructions:")
    print("  TSDF / Open3D:   pip install open3d")
    print("  Nerfstudio:      pip install nerfstudio   (needs CUDA GPU)")
    print("  GSFusion:        git clone https://github.com/ethz-mrl/GSFusion && make")
    print("                   export GSFUSION_BIN=/path/to/GSFusion/build/GSFusion")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="LingTu 本地三维重建工具",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--log-level", default="INFO",
                        help="日志级别 (DEBUG/INFO/WARNING)")

    sub = parser.add_subparsers(dest="command")

    # ── reconstruct（默认命令）─────────────────────────────────────────────
    rec = sub.add_parser("reconstruct", aliases=["r"],
                         help="从数据集目录重建（DatasetRecorderModule 输出）")
    rec.add_argument("--dataset", required=True,
                     help="含 transforms.json 的数据集目录")
    rec.add_argument("--backend", default="tsdf",
                     help="重建后端: tsdf / open3d / nerfstudio / gsfusion")
    rec.add_argument("--method", default="",
                     help="nerfstudio 方法: instant-ngp / nerfacto / splatfacto / depth-nerfacto")
    rec.add_argument("--output", default="",
                     help="输出目录（默认: outputs/reconstruction/<dataset_name>）")
    rec.add_argument("--voxel-size", type=float, dest="voxel_size", default=0.0,
                     help="TSDF/Open3D 体素大小（m），0=使用后端默认值")
    rec.add_argument("--extract-mesh", action="store_true", dest="extract_mesh",
                     help="同时提取三角网格（TSDF 后端）")
    rec.add_argument("--export-tum", metavar="OUT_DIR", dest="export_tum",
                     help="仅将数据集导出为 TUM 格式，不运行重建")

    # ── bag（从 bag 文件处理）──────────────────────────────────────────────
    bag = sub.add_parser("bag", aliases=["b"],
                         help="从 ROS2 bag 提取帧后重建（或直接合并点云）")
    bag.add_argument("--bag", required=True,
                     help="ROS2 bag 目录或 .db3 文件路径")
    bag.add_argument("--lidar", action="store_true",
                     help="直接从点云话题合并 PLY，不做图像重建")
    bag.add_argument("--backend", default="tsdf")
    bag.add_argument("--method", default="")
    bag.add_argument("--output", default="",
                     help="输出文件/目录")
    bag.add_argument("--frames-dir", default="", dest="frames_dir",
                     help="关键帧提取目录（默认: outputs/reconstruction/<bag_name>_frames）")
    bag.add_argument("--color-topic",  default="", dest="color_topic")
    bag.add_argument("--depth-topic",  default="", dest="depth_topic")
    bag.add_argument("--odom-topic",   default="", dest="odom_topic")
    bag.add_argument("--cloud-topic",  default="", dest="cloud_topic")
    bag.add_argument("--kf-dist",  type=float, dest="kf_dist",  default=0.0)
    bag.add_argument("--kf-rot",   type=float, dest="kf_rot",   default=0.0)
    bag.add_argument("--kf-time",  type=float, dest="kf_time",  default=0.0)
    bag.add_argument("--max-depth", type=float, dest="max_depth", default=0.0)
    bag.add_argument("--max-frames", type=int, dest="max_frames", default=0)
    bag.add_argument("--voxel-size", type=float, dest="voxel_size", default=0.0)
    bag.add_argument("--extract-mesh", action="store_true", dest="extract_mesh")
    bag.add_argument("--extract-only", action="store_true", dest="extract_only",
                     help="仅提取帧，不运行重建")

    # ── backends（列出后端）────────────────────────────────────────────────
    sub.add_parser("backends", aliases=["ls"],
                   help="列出所有可用后端及其依赖状态")

    # ── 兼容旧风格：直接传 --dataset 或 --bag（不加子命令）─────────────────
    parser.add_argument("--dataset", default="",
                        help="（旧风格）含 transforms.json 的数据集目录")
    parser.add_argument("--bag",      default="",
                        help="（旧风格）ROS2 bag 路径")
    parser.add_argument("--lidar",    action="store_true",
                        help="（旧风格）点云 bag 模式")
    parser.add_argument("--backend",  default="tsdf")
    parser.add_argument("--method",   default="")
    parser.add_argument("--output",   default="")
    parser.add_argument("--voxel-size", type=float, dest="voxel_size", default=0.0)
    parser.add_argument("--extract-mesh", action="store_true", dest="extract_mesh")
    parser.add_argument("--export-tum", metavar="OUT_DIR", dest="export_tum", default="")
    parser.add_argument("--list-backends", action="store_true", dest="list_backends")
    parser.add_argument("--cloud-topic", default="", dest="cloud_topic")
    parser.add_argument("--color-topic", default="", dest="color_topic")
    parser.add_argument("--depth-topic", default="", dest="depth_topic")
    parser.add_argument("--odom-topic",  default="", dest="odom_topic")
    parser.add_argument("--kf-dist",  type=float, dest="kf_dist",  default=0.0)
    parser.add_argument("--kf-rot",   type=float, dest="kf_rot",   default=0.0)
    parser.add_argument("--kf-time",  type=float, dest="kf_time",  default=0.0)
    parser.add_argument("--max-depth",  type=float, dest="max_depth",  default=0.0)
    parser.add_argument("--max-frames", type=int,   dest="max_frames", default=0)
    parser.add_argument("--frames-dir", default="", dest="frames_dir")
    parser.add_argument("--extract-only", action="store_true", dest="extract_only")

    args = parser.parse_args()
    setup_logging(args.log_level)

    # 路由
    cmd = getattr(args, "command", None)

    if cmd in ("reconstruct", "r") or (not cmd and args.dataset):
        return cmd_reconstruct(args)
    elif cmd in ("bag", "b") or (not cmd and args.bag):
        return cmd_from_bag(args)
    elif cmd in ("backends", "ls") or getattr(args, "list_backends", False):
        return cmd_list_backends(args)
    else:
        parser.print_help()
        return 0


if __name__ == "__main__":
    sys.exit(main())
