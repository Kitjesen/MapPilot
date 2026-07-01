#!/usr/bin/env python3
"""reconstruct_local.py 鈥?鏈湴绂荤嚎涓夌淮閲嶅缓宸ュ叿锛堟棤闇€鏈嶅姟鍣級銆?
浠庝互涓嬭緭鍏ヤ箣涓€鍔犺浇 RGB-D 鏁版嵁骞跺湪鏈満杩愯閲嶅缓锛?  1. 褰曞埗鏁版嵁闆嗙洰褰曪紙DatasetRecorderModule 杈撳嚭锛屽惈 transforms.json锛?  2. ROS2 bag锛?db3 鏂囦欢鎴栧惈 metadata.yaml 鐨勭洰褰曪級
  3. 鐐逛簯 bag锛堢洿鎺ュ悎骞?LiDAR 鐐逛簯涓?PLY锛?
鐢ㄦ硶绀轰緥:
  # 浠?recorder 褰曞埗鐨勬暟鎹泦閲嶅缓锛圱SDF锛孋PU锛?  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000

  # 浠?ROS2 bag 鎻愬彇甯у悗閲嶅缓
  python tools/reconstruction/reconstruct_local.py --bag ~/data/run1/ --backend tsdf

  # 鐩存帴浠庣偣浜?bag 鐢熸垚 PLY锛堟渶绠€鍗曪級
  python tools/reconstruction/reconstruct_local.py --bag ~/data/run1/ --lidar

  # 鐢?NeRF锛堥渶瑕?GPU + nerfstudio锛?  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000 \\
      --backend nerfstudio --method instant-ngp

  # 鐢?3D Gaussian Splatting
  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000 \\
      --backend nerfstudio --method splatfacto

  # 鍒楀嚭鎵€鏈夊彲鐢ㄥ悗绔強鍏朵緷璧栫姸鎬?  python tools/reconstruction/reconstruct_local.py --list-backends

  # 灏嗘暟鎹泦杞崲涓?TUM 鏍煎紡锛堢敤浜?RTAB-Map / GSFusion锛?  python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/20250409_120000 \\
      --export-tum /tmp/my_tum_dataset

杈撳嚭:
  outputs/reconstruction/<session>/
  鈹溾攢鈹€ reconstruction.ply    (TSDF / Open3D)
  鈹溾攢鈹€ mesh.obj              (Open3D锛屽彲閫?
  鈹斺攢鈹€ ns_output/            (nerfstudio checkpoint)
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path

# 鈹€鈹€ sys.path 璁剧疆 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
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
        format="%(asctime)s %(levelname)s %(name)s 鈥?%(message)s",
        datefmt="%H:%M:%S",
    )


def cmd_reconstruct(args) -> int:
    """浠庢暟鎹泦鐩綍杩愯閲嶅缓銆?""
    from perception.reconstruction.dataset_io import (
        load_dataset, dataset_stats, export_tum
    )
    from perception.reconstruction.server.backends.registry import get_backend, list_backends

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

    # 浠呭鍑烘牸寮忚浆鎹?    if args.export_tum:
        out = Path(args.export_tum).expanduser()
        export_tum(keyframes, out)
        print(f"Exported TUM dataset to: {out}")
        return 0

    # 閲嶅缓
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
        print(f"\n鉁?Done in {elapsed:.1f}s")
        print(f"  Output:  {result.output_path}")
        print(f"  Format:  {result.output_format}")
        print(f"  Frames:  {result.num_frames}")
        if result.extra:
            for k, v in result.extra.items():
                print(f"  {k}: {v}")
        return 0
    else:
        print(f"\n鉁?Reconstruction failed: {result.message}")
        return 1


def cmd_from_bag(args) -> int:
    """浠?bag 鎻愬彇甯у悗閲嶅缓锛堟垨鐩存帴鍚堝苟鐐逛簯锛夈€?""
    bag_path = Path(args.bag).expanduser()
    if not bag_path.exists():
        print(f"[ERROR] Bag not found: {bag_path}")
        return 1

    if args.lidar:
        # 鐩存帴浠庣偣浜?bag 鐢熸垚 PLY
        from perception.adapters.ros2.bag_reader import read_lidar_bag
        out_ply = Path(args.output or f"outputs/reconstruction/{bag_path.stem}_lidar.ply")
        out_ply.parent.mkdir(parents=True, exist_ok=True)
        print(f"Extracting LiDAR point cloud from {bag_path} 鈫?{out_ply}")
        try:
            result_path = read_lidar_bag(
                bag_path, out_ply,
                cloud_topic=args.cloud_topic,
                voxel_size=args.voxel_size or 0.05,
            )
            print(f"鉁?Point cloud saved: {result_path}")
            return 0
        except Exception as exc:
            print(f"鉁?Error: {exc}")
            return 1

    # RGB-D bag 鈫?鎻愬彇甯?鈫?閲嶅缓
    from perception.adapters.ros2.bag_reader import read_rgb_d_bag

    frames_dir = Path(args.frames_dir or
                      f"outputs/reconstruction/{bag_path.stem}_frames").expanduser()
    frames_dir.mkdir(parents=True, exist_ok=True)

    print(f"Extracting RGB-D keyframes from {bag_path} 鈫?{frames_dir}")
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
        print(f"鉁?Bag extraction failed: {exc}")
        return 1

    print(f"  Extracted {len(keyframes)} keyframes")

    if args.extract_only:
        print(f"鉁?Frames saved to {frames_dir}")
        return 0

    # 濮旀墭 cmd_reconstruct
    args.dataset = str(frames_dir)
    return cmd_reconstruct(args)


def cmd_list_backends(args) -> int:
    from perception.reconstruction.server.backends.registry import list_backends, get_backend
    print("Available reconstruction backends:\n")
    for name in list_backends():
        cls  = get_backend(name)
        inst = cls()
        ok, reason = inst.check_dependencies()
        mark = "鉁? if ok else "鉁?
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
        description="LingTu 鏈湴涓夌淮閲嶅缓宸ュ叿",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--log-level", default="INFO",
                        help="鏃ュ織绾у埆 (DEBUG/INFO/WARNING)")

    sub = parser.add_subparsers(dest="command")

    # 鈹€鈹€ reconstruct锛堥粯璁ゅ懡浠わ級鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    rec = sub.add_parser("reconstruct", aliases=["r"],
                         help="浠庢暟鎹泦鐩綍閲嶅缓锛圖atasetRecorderModule 杈撳嚭锛?)
    rec.add_argument("--dataset", required=True,
                     help="鍚?transforms.json 鐨勬暟鎹泦鐩綍")
    rec.add_argument("--backend", default="tsdf",
                     help="閲嶅缓鍚庣: tsdf / open3d / nerfstudio / gsfusion")
    rec.add_argument("--method", default="",
                     help="nerfstudio 鏂规硶: instant-ngp / nerfacto / splatfacto / depth-nerfacto")
    rec.add_argument("--output", default="",
                     help="杈撳嚭鐩綍锛堥粯璁? outputs/reconstruction/<dataset_name>锛?)
    rec.add_argument("--voxel-size", type=float, dest="voxel_size", default=0.0,
                     help="TSDF/Open3D 浣撶礌澶у皬锛坢锛夛紝0=浣跨敤鍚庣榛樿鍊?)
    rec.add_argument("--extract-mesh", action="store_true", dest="extract_mesh",
                     help="鍚屾椂鎻愬彇涓夎缃戞牸锛圱SDF 鍚庣锛?)
    rec.add_argument("--export-tum", metavar="OUT_DIR", dest="export_tum",
                     help="浠呭皢鏁版嵁闆嗗鍑轰负 TUM 鏍煎紡锛屼笉杩愯閲嶅缓")

    # 鈹€鈹€ bag锛堜粠 bag 鏂囦欢澶勭悊锛夆攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    bag = sub.add_parser("bag", aliases=["b"],
                         help="浠?ROS2 bag 鎻愬彇甯у悗閲嶅缓锛堟垨鐩存帴鍚堝苟鐐逛簯锛?)
    bag.add_argument("--bag", required=True,
                     help="ROS2 bag 鐩綍鎴?.db3 鏂囦欢璺緞")
    bag.add_argument("--lidar", action="store_true",
                     help="鐩存帴浠庣偣浜戣瘽棰樺悎骞?PLY锛屼笉鍋氬浘鍍忛噸寤?)
    bag.add_argument("--backend", default="tsdf")
    bag.add_argument("--method", default="")
    bag.add_argument("--output", default="",
                     help="杈撳嚭鏂囦欢/鐩綍")
    bag.add_argument("--frames-dir", default="", dest="frames_dir",
                     help="鍏抽敭甯ф彁鍙栫洰褰曪紙榛樿: outputs/reconstruction/<bag_name>_frames锛?)
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
                     help="浠呮彁鍙栧抚锛屼笉杩愯閲嶅缓")

    # 鈹€鈹€ backends锛堝垪鍑哄悗绔級鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    sub.add_parser("backends", aliases=["ls"],
                   help="鍒楀嚭鎵€鏈夊彲鐢ㄥ悗绔強鍏朵緷璧栫姸鎬?)

    # 鈹€鈹€ 鍏煎鏃ч鏍硷細鐩存帴浼?--dataset 鎴?--bag锛堜笉鍔犲瓙鍛戒护锛夆攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    parser.add_argument("--dataset", default="",
                        help="锛堟棫椋庢牸锛夊惈 transforms.json 鐨勬暟鎹泦鐩綍")
    parser.add_argument("--bag",      default="",
                        help="锛堟棫椋庢牸锛塕OS2 bag 璺緞")
    parser.add_argument("--lidar",    action="store_true",
                        help="锛堟棫椋庢牸锛夌偣浜?bag 妯″紡")
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

    # 璺敱
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
