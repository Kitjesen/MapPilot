#!/usr/bin/env python3
"""
YOLO-World → ONNX 导出 (BPU 转换 Step 1)

将 YOLO-World 模型以固定词汇表导出为 ONNX，
文本 embedding 烘焙进模型权重，推理时无需文本编码器。

用法:
  # 在 S100P 或任何有 ultralytics 的机器上执行
  python export_yoloworld_bpu.py --model yolov8s-worldv2 --output /home/sunrise/models/

导出后:
  1. 将 .onnx 文件拷贝到 x86 Linux (有 Horizon Docker 工具链)
  2. 运行 tools/perception/convert_onnx_to_hbm.sh 转为 .hbm
  3. 将 .hbm 放到 S100P /home/sunrise/models/
"""

import argparse
import json
import os
import sys


# 导航场景词汇表 — 涵盖室内/室外/工业环境常见物体
# 比 COCO-80 大幅扩展, 针对四足机器人导航场景
NAV_VOCABULARY = [
    # === 建筑结构 ===
    "door", "wall", "window", "stairs", "staircase",
    "elevator", "escalator", "corridor", "hallway",
    "entrance", "exit", "gate", "fence", "pillar", "column",

    # === 安全设备 ===
    "fire extinguisher", "fire hydrant", "fire alarm",
    "emergency exit sign", "exit sign", "sign", "safety cone",
    "barrier", "guardrail", "handrail",

    # === 家具 ===
    "chair", "desk", "table", "couch", "sofa", "bed",
    "shelf", "cabinet", "drawer", "wardrobe", "bookshelf",
    "bench", "stool",

    # === 电子设备 ===
    "monitor", "screen", "tv", "laptop", "computer",
    "keyboard", "mouse", "printer", "phone", "cell phone",
    "projector", "camera", "speaker",

    # === 厨房/卫浴 ===
    "sink", "toilet", "mirror", "refrigerator", "microwave",
    "oven", "washing machine", "water dispenser", "faucet",

    # === 照明 ===
    "lamp", "light", "ceiling light", "floor lamp",

    # === 容器 ===
    "box", "bin", "trash can", "recycling bin",
    "bottle", "cup", "bucket", "container",

    # === 植物/装饰 ===
    "potted plant", "plant", "flower", "vase", "painting",
    "clock", "calendar",

    # === 交通/户外 ===
    "car", "truck", "bicycle", "motorcycle",
    "traffic light", "stop sign", "parking meter",
    "sidewalk", "crosswalk", "road", "curb",

    # === 人物 ===
    "person", "people", "pedestrian",

    # === 工业设备 ===
    "machine", "equipment", "robot", "forklift",
    "pallet", "conveyor", "pipe", "valve", "panel",
    "control panel", "switch", "button",

    # === 地面/障碍 ===
    "obstacle", "rock", "cone", "pole", "bollard",
    "manhole", "drain", "grate",

    # === 其他常见 ===
    "backpack", "bag", "umbrella", "suitcase",
    "book", "paper", "whiteboard", "blackboard",
    "carpet", "rug", "mat", "curtain",
]


def export_yoloworld(
    model_name: str = "yolov8s-worldv2",
    output_dir: str = ".",
    imgsz: int = 640,
    vocabulary: list = None,
):
    """导出 YOLO-World 为带固定词汇表的 ONNX。"""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed. pip install ultralytics")
        sys.exit(1)

    if vocabulary is None:
        vocabulary = NAV_VOCABULARY

    # 去重 + 排序
    vocabulary = sorted(set(v.strip().lower() for v in vocabulary if v.strip()))
    print(f"Vocabulary: {len(vocabulary)} classes")
    print(f"  First 10: {vocabulary[:10]}")
    print(f"  Last 10:  {vocabulary[-10:]}")

    # 加载 YOLO-World
    print(f"\nLoading {model_name}...")
    model = YOLO(model_name)

    # 设置自定义词汇表 (烘焙 text embedding 到模型)
    print("Setting custom vocabulary (baking text embeddings)...")
    model.set_classes(vocabulary)

    # 导出 ONNX
    onnx_name = f"{model_name.replace('/', '-')}_nav{len(vocabulary)}_{imgsz}.onnx"
    onnx_path = os.path.join(output_dir, onnx_name)

    print(f"Exporting to ONNX: {onnx_path}")
    model.export(
        format="onnx",
        imgsz=imgsz,
        simplify=True,
        opset=12,  # YOLO-World 需要 opset 12+ (einsum 算子)
    )

    # ultralytics 默认导出到模型同目录, 找到并移动
    default_onnx = model_name.replace("/", "-") + ".onnx"
    for candidate in [default_onnx, model_name + ".onnx"]:
        if os.path.exists(candidate):
            os.rename(candidate, onnx_path)
            break

    if not os.path.exists(onnx_path):
        # 可能导出到了其他位置, 搜索
        import glob
        candidates = glob.glob(f"**/*world*.onnx", recursive=True)
        if candidates:
            os.rename(candidates[0], onnx_path)

    if os.path.exists(onnx_path):
        size_mb = os.path.getsize(onnx_path) / 1024 / 1024
        print(f"\nExport OK: {onnx_path} ({size_mb:.1f} MB)")
    else:
        print(f"\nWARNING: ONNX file not found at expected path.")
        print("Check current directory for .onnx files.")

    # 保存词汇表映射 (BPU 检测器加载时需要)
    vocab_path = os.path.join(output_dir, onnx_name.replace(".onnx", "_vocab.json"))
    vocab_data = {
        "model": model_name,
        "imgsz": imgsz,
        "num_classes": len(vocabulary),
        "names": {i: name for i, name in enumerate(vocabulary)},
    }
    with open(vocab_path, "w", encoding="utf-8") as f:
        json.dump(vocab_data, f, ensure_ascii=False, indent=2)
    print(f"Vocabulary saved: {vocab_path}")

    print(f"\n=== Next Step ===")
    print(f"1. Copy {onnx_path} to x86 Linux with Horizon Docker")
    print(f"2. Run: bash tools/perception/convert_onnx_to_hbm.sh {onnx_path}")
    print(f"3. Copy resulting .hbm to S100P /home/sunrise/models/")

    return onnx_path, vocab_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export YOLO-World for BPU")
    parser.add_argument(
        "--model", default="yolov8s-worldv2",
        help="YOLO-World model name (default: yolov8s-worldv2)",
    )
    parser.add_argument(
        "--output", default=".",
        help="Output directory",
    )
    parser.add_argument(
        "--imgsz", type=int, default=640,
        help="Input image size (default: 640)",
    )
    args = parser.parse_args()

    export_yoloworld(
        model_name=args.model,
        output_dir=args.output,
        imgsz=args.imgsz,
    )
