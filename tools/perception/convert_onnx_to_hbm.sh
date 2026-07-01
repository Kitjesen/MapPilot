#!/bin/bash
# ONNX → HBM 转换 (Horizon ai_toolchain Docker)
#
# 前置条件:
#   1. x86 Linux 主机, Docker 已安装
#   2. 拉取 Horizon 工具链镜像:
#      docker pull openexplorer/ai_toolchain:v2.6.2b
#   3. 准备校准图片 (100-200 张 640x640 JPEG) 放到 ./calib_data/
#
# 用法:
#   bash tools/perception/convert_onnx_to_hbm.sh /path/to/yolov8s-worldv2_nav120_640.onnx
#
set -euo pipefail

ONNX_PATH="${1:?Usage: $0 <onnx_path>}"
ONNX_NAME=$(basename "$ONNX_PATH" .onnx)
WORK_DIR="/tmp/hbm_convert_${ONNX_NAME}"
CALIB_DIR="${2:-./calib_data}"
DOCKER_IMAGE="openexplorer/ai_toolchain:v2.6.2b"

echo "=== YOLO-World ONNX → HBM Conversion ==="
echo "Input:  $ONNX_PATH"
echo "Output: ${WORK_DIR}/${ONNX_NAME}_nashe_640x640_nv12.hbm"

# 1. 准备工作目录
mkdir -p "$WORK_DIR"
cp "$ONNX_PATH" "$WORK_DIR/"

# 2. 生成校准数据列表
if [ ! -d "$CALIB_DIR" ] || [ -z "$(ls -A "$CALIB_DIR" 2>/dev/null)" ]; then
    echo ""
    echo "WARNING: 校准数据目录 $CALIB_DIR 为空或不存在。"
    echo "请准备 100-200 张 640x640 的导航场景图片。"
    echo "可以从 S100P 抓取: ssh sunrise@192.168.66.190 'python3 -c \"...\"'"
    echo ""
    echo "快速生成随机校准数据 (精度会略差)..."
    mkdir -p "$CALIB_DIR"
    python3 -c "
import numpy as np
from PIL import Image
import os
for i in range(100):
    img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
    Image.fromarray(img).save(os.path.join('$CALIB_DIR', f'calib_{i:04d}.jpg'))
print('Generated 100 random calibration images')
"
fi

ls "$CALIB_DIR"/*.jpg > "${WORK_DIR}/calib_list.txt" 2>/dev/null || \
ls "$CALIB_DIR"/*.png > "${WORK_DIR}/calib_list.txt" 2>/dev/null

CALIB_COUNT=$(wc -l < "${WORK_DIR}/calib_list.txt")
echo "Calibration images: $CALIB_COUNT"

# 3. 生成 Horizon 转换 YAML 配置
cat > "${WORK_DIR}/convert_config.yaml" << YAML
model_parameters:
  onnx_model: "${ONNX_NAME}.onnx"
  march: "nash_e"                    # Nash BPU (S100P / X5)
  output_model_file_prefix: "${ONNX_NAME}_nashe_640x640_nv12"
  working_dir: "model_output"

input_parameters:
  input_name: "images"
  input_type_rt: "nv12"              # BPU 原生 NV12 输入
  input_type_train: "rgb"            # 训练时用 RGB
  input_layout_train: "NCHW"
  input_shape: "1x3x640x640"
  norm_type: "data_scale"
  scale_value: "0.003921568627"      # 1/255

calibration_parameters:
  cal_data_dir: "calib_data"
  calibration_type: "default"
  max_percentile: 0.9999
  per_channel: false

compiler_parameters:
  compile_mode: "latency"            # 优先延迟 (实时导航)
  optimize_level: "O3"
  debug: false
  core_num: 2                        # Nash 双核 BPU
YAML

echo "Config written: ${WORK_DIR}/convert_config.yaml"

# 4. 运行 Docker 转换
echo ""
echo "Starting Docker conversion..."
docker run --rm \
    -v "${WORK_DIR}:/work" \
    -v "$(realpath "$CALIB_DIR"):/work/calib_data" \
    -w /work \
    "$DOCKER_IMAGE" \
    hb_mapper makertbin \
    --config convert_config.yaml \
    --model-type onnx

# 5. 检查输出
HBM_FILE="${WORK_DIR}/model_output/${ONNX_NAME}_nashe_640x640_nv12.hbm"
if [ -f "$HBM_FILE" ]; then
    SIZE_MB=$(du -m "$HBM_FILE" | cut -f1)
    echo ""
    echo "=== SUCCESS ==="
    echo "HBM model: $HBM_FILE (${SIZE_MB} MB)"
    echo ""
    echo "Deploy to S100P:"
    echo "  scp $HBM_FILE sunrise@192.168.66.190:/home/sunrise/models/"
    echo ""
    echo "Then update bpu_detector.py MODEL_CANDIDATES to include the new model."
else
    echo ""
    echo "=== FAILED ==="
    echo "HBM file not found. Check logs in ${WORK_DIR}/model_output/"
    ls -la "${WORK_DIR}/model_output/" 2>/dev/null
    exit 1
fi
