#!/bin/bash
# 运行所有性能基准测试
# 用途: 建立性能基线，用于回归测试

set -e

echo "=========================================="
echo "  MapPilot 性能基准测试"
echo "=========================================="
echo ""
echo "测试时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# 创建结果目录
RESULT_DIR="tests/benchmark/results"
mkdir -p "$RESULT_DIR"
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
RESULT_FILE="$RESULT_DIR/benchmark_${TIMESTAMP}.txt"

# 记录系统信息
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" | tee -a "$RESULT_FILE"
echo "系统信息" | tee -a "$RESULT_FILE"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" | tee -a "$RESULT_FILE"
echo "操作系统: $(uname -s)" | tee -a "$RESULT_FILE"
echo "内核版本: $(uname -r)" | tee -a "$RESULT_FILE"
echo "CPU 核心数: $(nproc)" | tee -a "$RESULT_FILE"
echo "内存总量: $(free -h | awk '/^Mem:/ {print $2}')" | tee -a "$RESULT_FILE"
echo "" | tee -a "$RESULT_FILE"

# 1. SLAM 性能测试
if [ -f "tests/benchmark/benchmark_slam.sh" ]; then
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}1. SLAM 性能测试${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    bash tests/benchmark/benchmark_slam.sh | tee -a "$RESULT_FILE"
    echo "" | tee -a "$RESULT_FILE"
else
    echo "⚠️  benchmark_slam.sh 不存在，跳过" | tee -a "$RESULT_FILE"
fi

# 2. 规划器性能测试
if [ -f "tests/benchmark/benchmark_planner.sh" ]; then
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}2. 规划器性能测试${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    bash tests/benchmark/benchmark_planner.sh | tee -a "$RESULT_FILE"
    echo "" | tee -a "$RESULT_FILE"
else
    echo "⚠️  benchmark_planner.sh 不存在，跳过" | tee -a "$RESULT_FILE"
fi

# 3. gRPC 吞吐量测试
if [ -f "tests/benchmark/benchmark_grpc.sh" ]; then
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}3. gRPC 吞吐量测试${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    bash tests/benchmark/benchmark_grpc.sh | tee -a "$RESULT_FILE"
    echo "" | tee -a "$RESULT_FILE"
else
    echo "⚠️  benchmark_grpc.sh 不存在，跳过" | tee -a "$RESULT_FILE"
fi

# 4. 编译性能测试
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}4. 编译性能测试${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "测试完整编译时间..." | tee -a "$RESULT_FILE"

# 清理编译产物
rm -rf build/ install/ log/ 2>/dev/null || true

# 计时编译
START=$(date +%s)
source /opt/ros/humble/setup.bash 2>/dev/null || true
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1 || echo "编译失败"
END=$(date +%s)
DURATION=$((END - START))

echo "  完整编译时间: ${DURATION}s" | tee -a "$RESULT_FILE"
echo "" | tee -a "$RESULT_FILE"

# 总结
echo "=========================================="
echo "  基准测试完成"
echo "=========================================="
echo ""
echo -e "${GREEN}✅ 结果已保存到: $RESULT_FILE${NC}"
echo ""
echo "提示:"
echo "  - 查看结果: cat $RESULT_FILE"
echo "  - 对比历史: ls -lh $RESULT_DIR"
echo ""
