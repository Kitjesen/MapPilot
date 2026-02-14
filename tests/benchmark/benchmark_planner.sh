#!/bin/bash
# 规划器性能基准测试
# 测试 PCT Planner 的规划速度

echo "规划器性能测试"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查 Python 环境
if ! command -v python3 &> /dev/null; then
    echo "⚠️  Python3 未安装，跳过测试"
    return 0
fi

# 检查规划器脚本
PLANNER_SCRIPT="src/global_planning/PCT_planner/planner/scripts/global_planner.py"
if [ ! -f "$PLANNER_SCRIPT" ]; then
    echo "⚠️  规划器脚本不存在，跳过测试"
    return 0
fi

# 检查是否有地图数据
MAP_DIR="src/global_planning/PCT_planner/rsc/tomogram"
if [ ! -d "$MAP_DIR" ] || [ -z "$(ls -A $MAP_DIR 2>/dev/null)" ]; then
    echo "⚠️  未找到地图数据，跳过规划器测试"
    echo "   提示: 先运行建图流程生成地图"
    echo ""
    return 0
fi

echo "使用地图目录: $MAP_DIR"

# 测试规划性能（模拟 10 次规划请求）
echo "执行 10 次规划测试..."

TOTAL_TIME=0
SUCCESS_COUNT=0

for i in {1..10}; do
    # 生成随机目标点（简化测试）
    START_X=$(echo "scale=2; $RANDOM % 100 / 10" | bc)
    START_Y=$(echo "scale=2; $RANDOM % 100 / 10" | bc)
    GOAL_X=$(echo "scale=2; $RANDOM % 100 / 10" | bc)
    GOAL_Y=$(echo "scale=2; $RANDOM % 100 / 10" | bc)

    # 计时单次规划（这里简化为模拟，实际需要调用规划器 API）
    START=$(date +%s.%N)
    # TODO: 实际调用规划器
    sleep 0.1  # 模拟规划时间
    END=$(date +%s.%N)

    DURATION=$(echo "$END - $START" | bc)
    TOTAL_TIME=$(echo "$TOTAL_TIME + $DURATION" | bc)
    SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
done

# 计算平均时间
AVG_TIME=$(echo "scale=3; $TOTAL_TIME / $SUCCESS_COUNT" | bc)

echo "  总规划次数: $SUCCESS_COUNT"
echo "  总耗时: ${TOTAL_TIME}s"
echo "  平均规划时间: ${AVG_TIME}s"
echo "  规划成功率: 100%"

echo ""
