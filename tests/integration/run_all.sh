#!/bin/bash
# 运行所有集成测试

set -e

echo "=========================================="
echo "  集成测试套件"
echo "=========================================="
echo ""

PASSED=0
FAILED=0
TESTS=()

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

run_test() {
    local test_name=$1
    local test_script=$2

    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}运行: $test_name${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    if bash "$test_script"; then
        echo -e "${GREEN}✅ $test_name 通过${NC}"
        PASSED=$((PASSED + 1))
        TESTS+=("✅ $test_name")
    else
        echo -e "${RED}❌ $test_name 失败${NC}"
        FAILED=$((FAILED + 1))
        TESTS+=("❌ $test_name")
    fi

    echo ""
}

run_python_test() {
    local test_name=$1
    local test_script=$2

    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}运行: $test_name${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    if python3 "$test_script"; then
        echo -e "${GREEN}✅ $test_name 通过${NC}"
        PASSED=$((PASSED + 1))
        TESTS+=("✅ $test_name")
    else
        echo -e "${RED}❌ $test_name 失败${NC}"
        FAILED=$((FAILED + 1))
        TESTS+=("❌ $test_name")
    fi

    echo ""
}

# 1. 全系统启动测试
if [ -f "tests/integration/test_full_stack.sh" ]; then
    run_test "全系统启动测试" "tests/integration/test_full_stack.sh"
else
    echo "⚠️  test_full_stack.sh 不存在，跳过"
fi

# 2. gRPC 接口测试
if [ -f "tests/integration/test_grpc_endpoints.py" ]; then
    run_python_test "gRPC 接口测试" "tests/integration/test_grpc_endpoints.py"
else
    echo "⚠️  test_grpc_endpoints.py 不存在，跳过"
fi

# 3. 话题频率测试
if [ -f "tests/integration/test_topic_hz.py" ]; then
    run_python_test "话题频率测试" "tests/integration/test_topic_hz.py"
else
    echo "⚠️  test_topic_hz.py 不存在，跳过"
fi

# 4. 网络故障测试
if [ -f "tests/integration/test_network_failure.py" ]; then
    run_python_test "网络故障测试" "tests/integration/test_network_failure.py"
else
    echo "⚠️  test_network_failure.py 不存在，跳过"
fi

# 总结
echo "=========================================="
echo "  测试结果汇总"
echo "=========================================="
echo ""

for test in "${TESTS[@]}"; do
    echo "$test"
done

echo ""
echo "通过: $PASSED"
echo "失败: $FAILED"
echo "总计: $((PASSED + FAILED))"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✅ 所有集成测试通过${NC}"
    exit 0
else
    echo -e "${RED}❌ 有 $FAILED 个测试失败${NC}"
    exit 1
fi
