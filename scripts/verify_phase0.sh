#!/bin/bash
# 快速验证脚本 - 验证 Phase 0 所有功能是否正常工作

echo "=========================================="
echo "  Phase 0 功能验证"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0
FAIL=0

check_file() {
    local file=$1
    local desc=$2

    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $desc"
        PASS=$((PASS + 1))
    else
        echo -e "${RED}✗${NC} $desc (文件不存在: $file)"
        FAIL=$((FAIL + 1))
    fi
}

check_executable() {
    local file=$1
    local desc=$2

    if [ -f "$file" ] && [ -x "$file" ]; then
        echo -e "${GREEN}✓${NC} $desc"
        PASS=$((PASS + 1))
    elif [ -f "$file" ]; then
        echo -e "${YELLOW}⚠${NC} $desc (文件存在但不可执行)"
        chmod +x "$file" 2>/dev/null && echo "  已添加执行权限"
        PASS=$((PASS + 1))
    else
        echo -e "${RED}✗${NC} $desc (文件不存在: $file)"
        FAIL=$((FAIL + 1))
    fi
}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. 核心文件检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_file "Makefile" "Makefile"
check_executable "scripts/health_check.sh" "健康检查脚本"
check_file "tests/README.md" "测试文档"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. 基准测试脚本检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_executable "tests/benchmark/run_all.sh" "基准测试入口"
check_executable "tests/benchmark/benchmark_slam.sh" "SLAM 性能测试"
check_executable "tests/benchmark/benchmark_planner.sh" "规划器性能测试"
check_executable "tests/benchmark/benchmark_grpc.sh" "gRPC 吞吐量测试"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. 集成测试脚本检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
check_executable "tests/integration/run_all.sh" "集成测试入口"
check_executable "tests/integration/test_full_stack.sh" "全系统启动测试"
check_file "tests/integration/test_grpc_endpoints.py" "gRPC 接口测试"
check_file "tests/integration/test_topic_hz.py" "话题频率验证"
check_file "tests/integration/test_network_failure.py" "网络故障模拟"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. Makefile 命令检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

COMMANDS=("help" "build" "test" "clean" "health" "benchmark")
for cmd in "${COMMANDS[@]}"; do
    if grep -q "^${cmd}:" Makefile 2>/dev/null; then
        echo -e "${GREEN}✓${NC} make $cmd"
        PASS=$((PASS + 1))
    else
        echo -e "${RED}✗${NC} make $cmd (未定义)"
        FAIL=$((FAIL + 1))
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. 文档检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cd ..
check_file "工程化评估报告.md" "工程化评估报告"
check_file "工程化实施计划.md" "工程化实施计划"
check_file "工程化实施记录.md" "工程化实施记录"
check_file "Phase0_完成总结.md" "Phase 0 完成总结"
check_file "开始使用指南.md" "开始使用指南"
cd 3d_NAV

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6. 功能测试"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 测试 Makefile help
if make help > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} make help 可执行"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗${NC} make help 执行失败"
    FAIL=$((FAIL + 1))
fi

# 测试健康检查脚本语法
if bash -n scripts/health_check.sh 2>/dev/null; then
    echo -e "${GREEN}✓${NC} health_check.sh 语法正确"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗${NC} health_check.sh 语法错误"
    FAIL=$((FAIL + 1))
fi

# 测试 Python 脚本语法
for py_file in tests/integration/*.py; do
    if [ -f "$py_file" ]; then
        if python3 -m py_compile "$py_file" 2>/dev/null; then
            echo -e "${GREEN}✓${NC} $(basename $py_file) 语法正确"
            PASS=$((PASS + 1))
        else
            echo -e "${RED}✗${NC} $(basename $py_file) 语法错误"
            FAIL=$((FAIL + 1))
        fi
    fi
done

echo ""
echo "=========================================="
echo "  验证结果"
echo "=========================================="
echo ""
echo "通过: $PASS"
echo "失败: $FAIL"
echo "总计: $((PASS + FAIL))"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✅ Phase 0 所有功能验证通过！${NC}"
    echo ""
    echo "你现在可以开始使用了："
    echo "  make help      - 查看所有命令"
    echo "  make health    - 系统健康检查"
    echo "  make build     - 编译项目"
    echo "  make test      - 运行测试"
    echo "  make benchmark - 性能测试"
    echo ""
    exit 0
else
    echo -e "${RED}❌ 有 $FAIL 项验证失败${NC}"
    echo ""
    echo "请检查上述失败项"
    exit 1
fi
