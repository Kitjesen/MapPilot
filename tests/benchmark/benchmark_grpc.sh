#!/bin/bash
# gRPC 接口性能测试
# 测试 gRPC Gateway 的吞吐量和延迟

echo "gRPC 吞吐量测试"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

GRPC_PORT=50051

# 检查 gRPC Gateway 是否运行
if ! command -v nc &> /dev/null; then
    echo "⚠️  nc 命令不可用，跳过端口检查"
elif ! nc -z localhost $GRPC_PORT 2>/dev/null; then
    echo "⚠️  gRPC Gateway 未运行 (端口 $GRPC_PORT)"
    echo "   提示: 先启动系统 (make navigation)"
    echo ""
    return 0
fi

echo "gRPC Gateway 运行中 (端口 $GRPC_PORT)"

# 检查是否安装 grpcurl
if ! command -v grpcurl &> /dev/null; then
    echo "⚠️  grpcurl 未安装，使用简化测试"
    echo ""
    echo "安装 grpcurl:"
    echo "  go install github.com/fullstorydev/grpcurl/cmd/grpcurl@latest"
    echo ""

    # 使用 Python 进行简化测试
    if command -v python3 &> /dev/null; then
        echo "使用 Python 进行简化测试..."

        python3 << 'EOF'
import socket
import time

host = 'localhost'
port = 50051
iterations = 100

print(f"测试连接延迟 ({iterations} 次)...")

total_time = 0
success_count = 0

for i in range(iterations):
    try:
        start = time.time()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((host, port))
        sock.close()
        end = time.time()

        total_time += (end - start)
        success_count += 1
    except Exception as e:
        pass

if success_count > 0:
    avg_latency = (total_time / success_count) * 1000
    qps = success_count / total_time
    print(f"  成功连接: {success_count}/{iterations}")
    print(f"  平均延迟: {avg_latency:.2f} ms")
    print(f"  QPS: {qps:.2f} req/s")
else:
    print("  ❌ 所有连接失败")
EOF
    fi

    echo ""
    return 0
fi

# 使用 grpcurl 进行完整测试
echo "使用 grpcurl 进行完整测试..."

# 测试 GetSystemInfo RPC（100 次调用）
echo ""
echo "测试 GetSystemInfo RPC (100 次调用)..."

START=$(date +%s.%N)
SUCCESS=0
FAILED=0

for i in {1..100}; do
    if grpcurl -plaintext -d '{}' localhost:$GRPC_PORT robot.v1.SystemService/GetSystemInfo > /dev/null 2>&1; then
        SUCCESS=$((SUCCESS + 1))
    else
        FAILED=$((FAILED + 1))
    fi
done

END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
QPS=$(echo "scale=2; $SUCCESS / $DURATION" | bc)
AVG_LATENCY=$(echo "scale=2; $DURATION * 1000 / 100" | bc)

echo "  成功: $SUCCESS, 失败: $FAILED"
echo "  总耗时: ${DURATION}s"
echo "  QPS: ${QPS} req/s"
echo "  平均延迟: ${AVG_LATENCY} ms"

# 测试并发性能
echo ""
echo "测试并发性能 (10 并发 × 10 次)..."

START=$(date +%s.%N)

# 启动 10 个并发请求
for i in {1..10}; do
    (
        for j in {1..10}; do
            grpcurl -plaintext -d '{}' localhost:$GRPC_PORT robot.v1.SystemService/GetSystemInfo > /dev/null 2>&1
        done
    ) &
done

# 等待所有后台任务完成
wait

END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
QPS=$(echo "scale=2; 100 / $DURATION" | bc)

echo "  总耗时: ${DURATION}s"
echo "  并发 QPS: ${QPS} req/s"

echo ""
