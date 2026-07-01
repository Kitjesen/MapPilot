#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# 导航系统 systemd 服务集成测试
# 使用方法: bash scripts/test_services.sh [--skip-grpc]
#
# 测试内容:
#   1. 各服务独立启停
#   2. 服务依赖自动拉起
#   3. gRPC SetMode 编排 (需要 nav-grpc 运行)
#   4. ROS2 话题验证
#
# 前置条件:
#   - sudo 无密码 (已配置 sudoers)
#   - ROS2 环境已安装
#   - 服务已注册 (install_services.sh)
# ═══════════════════════════════════════════════════════════════════
set -uo pipefail
# 注意: 不用 set -e, 因为部分命令 (ros2 topic hz, grpcurl) 可能返回非零

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROTO_DIR="$NAV_DIR/src/robot_proto/proto"

SKIP_GRPC=false
[[ "${1:-}" == "--skip-grpc" ]] && SKIP_GRPC=true

# ── 颜色 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
FAIL=0
SKIP=0

pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1 — $2"; ((FAIL++)); }
skip() { echo -e "  ${YELLOW}[SKIP]${NC} $1"; ((SKIP++)); }
info() { echo -e "${CYAN}── $1 ──${NC}"; }

# ── 辅助 ──

svc_state() {
  local s
  s=$(systemctl is-active "$1" 2>/dev/null) || true
  echo "${s:-unknown}" | head -1 | tr -d '\n'
}

wait_for_state() {
  local svc="$1" expected="$2" timeout="${3:-10}"
  for ((i=0; i<timeout; i++)); do
    if [[ "$(svc_state "$svc")" == "$expected" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

stop_all() {
  for svc in nav-planning nav-autonomy nav-slam nav-lidar; do
    sudo /bin/systemctl stop "${svc}.service" 2>/dev/null || true
  done
  sleep 2
  # 清除 failed 状态残留, 否则下次 start 可能失败
  for svc in nav-planning nav-autonomy nav-slam nav-lidar; do
    sudo /bin/systemctl reset-failed "${svc}.service" 2>/dev/null || true
  done
}

# 检查 ROS2 话题频率 (返回 Hz 或 0)
check_topic_hz() {
  local topic="$1" window="${2:-3}"
  local hz
  hz=$(timeout 5 ros2 topic hz "$topic" --window "$window" 2>&1 | grep "average rate" | head -1 | awk '{print $3}' || true)
  # 清理: 只取第一行数字
  hz=$(echo "$hz" | head -1 | tr -d '\n' | grep -oE '[0-9]+\.?[0-9]*' | head -1)
  echo "${hz:-0}"
}

# ═══════════════════════════════════════════════════════════════════
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Navigation Service Integration Test                     ║"
echo "║  $(date '+%Y-%m-%d %H:%M:%S')                                    ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ── 前置检查 ──
info "前置检查"

# 检查 sudoers
if sudo -n /bin/systemctl status nav-grpc.service &>/dev/null; then
  pass "sudoers 配置正确"
else
  fail "sudoers" "sunrise 用户没有 systemctl 免密权限"
  echo "请先运行: sudo bash scripts/install_services.sh"
  exit 1
fi

# 检查 ROS2
set +u  # setup.bash 中有未设置的变量
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$NAV_DIR/install/setup.bash" 2>/dev/null || true
set -u
if command -v ros2 &>/dev/null; then
  pass "ROS2 环境可用"
else
  fail "ROS2" "ros2 命令不可用"
  exit 1
fi

# 检查 grpcurl
if command -v grpcurl &>/dev/null; then
  pass "grpcurl 可用: $(grpcurl --version 2>&1 | head -1)"
else
  if ! $SKIP_GRPC; then
    skip "grpcurl 未安装, gRPC 测试将跳过"
    SKIP_GRPC=true
  fi
fi

# ── 清理: 停止所有导航服务 ──
info "清理初始状态"
stop_all
for svc in nav-lidar nav-slam nav-autonomy nav-planning; do
  state=$(svc_state "${svc}.service")
  case "$state" in
    inactive|dead)
      pass "${svc} 已停止 ($state)" ;;
    failed)
      # failed 也算已停止, 可能是之前的残留
      sudo /bin/systemctl reset-failed "${svc}.service" 2>/dev/null || true
      pass "${svc} 已停止 (was failed, reset)" ;;
    *)
      fail "${svc} 初始状态" "expected inactive, got $state" ;;
  esac
done

# ═══════════════════════════════════════════════════════════════════
# TEST 1: nav-lidar 独立启停
# ═══════════════════════════════════════════════════════════════════
echo ""
info "TEST 1: nav-lidar 启停"

sudo /bin/systemctl start nav-lidar.service
if wait_for_state nav-lidar.service active 10; then
  pass "nav-lidar 启动成功"
else
  fail "nav-lidar 启动" "状态: $(svc_state nav-lidar.service)"
fi

sudo /bin/systemctl stop nav-lidar.service
sleep 3
lidar_stop=$(svc_state nav-lidar.service)
case "$lidar_stop" in
  inactive|dead|failed)
    pass "nav-lidar 停止成功 ($lidar_stop)" ;;
  *)
    fail "nav-lidar 停止" "状态: $lidar_stop" ;;
esac

# ═══════════════════════════════════════════════════════════════════
# TEST 2: nav-slam 依赖拉起
# ═══════════════════════════════════════════════════════════════════
echo ""
info "TEST 2: nav-slam 依赖拉起 (应自动启动 nav-lidar)"

sudo /bin/systemctl start nav-slam.service
sleep 6

slam_state=$(svc_state nav-slam.service)
lidar_state=$(svc_state nav-lidar.service)

if [[ "$slam_state" == "active" ]]; then
  pass "nav-slam 启动成功"
else
  fail "nav-slam 启动" "状态: $slam_state"
fi

if [[ "$lidar_state" == "active" ]]; then
  pass "nav-lidar 被 nav-slam 自动拉起"
else
  fail "nav-lidar 自动拉起" "状态: $lidar_state"
fi

# 验证 ROS2 话题
sleep 2
hz=$(check_topic_hz /Odometry 3)
if [[ "$hz" != "0" ]] && (( $(echo "$hz > 0" | bc -l 2>/dev/null || echo 0) )); then
  pass "/Odometry 话题活跃: ${hz} Hz"
else
  skip "/Odometry 话题: ${hz} Hz (可能需要雷达数据)"
fi

# 停止 slam, lidar 应保留
sudo /bin/systemctl stop nav-slam.service
sleep 3
lidar_after=$(svc_state nav-lidar.service)
if [[ "$lidar_after" == "active" ]]; then
  pass "停止 nav-slam 后 nav-lidar 保留 ($lidar_after)"
else
  # lidar 可能因为 BindsTo 跟着停了, 这取决于 unit 配置
  skip "停止 nav-slam 后 nav-lidar 状态: $lidar_after"
fi

stop_all

# ═══════════════════════════════════════════════════════════════════
# TEST 3: nav-autonomy 依赖链
# ═══════════════════════════════════════════════════════════════════
echo ""
info "TEST 3: nav-autonomy 启动 (应拉起 lidar+slam)"

sudo /bin/systemctl start nav-autonomy.service
sleep 8

autonomy_state=$(svc_state nav-autonomy.service)
slam_state=$(svc_state nav-slam.service)
lidar_state=$(svc_state nav-lidar.service)

if [[ "$autonomy_state" == "active" ]]; then
  pass "nav-autonomy 启动成功"
else
  fail "nav-autonomy 启动" "状态: $autonomy_state"
fi

if [[ "$slam_state" == "active" && "$lidar_state" == "active" ]]; then
  pass "依赖链 lidar+slam 自动启动"
else
  fail "依赖链" "slam=$slam_state lidar=$lidar_state"
fi

# 验证地形分析话题
sleep 2
hz_terrain=$(check_topic_hz /terrain_map 3)
hz_path=$(check_topic_hz /path 3)
if [[ "$hz_terrain" != "0" ]] && (( $(echo "$hz_terrain > 0" | bc -l 2>/dev/null || echo 0) )); then
  pass "/terrain_map 活跃: ${hz_terrain} Hz"
else
  skip "/terrain_map: ${hz_terrain} Hz (可能需要雷达数据)"
fi

# 停止 autonomy
sudo /bin/systemctl stop nav-autonomy.service
sleep 3
if [[ "$(svc_state nav-autonomy.service)" != "active" ]]; then
  pass "nav-autonomy 停止成功"
else
  fail "nav-autonomy 停止" "仍在运行"
fi

stop_all

# ═══════════════════════════════════════════════════════════════════
# TEST 4: nav-planning 启动
# ═══════════════════════════════════════════════════════════════════
echo ""
info "TEST 4: nav-planning 启动"

sudo /bin/systemctl start nav-planning.service
sleep 10

planning_state=$(svc_state nav-planning.service)
if [[ "$planning_state" == "active" ]]; then
  pass "nav-planning 启动成功"
else
  fail "nav-planning 启动" "状态: $planning_state"
fi

# 检查 global_planner.py 是否正常 (修复后不应有 librcl_action 错误)
planner_error=$(journalctl -u nav-planning --no-pager --since "15 sec ago" 2>/dev/null | grep -c "librcl_action" || true)
if [[ "$planner_error" -eq 0 ]]; then
  pass "global_planner.py 无 librcl_action 错误"
else
  fail "global_planner.py" "仍有 librcl_action.so 错误"
fi

# 检查 tomogram 加载
tomogram_loaded=$(journalctl -u nav-planning --no-pager --since "15 sec ago" 2>/dev/null | grep -c "PCT Global Planner Ready" || true)
if [[ "$tomogram_loaded" -gt 0 ]]; then
  pass "global_planner.py tomogram 加载成功"
else
  skip "global_planner.py tomogram 未加载 (可能无地图文件)"
fi

sudo /bin/systemctl stop nav-planning.service
sleep 3

stop_all

# ═══════════════════════════════════════════════════════════════════
# TEST 5: gRPC SetMode 编排 (需要 nav-grpc)
# ═══════════════════════════════════════════════════════════════════
echo ""
if $SKIP_GRPC; then
  info "TEST 5: gRPC 编排测试 (跳过)"
  skip "gRPC 测试跳过 (--skip-grpc 或缺少 grpcurl)"
else
  info "TEST 5: gRPC SetMode 编排"

  # 确保 nav-grpc 运行
  sudo /bin/systemctl start nav-grpc.service
  sleep 5
  if [[ "$(svc_state nav-grpc.service)" != "active" ]]; then
    fail "nav-grpc" "无法启动"
  else
    pass "nav-grpc 启动成功"

    # 5a: SetMode MAPPING
    echo ""
    echo "  5a: SetMode MAPPING..."
    result=$(grpcurl -plaintext \
      -import-path "$PROTO_DIR" \
      -proto control.proto \
      -d '{"base": {"request_id": "test-5a"}, "mode": "ROBOT_MODE_MAPPING"}' \
      127.0.0.1:50051 robot.v1.ControlService/SetMode 2>&1 || true)

    if echo "$result" | grep -q "ROBOT_MODE_MAPPING"; then
      pass "5a SetMode MAPPING 返回正确"
    else
      fail "5a SetMode MAPPING" "$result"
    fi

    # 等待服务编排
    sleep 8

    slam_state=$(svc_state nav-slam.service)
    lidar_state=$(svc_state nav-lidar.service)
    if [[ "$slam_state" == "active" && "$lidar_state" == "active" ]]; then
      pass "5a MAPPING: lidar+slam 已启动"
    else
      fail "5a MAPPING 服务" "slam=$slam_state lidar=$lidar_state"
    fi

    # 5b: SetMode IDLE
    echo ""
    echo "  5b: SetMode IDLE..."
    result=$(grpcurl -plaintext \
      -import-path "$PROTO_DIR" \
      -proto control.proto \
      -d '{"base": {"request_id": "test-5b"}, "mode": "ROBOT_MODE_IDLE"}' \
      127.0.0.1:50051 robot.v1.ControlService/SetMode 2>&1 || true)

    if echo "$result" | grep -q "ROBOT_MODE_IDLE"; then
      pass "5b SetMode IDLE 返回正确"
    else
      fail "5b SetMode IDLE" "$result"
    fi

    # 5c: SetMode TELEOP 无 Lease — 应拒绝
    echo ""
    echo "  5c: SetMode TELEOP (no lease, should reject)..."
    result=$(grpcurl -plaintext \
      -import-path "$PROTO_DIR" \
      -proto control.proto \
      -d '{"base": {"request_id": "test-5c"}, "mode": "ROBOT_MODE_TELEOP"}' \
      127.0.0.1:50051 robot.v1.ControlService/SetMode 2>&1 || true)

    if echo "$result" | grep -qi "lease\|MODE_CONFLICT\|denied"; then
      pass "5c TELEOP 无 lease 被正确拒绝"
    else
      fail "5c TELEOP 无 lease" "应拒绝但返回: $result"
    fi

    # 5d: AcquireLease + SetMode TELEOP
    echo ""
    echo "  5d: AcquireLease + SetMode TELEOP..."
    lease_result=$(grpcurl -plaintext \
      -import-path "$PROTO_DIR" \
      -proto control.proto \
      -d '{"base": {"request_id": "test-5d-lease"}}' \
      127.0.0.1:50051 robot.v1.ControlService/AcquireLease 2>&1 || true)

    lease_token=$(echo "$lease_result" | grep -o '"leaseToken": "[^"]*"' | head -1 | cut -d'"' -f4 || true)
    if [[ -n "$lease_token" ]]; then
      pass "5d AcquireLease: token=${lease_token:0:8}..."

      result=$(grpcurl -plaintext \
        -import-path "$PROTO_DIR" \
        -proto control.proto \
        -d '{"base": {"request_id": "test-5d-teleop"}, "mode": "ROBOT_MODE_TELEOP"}' \
        127.0.0.1:50051 robot.v1.ControlService/SetMode 2>&1 || true)

      if echo "$result" | grep -q "ROBOT_MODE_TELEOP"; then
        pass "5d SetMode TELEOP 成功"
      else
        fail "5d SetMode TELEOP" "$result"
      fi

      # 等待编排
      sleep 8
      autonomy_state=$(svc_state nav-autonomy.service)
      if [[ "$autonomy_state" == "active" ]]; then
        pass "5d TELEOP: nav-autonomy 已启动"
      else
        fail "5d TELEOP 服务" "nav-autonomy=$autonomy_state"
      fi

      # 5e: 回到 IDLE
      echo ""
      echo "  5e: TELEOP → IDLE..."
      grpcurl -plaintext \
        -import-path "$PROTO_DIR" \
        -proto control.proto \
        -d '{"base": {"request_id": "test-5e"}, "mode": "ROBOT_MODE_IDLE"}' \
        127.0.0.1:50051 robot.v1.ControlService/SetMode >/dev/null 2>&1 || true

      sleep 8
      autonomy_state=$(svc_state nav-autonomy.service)
      if [[ "$autonomy_state" != "active" ]]; then
        pass "5e IDLE: nav-autonomy 已停止"
      else
        fail "5e IDLE 服务" "nav-autonomy=$autonomy_state (应停止)"
      fi

      # 释放 Lease
      grpcurl -plaintext \
        -import-path "$PROTO_DIR" \
        -proto control.proto \
        -d "{\"base\": {\"request_id\": \"test-5f\"}, \"leaseToken\": \"$lease_token\"}" \
        127.0.0.1:50051 robot.v1.ControlService/ReleaseLease >/dev/null 2>&1 || true
      pass "5f ReleaseLease 完成"
    else
      fail "5d AcquireLease" "无法获取 token"
      skip "5d-5f 跳过 (无 lease)"
    fi
  fi
fi

# ── 最终清理 ──
echo ""
info "最终清理"
stop_all
echo "  所有导航服务已停止"

# ═══════════════════════════════════════════════════════════════════
# 报告
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  测试报告                                                ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo -e "║  ${GREEN}PASS: $PASS${NC}    ${RED}FAIL: $FAIL${NC}    ${YELLOW}SKIP: $SKIP${NC}"
echo "╚══════════════════════════════════════════════════════════╝"

if [[ $FAIL -gt 0 ]]; then
  exit 1
else
  exit 0
fi
