#!/usr/bin/env bash
# ============================================================
# proto_gen.sh — 一键从 proto 源文件生成 Dart 客户端代码
#
# Proto 唯一源头: src/robot_proto/proto/
# 生成目标:       src/robot_proto/dart/lib/src/
# Flutter 引用:   pubspec.yaml → path: ../../src/robot_proto/dart
#
# 用法:
#   ./scripts/proto_gen.sh          # 生成 Dart
#   ./scripts/proto_gen.sh --check  # 仅检查工具是否就绪
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

PROTO_DIR="$ROOT_DIR/src/robot_proto/proto"
DART_OUT="$ROOT_DIR/src/robot_proto/dart/lib/src"

PROTOS=(common.proto system.proto control.proto telemetry.proto data.proto)

# ---- colours ----
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }

# ---- pre-flight ----
check_tools() {
  command -v protoc >/dev/null 2>&1 || fail "protoc not found. Install: sudo apt install protobuf-compiler"
  export PATH="$PATH:$HOME/.pub-cache/bin"
  command -v protoc-gen-dart >/dev/null 2>&1 || fail "protoc-gen-dart not found. Install: dart pub global activate protoc_plugin"
  ok "protoc $(protoc --version 2>&1 | grep -oP '[\d.]+')"
  ok "protoc-gen-dart $(which protoc-gen-dart)"
}

# ---- generate ----
gen_dart() {
  info "Generating Dart gRPC stubs → robot_proto/dart/lib/src/"
  mkdir -p "$DART_OUT"

  local proto_args=()
  for p in "${PROTOS[@]}"; do
    [ -f "$PROTO_DIR/$p" ] || fail "Missing proto: $PROTO_DIR/$p"
    proto_args+=("$PROTO_DIR/$p")
  done

  protoc \
    --dart_out=grpc:"$DART_OUT" \
    -I "$PROTO_DIR" \
    "${proto_args[@]}"

  # 清理 well-known types 本地副本（Dart protobuf 包已自带）
  if [ -d "$DART_OUT/google" ]; then
    rm -rf "$DART_OUT/google"
    info "Removed generated google/ (using package:protobuf instead)"
  fi

  local count
  count=$(find "$DART_OUT" -maxdepth 1 -name "*.dart" | wc -l)
  ok "Generated $count Dart files"
}

# ---- main ----
main() {
  info "Proto source: $PROTO_DIR"
  info "Dart output:  $DART_OUT"
  echo

  check_tools
  echo

  if [ "${1:-}" = "--check" ]; then
    ok "All checks passed."
    exit 0
  fi

  gen_dart
  echo
  info "C++ side: colcon build --packages-select robot_proto remote_monitoring"
  echo
  ok "Done!"
}

main "$@"
