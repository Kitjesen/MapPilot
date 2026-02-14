# Proto Regeneration Playbook (Dart)

用于收口 v1.2.0 期间的 Dart proto 手工改动，确保后续统一由脚本生成。

## 1. 适用范围

- Proto 源：`src/robot_proto/proto/*.proto`
- Dart 产物：`src/robot_proto/dart/lib/src/*.dart`
- 生成脚本：
  - **Linux/macOS**: `scripts/proto_gen.sh`
  - **Windows (PowerShell)**: `scripts/proto_gen.ps1`

## 2. 推荐执行环境

### Linux / macOS

- Ubuntu 22.04+ / Debian 12+ / macOS 13+
- 已安装 `protobuf-compiler`、`dart`（或 flutter 自带 dart）

### Windows

- Windows 10/11, PowerShell 5.1+
- 已安装 `protoc`（[GitHub Releases](https://github.com/protocolbuffers/protobuf/releases) 或 `choco install protoc`）
- 已安装 `dart`（随 Flutter SDK 或独立安装）

## 3. 一次性准备

### Linux / macOS

```bash
sudo apt update
sudo apt install -y protobuf-compiler
dart pub global activate protoc_plugin
```

如 `protoc-gen-dart` 不在 PATH，请加入：

```bash
export PATH="$PATH:$HOME/.pub-cache/bin"
```

### Windows (PowerShell)

```powershell
# 安装 protoc（二选一）
choco install protoc          # 或手动下载解压并加入 PATH
# 安装 protoc-gen-dart
dart pub global activate protoc_plugin
# 脚本会自动将 %LOCALAPPDATA%\Pub\Cache\bin 加入当次 PATH
```

## 4. 执行流程

### Linux / macOS

```bash
./scripts/proto_gen.sh --check
./scripts/proto_gen.sh
```

### Windows (PowerShell)

```powershell
.\scripts\proto_gen.ps1 -Check
.\scripts\proto_gen.ps1
```

## 5. 结果核对

- 目标目录：`src/robot_proto/dart/lib/src/`
- 关键文件应更新：
  - `control.pb.dart`
  - `control.pbenum.dart`
  - `control.pbgrpc.dart`
- 核对是否包含：
  - `WaypointSource`
  - `ActiveWaypoint`
  - `GetActiveWaypoints*`
  - `ClearWaypoints*`

## 6. 回归验证

- Flutter 侧执行最小回归：`docs/REGRESSION_CHECKLIST_V1_2.md`
- 至少覆盖：
  - 任务启动与冲突清除
  - TaskPanel 航点信息刷新
  - E-stop 联动
  - 断线重连恢复

## 7. 注意事项

- 不要继续手工编辑 `*.pb*.dart`，会被脚本覆盖
- 如脚本失败，优先修复环境（`protoc`/`protoc-gen-dart`）后重试
