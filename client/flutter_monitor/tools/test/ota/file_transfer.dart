/// 文件传输全链路测试 — OTA Daemon (port 50052)
///
/// 测试内容:
///   1. GetDeviceInfo — 连通性检查
///   2. ListRemoteFiles — 列出远程目录
///   3. UploadFile — 上传测试文件
///   4. ListRemoteFiles — 确认文件已上传
///   5. DeleteRemoteFile — 删除测试文件
///   6. ListRemoteFiles — 确认文件已删除
///
/// 用法:
///   dart run tools/test/ota/file_transfer.dart [host] [port]
///   默认: 192.168.66.190:50052
import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:fixnum/fixnum.dart';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50052;

  print('╔══════════════════════════════════════════════════╗');
  print('║   OTA File Transfer Test | $host:$port        ║');
  print('╚══════════════════════════════════════════════════╝\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final ota = OtaServiceClient(ch);

  int passed = 0;
  int failed = 0;

  // ─── Test 1: GetDeviceInfo (连通性) ───
  print('━━━ Test 1: GetDeviceInfo (连通性检查) ━━━');
  try {
    final info = await ota.getDeviceInfo(Empty(),
        options: CallOptions(timeout: const Duration(seconds: 5)));
    print('  ✓ OTA Daemon 在线');
    print('    hostname: ${info.hostname}');
    print('    robotId:  ${info.robotId}');
    print('    version:  ${info.otaDaemonVersion}');
    print('    uptime:   ${info.uptimeSeconds}s');
    passed++;
  } on GrpcError catch (e) {
    print('  ✗ 连接失败: ${e.codeName} — ${e.message}');
    print('\n⚠ OTA Daemon 不可达, 后续测试无法继续。');
    print('  请确认机器人上 ota_daemon 已启动:');
    print('    ssh sunrise@$host "systemctl status ota_daemon"');
    await ch.shutdown();
    exit(1);
  }

  // ─── Test 2: ListRemoteFiles (列目录) ───
  // OTA 白名单: /home/sunrise/models, .../maps, .../staging 等
  const testDir = '/opt/robot/ota/staging';
  print('\n━━━ Test 2: ListRemoteFiles (列出 $testDir) ━━━');
  try {
    final req = ListRemoteFilesRequest()
      ..base = (RequestBase()..requestId = 'ls-${_rid()}')
      ..directory = testDir;
    final resp = await ota.listRemoteFiles(req,
        options: CallOptions(timeout: const Duration(seconds: 5)));
    print('  ✓ 共 ${resp.files.length} 个文件/目录');
    print('    总空间: ${_fmt(resp.totalSize.toInt())}  可用: ${_fmt(resp.freeSpace.toInt())}');
    if (resp.files.isNotEmpty) {
      final show = resp.files.take(5);
      for (final f in show) {
        print('    ${f.filename.padRight(30)} ${_fmt(f.size.toInt()).padLeft(10)}  ${f.category}');
      }
      if (resp.files.length > 5) print('    ... 还有 ${resp.files.length - 5} 个');
    }
    passed++;
  } on GrpcError catch (e) {
    print('  ✗ ListRemoteFiles 失败: ${e.codeName} — ${e.message}');
    failed++;
  }

  // ─── Test 2b: ListRemoteFiles (maps 目录) ───
  const mapsDir = '/home/sunrise/data/SLAM/navigation/maps';
  print('\n━━━ Test 2b: ListRemoteFiles (列出 maps) ━━━');
  try {
    final req = ListRemoteFilesRequest()
      ..base = (RequestBase()..requestId = 'ls-${_rid()}')
      ..directory = mapsDir;
    final resp = await ota.listRemoteFiles(req,
        options: CallOptions(timeout: const Duration(seconds: 5)));
    print('  ✓ 共 ${resp.files.length} 个文件/目录');
    if (resp.files.isNotEmpty) {
      for (final f in resp.files.take(8)) {
        print('    ${f.filename.padRight(30)} ${_fmt(f.size.toInt()).padLeft(10)}  ${f.category}');
      }
      if (resp.files.length > 8) print('    ... 还有 ${resp.files.length - 8} 个');
    }
    passed++;
  } on GrpcError catch (e) {
    print('  ✗ ListRemoteFiles 失败: ${e.codeName} — ${e.message}');
    failed++;
  }

  // ─── Test 3: UploadFile (上传测试文件) ───
  final testFilename = 'flutter_test_${DateTime.now().millisecondsSinceEpoch}.txt';
  final testRemotePath = '$testDir/$testFilename';
  final testContent = 'Hello from Flutter file transfer test!\n'
      'Timestamp: ${DateTime.now().toIso8601String()}\n'
      'Random: ${Random().nextInt(999999)}\n';
  final testBytes = testContent.codeUnits;

  print('\n━━━ Test 3: UploadFile (上传 $testFilename, ${testBytes.length} bytes) ━━━');
  try {
    final controller = StreamController<UploadFileChunk>();

    // 第一个 chunk: metadata
    final meta = UploadFileMetadata()
      ..base = (RequestBase()..requestId = 'up-${_rid()}')
      ..remotePath = testRemotePath
      ..filename = testFilename
      ..totalSize = Int64(testBytes.length)
      ..overwrite = true
      ..category = 'config';

    controller.add(UploadFileChunk()..metadata = meta);

    // 第二个 chunk: data
    controller.add(UploadFileChunk()..data = testBytes);

    // 关闭流
    controller.close();

    final resp = await ota.uploadFile(controller.stream,
        options: CallOptions(timeout: const Duration(seconds: 10)));
    if (resp.success) {
      print('  ✓ 上传成功');
      print('    remotePath: ${resp.remotePath}');
      print('    message:    ${resp.message}');
      passed++;
    } else {
      print('  ✗ 上传失败: ${resp.message}');
      failed++;
    }
  } on GrpcError catch (e) {
    print('  ✗ UploadFile gRPC 错误: ${e.codeName} — ${e.message}');
    failed++;
  } catch (e) {
    print('  ✗ UploadFile 异常: $e');
    failed++;
  }

  // ─── Test 4: ListRemoteFiles (验证上传) ───
  print('\n━━━ Test 4: ListRemoteFiles (验证文件存在) ━━━');
  bool fileFound = false;
  try {
    final req = ListRemoteFilesRequest()
      ..base = (RequestBase()..requestId = 'ls-${_rid()}')
      ..directory = testDir;
    final resp = await ota.listRemoteFiles(req,
        options: CallOptions(timeout: const Duration(seconds: 5)));
    fileFound = resp.files.any((f) => f.filename == testFilename);
    if (fileFound) {
      final f = resp.files.firstWhere((f) => f.filename == testFilename);
      print('  ✓ 文件已确认存在');
      print('    size: ${f.size} bytes  category: ${f.category}');
      passed++;
    } else {
      print('  ✗ 文件未找到 (可能上传路径不同)');
      print('    目录中共 ${resp.files.length} 个文件');
      failed++;
    }
  } on GrpcError catch (e) {
    print('  ✗ ListRemoteFiles 失败: ${e.codeName} — ${e.message}');
    failed++;
  }

  // ─── Test 5: DeleteRemoteFile (删除测试文件) ───
  print('\n━━━ Test 5: DeleteRemoteFile (清理测试文件) ━━━');
  try {
    final req = DeleteRemoteFileRequest()
      ..base = (RequestBase()..requestId = 'del-${_rid()}')
      ..remotePath = testRemotePath;
    final resp = await ota.deleteRemoteFile(req,
        options: CallOptions(timeout: const Duration(seconds: 5)));
    if (resp.success) {
      print('  ✓ 删除成功: ${resp.message}');
      passed++;
    } else {
      print('  ✗ 删除失败: ${resp.message}');
      failed++;
    }
  } on GrpcError catch (e) {
    print('  ✗ DeleteRemoteFile gRPC 错误: ${e.codeName} — ${e.message}');
    failed++;
  }

  // ─── Test 6: 确认删除 ───
  print('\n━━━ Test 6: ListRemoteFiles (确认删除) ━━━');
  try {
    final req = ListRemoteFilesRequest()
      ..base = (RequestBase()..requestId = 'ls-${_rid()}')
      ..directory = testDir;
    final resp = await ota.listRemoteFiles(req,
        options: CallOptions(timeout: const Duration(seconds: 5)));
    final stillExists = resp.files.any((f) => f.filename == testFilename);
    if (!stillExists) {
      print('  ✓ 文件已确认删除');
      passed++;
    } else {
      print('  ✗ 文件仍然存在');
      failed++;
    }
  } on GrpcError catch (e) {
    print('  ✗ ListRemoteFiles 失败: ${e.codeName} — ${e.message}');
    failed++;
  }

  // ─── 汇总 ───
  print('\n╔══════════════════════════════════════════════════╗');
  print('║   结果: $passed passed, $failed failed                       ║');
  print('╚══════════════════════════════════════════════════╝');

  await ch.shutdown();
  exit(failed > 0 ? 1 : 0);
}

String _rid() => Random().nextInt(99999).toString().padLeft(5, '0');

String _fmt(int bytes) {
  if (bytes <= 0) return '--';
  if (bytes < 1024) return '$bytes B';
  if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
  if (bytes < 1024 * 1024 * 1024) return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
  return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(1)} GB';
}
