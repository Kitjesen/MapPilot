/// 测试 OTA 相关功能
/// dart run tools/test/ota/device_info.dart [host] [port]
///
/// 测试内容:
///   1. DownloadFile via DataService (port 50051) — 地图下载
///   2. OTA GetDeviceInfo (需 grpcurl 单独测, OtaService 未在 Dart proto 包中)
import 'dart:io';
import 'dart:math';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  print('=== OTA & File Transfer Test | $host:$port ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = DataServiceClient(ch);

  // Test 1a: DownloadFile — 小地图 (first_map, 19KB)
  print('--- Test 1a: DownloadFile (first_map/map.pcd, ~19KB) ---');
  await _testDownload(c, '/home/sunrise/data/SLAM/navigation/maps/first_map/map.pcd', 10);

  // Test 1b: DownloadFile — 中等地图 (spiral, 3.6MB)
  print('\n--- Test 1b: DownloadFile (spiral0.3_2.pcd, ~3.6MB) ---');
  await _testDownload(c, '/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd/spiral0.3_2.pcd', 30);

  // Test 1c: DownloadFile — 大地图 (building, 46MB)
  print('\n--- Test 1c: DownloadFile (building2_9.pcd, ~46MB) ---');
  await _testDownload(c, '/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd/building2_9.pcd', 120);

  // Test 1d: DownloadFile — Tomogram pickle (624KB)
  print('\n--- Test 1d: DownloadFile (building2_9.pickle, ~624KB) ---');
  await _testDownload(c, '/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd/building2_9.pickle', 15);

  // Test 2: DownloadFile — 不存在的文件
  print('\n--- Test 2: DownloadFile (non-existent) ---');
  try {
    final req = DownloadFileRequest()
      ..base = (RequestBase()..requestId = 'dl-bad')
      ..filePath = '/tmp/no_such_file_12345.pcd';

    await for (final _ in c.downloadFile(req,
        options: CallOptions(timeout: const Duration(seconds: 5)))) {
      // should not get here
    }
    print('  [FAIL] Expected error but got data');
  } on GrpcError catch (e) {
    print('  [PASS] Correctly rejected: ${e.code} ${e.message}');
  }

  // Test 3: ListResources — 确认可用资源
  print('\n--- Test 3: ListResources ---');
  try {
    final resp = await c.listResources(
        Empty(),
        options: CallOptions(timeout: const Duration(seconds: 5)));
    print('  [PASS] ${resp.resources.length} resources');
    for (final r in resp.resources) {
      print('    ${r.id.name} (${r.id.type}) avail=${r.available}');
    }
  } catch (e) {
    print('  [FAIL] $e');
  }

  print('\n--- OTA Daemon (port 50052) ---');
  print('  NOTE: OtaServiceClient not in Dart proto package.');
  print('  Test via grpcurl:');
  print('    grpcurl -plaintext -import-path src/robot_proto/proto -proto data.proto \\');
  print('      $host:50052 robot.v1.OtaService/GetDeviceInfo');

  await ch.shutdown();
  exit(0);
}

Future<void> _testDownload(DataServiceClient c, String filePath, int timeoutSec) async {
  final sw = Stopwatch()..start();
  try {
    final req = DownloadFileRequest()
      ..base = (RequestBase()..requestId = 'dl-${Random().nextInt(99999)}')
      ..filePath = filePath
      ..chunkSize = 65536;

    int totalBytes = 0;
    int chunks = 0;
    bool isLast = false;
    int? totalSize;

    await for (final chunk in c.downloadFile(req,
        options: CallOptions(timeout: Duration(seconds: timeoutSec)))) {
      chunks++;
      totalBytes += chunk.data.length;
      if (chunk.totalSize > 0) totalSize = chunk.totalSize.toInt();
      if (chunk.isLast) isLast = true;
    }
    sw.stop();

    if (totalBytes > 0 && isLast) {
      final kb = totalBytes / 1024;
      final mb = kb / 1024;
      final sec = sw.elapsedMilliseconds / 1000.0;
      final speed = mb / sec;
      final sizeStr = mb >= 1.0 ? '${mb.toStringAsFixed(1)} MB' : '${kb.toStringAsFixed(1)} KB';
      print('  [PASS] $chunks chunks, $sizeStr in ${sec.toStringAsFixed(2)}s (${speed.toStringAsFixed(1)} MB/s)');
      if (totalSize != null) print('  Declared size: $totalSize bytes');
    } else {
      print('  [FAIL] $chunks chunks, $totalBytes bytes, isLast=$isLast');
    }
  } on GrpcError catch (e) {
    print('  [FAIL] gRPC error: ${e.code} ${e.message}');
  } catch (e) {
    print('  [FAIL] $e');
  }
}
