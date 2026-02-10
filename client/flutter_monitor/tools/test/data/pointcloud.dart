/// 测试 DataService.Subscribe 点云 + 地形图流
/// dart run tools/test/data/pointcloud.dart [host] [port] [seconds]
///
/// 并行订阅多个流 (每个流独立超时, 不会互相阻塞):
///   1. local_cloud   → /cloud_registered   — 实时点云
///   2. /terrain_map  → /terrain_map         — 地形分析 (bag/实时)
///   3. terrain       → /terrain_map_ext     — 扩展地形 (仅实时)
import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';

Future<Map<String, dynamic>> testStream(
    DataServiceClient c, String name, ResourceType type, double freq, int durSec) async {
  final rid = Random().nextInt(999999).toString().padLeft(6, '0');
  final req = SubscribeRequest()
    ..base = (RequestBase()..requestId = '$name-$rid')
    ..resourceId = (ResourceId()
      ..name = name
      ..type = type)
    ..profile = (SubscribeProfile()..frequency = freq);

  int count = 0, totalBytes = 0;
  int minSize = 1 << 30, maxSize = 0;
  final sw = Stopwatch()..start();
  final deadline = Duration(seconds: durSec + 5); // hard deadline

  try {
    await for (final chunk in c.subscribe(req,
        options: CallOptions(timeout: deadline))) {
      count++;
      final sz = chunk.data.length;
      totalBytes += sz;
      if (sz < minSize) minSize = sz;
      if (sz > maxSize) maxSize = sz;
      if (sw.elapsedMilliseconds > durSec * 1000) break;
    }
  } on GrpcError catch (e) {
    // DEADLINE_EXCEEDED is expected if no data arrived within timeout
    if (e.code != StatusCode.deadlineExceeded) {
      return {'name': name, 'count': count, 'error': e.toString()};
    }
  } catch (e) {
    return {'name': name, 'count': count, 'error': e.toString()};
  }

  sw.stop();
  final elapsed = sw.elapsedMilliseconds / 1000.0;
  return {
    'name': name,
    'count': count,
    'fps': count > 0 ? count / elapsed : 0.0,
    'totalKB': totalBytes / 1024.0,
    'avgKB': count > 0 ? totalBytes / count / 1024.0 : 0,
    'minKB': count > 0 ? minSize / 1024.0 : 0,
    'maxKB': maxSize / 1024.0,
    'elapsed': elapsed,
  };
}

void printResult(Map<String, dynamic> r) {
  final name = (r['name'] as String).padRight(20);
  if (r.containsKey('error')) {
    print('  $name [FAIL] ${r['error']}');
    return;
  }
  final count = r['count'] as int;
  if (count == 0) {
    print('  $name [FAIL] 0 frames');
    return;
  }
  final fps = (r['fps'] as double).toStringAsFixed(1);
  final avg = (r['avgKB'] as double).toStringAsFixed(1);
  final min = (r['minKB'] as double).toStringAsFixed(1);
  final max = (r['maxKB'] as double).toStringAsFixed(1);
  final total = (r['totalKB'] as double).toStringAsFixed(0);
  print('  $name [PASS] $count frames | $fps fps | avg $avg KB (${min}~${max}) | total ${total} KB');
}

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  final dur = args.length > 2 ? int.parse(args[2]) : 10;
  print('=== PointCloud & Terrain Test | $host:$port | ${dur}s ===\n');

  final ch = ClientChannel(host, port: port,
      options: ChannelOptions(
        credentials: const ChannelCredentials.insecure(),
        connectionTimeout: const Duration(seconds: 5),
      ));
  final c = DataServiceClient(ch);

  // 并行运行多个流测试 (每个有独立超时, 不会互相阻塞)
  final results = await Future.wait([
    testStream(c, 'local_cloud', ResourceType.RESOURCE_TYPE_POINTCLOUD, 15.0, dur),
    testStream(c, '/terrain_map', ResourceType.RESOURCE_TYPE_POINTCLOUD, 15.0, dur),
    testStream(c, 'terrain', ResourceType.RESOURCE_TYPE_POINTCLOUD, 15.0, dur),
  ]);

  print('Results:');
  for (final r in results) {
    printResult(r);
  }

  final passed = results.where((r) => (r['count'] as int) > 0).length;
  print('\n$passed / ${results.length} streams OK');

  await ch.shutdown();
  exit(passed > 0 ? 0 : 1);
}
