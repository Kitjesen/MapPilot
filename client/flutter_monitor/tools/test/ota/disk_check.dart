/// 查看机器人磁盘使用情况
/// dart run tools/test/ota/disk_check.dart [host] [port]
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50052;

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final ota = OtaServiceClient(ch);

  try {
    final info = await ota.getDeviceInfo(Empty(),
        options: CallOptions(timeout: const Duration(seconds: 5)));

    final totalBytes = info.diskTotalBytes.toInt();
    final freeBytes = info.diskFreeBytes.toInt();
    final usedBytes = totalBytes - freeBytes;
    final usedPct = totalBytes > 0 ? (usedBytes / totalBytes * 100) : 0.0;

    print('╔══════════════════════════════════════════════╗');
    print('║         Robot Disk Usage Report              ║');
    print('╠══════════════════════════════════════════════╣');
    print('║  hostname:  ${info.hostname.padRight(32)}║');
    print('║  robotId:   ${info.robotId.padRight(32)}║');
    print('║  OS:        ${info.osVersion.padRight(32)}║');
    print('║  OTA ver:   ${info.otaDaemonVersion.padRight(32)}║');
    print('║  uptime:    ${_fmtUptime(info.uptimeSeconds.toInt()).padRight(32)}║');
    print('╠══════════════════════════════════════════════╣');
    print('║  总容量:    ${_fmtSize(totalBytes).padRight(32)}║');
    print('║  已使用:    ${('${_fmtSize(usedBytes)} (${usedPct.toStringAsFixed(1)}%)').padRight(32)}║');
    print('║  可用:      ${_fmtSize(freeBytes).padRight(32)}║');
    print('╠══════════════════════════════════════════════╣');

    // Visual bar
    final barLen = 30;
    final filled = (usedPct / 100 * barLen).round().clamp(0, barLen);
    final bar = '${'█' * filled}${'░' * (barLen - filled)}';
    print('║  [$bar] ${usedPct.toStringAsFixed(0)}%  ║');
    print('╚══════════════════════════════════════════════╝');

    // List key directories
    print('\n各白名单目录大小:');
    final dirs = [
      '/home/sunrise/models',
      '/home/sunrise/data/SLAM/navigation/maps',
      '/home/sunrise/data/SLAM/navigation/install',
      '/home/sunrise/data/SLAM/navigation/src/global_planning',
      '/opt/robot/ota/staging',
      '/opt/robot/firmware',
    ];

    for (final dir in dirs) {
      try {
        final req = ListRemoteFilesRequest()
          ..base = (RequestBase()..requestId = 'ls')
          ..directory = dir;
        final resp = await ota.listRemoteFiles(req,
            options: CallOptions(timeout: const Duration(seconds: 3)));
        final fileCount = resp.files.length;
        final totalSize = resp.files.fold<int>(0, (sum, f) => sum + f.size.toInt());
        print('  ${dir.padRight(55)} ${fileCount.toString().padLeft(3)} files  ${_fmtSize(totalSize).padLeft(10)}');
      } on GrpcError {
        print('  ${dir.padRight(55)} (不存在或无权限)');
      }
    }
  } catch (e) {
    print('Error: $e');
  }

  await ch.shutdown();
  exit(0);
}

String _fmtSize(int bytes) {
  if (bytes <= 0) return '0 B';
  if (bytes < 1024) return '$bytes B';
  if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
  if (bytes < 1024 * 1024 * 1024) return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
  return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(2)} GB';
}

String _fmtUptime(int seconds) {
  final d = seconds ~/ 86400;
  final h = (seconds % 86400) ~/ 3600;
  final m = (seconds % 3600) ~/ 60;
  if (d > 0) return '${d}天 ${h}时 ${m}分';
  if (h > 0) return '${h}时 ${m}分';
  return '${m}分';
}
