/// 测试 TelemetryService.StreamFastState (位姿/RPY/关节/IMU)
/// dart run tools/test/telemetry/fast_state.dart [host] [port] [seconds]
import 'dart:async';
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/telemetry.pbgrpc.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  final dur = args.length > 2 ? int.parse(args[2]) : 5;
  print('=== FastState Test | $host:$port | ${dur}s ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = TelemetryServiceClient(ch);
  int count = 0;
  bool hasIMU = false, hasPose = false, hasRPY = false;
  final sw = Stopwatch()..start();

  try {
    await for (final s in c.streamFastState(FastStateRequest()..desiredHz = 10.0,
        options: CallOptions(timeout: Duration(seconds: dur + 3)))) {
      count++;
      if (s.pose.position.x != 0 || s.pose.position.y != 0) hasPose = true;
      if (s.hasLinearAcceleration() && s.linearAcceleration.z != 0) hasIMU = true;
      if (s.hasRpyDeg() && s.rpyDeg.x != 0) hasRPY = true;
      if (count == 1) {
        print('Frame 1:');
        print('  Pose: (${s.pose.position.x.toStringAsFixed(3)}, ${s.pose.position.y.toStringAsFixed(3)}, ${s.pose.position.z.toStringAsFixed(3)})');
        print('  RPY:  (${s.rpyDeg.x.toStringAsFixed(1)}, ${s.rpyDeg.y.toStringAsFixed(1)}, ${s.rpyDeg.z.toStringAsFixed(1)}) deg');
        print('  Vel:  (${s.velocity.linear.x.toStringAsFixed(3)}, ${s.velocity.linear.y.toStringAsFixed(3)}, ${s.velocity.linear.z.toStringAsFixed(3)}) m/s');
        print('  Accel: (${s.linearAcceleration.x.toStringAsFixed(2)}, ${s.linearAcceleration.y.toStringAsFixed(2)}, ${s.linearAcceleration.z.toStringAsFixed(2)}) m/s2');
        print('  Gyro:  (${s.angularVelocity.x.toStringAsFixed(2)}, ${s.angularVelocity.y.toStringAsFixed(2)}, ${s.angularVelocity.z.toStringAsFixed(2)}) rad/s');
        print('  TF OK: ${s.tfOk}');
      }
      if (sw.elapsedMilliseconds > dur * 1000) break;
    }
  } on GrpcError catch (_) { /* deadline exceeded — normal */ }
  on TimeoutException { /* normal */ }

  sw.stop();
  final elapsed = sw.elapsedMilliseconds / 1000.0;
  print('\n$count frames (${(count / elapsed).toStringAsFixed(1)} Hz)');
  print('Pose: ${hasPose ? "YES" : "NO"}  RPY: ${hasRPY ? "YES" : "NO"}  IMU: ${hasIMU ? "YES" : "NO"}');
  await ch.shutdown();
  exit(0);
}
