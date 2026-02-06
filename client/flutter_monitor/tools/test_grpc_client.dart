#!/usr/bin/env dart

import 'dart:io';
import 'package:grpc/grpc.dart';
import 'lib/generated/telemetry.pbgrpc.dart';
import 'lib/generated/system.pbgrpc.dart';
import 'lib/generated/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  
  print('🚀 连接到机器人 gRPC 服务: $host:$port');
  print('─' * 60);
  
  final channel = ClientChannel(
    host,
    port: port,
    options: const ChannelOptions(
      credentials: ChannelCredentials.insecure(),
    ),
  );
  
  final telemetryClient = TelemetryServiceClient(channel);
  final systemClient = SystemServiceClient(channel);
  
  try {
    // 1. 获取机器人信息
    print('\n📋 获取机器人信息...');
    final info = await systemClient.getRobotInfo(
      Empty(),
      options: CallOptions(timeout: Duration(seconds: 5)),
    );
    print('   ✓ 机器人 ID: ${info.robotId}');
    print('   ✓ 显示名称: ${info.displayName}');
    print('   ✓ 固件版本: ${info.firmwareVersion}');
    print('   ✓ 软件版本: ${info.softwareVersion}');
    
    // 2. 获取机器人能力
    print('\n🔧 获取机器人能力...');
    final capabilities = await systemClient.getCapabilities(
      Empty(),
      options: CallOptions(timeout: Duration(seconds: 5)),
    );
    print('   ✓ 支持的资源: ${capabilities.supportedResources.join(", ")}');
    print('   ✓ 支持的任务: ${capabilities.supportedTasks.join(", ")}');
    print('   ✓ 支持遥控: ${capabilities.teleopSupported}');
    print('   ✓ 支持建图: ${capabilities.mappingSupported}');
    
    // 3. 订阅快速状态流（10秒）
    print('\n📡 订阅快速状态流 (10秒)...');
    final request = FastStateRequest()..desiredHz = 10.0;
    var count = 0;
    var startTime = DateTime.now();
    
    await for (final state in telemetryClient.streamFastState(request)
        .timeout(Duration(seconds: 10))) {
      count++;
      if (count == 1 || count % 10 == 0) {
        final elapsed = DateTime.now().difference(startTime).inMilliseconds / 1000.0;
        final actualHz = count / elapsed;
        print('   [$count] 位姿: (${state.pose.position.x.toStringAsFixed(2)}, '
            '${state.pose.position.y.toStringAsFixed(2)}, ${state.pose.position.z.toStringAsFixed(2)}) | '
            '速度: ${state.velocity.linear.x.toStringAsFixed(2)} m/s | '
            '频率: ${actualHz.toStringAsFixed(1)} Hz');
      }
    }
    
    print('\n✅ 测试成功！');
    print('   • 总共接收: $count 条消息');
    print('   • 平均频率: ${(count / 10.0).toStringAsFixed(1)} Hz');
    
  } catch (e, stackTrace) {
    print('\n❌ 错误: $e');
    if (e is GrpcError) {
      print('   gRPC 状态码: ${e.code}');
      print('   消息: ${e.message}');
    }
    print('\n堆栈跟踪:');
    print(stackTrace);
    exit(1);
  } finally {
    await channel.shutdown();
    print('\n🔌 已断开连接');
  }
}
