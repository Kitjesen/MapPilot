import 'dart:convert';
import 'package:flutter/material.dart';

/// 机器人型号配置
///
/// 定义一种机器人的所有参数：名称、URDF、关节数、默认端口等。
/// 每个 profile 对应一种物理机器人型号。
class RobotProfile {
  /// 唯一标识 (e.g. 'thunder', 'mini', 'mini2')
  final String id;

  /// 显示名称
  final String name;

  /// 型号副标题
  final String subtitle;

  /// 对应 proto 的 RobotType 枚举名
  final String protoType;

  /// URDF 文件在 assets 中的路径 (null = 通用模型)
  final String? urdfAsset;

  /// 关节数量 (12 = 3-DOF legs, 16 = 4-DOF legs)
  final int jointCount;

  /// 腿的顺序名称
  final List<String> legNames;

  /// 每条腿的关节名称
  final List<String> jointNames;

  /// 默认 CMS gRPC 端口
  final int defaultPort;

  /// 默认 IP 地址
  final String defaultHost;

  /// 最大线速度 (m/s)
  final double maxLinearSpeed;

  /// 最大角速度 (rad/s)
  final double maxAngularSpeed;

  /// 机器人质量 (kg)
  final double mass;

  /// 主题色
  final Color themeColor;

  /// 图标
  final IconData icon;

  /// 描述
  final String description;

  /// 初始关节角度 (rad), 用作 idle 姿态 (可为空, 由 GetParams 获取)
  final List<double>? defaultJointPositions;

  /// 是否可用 (false = 开发中/即将推出)
  final bool available;

  const RobotProfile({
    required this.id,
    required this.name,
    required this.subtitle,
    required this.protoType,
    this.urdfAsset,
    this.jointCount = 16,
    this.legNames = const ['FR', 'FL', 'RR', 'RL'],
    this.jointNames = const ['Hip', 'Thigh', 'Calf', 'Foot'],
    this.defaultPort = 13145,
    this.defaultHost = '192.168.66.190',
    this.maxLinearSpeed = 1.0,
    this.maxAngularSpeed = 1.0,
    this.mass = 12.0,
    this.themeColor = const Color(0xFF007AFF),
    this.icon = Icons.smart_toy_outlined,
    this.description = '',
    this.defaultJointPositions,
    this.available = true,
  });

  /// 每条腿有几个关节
  int get jointsPerLeg => jointCount ~/ legNames.length;

  /// JSON 序列化 (存储选中的 profile id)
  Map<String, dynamic> toJson() => {
        'id': id,
        'name': name,
        'defaultHost': defaultHost,
        'defaultPort': defaultPort,
      };

  static RobotProfile? fromJson(Map<String, dynamic> json, List<RobotProfile> registry) {
    final id = json['id'] as String?;
    if (id == null) return null;
    try {
      final profile = registry.firstWhere((p) => p.id == id);
      return profile;
    } catch (_) {
      return null;
    }
  }

  @override
  bool operator ==(Object other) =>
      identical(this, other) || other is RobotProfile && id == other.id;

  @override
  int get hashCode => id.hashCode;
}

/// 所有预定义机器人型号
class RobotProfiles {
  RobotProfiles._();

  static const thunder = RobotProfile(
    id: 'thunder',
    name: 'Thunder',
    subtitle: 'Han Dog 标准版',
    protoType: 'HAN',
    urdfAsset: 'urdf/轮足狗机器人v3.urdf',
    jointCount: 16,
    legNames: ['FR', 'FL', 'RR', 'RL'],
    jointNames: ['Hip', 'Thigh', 'Calf', 'Foot'],
    defaultPort: 13145,
    defaultHost: '192.168.66.190',
    maxLinearSpeed: 3.0,
    maxAngularSpeed: 1.5,
    mass: 40.0,
    themeColor: Color(0xFFFF9500),
    icon: Icons.pets,
    description: '四足轮足混合机器人，支持 RL 步态控制。\n'
        '16 关节 (4×4)，带轮足切换。',
    available: true,
  );

  static const mini = RobotProfile(
    id: 'mini',
    name: 'Mini',
    subtitle: 'Mini Dog 小型版',
    protoType: 'MINI',
    urdfAsset: null, // 待添加
    jointCount: 12,
    legNames: ['FR', 'FL', 'RR', 'RL'],
    jointNames: ['Hip', 'Thigh', 'Calf'],
    defaultPort: 13145,
    defaultHost: '192.168.66.190',
    maxLinearSpeed: 0.6,
    maxAngularSpeed: 1.0,
    mass: 5.0,
    themeColor: Color(0xFF5856D6),
    icon: Icons.cruelty_free,
    description: '轻量级四足机器人，12 关节 (4×3)。\n'
        '适合教学和室内巡检。',
    available: false,
  );

  static const mini2 = RobotProfile(
    id: 'mini2',
    name: 'Mini 2',
    subtitle: 'Mini Dog 二代',
    protoType: 'MINI2',
    urdfAsset: null,
    jointCount: 12,
    legNames: ['FR', 'FL', 'RR', 'RL'],
    jointNames: ['Hip', 'Thigh', 'Calf'],
    defaultPort: 13145,
    defaultHost: '192.168.66.190',
    maxLinearSpeed: 0.8,
    maxAngularSpeed: 1.2,
    mass: 6.0,
    themeColor: Color(0xFF34C759),
    icon: Icons.cruelty_free,
    description: '二代小型四足，改进电机和结构。\n'
        '更强动力，更轻框架。',
    available: false,
  );

  static const skinny = RobotProfile(
    id: 'skinny',
    name: 'Skinny',
    subtitle: '原型机',
    protoType: 'SKINNY',
    urdfAsset: null,
    jointCount: 16,
    legNames: ['FR', 'FL', 'RR', 'RL'],
    jointNames: ['Hip', 'Thigh', 'Calf', 'Foot'],
    defaultPort: 13145,
    defaultHost: '192.168.66.190',
    maxLinearSpeed: 0.5,
    maxAngularSpeed: 0.8,
    mass: 10.0,
    themeColor: Color(0xFFFF3B30),
    icon: Icons.science_outlined,
    description: '初始原型机，用于算法验证。\n'
        '16 关节，基础步态。',
    available: false,
  );

  /// 所有已注册的型号
  static const List<RobotProfile> all = [thunder, mini, mini2, skinny];

  /// 通过 id 查找
  static RobotProfile? byId(String id) {
    try {
      return all.firstWhere((p) => p.id == id);
    } catch (_) {
      return null;
    }
  }

  /// 通过 proto RobotType 名称匹配
  static RobotProfile? byProtoType(String protoType) {
    try {
      return all.firstWhere(
          (p) => p.protoType.toUpperCase() == protoType.toUpperCase());
    } catch (_) {
      return null;
    }
  }
}
