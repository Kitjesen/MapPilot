import 'dart:convert';

import 'package:flutter/material.dart';

/// 定时任务 — 绑定一个 TaskTemplate，按指定星期和时间自动执行。
///
/// 通过 [SettingsPreferences.scheduledTasks] 持久化。
class ScheduledTask {
  /// UUID
  final String id;

  /// 对应 TaskTemplate.name
  final String templateName;

  /// 执行星期：0=周日, 1=周一 ... 6=周六；空列表 = 每天
  final List<int> weekdays;

  /// 执行时间 (时:分)
  final TimeOfDay time;

  /// 是否启用
  final bool enabled;

  /// 创建时间 (ISO 8601)
  final String createdAt;

  const ScheduledTask({
    required this.id,
    required this.templateName,
    required this.weekdays,
    required this.time,
    this.enabled = true,
    required this.createdAt,
  });

  ScheduledTask copyWith({
    String? id,
    String? templateName,
    List<int>? weekdays,
    TimeOfDay? time,
    bool? enabled,
    String? createdAt,
  }) {
    return ScheduledTask(
      id: id ?? this.id,
      templateName: templateName ?? this.templateName,
      weekdays: weekdays ?? this.weekdays,
      time: time ?? this.time,
      enabled: enabled ?? this.enabled,
      createdAt: createdAt ?? this.createdAt,
    );
  }

  factory ScheduledTask.fromJson(Map<String, dynamic> j) => ScheduledTask(
        id: j['id'] as String? ?? '',
        templateName: j['template_name'] as String? ?? '',
        weekdays: ((j['weekdays'] as List?)?.cast<int>()) ?? [],
        time: TimeOfDay(
          hour: (j['hour'] as num?)?.toInt() ?? 0,
          minute: (j['minute'] as num?)?.toInt() ?? 0,
        ),
        enabled: j['enabled'] as bool? ?? true,
        createdAt: j['created_at'] as String? ?? '',
      );

  Map<String, dynamic> toJson() => {
        'id': id,
        'template_name': templateName,
        'weekdays': weekdays,
        'hour': time.hour,
        'minute': time.minute,
        'enabled': enabled,
        'created_at': createdAt,
      };

  String toJsonString() => jsonEncode(toJson());

  factory ScheduledTask.fromJsonString(String s) =>
      ScheduledTask.fromJson(jsonDecode(s) as Map<String, dynamic>);
}
