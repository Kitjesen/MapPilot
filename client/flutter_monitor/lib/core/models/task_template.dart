import 'dart:convert';

/// 任务模板 — 本地持久化的导航路线方案。
///
/// 包含路点列表和任务参数，支持 JSON 序列化。
/// 通过 [SettingsPreferences.getTaskTemplates] / [saveTaskTemplates] 持久化。
class TaskTemplate {
  /// 用户命名的模板名称
  final String name;

  /// 任务类型 (TaskType 枚举值)
  final int taskType;

  /// 路点列表 (NavigationGoal 的轻量化表示)
  final List<TemplateWaypoint> waypoints;

  /// 是否循环
  final bool loop;

  /// 创建时间 (ISO 8601)
  final String createdAt;

  const TaskTemplate({
    required this.name,
    required this.taskType,
    required this.waypoints,
    this.loop = false,
    required this.createdAt,
  });

  factory TaskTemplate.create({
    required String name,
    required int taskType,
    required List<TemplateWaypoint> waypoints,
    bool loop = false,
  }) {
    return TaskTemplate(
      name: name,
      taskType: taskType,
      waypoints: waypoints,
      loop: loop,
      createdAt: DateTime.now().toIso8601String(),
    );
  }

  factory TaskTemplate.fromJson(Map<String, dynamic> j) => TaskTemplate(
    name: j['name'] as String? ?? '',
    taskType: (j['task_type'] as num?)?.toInt() ?? 1,
    waypoints: ((j['waypoints'] as List?)?.cast<Map<String, dynamic>>() ?? [])
        .map(TemplateWaypoint.fromJson)
        .toList(),
    loop: j['loop'] as bool? ?? false,
    createdAt: j['created_at'] as String? ?? '',
  );

  Map<String, dynamic> toJson() => {
    'name': name,
    'task_type': taskType,
    'waypoints': waypoints.map((w) => w.toJson()).toList(),
    'loop': loop,
    'created_at': createdAt,
  };

  String toJsonString() => jsonEncode(toJson());
  factory TaskTemplate.fromJsonString(String s) =>
      TaskTemplate.fromJson(jsonDecode(s) as Map<String, dynamic>);
}

/// 路点的轻量化表示 (对应 proto NavigationGoal)
class TemplateWaypoint {
  final double x;
  final double y;
  final double z;
  final double yaw;
  final double arrivalRadius;
  final String label;

  const TemplateWaypoint({
    required this.x,
    required this.y,
    this.z = 0,
    this.yaw = 0,
    this.arrivalRadius = 1.0,
    this.label = '',
  });

  factory TemplateWaypoint.fromJson(Map<String, dynamic> j) => TemplateWaypoint(
    x: (j['x'] as num?)?.toDouble() ?? 0.0,
    y: (j['y'] as num?)?.toDouble() ?? 0.0,
    z: (j['z'] as num?)?.toDouble() ?? 0.0,
    yaw: (j['yaw'] as num?)?.toDouble() ?? 0.0,
    arrivalRadius: (j['arrival_radius'] as num?)?.toDouble() ?? 1.0,
    label: j['label'] as String? ?? '',
  );

  Map<String, dynamic> toJson() => {
    'x': x, 'y': y, 'z': z,
    'yaw': yaw,
    'arrival_radius': arrivalRadius,
    'label': label,
  };
}
