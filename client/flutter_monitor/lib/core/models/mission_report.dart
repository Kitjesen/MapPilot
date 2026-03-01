import 'package:robot_proto/src/common.pb.dart';

/// A single notable event captured during a mission.
class MissionEvent {
  final String title;
  final EventSeverity severity;
  final DateTime timestamp;

  const MissionEvent({
    required this.title,
    required this.severity,
    required this.timestamp,
  });
}

/// Summary report generated when a task reaches a terminal state.
class MissionReport {
  final String? taskId;
  final TaskType taskType;
  final TaskStatus finalStatus;
  final DateTime startTime;
  final DateTime endTime;
  final int waypointsCompleted;
  final int waypointsTotal;
  final int replanCount;
  final String? failureReason;
  final List<MissionEvent> events;

  const MissionReport({
    this.taskId,
    required this.taskType,
    required this.finalStatus,
    required this.startTime,
    required this.endTime,
    this.waypointsCompleted = 0,
    this.waypointsTotal = 0,
    this.replanCount = 0,
    this.failureReason,
    this.events = const [],
  });

  Duration get duration => endTime.difference(startTime);
}
