import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:flutter_monitor/core/models/mission_report.dart';
import 'package:robot_proto/src/common.pb.dart';

/// Shows a modal bottom sheet with the mission report card.
void showMissionReportSheet(
  BuildContext context, {
  required MissionReport report,
  required LocaleProvider locale,
  VoidCallback? onRetry,
}) {
  HapticFeedback.mediumImpact();
  showModalBottomSheet(
    context: context,
    isScrollControlled: true,
    backgroundColor: Colors.transparent,
    builder: (ctx) => _MissionReportSheet(
      report: report,
      locale: locale,
      onRetry: onRetry,
    ),
  );
}

class _MissionReportSheet extends StatelessWidget {
  final MissionReport report;
  final LocaleProvider locale;
  final VoidCallback? onRetry;

  const _MissionReportSheet({
    required this.report,
    required this.locale,
    this.onRetry,
  });

  bool get _isSuccess =>
      report.finalStatus == TaskStatus.TASK_STATUS_COMPLETED;
  bool get _isCancelled =>
      report.finalStatus == TaskStatus.TASK_STATUS_CANCELLED;

  String get _taskTypeLabel => switch (report.taskType) {
        TaskType.TASK_TYPE_NAVIGATION => locale.tr('导航', 'Navigation'),
        TaskType.TASK_TYPE_MAPPING => locale.tr('建图', 'Mapping'),
        TaskType.TASK_TYPE_INSPECTION => locale.tr('巡检', 'Patrol'),
        TaskType.TASK_TYPE_RETURN_HOME => locale.tr('回家', 'Return home'),
        TaskType.TASK_TYPE_FOLLOW_PATH => locale.tr('循迹', 'Follow path'),
        TaskType.TASK_TYPE_SEMANTIC_NAV => locale.tr('语义导航', 'Semantic nav'),
        TaskType.TASK_TYPE_FOLLOW_PERSON => locale.tr('跟随', 'Follow person'),
        _ => locale.tr('任务', 'Task'),
      };

  String get _statusLabel {
    if (_isSuccess) return locale.tr('任务完成', 'Task completed');
    if (_isCancelled) return locale.tr('任务取消', 'Task cancelled');
    return locale.tr('任务失败', 'Task failed');
  }

  Color get _statusColor {
    if (_isSuccess) return AppColors.success;
    if (_isCancelled) return AppColors.warning;
    return AppColors.error;
  }

  IconData get _statusIcon {
    if (_isSuccess) return Icons.check_circle_rounded;
    if (_isCancelled) return Icons.cancel_rounded;
    return Icons.error_rounded;
  }

  String _formatDuration(Duration d) {
    final min = d.inMinutes;
    final sec = d.inSeconds % 60;
    if (min > 0 && sec > 0) {
      return locale.tr('$min 分 $sec 秒', '${min}m ${sec}s');
    }
    if (min > 0) return locale.tr('$min 分', '${min}m');
    return locale.tr('$sec 秒', '${sec}s');
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final warningEvents = report.events.where((e) =>
        e.severity.value >= EventSeverity.EVENT_SEVERITY_WARNING.value).toList();
    final showEvents = warningEvents.take(5).toList();

    return Container(
      margin: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.xl),
        boxShadow: AppShadows.elevated(isDark: isDark),
      ),
      child: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(24, 20, 24, 24),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              // Drag handle
              Container(
                width: 40,
                height: 4,
                decoration: BoxDecoration(
                  color: context.subtitleColor.withValues(alpha: 0.3),
                  borderRadius: BorderRadius.circular(2),
                ),
              ),
              const SizedBox(height: 20),

              // Status icon + label
              Icon(_statusIcon, color: _statusColor, size: 48),
              const SizedBox(height: 10),
              Text(
                _statusLabel,
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.w700,
                  color: context.titleColor,
                ),
              ),
              const SizedBox(height: 4),
              Text(
                _taskTypeLabel,
                style: TextStyle(fontSize: 13, color: context.subtitleColor),
              ),
              const SizedBox(height: 20),

              // Stats row
              _buildStatsRow(context),
              const SizedBox(height: 16),

              // Failure reason
              if (!_isSuccess && !_isCancelled && report.failureReason != null) ...[
                _buildFailureBanner(context),
                const SizedBox(height: 16),
              ],

              // Events
              _buildEventsSection(context, showEvents, warningEvents.length),
              const SizedBox(height: 20),

              // Buttons
              _buildButtons(context),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildStatsRow(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 16),
      decoration: BoxDecoration(
        color: context.isDark
            ? Colors.white.withValues(alpha: 0.04)
            : Colors.black.withValues(alpha: 0.03),
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          _statItem(
            context,
            Icons.timer_outlined,
            _formatDuration(report.duration),
            locale.tr('用时', 'Duration'),
          ),
          if (report.waypointsTotal > 0)
            _statItem(
              context,
              Icons.place_outlined,
              '${report.waypointsCompleted}/${report.waypointsTotal}',
              locale.tr('路点', 'Waypoints'),
            ),
          if (report.replanCount > 0)
            _statItem(
              context,
              Icons.alt_route_rounded,
              '${report.replanCount}',
              locale.tr('重规划', 'Replans'),
            ),
        ],
      ),
    );
  }

  Widget _statItem(
      BuildContext context, IconData icon, String value, String label) {
    return Column(
      children: [
        Icon(icon, size: 18, color: context.subtitleColor),
        const SizedBox(height: 4),
        Text(
          value,
          style: TextStyle(
            fontSize: 15,
            fontWeight: FontWeight.w700,
            color: context.titleColor,
          ),
        ),
        const SizedBox(height: 2),
        Text(
          label,
          style: TextStyle(fontSize: 11, color: context.subtitleColor),
        ),
      ],
    );
  }

  Widget _buildFailureBanner(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: AppColors.error.withValues(alpha: 0.1),
        borderRadius: BorderRadius.circular(AppRadius.sm),
        border: Border.all(color: AppColors.error.withValues(alpha: 0.3)),
      ),
      child: Row(
        children: [
          const Icon(Icons.warning_amber_rounded,
              color: AppColors.error, size: 18),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              report.failureReason!,
              style: const TextStyle(fontSize: 13, color: AppColors.error),
              maxLines: 3,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEventsSection(
      BuildContext context, List<MissionEvent> showEvents, int totalCount) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: context.isDark
            ? Colors.white.withValues(alpha: 0.04)
            : Colors.black.withValues(alpha: 0.03),
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.event_note_outlined,
                  size: 15, color: context.subtitleColor),
              const SizedBox(width: 6),
              Text(
                locale.tr('任务事件', 'Events'),
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          if (showEvents.isEmpty)
            Text(
              locale.tr('任务顺利完成，无异常事件', 'No issues during task'),
              style: TextStyle(fontSize: 13, color: context.subtitleColor),
            )
          else ...[
            for (final event in showEvents) _eventRow(context, event),
            if (totalCount > 5) ...[
              const SizedBox(height: 4),
              Text(
                locale.tr('... 还有 ${totalCount - 5} 条',
                    '... ${totalCount - 5} more'),
                style: TextStyle(fontSize: 11, color: context.subtitleColor),
              ),
            ],
          ],
        ],
      ),
    );
  }

  Widget _eventRow(BuildContext context, MissionEvent event) {
    final color = _severityColor(event.severity);
    final timeStr =
        '${event.timestamp.hour.toString().padLeft(2, '0')}:${event.timestamp.minute.toString().padLeft(2, '0')}:${event.timestamp.second.toString().padLeft(2, '0')}';
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        children: [
          Container(
            width: 6,
            height: 6,
            decoration: BoxDecoration(color: color, shape: BoxShape.circle),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              event.title,
              style: TextStyle(fontSize: 12, color: context.titleColor),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
            ),
          ),
          const SizedBox(width: 8),
          Text(timeStr,
              style: TextStyle(fontSize: 10, color: context.subtitleColor)),
        ],
      ),
    );
  }

  Color _severityColor(EventSeverity severity) {
    switch (severity) {
      case EventSeverity.EVENT_SEVERITY_WARNING:
        return AppColors.warning;
      case EventSeverity.EVENT_SEVERITY_ERROR:
        return AppColors.error;
      case EventSeverity.EVENT_SEVERITY_CRITICAL:
        return AppColors.accent;
      default:
        return AppColors.info;
    }
  }

  Widget _buildButtons(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: SizedBox(
            height: 44,
            child: TextButton(
              onPressed: () => Navigator.pop(context),
              style: TextButton.styleFrom(
                foregroundColor: context.subtitleColor,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                  side: BorderSide(color: context.borderColor),
                ),
              ),
              child: Text(locale.tr('关闭', 'Close'),
                  style: const TextStyle(fontSize: 14)),
            ),
          ),
        ),
        if (onRetry != null) ...[
          const SizedBox(width: 10),
          Expanded(
            child: SizedBox(
              height: 44,
              child: FilledButton(
                onPressed: () {
                  Navigator.pop(context);
                  onRetry?.call();
                },
                style: FilledButton.styleFrom(
                  backgroundColor: AppColors.primary,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(8),
                  ),
                ),
                child: Text(locale.tr('再次执行', 'Run again'),
                    style: const TextStyle(
                        fontSize: 14, fontWeight: FontWeight.w600)),
              ),
            ),
          ),
        ],
      ],
    );
  }
}
