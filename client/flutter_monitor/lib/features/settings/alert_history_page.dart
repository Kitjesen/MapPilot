import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:intl/intl.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/services/alert_monitor_service.dart';

/// 告警历史页面
///
/// 展示 [AlertMonitorService] 维护的历史告警列表 (最多 200 条)。
/// 支持按级别筛选、单条忽略、批量清空。
class AlertHistoryPage extends StatefulWidget {
  const AlertHistoryPage({super.key});

  @override
  State<AlertHistoryPage> createState() => _AlertHistoryPageState();
}

class _AlertHistoryPageState extends State<AlertHistoryPage> {
  AlertLevel? _filterLevel;

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: const Text('告警历史'),
        actions: [
          // 筛选
          PopupMenuButton<AlertLevel?>(
            icon: Icon(Icons.filter_list,
                color: _filterLevel != null ? context.titleColor : context.subtitleColor,
                size: 20),
            tooltip: '按级别筛选',
            onSelected: (level) => setState(() => _filterLevel = level),
            itemBuilder: (_) => [
              const PopupMenuItem(value: null, child: Text('全部')),
              const PopupMenuItem(
                  value: AlertLevel.critical, child: Text('紧急')),
              const PopupMenuItem(
                  value: AlertLevel.warning, child: Text('警告')),
              const PopupMenuItem(value: AlertLevel.info, child: Text('信息')),
            ],
          ),
          // 清空
          IconButton(
            icon: const Icon(Icons.delete_sweep, size: 22),
            tooltip: '清空全部',
            onPressed: () {
              showDialog(
                context: context,
                builder: (ctx) => AlertDialog(
                  title: const Text('确认清空'),
                  content: const Text('确定要清空所有历史告警吗？'),
                  actions: [
                    TextButton(
                      onPressed: () => Navigator.pop(ctx),
                      child: const Text('取消'),
                    ),
                    TextButton(
                      onPressed: () {
                        ctx.read<AlertMonitorService>().clearHistory();
                        Navigator.pop(ctx);
                      },
                      child: const Text('清空',
                          style: TextStyle(color: AppColors.error)),
                    ),
                  ],
                ),
              );
            },
          ),
        ],
      ),
      body: Consumer<AlertMonitorService>(
        builder: (context, alertService, _) {
          final allAlerts = alertService.history;
          final filtered = _filterLevel == null
              ? allAlerts
              : allAlerts.where((a) => a.level == _filterLevel).toList();

          if (filtered.isEmpty) {
            return Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(Icons.notifications_none,
                      size: 56, color: context.subtitleColor),
                  const SizedBox(height: 16),
                  Text(
                    _filterLevel != null ? '没有该级别的告警' : '暂无告警记录',
                    style: TextStyle(
                      fontSize: 17,
                      fontWeight: FontWeight.w600,
                      color: isDark ? Colors.white70 : Colors.black54,
                    ),
                  ),
                  const SizedBox(height: 6),
                  Text(
                    '系统正常运行中',
                    style:
                        TextStyle(fontSize: 13, color: context.subtitleColor),
                  ),
                ],
              ),
            );
          }

          return ListView.builder(
            padding: const EdgeInsets.all(16),
            itemCount: filtered.length,
            itemBuilder: (context, index) {
              final alert = filtered[index];
              return _buildAlertItem(alert, isDark);
            },
          );
        },
      ),
    );
  }

  Widget _buildAlertItem(AlertRecord alert, bool isDark) {
    final IconData icon;
    switch (alert.level) {
      case AlertLevel.critical:
        icon = Icons.error_outline;
      case AlertLevel.warning:
        icon = Icons.warning_amber_rounded;
      case AlertLevel.info:
        icon = Icons.info_outline;
    }

    final timeStr = DateFormat('MM-dd HH:mm:ss').format(alert.timestamp);

    return Opacity(
      opacity: alert.dismissed ? 0.5 : 1.0,
      child: Container(
        margin: const EdgeInsets.only(bottom: 10),
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
          boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
        ),
        child: ListTile(
          contentPadding:
              const EdgeInsets.symmetric(horizontal: 14, vertical: 6),
          leading: Icon(icon, color: context.subtitleColor, size: 18),
          title: Text(
            alert.title,
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w600,
              color: isDark ? Colors.white : Colors.black87,
              decoration:
                  alert.dismissed ? TextDecoration.lineThrough : null,
            ),
          ),
          subtitle: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const SizedBox(height: 2),
              Text(
                alert.message,
                style:
                    TextStyle(fontSize: 12, color: context.subtitleColor),
              ),
              const SizedBox(height: 4),
              Text(
                timeStr,
                style: TextStyle(
                    fontSize: 11,
                    color: context.subtitleColor.withValues(alpha:0.7)),
              ),
            ],
          ),
          trailing: alert.dismissed
              ? null
              : IconButton(
                  icon: Icon(Icons.check_circle_outline,
                      size: 20, color: context.subtitleColor),
                  tooltip: '标记已读',
                  onPressed: () {
                    context.read<AlertMonitorService>().dismiss(alert);
                  },
                ),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(14),
          ),
        ),
      ),
    );
  }
}
