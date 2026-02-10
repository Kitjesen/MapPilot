import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

/// A hero card representing a saved or connected robot on the dashboard.
/// Clean card with subtle border — no glass morphism.
class RobotCard extends StatelessWidget {
  final String name;
  final String address;
  final ConnectionStatus connectionStatus;
  final double? batteryPercent;
  final double? cpuPercent;
  final String connectionType; // 'wifi' or 'ble'
  final VoidCallback? onTap;
  final VoidCallback? onLongPress;
  final String? heroTag;

  const RobotCard({
    super.key,
    required this.name,
    required this.address,
    this.connectionStatus = ConnectionStatus.disconnected,
    this.batteryPercent,
    this.cpuPercent,
    this.connectionType = 'wifi',
    this.onTap,
    this.onLongPress,
    this.heroTag,
  });

  Color _statusColor() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return AppColors.success;
      case ConnectionStatus.connecting:
      case ConnectionStatus.reconnecting:
        return AppColors.warning;
      case ConnectionStatus.error:
        return AppColors.error;
      case ConnectionStatus.disconnected:
        return Colors.grey;
    }
  }

  String _statusText() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return '已连接';
      case ConnectionStatus.connecting:
        return '连接中...';
      case ConnectionStatus.reconnecting:
        return '重连中...';
      case ConnectionStatus.error:
        return '连接错误';
      case ConnectionStatus.disconnected:
        return '未连接';
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final isConnected = connectionStatus == ConnectionStatus.connected;
    final statusColor = _statusColor();

    Widget card = GestureDetector(
      onTap: () {
        HapticFeedback.lightImpact();
        onTap?.call();
      },
      onLongPress: onLongPress,
      child: Container(
        padding: const EdgeInsets.all(16),
        decoration: BoxDecoration(
          color: context.cardColor,
          borderRadius: BorderRadius.circular(10),
          border: Border.all(color: context.borderColor),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Header row: name + status
            Row(
              children: [
                // Name + address
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        name,
                        style: TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w600,
                          color: context.titleColor,
                          letterSpacing: -0.3,
                        ),
                      ),
                      const SizedBox(height: 2),
                      Text(
                        address,
                        style: TextStyle(
                          fontSize: 13,
                          color: context.subtitleColor,
                        ),
                      ),
                    ],
                  ),
                ),
                // Status badge — minimal dot + text
                Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                  decoration: BoxDecoration(
                    color: statusColor.withValues(alpha: isDark ? 0.15 : 0.08),
                    borderRadius: BorderRadius.circular(AppRadius.pill),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Container(
                        width: 6,
                        height: 6,
                        decoration: BoxDecoration(
                          color: statusColor,
                          shape: BoxShape.circle,
                        ),
                      ),
                      const SizedBox(width: 5),
                      Text(
                        _statusText(),
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w600,
                          color: statusColor,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),

            // Quick metrics (only when connected)
            if (isConnected &&
                (batteryPercent != null || cpuPercent != null)) ...[
              const SizedBox(height: 14),
              Divider(height: 1, color: context.dividerColor),
              const SizedBox(height: 14),
              Row(
                children: [
                  if (batteryPercent != null)
                    _MetricChip(
                      label: '电量 ${batteryPercent!.toStringAsFixed(0)}%',
                      color: _batteryColor(batteryPercent!),
                    ),
                  if (batteryPercent != null && cpuPercent != null)
                    const SizedBox(width: 10),
                  if (cpuPercent != null)
                    _MetricChip(
                      label: 'CPU ${cpuPercent!.toStringAsFixed(0)}%',
                      color: _cpuColor(cpuPercent!),
                    ),
                  const Spacer(),
                  Icon(
                    Icons.chevron_right,
                    size: 18,
                    color: context.subtitleColor,
                  ),
                ],
              ),
            ],
          ],
        ),
      ),
    );

    if (heroTag != null) {
      card = Hero(tag: heroTag!, child: card);
    }
    return card;
  }

  Color _batteryColor(double pct) {
    if (pct > 60) return AppColors.success;
    if (pct > 20) return AppColors.warning;
    return AppColors.error;
  }

  Color _cpuColor(double pct) {
    if (pct < 50) return AppColors.success;
    if (pct < 80) return AppColors.warning;
    return AppColors.error;
  }
}

class _MetricChip extends StatelessWidget {
  final String label;
  final Color color;

  const _MetricChip({
    required this.label,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    return Text(
      label,
      style: TextStyle(
        fontSize: 12,
        fontWeight: FontWeight.w600,
        color: color,
      ),
    );
  }
}
