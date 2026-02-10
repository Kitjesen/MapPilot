import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

/// Dashboard-style robot card with soft shadow and purple accents.
class RobotCard extends StatelessWidget {
  final String name;
  final String address;
  final ConnectionStatus connectionStatus;
  final double? batteryPercent;
  final double? cpuPercent;
  final String connectionType;
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
        return AppColors.textTertiary;
    }
  }

  String _statusText() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return 'Online';
      case ConnectionStatus.connecting:
        return 'Connecting...';
      case ConnectionStatus.reconnecting:
        return 'Reconnecting...';
      case ConnectionStatus.error:
        return 'Error';
      case ConnectionStatus.disconnected:
        return 'Offline';
    }
  }

  @override
  Widget build(BuildContext context) {
    final isConnected = connectionStatus == ConnectionStatus.connected;
    final statusColor = _statusColor();

    Widget card = GestureDetector(
      onTap: () {
        HapticFeedback.lightImpact();
        onTap?.call();
      },
      onLongPress: onLongPress,
      child: Container(
        padding: const EdgeInsets.all(20),
        decoration: context.elevatedCardDecoration,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Header row
            Row(
              children: [
                // Robot icon
                Container(
                  width: 44,
                  height: 44,
                  decoration: BoxDecoration(
                    gradient: AppColors.brandGradient,
                    borderRadius: BorderRadius.circular(14),
                  ),
                  child: const Icon(Icons.smart_toy, color: Colors.white, size: 22),
                ),
                const SizedBox(width: 14),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Flexible(
                            child: Text(
                              name,
                              style: TextStyle(
                                fontSize: 16,
                                fontWeight: FontWeight.w700,
                                color: context.titleColor,
                                letterSpacing: -0.3,
                              ),
                              overflow: TextOverflow.ellipsis,
                            ),
                          ),
                          const SizedBox(width: 8),
                          // Status badge
                          Container(
                            padding: const EdgeInsets.symmetric(
                                horizontal: 10, vertical: 4),
                            decoration: BoxDecoration(
                              color: statusColor.withValues(alpha: 0.12),
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
                      const SizedBox(height: 3),
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
                Icon(
                  Icons.chevron_right_rounded,
                  size: 22,
                  color: context.hintColor,
                ),
              ],
            ),

            // Quick metrics
            if (isConnected &&
                (batteryPercent != null || cpuPercent != null)) ...[
              const SizedBox(height: 16),
              Container(
                padding: const EdgeInsets.all(14),
                decoration: BoxDecoration(
                  color: context.surfaceColor,
                  borderRadius: BorderRadius.circular(AppRadius.md),
                ),
                child: Row(
                  children: [
                    if (batteryPercent != null)
                      _MetricChip(
                        icon: Icons.battery_charging_full_rounded,
                        label: '${batteryPercent!.toStringAsFixed(0)}%',
                        color: _batteryColor(batteryPercent!),
                      ),
                    if (batteryPercent != null && cpuPercent != null)
                      Container(
                        width: 1,
                        height: 24,
                        margin: const EdgeInsets.symmetric(horizontal: 16),
                        color: context.borderColor,
                      ),
                    if (cpuPercent != null)
                      _MetricChip(
                        icon: Icons.memory_rounded,
                        label: '${cpuPercent!.toStringAsFixed(0)}%',
                        color: _cpuColor(cpuPercent!),
                      ),
                  ],
                ),
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
  final IconData icon;
  final String label;
  final Color color;

  const _MetricChip({
    required this.icon,
    required this.label,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(icon, size: 18, color: color),
        const SizedBox(width: 6),
        Text(
          label,
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w700,
            color: color,
          ),
        ),
      ],
    );
  }
}
