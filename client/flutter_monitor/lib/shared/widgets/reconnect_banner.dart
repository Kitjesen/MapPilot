import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

/// Top banner shown during reconnection: "连接断开，Xs后重试（第N次）"
class ReconnectBanner extends StatelessWidget {
  const ReconnectBanner({super.key});

  @override
  Widget build(BuildContext context) {
    final status = context.select<RobotConnectionProvider, ConnectionStatus>(
      (p) => p.status,
    );
    if (status != ConnectionStatus.reconnecting) {
      return const SizedBox.shrink();
    }

    final countdown = context.select<RobotConnectionProvider, int>(
      (p) => p.reconnectCountdown,
    );
    final attempt = context.select<RobotConnectionProvider, int>(
      (p) => p.reconnectAttempts,
    );
    final isDark = context.isDark;
    const color = AppColors.warning;

    final text = countdown > 0
        ? '连接断开，${countdown}s后重试（第${attempt + 1}次）'
        : '正在重连...（第${attempt + 1}次）';

    return TweenAnimationBuilder<double>(
      tween: Tween(begin: -1.0, end: 0.0),
      duration: const Duration(milliseconds: 350),
      curve: Curves.easeOutCubic,
      builder: (context, value, child) => Transform.translate(
        offset: Offset(0, value * 48),
        child: child,
      ),
      child: Container(
        width: double.infinity,
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
        decoration: BoxDecoration(
          color: isDark
              ? color.withValues(alpha: 0.2)
              : color.withValues(alpha: 0.1),
          border: Border(
            bottom: BorderSide(
              color: color.withValues(alpha: 0.3),
              width: 1,
            ),
          ),
        ),
        child: Row(
          children: [
            const SizedBox(
              width: 16,
              height: 16,
              child: CircularProgressIndicator(
                strokeWidth: 2,
                color: color,
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: Text(
                text,
                style: const TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                  color: color,
                ),
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
