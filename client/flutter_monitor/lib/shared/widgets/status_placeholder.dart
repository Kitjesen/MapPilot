import 'package:flutter/material.dart';
import 'package:flutter_monitor/app/theme.dart';

/// A reusable placeholder shown when a screen has no data, no connection,
/// or is in a loading state. Consistent across all feature screens.
class StatusPlaceholder extends StatelessWidget {
  final IconData icon;
  final String title;
  final String? subtitle;
  final Color? iconColor;
  final Widget? action;

  const StatusPlaceholder({
    super.key,
    required this.icon,
    required this.title,
    this.subtitle,
    this.iconColor,
    this.action,
  });

  /// Disconnected state — shown when the robot connection is lost.
  factory StatusPlaceholder.disconnected({
    String title = '未连接到机器人',
    String subtitle = '请先在主页建立连接',
    Widget? action,
  }) {
    return StatusPlaceholder(
      icon: Icons.link_off,
      title: title,
      subtitle: subtitle,
      iconColor: AppColors.error,
      action: action,
    );
  }

  /// Empty data state — shown when a list or section has no content.
  factory StatusPlaceholder.empty({
    IconData icon = Icons.inbox_outlined,
    String title = '暂无数据',
    String? subtitle,
    Widget? action,
  }) {
    return StatusPlaceholder(
      icon: icon,
      title: title,
      subtitle: subtitle,
      action: action,
    );
  }

  /// Loading state — shown while data is being fetched.
  factory StatusPlaceholder.loading({
    String title = '正在加载...',
  }) {
    return StatusPlaceholder(
      icon: Icons.hourglass_empty,
      title: title,
    );
  }

  /// Error state — shown when data fetch failed.
  factory StatusPlaceholder.error({
    String title = '加载失败',
    String? subtitle,
    VoidCallback? onRetry,
  }) {
    return StatusPlaceholder(
      icon: Icons.error_outline,
      title: title,
      subtitle: subtitle,
      iconColor: AppColors.error,
      action: onRetry != null
          ? _RetryButton(onTap: onRetry)
          : null,
    );
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final effectiveIconColor = iconColor ??
        (isDark ? Colors.white.withValues(alpha: 0.2) : Colors.black.withValues(alpha: 0.15));

    return Center(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 40, vertical: 48),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // Icon with tinted circle background
            Container(
              width: 72,
              height: 72,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: effectiveIconColor.withValues(alpha: 0.1),
              ),
              child: Icon(
                icon,
                size: 32,
                color: effectiveIconColor,
              ),
            ),
            const SizedBox(height: 18),
            Text(
              title,
              textAlign: TextAlign.center,
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white70 : Colors.black87,
                letterSpacing: -0.3,
              ),
            ),
            if (subtitle != null) ...[
              const SizedBox(height: 6),
              Text(
                subtitle!,
                textAlign: TextAlign.center,
                style: TextStyle(
                  fontSize: 13,
                  color: context.subtitleColor,
                  height: 1.4,
                ),
              ),
            ],
            if (action != null) ...[
              const SizedBox(height: 20),
              action!,
            ],
          ],
        ),
      ),
    );
  }
}

class _RetryButton extends StatelessWidget {
  final VoidCallback onTap;

  const _RetryButton({required this.onTap});

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
        decoration: BoxDecoration(
          color: AppColors.primary.withValues(alpha: isDark ? 0.15 : 0.1),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: AppColors.primary.withValues(alpha: 0.2)),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(Icons.refresh, size: 16, color: AppColors.primary),
            const SizedBox(width: 6),
            Text(
              '重试',
              style: TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.w600,
                color: AppColors.primary,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
