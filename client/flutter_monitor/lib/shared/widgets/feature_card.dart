import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';

/// A card-based entry point for a robot feature (Status, Control, Map, etc.)
/// Clean, minimal design with tinted icon container — subtle border + shadow.
class FeatureCard extends StatelessWidget {
  final IconData icon;
  final String title;
  final String? subtitle;
  final Color? color;
  final VoidCallback? onTap;
  final Widget? badge;
  final Widget? trailing;

  const FeatureCard({
    super.key,
    required this.icon,
    required this.title,
    this.subtitle,
    this.color,
    this.onTap,
    this.badge,
    this.trailing,
  });

  /// Deterministic icon accent from the icon's codePoint.
  Color _iconAccent() {
    const palette = [
      AppColors.primary,
      AppColors.secondary,
      AppColors.success,
      AppColors.warning,
      AppColors.info,
      AppColors.accent,
    ];
    return color ?? palette[icon.codePoint % palette.length];
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final accent = _iconAccent();

    return GestureDetector(
      onTap: () {
        HapticFeedback.selectionClick();
        onTap?.call();
      },
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        padding: const EdgeInsets.all(14),
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
          boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                // Tinted icon container
                Container(
                  width: 34,
                  height: 34,
                  decoration: BoxDecoration(
                    color: accent.withValues(alpha: isDark ? 0.15 : 0.1),
                    borderRadius: BorderRadius.circular(9),
                  ),
                  child: Icon(icon, color: accent, size: 18),
                ),
                if (badge != null) ...[
                  const Spacer(),
                  badge!,
                ],
              ],
            ),
            const Spacer(),
            // Title
            Text(
              title,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.w600,
                color: context.titleColor,
                letterSpacing: -0.2,
              ),
            ),
            if (subtitle != null) ...[
              const SizedBox(height: 3),
              Text(
                subtitle!,
                style: TextStyle(
                  fontSize: 12,
                  color: context.subtitleColor,
                ),
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
              ),
            ],
            if (trailing != null) ...[
              const SizedBox(height: 6),
              trailing!,
            ],
          ],
        ),
      ),
    );
  }
}

/// A grouped settings section — Cursor-style.
/// No borders or shadows; relies on subtle background color contrast.
class SettingsSection extends StatelessWidget {
  final String? title;
  final List<Widget> children;
  final EdgeInsetsGeometry? margin;

  const SettingsSection({
    super.key,
    this.title,
    required this.children,
    this.margin,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Padding(
      padding: margin ?? const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          if (title != null)
            Padding(
              padding: const EdgeInsets.only(left: 4, bottom: 6, top: 8),
              child: Text(
                title!,
                style: TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                ),
              ),
            ),
          Container(
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(AppRadius.card),
              child: Column(
                children: _insertDividers(context, children),
              ),
            ),
          ),
        ],
      ),
    );
  }

  List<Widget> _insertDividers(BuildContext context, List<Widget> items) {
    if (items.length <= 1) return items;
    final result = <Widget>[];
    for (int i = 0; i < items.length; i++) {
      result.add(items[i]);
      if (i < items.length - 1) {
        result.add(Divider(
          height: 0.5,
          indent: 16,
          endIndent: 16,
          color: context.dividerColor,
        ));
      }
    }
    return result;
  }
}

/// A single tile in a settings section — Cursor-style.
/// Icons are optional. Trailing can be a button, switch, or chevron.
class SettingsTile extends StatelessWidget {
  final IconData? icon;
  final Color iconColor;
  final String title;
  final String? subtitle;
  final Widget? trailing;
  final VoidCallback? onTap;

  const SettingsTile({
    super.key,
    this.icon,
    this.iconColor = Colors.grey,
    required this.title,
    this.subtitle,
    this.trailing,
    this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onTap,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        child: Row(
          children: [
            if (icon != null) ...[
              Icon(
                icon,
                size: 18,
                color: context.subtitleColor,
              ),
              const SizedBox(width: 12),
            ],
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    title,
                    style: TextStyle(
                      fontSize: 14,
                      fontWeight: FontWeight.w500,
                      color: context.titleColor,
                    ),
                  ),
                  if (subtitle != null) ...[
                    const SizedBox(height: 2),
                    Text(
                      subtitle!,
                      style: TextStyle(
                        fontSize: 12,
                        color: context.subtitleColor,
                      ),
                    ),
                  ],
                ],
              ),
            ),
            const SizedBox(width: 12),
            trailing ??
                Icon(
                  Icons.chevron_right,
                  size: 18,
                  color: context.subtitleColor,
                ),
          ],
        ),
      ),
    );
  }
}

/// A small outlined action button for settings trailing — like "Open", "Import".
class SettingsActionButton extends StatelessWidget {
  final String label;
  final VoidCallback? onTap;

  const SettingsActionButton({
    super.key,
    required this.label,
    this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 6),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(6),
          border: Border.all(
            color: isDark
                ? Colors.white.withValues(alpha: 0.15)
                : Colors.black.withValues(alpha: 0.12),
          ),
        ),
        child: Text(
          label,
          style: TextStyle(
            fontSize: 13,
            fontWeight: FontWeight.w500,
            color: context.titleColor,
          ),
        ),
      ),
    );
  }
}
