import 'package:flutter/material.dart';

/// 响应式断点定义
class Breakpoint {
  /// 手机竖屏 (< 600)
  static const double mobile = 600;

  /// 平板/小桌面 (600 ~ 960)
  static const double tablet = 960;

  /// 桌面 (> 960)
  static const double desktop = 960;
}

/// 设备类型
enum DeviceType { mobile, tablet, desktop }

/// 响应式工具扩展
extension ResponsiveContext on BuildContext {
  double get screenWidth => MediaQuery.sizeOf(this).width;
  double get screenHeight => MediaQuery.sizeOf(this).height;
  bool get isLandscape => screenWidth > screenHeight;

  DeviceType get deviceType {
    final w = screenWidth;
    if (w < Breakpoint.mobile) return DeviceType.mobile;
    if (w < Breakpoint.desktop) return DeviceType.tablet;
    return DeviceType.desktop;
  }

  bool get isMobile => deviceType == DeviceType.mobile;
  bool get isTablet => deviceType == DeviceType.tablet;
  bool get isDesktop => deviceType == DeviceType.desktop;

  /// 是否显示侧边导航（平板及以上）
  bool get useSideNav => screenWidth >= Breakpoint.mobile;

  /// 自适应水平 padding
  double get screenPadding {
    if (isDesktop) return 32;
    if (isTablet) return 24;
    return 16;
  }

  /// 自适应网格列数
  int get gridColumns {
    if (isDesktop) return 3;
    if (isTablet) return 2;
    return 1;
  }

  /// 内容区最大宽度（桌面模式下限制内容宽度）
  double get contentMaxWidth {
    if (isDesktop) return 1200;
    return double.infinity;
  }
}

/// 自适应内容容器 — 桌面模式下居中限宽
class AdaptiveContent extends StatelessWidget {
  final Widget child;
  final double? maxWidth;
  final EdgeInsetsGeometry? padding;

  const AdaptiveContent({
    super.key,
    required this.child,
    this.maxWidth,
    this.padding,
  });

  @override
  Widget build(BuildContext context) {
    final mw = maxWidth ?? context.contentMaxWidth;
    final p = padding ?? EdgeInsets.symmetric(horizontal: context.screenPadding);

    return Center(
      child: ConstrainedBox(
        constraints: BoxConstraints(maxWidth: mw),
        child: Padding(padding: p, child: child),
      ),
    );
  }
}

/// 自适应网格 — 根据屏幕宽度自动调整列数
class AdaptiveGrid extends StatelessWidget {
  final List<Widget> children;
  final double spacing;
  final double runSpacing;
  final int? columns;

  const AdaptiveGrid({
    super.key,
    required this.children,
    this.spacing = 12,
    this.runSpacing = 12,
    this.columns,
  });

  @override
  Widget build(BuildContext context) {
    final cols = columns ?? context.gridColumns;
    if (cols == 1) {
      return Column(
        children: children
            .map((c) => Padding(
                  padding: EdgeInsets.only(bottom: runSpacing),
                  child: c,
                ))
            .toList(),
      );
    }
    return LayoutBuilder(
      builder: (context, constraints) {
        final itemWidth =
            (constraints.maxWidth - spacing * (cols - 1)) / cols;
        return Wrap(
          spacing: spacing,
          runSpacing: runSpacing,
          children: children
              .map((c) => SizedBox(width: itemWidth, child: c))
              .toList(),
        );
      },
    );
  }
}
