import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/models/robot_profile.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

/// 机器人型号选择页面 — 毛玻璃卡片风格
class RobotSelectScreen extends StatefulWidget {
  const RobotSelectScreen({super.key});

  @override
  State<RobotSelectScreen> createState() => _RobotSelectScreenState();
}

class _RobotSelectScreenState extends State<RobotSelectScreen>
    with TickerProviderStateMixin {
  late AnimationController _staggerController;
  final List<AnimationController> _cardControllers = [];
  bool _loaded = false;

  @override
  void initState() {
    super.initState();
    _staggerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 400),
    )..forward();
    // Delay to simulate load + trigger card entrance
    Future.delayed(const Duration(milliseconds: 150), () {
      if (mounted) setState(() => _loaded = true);
    });
  }

  @override
  void dispose() {
    _staggerController.dispose();
    for (final c in _cardControllers) {
      c.dispose();
    }
    super.dispose();
  }

  AnimationController _getCardController(int index) {
    while (_cardControllers.length <= index) {
      final c = AnimationController(
        vsync: this,
        duration: const Duration(milliseconds: 500),
      );
      _cardControllers.add(c);
    }
    if (!_cardControllers[index].isAnimating &&
        _cardControllers[index].value == 0) {
      Future.delayed(Duration(milliseconds: 80 * index), () {
        if (mounted) _cardControllers[index].forward();
      });
    }
    return _cardControllers[index];
  }

  @override
  Widget build(BuildContext context) {
    final profileProvider = context.watch<RobotProfileProvider>();
    final profiles = profileProvider.allProfiles;
    final currentId = profileProvider.current.id;
    final isDark = context.isDark;

    return Scaffold(
      backgroundColor:
          isDark ? AppColors.darkBackground : AppColors.lightBackground,
      body: SafeArea(
        child: CustomScrollView(
          physics: const BouncingScrollPhysics(),
          slivers: [
            // ─── Header ───
            SliverToBoxAdapter(
              child: Padding(
                padding: const EdgeInsets.fromLTRB(20, 12, 20, 0),
                child: Row(
                  children: [
                    GestureDetector(
                      onTap: () => Navigator.pop(context),
                      child: Icon(Icons.arrow_back_ios_new,
                          size: 18, color: context.titleColor),
                    ),
                    const SizedBox(width: 16),
                    Text(
                      '选择机器人',
                      style: TextStyle(
                        fontSize: 17,
                        fontWeight: FontWeight.w600,
                        color: context.titleColor,
                      ),
                    ),
                  ],
                ),
              ),
            ),

            // ─── Subtitle ───
            SliverToBoxAdapter(
              child: Padding(
                padding: const EdgeInsets.fromLTRB(20, 20, 20, 16),
                child: Text(
                  '选择型号开始使用',
                  style: TextStyle(
                    fontSize: 13,
                    color: context.subtitleColor,
                  ),
                ),
              ),
            ),

            // ─── Cards ───
            if (_loaded)
              SliverPadding(
                padding: const EdgeInsets.fromLTRB(16, 0, 16, 32),
                sliver: SliverGrid(
                  gridDelegate:
                      const SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 2,
                    mainAxisSpacing: 10,
                    crossAxisSpacing: 10,
                    childAspectRatio: 1.15,
                  ),
                  delegate: SliverChildBuilderDelegate(
                    (context, index) {
                      final profile = profiles[index];
                      final isSelected = profile.id == currentId;
                      final ctrl = _getCardController(index);

                      return AnimatedBuilder(
                        listenable: ctrl,
                        child: _RobotCard(
                          profile: profile,
                          isSelected: isSelected,
                          onTap: () {
                            HapticFeedback.selectionClick();
                            _showDetailSheet(context, profile, isSelected);
                          },
                        ),
                        builder: (context, child) {
                          final t = CurvedAnimation(
                            parent: ctrl,
                            curve: Curves.easeOutCubic,
                          ).value;
                          return Opacity(
                            opacity: t,
                            child: Transform.translate(
                              offset: Offset(0, 16 * (1 - t)),
                              child: child,
                            ),
                          );
                        },
                      );
                    },
                    childCount: profiles.length,
                  ),
                ),
              ),

            // Loading placeholder
            if (!_loaded)
              SliverPadding(
                padding: const EdgeInsets.fromLTRB(16, 0, 16, 32),
                sliver: SliverGrid(
                  gridDelegate:
                      const SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 2,
                    mainAxisSpacing: 10,
                    crossAxisSpacing: 10,
                    childAspectRatio: 1.15,
                  ),
                  delegate: SliverChildBuilderDelegate(
                    (context, index) => _ShimmerCard(isDark: isDark),
                    childCount: 4,
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }

  void _showDetailSheet(
      BuildContext context, RobotProfile profile, bool isSelected) {
    final isDark = context.isDark;
    final color = profile.themeColor;

    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.transparent,
      isScrollControlled: true,
      builder: (ctx) {
        return BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 20, sigmaY: 20),
          child: Container(
            margin: const EdgeInsets.fromLTRB(12, 0, 12, 12),
            decoration: BoxDecoration(
              color: isDark
                  ? AppColors.darkCard.withValues(alpha: 0.92)
                  : Colors.white.withValues(alpha: 0.92),
              borderRadius: BorderRadius.circular(AppRadius.card),
              border: isDark ? Border.all(color: AppColors.borderDark) : null,
              boxShadow: isDark ? null : [AppShadows.light()],
            ),
            child: DraggableScrollableSheet(
              initialChildSize: 0.52,
              minChildSize: 0.3,
              maxChildSize: 0.75,
              expand: false,
              builder: (_, scrollController) {
                return SingleChildScrollView(
                  controller: scrollController,
                  padding: const EdgeInsets.fromLTRB(20, 8, 20, 28),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      // Drag handle
                      Center(
                        child: Container(
                          width: 36,
                          height: 4,
                          margin: const EdgeInsets.only(bottom: 16),
                          decoration: BoxDecoration(
                            color: context.subtitleColor.withValues(alpha: 0.25),
                            borderRadius: BorderRadius.circular(2),
                          ),
                        ),
                      ),

                      // Name row
                      Row(
                        children: [
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  profile.name,
                                  style: TextStyle(
                                    fontSize: 20,
                                    fontWeight: FontWeight.w700,
                                    color: context.titleColor,
                                    letterSpacing: -0.3,
                                  ),
                                ),
                                const SizedBox(height: 3),
                                Text(
                                  profile.subtitle,
                                  style: TextStyle(
                                    fontSize: 13,
                                    color: context.subtitleColor,
                                  ),
                                ),
                              ],
                            ),
                          ),
                          if (isSelected)
                            Text(
                              '当前',
                              style: TextStyle(
                                fontSize: 12,
                                fontWeight: FontWeight.w500,
                                color: context.subtitleColor,
                              ),
                            ),
                        ],
                      ),
                      const SizedBox(height: 14),

                      // Description
                      Text(
                        profile.description,
                        style: TextStyle(
                          fontSize: 13,
                          height: 1.6,
                          color: context.subtitleColor,
                        ),
                      ),
                      const SizedBox(height: 18),

                      // Specs — compact row
                      _buildSpecChips(context, profile),
                      const SizedBox(height: 14),

                      // Network
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 12, vertical: 10),
                        decoration: BoxDecoration(
                          color: isDark
                              ? Colors.white.withValues(alpha: 0.04)
                              : Colors.black.withValues(alpha: 0.03),
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: Row(
                          children: [
                            Text(
                              '${profile.defaultHost}:${profile.defaultPort}',
                              style: TextStyle(
                                fontSize: 12,
                                fontFamily: 'monospace',
                                color: context.subtitleColor,
                              ),
                            ),
                            const Spacer(),
                            Text(
                              profile.urdfAsset != null ? '3D 模型' : '无 URDF',
                              style: TextStyle(
                                fontSize: 11,
                                fontWeight: FontWeight.w500,
                                color: profile.urdfAsset != null
                                    ? color
                                    : context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                      const SizedBox(height: 20),

                      // Select button
                      SizedBox(
                        width: double.infinity,
                        height: 44,
                        child: TextButton(
                          onPressed: profile.available
                              ? () async {
                                  final provider =
                                      context.read<RobotProfileProvider>();
                                  HapticFeedback.mediumImpact();
                                  await provider.select(profile);
                                  if (ctx.mounted) Navigator.pop(ctx);
                                  if (context.mounted) {
                                    ScaffoldMessenger.of(context).showSnackBar(
                                      SnackBar(
                                        content:
                                            Text('已选择 ${profile.name}'),
                                        behavior: SnackBarBehavior.floating,
                                        duration:
                                            const Duration(seconds: 1),
                                        shape: RoundedRectangleBorder(
                                            borderRadius:
                                                BorderRadius.circular(8)),
                                      ),
                                    );
                                  }
                                }
                              : null,
                          style: TextButton.styleFrom(
                            foregroundColor: context.titleColor,
                            backgroundColor: isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.04),
                            disabledForegroundColor: context.subtitleColor,
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                              side: BorderSide(color: context.borderColor),
                            ),
                          ),
                          child: Text(
                            isSelected
                                ? '当前已选择'
                                : (profile.available ? '选择此型号' : '敬请期待'),
                            style: const TextStyle(
                              fontSize: 14,
                              fontWeight: FontWeight.w600,
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ),
        );
      },
    );
  }

  Widget _buildSpecChips(BuildContext context, RobotProfile profile) {
    final isDark = context.isDark;
    final specs = [
      '${profile.jointCount} 关节',
      '${profile.legNames.length} 腿',
      '${profile.jointsPerLeg} DOF',
      '${profile.maxLinearSpeed} m/s',
      '${profile.mass} kg',
    ];

    return Wrap(
      spacing: 6,
      runSpacing: 6,
      children: specs.map((s) {
        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
          decoration: BoxDecoration(
            color: isDark
                ? Colors.white.withValues(alpha: 0.05)
                : Colors.black.withValues(alpha: 0.04),
            borderRadius: BorderRadius.circular(6),
          ),
          child: Text(
            s,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w500,
              color: context.subtitleColor,
            ),
          ),
        );
      }).toList(),
    );
  }
}

// ================================================================
// 毛玻璃风格机器人卡片
// ================================================================

class _RobotCard extends StatelessWidget {
  final RobotProfile profile;
  final bool isSelected;
  final VoidCallback onTap;

  const _RobotCard({
    required this.profile,
    required this.isSelected,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return GestureDetector(
      onTap: onTap,
      child: Container(
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
          border: isDark ? Border.all(color: AppColors.borderDark) : null,
          boxShadow: isDark ? null : [AppShadows.light()],
        ),
        clipBehavior: Clip.antiAlias,
        child: Stack(
          fit: StackFit.expand,
          children: [
            Padding(
              padding: const EdgeInsets.all(12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Row(
                    children: [
                      Container(
                        width: 32,
                        height: 32,
                        decoration: BoxDecoration(
                          color: profile.themeColor.withValues(alpha: isDark ? 0.15 : 0.08),
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: Icon(
                          profile.icon,
                          size: 18,
                          color: profile.themeColor,
                        ),
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              profile.name,
                              style: TextStyle(
                                fontSize: 13,
                                fontWeight: FontWeight.w600,
                                color: context.titleColor,
                              ),
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                            ),
                            Text(
                              '${profile.jointCount}J · ${profile.mass.toStringAsFixed(0)}kg',
                              style: TextStyle(
                                fontSize: 11,
                                color: context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                      if (isSelected)
                        Icon(Icons.check, size: 14, color: profile.themeColor),
                    ],
                  ),
                  const SizedBox(height: 6),
                  Text(
                    profile.subtitle,
                    style: TextStyle(
                      fontSize: 11,
                      color: context.subtitleColor,
                    ),
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                  ),
                ],
              ),
            ),
            if (!profile.available)
              Container(
                color: isDark
                    ? Colors.black.withValues(alpha: 0.5)
                    : Colors.white.withValues(alpha: 0.6),
                child: Center(
                  child: Text(
                    '即将推出',
                    style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w500,
                      color: context.subtitleColor,
                    ),
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }
}

// ================================================================
// 加载骨架卡片（shimmer placeholder）
// ================================================================

class _ShimmerCard extends StatefulWidget {
  final bool isDark;
  const _ShimmerCard({required this.isDark});

  @override
  State<_ShimmerCard> createState() => _ShimmerCardState();
}

class _ShimmerCardState extends State<_ShimmerCard>
    with SingleTickerProviderStateMixin {
  late AnimationController _shimmerController;

  @override
  void initState() {
    super.initState();
    _shimmerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1200),
    )..repeat();
  }

  @override
  void dispose() {
    _shimmerController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      listenable: _shimmerController,
      builder: (context, _) {
        final shimmer = _shimmerController.value;
        final baseColor = widget.isDark
            ? Colors.white.withValues(alpha: 0.04)
            : Colors.black.withValues(alpha: 0.04);
        final highlightColor = widget.isDark
            ? Colors.white.withValues(alpha: 0.08)
            : Colors.black.withValues(alpha: 0.07);
        final color =
            Color.lerp(baseColor, highlightColor, (shimmer * 2 - 1).abs())!;

        return Container(
          decoration: BoxDecoration(
            color: color,
            borderRadius: BorderRadius.circular(AppRadius.card),
            border: widget.isDark ? Border.all(color: AppColors.borderDark) : null,
            boxShadow: widget.isDark ? null : [AppShadows.light()],
          ),
          padding: const EdgeInsets.all(14),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Spacer(),
              // Name placeholder
              Container(
                height: 14,
                width: 80,
                decoration: BoxDecoration(
                  color: baseColor,
                  borderRadius: BorderRadius.circular(4),
                ),
              ),
              const SizedBox(height: 6),
              // Subtitle placeholder
              Container(
                height: 10,
                width: 60,
                decoration: BoxDecoration(
                  color: baseColor,
                  borderRadius: BorderRadius.circular(3),
                ),
              ),
              const SizedBox(height: 8),
              // Specs placeholder
              Container(
                height: 10,
                width: 50,
                decoration: BoxDecoration(
                  color: baseColor,
                  borderRadius: BorderRadius.circular(3),
                ),
              ),
            ],
          ),
        );
      },
    );
  }
}

// ================================================================
// AnimatedBuilder — simple alias for AnimatedWidget pattern
// ================================================================

class AnimatedBuilder extends AnimatedWidget {
  final Widget? child;
  final Widget Function(BuildContext context, Widget? child) builder;

  const AnimatedBuilder({
    super.key,
    required super.listenable,
    required this.builder,
    this.child,
  }) : super();

  AnimationController get controller => listenable as AnimationController;

  @override
  Widget build(BuildContext context) => builder(context, child);
}
