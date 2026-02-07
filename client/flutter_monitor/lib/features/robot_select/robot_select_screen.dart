import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/models/robot_profile.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

/// 机器人型号选择页面
///
/// 以卡片沙盒形式展示所有已注册的机器人型号。
/// 点击卡片展开详情，选择后自动返回。
class RobotSelectScreen extends StatefulWidget {
  const RobotSelectScreen({super.key});

  @override
  State<RobotSelectScreen> createState() => _RobotSelectScreenState();
}

class _RobotSelectScreenState extends State<RobotSelectScreen>
    with SingleTickerProviderStateMixin {
  String? _expandedId;
  late AnimationController _staggerController;

  @override
  void initState() {
    super.initState();
    _staggerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 600),
    )..forward();
  }

  @override
  void dispose() {
    _staggerController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final profileProvider = context.watch<RobotProfileProvider>();
    final profiles = profileProvider.allProfiles;
    final currentId = profileProvider.current.id;

    return Scaffold(
      body: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: isDark
                ? [
                    const Color(0xFF0A0A0A),
                    const Color(0xFF0E0E1A),
                    const Color(0xFF0A0A0A),
                  ]
                : [
                    const Color(0xFFF2F2F7),
                    const Color(0xFFE8ECF4),
                    const Color(0xFFF2F2F7),
                  ],
          ),
        ),
        child: SafeArea(
          child: CustomScrollView(
            physics: const BouncingScrollPhysics(),
            slivers: [
              // Header
              SliverToBoxAdapter(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(24, 16, 24, 0),
                  child: Row(
                    children: [
                      IconButton(
                        icon: Icon(Icons.arrow_back_ios_new,
                            size: 20,
                            color: isDark ? Colors.white : Colors.black87),
                        onPressed: () => Navigator.pop(context),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              '选择机器人',
                              style: TextStyle(
                                fontSize: 22,
                                fontWeight: FontWeight.w800,
                                letterSpacing: -0.5,
                                color: isDark ? Colors.white : Colors.black87,
                              ),
                            ),
                            Text(
                              '轻触卡片查看详情，选择型号开始使用',
                              style: TextStyle(
                                fontSize: 12,
                                color: context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),

              // Current selection indicator
              SliverToBoxAdapter(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(24, 20, 24, 8),
                  child: Row(
                    children: [
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 12, vertical: 6),
                        decoration: BoxDecoration(
                          color:
                              profileProvider.current.themeColor.withOpacity(0.12),
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(
                              profileProvider.current.icon,
                              size: 14,
                              color: profileProvider.current.themeColor,
                            ),
                            const SizedBox(width: 6),
                            Text(
                              '当前: ${profileProvider.current.name}',
                              style: TextStyle(
                                fontSize: 12,
                                fontWeight: FontWeight.w700,
                                color: profileProvider.current.themeColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),

              // Robot Cards
              SliverPadding(
                padding: const EdgeInsets.fromLTRB(20, 8, 20, 32),
                sliver: SliverList(
                  delegate: SliverChildBuilderDelegate(
                    (context, index) {
                      final profile = profiles[index];
                      final isSelected = profile.id == currentId;
                      final isExpanded = profile.id == _expandedId;

                      return _buildAnimatedChild(
                        index,
                        Padding(
                          padding: const EdgeInsets.only(bottom: 14),
                          child: _RobotSandboxCard(
                            profile: profile,
                            isSelected: isSelected,
                            isExpanded: isExpanded,
                            onTap: () {
                              HapticFeedback.selectionClick();
                              setState(() {
                                _expandedId =
                                    isExpanded ? null : profile.id;
                              });
                            },
                            onSelect: () async {
                              HapticFeedback.mediumImpact();
                              await profileProvider.select(profile);
                              if (context.mounted) {
                                ScaffoldMessenger.of(context).showSnackBar(
                                  SnackBar(
                                    content: Text(
                                        '已选择 ${profile.name}'),
                                    behavior: SnackBarBehavior.floating,
                                    duration: const Duration(seconds: 1),
                                  ),
                                );
                              }
                            },
                          ),
                        ),
                      );
                    },
                    childCount: profiles.length,
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildAnimatedChild(int index, Widget child) {
    final delay = index * 0.12;
    final animation = CurvedAnimation(
      parent: _staggerController,
      curve: Interval(
        delay.clamp(0.0, 0.7),
        (delay + 0.3).clamp(0.0, 1.0),
        curve: Curves.easeOutCubic,
      ),
    );
    return FadeTransition(
      opacity: animation,
      child: SlideTransition(
        position: Tween<Offset>(
          begin: const Offset(0, 0.08),
          end: Offset.zero,
        ).animate(animation),
        child: child,
      ),
    );
  }
}

/// 沙盒卡片 — 可展开的机器人型号卡片
class _RobotSandboxCard extends StatelessWidget {
  final RobotProfile profile;
  final bool isSelected;
  final bool isExpanded;
  final VoidCallback onTap;
  final VoidCallback onSelect;

  const _RobotSandboxCard({
    required this.profile,
    required this.isSelected,
    required this.isExpanded,
    required this.onTap,
    required this.onSelect,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final color = profile.themeColor;

    return GestureDetector(
      onTap: onTap,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 350),
        curve: Curves.easeOutCubic,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(22),
          color: isDark ? AppColors.darkCard : Colors.white,
          border: Border.all(
            color: isSelected ? color.withOpacity(0.5) : Colors.transparent,
            width: 2,
          ),
          boxShadow: [
            BoxShadow(
              color: isSelected
                  ? color.withOpacity(isDark ? 0.25 : 0.15)
                  : context.cardShadowColor,
              blurRadius: isSelected ? 24 : 16,
              offset: const Offset(0, 8),
            ),
          ],
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(22),
          child: Column(
            children: [
              // ─── Collapsed Header (always visible) ───
              Padding(
                padding: const EdgeInsets.all(18),
                child: Row(
                  children: [
                    // Robot icon
                    Container(
                      width: 52,
                      height: 52,
                      decoration: BoxDecoration(
                        gradient: LinearGradient(
                          begin: Alignment.topLeft,
                          end: Alignment.bottomRight,
                          colors: profile.available
                              ? [color, color.withOpacity(0.7)]
                              : [Colors.grey.shade400, Colors.grey.shade500],
                        ),
                        borderRadius: BorderRadius.circular(16),
                        boxShadow: [
                          BoxShadow(
                            color: color.withOpacity(0.3),
                            blurRadius: 12,
                            offset: const Offset(0, 4),
                          ),
                        ],
                      ),
                      child: Icon(profile.icon, color: Colors.white, size: 26),
                    ),
                    const SizedBox(width: 14),
                    // Name + subtitle
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Row(
                            children: [
                              Text(
                                profile.name,
                                style: TextStyle(
                                  fontSize: 18,
                                  fontWeight: FontWeight.w700,
                                  color: isDark ? Colors.white : Colors.black87,
                                  letterSpacing: -0.3,
                                ),
                              ),
                              if (!profile.available) ...[
                                const SizedBox(width: 8),
                                Container(
                                  padding: const EdgeInsets.symmetric(
                                      horizontal: 8, vertical: 2),
                                  decoration: BoxDecoration(
                                    color: Colors.grey.withOpacity(0.15),
                                    borderRadius: BorderRadius.circular(8),
                                  ),
                                  child: Text(
                                    '即将推出',
                                    style: TextStyle(
                                      fontSize: 10,
                                      fontWeight: FontWeight.w600,
                                      color: context.subtitleColor,
                                    ),
                                  ),
                                ),
                              ],
                            ],
                          ),
                          const SizedBox(height: 2),
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
                    // Selection checkmark / expand indicator
                    if (isSelected)
                      Container(
                        width: 28,
                        height: 28,
                        decoration: BoxDecoration(
                          color: color,
                          shape: BoxShape.circle,
                        ),
                        child:
                            const Icon(Icons.check, color: Colors.white, size: 16),
                      )
                    else
                      AnimatedRotation(
                        turns: isExpanded ? 0.5 : 0.0,
                        duration: const Duration(milliseconds: 300),
                        child: Icon(
                          Icons.keyboard_arrow_down,
                          color: context.subtitleColor,
                        ),
                      ),
                  ],
                ),
              ),

              // ─── Quick specs row (always visible) ───
              Padding(
                padding: const EdgeInsets.fromLTRB(18, 0, 18, 14),
                child: Row(
                  children: [
                    _SpecChip(
                      icon: Icons.precision_manufacturing,
                      label: '${profile.jointCount}J',
                      color: color,
                    ),
                    const SizedBox(width: 8),
                    _SpecChip(
                      icon: Icons.speed,
                      label: '${profile.maxLinearSpeed} m/s',
                      color: color,
                    ),
                    const SizedBox(width: 8),
                    _SpecChip(
                      icon: Icons.fitness_center,
                      label: '${profile.mass} kg',
                      color: color,
                    ),
                    const Spacer(),
                    Text(
                      profile.protoType,
                      style: TextStyle(
                        fontSize: 11,
                        fontWeight: FontWeight.w600,
                        color: color.withOpacity(0.6),
                        fontFamily: 'monospace',
                        letterSpacing: 0.5,
                      ),
                    ),
                  ],
                ),
              ),

              // ─── Expanded Detail Panel ───
              AnimatedCrossFade(
                firstChild: const SizedBox.shrink(),
                secondChild: _buildExpandedPanel(context),
                crossFadeState: isExpanded
                    ? CrossFadeState.showSecond
                    : CrossFadeState.showFirst,
                duration: const Duration(milliseconds: 300),
                sizeCurve: Curves.easeOutCubic,
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildExpandedPanel(BuildContext context) {
    final isDark = context.isDark;
    final color = profile.themeColor;

    return Container(
      decoration: BoxDecoration(
        color: isDark
            ? Colors.white.withOpacity(0.03)
            : Colors.black.withOpacity(0.02),
      ),
      padding: const EdgeInsets.fromLTRB(18, 14, 18, 18),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Divider
          Container(
            height: 1,
            color: context.dividerColor,
            margin: const EdgeInsets.only(bottom: 14),
          ),

          // Description
          Text(
            profile.description,
            style: TextStyle(
              fontSize: 13,
              height: 1.5,
              color: isDark ? Colors.white70 : Colors.black54,
            ),
          ),
          const SizedBox(height: 16),

          // Detail specs grid
          Row(
            children: [
              Expanded(
                child: _DetailItem(
                  label: '关节数',
                  value: '${profile.jointCount}',
                  icon: Icons.precision_manufacturing,
                ),
              ),
              Expanded(
                child: _DetailItem(
                  label: '腿数',
                  value: '${profile.legNames.length}',
                  icon: Icons.pets,
                ),
              ),
              Expanded(
                child: _DetailItem(
                  label: '每腿关节',
                  value: '${profile.jointsPerLeg}',
                  icon: Icons.settings,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Expanded(
                child: _DetailItem(
                  label: '最大线速',
                  value: '${profile.maxLinearSpeed} m/s',
                  icon: Icons.speed,
                ),
              ),
              Expanded(
                child: _DetailItem(
                  label: '最大角速',
                  value: '${profile.maxAngularSpeed} r/s',
                  icon: Icons.rotate_right,
                ),
              ),
              Expanded(
                child: _DetailItem(
                  label: '质量',
                  value: '${profile.mass} kg',
                  icon: Icons.fitness_center,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),

          // Network defaults
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: isDark
                  ? Colors.white.withOpacity(0.04)
                  : Colors.black.withOpacity(0.03),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Row(
              children: [
                Icon(Icons.wifi, size: 16, color: context.subtitleColor),
                const SizedBox(width: 8),
                Text(
                  '${profile.defaultHost}:${profile.defaultPort}',
                  style: TextStyle(
                    fontSize: 13,
                    fontFamily: 'monospace',
                    color: isDark ? Colors.white70 : Colors.black87,
                  ),
                ),
                const Spacer(),
                if (profile.urdfAsset != null)
                  Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.view_in_ar, size: 14, color: color),
                      const SizedBox(width: 4),
                      Text(
                        '3D',
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          color: color,
                        ),
                      ),
                    ],
                  )
                else
                  Text(
                    'No URDF',
                    style: TextStyle(
                      fontSize: 11,
                      color: context.subtitleColor,
                    ),
                  ),
              ],
            ),
          ),
          const SizedBox(height: 16),

          // Action button
          SizedBox(
            width: double.infinity,
            height: 48,
            child: ElevatedButton(
              onPressed: profile.available ? onSelect : null,
              style: ElevatedButton.styleFrom(
                backgroundColor: color,
                foregroundColor: Colors.white,
                disabledBackgroundColor: Colors.grey.withOpacity(0.3),
                disabledForegroundColor: Colors.grey,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(14),
                ),
                elevation: 0,
              ),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(
                    isSelected ? Icons.check_circle : Icons.radio_button_unchecked,
                    size: 18,
                  ),
                  const SizedBox(width: 8),
                  Text(
                    isSelected
                        ? '当前已选择'
                        : (profile.available ? '选择此型号' : '敬请期待'),
                    style: const TextStyle(
                      fontSize: 15,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

// ─── Helper Widgets ───

class _SpecChip extends StatelessWidget {
  final IconData icon;
  final String label;
  final Color color;

  const _SpecChip({
    required this.icon,
    required this.label,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: color.withOpacity(isDark ? 0.12 : 0.08),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 12, color: color),
          const SizedBox(width: 4),
          Text(
            label,
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.w600,
              color: color,
            ),
          ),
        ],
      ),
    );
  }
}

class _DetailItem extends StatelessWidget {
  final String label;
  final String value;
  final IconData icon;

  const _DetailItem({
    required this.label,
    required this.value,
    required this.icon,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return Column(
      children: [
        Icon(icon, size: 18, color: context.subtitleColor),
        const SizedBox(height: 6),
        Text(
          value,
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w700,
            color: isDark ? Colors.white : Colors.black87,
          ),
        ),
        const SizedBox(height: 2),
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            color: context.subtitleColor,
          ),
        ),
      ],
    );
  }
}
