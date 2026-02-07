import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';
import 'package:flutter_monitor/shared/widgets/robot_card.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen>
    with SingleTickerProviderStateMixin {
  late AnimationController _staggerController;

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
    _staggerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 800),
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
    final provider = context.watch<RobotConnectionProvider>();
    final profileProvider = context.watch<RobotProfileProvider>();
    final profile = profileProvider.current;
    final isConnected = provider.isConnected;

    return Scaffold(
      body: Container(
        decoration: BoxDecoration(gradient: context.bgGradient),
        child: SafeArea(
          child: CustomScrollView(
            physics: const BouncingScrollPhysics(),
            slivers: [
              // ========= Header =========
              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  0,
                  Padding(
                    padding: const EdgeInsets.fromLTRB(24, 24, 24, 4),
                    child: Row(
                      children: [
                        // Logo — tap navigates to model selector
                        Material(
                          color: Colors.transparent,
                          child: InkWell(
                            borderRadius: BorderRadius.circular(AppRadius.md),
                            onTap: () {
                              HapticFeedback.selectionClick();
                              Navigator.of(context).pushNamed('/robot-select');
                            },
                            child: Ink(
                              width: 46,
                              height: 46,
                              decoration: BoxDecoration(
                                gradient: LinearGradient(
                                  begin: Alignment.topLeft,
                                  end: Alignment.bottomRight,
                                  colors: [
                                    profile.themeColor,
                                    profile.themeColor.withOpacity(0.65),
                                  ],
                                ),
                                borderRadius: BorderRadius.circular(AppRadius.md),
                                boxShadow: [
                                  AppShadows.glow(profile.themeColor),
                                ],
                              ),
                              child: Icon(
                                profile.icon,
                                color: Colors.white,
                                size: 24,
                              ),
                            ),
                          ),
                        ),
                        const SizedBox(width: 14),
                        Expanded(
                          child: GestureDetector(
                            onTap: () =>
                                Navigator.of(context).pushNamed('/robot-select'),
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Row(
                                  children: [
                                    Text(
                                      profile.name,
                                      style: TextStyle(
                                        fontSize: 22,
                                        fontWeight: FontWeight.w800,
                                        letterSpacing: -0.5,
                                        color: context.titleColor,
                                      ),
                                    ),
                                    const SizedBox(width: 6),
                                    Icon(
                                      Icons.unfold_more,
                                      size: 18,
                                      color: context.subtitleColor,
                                    ),
                                  ],
                                ),
                                Text(
                                  profile.subtitle,
                                  style: TextStyle(
                                    fontSize: 12,
                                    color: context.subtitleColor,
                                    letterSpacing: 0.3,
                                  ),
                                ),
                              ],
                            ),
                          ),
                        ),
                        // Scan button with ripple
                        Material(
                          color: Colors.transparent,
                          child: InkWell(
                            borderRadius: BorderRadius.circular(AppRadius.icon),
                            onTap: () {
                              HapticFeedback.selectionClick();
                              Navigator.of(context).pushNamed('/scan');
                            },
                            child: Ink(
                              width: 40,
                              height: 40,
                              decoration: BoxDecoration(
                                color: context.inputFillColor,
                                borderRadius: BorderRadius.circular(AppRadius.icon),
                              ),
                              child: Icon(
                                Icons.qr_code_scanner_rounded,
                                size: 20,
                                color: context.subtitleColor,
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),

              // ========= Connection Summary =========
              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  1,
                  Padding(
                    padding: const EdgeInsets.fromLTRB(
                        AppSpacing.xxl, AppSpacing.xl, AppSpacing.xxl, AppSpacing.xs),
                    child: Text(
                      isConnected ? '已连接设备' : '我的设备',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w700,
                        color: context.titleColor,
                        letterSpacing: -0.3,
                      ),
                    ),
                  ),
                ),
              ),

              // ========= Robot Card(s) =========
              if (isConnected)
                SliverToBoxAdapter(
                  child: _buildAnimatedChild(
                    2,
                    Padding(
                      padding: const EdgeInsets.fromLTRB(20, 12, 20, 0),
                      child: RobotCard(
                        name: profile.name,
                        address: _getAddress(provider),
                        connectionStatus: provider.status,
                        batteryPercent: provider
                            .latestSlowState?.resources?.batteryPercent
                            .toDouble(),
                        cpuPercent: provider
                            .latestSlowState?.resources?.cpuPercent
                            .toDouble(),
                        heroTag: 'robot-main',
                        onTap: () {
                          Navigator.of(context).pushNamed('/robot-detail');
                        },
                      ),
                    ),
                  ),
                ),

              // Empty state card when not connected
              if (!isConnected)
                SliverToBoxAdapter(
                  child: _buildAnimatedChild(
                    2,
                    Padding(
                      padding: const EdgeInsets.fromLTRB(
                          AppSpacing.screenH, AppSpacing.md, AppSpacing.screenH, 0),
                      child: Material(
                        color: Colors.transparent,
                        child: InkWell(
                          borderRadius: BorderRadius.circular(AppRadius.card),
                          onTap: () {
                            HapticFeedback.lightImpact();
                            Navigator.of(context).pushNamed('/scan');
                          },
                          child: Ink(
                            padding: const EdgeInsets.symmetric(
                                vertical: 32, horizontal: 24),
                            decoration: BoxDecoration(
                              color: context.cardColor,
                              borderRadius: BorderRadius.circular(AppRadius.card),
                              border: Border.all(
                                color: AppColors.primary.withOpacity(
                                    isDark ? 0.25 : 0.15),
                                width: 1.5,
                                strokeAlign: BorderSide.strokeAlignInside,
                              ),
                              boxShadow: [
                                isDark
                                    ? AppShadows.dark()
                                    : AppShadows.light(),
                              ],
                            ),
                            child: Column(
                              children: [
                                // Pulsing icon container
                                TweenAnimationBuilder<double>(
                                  tween: Tween(begin: 0.85, end: 1.0),
                                  duration: const Duration(seconds: 2),
                                  curve: Curves.easeInOut,
                                  builder: (context, scale, child) {
                                    return Transform.scale(
                                        scale: scale, child: child);
                                  },
                                  child: Container(
                                    width: 68,
                                    height: 68,
                                    decoration: BoxDecoration(
                                      gradient: LinearGradient(
                                        begin: Alignment.topLeft,
                                        end: Alignment.bottomRight,
                                        colors: [
                                          AppColors.primary.withOpacity(
                                              isDark ? 0.18 : 0.10),
                                          AppColors.secondary.withOpacity(
                                              isDark ? 0.12 : 0.06),
                                        ],
                                      ),
                                      shape: BoxShape.circle,
                                    ),
                                    child: Icon(
                                      Icons.add_rounded,
                                      size: 32,
                                      color: AppColors.primary
                                          .withOpacity(isDark ? 0.9 : 0.7),
                                    ),
                                  ),
                                ),
                                const SizedBox(height: AppSpacing.lg),
                                Text(
                                  '连接机器人',
                                  style: TextStyle(
                                    fontSize: 16,
                                    fontWeight: FontWeight.w700,
                                    color: context.titleColor,
                                  ),
                                ),
                                const SizedBox(height: AppSpacing.xs),
                                Text(
                                  '扫描网络或蓝牙发现设备',
                                  style: TextStyle(
                                    fontSize: 13,
                                    color: context.subtitleColor,
                                  ),
                                ),
                              ],
                            ),
                          ),
                        ),
                      ),
                    ),
                  ),
                ),

              // ========= Quick Actions =========
              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  3,
                  Padding(
                    padding: const EdgeInsets.fromLTRB(
                        AppSpacing.xxl, 28, AppSpacing.xxl, AppSpacing.md),
                    child: Text(
                      '快速操作',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w700,
                        color: context.titleColor,
                        letterSpacing: -0.3,
                      ),
                    ),
                  ),
                ),
              ),

              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  4,
                  Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 20),
                    child: Row(
                      children: [
                        Expanded(
                          child: _QuickActionButton(
                            icon: Icons.wifi_find,
                            label: '扫描设备',
                            color: AppColors.primary,
                            onTap: () =>
                                Navigator.of(context).pushNamed('/scan'),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: _QuickActionButton(
                            icon: Icons.pets,
                            label: '选择型号',
                            color: profile.themeColor,
                            onTap: () =>
                                Navigator.of(context).pushNamed('/robot-select'),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: _QuickActionButton(
                            icon: Icons.science_outlined,
                            label: '演示模式',
                            color: AppColors.warning,
                            onTap: () => _startMock(context),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),

              // ========= App Info =========
              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  5,
                  Padding(
                    padding: const EdgeInsets.fromLTRB(
                        AppSpacing.xxl, 28, AppSpacing.xxl, AppSpacing.md),
                    child: Text(
                      'APP 信息',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w700,
                        color: context.titleColor,
                        letterSpacing: -0.3,
                      ),
                    ),
                  ),
                ),
              ),

              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  6,
                  Padding(
                    padding: const EdgeInsets.symmetric(
                        horizontal: AppSpacing.screenH),
                    child: Container(
                      padding: const EdgeInsets.all(AppSpacing.lg),
                      decoration: context.cardDecoration,
                      child: Column(
                        children: [
                          _InfoRow(
                            icon: Icons.info_outline,
                            label: '版本',
                            value: 'v1.0.0',
                          ),
                          Divider(height: 16, color: context.dividerColor),
                          _InfoRow(
                            icon: Icons.router_outlined,
                            label: '协议',
                            value: 'gRPC + WebRTC',
                          ),
                          Divider(height: 16, color: context.dividerColor),
                          _InfoRow(
                            icon: Icons.network_check,
                            label: '网络状态',
                            value: isConnected ? '已连接' : '未连接',
                            valueColor:
                                isConnected ? AppColors.success : Colors.grey,
                          ),
                          Divider(height: 16, color: context.dividerColor),
                          _InfoRow(
                            icon: Icons.precision_manufacturing_outlined,
                            label: '平台',
                            value: 'ROS 2',
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),

              // Footer
              SliverToBoxAdapter(
                child: Padding(
                  padding: const EdgeInsets.symmetric(vertical: 32),
                  child: Center(
                    child: Text(
                      'gRPC · WebRTC · ROS 2 · BLE',
                      style: TextStyle(
                        fontSize: 11,
                        color: context.subtitleColor,
                        letterSpacing: 1.5,
                      ),
                    ),
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
    final delay = index * 0.1;
    final animation = CurvedAnimation(
      parent: _staggerController,
      curve: Interval(delay.clamp(0.0, 0.7), (delay + 0.3).clamp(0.0, 1.0),
          curve: Curves.easeOutCubic),
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

  String _getAddress(RobotConnectionProvider provider) {
    final client = provider.client;
    if (client == null) return '未知';
    // Try to get host from client
    return client.toString().contains('host')
        ? client.toString()
        : 'gRPC 连接';
  }

  Future<void> _startMock(BuildContext context) async {
    HapticFeedback.lightImpact();
    final provider = context.read<RobotConnectionProvider>();
    final client = MockRobotClient();
    await provider.connect(client);
    if (mounted) {
      Navigator.of(context).pushNamed('/robot-detail');
    }
  }
}

// ==================== Quick Action Button ====================

class _QuickActionButton extends StatefulWidget {
  final IconData icon;
  final String label;
  final Color color;
  final VoidCallback? onTap;

  const _QuickActionButton({
    required this.icon,
    required this.label,
    required this.color,
    this.onTap,
  });

  @override
  State<_QuickActionButton> createState() => _QuickActionButtonState();
}

class _QuickActionButtonState extends State<_QuickActionButton>
    with SingleTickerProviderStateMixin {
  double _scale = 1.0;

  void _onTapDown(TapDownDetails _) => setState(() => _scale = 0.95);
  void _onTapUp(TapUpDetails _) => setState(() => _scale = 1.0);
  void _onTapCancel() => setState(() => _scale = 1.0);

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return GestureDetector(
      onTapDown: _onTapDown,
      onTapUp: _onTapUp,
      onTapCancel: _onTapCancel,
      onTap: () {
        HapticFeedback.selectionClick();
        widget.onTap?.call();
      },
      child: AnimatedScale(
        scale: _scale,
        duration: AppDurations.fast,
        curve: Curves.easeOut,
        child: Container(
          padding: const EdgeInsets.symmetric(vertical: AppSpacing.lg),
          decoration: BoxDecoration(
            color: context.cardColor,
            borderRadius: BorderRadius.circular(AppRadius.lg),
            boxShadow: [
              isDark ? AppShadows.dark() : AppShadows.light(),
            ],
          ),
          child: Column(
            children: [
              Container(
                width: 42,
                height: 42,
                decoration: BoxDecoration(
                  color: widget.color.withOpacity(isDark ? 0.18 : 0.1),
                  borderRadius: BorderRadius.circular(AppRadius.icon),
                ),
                child: Icon(widget.icon, color: widget.color, size: 20),
              ),
              const SizedBox(height: AppSpacing.sm),
              Text(
                widget.label,
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                  color: isDark ? Colors.white70 : Colors.black87,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// ==================== Info Row ====================

class _InfoRow extends StatelessWidget {
  final IconData icon;
  final String label;
  final String value;
  final Color? valueColor;

  const _InfoRow({
    required this.icon,
    required this.label,
    required this.value,
    this.valueColor,
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Icon(icon, size: 18, color: context.subtitleColor),
        const SizedBox(width: 10),
        Text(
          label,
          style: TextStyle(
            fontSize: 14,
            color: context.subtitleColor,
          ),
        ),
        const Spacer(),
        Text(
          value,
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w600,
            color: valueColor ?? context.titleColor,
          ),
        ),
      ],
    );
  }
}

