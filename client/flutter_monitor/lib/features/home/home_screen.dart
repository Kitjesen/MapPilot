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
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Row(
                          children: [
                            Image.asset(
                              'assets/logo.png',
                              width: 22,
                              height: 22,
                              color: context.isDark ? Colors.white70 : const Color(0xFF0A1E3D),
                            ),
                            const SizedBox(width: 8),
                            Text(
                              '大算机器人',
                              style: TextStyle(
                                fontSize: 15,
                                fontWeight: FontWeight.w700,
                                letterSpacing: 0.3,
                                color: context.titleColor,
                              ),
                            ),
                            const Spacer(),
                            IconButton(
                              onPressed: () {
                                HapticFeedback.selectionClick();
                                Navigator.of(context).pushNamed('/scan');
                              },
                              icon: Icon(
                                Icons.search,
                                size: 20,
                                color: context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                        Padding(
                          padding: const EdgeInsets.only(left: 30),
                          child: Text(
                            '让每一台机器人都拥有智慧',
                            style: TextStyle(
                              fontSize: 11,
                              fontWeight: FontWeight.w400,
                              color: context.subtitleColor,
                              letterSpacing: 0.3,
                            ),
                          ),
                        ),
                        const SizedBox(height: 12),
                        // Profile selector
                        GestureDetector(
                          onTap: () =>
                              Navigator.of(context).pushNamed('/robot-select'),
                          child: Row(
                            children: [
                              Text(
                                profile.name,
                                style: TextStyle(
                                  fontSize: 18,
                                  fontWeight: FontWeight.w700,
                                  letterSpacing: -0.3,
                                  color: context.titleColor,
                                ),
                              ),
                              const SizedBox(width: 4),
                              Icon(
                                Icons.keyboard_arrow_down,
                                size: 16,
                                color: context.subtitleColor,
                              ),
                            ],
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
                    padding: const EdgeInsets.fromLTRB(24, 20, 24, 4),
                    child: Text(
                      isConnected ? '已连接设备' : '我的设备',
                      style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w600,
                        color: context.subtitleColor,
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
                            .latestSlowState?.resources.batteryPercent
                            .toDouble(),
                        cpuPercent: provider
                            .latestSlowState?.resources.cpuPercent
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
                          borderRadius: BorderRadius.circular(10),
                          onTap: () {
                            HapticFeedback.lightImpact();
                            Navigator.of(context).pushNamed('/scan');
                          },
                          child: Ink(
                            padding: const EdgeInsets.symmetric(
                                vertical: 16, horizontal: 16),
                            decoration: BoxDecoration(
                              color: context.cardColor,
                              borderRadius: BorderRadius.circular(10),
                              border: Border.all(
                                color: context.borderColor,
                              ),
                            ),
                            child: Row(
                              children: [
                                Icon(
                                  Icons.add_circle_outline,
                                  size: 20,
                                  color: context.subtitleColor,
                                ),
                                const SizedBox(width: 12),
                                Expanded(
                                  child: Column(
                                    crossAxisAlignment: CrossAxisAlignment.start,
                                    children: [
                                      Text(
                                        '连接机器人',
                                        style: TextStyle(
                                          fontSize: 14,
                                          fontWeight: FontWeight.w500,
                                          color: context.titleColor,
                                        ),
                                      ),
                                      Text(
                                        '扫描网络或蓝牙发现设备',
                                        style: TextStyle(
                                          fontSize: 12,
                                          color: context.subtitleColor,
                                        ),
                                      ),
                                    ],
                                  ),
                                ),
                                Icon(
                                  Icons.chevron_right,
                                  size: 16,
                                  color: context.subtitleColor,
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
                    padding: const EdgeInsets.fromLTRB(24, 20, 24, 8),
                    child: Text(
                      '快速操作',
                      style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w600,
                        color: context.subtitleColor,
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
                    child: Column(
                      children: [
                        Row(
                          children: [
                            Expanded(
                              child: _QuickActionButton(
                                icon: Icons.wifi_find,
                                label: '扫描设备',
                                onTap: () =>
                                    Navigator.of(context).pushNamed('/scan'),
                              ),
                            ),
                            const SizedBox(width: 10),
                            Expanded(
                              child: _QuickActionButton(
                                icon: Icons.widgets_outlined,
                                label: '选择型号',
                                onTap: () =>
                                    Navigator.of(context).pushNamed('/robot-select'),
                              ),
                            ),
                            const SizedBox(width: 10),
                            Expanded(
                              child: _QuickActionButton(
                                icon: Icons.play_circle_outline,
                                label: '演示模式',
                                onTap: () => _startMock(context),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 12),
                        Row(
                          children: [
                            Expanded(
                              child: _QuickActionButton(
                                icon: Icons.navigation_outlined,
                                label: '任务导航',
                                onTap: () =>
                                    Navigator.of(context).pushNamed('/task-panel'),
                              ),
                            ),
                            const SizedBox(width: 10),
                            Expanded(
                              child: _QuickActionButton(
                                icon: Icons.folder_outlined,
                                label: '地图管理',
                                onTap: () =>
                                    Navigator.of(context).pushNamed('/map-manager'),
                              ),
                            ),
                            const SizedBox(width: 12),
                            const Expanded(child: SizedBox()), // placeholder for alignment
                          ],
                        ),
                      ],
                    ),
                  ),
                ),
              ),

              // Bottom spacing for floating nav bar
              const SliverToBoxAdapter(
                child: SizedBox(height: 100),
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
  final VoidCallback? onTap;

  const _QuickActionButton({
    required this.icon,
    required this.label,
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
          padding: const EdgeInsets.symmetric(vertical: 14),
          decoration: BoxDecoration(
            color: context.isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(10),
            border: Border.all(color: context.borderColor),
          ),
          child: Column(
            children: [
              Icon(widget.icon, color: context.subtitleColor, size: 18),
              const SizedBox(height: 6),
              Text(
                widget.label,
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w500,
                  color: context.titleColor,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}


