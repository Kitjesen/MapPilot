import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../theme/app_theme.dart';
import '../services/robot_connection_provider.dart';
import '../services/mock_robot_client.dart';
import '../widgets/robot_card.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen>
    with SingleTickerProviderStateMixin {
  late AnimationController _staggerController;

  late AnimationController _navAnimController;
  late Animation<double> _navSlide;

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
    final isConnected = provider.isConnected;

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
              // ========= Header =========
              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  0,
                  Padding(
                    padding: const EdgeInsets.fromLTRB(24, 24, 24, 4),
                    child: Row(
                      children: [
                        // Logo
                        Container(
                          width: 46,
                          height: 46,
                          decoration: BoxDecoration(
                            gradient: const LinearGradient(
                              begin: Alignment.topLeft,
                              end: Alignment.bottomRight,
                              colors: [AppColors.primary, AppColors.secondary],
                            ),
                            borderRadius: BorderRadius.circular(14),
                            boxShadow: [
                              BoxShadow(
                                color: AppColors.primary.withOpacity(0.25),
                                blurRadius: 16,
                                offset: const Offset(0, 6),
                              ),
                            ],
                          ),
                          child: const Icon(
                            Icons.smart_toy_outlined,
                            color: Colors.white,
                            size: 24,
                          ),
                        ),
                        const SizedBox(width: 14),
                        Expanded(
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                '大算机器人',
                                style: TextStyle(
                                  fontSize: 22,
                                  fontWeight: FontWeight.w800,
                                  letterSpacing: -0.5,
                                  color: isDark ? Colors.white : Colors.black87,
                                ),
                              ),
                              Text(
                                'Robot Monitor & Control',
                                style: TextStyle(
                                  fontSize: 12,
                                  color: context.subtitleColor,
                                  letterSpacing: 0.3,
                                ),
                              ),
                            ],
                          ),
                        ),
                        // Scan button
                        GestureDetector(
                          onTap: () {
                            HapticFeedback.selectionClick();
                            Navigator.of(context).pushNamed('/scan');
                          },
                          child: Container(
                            width: 40,
                            height: 40,
                            decoration: BoxDecoration(
                              color: context.inputFillColor,
                              borderRadius: BorderRadius.circular(12),
                            ),
                            child: Icon(
                              Icons.qr_code_scanner_rounded,
                              size: 20,
                              color: context.subtitleColor,
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
                    padding: const EdgeInsets.fromLTRB(24, 20, 24, 4),
                    child: Text(
                      isConnected ? '已连接设备' : '我的设备',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w700,
                        color: isDark ? Colors.white : Colors.black87,
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
                        name: '大算机器人',
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
                      padding: const EdgeInsets.fromLTRB(20, 12, 20, 0),
                      child: GestureDetector(
                        onTap: () =>
                            Navigator.of(context).pushNamed('/scan'),
                        child: Container(
                          padding: const EdgeInsets.all(28),
                          decoration: BoxDecoration(
                            color: isDark ? AppColors.darkCard : Colors.white,
                            borderRadius: BorderRadius.circular(20),
                            border: Border.all(
                              color: AppColors.primary.withOpacity(0.2),
                              width: 1.5,
                              strokeAlign: BorderSide.strokeAlignInside,
                            ),
                            boxShadow: [
                              BoxShadow(
                                color: context.cardShadowColor,
                                blurRadius: 20,
                                offset: const Offset(0, 8),
                              ),
                            ],
                          ),
                          child: Column(
                            children: [
                              Container(
                                width: 64,
                                height: 64,
                                decoration: BoxDecoration(
                                  color: AppColors.primary.withOpacity(
                                      isDark ? 0.12 : 0.08),
                                  shape: BoxShape.circle,
                                ),
                                child: Icon(
                                  Icons.add_rounded,
                                  size: 32,
                                  color: AppColors.primary.withOpacity(0.7),
                                ),
                              ),
                              const SizedBox(height: 14),
                              Text(
                                '连接机器人',
                                style: TextStyle(
                                  fontSize: 16,
                                  fontWeight: FontWeight.w600,
                                  color:
                                      isDark ? Colors.white : Colors.black87,
                                ),
                              ),
                              const SizedBox(height: 4),
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

              // ========= Quick Actions =========
              SliverToBoxAdapter(
                child: _buildAnimatedChild(
                  3,
                  Padding(
                    padding: const EdgeInsets.fromLTRB(24, 28, 24, 12),
                    child: Text(
                      '快速操作',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w700,
                        color: isDark ? Colors.white : Colors.black87,
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
                            icon: Icons.bluetooth_searching,
                            label: '蓝牙连接',
                            color: AppColors.secondary,
                            onTap: () =>
                                Navigator.of(context).pushNamed('/scan'),
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
                    padding: const EdgeInsets.fromLTRB(24, 28, 24, 12),
                    child: Text(
                      'APP 信息',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w700,
                        color: isDark ? Colors.white : Colors.black87,
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
                    padding: const EdgeInsets.symmetric(horizontal: 20),
                    child: Container(
                      padding: const EdgeInsets.all(16),
                      decoration: BoxDecoration(
                        color: isDark ? AppColors.darkCard : Colors.white,
                        borderRadius: BorderRadius.circular(16),
                        boxShadow: [
                          BoxShadow(
                            color: context.cardShadowColor,
                            blurRadius: 12,
                            offset: const Offset(0, 4),
                          ),
                        ],
                      ),
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

class _QuickActionButton extends StatelessWidget {
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
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return GestureDetector(
      onTap: () {
        HapticFeedback.selectionClick();
        onTap?.call();
      },
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 16),
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(16),
          boxShadow: [
            BoxShadow(
              color: context.cardShadowColor,
              blurRadius: 10,
              offset: const Offset(0, 4),
            ),
          ],
        ),
        child: Column(
          children: [
            Container(
              width: 40,
              height: 40,
              decoration: BoxDecoration(
                color: color.withOpacity(isDark ? 0.18 : 0.1),
                borderRadius: BorderRadius.circular(12),
              ),
              child: Icon(icon, color: color, size: 20),
            ),
            const SizedBox(height: 8),
            Text(
              label,
              style: TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white70 : Colors.black87,
              ),
            ),
          ],
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
    final isDark = context.isDark;

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
            color: valueColor ?? (isDark ? Colors.white : Colors.black87),
          ),
        ),
      ],
    );
  }
}

