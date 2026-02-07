import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/shared/widgets/feature_card.dart';
import 'package:flutter_monitor/features/status/status_screen.dart';
import 'package:flutter_monitor/features/control/control_screen.dart';
import 'package:flutter_monitor/features/map/map_screen.dart';
import 'package:flutter_monitor/features/events/events_screen.dart';
import 'package:flutter_monitor/features/settings/app_settings_screen.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

class RobotDetailScreen extends StatefulWidget {
  const RobotDetailScreen({super.key});

  @override
  State<RobotDetailScreen> createState() => _RobotDetailScreenState();
}

class _RobotDetailScreenState extends State<RobotDetailScreen> {
  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
  }

  Future<void> _handleDisconnect() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('断开连接'),
        content: const Text('确定要断开与机器人的连接吗？'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: const Text('取消'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: const Text('断开'),
          ),
        ],
      ),
    );

    if (confirmed == true && mounted) {
      final provider = context.read<RobotConnectionProvider>();
      await provider.disconnect();
      if (mounted) {
        Navigator.of(context).pushNamedAndRemoveUntil('/main', (_) => false);
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      body: Consumer<RobotConnectionProvider>(
        builder: (context, provider, _) {
          final fastState = provider.latestFastState;
          final slowState = provider.latestSlowState;
          final isConnected = provider.isConnected;
          final profile = context.watch<RobotProfileProvider>().current;

          return CustomScrollView(
            physics: const BouncingScrollPhysics(),
            slivers: [
              // App Bar
              SliverAppBar(
                expandedHeight: 140,
                pinned: true,
                leading: IconButton(
                  icon: const Icon(Icons.arrow_back_ios_new, size: 20),
                  onPressed: () => Navigator.pop(context),
                ),
                actions: [
                  IconButton(
                    icon: Icon(
                      Icons.power_settings_new,
                      color: AppColors.error.withOpacity(0.7),
                    ),
                    onPressed: _handleDisconnect,
                  ),
                ],
                flexibleSpace: FlexibleSpaceBar(
                  titlePadding:
                      const EdgeInsets.only(left: 56, bottom: 14, right: 16),
                  title: Column(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        profile.name,
                        style: TextStyle(
                          fontSize: 17,
                          fontWeight: FontWeight.w700,
                          color: isDark ? Colors.white : Colors.black87,
                        ),
                      ),
                      Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Container(
                            width: 7,
                            height: 7,
                            decoration: BoxDecoration(
                              color: isConnected
                                  ? AppColors.success
                                  : AppColors.error,
                              shape: BoxShape.circle,
                            ),
                          ),
                          const SizedBox(width: 5),
                          Text(
                            isConnected ? '已连接' : '未连接',
                            style: TextStyle(
                              fontSize: 11,
                              fontWeight: FontWeight.w500,
                              color: isConnected
                                  ? AppColors.success
                                  : AppColors.error,
                            ),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              ),

              // Connection status banner
              if (provider.status == ConnectionStatus.reconnecting ||
                  provider.status == ConnectionStatus.error)
                SliverToBoxAdapter(
                  child: _buildStatusBanner(provider),
                ),

              // Quick stats
              SliverToBoxAdapter(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(16, 8, 16, 8),
                  child: _buildQuickStats(slowState, fastState),
                ),
              ),

              // Feature cards heading
              SliverToBoxAdapter(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(20, 16, 20, 12),
                  child: Text(
                    '功能模块',
                    style: TextStyle(
                      fontSize: 20,
                      fontWeight: FontWeight.w700,
                      color: isDark ? Colors.white : Colors.black87,
                      letterSpacing: -0.3,
                    ),
                  ),
                ),
              ),

              // Feature cards grid
              SliverPadding(
                padding: const EdgeInsets.symmetric(horizontal: 16),
                sliver: SliverGrid(
                  gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 2,
                    mainAxisSpacing: 12,
                    crossAxisSpacing: 12,
                    childAspectRatio: 1.05,
                  ),
                  delegate: SliverChildListDelegate([
                    FeatureCard(
                      icon: Icons.dashboard_outlined,
                      title: '状态监控',
                      subtitle: _buildStatusSubtitle(slowState),
                      color: AppColors.primary,
                      onTap: () => _pushScreen(const StatusScreen()),
                      trailing: _buildMiniMetrics(slowState),
                    ),
                    FeatureCard(
                      icon: Icons.gamepad_outlined,
                      title: '遥控操作',
                      subtitle: provider.hasLease ? '租约已获取' : '需要获取租约',
                      color: AppColors.accent,
                      onTap: () => _pushScreen(const ControlScreen()),
                      badge: provider.hasLease
                          ? _buildBadge('LIVE', AppColors.success)
                          : null,
                    ),
                    FeatureCard(
                      icon: Icons.map_outlined,
                      title: '地图导航',
                      subtitle: _poseText(fastState),
                      color: AppColors.success,
                      onTap: () => _pushScreen(const MapScreen()),
                    ),
                    FeatureCard(
                      icon: Icons.notifications_outlined,
                      title: '事件日志',
                      subtitle: '查看系统事件',
                      color: AppColors.warning,
                      onTap: () => _pushScreen(const EventsScreen()),
                    ),
                    FeatureCard(
                      icon: Icons.folder_outlined,
                      title: '文件管理',
                      subtitle: '模型/地图/配置',
                      color: AppColors.info,
                      onTap: () => _pushScreen(const AppSettingsScreen()),
                    ),
                    FeatureCard(
                      icon: Icons.videocam_outlined,
                      title: '相机画面',
                      subtitle: '实时视频流',
                      color: const Color(0xFFFF2D55),
                      onTap: () {
                        // Future: dedicated camera screen
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(content: Text('相机功能开发中...')),
                        );
                      },
                    ),
                  ]),
                ),
              ),

              // Bottom padding
              const SliverToBoxAdapter(child: SizedBox(height: 32)),
            ],
          );
        },
      ),
    );
  }

  void _pushScreen(Widget screen) {
    Navigator.of(context).push(
      PageRouteBuilder(
        pageBuilder: (_, __, ___) => screen,
        transitionsBuilder: (_, a, __, child) {
          return FadeTransition(
            opacity: CurvedAnimation(parent: a, curve: Curves.easeOut),
            child: child,
          );
        },
        transitionDuration: const Duration(milliseconds: 250),
      ),
    );
  }

  Widget _buildStatusBanner(RobotConnectionProvider provider) {
    final isReconnecting = provider.status == ConnectionStatus.reconnecting;
    final color = isReconnecting ? AppColors.warning : AppColors.error;
    final text = isReconnecting
        ? '正在重新连接...'
        : provider.errorMessage ?? '连接错误';

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      decoration: BoxDecoration(
        color: color.withOpacity(0.12),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          Icon(
            isReconnecting ? Icons.sync : Icons.error_outline,
            size: 18,
            color: color,
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              text,
              style: TextStyle(
                color: color,
                fontSize: 13,
                fontWeight: FontWeight.w600,
              ),
            ),
          ),
          if (isReconnecting)
            SizedBox(
              width: 16,
              height: 16,
              child: CircularProgressIndicator(
                strokeWidth: 2,
                color: color,
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildQuickStats(dynamic slowState, dynamic fastState) {
    final isDark = context.isDark;
    final battery = slowState?.resources?.batteryPercent ?? 0.0;
    final cpu = slowState?.resources?.cpuPercent ?? 0.0;
    final temp = slowState?.resources?.cpuTemp ?? 0.0;
    final mode = slowState?.currentMode ?? 'N/A';

    return Container(
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
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          _buildStatItem(Icons.battery_charging_full, '电量',
              '${battery.toStringAsFixed(0)}%', _batteryColor(battery)),
          _buildStatDivider(),
          _buildStatItem(Icons.memory, 'CPU',
              '${cpu.toStringAsFixed(0)}%', _cpuColor(cpu)),
          _buildStatDivider(),
          _buildStatItem(Icons.thermostat, '温度',
              '${temp.toStringAsFixed(0)}°', _tempColor(temp)),
          _buildStatDivider(),
          _buildStatItem(
              Icons.tune, '模式', _formatMode(mode), AppColors.primary),
        ],
      ),
    );
  }

  Widget _buildStatItem(
      IconData icon, String label, String value, Color color) {
    final isDark = context.isDark;
    return Column(
      children: [
        Icon(icon, size: 20, color: color),
        const SizedBox(height: 6),
        Text(
          value,
          style: TextStyle(
            fontSize: 15,
            fontWeight: FontWeight.w700,
            color: isDark ? Colors.white : Colors.black87,
          ),
        ),
        const SizedBox(height: 2),
        Text(
          label,
          style: TextStyle(fontSize: 11, color: context.subtitleColor),
        ),
      ],
    );
  }

  Widget _buildStatDivider() {
    return Container(
      width: 1,
      height: 36,
      color: context.dividerColor,
    );
  }

  Widget _buildMiniMetrics(dynamic slowState) {
    if (slowState == null) return const SizedBox.shrink();
    final cpu = slowState.resources?.cpuPercent ?? 0.0;
    return ClipRRect(
      borderRadius: BorderRadius.circular(4),
      child: SizedBox(
        width: double.infinity,
        height: 4,
        child: LinearProgressIndicator(
          value: cpu / 100.0,
          backgroundColor: AppColors.primary.withOpacity(0.1),
          valueColor: AlwaysStoppedAnimation(_cpuColor(cpu)),
        ),
      ),
    );
  }

  Widget _buildBadge(String text, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 7, vertical: 3),
      decoration: BoxDecoration(
        color: color.withOpacity(0.15),
        borderRadius: BorderRadius.circular(6),
      ),
      child: Text(
        text,
        style: TextStyle(
          fontSize: 10,
          fontWeight: FontWeight.w700,
          color: color,
        ),
      ),
    );
  }

  String _buildStatusSubtitle(dynamic slowState) {
    if (slowState == null) return '等待数据...';
    final cpu = slowState.resources?.cpuPercent ?? 0.0;
    final mem = slowState.resources?.memPercent ?? 0.0;
    return 'CPU ${cpu.toStringAsFixed(0)}% · MEM ${mem.toStringAsFixed(0)}%';
  }

  String _poseText(dynamic fastState) {
    if (fastState == null) return '等待位姿数据...';
    final x = fastState.pose?.position?.x ?? 0.0;
    final y = fastState.pose?.position?.y ?? 0.0;
    return 'X: ${x.toStringAsFixed(1)} Y: ${y.toStringAsFixed(1)}';
  }

  String _formatMode(String mode) {
    if (mode.contains('IDLE')) return 'IDLE';
    if (mode.contains('MANUAL')) return 'MAN';
    if (mode.contains('AUTO')) return 'AUTO';
    if (mode.contains('ESTOP')) return 'STOP';
    return mode.length > 4 ? mode.substring(0, 4) : mode;
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

  Color _tempColor(double temp) {
    if (temp < 50) return AppColors.success;
    if (temp < 70) return AppColors.warning;
    return AppColors.error;
  }
}
