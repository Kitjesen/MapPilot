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
import 'package:flutter_monitor/features/files/file_browser_screen.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:robot_proto/robot_proto.dart';

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
          final taskGw = context.watch<TaskGateway>();
          final mapGw = context.watch<MapGateway>();

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
                      color: AppColors.error.withValues(alpha:0.7),
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
                          if (provider.connectionRttMs != null) ...[
                            const SizedBox(width: 8),
                            Text(
                              '${provider.connectionRttMs!.toStringAsFixed(0)}ms',
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.w500,
                                color: provider.connectionRttMs! < 50
                                    ? AppColors.success
                                    : provider.connectionRttMs! < 150
                                        ? AppColors.warning
                                        : AppColors.error,
                              ),
                            ),
                          ],
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
                      fontSize: 13,
                      fontWeight: FontWeight.w600,
                      color: context.subtitleColor,
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
                  delegate: SliverChildListDelegate(_buildFeatureCards(
                    provider: provider,
                    fastState: fastState,
                    slowState: slowState,
                    mapGw: mapGw,
                    taskGw: taskGw,
                  )),
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

  List<Widget> _buildFeatureCards({
    required RobotConnectionProvider provider,
    required FastState? fastState,
    required SlowState? slowState,
    required MapGateway mapGw,
    required TaskGateway taskGw,
  }) {
    final caps = provider.capabilities;
    // Soft-gate: if capabilities are known, check support; if unknown, show all.
    final teleopOk = caps?.teleopSupported ?? true;
    final mappingOk = caps?.mappingSupported ?? true;
    final cameraOk = caps == null ||
        caps.supportedResources.any((r) => r.toLowerCase().contains('camera'));

    return [
      FeatureCard(
        icon: Icons.dashboard_outlined,
        title: '状态监控',
        subtitle: _buildStatusSubtitle(slowState),
        onTap: () => _pushScreen(const StatusScreen()),
        trailing: _buildMiniMetrics(slowState),
      ),
      _gatedCard(
        enabled: teleopOk,
        child: FeatureCard(
          icon: Icons.gamepad_outlined,
          title: '遥控操作',
          subtitle: !teleopOk
              ? '不支持'
              : provider.hasLease
                  ? '控制中'
                  : '点击获取控制权',
          onTap: teleopOk ? () => _pushScreen(const ControlScreen()) : null,
          badge: provider.hasLease
              ? _buildBadge('LIVE', AppColors.success)
              : null,
        ),
      ),
      _gatedCard(
        enabled: mappingOk,
        child: FeatureCard(
          icon: Icons.map_outlined,
          title: '地图导航',
          subtitle: !mappingOk
              ? '不支持'
              : mapGw.maps.isNotEmpty
                  ? '${mapGw.maps.length} 张地图'
                  : _poseText(fastState),
          onTap: mappingOk ? () => _pushScreen(const MapScreen()) : null,
          badge: taskGw.isRunning
              ? _buildBadge('任务中', AppColors.warning)
              : null,
        ),
      ),
      FeatureCard(
        icon: Icons.notifications_outlined,
        title: '事件日志',
        subtitle: '查看系统事件',
        onTap: () => _pushScreen(const EventsScreen()),
      ),
      FeatureCard(
        icon: Icons.folder_outlined,
        title: '文件管理',
        subtitle: '模型/地图/固件',
        onTap: () => _pushScreen(const FileBrowserScreen()),
      ),
      _gatedCard(
        enabled: cameraOk,
        child: FeatureCard(
          icon: Icons.videocam_outlined,
          title: '相机画面',
          subtitle: cameraOk ? '实时视频流' : '不支持',
          onTap: cameraOk ? () => Navigator.pushNamed(context, '/camera') : null,
        ),
      ),
    ];
  }

  Widget _gatedCard({required bool enabled, required Widget child}) {
    if (enabled) return child;
    return Opacity(opacity: 0.45, child: child);
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
        color: color.withValues(alpha:0.12),
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
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 14),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Row(
        children: [
          Expanded(
            child: _buildRingStatItem(
              '电量',
              '${battery.toStringAsFixed(0)}%',
              battery / 100.0,
              _batteryColor(battery),
              Icons.bolt,
              isDark,
            ),
          ),
          Expanded(
            child: _buildRingStatItem(
              'CPU',
              '${cpu.toStringAsFixed(0)}%',
              cpu / 100.0,
              _cpuColor(cpu),
              Icons.memory,
              isDark,
            ),
          ),
          Expanded(
            child: _buildRingStatItem(
              '温度',
              '${temp.toStringAsFixed(0)}°C',
              (temp / 100.0).clamp(0.0, 1.0),
              _tempColor(temp),
              Icons.thermostat,
              isDark,
            ),
          ),
          Expanded(
            child: _buildModeStatItem(
              _formatMode(mode),
              isDark,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRingStatItem(String label, String value, double ratio,
      Color color, IconData icon, bool isDark) {
    return Column(
      children: [
        SizedBox(
          width: 46,
          height: 46,
          child: Stack(
            alignment: Alignment.center,
            children: [
              SizedBox(
                width: 46,
                height: 46,
                child: CircularProgressIndicator(
                  value: ratio.clamp(0.0, 1.0),
                  strokeWidth: 3,
                  color: color,
                  backgroundColor: isDark
                      ? Colors.white.withValues(alpha:0.06)
                      : Colors.black.withValues(alpha:0.04),
                ),
              ),
              Icon(icon, size: 16, color: color),
            ],
          ),
        ),
        const SizedBox(height: 6),
        Text(
          value,
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w700,
            color: isDark ? Colors.white : Colors.black87,
          ),
        ),
        const SizedBox(height: 1),
        Text(
          label,
          style: TextStyle(
            fontSize: 11,
            color: context.subtitleColor,
          ),
        ),
      ],
    );
  }

  Widget _buildModeStatItem(String mode, bool isDark) {
    return Column(
      children: [
        Container(
          width: 46,
          height: 46,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: isDark
                ? Colors.white.withValues(alpha:0.06)
                : Colors.black.withValues(alpha:0.04),
          ),
          child: Icon(Icons.tune, size: 18, color: context.subtitleColor),
        ),
        const SizedBox(height: 6),
        Text(
          mode,
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w700,
            color: isDark ? Colors.white : Colors.black87,
          ),
        ),
        const SizedBox(height: 1),
        Text(
          '模式',
          style: TextStyle(fontSize: 11, color: context.subtitleColor),
        ),
      ],
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
          backgroundColor: context.isDark ? Colors.white.withValues(alpha:0.06) : Colors.black.withValues(alpha:0.05),
          valueColor: AlwaysStoppedAnimation(_cpuColor(cpu)),
        ),
      ),
    );
  }

  Widget _buildBadge(String text, Color color) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 7, vertical: 3),
      decoration: BoxDecoration(
        color: color.withValues(alpha:0.15),
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
