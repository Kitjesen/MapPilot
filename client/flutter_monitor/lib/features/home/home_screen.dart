import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';
import 'package:flutter_monitor/shared/widgets/robot_card.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:robot_proto/robot_proto.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen>
    with SingleTickerProviderStateMixin {
  late AnimationController _staggerController;

  // ── Events mini-feed ──
  final List<Event> _recentEvents = [];
  StreamSubscription? _eventSub;
  static const int _maxRecentEvents = 5;

  @override
  void initState() {
    super.initState();
    _staggerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 800),
    )..forward();

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startEventStream();
    });
  }

  @override
  void dispose() {
    _staggerController.dispose();
    _eventSub?.cancel();
    super.dispose();
  }

  void _startEventStream() {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;

    _eventSub?.cancel();
    try {
      _eventSub = client.streamEvents().listen(
        (event) {
          if (!mounted) return;
          setState(() {
            if (!_recentEvents.any((e) => e.eventId == event.eventId)) {
              _recentEvents.insert(0, event);
              if (_recentEvents.length > _maxRecentEvents) {
                _recentEvents.removeRange(_maxRecentEvents, _recentEvents.length);
              }
            }
          });
        },
        onError: (_) {},
      );
    } catch (_) {}
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // Re-start event stream when connection state changes
    final provider = context.watch<RobotConnectionProvider>();
    if (provider.isConnected && _eventSub == null) {
      _startEventStream();
    } else if (!provider.isConnected && _eventSub != null) {
      _eventSub?.cancel();
      _eventSub = null;
      _recentEvents.clear();
    }
  }

  @override
  Widget build(BuildContext context) {
    final provider = context.watch<RobotConnectionProvider>();
    final profileProvider = context.watch<RobotProfileProvider>();
    final profile = profileProvider.current;
    final isConnected = provider.isConnected;
    final pad = context.screenPadding;

    return Scaffold(
      body: Container(
        decoration: BoxDecoration(gradient: context.bgGradient),
        child: SafeArea(
          child: Center(
            child: ConstrainedBox(
              constraints: BoxConstraints(maxWidth: context.contentMaxWidth),
              child: CustomScrollView(
                physics: const BouncingScrollPhysics(),
                slivers: [
                  // ========= Top Bar =========
                  SliverToBoxAdapter(
                    child: _buildAnimatedChild(
                      0,
                      _buildTopBar(context, profile, isConnected, provider, pad),
                    ),
                  ),

                  // ========= Robot Card (connected) =========
                  if (isConnected)
                    SliverToBoxAdapter(
                      child: _buildAnimatedChild(
                        1,
                        Padding(
                          padding: EdgeInsets.fromLTRB(pad, 8, pad, 0),
                          child: _buildConnectedSummary(context, provider, profile),
                        ),
                      ),
                    ),

                  // ========= Connect prompt (disconnected) =========
                  if (!isConnected)
                    SliverToBoxAdapter(
                      child: _buildAnimatedChild(
                        1,
                        Padding(
                          padding: EdgeInsets.fromLTRB(pad, 8, pad, 0),
                          child: _buildConnectPrompt(context),
                        ),
                      ),
                    ),

                  // ========= Quick Actions Section =========
                  SliverToBoxAdapter(
                    child: _buildAnimatedChild(
                      2,
                      Padding(
                        padding: EdgeInsets.fromLTRB(pad, 28, pad, 12),
                        child: Text(
                          '快速操作',
                          style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w600,
                            color: context.subtitleColor,
                            letterSpacing: 0.5,
                          ),
                        ),
                      ),
                    ),
                  ),

                  SliverToBoxAdapter(
                    child: _buildAnimatedChild(
                      3,
                      Padding(
                        padding: EdgeInsets.symmetric(horizontal: pad),
                        child: _buildQuickActions(context, isConnected),
                      ),
                    ),
                  ),

                  // ========= Recent Events (connected) =========
                  if (isConnected) ...[
                    SliverToBoxAdapter(
                      child: _buildAnimatedChild(
                        4,
                        Padding(
                          padding: EdgeInsets.fromLTRB(pad, 28, pad, 12),
                          child: Row(
                            children: [
                              Text(
                                '最近事件',
                                style: TextStyle(
                                  fontSize: 14,
                                  fontWeight: FontWeight.w600,
                                  color: context.subtitleColor,
                                  letterSpacing: 0.5,
                                ),
                              ),
                              const Spacer(),
                              if (_recentEvents.isNotEmpty)
                                GestureDetector(
                                  onTap: () => Navigator.of(context).pushNamed('/events'),
                                  child: Text(
                                    '查看全部',
                                    style: TextStyle(
                                      fontSize: 13,
                                      fontWeight: FontWeight.w600,
                                      color: AppColors.primary,
                                    ),
                                  ),
                                ),
                            ],
                          ),
                        ),
                      ),
                    ),
                    SliverToBoxAdapter(
                      child: _buildAnimatedChild(
                        5,
                        Padding(
                          padding: EdgeInsets.symmetric(horizontal: pad),
                          child: _buildEventsFeed(context),
                        ),
                      ),
                    ),
                  ],

                  // ========= Connected Modules (connected) =========
                  if (isConnected) ...[
                    SliverToBoxAdapter(
                      child: _buildAnimatedChild(
                        6,
                        Padding(
                          padding: EdgeInsets.fromLTRB(pad, 28, pad, 12),
                          child: Text(
                            '连接模块',
                            style: TextStyle(
                              fontSize: 14,
                              fontWeight: FontWeight.w600,
                              color: context.subtitleColor,
                              letterSpacing: 0.5,
                            ),
                          ),
                        ),
                      ),
                    ),
                    SliverToBoxAdapter(
                      child: _buildAnimatedChild(
                        7,
                        Padding(
                          padding: EdgeInsets.symmetric(horizontal: pad),
                          child: _buildModuleList(context, provider),
                        ),
                      ),
                    ),
                  ],

                  // Bottom spacing
                  const SliverToBoxAdapter(child: SizedBox(height: 100)),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }

  // ─────────────────────────────────────────────────────────────
  //  Top Bar: Robot name + Online badge + search + notification
  // ─────────────────────────────────────────────────────────────
  Widget _buildTopBar(BuildContext context, dynamic profile, bool isConnected,
      RobotConnectionProvider provider, double pad) {
    return Padding(
      padding: EdgeInsets.fromLTRB(pad, 20, pad, 12),
      child: Row(
        children: [
          // Robot avatar
          Container(
            width: 44,
            height: 44,
            decoration: BoxDecoration(
              gradient: AppColors.brandGradient,
              borderRadius: BorderRadius.circular(14),
              boxShadow: [AppShadows.glow(AppColors.primary, blur: 12)],
            ),
            child: const Icon(Icons.smart_toy, color: Colors.white, size: 22),
          ),
          const SizedBox(width: 14),
          // Title + model
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    Text(
                      'Robot Control Unit',
                      style: TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.w800,
                        color: context.titleColor,
                        letterSpacing: -0.3,
                      ),
                    ),
                    const SizedBox(width: 8),
                    if (isConnected)
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 10, vertical: 3),
                        decoration: BoxDecoration(
                          color: AppColors.successLight,
                          borderRadius: BorderRadius.circular(AppRadius.pill),
                        ),
                        child: const Text(
                          'Online',
                          style: TextStyle(
                            fontSize: 11,
                            fontWeight: FontWeight.w700,
                            color: AppColors.success,
                          ),
                        ),
                      ),
                  ],
                ),
                const SizedBox(height: 2),
                GestureDetector(
                  onTap: () => Navigator.of(context).pushNamed('/robot-select'),
                  child: Row(
                    children: [
                      Text(
                        '${profile.name} Model',
                        style: TextStyle(
                          fontSize: 13,
                          color: context.subtitleColor,
                        ),
                      ),
                      const SizedBox(width: 4),
                      Icon(
                        Icons.keyboard_arrow_down_rounded,
                        size: 16,
                        color: context.subtitleColor,
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
          // Search / Scan
          _TopBarIcon(
            icon: Icons.wifi_find_rounded,
            onTap: () => Navigator.of(context).pushNamed('/scan'),
          ),
          const SizedBox(width: 8),
          // Camera shortcut (visible when connected)
          if (isConnected)
            _TopBarIcon(
              icon: Icons.videocam_outlined,
              onTap: () => Navigator.of(context).pushNamed('/camera'),
            ),
        ],
      ),
    );
  }

  // ─────────────────────────────────────────────────────────────
  //  Connected Summary — quick stats card (battery / cpu / temp / OTA)
  // ─────────────────────────────────────────────────────────────
  Widget _buildConnectedSummary(
      BuildContext context, RobotConnectionProvider provider, dynamic profile) {
    final slow = provider.latestSlowState;
    final battery = slow?.resources.batteryPercent ?? 0.0;
    final cpu = slow?.resources.cpuPercent ?? 0.0;
    final temp = slow?.resources.cpuTemp ?? 0.0;
    final isDark = context.isDark;

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: context.elevatedCardDecoration,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Robot name + status
          Row(
            children: [
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      profile.name,
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w700,
                        color: context.titleColor,
                      ),
                    ),
                    const SizedBox(height: 2),
                    Row(
                      children: [
                        Container(
                          width: 7,
                          height: 7,
                          decoration: const BoxDecoration(
                            color: AppColors.success,
                            shape: BoxShape.circle,
                          ),
                        ),
                        const SizedBox(width: 5),
                        Text(
                          '已连接',
                          style: TextStyle(
                            fontSize: 12,
                            fontWeight: FontWeight.w500,
                            color: AppColors.success,
                          ),
                        ),
                        if (provider.connectionRttMs != null) ...[
                          const SizedBox(width: 8),
                          Text(
                            '${provider.connectionRttMs!.toStringAsFixed(0)}ms',
                            style: TextStyle(
                              fontSize: 11,
                              color: context.subtitleColor,
                            ),
                          ),
                        ],
                        const SizedBox(width: 12),
                        if (provider.otaAvailable)
                          Container(
                            padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                            decoration: BoxDecoration(
                              color: AppColors.info.withValues(alpha: 0.1),
                              borderRadius: BorderRadius.circular(4),
                            ),
                            child: const Text(
                              'OTA',
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.w600,
                                color: AppColors.info,
                              ),
                            ),
                          ),
                      ],
                    ),
                  ],
                ),
              ),
              // Disconnect button
              GestureDetector(
                onTap: () => _showDisconnectDialog(context),
                child: Container(
                  padding: const EdgeInsets.all(8),
                  decoration: BoxDecoration(
                    color: AppColors.error.withValues(alpha: 0.08),
                    borderRadius: BorderRadius.circular(10),
                  ),
                  child: const Icon(Icons.power_settings_new, size: 18, color: AppColors.error),
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          // Quick stats row
          Row(
            children: [
              _MiniStatChip(
                icon: Icons.bolt,
                label: '${battery.toStringAsFixed(0)}%',
                color: _batteryColor(battery),
                isDark: isDark,
              ),
              const SizedBox(width: 10),
              _MiniStatChip(
                icon: Icons.memory,
                label: 'CPU ${cpu.toStringAsFixed(0)}%',
                color: _cpuColor(cpu),
                isDark: isDark,
              ),
              const SizedBox(width: 10),
              _MiniStatChip(
                icon: Icons.thermostat,
                label: '${temp.toStringAsFixed(0)}°C',
                color: _tempColor(temp),
                isDark: isDark,
              ),
            ],
          ),
        ],
      ),
    );
  }

  // ─────────────────────────────────────────────────────────────
  //  Connect Prompt (when disconnected)
  // ─────────────────────────────────────────────────────────────
  Widget _buildConnectPrompt(BuildContext context) {
    return GestureDetector(
      onTap: () {
        HapticFeedback.lightImpact();
        Navigator.of(context).pushNamed('/scan');
      },
      child: Container(
        padding: const EdgeInsets.all(20),
        decoration: context.cardDecoration,
        child: Row(
          children: [
            Container(
              width: 48,
              height: 48,
              decoration: BoxDecoration(
                color: AppColors.primary.withValues(alpha: 0.08),
                borderRadius: BorderRadius.circular(14),
              ),
              child: const Icon(Icons.add_rounded, color: AppColors.primary, size: 24),
            ),
            const SizedBox(width: 16),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    '连接机器人',
                    style: TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.w600,
                      color: context.titleColor,
                    ),
                  ),
                  const SizedBox(height: 2),
                  Text(
                    '扫描网络或蓝牙发现附近设备',
                    style: TextStyle(
                      fontSize: 13,
                      color: context.subtitleColor,
                    ),
                  ),
                ],
              ),
            ),
            Container(
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: AppColors.primary.withValues(alpha: 0.08),
                borderRadius: BorderRadius.circular(10),
              ),
              child: const Icon(
                Icons.arrow_forward_rounded,
                color: AppColors.primary,
                size: 18,
              ),
            ),
          ],
        ),
      ),
    );
  }

  // ─────────────────────────────────────────────────────────────
  //  Quick Actions — context-aware grid
  // ─────────────────────────────────────────────────────────────
  Widget _buildQuickActions(BuildContext context, bool isConnected) {
    // Actions change depending on connection state
    final actions = isConnected
        ? [
            _ActionDef(Icons.monitor_heart_outlined, '实时监控', AppColors.primary, _goToTab(1)),
            _ActionDef(Icons.gamepad_outlined, '遥控操作', AppColors.secondary, '/control'),
            _ActionDef(Icons.navigation_outlined, '任务导航', AppColors.info, '/task-panel'),
            _ActionDef(Icons.map_outlined, '地图管理', AppColors.warning, '/map-manager'),
            _ActionDef(Icons.videocam_outlined, '摄像头', AppColors.error, '/camera'),
            _ActionDef(Icons.folder_outlined, '文件管理', AppColors.success, _goToTab(3)),
          ]
        : [
            _ActionDef(Icons.wifi_find_rounded, '扫描设备', AppColors.primary, '/scan'),
            _ActionDef(Icons.widgets_outlined, '选择型号', AppColors.secondary, '/robot-select'),
            _ActionDef(Icons.play_circle_outline_rounded, '演示模式', AppColors.success, null),
          ];

    return GridView.count(
      crossAxisCount: context.isMobile ? 3 : 4,
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      mainAxisSpacing: 12,
      crossAxisSpacing: 12,
      childAspectRatio: 1.1,
      children: actions.map((a) => _QuickActionCard(
        icon: a.icon,
        label: a.label,
        color: a.color,
        onTap: () {
          HapticFeedback.selectionClick();
          if (a.route == null) {
            _startMock(context);
          } else if (a.route!.startsWith('_tab:')) {
            // Switch to tab in MainShell
            final tabIndex = int.tryParse(a.route!.split(':').last) ?? 0;
            _switchToTab(tabIndex);
          } else {
            Navigator.of(context).pushNamed(a.route!);
          }
        },
      )).toList(),
    );
  }

  /// Special route prefix to indicate tab switching
  String _goToTab(int index) => '_tab:$index';

  void _switchToTab(int index) {
    // Find the MainShellScreen ancestor's state and switch tab
    // We use a simple approach: pop to main shell and set tab
    final mainShellState = context.findAncestorStateOfType<State>();
    // Use a shared notification approach
    MainShellTabNotification(index).dispatch(context);
  }

  // ─────────────────────────────────────────────────────────────
  //  Events mini-feed
  // ─────────────────────────────────────────────────────────────
  Widget _buildEventsFeed(BuildContext context) {
    if (_recentEvents.isEmpty) {
      return Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 20),
        decoration: context.cardDecoration,
        child: Row(
          children: [
            Icon(Icons.check_circle_outline, size: 18, color: context.subtitleColor),
            const SizedBox(width: 10),
            Text(
              '暂无最近事件',
              style: TextStyle(fontSize: 13, color: context.subtitleColor),
            ),
          ],
        ),
      );
    }

    return Container(
      decoration: context.cardDecoration,
      child: Column(
        children: [
          for (int i = 0; i < _recentEvents.length; i++) ...[
            _buildEventItem(context, _recentEvents[i]),
            if (i < _recentEvents.length - 1)
              Divider(
                height: 1,
                indent: 48,
                color: context.borderColor.withValues(alpha: 0.5),
              ),
          ],
        ],
      ),
    );
  }

  Widget _buildEventItem(BuildContext context, Event event) {
    final severity = event.severity;
    final color = severity == EventSeverity.EVENT_SEVERITY_ERROR
        ? AppColors.error
        : severity == EventSeverity.EVENT_SEVERITY_WARNING
            ? AppColors.warning
            : AppColors.info;
    final icon = severity == EventSeverity.EVENT_SEVERITY_ERROR
        ? Icons.error_outline
        : severity == EventSeverity.EVENT_SEVERITY_WARNING
            ? Icons.warning_amber_rounded
            : Icons.info_outline;

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Container(
            width: 28,
            height: 28,
            decoration: BoxDecoration(
              color: color.withValues(alpha: 0.1),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Icon(icon, size: 14, color: color),
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  event.title.isNotEmpty ? event.title : event.type.name,
                  style: TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w500,
                    color: context.titleColor,
                  ),
                  maxLines: 2,
                  overflow: TextOverflow.ellipsis,
                ),
                if (event.description.isNotEmpty)
                  Padding(
                    padding: const EdgeInsets.only(top: 2),
                    child: Text(
                      event.description,
                      style: TextStyle(fontSize: 11, color: context.subtitleColor),
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ─────────────────────────────────────────────────────────────
  //  Connected Modules list
  // ─────────────────────────────────────────────────────────────
  Widget _buildModuleList(
      BuildContext context, RobotConnectionProvider provider) {
    final slow = provider.latestSlowState;
    final modules = <_ModuleDef>[
      _ModuleDef(
        Icons.sensors_rounded,
        'LiDAR Array',
        'Velodyne Puck',
        'Active',
        AppColors.success,
        AppColors.primary.withValues(alpha: 0.1),
      ),
      _ModuleDef(
        Icons.memory_rounded,
        'Main Processor',
        'NVIDIA Jetson',
        slow != null ? '${slow.resources.cpuPercent.toStringAsFixed(0)}% load' : '--',
        AppColors.warning,
        AppColors.warning.withValues(alpha: 0.1),
      ),
      _ModuleDef(
        Icons.battery_charging_full_rounded,
        'Power Unit',
        'Lithium Pack',
        slow != null ? '${slow.resources.batteryPercent.toStringAsFixed(0)}%' : 'Charging',
        AppColors.success,
        AppColors.success.withValues(alpha: 0.1),
      ),
      _ModuleDef(
        Icons.videocam_rounded,
        'Front Camera',
        'Sony IMX',
        'Recording',
        AppColors.error,
        AppColors.error.withValues(alpha: 0.1),
      ),
    ];

    return Container(
      decoration: context.cardDecoration,
      child: Column(
        children: [
          for (int i = 0; i < modules.length; i++) ...[
            _buildModuleItem(context, modules[i]),
            if (i < modules.length - 1)
              Divider(
                height: 1,
                indent: 68,
                color: context.borderColor.withValues(alpha: 0.5),
              ),
          ],
        ],
      ),
    );
  }

  Widget _buildModuleItem(BuildContext context, _ModuleDef module) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
      child: Row(
        children: [
          Container(
            width: 40,
            height: 40,
            decoration: BoxDecoration(
              color: module.iconBg,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(module.icon, size: 20, color: AppColors.primary),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  module.name,
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w600,
                    color: context.titleColor,
                  ),
                ),
                Text(
                  module.subtitle,
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                  ),
                ),
              ],
            ),
          ),
          Text(
            module.status,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w600,
              color: module.statusColor,
            ),
          ),
        ],
      ),
    );
  }

  // ─────────────────────────────────────────────────────────────
  //  Disconnect dialog
  // ─────────────────────────────────────────────────────────────
  Future<void> _showDisconnectDialog(BuildContext context) async {
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
    }
  }

  // ─────────────────────────────────────────────────────────────
  //  Animation helper
  // ─────────────────────────────────────────────────────────────
  Widget _buildAnimatedChild(int index, Widget child) {
    final delay = index * 0.08;
    final animation = CurvedAnimation(
      parent: _staggerController,
      curve: Interval(delay.clamp(0.0, 0.7), (delay + 0.3).clamp(0.0, 1.0),
          curve: Curves.easeOutCubic),
    );
    return FadeTransition(
      opacity: animation,
      child: SlideTransition(
        position: Tween<Offset>(
          begin: const Offset(0, 0.06),
          end: Offset.zero,
        ).animate(animation),
        child: child,
      ),
    );
  }

  Future<void> _startMock(BuildContext context) async {
    HapticFeedback.lightImpact();
    final provider = context.read<RobotConnectionProvider>();
    final client = MockRobotClient();
    await provider.connect(client);
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

// ─────────────────────────────────────────────────────────────────
//  Notification for tab switching from child widgets
// ─────────────────────────────────────────────────────────────────

class MainShellTabNotification extends Notification {
  final int tabIndex;
  const MainShellTabNotification(this.tabIndex);
}

// ─────────────────────────────────────────────────────────────────
//  Top bar icon button
// ─────────────────────────────────────────────────────────────────

class _TopBarIcon extends StatelessWidget {
  final IconData icon;
  final VoidCallback onTap;

  const _TopBarIcon({required this.icon, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: context.cardColor,
          borderRadius: BorderRadius.circular(12),
          boxShadow: [AppShadows.light()],
        ),
        child: Icon(icon, size: 20, color: context.subtitleColor),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────
//  Quick action card
// ─────────────────────────────────────────────────────────────────

class _QuickActionCard extends StatefulWidget {
  final IconData icon;
  final String label;
  final Color color;
  final VoidCallback? onTap;

  const _QuickActionCard({
    required this.icon,
    required this.label,
    required this.color,
    this.onTap,
  });

  @override
  State<_QuickActionCard> createState() => _QuickActionCardState();
}

class _QuickActionCardState extends State<_QuickActionCard> {
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
      onTap: widget.onTap,
      child: AnimatedScale(
        scale: _scale,
        duration: AppDurations.fast,
        curve: Curves.easeOut,
        child: Container(
          decoration: context.cardDecoration,
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
                width: 42,
                height: 42,
                decoration: BoxDecoration(
                  color: widget.color.withValues(alpha: 0.1),
                  borderRadius: BorderRadius.circular(13),
                ),
                child: Icon(widget.icon, color: widget.color, size: 22),
              ),
              const SizedBox(height: 10),
              Text(
                widget.label,
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                  color: context.titleColor,
                ),
                textAlign: TextAlign.center,
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────
//  Mini stat chip for summary card
// ─────────────────────────────────────────────────────────────────

class _MiniStatChip extends StatelessWidget {
  final IconData icon;
  final String label;
  final Color color;
  final bool isDark;

  const _MiniStatChip({
    required this.icon,
    required this.label,
    required this.color,
    required this.isDark,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: color.withValues(alpha: isDark ? 0.12 : 0.08),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 14, color: color),
          const SizedBox(width: 4),
          Text(
            label,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w600,
              color: color,
            ),
          ),
        ],
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────
//  Data definitions
// ─────────────────────────────────────────────────────────────────

class _ActionDef {
  final IconData icon;
  final String label;
  final Color color;
  final String? route;
  const _ActionDef(this.icon, this.label, this.color, this.route);
}

class _ModuleDef {
  final IconData icon;
  final String name;
  final String subtitle;
  final String status;
  final Color statusColor;
  final Color iconBg;
  const _ModuleDef(
      this.icon, this.name, this.subtitle, this.status, this.statusColor, this.iconBg);
}
