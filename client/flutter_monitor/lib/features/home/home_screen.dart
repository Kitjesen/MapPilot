import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:robot_proto/robot_proto.dart';

// ═══════════════════════════════════════════════════════════════
//  HomeScreen — Glassmorphic Dashboard
// ═══════════════════════════════════════════════════════════════

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});
  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen>
    with SingleTickerProviderStateMixin {
  late AnimationController _stagger;
  final List<Event> _recentEvents = [];
  StreamSubscription? _eventSub;

  @override
  void initState() {
    super.initState();
    _stagger = AnimationController(
        vsync: this, duration: const Duration(milliseconds: 900))
      ..forward();
    WidgetsBinding.instance.addPostFrameCallback((_) => _startEvents());
  }

  @override
  void dispose() {
    _stagger.dispose();
    _eventSub?.cancel();
    super.dispose();
  }

  void _startEvents() {
    final c = context.read<RobotConnectionProvider>().client;
    if (c == null) return;
    _eventSub?.cancel();
    try {
      _eventSub = c.streamEvents().listen((e) {
        if (!mounted) return;
        setState(() {
          if (!_recentEvents.any((x) => x.eventId == e.eventId)) {
            _recentEvents.insert(0, e);
            if (_recentEvents.length > 5) _recentEvents.removeRange(5, _recentEvents.length);
          }
        });
      }, onError: (_) {});
    } catch (_) {}
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final p = context.watch<RobotConnectionProvider>();
    if (p.isConnected && _eventSub == null) {
      _startEvents();
    } else if (!p.isConnected && _eventSub != null) {
      _eventSub?.cancel();
      _eventSub = null;
      _recentEvents.clear();
    }
  }

  // ═══════════════════════════════════════════════════════════
  //  BUILD
  // ═══════════════════════════════════════════════════════════
  @override
  Widget build(BuildContext context) {
    final conn = context.watch<RobotConnectionProvider>();
    final profile = context.watch<RobotProfileProvider>().current;
    final online = conn.isConnected;
    final pad = context.screenPadding;
    final dark = context.isDark;
    final inShell = context.useSideNav;

    return Scaffold(
      backgroundColor: inShell ? Colors.transparent : null,
      body: Stack(
        children: [
          // ── Background gradient (only when not inside shell) ──
          if (!inShell)
            Positioned.fill(
              child: Container(decoration: BoxDecoration(gradient: context.bgGradient)),
            ),

          // ── Decorative background blobs (only when not inside shell) ──
          if (!inShell && !dark) ...[
            _posBlob(top: -100, left: -80, color: const Color(0x35C4B5FD), size: 400),
            _posBlob(bottom: -100, right: -80, color: const Color(0x3093C5FD), size: 400),
            _posBlob(top: 280, left: 350, color: const Color(0x25FBCFE8), size: 280),
          ] else if (!inShell && dark) ...[
            _posBlob(top: -100, left: -80, color: const Color(0x12C4B5FD), size: 400),
            _posBlob(bottom: -100, right: -80, color: const Color(0x1093C5FD), size: 400),
          ],

          // ── Scrollable content ──
          SafeArea(
            child: Center(
              child: ConstrainedBox(
                constraints: BoxConstraints(maxWidth: context.contentMaxWidth),
                child: CustomScrollView(
                  physics: const BouncingScrollPhysics(),
                  slivers: [
                    // ── Header ──
                    SliverToBoxAdapter(
                      child: _anim(0, _buildHeader(context, profile, online, conn, pad)),
                    ),

                    // ── Connect prompt ──
                    if (!online)
                      SliverToBoxAdapter(
                        child: _anim(1, Padding(
                          padding: EdgeInsets.fromLTRB(pad, 0, pad, 20),
                          child: _buildConnectPrompt(context),
                        )),
                      ),

                    // ── Feature cards ──
                    SliverToBoxAdapter(
                      child: _anim(online ? 1 : 2, Padding(
                        padding: EdgeInsets.symmetric(horizontal: pad),
                        child: _buildCardGrid(context, online, conn),
                      )),
                    ),

                    const SliverToBoxAdapter(child: SizedBox(height: 100)),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  HEADER — Robot identity LEFT + KPI pills RIGHT
  // ═══════════════════════════════════════════════════════════
  Widget _buildHeader(BuildContext context, dynamic profile, bool online,
      RobotConnectionProvider conn, double pad) {
    final slow = conn.latestSlowState;
    final bat = slow?.resources.batteryPercent ?? 0.0;
    final cpu = slow?.resources.cpuPercent ?? 0.0;
    final temp = slow?.resources.cpuTemp ?? 0.0;

    return Padding(
      padding: EdgeInsets.fromLTRB(pad, 20, pad, 20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Title row
          Wrap(
            crossAxisAlignment: WrapCrossAlignment.center,
            spacing: 10,
            runSpacing: 8,
            children: [
              Text.rich(TextSpan(children: [
                TextSpan(
                  text: 'Robot ',
                  style: TextStyle(
                    fontSize: 26, fontWeight: FontWeight.w700,
                    color: context.titleColor, letterSpacing: -0.5,
                  ),
                ),
                TextSpan(
                  text: 'ID #X-402',
                  style: TextStyle(
                    fontSize: 26, fontWeight: FontWeight.w700,
                    color: AppColors.primary, letterSpacing: -0.5,
                  ),
                ),
              ])),
              if (online) _onlineBadge(),
            ],
          ),
          const SizedBox(height: 4),
          GestureDetector(
            onTap: () => Navigator.of(context).pushNamed('/robot-select'),
            child: Text(
              'Model: ${profile.name} • Firmware v4.5.1',
              style: TextStyle(fontSize: 13, fontWeight: FontWeight.w500,
                  color: context.subtitleColor),
            ),
          ),

          // KPI row (connected, non-mobile)
          if (online && !context.isMobile) ...[
            const SizedBox(height: 20),
            Wrap(
              spacing: 16,
              runSpacing: 12,
              children: [
                _KpiPill(
                  icon: Icons.battery_charging_full_rounded,
                  iconBg: const Color(0xFFF3E8FF), iconColor: AppColors.primary,
                  label: 'BATTERY LEVEL',
                  value: '${bat.toStringAsFixed(0)}%',
                  progress: bat / 100, progressColor: AppColors.primary,
                ),
                _KpiPill(
                  icon: Icons.memory_rounded,
                  iconBg: const Color(0xFFE3F2FD), iconColor: AppColors.info,
                  label: 'CPU LOAD',
                  value: '${cpu.toStringAsFixed(0)}%',
                ),
                _KpiPill(
                  icon: Icons.thermostat_rounded,
                  iconBg: const Color(0xFFFFF3E0), iconColor: const Color(0xFFEA580C),
                  label: 'TEMP',
                  value: '${temp.toStringAsFixed(0)}°C',
                ),
              ],
            ),
          ],
        ],
      ),
    );
  }

  Widget _onlineBadge() => Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
        decoration: BoxDecoration(
          color: const Color(0x334ADE80),
          borderRadius: BorderRadius.circular(AppRadius.pill),
          border: Border.all(color: const Color(0x504ADE80)),
        ),
        child: const Text('ONLINE', style: TextStyle(
          fontSize: 11, fontWeight: FontWeight.w700,
          color: Color(0xFF15803D), letterSpacing: 0.5,
        )),
      );

  // ═══════════════════════════════════════════════════════════
  //  CONNECT PROMPT
  // ═══════════════════════════════════════════════════════════
  Widget _buildConnectPrompt(BuildContext context) {
    return GestureDetector(
      onTap: () {
        HapticFeedback.lightImpact();
        Navigator.of(context).pushNamed('/scan');
      },
      child: _GlassContainer(
        radius: 24,
        padding: const EdgeInsets.all(18),
        child: Row(children: [
          Container(
            width: 48, height: 48,
            decoration: BoxDecoration(
              color: AppColors.primary.withValues(alpha: 0.08),
              borderRadius: BorderRadius.circular(14),
            ),
            child: const Icon(Icons.add_rounded, color: AppColors.primary, size: 24),
          ),
          const SizedBox(width: 14),
          Expanded(child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisSize: MainAxisSize.min,
            children: [
              Text('连接机器人', style: TextStyle(
                fontSize: 15, fontWeight: FontWeight.w600, color: context.titleColor,
              )),
              const SizedBox(height: 2),
              Text('扫描网络或蓝牙发现附近设备', style: TextStyle(
                fontSize: 12, color: context.subtitleColor,
              )),
            ],
          )),
          Icon(Icons.arrow_forward_ios_rounded, size: 16, color: context.hintColor),
        ]),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  CARD GRID — 3×2 with fixed 280px height
  // ═══════════════════════════════════════════════════════════
  Widget _buildCardGrid(BuildContext context, bool online, RobotConnectionProvider conn) {
    final cols = context.isMobile ? 2 : 3;
    const gap = 20.0;
    const h = 280.0;

    final cards = [
      _systemStatus(context, online),
      _remoteControl(context, online),
      _liveLocation(context, online),
      _recentEventsCard(context),
      _dataLogs(context),
      _cameraFeed(context, online),
    ];

    // Build rows manually for exact height control
    final rows = <Widget>[];
    for (var i = 0; i < cards.length; i += cols) {
      if (i > 0) rows.add(const SizedBox(height: gap));
      final items = <Widget>[];
      for (var j = 0; j < cols && i + j < cards.length; j++) {
        if (j > 0) items.add(const SizedBox(width: gap));
        items.add(Expanded(child: cards[i + j]));
      }
      rows.add(SizedBox(
        height: h,
        child: Row(crossAxisAlignment: CrossAxisAlignment.stretch, children: items),
      ));
    }
    return Column(children: rows);
  }

  // ───────────── System Status ─────────────
  Widget _systemStatus(BuildContext context, bool online) {
    return _GlassFeatureCard(
      glowColor: const Color(0xFF4ADE80),
      icon: Icons.check_circle_rounded,
      iconColor: AppColors.success,
      topRight: Icon(Icons.open_in_new_rounded, size: 18, color: context.hintColor),
      title: 'System Status',
      onTap: () {
        HapticFeedback.selectionClick();
        MainShellTabNotification(1).dispatch(context);
      },
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            online
                ? 'All subsystems are functioning within normal parameters. Last diagnostic run 2m ago.'
                : '未连接 — 无法获取状态',
            style: TextStyle(fontSize: 13, color: context.subtitleColor, height: 1.5),
            maxLines: 2, overflow: TextOverflow.ellipsis,
          ),
          const Spacer(),
          // Status dot
          Row(children: [
            Container(width: 8, height: 8, decoration: BoxDecoration(
              color: online ? AppColors.success : context.hintColor,
              shape: BoxShape.circle,
            )),
            const SizedBox(width: 6),
            Text(
              online ? 'Operational' : 'Offline',
              style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600,
                  color: online ? const Color(0xFF15803D) : context.hintColor),
            ),
          ]),
          const SizedBox(height: 8),
          // Glass bar
          Container(
            height: 40,
            padding: const EdgeInsets.symmetric(horizontal: 12),
            decoration: _glassSurface(context, radius: 14),
            alignment: Alignment.centerLeft,
            child: Text(
              online ? 'Next maintenance in 48h' : '—',
              style: TextStyle(fontSize: 12, color: context.hintColor),
            ),
          ),
        ],
      ),
    );
  }

  // ───────────── Remote Control ─────────────
  Widget _remoteControl(BuildContext context, bool online) {
    return _GlassFeatureCard(
      glowColor: const Color(0xFF818CF8),
      icon: Icons.sports_esports_rounded,
      iconColor: AppColors.primary,
      topRight: online
          ? Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
              decoration: BoxDecoration(
                color: const Color(0xFFE0E7FF),
                borderRadius: BorderRadius.circular(4),
              ),
              child: const Text('MANUAL', style: TextStyle(
                fontSize: 10, fontWeight: FontWeight.w700,
                color: Color(0xFF4338CA), letterSpacing: 0.3,
              )),
            )
          : null,
      title: 'Remote Control',
      onTap: null,
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Take manual control of the unit for precision navigation.',
            style: TextStyle(fontSize: 13, color: context.subtitleColor, height: 1.5),
          ),
          const Spacer(),
          Row(children: [
            Expanded(
              child: GestureDetector(
                onTap: online
                    ? () { HapticFeedback.selectionClick(); Navigator.of(context).pushNamed('/control'); }
                    : null,
                child: Container(
                  height: 42,
                  decoration: BoxDecoration(
                    color: online ? AppColors.primary : context.hintColor.withValues(alpha: 0.15),
                    borderRadius: BorderRadius.circular(14),
                    boxShadow: online
                        ? [BoxShadow(color: AppColors.primary.withValues(alpha: 0.3), blurRadius: 12, offset: const Offset(0, 4))]
                        : null,
                  ),
                  alignment: Alignment.center,
                  child: Text('Engage', style: TextStyle(
                    fontSize: 14, fontWeight: FontWeight.w600,
                    color: online ? Colors.white : context.hintColor,
                  )),
                ),
              ),
            ),
            const SizedBox(width: 8),
            GestureDetector(
              onTap: () { HapticFeedback.selectionClick(); MainShellTabNotification(4).dispatch(context); },
              child: Container(
                width: 42, height: 42,
                decoration: BoxDecoration(
                  color: context.isDark
                      ? Colors.white.withValues(alpha: 0.06)
                      : Colors.white.withValues(alpha: 0.5),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.white.withValues(alpha: context.isDark ? 0.1 : 0.4)),
                ),
                child: Icon(Icons.settings_rounded, size: 18, color: context.subtitleColor),
              ),
            ),
          ]),
        ],
      ),
    );
  }

  // ───────────── Live Location ─────────────
  Widget _liveLocation(BuildContext context, bool online) {
    return _GlassFeatureCard(
      glowColor: const Color(0xFF60A5FA),
      icon: Icons.near_me_rounded,
      iconColor: AppColors.info,
      topRight: Container(
        padding: const EdgeInsets.all(4),
        decoration: BoxDecoration(
          color: Colors.white.withValues(alpha: 0.6),
          shape: BoxShape.circle,
        ),
        child: Icon(Icons.fullscreen_rounded, size: 16, color: context.subtitleColor),
      ),
      title: 'Live Location',
      onTap: () { HapticFeedback.selectionClick(); MainShellTabNotification(2).dispatch(context); },
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Sector 7 • Warehouse B', style: TextStyle(
            fontSize: 13, color: context.subtitleColor, height: 1.5,
          )),
          const Spacer(),
          // Speed badge
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 7),
            decoration: BoxDecoration(
              color: context.isDark
                  ? Colors.white.withValues(alpha: 0.08)
                  : Colors.white.withValues(alpha: 0.8),
              borderRadius: BorderRadius.circular(10),
              border: Border.all(color: Colors.white.withValues(alpha: context.isDark ? 0.1 : 0.5)),
              boxShadow: [BoxShadow(color: Colors.black.withValues(alpha: 0.03), blurRadius: 4)],
            ),
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Icon(Icons.speed_rounded, size: 14, color: AppColors.primary),
              const SizedBox(width: 6),
              Text(
                online ? 'Moving at 1.2 m/s' : 'No data',
                style: TextStyle(fontSize: 12, color: context.subtitleColor),
              ),
            ]),
          ),
        ],
      ),
    );
  }

  // ───────────── Recent Events ─────────────
  Widget _recentEventsCard(BuildContext context) {
    return _GlassFeatureCard(
      glowColor: const Color(0xFFFB923C),
      icon: Icons.notifications_active_rounded,
      iconColor: const Color(0xFFF97316),
      topRight: _recentEvents.isNotEmpty
          ? Container(
              width: 24, height: 24,
              decoration: const BoxDecoration(color: Color(0xFFEF4444), shape: BoxShape.circle),
              alignment: Alignment.center,
              child: Text('${_recentEvents.length}', style: const TextStyle(
                fontSize: 11, fontWeight: FontWeight.w700, color: Colors.white,
              )),
            )
          : null,
      title: 'Recent Events',
      onTap: () { HapticFeedback.selectionClick(); Navigator.of(context).pushNamed('/events'); },
      body: _recentEvents.isEmpty
          ? Center(child: Text('暂无最近事件', style: TextStyle(fontSize: 13, color: context.hintColor)))
          : Column(
              mainAxisAlignment: MainAxisAlignment.end,
              children: [
                for (int i = 0; i < _recentEvents.length.clamp(0, 2); i++) ...[
                  if (i > 0) const SizedBox(height: 10),
                  _eventRow(context, _recentEvents[i]),
                ],
              ],
            ),
    );
  }

  Widget _eventRow(BuildContext context, Event e) {
    final color = e.severity == EventSeverity.EVENT_SEVERITY_ERROR
        ? AppColors.error
        : e.severity == EventSeverity.EVENT_SEVERITY_WARNING
            ? AppColors.warning
            : AppColors.info;
    final eventTime = e.hasTimestamp()
        ? DateTime.fromMillisecondsSinceEpoch(e.timestamp.seconds.toInt() * 1000)
        : DateTime.now();
    final ago = DateTime.now().difference(eventTime).inMinutes;
    final agoStr = ago < 60 ? '${ago}m' : '${(ago / 60).floor()}h';

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: _glassSurface(context, radius: 10),
      child: Row(children: [
        Container(width: 6, height: 6, decoration: BoxDecoration(color: color, shape: BoxShape.circle)),
        const SizedBox(width: 10),
        Expanded(child: Text(
          e.title.isNotEmpty ? e.title : e.type.name,
          style: TextStyle(fontSize: 13, color: context.titleColor, fontWeight: FontWeight.w500),
          maxLines: 1, overflow: TextOverflow.ellipsis,
        )),
        const SizedBox(width: 8),
        Text(agoStr, style: TextStyle(fontSize: 11, color: context.hintColor)),
      ]),
    );
  }

  // ───────────── Data Logs ─────────────
  Widget _dataLogs(BuildContext context) {
    return _GlassFeatureCard(
      glowColor: const Color(0xFFA78BFA),
      icon: Icons.folder_open_rounded,
      iconColor: const Color(0xFF8B5CF6),
      topRight: Icon(Icons.download_rounded, size: 18, color: context.hintColor),
      title: 'Data Logs',
      onTap: () { HapticFeedback.selectionClick(); MainShellTabNotification(3).dispatch(context); },
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'Access sensor logs, telemetry data, and visual records.',
            style: TextStyle(fontSize: 13, color: context.subtitleColor, height: 1.5),
          ),
          const Spacer(),
          Row(children: [
            Expanded(child: _dataBox(context, 'LIDAR', '1.2 GB')),
            const SizedBox(width: 12),
            Expanded(child: _dataBox(context, 'Visual', '4.5 GB')),
          ]),
        ],
      ),
    );
  }

  Widget _dataBox(BuildContext context, String label, String value) {
    return Container(
      padding: const EdgeInsets.symmetric(vertical: 10),
      decoration: _glassSurface(context, radius: 14),
      child: Column(children: [
        Text(label, style: TextStyle(
          fontSize: 12, fontWeight: FontWeight.w700, color: context.subtitleColor,
        )),
        const SizedBox(height: 2),
        Text(value, style: TextStyle(
          fontSize: 10, color: context.hintColor,
        )),
      ]),
    );
  }

  // ───────────── Camera Feed ─────────────
  Widget _cameraFeed(BuildContext context, bool online) {
    return GestureDetector(
      onTap: () { HapticFeedback.selectionClick(); Navigator.of(context).pushNamed('/camera'); },
      child: ClipRRect(
        borderRadius: BorderRadius.circular(32),
        child: Container(
          decoration: BoxDecoration(
            color: const Color(0xFF1E1E2A),
            borderRadius: BorderRadius.circular(32),
            border: Border.all(color: const Color(0xFF333340)),
          ),
          child: Stack(children: [
            // Dark gradient overlay
            Positioned.fill(child: Container(
              decoration: const BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter, end: Alignment.bottomCenter,
                  colors: [Color(0xCC1E1E2A), Colors.transparent, Color(0xDD111118)],
                ),
              ),
            )),
            // Content
            Padding(
              padding: const EdgeInsets.all(24),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(children: [
                    Container(
                      width: 56, height: 56,
                      decoration: BoxDecoration(
                        color: Colors.white.withValues(alpha: 0.1),
                        borderRadius: BorderRadius.circular(16),
                        border: Border.all(color: Colors.white.withValues(alpha: 0.15)),
                      ),
                      child: const Icon(Icons.videocam_rounded, size: 28, color: Colors.white70),
                    ),
                    const Spacer(),
                    if (online) Container(
                      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                      decoration: BoxDecoration(
                        color: const Color(0xCCEF4444),
                        borderRadius: BorderRadius.circular(4),
                        border: Border.all(color: const Color(0x60EF4444)),
                      ),
                      child: const Text('LIVE', style: TextStyle(
                        fontSize: 10, fontWeight: FontWeight.w800, color: Colors.white, letterSpacing: 0.5,
                      )),
                    ),
                  ]),
                  const SizedBox(height: 14),
                  const Text('Camera Feed', style: TextStyle(
                    fontSize: 18, fontWeight: FontWeight.w700, color: Colors.white, letterSpacing: -0.2,
                  )),
                  const SizedBox(height: 4),
                  Text('Front Optical Sensor', style: TextStyle(
                    fontSize: 13, color: Colors.white.withValues(alpha: 0.5),
                  )),
                  const Spacer(),
                  Row(children: [
                    Text('RES: 1080p', style: TextStyle(
                      fontSize: 10, fontFamily: 'monospace',
                      color: Colors.white.withValues(alpha: 0.4),
                    )),
                    const SizedBox(width: 4),
                    Text('FPS: 60', style: TextStyle(
                      fontSize: 10, fontFamily: 'monospace',
                      color: Colors.white.withValues(alpha: 0.4),
                    )),
                    const Spacer(),
                    Icon(Icons.open_in_full_rounded, size: 18,
                        color: Colors.white.withValues(alpha: 0.6)),
                  ]),
                ],
              ),
            ),
          ]),
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  HELPERS
  // ═══════════════════════════════════════════════════════════

  BoxDecoration _glassSurface(BuildContext context, {double radius = 12}) {
    return BoxDecoration(
      color: context.isDark
          ? Colors.white.withValues(alpha: 0.06)
          : Colors.white.withValues(alpha: 0.4),
      borderRadius: BorderRadius.circular(radius),
      border: Border.all(
        color: context.isDark
            ? Colors.white.withValues(alpha: 0.08)
            : Colors.white.withValues(alpha: 0.3),
      ),
    );
  }

  Widget _posBlob({double? top, double? bottom, double? left, double? right,
      required Color color, required double size}) {
    return Positioned(
      top: top, bottom: bottom, left: left, right: right,
      child: Container(
        width: size, height: size,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          gradient: RadialGradient(colors: [color, color.withValues(alpha: 0)]),
        ),
      ),
    );
  }

  Widget _anim(int i, Widget child) {
    final d = i * 0.1;
    final a = CurvedAnimation(
      parent: _stagger,
      curve: Interval(d.clamp(0.0, 0.7), (d + 0.3).clamp(0.0, 1.0), curve: Curves.easeOutCubic),
    );
    return FadeTransition(opacity: a, child: SlideTransition(
      position: Tween(begin: const Offset(0, 0.04), end: Offset.zero).animate(a),
      child: child,
    ));
  }
}

// ═════════════════════════════════════════════════════════════════
//  Tab-switch notification
// ═════════════════════════════════════════════════════════════════
class MainShellTabNotification extends Notification {
  final int tabIndex;
  const MainShellTabNotification(this.tabIndex);
}

// ═════════════════════════════════════════════════════════════════
//  GlassContainer — semi-transparent card with blur + border
// ═════════════════════════════════════════════════════════════════
class _GlassContainer extends StatelessWidget {
  final double radius;
  final EdgeInsetsGeometry padding;
  final Widget child;

  const _GlassContainer({
    this.radius = 32,
    this.padding = EdgeInsets.zero,
    required this.child,
  });

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return ClipRRect(
      borderRadius: BorderRadius.circular(radius),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 12, sigmaY: 12),
        child: Container(
          padding: padding,
          decoration: BoxDecoration(
            color: dark ? Colors.white.withValues(alpha: 0.08) : Colors.white.withValues(alpha: 0.45),
            borderRadius: BorderRadius.circular(radius),
            border: Border.all(
              color: dark ? Colors.white.withValues(alpha: 0.1) : Colors.white.withValues(alpha: 0.55),
            ),
            boxShadow: [BoxShadow(
              color: Colors.black.withValues(alpha: dark ? 0.18 : 0.06),
              blurRadius: 20, offset: const Offset(0, 8),
            )],
          ),
          child: child,
        ),
      ),
    );
  }
}

// ═════════════════════════════════════════════════════════════════
//  GlassFeatureCard — card with icon, title, glow blob, body
// ═════════════════════════════════════════════════════════════════
class _GlassFeatureCard extends StatefulWidget {
  final Color glowColor;
  final IconData icon;
  final Color iconColor;
  final Widget? topRight;
  final String title;
  final Widget body;
  final VoidCallback? onTap;

  const _GlassFeatureCard({
    required this.glowColor,
    required this.icon,
    required this.iconColor,
    this.topRight,
    required this.title,
    required this.body,
    this.onTap,
  });

  @override
  State<_GlassFeatureCard> createState() => _GlassFeatureCardState();
}

class _GlassFeatureCardState extends State<_GlassFeatureCard> {
  double _s = 1.0;
  void _d(TapDownDetails _) => setState(() => _s = 0.97);
  void _u(TapUpDetails _) => setState(() => _s = 1.0);
  void _c() => setState(() => _s = 1.0);

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return GestureDetector(
      onTapDown: widget.onTap != null ? _d : null,
      onTapUp: widget.onTap != null ? _u : null,
      onTapCancel: widget.onTap != null ? _c : null,
      onTap: widget.onTap,
      child: AnimatedScale(
        scale: _s,
        duration: AppDurations.fast,
        curve: Curves.easeOut,
        child: ClipRRect(
          borderRadius: BorderRadius.circular(32),
          child: BackdropFilter(
            filter: ImageFilter.blur(sigmaX: 12, sigmaY: 12),
            child: Container(
              decoration: BoxDecoration(
                color: dark ? Colors.white.withValues(alpha: 0.08) : Colors.white.withValues(alpha: 0.45),
                borderRadius: BorderRadius.circular(32),
                border: Border.all(
                  color: dark ? Colors.white.withValues(alpha: 0.1) : Colors.white.withValues(alpha: 0.55),
                ),
                boxShadow: [BoxShadow(
                  color: Colors.black.withValues(alpha: dark ? 0.18 : 0.06),
                  blurRadius: 20, offset: const Offset(0, 8),
                )],
              ),
              child: Stack(children: [
                // Decorative glow blob (top-right)
                Positioned(
                  top: -40, right: -40,
                  child: Container(
                    width: 128, height: 128,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      gradient: RadialGradient(colors: [
                        widget.glowColor.withValues(alpha: dark ? 0.06 : 0.12),
                        widget.glowColor.withValues(alpha: 0),
                      ]),
                    ),
                  ),
                ),
                // Card content
                Padding(
                  padding: const EdgeInsets.all(24),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      // Header: glass icon + topRight
                      Row(children: [
                        // Glass icon
                        Container(
                          width: 56, height: 56,
                          decoration: BoxDecoration(
                            gradient: LinearGradient(
                              begin: Alignment.topLeft, end: Alignment.bottomRight,
                              colors: dark
                                  ? [Colors.white.withValues(alpha: 0.1), Colors.white.withValues(alpha: 0.05)]
                                  : [Colors.white.withValues(alpha: 0.85), Colors.white.withValues(alpha: 0.45)],
                            ),
                            borderRadius: BorderRadius.circular(16),
                            border: Border.all(
                              color: dark ? Colors.white.withValues(alpha: 0.1) : Colors.white.withValues(alpha: 0.65),
                            ),
                            boxShadow: [BoxShadow(
                              color: Colors.black.withValues(alpha: 0.06), blurRadius: 8, offset: const Offset(0, 2),
                            )],
                          ),
                          child: Icon(widget.icon, size: 28, color: widget.iconColor),
                        ),
                        const Spacer(),
                        if (widget.topRight != null) widget.topRight!,
                      ]),
                      const SizedBox(height: 16),
                      // Title
                      Text(widget.title, style: TextStyle(
                        fontSize: 18, fontWeight: FontWeight.w700,
                        color: context.titleColor, letterSpacing: -0.2,
                      )),
                      const SizedBox(height: 6),
                      // Body
                      Expanded(child: widget.body),
                    ],
                  ),
                ),
              ]),
            ),
          ),
        ),
      ),
    );
  }
}

// ═════════════════════════════════════════════════════════════════
//  KPI Pill — glass card with icon, label, value
// ═════════════════════════════════════════════════════════════════
class _KpiPill extends StatelessWidget {
  final IconData icon;
  final Color iconBg;
  final Color iconColor;
  final String label;
  final String value;
  final double? progress;
  final Color? progressColor;

  const _KpiPill({
    required this.icon,
    required this.iconBg,
    required this.iconColor,
    required this.label,
    required this.value,
    this.progress,
    this.progressColor,
  });

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return ClipRRect(
      borderRadius: BorderRadius.circular(20),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 12, sigmaY: 12),
        child: Container(
          padding: const EdgeInsets.fromLTRB(16, 14, 28, 14),
          constraints: const BoxConstraints(minWidth: 190),
          decoration: BoxDecoration(
            color: dark ? Colors.white.withValues(alpha: 0.08) : Colors.white.withValues(alpha: 0.45),
            borderRadius: BorderRadius.circular(20),
            border: Border.all(
              color: dark ? Colors.white.withValues(alpha: 0.1) : Colors.white.withValues(alpha: 0.5),
            ),
            boxShadow: [BoxShadow(
              color: Colors.black.withValues(alpha: dark ? 0.15 : 0.05),
              blurRadius: 16, offset: const Offset(0, 6),
            )],
          ),
          child: Row(children: [
            Container(
              width: 48, height: 48,
              decoration: BoxDecoration(
                color: iconBg,
                borderRadius: BorderRadius.circular(14),
              ),
              child: Icon(icon, size: 26, color: iconColor),
            ),
            const SizedBox(width: 14),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(label, style: TextStyle(
                  fontSize: 10, fontWeight: FontWeight.w600,
                  color: context.hintColor, letterSpacing: 1,
                )),
                const SizedBox(height: 2),
                Row(children: [
                  Text(value, style: TextStyle(
                    fontSize: 22, fontWeight: FontWeight.w800,
                    color: context.titleColor, letterSpacing: -0.5,
                  )),
                  if (progress != null) ...[
                    const SizedBox(width: 8),
                    SizedBox(
                      width: 56, height: 6,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(3),
                        child: LinearProgressIndicator(
                          value: progress!,
                          backgroundColor: dark
                              ? Colors.white.withValues(alpha: 0.08)
                              : const Color(0xFFE5E7EB),
                          color: progressColor ?? iconColor,
                        ),
                      ),
                    ),
                  ],
                ]),
              ],
            ),
          ]),
        ),
      ),
    );
  }
}
