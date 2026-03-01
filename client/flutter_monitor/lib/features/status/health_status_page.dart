import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/system_gateway.dart';
import 'package:robot_proto/robot_proto.dart';

/// 健康状态页面 — 显示子系统健康、定位质量和围栏状态
class HealthStatusPage extends StatefulWidget {
  const HealthStatusPage({super.key});

  @override
  State<HealthStatusPage> createState() => _HealthStatusPageState();
}

class _HealthStatusPageState extends State<HealthStatusPage> {
  StreamSubscription<SlowState>? _sub;
  StreamSubscription<FastState>? _fastSub;
  SlowState? _latest;
  FastState? _latestFast;

  @override
  void initState() {
    super.initState();
    final conn = context.read<RobotConnectionProvider>();
    _latest = conn.latestSlowState;
    _latestFast = conn.latestFastState;
    _sub = conn.slowStateStream.listen((ss) {
      if (mounted) setState(() => _latest = ss);
    });
    _fastSub = conn.fastStateStream.listen((fs) {
      if (mounted) setState(() => _latestFast = fs);
    });
  }

  @override
  void dispose() {
    _sub?.cancel();
    _fastSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    final health = _latest?.health;
    final geofence = _latest?.geofence;

    return Scaffold(
      appBar: AppBar(
        title: const Text('系统健康'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: _latest == null
          ? Center(
              child: Text('等待数据...',
                  style: TextStyle(color: context.subtitleColor)))
          : ListView(
              physics: const BouncingScrollPhysics(),
              padding: const EdgeInsets.all(20),
              children: [
                // ── 综合健康 ──
                _overallCard(dark, health),
                const SizedBox(height: 16),

                // ── 定位质量 ──
                _localizationCard(dark, health),
                const SizedBox(height: 16),

                // ── 围栏状态 ──
                _geofenceCard(dark, geofence),
                const SizedBox(height: 16),

                // ── 规划质量 ──
                _navQualityCard(dark, _latest!.navigation, _latest!.topicRates, _latestFast),
                const SizedBox(height: 16),

                // ── 系统服务 ──
                _buildServiceDiagCard(dark),
                const SizedBox(height: 16),

                // ── 子系统列表 ──
                Text('子系统', style: TextStyle(
                  fontSize: 13, fontWeight: FontWeight.w600,
                  color: context.titleColor,
                )),
                const SizedBox(height: 8),
                if (health == null || health.subsystems.isEmpty)
                  _emptyHint('暂无子系统数据')
                else
                  ...health.subsystems.map((s) => Padding(
                    padding: const EdgeInsets.only(bottom: 8),
                    child: _subsystemCard(dark, s),
                  )),
              ],
            ),
    );
  }

  // ═══════════════════════════════════════════════════════
  //  Cards
  // ═══════════════════════════════════════════════════════

  Widget _overallCard(bool dark, HealthStatus? h) {
    final level = h?.overallLevel ?? 'UNKNOWN';
    final (color, icon) = _levelStyle(level);
    return _card(dark, child: ListTile(
      leading: Icon(icon, color: color, size: 28),
      title: Text('综合健康',
        style: TextStyle(fontSize: 15, fontWeight: FontWeight.w600,
          color: context.titleColor)),
      subtitle: Text(level,
        style: TextStyle(fontSize: 13, fontWeight: FontWeight.w500,
          color: color)),
    ));
  }

  Widget _localizationCard(bool dark, HealthStatus? h) {
    // 优先使用 FastState 的 0-100 定位健康评分 (由 LocalizationScorer 生成)
    final healthScore = _latestFast?.localizationScore ?? 0.0;
    final speedScale = _latestFast?.speedScale ?? 1.0;
    final icpRaw = h?.localizationScore ?? -1;
    final available = healthScore > 0 || icpRaw >= 0;

    final quality = !available
        ? '不可用'
        : healthScore >= 80
            ? '优秀'
            : healthScore >= 60
                ? '良好'
                : healthScore >= 40
                    ? '降级'
                    : healthScore >= 20
                        ? '差'
                        : '丢失';
    final color = !available
        ? context.subtitleColor
        : healthScore >= 80
            ? AppColors.success
            : healthScore >= 60
                ? const Color(0xFF4CAF50)
                : healthScore >= 40
                    ? AppColors.warning
                    : healthScore >= 20
                        ? const Color(0xFFFF6B00)
                        : AppColors.error;

    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Icon(Icons.gps_fixed, color: color, size: 22),
            const SizedBox(width: 14),
            Expanded(child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('定位健康', style: TextStyle(
                  fontSize: 14, fontWeight: FontWeight.w600,
                  color: context.titleColor)),
                const SizedBox(height: 2),
                Text(available
                    ? '$quality — ${healthScore.toStringAsFixed(0)}/100'
                    : '不可用',
                  style: TextStyle(fontSize: 12, color: color)),
              ],
            )),
            // 评分圆环
            if (available)
              SizedBox(
                width: 44, height: 44,
                child: Stack(alignment: Alignment.center, children: [
                  CircularProgressIndicator(
                    value: (healthScore / 100).clamp(0.0, 1.0),
                    strokeWidth: 4,
                    backgroundColor: color.withValues(alpha: 0.15),
                    valueColor: AlwaysStoppedAnimation(color),
                  ),
                  Text(healthScore.toStringAsFixed(0),
                    style: TextStyle(
                      fontSize: 12, fontWeight: FontWeight.w700,
                      color: context.titleColor)),
                ]),
              ),
          ]),
          if (available) ...[
            const SizedBox(height: 10),
            // 速度缩放条
            Row(children: [
              Icon(Icons.speed, size: 14, color: context.subtitleColor),
              const SizedBox(width: 6),
              Text('限速因子: ${(speedScale * 100).toStringAsFixed(0)}%',
                style: TextStyle(fontSize: 11, color: context.subtitleColor)),
              if (speedScale < 1.0 && speedScale > 0) ...[
                const SizedBox(width: 8),
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                  decoration: BoxDecoration(
                    color: AppColors.warning.withValues(alpha: 0.15),
                    borderRadius: BorderRadius.circular(4)),
                  child: Text('自动降速中',
                    style: TextStyle(fontSize: 10, color: AppColors.warning,
                      fontWeight: FontWeight.w600)),
                ),
              ],
              if (speedScale <= 0) ...[
                const SizedBox(width: 8),
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                  decoration: BoxDecoration(
                    color: AppColors.error.withValues(alpha: 0.15),
                    borderRadius: BorderRadius.circular(4)),
                  child: Text('已停车',
                    style: TextStyle(fontSize: 10, color: AppColors.error,
                      fontWeight: FontWeight.w600)),
                ),
              ],
            ]),
            if (icpRaw >= 0) ...[
              const SizedBox(height: 4),
              Text('ICP 原始偏差: ${icpRaw.toStringAsFixed(3)}',
                style: TextStyle(fontSize: 10, color: context.subtitleColor)),
            ],
          ],
        ],
      ),
    ));
  }

  Widget _geofenceCard(bool dark, GeofenceStatus? g) {
    final state = g?.state ?? 'NO_FENCE';
    final hasFence = g?.hasFence ?? false;
    final margin = g?.marginDistance ?? 0;
    final (color, icon) = switch (state) {
      'SAFE' => (AppColors.success, Icons.check_circle_outline),
      'WARNING' => (Colors.orange, Icons.warning_amber),
      'VIOLATION' => (AppColors.error, Icons.dangerous),
      _ => (context.subtitleColor, Icons.fence),
    };

    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Row(children: [
        Icon(icon, color: color, size: 22),
        const SizedBox(width: 14),
        Expanded(child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('围栏状态', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600,
              color: context.titleColor)),
            const SizedBox(height: 2),
            Text(
              hasFence
                  ? '$state (边距: ${margin.toStringAsFixed(1)} m)'
                  : '未设置围栏',
              style: TextStyle(fontSize: 12, color: color)),
          ],
        )),
      ]),
    ));
  }

  Widget _navQualityCard(bool dark, NavigationStatus? nav, TopicRates? rates, FastState? fast) {
    final status    = nav?.globalPlannerStatus ?? '';
    final pathLen   = nav?.hasGlobalPath == true ? nav!.globalPathLength : -1.0;
    final slowLevel = nav?.slowDownLevel ?? 0;
    final hasWp     = nav?.hasWaypoint ?? false;
    final speedScale = fast?.speedScale ?? 1.0;

    final (statusColor, statusIcon) = switch (status.toUpperCase()) {
      'PLANNING' => (Colors.blue, Icons.cached),
      'SUCCESS'  => (AppColors.success, Icons.check_circle_outline),
      'FAILED'   => (AppColors.error, Icons.cancel_outlined),
      'IDLE'     => (context.subtitleColor, Icons.pause_circle_outline),
      _          => (context.subtitleColor, Icons.help_outline),
    };

    final slowColors = [AppColors.success, AppColors.warning, Colors.orange, AppColors.error];
    final slowColor  = slowColors[slowLevel.clamp(0, 3)];
    final slowLabel  = ['正常', '一级减速', '二级减速', '三级减速'][slowLevel.clamp(0, 3)];

    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // ── 标题行 ──
          Row(children: [
            Icon(Icons.alt_route, color: context.subtitleColor, size: 20),
            const SizedBox(width: 10),
            Text('规划质量', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600, color: context.titleColor)),
          ]),
          const SizedBox(height: 12),

          // ── 规划状态 + 路径长度 ──
          Row(children: [
            Icon(statusIcon, color: statusColor, size: 16),
            const SizedBox(width: 6),
            Text(status.isEmpty ? '—' : status,
              style: TextStyle(fontSize: 12, color: statusColor, fontWeight: FontWeight.w500)),
            const Spacer(),
            if (pathLen >= 0)
              Text('路径 ${pathLen.toStringAsFixed(1)} m',
                style: TextStyle(fontSize: 12, color: context.subtitleColor)),
            if (!hasWp)
              Text('无航点', style: TextStyle(fontSize: 12, color: context.subtitleColor)),
          ]),
          const SizedBox(height: 8),

          // ── 减速等级 ──
          Row(children: [
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
              decoration: BoxDecoration(
                color: slowColor.withValues(alpha: 0.13),
                borderRadius: BorderRadius.circular(4)),
              child: Text(slowLabel,
                style: TextStyle(fontSize: 11, fontWeight: FontWeight.w600, color: slowColor)),
            ),
            const SizedBox(width: 8),
            if (speedScale < 1.0 && speedScale > 0)
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                decoration: BoxDecoration(
                  color: AppColors.warning.withValues(alpha: 0.13),
                  borderRadius: BorderRadius.circular(4)),
                child: Text('限速 ${(speedScale * 100).toStringAsFixed(0)}%',
                  style: TextStyle(fontSize: 11, color: AppColors.warning, fontWeight: FontWeight.w600)),
              ),
          ]),

          // ── 话题频率 ──
          if (rates != null) ...[
            const SizedBox(height: 10),
            _rateRow('全局路径', rates.globalPathHz, 1.0),
            const SizedBox(height: 4),
            _rateRow('局部路径', rates.pathHz, 4.0),
            const SizedBox(height: 4),
            _rateRow('控制指令', rates.cmdVelHz, 20.0),
          ],
        ],
      ),
    ));
  }

  Widget _rateRow(String label, double actual, double expected) {
    final ratio = expected > 0 ? (actual / expected).clamp(0.0, 1.0) : 0.0;
    final color = ratio >= 0.8
        ? AppColors.success
        : ratio >= 0.4
            ? AppColors.warning
            : AppColors.error;
    return Row(children: [
      SizedBox(width: 64,
        child: Text(label, style: TextStyle(fontSize: 11, color: context.subtitleColor))),
      Expanded(child: ClipRRect(
        borderRadius: BorderRadius.circular(2),
        child: LinearProgressIndicator(
          value: ratio,
          backgroundColor: context.isDark
              ? Colors.white.withValues(alpha: 0.06)
              : Colors.black.withValues(alpha: 0.04),
          valueColor: AlwaysStoppedAnimation(color.withValues(alpha: 0.7)),
          minHeight: 4,
        ),
      )),
      const SizedBox(width: 8),
      SizedBox(width: 52,
        child: Text('${actual.toStringAsFixed(1)} Hz',
          textAlign: TextAlign.right,
          style: TextStyle(fontSize: 11, color: context.subtitleColor))),
    ]);
  }

  Widget _subsystemCard(bool dark, SubsystemHealth s) {
    final (color, icon) = _levelStyle(s.level);
    final hasHz = s.expectedHz > 0;
    return _card(dark, child: Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      child: Row(children: [
        Icon(icon, color: color, size: 18),
        const SizedBox(width: 12),
        Expanded(child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(children: [
              Text(s.name, style: TextStyle(
                fontSize: 14, fontWeight: FontWeight.w600,
                color: context.titleColor)),
              const Spacer(),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                decoration: BoxDecoration(
                  color: color.withValues(alpha: 0.12),
                  borderRadius: BorderRadius.circular(4),
                ),
                child: Text(s.level, style: TextStyle(
                  fontSize: 10, fontWeight: FontWeight.w600, color: color)),
              ),
            ]),
            if (s.message.isNotEmpty) ...[
              const SizedBox(height: 2),
              Text(s.message, style: TextStyle(
                fontSize: 12, color: context.subtitleColor)),
            ],
            if (hasHz) ...[
              const SizedBox(height: 4),
              _hzBar(s.actualHz, s.expectedHz, color),
            ],
          ],
        )),
      ]),
    ));
  }

  Widget _hzBar(double actual, double expected, Color color) {
    final ratio = expected > 0 ? (actual / expected).clamp(0.0, 1.0) : 0.0;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text('${actual.toStringAsFixed(1)} / ${expected.toStringAsFixed(0)} Hz',
          style: TextStyle(fontSize: 11, color: context.subtitleColor)),
        const SizedBox(height: 3),
        ClipRRect(
          borderRadius: BorderRadius.circular(2),
          child: LinearProgressIndicator(
            value: ratio,
            backgroundColor: context.isDark
                ? Colors.white.withValues(alpha: 0.06)
                : Colors.black.withValues(alpha: 0.04),
            valueColor: AlwaysStoppedAnimation(color.withValues(alpha: 0.6)),
            minHeight: 3,
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════
  //  Service Diagnostics
  // ═══════════════════════════════════════════════════════

  Widget _buildServiceDiagCard(bool dark) {
    return Consumer<SystemGateway>(
      builder: (context, gateway, _) {
        return _card(dark, child: Theme(
          data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
          child: ExpansionTile(
            leading: Icon(Icons.miscellaneous_services, size: 20,
              color: context.subtitleColor),
            title: Text('系统服务', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600,
              color: context.titleColor)),
            onExpansionChanged: (expanded) {
              if (expanded && gateway.services.isEmpty && gateway.error == null) {
                gateway.fetchServiceStatuses();
              }
            },
            children: [
              if (gateway.error != null)
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                  child: Row(children: [
                    Icon(Icons.info_outline, size: 14, color: AppColors.warning),
                    const SizedBox(width: 8),
                    Expanded(child: Text(gateway.error!,
                      style: TextStyle(fontSize: 12, color: AppColors.warning))),
                  ]),
                ),
              if (gateway.loading)
                const Padding(
                  padding: EdgeInsets.all(16),
                  child: Center(child: SizedBox(
                    width: 20, height: 20,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  )),
                )
              else if (gateway.services.isEmpty && gateway.error == null)
                Padding(
                  padding: const EdgeInsets.all(16),
                  child: Text('暂无服务数据',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor)),
                )
              else
                ...gateway.services.map((svc) => _serviceRow(dark, gateway, svc)),
              if (gateway.services.isNotEmpty)
                Padding(
                  padding: const EdgeInsets.only(bottom: 8),
                  child: TextButton.icon(
                    onPressed: gateway.loading ? null : () => gateway.fetchServiceStatuses(),
                    icon: const Icon(Icons.refresh, size: 14),
                    label: const Text('刷新', style: TextStyle(fontSize: 12)),
                  ),
                ),
            ],
          ),
        ));
      },
    );
  }

  Widget _serviceRow(bool dark, SystemGateway gateway, ServiceStatus svc) {
    final state = svc.state.toLowerCase();
    final isRunning = state == 'running' || state == 'active';
    final isDegraded = state == 'degraded' || state == 'activating' || state == 'reloading';
    final color = isRunning
        ? AppColors.success
        : isDegraded
            ? AppColors.warning
            : AppColors.error;
    final isRestarting = gateway.restartingService == svc.name;

    // 友好显示名
    final displayName = svc.name
        .replaceAll('nav-', '')
        .replaceAll('ota-', 'OTA ')
        .replaceAll('-', ' ');

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      child: Row(children: [
        // 状态点
        Container(
          width: 8, height: 8,
          decoration: BoxDecoration(
            color: color,
            shape: BoxShape.circle,
          ),
        ),
        const SizedBox(width: 10),
        // 服务名
        Expanded(child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(displayName, style: TextStyle(
              fontSize: 13, fontWeight: FontWeight.w500,
              color: context.titleColor)),
            Text(svc.state.isEmpty ? '--' : svc.state,
              style: TextStyle(fontSize: 11, color: context.subtitleColor)),
          ],
        )),
        // 重启按钮
        if (isRestarting)
          const SizedBox(
            width: 20, height: 20,
            child: CircularProgressIndicator(strokeWidth: 2),
          )
        else
          SizedBox(
            height: 28,
            child: TextButton(
              style: TextButton.styleFrom(
                padding: const EdgeInsets.symmetric(horizontal: 8),
                minimumSize: Size.zero,
                tapTargetSize: MaterialTapTargetSize.shrinkWrap,
              ),
              onPressed: () => _confirmRestart(context, gateway, svc.name, displayName),
              child: Text('重启', style: TextStyle(
                fontSize: 11, color: context.subtitleColor)),
            ),
          ),
      ]),
    );
  }

  void _confirmRestart(
    BuildContext context,
    SystemGateway gateway,
    String serviceName,
    String displayName,
  ) {
    final messenger = ScaffoldMessenger.of(context);
    showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('确认重启'),
        content: Text('确认重启 $displayName 服务？'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: const Text('取消'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            child: const Text('重启'),
          ),
        ],
      ),
    ).then((confirmed) async {
      if (confirmed != true) return;
      final ok = await gateway.restartService(serviceName);
      if (mounted) {
        messenger.showSnackBar(SnackBar(
          content: Text(ok ? '$displayName 已重启' : '重启失败: ${gateway.error ?? ""}'),
          duration: const Duration(seconds: 2),
        ));
      }
    });
  }

  // ═══════════════════════════════════════════════════════
  //  Helpers
  // ═══════════════════════════════════════════════════════

  (Color, IconData) _levelStyle(String level) => switch (level.toUpperCase()) {
    'OK' => (AppColors.success, Icons.check_circle),
    'DEGRADED' => (Colors.orange, Icons.warning_amber),
    'CRITICAL' => (AppColors.error, Icons.error),
    'FAULT' => (AppColors.error, Icons.cancel),
    _ => (context.subtitleColor, Icons.help_outline),
  };

  Widget _card(bool dark, {required Widget child}) {
    return Container(
      decoration: BoxDecoration(
        color: dark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [dark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: child,
    );
  }

  Widget _emptyHint(String text) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 24),
      child: Center(child: Text(text,
        style: TextStyle(fontSize: 13, color: context.subtitleColor))),
    );
  }
}
