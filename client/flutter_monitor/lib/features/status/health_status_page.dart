import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
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
                  Text('${healthScore.toStringAsFixed(0)}',
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
