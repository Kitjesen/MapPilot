import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/system_gateway.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/shared/widgets/skeleton_loader.dart';
import 'package:flutter_monitor/shared/widgets/empty_state.dart';

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
  String? _streamError;

  // ── 运行时间计数器 ──
  Timer? _uptimeTimer;
  final DateTime _sessionStart = DateTime.now();

  // ── RTT 历史 (最近 30 次) ──
  final List<double> _rttHistory = [];
  static const int _rttMaxSamples = 30;

  @override
  void initState() {
    super.initState();
    final conn = context.read<RobotConnectionProvider>();
    _latest = conn.latestSlowState;
    _latestFast = conn.latestFastState;
    _sub = conn.slowStateStream.listen((ss) {
      if (mounted) setState(() { _latest = ss; _streamError = null; });
    }, onError: (e) {
      if (mounted) setState(() => _streamError = '$e');
    });
    _fastSub = conn.fastStateStream.listen((fs) {
      if (mounted) setState(() { _latestFast = fs; _streamError = null; });
    }, onError: (e) {
      if (mounted) setState(() => _streamError = '$e');
    });
    // 每分钟刷新运行时间
    _uptimeTimer = Timer.periodic(const Duration(minutes: 1), (_) {
      if (mounted) setState(() {});
    });
    // 采集 RTT 历史
    conn.addListener(_collectRtt);
  }

  void _collectRtt() {
    final rtt = context.read<RobotConnectionProvider>().connectionRttMs;
    if (rtt != null && rtt > 0) {
      _rttHistory.add(rtt);
      if (_rttHistory.length > _rttMaxSamples) {
        _rttHistory.removeAt(0);
      }
    }
  }

  Future<void> _refreshData() async {
    final conn = context.read<RobotConnectionProvider>();
    // Re-subscribe to get fresh data
    _sub?.cancel();
    _fastSub?.cancel();
    _latest = conn.latestSlowState;
    _latestFast = conn.latestFastState;
    _sub = conn.slowStateStream.listen((ss) {
      if (mounted) setState(() { _latest = ss; _streamError = null; });
    }, onError: (e) {
      if (mounted) setState(() => _streamError = '$e');
    });
    _fastSub = conn.fastStateStream.listen((fs) {
      if (mounted) setState(() { _latestFast = fs; _streamError = null; });
    }, onError: (e) {
      if (mounted) setState(() => _streamError = '$e');
    });
    setState(() => _streamError = null);
    // Give streams a moment to deliver fresh data
    await Future.delayed(const Duration(milliseconds: 500));
  }

  @override
  void dispose() {
    _sub?.cancel();
    _fastSub?.cancel();
    _uptimeTimer?.cancel();
    try {
      context.read<RobotConnectionProvider>().removeListener(_collectRtt);
    } catch (_) {}
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
      body: RefreshIndicator(
        onRefresh: _refreshData,
        child: _streamError != null
          ? ListView(
              physics: const AlwaysScrollableScrollPhysics(),
              children: [
                EmptyState(
                  icon: Icons.cloud_off_rounded,
                  title: '数据加载失败',
                  subtitle: _streamError,
                  actionLabel: '重试',
                  onAction: _refreshData,
                ),
              ],
            )
          : _latest == null
          ? ListView(
              physics: const AlwaysScrollableScrollPhysics(),
              padding: const EdgeInsets.all(20),
              children: const [
                SkeletonListTile(),
                SkeletonListTile(),
                SkeletonListTile(),
              ],
            )
          : ListView(
              physics: const AlwaysScrollableScrollPhysics(
                parent: BouncingScrollPhysics()),
              padding: const EdgeInsets.all(20),
              children: [
                // ── 综合健康 ──
                _overallCard(dark, health),
                const SizedBox(height: 16),

                // ── 系统资源 (CPU/内存/温度圆形仪表) ──
                _systemResourceCard(dark),
                const SizedBox(height: 16),

                // ── 磁盘使用量 ──
                _diskUsageCard(dark),
                const SizedBox(height: 16),

                // ── 运行时间 ──
                _uptimeCard(dark),
                const SizedBox(height: 16),

                // ── 网络延迟 ──
                _networkLatencyCard(dark),
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

                // ── 语义导航 ──
                _buildSemanticNavCard(dark),
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
  //  Semantic Navigation Card
  // ═══════════════════════════════════════════════════════

  Widget _buildSemanticNavCard(bool dark) {
    final gw = context.watch<TaskGateway>();
    final isSemantic = gw.activeTaskType == TaskType.TASK_TYPE_SEMANTIC_NAV && gw.isRunning;

    // 尝试从 SlowState.activeTask.paramsJson 解析语义指令
    String? semanticGoal;
    if (isSemantic) {
      final paramsJson = _latest?.activeTask.paramsJson ?? '';
      if (paramsJson.isNotEmpty) {
        try {
          final params = jsonDecode(paramsJson) as Map<String, dynamic>;
          semanticGoal = params['instruction'] as String?;
        } catch (_) {}
      }
    }

    // 从 taskStatus 推断阶段文字
    final (stageLabel, stageColor) = isSemantic
        ? switch (gw.taskStatus) {
            TaskStatus.TASK_STATUS_PENDING => ('准备中', context.subtitleColor),
            TaskStatus.TASK_STATUS_RUNNING => (
                gw.progress > 0.8 ? '接近目标' : '导航中',
                Colors.blue,
              ),
            TaskStatus.TASK_STATUS_PAUSED => ('已暂停', AppColors.warning),
            TaskStatus.TASK_STATUS_COMPLETED => ('已到达', AppColors.success),
            TaskStatus.TASK_STATUS_FAILED => ('任务失败', AppColors.error),
            TaskStatus.TASK_STATUS_CANCELLED => ('已取消', context.subtitleColor),
            _ => ('定位目标中', Colors.blue),
          }
        : ('语义导航未启动', context.subtitleColor);

    // 置信度 = 任务进度 (0.0-1.0)
    final confidence = isSemantic ? gw.progress.clamp(0.0, 1.0) : 0.0;
    final confColor = confidence < 0.6
        ? AppColors.error
        : confidence < 0.8
            ? AppColors.warning
            : AppColors.success;

    // 推理路径: 暂无 proto 字段，根据进度推断简化展示
    // progress < 0.3 → 可能在 Slow Path (LLM 推理阶段)
    // progress >= 0.3 → Fast Path (直接匹配后导航)
    final pathType = !isSemantic
        ? 'unknown'
        : confidence < 0.3
            ? 'slow'
            : 'fast';

    return _card(dark, child: Theme(
      data: Theme.of(context).copyWith(dividerColor: Colors.transparent),
      child: ExpansionTile(
        leading: Icon(Icons.psychology_rounded, size: 20,
          color: isSemantic ? Colors.blue : context.subtitleColor),
        title: Text('语义导航', style: TextStyle(
          fontSize: 14, fontWeight: FontWeight.w600,
          color: context.titleColor)),
        subtitle: Text(stageLabel, style: TextStyle(
          fontSize: 12, color: stageColor)),
        initiallyExpanded: isSemantic,
        children: [
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 0, 16, 14),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                // ── 当前目标 ──
                Row(children: [
                  Icon(Icons.flag_rounded, size: 14, color: context.subtitleColor),
                  const SizedBox(width: 6),
                  Text('当前目标', style: TextStyle(
                    fontSize: 12, color: context.subtitleColor)),
                  const SizedBox(width: 8),
                  Expanded(child: Text(
                    isSemantic ? (semanticGoal ?? '--') : '--',
                    style: TextStyle(
                      fontSize: 13, fontWeight: FontWeight.w500,
                      color: context.titleColor),
                    overflow: TextOverflow.ellipsis,
                  )),
                ]),
                const SizedBox(height: 10),

                // ── 推理路径 ──
                Row(children: [
                  Icon(Icons.route_rounded, size: 14, color: context.subtitleColor),
                  const SizedBox(width: 6),
                  Text('推理路径', style: TextStyle(
                    fontSize: 12, color: context.subtitleColor)),
                  const SizedBox(width: 8),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                    decoration: BoxDecoration(
                      color: switch (pathType) {
                        'fast' => AppColors.success.withValues(alpha: 0.13),
                        'slow' => Colors.blue.withValues(alpha: 0.13),
                        _ => context.subtitleColor.withValues(alpha: 0.10),
                      },
                      borderRadius: BorderRadius.circular(10),
                    ),
                    child: Text(
                      switch (pathType) {
                        'fast' => 'Fast Path',
                        'slow' => 'Slow Path',
                        _ => '--',
                      },
                      style: TextStyle(
                        fontSize: 11, fontWeight: FontWeight.w600,
                        color: switch (pathType) {
                          'fast' => AppColors.success,
                          'slow' => Colors.blue,
                          _ => context.subtitleColor,
                        },
                      ),
                    ),
                  ),
                ]),
                const SizedBox(height: 10),

                // ── 目标置信度 ──
                Row(children: [
                  Icon(Icons.analytics_outlined, size: 14, color: context.subtitleColor),
                  const SizedBox(width: 6),
                  Text('目标置信度', style: TextStyle(
                    fontSize: 12, color: context.subtitleColor)),
                  const Spacer(),
                  Text(
                    isSemantic
                        ? '${(confidence * 100).toStringAsFixed(0)}%'
                        : '--',
                    style: TextStyle(
                      fontSize: 12, fontWeight: FontWeight.w600,
                      color: isSemantic ? confColor : context.subtitleColor)),
                ]),
                const SizedBox(height: 6),
                ClipRRect(
                  borderRadius: BorderRadius.circular(3),
                  child: LinearProgressIndicator(
                    value: confidence,
                    backgroundColor: dark
                        ? Colors.white.withValues(alpha: 0.06)
                        : Colors.black.withValues(alpha: 0.04),
                    valueColor: AlwaysStoppedAnimation(
                      isSemantic ? confColor : context.subtitleColor.withValues(alpha: 0.3)),
                    minHeight: 4,
                  ),
                ),

                // ── 状态消息 ──
                if (isSemantic && gw.statusMessage != null) ...[
                  const SizedBox(height: 10),
                  Row(children: [
                    Icon(Icons.info_outline, size: 14, color: context.subtitleColor),
                    const SizedBox(width: 6),
                    Expanded(child: Text(gw.statusMessage!,
                      style: TextStyle(fontSize: 11, color: context.subtitleColor),
                      maxLines: 2, overflow: TextOverflow.ellipsis)),
                  ]),
                ],
              ],
            ),
          ),
        ],
      ),
    ));
  }

  // ═══════════════════════════════════════════════════════
  //  System Resource Gauges (CPU / Memory / Temp)
  // ═══════════════════════════════════════════════════════

  Widget _systemResourceCard(bool dark) {
    final res = _latest?.resources;
    final cpu = res?.cpuPercent ?? 0.0;
    final mem = res?.memPercent ?? 0.0;
    final temp = res?.cpuTemp ?? 0.0;
    final hasData = res != null && (cpu > 0 || mem > 0 || temp > 0);

    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Icon(Icons.monitor_heart_outlined, size: 20, color: context.subtitleColor),
            const SizedBox(width: 10),
            Text('系统资源', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600, color: context.titleColor)),
          ]),
          const SizedBox(height: 16),
          if (!hasData)
            Center(child: Padding(
              padding: const EdgeInsets.symmetric(vertical: 8),
              child: Text('无数据', style: TextStyle(
                fontSize: 12, color: context.subtitleColor)),
            ))
          else
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                _circularGauge('CPU', cpu, 100, '%',
                  cpu > 80 ? AppColors.error : cpu > 60 ? AppColors.warning : AppColors.success),
                _circularGauge('内存', mem, 100, '%',
                  mem > 85 ? AppColors.error : mem > 70 ? AppColors.warning : AppColors.info),
                _circularGauge('温度', temp, 100, '°C',
                  temp > 75 ? AppColors.error : temp > 55 ? AppColors.warning : AppColors.success),
              ],
            ),
        ],
      ),
    ));
  }

  Widget _circularGauge(String label, double value, double max, String unit, Color color) {
    final ratio = (value / max).clamp(0.0, 1.0);
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        SizedBox(
          width: 64, height: 64,
          child: CustomPaint(
            painter: _ArcGaugePainter(
              ratio: ratio,
              color: color,
              bgColor: color.withValues(alpha: 0.12),
              strokeWidth: 6,
            ),
            child: Center(child: AnimatedSwitcher(
              duration: const Duration(milliseconds: 300),
              transitionBuilder: (child, anim) => FadeTransition(opacity: anim, child: child),
              child: Text(
                value.toStringAsFixed(0),
                key: ValueKey(value.toStringAsFixed(0)),
                style: TextStyle(
                  fontSize: 16, fontWeight: FontWeight.w700,
                  color: context.titleColor),
              ),
            )),
          ),
        ),
        const SizedBox(height: 6),
        Text(label, style: TextStyle(
          fontSize: 11, fontWeight: FontWeight.w500, color: context.subtitleColor)),
        Text('${value.toStringAsFixed(1)}$unit', style: TextStyle(
          fontSize: 10, color: color, fontWeight: FontWeight.w600)),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════
  //  Disk Usage Bar
  // ═══════════════════════════════════════════════════════

  Widget _diskUsageCard(bool dark) {
    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Icon(Icons.storage_rounded, size: 20, color: context.subtitleColor),
            const SizedBox(width: 10),
            Text('磁盘使用', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600, color: context.titleColor)),
          ]),
          const SizedBox(height: 12),
          _DiskUsageLoader(dark: dark, cardBuilder: _card),
        ],
      ),
    ));
  }

  // ═══════════════════════════════════════════════════════
  //  Uptime Counter
  // ═══════════════════════════════════════════════════════

  Widget _uptimeCard(bool dark) {
    final sessionDuration = DateTime.now().difference(_sessionStart);
    final hours = sessionDuration.inHours;
    final minutes = sessionDuration.inMinutes % 60;
    final seconds = sessionDuration.inSeconds % 60;
    final sessionStr = '${hours.toString().padLeft(2, '0')}:'
        '${minutes.toString().padLeft(2, '0')}:'
        '${seconds.toString().padLeft(2, '0')}';

    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Row(children: [
        Icon(Icons.timer_outlined, size: 22, color: AppColors.info),
        const SizedBox(width: 14),
        Expanded(child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('运行时间', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600, color: context.titleColor)),
            const SizedBox(height: 2),
            Text('本次会话: $sessionStr',
              style: TextStyle(fontSize: 12, color: context.subtitleColor)),
          ],
        )),
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
          decoration: BoxDecoration(
            color: AppColors.info.withValues(alpha: 0.10),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Text(sessionStr,
            style: TextStyle(
              fontSize: 16, fontWeight: FontWeight.w700,
              fontFeatures: const [FontFeature.tabularFigures()],
              color: AppColors.info)),
        ),
      ]),
    ));
  }

  // ═══════════════════════════════════════════════════════
  //  Network Latency + RTT Sparkline
  // ═══════════════════════════════════════════════════════

  Widget _networkLatencyCard(bool dark) {
    final conn = context.watch<RobotConnectionProvider>();
    final rtt = conn.connectionRttMs;
    final quality = conn.connectionQuality;

    final (rttColor, qualityLabel) = switch (quality) {
      'good' => (AppColors.success, '优秀'),
      'slow' => (AppColors.warning, '良好'),
      'unstable' => (AppColors.error, '不稳定'),
      _ => (context.subtitleColor, '未知'),
    };

    return _card(dark, child: Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Icon(Icons.wifi_rounded, size: 20, color: rttColor),
            const SizedBox(width: 10),
            Text('网络', style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w600, color: context.titleColor)),
            const Spacer(),
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
              decoration: BoxDecoration(
                color: rttColor.withValues(alpha: 0.12),
                borderRadius: BorderRadius.circular(10),
              ),
              child: Text(qualityLabel, style: TextStyle(
                fontSize: 11, fontWeight: FontWeight.w600, color: rttColor)),
            ),
          ]),
          const SizedBox(height: 12),
          // RTT value
          Row(children: [
            Icon(Icons.swap_vert, size: 16, color: context.subtitleColor),
            const SizedBox(width: 6),
            Text('RTT 延迟', style: TextStyle(
              fontSize: 12, color: context.subtitleColor)),
            const Spacer(),
            AnimatedSwitcher(
              duration: const Duration(milliseconds: 300),
              transitionBuilder: (child, anim) => FadeTransition(opacity: anim, child: child),
              child: Text(
                rtt != null ? '${rtt.toStringAsFixed(1)} ms' : '-- ms',
                key: ValueKey(rtt?.toStringAsFixed(1) ?? '--'),
                style: TextStyle(
                  fontSize: 14, fontWeight: FontWeight.w700,
                  fontFeatures: const [FontFeature.tabularFigures()],
                  color: rttColor),
              ),
            ),
          ]),
          // RTT sparkline
          if (_rttHistory.length >= 2) ...[
            const SizedBox(height: 12),
            SizedBox(
              height: 40,
              width: double.infinity,
              child: CustomPaint(
                painter: _RttSparklinePainter(
                  values: _rttHistory,
                  lineColor: rttColor,
                  fillColor: rttColor.withValues(alpha: 0.08),
                  isDark: dark,
                ),
              ),
            ),
            const SizedBox(height: 4),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text('最近 ${_rttHistory.length} 次',
                  style: TextStyle(fontSize: 10, color: context.subtitleColor)),
                Text('范围: ${_rttHistory.reduce(math.min).toStringAsFixed(0)}'
                    ' - ${_rttHistory.reduce(math.max).toStringAsFixed(0)} ms',
                  style: TextStyle(fontSize: 10, color: context.subtitleColor)),
              ],
            ),
          ],
        ],
      ),
    ));
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

/// 270-degree arc gauge painter for system resource metrics.
class _ArcGaugePainter extends CustomPainter {
  final double ratio;
  final Color color;
  final Color bgColor;
  final double strokeWidth;

  _ArcGaugePainter({
    required this.ratio,
    required this.color,
    required this.bgColor,
    required this.strokeWidth,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = (math.min(size.width, size.height) - strokeWidth) / 2;
    const startAngle = -math.pi * 0.75;
    const sweepTotal = math.pi * 1.5;

    final bgPaint = Paint()
      ..color = bgColor
      ..style = PaintingStyle.stroke
      ..strokeWidth = strokeWidth
      ..strokeCap = StrokeCap.round;

    final fgPaint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = strokeWidth
      ..strokeCap = StrokeCap.round;

    canvas.drawArc(
      Rect.fromCircle(center: center, radius: radius),
      startAngle, sweepTotal, false, bgPaint,
    );
    if (ratio > 0) {
      canvas.drawArc(
        Rect.fromCircle(center: center, radius: radius),
        startAngle, sweepTotal * ratio, false, fgPaint,
      );
    }
  }

  @override
  bool shouldRepaint(_ArcGaugePainter old) =>
      old.ratio != ratio || old.color != color;
}

/// Lazily fetches DeviceInfo for disk usage and renders a horizontal bar.
class _DiskUsageLoader extends StatefulWidget {
  final bool dark;
  final Widget Function(bool dark, {required Widget child}) cardBuilder;

  const _DiskUsageLoader({required this.dark, required this.cardBuilder});

  @override
  State<_DiskUsageLoader> createState() => _DiskUsageLoaderState();
}

class _DiskUsageLoaderState extends State<_DiskUsageLoader> {
  DeviceInfoResponse? _info;
  bool _loading = false;
  bool _attempted = false;

  @override
  void initState() {
    super.initState();
    _fetchDiskInfo();
  }

  Future<void> _fetchDiskInfo() async {
    if (_attempted) return;
    _attempted = true;
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null || !client.otaAvailable) return;

    setState(() => _loading = true);
    try {
      final resp = await client.getDeviceInfo();
      if (mounted) setState(() => _info = resp);
    } catch (_) {}
    if (mounted) setState(() => _loading = false);
  }

  @override
  Widget build(BuildContext context) {
    if (_loading) {
      return const Center(child: Padding(
        padding: EdgeInsets.symmetric(vertical: 8),
        child: SizedBox(width: 18, height: 18,
          child: CircularProgressIndicator(strokeWidth: 2)),
      ));
    }

    if (_info == null || _info!.diskTotalBytes.toInt() == 0) {
      return Text('磁盘信息不可用',
        style: TextStyle(fontSize: 12, color: context.subtitleColor));
    }

    final total = _info!.diskTotalBytes.toInt();
    final free = _info!.diskFreeBytes.toInt();
    final used = total - free;
    final ratio = total > 0 ? (used / total).clamp(0.0, 1.0) : 0.0;
    final usedGB = used / (1024 * 1024 * 1024);
    final totalGB = total / (1024 * 1024 * 1024);
    final pct = (ratio * 100).toStringAsFixed(1);

    final barColor = ratio > 0.9
        ? AppColors.error
        : ratio > 0.75
            ? AppColors.warning
            : AppColors.info;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        ClipRRect(
          borderRadius: BorderRadius.circular(4),
          child: SizedBox(
            height: 10,
            child: LinearProgressIndicator(
              value: ratio,
              backgroundColor: widget.dark
                  ? Colors.white.withValues(alpha: 0.06)
                  : Colors.black.withValues(alpha: 0.04),
              valueColor: AlwaysStoppedAnimation(barColor),
            ),
          ),
        ),
        const SizedBox(height: 6),
        Text(
          '已用 ${usedGB.toStringAsFixed(1)} GB / 总共 ${totalGB.toStringAsFixed(1)} GB ($pct%)',
          style: TextStyle(fontSize: 12, color: context.subtitleColor),
        ),
      ],
    );
  }
}

/// Sparkline chart for RTT history values.
class _RttSparklinePainter extends CustomPainter {
  final List<double> values;
  final Color lineColor;
  final Color fillColor;
  final bool isDark;

  _RttSparklinePainter({
    required this.values,
    required this.lineColor,
    required this.fillColor,
    required this.isDark,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (values.length < 2) return;

    final minV = values.reduce(math.min);
    final maxV = values.reduce(math.max);
    final range = maxV - minV;
    final effectiveRange = range < 1 ? 1.0 : range;

    final points = <Offset>[];
    for (var i = 0; i < values.length; i++) {
      final x = i / (values.length - 1) * size.width;
      final y = size.height -
          ((values[i] - minV) / effectiveRange) * size.height * 0.9 -
          size.height * 0.05;
      points.add(Offset(x, y));
    }

    // Fill area under line
    final fillPath = Path()
      ..moveTo(0, size.height)
      ..lineTo(points.first.dx, points.first.dy);
    for (final p in points.skip(1)) {
      fillPath.lineTo(p.dx, p.dy);
    }
    fillPath
      ..lineTo(size.width, size.height)
      ..close();
    canvas.drawPath(fillPath, Paint()..color = fillColor);

    // Line
    final linePaint = Paint()
      ..color = lineColor
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5
      ..strokeJoin = StrokeJoin.round;

    final linePath = Path()..moveTo(points.first.dx, points.first.dy);
    for (final p in points.skip(1)) {
      linePath.lineTo(p.dx, p.dy);
    }
    canvas.drawPath(linePath, linePaint);

    // Latest point dot
    if (points.isNotEmpty) {
      canvas.drawCircle(points.last, 3, Paint()..color = lineColor);
    }
  }

  @override
  bool shouldRepaint(_RttSparklinePainter old) => true;
}
