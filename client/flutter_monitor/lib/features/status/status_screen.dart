import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'package:provider/provider.dart';
import 'package:shimmer/shimmer.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/shared/widgets/glass_widgets.dart';
import 'package:flutter_monitor/core/grpc/dog_direct_client.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

class StatusScreen extends StatefulWidget {
  const StatusScreen({super.key});

  @override
  State<StatusScreen> createState() => _StatusScreenState();
}

class _StatusScreenState extends State<StatusScreen>
    with AutomaticKeepAliveClientMixin, SingleTickerProviderStateMixin {

  // 本地缓存 — 只在 stream 回调中更新
  FastState? _latestFastState;
  SlowState? _latestSlowState;
  StreamSubscription<FastState>? _fastSub;
  StreamSubscription<SlowState>? _slowSub;

  // Dog direct sensor subscriptions
  StreamSubscription? _dogImuSub;
  StreamSubscription? _dogJointSub;

  late AnimationController _breathingController;
  late Animation<double> _breathingAnimation;

  // UI 刷新节流
  DateTime _lastUiUpdate = DateTime(2000);
  static const _uiThrottleInterval = Duration(milliseconds: 100); // 10 FPS max

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    _breathingController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    )..repeat(reverse: true);

    _breathingAnimation = Tween<double>(begin: 0.4, end: 1.0).animate(
      CurvedAnimation(parent: _breathingController, curve: Curves.easeInOut),
    );

    // 使用 Provider 的广播流
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _subscribeStreams();
    });
  }

  void _subscribeStreams() {
    final provider = context.read<RobotConnectionProvider>();

    // 初始数据
    _latestFastState = provider.latestFastState;
    _latestSlowState = provider.latestSlowState;

    _fastSub = provider.fastStateStream.listen((state) {
      _latestFastState = state;
      _throttledSetState();
    });

    _slowSub = provider.slowStateStream.listen((state) {
      _latestSlowState = state;
      if (mounted) setState(() {});
    });

    // Dog direct sensor streams
    _subscribeDogStreams(provider);
  }

  void _subscribeDogStreams(RobotConnectionProvider provider) {
    _dogImuSub?.cancel();
    _dogJointSub?.cancel();
    final dogClient = provider.dogClient;
    if (dogClient == null) return;

    _dogImuSub = dogClient.imuStream.listen((_) => _throttledSetState());
    _dogJointSub = dogClient.jointStream.listen((_) {
      if (mounted) setState(() {});
    });
  }

  void _throttledSetState() {
    final now = DateTime.now();
    if (now.difference(_lastUiUpdate) >= _uiThrottleInterval) {
      _lastUiUpdate = now;
      if (mounted) setState(() {});
    }
  }

  @override
  void dispose() {
    _fastSub?.cancel();
    _slowSub?.cancel();
    _dogImuSub?.cancel();
    _dogJointSub?.cancel();
    _breathingController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    super.build(context);
    final isDark = context.isDark;

    final provider = context.watch<RobotConnectionProvider>();
    final dogClient = provider.dogClient;
    final isDogConnected = provider.isDogConnected;
    final hasAnyData = _latestFastState != null || isDogConnected;

    return Scaffold(
      appBar: AppBar(
        title: const Text('状态总览'),
        leading: Navigator.canPop(context)
            ? IconButton(
                icon: const Icon(Icons.arrow_back_ios_new, size: 20),
                onPressed: () => Navigator.pop(context),
              )
            : null,
        actions: [
          // Dog connection badge
          if (isDogConnected)
            Padding(
              padding: const EdgeInsets.only(right: 8.0),
              child: Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                decoration: BoxDecoration(
                  color: AppColors.warning.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(20),
                ),
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Container(
                      width: 8,
                      height: 8,
                      decoration: const BoxDecoration(
                        color: AppColors.warning,
                        shape: BoxShape.circle,
                      ),
                    ),
                    const SizedBox(width: 6),
                    const Text(
                      'DOG',
                      style: TextStyle(
                        fontSize: 10,
                        fontWeight: FontWeight.w700,
                        color: AppColors.warning,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          Padding(
            padding: const EdgeInsets.only(right: 16.0),
            child: _buildStatusBadge(),
          ),
        ],
      ),
      body: !hasAnyData
          ? _buildLoadingSkeleton()
          : RefreshIndicator(
              onRefresh: () async {
                _fastSub?.cancel();
                _slowSub?.cancel();
                _subscribeStreams();
              },
              child: SingleChildScrollView(
                physics: const AlwaysScrollableScrollPhysics(),
                padding: EdgeInsets.fromLTRB(
                    16, MediaQuery.of(context).padding.top + 16, 16, 100),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    if (_latestFastState != null) ...[
                    RepaintBoundary(child: _buildRobotCard()),
                    const SizedBox(height: 14),
                    Row(
                      children: [
                        Expanded(
                          child: RepaintBoundary(
                            child: _buildMetricCard(
                              icon: Icons.battery_charging_full,
                              iconColor: AppColors.success,
                              label: 'BATTERY',
                              value: (_latestSlowState?.resources.batteryPercent ?? 0)
                                  .toStringAsFixed(0),
                              unit: '%',
                              progress: (_latestSlowState?.resources.batteryPercent ?? 0) / 100,
                              progressColor: _getBatteryColor(
                                  _latestSlowState?.resources.batteryPercent ?? 100),
                            ),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: RepaintBoundary(
                            child: _buildMetricCard(
                              icon: Icons.memory,
                              iconColor: AppColors.primary,
                              label: 'CPU',
                              value: (_latestSlowState?.resources.cpuPercent ?? 0)
                                  .toStringAsFixed(0),
                              unit: '%',
                              progress: (_latestSlowState?.resources.cpuPercent ?? 0) / 100,
                              progressColor: AppColors.primary,
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(
                          child: RepaintBoundary(
                            child: _buildMetricCard(
                              icon: Icons.thermostat,
                              iconColor: AppColors.warning,
                              label: 'TEMP',
                              value: (_latestSlowState?.resources.cpuTemp ?? 0)
                                  .toStringAsFixed(1),
                              unit: '°C',
                              progress: math.min(
                                  (_latestSlowState?.resources.cpuTemp ?? 0) / 80, 1.0),
                              progressColor: _getTempColor(
                                  _latestSlowState?.resources.cpuTemp ?? 0),
                            ),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: RepaintBoundary(
                            child: _buildMetricCard(
                              icon: Icons.bolt,
                              iconColor: const Color(0xFFFFCC00),
                              label: 'VOLTAGE',
                              value: (_latestSlowState?.resources.batteryVoltage ?? 0)
                                  .toStringAsFixed(1),
                              unit: 'V',
                              progress: math.min(
                                  (_latestSlowState?.resources.batteryVoltage ?? 0) / 30, 1.0),
                              progressColor: const Color(0xFFFFCC00),
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 14),
                    RepaintBoundary(child: _buildMotionCard()),
                    if (_latestSlowState != null) ...[
                      const SizedBox(height: 14),
                      RepaintBoundary(child: _buildTopicRatesCard()),
                    ],
                    ], // end _latestFastState != null
                    // ─── Dog Direct Sensor Data ───
                    if (isDogConnected && dogClient != null) ...[
                      const SizedBox(height: 14),
                      RepaintBoundary(
                          child: _buildDogImuCard(dogClient)),
                      const SizedBox(height: 14),
                      RepaintBoundary(
                          child: _buildDogJointCard(dogClient)),
                    ],
                  ],
                ),
              ),
            ),
    );
  }

  Widget _buildLoadingSkeleton() {
    final isDark = context.isDark;
    return Padding(
      padding: EdgeInsets.fromLTRB(
          16, MediaQuery.of(context).padding.top + 16, 16, 100),
      child: Column(
        children: [
          _buildSkeletonCard(height: 180),
          const SizedBox(height: 14),
          Row(
            children: [
              Expanded(child: _buildSkeletonCard(height: 130)),
              const SizedBox(width: 12),
              Expanded(child: _buildSkeletonCard(height: 130)),
            ],
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Expanded(child: _buildSkeletonCard(height: 130)),
              const SizedBox(width: 12),
              Expanded(child: _buildSkeletonCard(height: 130)),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildSkeletonCard({required double height}) {
    final isDark = context.isDark;
    return Shimmer.fromColors(
      baseColor: isDark ? Colors.white.withOpacity(0.06) : Colors.grey.shade200,
      highlightColor: isDark ? Colors.white.withOpacity(0.12) : Colors.grey.shade100,
      child: Container(
        height: height,
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(22),
        ),
      ),
    );
  }

  Color _getBatteryColor(double percent) {
    if (percent > 60) return AppColors.success;
    if (percent > 30) return AppColors.warning;
    return AppColors.error;
  }

  Color _getTempColor(double temp) {
    if (temp < 50) return AppColors.warning;
    if (temp < 70) return const Color(0xFFFF6B00);
    return AppColors.error;
  }

  Widget _buildStatusBadge() {
    return Selector<RobotConnectionProvider, ConnectionStatus>(
      selector: (_, p) => p.status,
      builder: (context, status, _) {
        final isOnline = status == ConnectionStatus.connected;
        final isReconnecting = status == ConnectionStatus.reconnecting;
        final color = isOnline
            ? AppColors.success
            : isReconnecting
                ? AppColors.warning
                : AppColors.error;
        final label = isOnline
            ? 'ONLINE'
            : isReconnecting
                ? 'RECONNECTING'
                : 'OFFLINE';

        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          decoration: BoxDecoration(
            color: color.withOpacity(0.1),
            borderRadius: BorderRadius.circular(30),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              AnimatedBuilder(
                animation: _breathingAnimation,
                builder: (context, _) {
                  return Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      color: color,
                      shape: BoxShape.circle,
                      boxShadow: isOnline
                          ? [
                              BoxShadow(
                                color: color.withOpacity(0.5 * _breathingAnimation.value),
                                blurRadius: 6,
                                spreadRadius: 1,
                              ),
                            ]
                          : [],
                    ),
                  );
                },
              ),
              const SizedBox(width: 8),
              Text(
                label,
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w700,
                  letterSpacing: 0.5,
                  color: color,
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _buildRobotCard() {
    final isDark = context.isDark;
    final pose = _latestFastState!.pose;

    return Container(
      padding: const EdgeInsets.all(22),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(24),
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
            width: 72,
            height: 72,
            decoration: BoxDecoration(
              gradient: const LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [AppColors.primary, AppColors.secondary],
              ),
              borderRadius: BorderRadius.circular(22),
              boxShadow: [
                BoxShadow(
                  color: AppColors.primary.withOpacity(0.3),
                  blurRadius: 16,
                  offset: const Offset(0, 6),
                ),
              ],
            ),
            child: const Icon(Icons.smart_toy, size: 36, color: Colors.white),
          ),
          const SizedBox(height: 16),
          Text(
            context.watch<RobotProfileProvider>().current.name,
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.w700,
              color: isDark ? Colors.white : Colors.black87,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            'Mode: ${_latestSlowState?.currentMode ?? "IDLE"}',
            style: TextStyle(
              fontSize: 12,
              color: context.subtitleColor,
              letterSpacing: 0.5,
            ),
          ),
          const SizedBox(height: 20),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildPoseValue('X', pose.position.x.toStringAsFixed(2), 'm'),
              _buildDivider(),
              _buildPoseValue('Y', pose.position.y.toStringAsFixed(2), 'm'),
              _buildDivider(),
              _buildPoseValue('YAW', _latestFastState!.rpyDeg.z.toStringAsFixed(1), '°'),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildPoseValue(String label, String value, String unit) {
    final isDark = context.isDark;
    return Column(
      children: [
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            fontWeight: FontWeight.w600,
            color: context.subtitleColor,
            letterSpacing: 1.0,
          ),
        ),
        const SizedBox(height: 4),
        RichText(
          text: TextSpan(
            children: [
              TextSpan(
                text: value,
                style: TextStyle(
                  fontSize: 22,
                  fontWeight: FontWeight.w700,
                  color: isDark ? Colors.white : Colors.black87,
                  letterSpacing: -0.5,
                ),
              ),
              TextSpan(
                text: ' $unit',
                style: TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w500,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildDivider() {
    return Container(
      width: 1,
      height: 36,
      color: context.dividerColor,
    );
  }

  Widget _buildMetricCard({
    required IconData icon,
    required Color iconColor,
    required String label,
    required String value,
    required String unit,
    required double progress,
    required Color progressColor,
  }) {
    final isDark = context.isDark;

    return Container(
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 34,
                height: 34,
                decoration: BoxDecoration(
                  color: iconColor.withOpacity(isDark ? 0.18 : 0.1),
                  borderRadius: BorderRadius.circular(11),
                ),
                child: Icon(icon, size: 18, color: iconColor),
              ),
              const Spacer(),
              Text(
                label,
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 14),
          RichText(
            text: TextSpan(
              children: [
                TextSpan(
                  text: value,
                  style: TextStyle(
                    fontSize: 26,
                    fontWeight: FontWeight.w700,
                    color: isDark ? Colors.white : Colors.black87,
                    letterSpacing: -0.5,
                  ),
                ),
                TextSpan(
                  text: ' $unit',
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w500,
                    color: context.subtitleColor,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 12),
          ClipRRect(
            borderRadius: BorderRadius.circular(6),
            child: TweenAnimationBuilder<double>(
              tween: Tween(begin: 0, end: progress.clamp(0.0, 1.0)),
              duration: const Duration(milliseconds: 500),
              curve: Curves.easeOutCubic,
              builder: (context, value, _) {
                return LinearProgressIndicator(
                  value: value,
                  minHeight: 5,
                  backgroundColor: isDark
                      ? Colors.white.withOpacity(0.06)
                      : Colors.black.withOpacity(0.05),
                  valueColor: AlwaysStoppedAnimation<Color>(progressColor),
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMotionCard() {
    final isDark = context.isDark;
    final linear = _latestFastState!.velocity.linear.x;
    final angular = _latestFastState!.velocity.angular.z;

    return Container(
      padding: const EdgeInsets.all(22),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 34,
                height: 34,
                decoration: BoxDecoration(
                  color: AppColors.accent.withOpacity(isDark ? 0.18 : 0.1),
                  borderRadius: BorderRadius.circular(11),
                ),
                child: const Icon(Icons.speed, size: 18, color: AppColors.accent),
              ),
              const SizedBox(width: 10),
              Text(
                'MOTION',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            children: [
              Expanded(
                child: _buildMotionValue(
                  Icons.arrow_forward, 'Linear',
                  linear.toStringAsFixed(2), 'm/s', AppColors.primary,
                ),
              ),
              Container(width: 1, height: 48, color: context.dividerColor),
              Expanded(
                child: _buildMotionValue(
                  Icons.rotate_right, 'Angular',
                  angular.toStringAsFixed(2), 'rad/s', AppColors.accent,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildMotionValue(
      IconData icon, String label, String value, String unit, Color color) {
    final isDark = context.isDark;
    return Column(
      children: [
        Icon(icon, size: 20, color: color.withOpacity(0.6)),
        const SizedBox(height: 8),
        RichText(
          textAlign: TextAlign.center,
          text: TextSpan(
            children: [
              TextSpan(
                text: value,
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.w700,
                  color: isDark ? Colors.white : Colors.black87,
                ),
              ),
              TextSpan(
                text: '\n$unit',
                style: TextStyle(
                  fontSize: 11,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ),
        ),
        const SizedBox(height: 4),
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            fontWeight: FontWeight.w600,
            color: context.subtitleColor,
            letterSpacing: 0.8,
          ),
        ),
      ],
    );
  }

  Widget _buildTopicRatesCard() {
    final isDark = context.isDark;
    final rates = _latestSlowState!.topicRates;

    return Container(
      padding: const EdgeInsets.all(22),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 34,
                height: 34,
                decoration: BoxDecoration(
                  color: AppColors.info.withOpacity(isDark ? 0.18 : 0.1),
                  borderRadius: BorderRadius.circular(11),
                ),
                child: const Icon(Icons.wifi, size: 18, color: AppColors.info),
              ),
              const SizedBox(width: 10),
              Text(
                'TOPIC RATES',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildRateChip('Odom', rates.odomHz, AppColors.primary),
              _buildRateChip('Lidar', rates.lidarHz, AppColors.success),
              _buildRateChip('Map', rates.terrainMapHz, AppColors.warning),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildRateChip(String label, double hz, Color color) {
    final isDark = context.isDark;
    return Column(
      children: [
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 7),
          decoration: BoxDecoration(
            color: color.withOpacity(isDark ? 0.15 : 0.08),
            borderRadius: BorderRadius.circular(22),
          ),
          child: Text(
            '${hz.toStringAsFixed(1)} Hz',
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w700,
              color: color,
            ),
          ),
        ),
        const SizedBox(height: 6),
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

  // ═══════════════════════════════════════════════
  //  Dog Direct Sensor Cards
  // ═══════════════════════════════════════════════

  Widget _buildDogImuCard(DogDirectClient dogClient) {
    final isDark = context.isDark;
    final rpy = dogClient.rpyDegrees;
    final gyro = dogClient.imuGyroscope;

    return Container(
      padding: const EdgeInsets.all(22),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 34,
                height: 34,
                decoration: BoxDecoration(
                  color: AppColors.warning.withOpacity(isDark ? 0.18 : 0.1),
                  borderRadius: BorderRadius.circular(11),
                ),
                child: const Icon(Icons.explore, size: 18,
                    color: AppColors.warning),
              ),
              const SizedBox(width: 10),
              Text(
                'DOG IMU',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              if (dogClient.isConnected)
                Container(
                  width: 8,
                  height: 8,
                  decoration: const BoxDecoration(
                    color: AppColors.success,
                    shape: BoxShape.circle,
                  ),
                ),
            ],
          ),
          const SizedBox(height: 16),
          // Roll / Pitch / Yaw
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildImuValue('ROLL', rpy?[0], '°'),
              Container(width: 1, height: 36, color: context.dividerColor),
              _buildImuValue('PITCH', rpy?[1], '°'),
              Container(width: 1, height: 36, color: context.dividerColor),
              _buildImuValue('YAW', rpy?[2], '°'),
            ],
          ),
          const SizedBox(height: 16),
          // Gyroscope
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: isDark
                  ? Colors.white.withOpacity(0.04)
                  : Colors.black.withOpacity(0.03),
              borderRadius: BorderRadius.circular(14),
            ),
            child: Row(
              children: [
                Icon(Icons.rotate_right, size: 14,
                    color: context.subtitleColor),
                const SizedBox(width: 8),
                Text(
                  'GYRO',
                  style: TextStyle(
                    fontSize: 10,
                    fontWeight: FontWeight.w600,
                    color: context.subtitleColor,
                    letterSpacing: 0.8,
                  ),
                ),
                const Spacer(),
                Text(
                  gyro != null
                      ? 'X: ${gyro[0].toStringAsFixed(2)}  Y: ${gyro[1].toStringAsFixed(2)}  Z: ${gyro[2].toStringAsFixed(2)}'
                      : '-- -- --',
                  style: TextStyle(
                    fontSize: 12,
                    fontFamily: 'monospace',
                    color: isDark ? Colors.white70 : Colors.black87,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildImuValue(String label, double? value, String unit) {
    final isDark = context.isDark;
    return Column(
      children: [
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            fontWeight: FontWeight.w600,
            color: context.subtitleColor,
            letterSpacing: 1.0,
          ),
        ),
        const SizedBox(height: 4),
        RichText(
          text: TextSpan(
            children: [
              TextSpan(
                text: value?.toStringAsFixed(1) ?? '--',
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.w700,
                  color: isDark ? Colors.white : Colors.black87,
                  letterSpacing: -0.5,
                ),
              ),
              TextSpan(
                text: unit,
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w500,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildDogJointCard(DogDirectClient dogClient) {
    final isDark = context.isDark;
    final positions = dogClient.jointPositions;
    final torques = dogClient.jointTorques;

    // Joint names (first 12 = leg joints)
    const legNames = ['FR', 'FL', 'RR', 'RL'];
    const jointNames = ['Hip', 'Thigh', 'Calf'];

    return Container(
      padding: const EdgeInsets.all(22),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 34,
                height: 34,
                decoration: BoxDecoration(
                  color: AppColors.secondary.withOpacity(isDark ? 0.18 : 0.1),
                  borderRadius: BorderRadius.circular(11),
                ),
                child: const Icon(Icons.precision_manufacturing, size: 18,
                    color: AppColors.secondary),
              ),
              const SizedBox(width: 10),
              Text(
                'DOG JOINTS',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              Text(
                positions != null ? '${positions.length} joints' : 'N/A',
                style: TextStyle(
                  fontSize: 11,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          // 4 legs x 3 joints grid
          if (positions != null && positions.length >= 12)
            Table(
              defaultVerticalAlignment: TableCellVerticalAlignment.middle,
              columnWidths: const {
                0: FixedColumnWidth(36),
                1: FlexColumnWidth(1),
                2: FlexColumnWidth(1),
                3: FlexColumnWidth(1),
              },
              children: [
                // Header
                TableRow(
                  children: [
                    const SizedBox(),
                    ...jointNames.map((name) => Padding(
                          padding: const EdgeInsets.only(bottom: 8),
                          child: Text(
                            name,
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              fontSize: 10,
                              fontWeight: FontWeight.w600,
                              color: context.subtitleColor,
                              letterSpacing: 0.5,
                            ),
                          ),
                        )),
                  ],
                ),
                // Data rows
                for (var leg = 0; leg < 4; leg++)
                  TableRow(
                    decoration: BoxDecoration(
                      color: leg.isEven
                          ? (isDark
                              ? Colors.white.withOpacity(0.02)
                              : Colors.black.withOpacity(0.02))
                          : Colors.transparent,
                      borderRadius: BorderRadius.circular(8),
                    ),
                    children: [
                      Padding(
                        padding: const EdgeInsets.symmetric(vertical: 6),
                        child: Text(
                          legNames[leg],
                          style: TextStyle(
                            fontSize: 11,
                            fontWeight: FontWeight.w700,
                            color: isDark ? Colors.white70 : Colors.black87,
                          ),
                        ),
                      ),
                      for (var joint = 0; joint < 3; joint++)
                        Padding(
                          padding: const EdgeInsets.symmetric(vertical: 6),
                          child: Text(
                            _radToDeg(positions[leg * 3 + joint])
                                .toStringAsFixed(1),
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              fontSize: 13,
                              fontWeight: FontWeight.w600,
                              fontFamily: 'monospace',
                              color: isDark ? Colors.white : Colors.black87,
                            ),
                          ),
                        ),
                    ],
                  ),
              ],
            )
          else
            Center(
              child: Padding(
                padding: const EdgeInsets.all(20),
                child: Text(
                  '等待关节数据...',
                  style: TextStyle(color: context.subtitleColor),
                ),
              ),
            ),
          // Torque summary
          if (torques != null && torques.length >= 12) ...[
            const SizedBox(height: 12),
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: isDark
                    ? Colors.white.withOpacity(0.04)
                    : Colors.black.withOpacity(0.03),
                borderRadius: BorderRadius.circular(14),
              ),
              child: Row(
                children: [
                  Icon(Icons.bolt, size: 14, color: context.subtitleColor),
                  const SizedBox(width: 8),
                  Text(
                    'TORQUE',
                    style: TextStyle(
                      fontSize: 10,
                      fontWeight: FontWeight.w600,
                      color: context.subtitleColor,
                      letterSpacing: 0.8,
                    ),
                  ),
                  const Spacer(),
                  Text(
                    'Max: ${torques.take(12).map((t) => t.abs()).reduce(math.max).toStringAsFixed(2)} N·m',
                    style: TextStyle(
                      fontSize: 12,
                      fontFamily: 'monospace',
                      color: isDark ? Colors.white70 : Colors.black87,
                    ),
                  ),
                ],
              ),
            ),
          ],
        ],
      ),
    );
  }

  double _radToDeg(double rad) => rad * 180 / math.pi;
}
