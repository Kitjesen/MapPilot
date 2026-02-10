import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'package:provider/provider.dart';
import 'package:shimmer/shimmer.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
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
          if (isDogConnected)
            Padding(
              padding: const EdgeInsets.only(right: 8.0),
              child: Text(
                'DOG',
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
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
                    context.screenPadding, MediaQuery.of(context).padding.top + 16, context.screenPadding, 100),
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
                    const SizedBox(height: 12),
                    // Memory metric
                    Row(
                      children: [
                        Expanded(
                          child: RepaintBoundary(
                            child: _buildMetricCard(
                              icon: Icons.sd_storage_outlined,
                              iconColor: AppColors.secondary,
                              label: 'MEMORY',
                              value: (_latestSlowState?.resources.memPercent ?? 0)
                                  .toStringAsFixed(0),
                              unit: '%',
                              progress: (_latestSlowState?.resources.memPercent ?? 0) / 100,
                              progressColor: AppColors.secondary,
                            ),
                          ),
                        ),
                        const SizedBox(width: 12),
                        const Expanded(child: SizedBox()), // Placeholder for symmetry
                      ],
                    ),
                    const SizedBox(height: 14),
                    RepaintBoundary(child: _buildMotionCard()),
                    if (_latestSlowState != null) ...[
                      const SizedBox(height: 14),
                      RepaintBoundary(child: _buildTopicRatesCard()),
                      // ─── Network Quality ───
                      if (_latestSlowState!.hasNetwork()) ...[
                        const SizedBox(height: 14),
                        RepaintBoundary(child: _buildNetworkQualityCard()),
                      ],
                      // ─── System Health ───
                      if (_latestSlowState!.hasHealth()) ...[
                        const SizedBox(height: 14),
                        RepaintBoundary(child: _buildHealthCard()),
                      ],
                      // ─── Navigation Status ───
                      if (_latestSlowState!.hasNavigation()) ...[
                        const SizedBox(height: 14),
                        RepaintBoundary(child: _buildNavigationCard()),
                      ],
                      // ─── Geofence ───
                      if (_latestSlowState!.hasGeofence()) ...[
                        const SizedBox(height: 14),
                        RepaintBoundary(child: _buildGeofenceCard()),
                      ],
                    ],
                    // ─── FastState IMU (Nav Board) ───
                    if (_latestFastState != null) ...[
                      const SizedBox(height: 14),
                      RepaintBoundary(child: _buildNavImuCard()),
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
                    // ─── FastState Joint Angles Fallback (when Dog not connected) ───
                    if (!(isDogConnected && dogClient != null) &&
                        _latestFastState != null &&
                        _latestFastState!.jointAngles.isNotEmpty) ...[
                      const SizedBox(height: 14),
                      RepaintBoundary(child: _buildNavJointCard()),
                    ],
                  ],
                ),
              ),
            ),
    );
  }

  Widget _buildLoadingSkeleton() {
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
      baseColor: isDark ? Colors.white.withValues(alpha: 0.06) : const Color(0xFFEEEBFF),
      highlightColor: isDark ? Colors.white.withValues(alpha: 0.12) : const Color(0xFFF8F6FF),
      child: Container(
        height: height,
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
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
        final label = isOnline
            ? 'ONLINE'
            : isReconnecting
                ? 'RECONNECTING'
                : 'OFFLINE';

        return Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              width: 6,
              height: 6,
              decoration: BoxDecoration(
                color: isOnline ? AppColors.success : (isReconnecting ? AppColors.warning : AppColors.error),
                shape: BoxShape.circle,
              ),
            ),
            const SizedBox(width: 6),
            Text(
              label,
              style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w500,
                color: context.subtitleColor,
              ),
            ),
          ],
        );
      },
    );
  }

  Widget _buildRobotCard() {
    final pose = _latestFastState!.pose;
    final profile = context.watch<RobotProfileProvider>().current;

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        children: [
          // Simplified: just name + mode, no big gradient icon
          Text(
            profile.name,
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.w600,
              color: context.titleColor,
            ),
          ),
          const SizedBox(height: 6),
          Text(
            _latestSlowState?.currentMode ?? 'IDLE',
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.w500,
              color: context.subtitleColor,
              letterSpacing: 0.8,
            ),
          ),
          const SizedBox(height: AppSpacing.lg),
          Divider(height: 1, color: context.dividerColor),
          const SizedBox(height: AppSpacing.lg),
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
                  color: context.titleColor,
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
    final clampedProgress = progress.clamp(0.0, 1.0);

    return Container(
      padding: const EdgeInsets.all(AppSpacing.lg),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(icon, size: 16, color: context.subtitleColor),
              const SizedBox(width: 6),
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
          const SizedBox(height: AppSpacing.md),
          RichText(
            text: TextSpan(
              children: [
                TextSpan(
                  text: value,
                  style: TextStyle(
                    fontSize: 24,
                    fontWeight: FontWeight.w700,
                    color: context.titleColor,
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
          const SizedBox(height: AppSpacing.md),
          // Clean progress bar — no glow
          ClipRRect(
            borderRadius: BorderRadius.circular(2),
            child: TweenAnimationBuilder<double>(
              tween: Tween(begin: 0, end: clampedProgress),
              duration: AppDurations.slow,
              curve: Curves.easeOutCubic,
              builder: (context, animValue, _) {
                return Stack(
                  children: [
                    Container(
                      height: 4,
                      decoration: BoxDecoration(
                        color: progressColor.withValues(alpha: 0.1),
                        borderRadius: BorderRadius.circular(2),
                      ),
                    ),
                    FractionallySizedBox(
                      widthFactor: animValue,
                      child: Container(
                        height: 4,
                        decoration: BoxDecoration(
                          color: progressColor,
                          borderRadius: BorderRadius.circular(2),
                        ),
                      ),
                    ),
                  ],
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMotionCard() {
    final linear = _latestFastState!.velocity.linear.x;
    final angular = _latestFastState!.velocity.angular.z;

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'MOTION',
            style: TextStyle(
              fontSize: 10,
              fontWeight: FontWeight.w600,
              color: context.subtitleColor,
              letterSpacing: 1.0,
            ),
          ),
          const SizedBox(height: 14),
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
    return Column(
      children: [
        Icon(icon, size: 18, color: context.subtitleColor),
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
                  color: context.titleColor,
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
    final rates = _latestSlowState!.topicRates;

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'TOPIC RATES',
            style: TextStyle(
              fontSize: 10,
              fontWeight: FontWeight.w600,
              color: context.subtitleColor,
              letterSpacing: 1.0,
            ),
          ),
          const SizedBox(height: 14),
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
    return Column(
      children: [
        Text(
          '${hz.toStringAsFixed(1)}',
          style: TextStyle(
            fontSize: 18,
            fontWeight: FontWeight.w700,
            color: context.titleColor,
          ),
        ),
        const SizedBox(height: 2),
        Text(
          'Hz',
          style: TextStyle(fontSize: 11, color: context.subtitleColor),
        ),
        const SizedBox(height: 4),
        Text(
          label,
          style: TextStyle(
            fontSize: 11,
            fontWeight: FontWeight.w500,
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
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
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
                  width: 6,
                  height: 6,
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
                  ? Colors.white.withValues(alpha: 0.04)
                  : Colors.black.withValues(alpha: 0.03),
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
                  color: context.titleColor,
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

  // ═══════════════════════════════════════════════
  //  Nav Board IMU Card (from FastState stream)
  // ═══════════════════════════════════════════════

  Widget _buildNavImuCard() {
    final isDark = context.isDark;
    final state = _latestFastState;
    if (state == null) return const SizedBox.shrink();

    // RPY from FastState
    final hasRpy = state.hasRpyDeg();
    final roll = hasRpy ? state.rpyDeg.x : null;
    final pitch = hasRpy ? state.rpyDeg.y : null;
    final yaw = hasRpy ? state.rpyDeg.z : null;

    // IMU accelerometer
    final hasAccel = state.hasLinearAcceleration();
    final ax = hasAccel ? state.linearAcceleration.x : null;
    final ay = hasAccel ? state.linearAcceleration.y : null;
    final az = hasAccel ? state.linearAcceleration.z : null;

    // IMU gyroscope
    final hasGyro = state.hasAngularVelocity();
    final gx = hasGyro ? state.angularVelocity.x : null;
    final gy = hasGyro ? state.angularVelocity.y : null;
    final gz = hasGyro ? state.angularVelocity.z : null;

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'NAV IMU',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              Container(
                width: 6,
                height: 6,
                decoration: const BoxDecoration(
                  color: AppColors.primary,
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
              _buildImuValue('ROLL', roll, '°'),
              Container(width: 1, height: 36, color: context.dividerColor),
              _buildImuValue('PITCH', pitch, '°'),
              Container(width: 1, height: 36, color: context.dividerColor),
              _buildImuValue('YAW', yaw, '°'),
            ],
          ),
          const SizedBox(height: 12),
          // Accelerometer
          _buildImuRow(
            icon: Icons.speed,
            label: 'ACCEL',
            x: ax, y: ay, z: az,
            unit: 'm/s²',
            isDark: isDark,
          ),
          const SizedBox(height: 8),
          // Gyroscope
          _buildImuRow(
            icon: Icons.rotate_right,
            label: 'GYRO',
            x: gx, y: gy, z: gz,
            unit: 'rad/s',
            isDark: isDark,
          ),
        ],
      ),
    );
  }

  Widget _buildImuRow({
    required IconData icon,
    required String label,
    required double? x,
    required double? y,
    required double? z,
    required String unit,
    required bool isDark,
  }) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: isDark
            ? Colors.white.withValues(alpha: 0.04)
            : Colors.black.withValues(alpha: 0.03),
        borderRadius: BorderRadius.circular(14),
      ),
      child: Row(
        children: [
          Icon(icon, size: 14, color: context.subtitleColor),
          const SizedBox(width: 8),
          Text(
            label,
            style: TextStyle(
              fontSize: 10,
              fontWeight: FontWeight.w600,
              color: context.subtitleColor,
              letterSpacing: 0.8,
            ),
          ),
          const Spacer(),
          Text(
            (x != null && y != null && z != null)
                ? 'X: ${x.toStringAsFixed(2)}  Y: ${y.toStringAsFixed(2)}  Z: ${z.toStringAsFixed(2)}'
                : '-- -- --',
            style: TextStyle(
              fontSize: 12,
              fontFamily: 'monospace',
              color: isDark ? Colors.white70 : Colors.black87,
            ),
          ),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════
  //  Nav Board Joint Card (FastState fallback)
  // ═══════════════════════════════════════════════

  Widget _buildNavJointCard() {
    final isDark = context.isDark;
    final state = _latestFastState;
    if (state == null || state.jointAngles.isEmpty) {
      return const SizedBox.shrink();
    }

    final angles = state.jointAngles;
    const legNames = ['FR', 'FL', 'RR', 'RL'];
    const jointNames = ['Hip', 'Thigh', 'Calf', 'Foot'];

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'NAV JOINT ANGLES',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              Text(
                '${angles.length} DOF',
                style: TextStyle(
                  fontSize: 10,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          // Joint angles table: 4 legs × 4 joints
          Table(
            columnWidths: const {
              0: FlexColumnWidth(1.2),
              1: FlexColumnWidth(1),
              2: FlexColumnWidth(1),
              3: FlexColumnWidth(1),
              4: FlexColumnWidth(1),
            },
            children: [
              // Header
              TableRow(
                children: [
                  const SizedBox(),
                  ...jointNames.map((j) => Padding(
                    padding: const EdgeInsets.only(bottom: 8),
                    child: Text(
                      j,
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
              for (int leg = 0; leg < 4 && (leg * 4 + 3) < angles.length; leg++)
                TableRow(
                  decoration: BoxDecoration(
                    color: leg.isEven
                        ? (isDark ? Colors.white.withValues(alpha: 0.02) : Colors.black.withValues(alpha: 0.02))
                        : null,
                  ),
                  children: [
                    Padding(
                      padding: const EdgeInsets.symmetric(vertical: 6),
                      child: Text(
                        legNames[leg],
                        style: TextStyle(
                          fontSize: 12,
                          fontWeight: FontWeight.w600,
                          color: context.titleColor,
                        ),
                      ),
                    ),
                    for (int j = 0; j < 4; j++)
                      Padding(
                        padding: const EdgeInsets.symmetric(vertical: 6),
                        child: Text(
                          _radToDeg(angles[leg * 4 + j]).toStringAsFixed(1),
                          textAlign: TextAlign.center,
                          style: TextStyle(
                            fontSize: 12,
                            fontFamily: 'monospace',
                            color: isDark ? Colors.white70 : Colors.black87,
                          ),
                        ),
                      ),
                  ],
                ),
            ],
          ),
        ],
      ),
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
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
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
                              ? Colors.white.withValues(alpha: 0.02)
                              : Colors.black.withValues(alpha: 0.02))
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
                    ? Colors.white.withValues(alpha: 0.04)
                    : Colors.black.withValues(alpha: 0.03),
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

  // ═══════════════════════════════════════════════
  //  Network Quality Card (from SlowState.network)
  // ═══════════════════════════════════════════════

  Widget _buildNetworkQualityCard() {
    final net = _latestSlowState!.network;
    final rtt = net.rttMs;
    final loss = net.packetLoss;
    final jitter = net.jitterMs;
    final bw = net.bandwidthKbps;
    final signal = net.signalStrength;

    Color rttColor() {
      if (rtt < 50) return AppColors.success;
      if (rtt < 150) return AppColors.warning;
      return AppColors.error;
    }

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'NETWORK',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              Container(
                width: 8,
                height: 8,
                decoration: BoxDecoration(
                  color: rttColor(),
                  shape: BoxShape.circle,
                ),
              ),
            ],
          ),
          const SizedBox(height: 14),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildNetStat('RTT', '${rtt.toStringAsFixed(0)}', 'ms', rttColor()),
              _buildNetStat('丢包', '${loss.toStringAsFixed(1)}', '%',
                  loss < 1 ? AppColors.success : loss < 5 ? AppColors.warning : AppColors.error),
              _buildNetStat('抖动', '${jitter.toStringAsFixed(0)}', 'ms', context.subtitleColor),
              _buildNetStat('带宽', bw > 1000 ? '${(bw / 1000).toStringAsFixed(1)}' : '$bw',
                  bw > 1000 ? 'Mbps' : 'Kbps', context.subtitleColor),
            ],
          ),
          if (signal > 0) ...[
            const SizedBox(height: 12),
            Row(
              children: [
                Icon(Icons.signal_cellular_alt, size: 14, color: context.subtitleColor),
                const SizedBox(width: 6),
                Text('信号强度: $signal dBm',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor)),
              ],
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildNetStat(String label, String value, String unit, Color color) {
    return Column(
      children: [
        Text(value,
            style: TextStyle(
                fontSize: 18, fontWeight: FontWeight.w700, color: color)),
        Text(unit,
            style: TextStyle(fontSize: 10, color: context.subtitleColor)),
        const SizedBox(height: 4),
        Text(label,
            style: TextStyle(
                fontSize: 10,
                fontWeight: FontWeight.w500,
                color: context.subtitleColor)),
      ],
    );
  }

  // ═══════════════════════════════════════════════
  //  System Health Card (from SlowState.health)
  // ═══════════════════════════════════════════════

  Widget _buildHealthCard() {
    final health = _latestSlowState!.health;
    final isDark = context.isDark;

    Color levelColor(String level) {
      switch (level.toUpperCase()) {
        case 'OK':
          return AppColors.success;
        case 'DEGRADED':
          return AppColors.warning;
        case 'CRITICAL':
        case 'FAULT':
          return AppColors.error;
        default:
          return context.subtitleColor;
      }
    }

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'HEALTH',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                decoration: BoxDecoration(
                  color: levelColor(health.overallLevel).withValues(alpha: 0.15),
                  borderRadius: BorderRadius.circular(6),
                ),
                child: Text(
                  health.overallLevel.isNotEmpty ? health.overallLevel : 'N/A',
                  style: TextStyle(
                    fontSize: 11,
                    fontWeight: FontWeight.w600,
                    color: levelColor(health.overallLevel),
                  ),
                ),
              ),
            ],
          ),
          if (health.localizationScore > 0) ...[
            const SizedBox(height: 10),
            Row(
              children: [
                Icon(Icons.my_location, size: 14, color: context.subtitleColor),
                const SizedBox(width: 6),
                Text('定位置信度: ${(health.localizationScore * 100).toStringAsFixed(0)}%',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor)),
              ],
            ),
          ],
          if (health.subsystems.isNotEmpty) ...[
            const SizedBox(height: 12),
            Wrap(
              spacing: 6,
              runSpacing: 6,
              children: health.subsystems.map((sub) {
                final color = levelColor(sub.level);
                return Container(
                  padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 5),
                  decoration: BoxDecoration(
                    color: color.withValues(alpha: isDark ? 0.12 : 0.08),
                    borderRadius: BorderRadius.circular(6),
                    border: Border.all(color: color.withValues(alpha: 0.2)),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Container(
                        width: 6, height: 6,
                        decoration: BoxDecoration(color: color, shape: BoxShape.circle),
                      ),
                      const SizedBox(width: 4),
                      Text(sub.name,
                          style: TextStyle(fontSize: 11, fontWeight: FontWeight.w500, color: color)),
                      if (sub.actualHz > 0) ...[
                        const SizedBox(width: 4),
                        Text('${sub.actualHz.toStringAsFixed(0)}Hz',
                            style: TextStyle(fontSize: 10, color: color.withValues(alpha: 0.7))),
                      ],
                    ],
                  ),
                );
              }).toList(),
            ),
          ],
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════
  //  Navigation Status Card (from SlowState.navigation)
  // ═══════════════════════════════════════════════

  Widget _buildNavigationCard() {
    final nav = _latestSlowState!.navigation;

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'NAVIGATION',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              Icon(
                nav.localizationValid ? Icons.check_circle : Icons.warning_amber,
                size: 14,
                color: nav.localizationValid ? AppColors.success : AppColors.warning,
              ),
              const SizedBox(width: 4),
              Text(
                nav.localizationValid ? '定位有效' : '定位无效',
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w500,
                  color: nav.localizationValid ? AppColors.success : AppColors.warning,
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          // Planner status
          _buildNavRow(Icons.route, '规划器', nav.globalPlannerStatus.isNotEmpty
              ? nav.globalPlannerStatus
              : 'N/A'),
          if (nav.hasGlobalPath) ...[
            const SizedBox(height: 6),
            _buildNavRow(Icons.straighten, '路径长度',
                '${nav.globalPathLength.toStringAsFixed(1)} m'),
          ],
          if (nav.hasWaypoint) ...[
            const SizedBox(height: 6),
            _buildNavRow(Icons.place, '当前航点',
                '(${nav.currentWaypoint.x.toStringAsFixed(1)}, ${nav.currentWaypoint.y.toStringAsFixed(1)})'),
          ],
          if (nav.slowDownLevel > 0) ...[
            const SizedBox(height: 6),
            _buildNavRow(Icons.speed, '减速等级', '${nav.slowDownLevel}'),
          ],
        ],
      ),
    );
  }

  Widget _buildNavRow(IconData icon, String label, String value) {
    return Row(
      children: [
        Icon(icon, size: 14, color: context.subtitleColor),
        const SizedBox(width: 8),
        Text('$label: ', style: TextStyle(fontSize: 12, color: context.subtitleColor)),
        Expanded(
          child: Text(value,
              style: TextStyle(fontSize: 12, fontWeight: FontWeight.w600,
                  color: context.titleColor),
              textAlign: TextAlign.end),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════
  //  Geofence Status Card (from SlowState.geofence)
  // ═══════════════════════════════════════════════

  Widget _buildGeofenceCard() {
    final geo = _latestSlowState!.geofence;

    Color stateColor() {
      switch (geo.state.toUpperCase()) {
        case 'OK':
        case 'INSIDE':
          return AppColors.success;
        case 'WARNING':
        case 'NEAR_BOUNDARY':
          return AppColors.warning;
        case 'VIOLATION':
        case 'OUTSIDE':
          return AppColors.error;
        default:
          return context.subtitleColor;
      }
    }

    return Container(
      padding: const EdgeInsets.all(AppSpacing.xl),
      decoration: BoxDecoration(
        color: context.cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Text(
                'GEOFENCE',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 1.0,
                ),
              ),
              const Spacer(),
              if (!geo.hasFence)
                Text('未设置围栏',
                    style: TextStyle(fontSize: 11, color: context.subtitleColor))
              else ...[
                Container(
                  width: 8, height: 8,
                  decoration: BoxDecoration(color: stateColor(), shape: BoxShape.circle),
                ),
                const SizedBox(width: 6),
                Text(geo.state.isNotEmpty ? geo.state : 'N/A',
                    style: TextStyle(
                        fontSize: 11, fontWeight: FontWeight.w600, color: stateColor())),
              ],
            ],
          ),
          if (geo.hasFence && geo.marginDistance > 0) ...[
            const SizedBox(height: 10),
            Row(
              children: [
                Icon(Icons.fence, size: 14, color: context.subtitleColor),
                const SizedBox(width: 6),
                Text('边界距离: ${geo.marginDistance.toStringAsFixed(1)} m',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor)),
              ],
            ),
          ],
        ],
      ),
    );
  }
}
