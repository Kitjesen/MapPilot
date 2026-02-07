import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'package:provider/provider.dart';
import 'package:shimmer/shimmer.dart';
import '../services/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import '../theme/app_theme.dart';
import '../widgets/glass_widgets.dart';
import '../theme/app_theme.dart';

class StatusScreen extends StatefulWidget {
  const StatusScreen({super.key});

  @override
  State<StatusScreen> createState() => _StatusScreenState();
}

class _StatusScreenState extends State<StatusScreen>
    with AutomaticKeepAliveClientMixin, TickerProviderStateMixin {
  FastState? _latestFastState;
  SlowState? _latestSlowState;
  StreamSubscription<FastState>? _fastSub;
  StreamSubscription<SlowState>? _slowSub;

  late AnimationController _breathingController;
  late Animation<double> _breathingAnimation;

  // Stagger entrance
  late AnimationController _staggerController;
  late List<Animation<double>> _staggerSlides;
  late List<Animation<double>> _staggerFades;

  DateTime _lastUiUpdate = DateTime(2000);
  static const _uiThrottleInterval = Duration(milliseconds: 100);

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

    // Stagger entrance for 5 card groups
    _staggerController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1200),
    );
    _staggerSlides = List.generate(5, (i) {
      final start = (i * 0.12).clamp(0.0, 0.7);
      final end = (start + 0.4).clamp(0.0, 1.0);
      return Tween<double>(begin: 40, end: 0).animate(CurvedAnimation(
        parent: _staggerController,
        curve: Interval(start, end, curve: Curves.easeOutCubic),
      ));
    });
    _staggerFades = List.generate(5, (i) {
      final start = (i * 0.12).clamp(0.0, 0.7);
      final end = (start + 0.35).clamp(0.0, 1.0);
      return Tween<double>(begin: 0, end: 1).animate(CurvedAnimation(
        parent: _staggerController,
        curve: Interval(start, end, curve: Curves.easeOut),
      ));
    });

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _subscribeStreams();
      _staggerController.forward();
    });
  }

  void _subscribeStreams() {
    final provider = context.read<RobotConnectionProvider>();
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
    _breathingController.dispose();
    _staggerController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    super.build(context);
    final isDark = context.isDark;

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
          Padding(
            padding: const EdgeInsets.only(right: 16.0),
            child: _buildStatusBadge(),
          ),
        ],
      ),
      body: _latestFastState == null
          ? _buildLoadingSkeleton()
          : RefreshIndicator(
              color: AppColors.lime,
              backgroundColor: AppColors.surface,
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
                  ),
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
            child: const Icon(Icons.smart_toy_rounded,
                size: 36, color: AppColors.lime),
          ),
          const SizedBox(height: 16),
          const Text(
            '大算机器人',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.w700,
              color: isDark ? Colors.white : Colors.black87,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            'Mode: ${_latestSlowState?.currentMode ?? "IDLE"}',
            style: const TextStyle(
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
          style: const TextStyle(
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
            ),
          ]),
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
                style: const TextStyle(
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
              ),
            ]),
          ),
          const SizedBox(height: 12),
          ClipRRect(
            borderRadius: BorderRadius.circular(6),
            child: TweenAnimationBuilder<double>(
              tween: Tween(begin: 0, end: progress.clamp(0.0, 1.0)),
              duration: const Duration(milliseconds: 600),
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
              const Text(
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
        Icon(icon, size: 20, color: color.withOpacity(0.7)),
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
            ),
          ]),
        ),
        const SizedBox(height: 4),
        Text(
          label,
          style: const TextStyle(
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
              const Text(
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
          style: const TextStyle(
            fontSize: 11,
            color: context.subtitleColor,
          ),
        ),
      ],
    );
  }
}
