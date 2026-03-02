import 'dart:math' show sqrt;
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/shared/widgets/glass_widgets.dart';
import 'package:flutter_monitor/features/control/camera_stream_widget.dart';
import 'package:flutter_monitor/features/control/webrtc_video_widget.dart';
import 'package:flutter_monitor/core/grpc/dog_direct_client.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen>
    with SingleTickerProviderStateMixin {
  // ─── Double-tap e-stop red flash ───
  late final AnimationController _flashController;
  late final Animation<double> _flashOpacity;

  // ─── Joystick deadzone (0% ~ 30%, default 10%) ───
  double _deadzone = 0.10;

  // ─── Max speed limit (0.1 ~ 2.0 m/s, default 1.0) ───
  double _maxSpeedLimit = 1.0;

  /// Apply deadzone: values within deadzone radius return 0, otherwise rescale.
  double _applyDeadzone(double value) {
    if (value.abs() <= _deadzone) return 0.0;
    final sign = value > 0 ? 1.0 : -1.0;
    return sign * (value.abs() - _deadzone) / (1.0 - _deadzone);
  }

  @override
  void initState() {
    super.initState();
    _flashController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 300),
    );
    _flashOpacity = Tween<double>(begin: 0.6, end: 0.0).animate(
      CurvedAnimation(parent: _flashController, curve: Curves.easeOut),
    );
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    // 隐藏系统状态栏提升沉浸感
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
  }

  @override
  void dispose() {
    _flashController.dispose();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    super.dispose();
  }

  // ─── Joystick callbacks ───

  void _onLeftJoystickChange(StickDragDetails details) {
    final gw = context.read<ControlGateway>();
    final dx = _applyDeadzone(details.x);
    final dy = _applyDeadzone(-details.y);
    if (gw.useDogDirect) {
      gw.sendVelocity(dy, dx, gw.angularZ);
    } else {
      if (!gw.hasLease) return;
      final profile = context.read<RobotProfileProvider>().current;
      final speedScale = _maxSpeedLimit / (profile.maxLinearSpeed > 0 ? profile.maxLinearSpeed : 1.0);
      gw.sendVelocity(
        dy * profile.maxLinearSpeed * speedScale,
        dx * profile.maxLinearSpeed * speedScale,
        gw.angularZ,
      );
    }
  }

  void _onRightJoystickChange(StickDragDetails details) {
    final gw = context.read<ControlGateway>();
    final dx = _applyDeadzone(-details.x);
    if (gw.useDogDirect) {
      gw.sendVelocity(gw.linearX, gw.linearY, dx);
    } else {
      if (!gw.hasLease) return;
      final profile = context.read<RobotProfileProvider>().current;
      gw.sendVelocity(gw.linearX, gw.linearY, dx * profile.maxAngularSpeed);
    }
  }

  // ─── Button handlers ───

  Future<void> _toggleLease() async {
    HapticFeedback.mediumImpact();
    final gw = context.read<ControlGateway>();
    final ok = await gw.toggleLease();
    if (!ok && mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(gw.statusMessage ?? context.read<LocaleProvider>().tr('无法获取控制权', 'Cannot acquire control')),
          behavior: SnackBarBehavior.floating,
        ),
      );
    }
  }

  Future<void> _setMode(RobotMode mode) async {
    HapticFeedback.lightImpact();
    final (ok, msg) = await context.read<ControlGateway>().setMode(mode);
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(msg),
          behavior: SnackBarBehavior.floating,
          duration: const Duration(seconds: 1),
        ),
      );
    }
  }

  Future<void> _emergencyStop() async {
    HapticFeedback.heavyImpact();
    await context.read<ControlGateway>().emergencyStop();
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(context.read<LocaleProvider>().tr('紧急停止已触发', 'Emergency stop triggered')),
          backgroundColor: Colors.red.shade700,
          behavior: SnackBarBehavior.floating,
        ),
      );
    }
  }

  Future<void> _dogEnable() async {
    HapticFeedback.mediumImpact();
    final (ok, err) = await context.read<ControlGateway>().dogEnable();
    if (mounted && !ok) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('${context.read<LocaleProvider>().tr('使能失败', 'Enable failed')}: ${err ?? ""}'), behavior: SnackBarBehavior.floating),
      );
    }
  }

  Future<void> _dogDisable() async {
    HapticFeedback.mediumImpact();
    await context.read<ControlGateway>().dogDisable();
  }

  Future<void> _dogStandUp() async {
    HapticFeedback.mediumImpact();
    final (ok, err) = await context.read<ControlGateway>().dogStandUp();
    if (mounted && !ok) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('${context.read<LocaleProvider>().tr('站立失败', 'Stand up failed')}: ${err ?? ""}'), behavior: SnackBarBehavior.floating),
      );
    }
  }

  Future<void> _dogSitDown() async {
    HapticFeedback.mediumImpact();
    await context.read<ControlGateway>().dogSitDown();
  }

  // ─── Return home ───

  Future<void> _returnHome() async {
    final locale = context.read<LocaleProvider>();
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(locale.tr('确认返航', 'Return Home')),
        content: Text(locale.tr(
          '确认返回起点？当前任务将取消',
          'Return to origin? Current task will be cancelled.',
        )),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: Text(locale.tr('取消', 'Cancel')),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(ctx, true),
            child: Text(locale.tr('确认', 'Confirm')),
          ),
        ],
      ),
    );
    if (confirmed != true || !mounted) return;
    HapticFeedback.mediumImpact();
    final taskGw = context.read<TaskGateway>();
    final ok = await taskGw.startSingleGoalNavigation(x: 0, y: 0, z: 0, yaw: 0, label: 'Home');
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok
            ? context.read<LocaleProvider>().tr('正在返航...', 'Returning home...')
            : context.read<LocaleProvider>().tr('返航指令发送失败', 'Failed to send return home')),
        behavior: SnackBarBehavior.floating,
      ));
    }
  }

  // ─── Double-tap e-stop with red flash ───

  Future<void> _doubleTapEmergencyStop() async {
    HapticFeedback.heavyImpact();
    _flashController.forward(from: 0.0);
    await context.read<ControlGateway>().emergencyStop();
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(context.read<LocaleProvider>().tr('紧急停止已触发', 'Emergency stop triggered')),
        backgroundColor: Colors.red.shade700,
        behavior: SnackBarBehavior.floating,
      ));
    }
  }

  @override
  Widget build(BuildContext context) {
    context.watch<LocaleProvider>(); // rebuild on locale change
    // 使用 select 代替 watch，避免摇杆速度更新触发全树重建。
    // 只在 hasLease / useDogDirect / dogClient 变化时重建。
    final gw = context.read<ControlGateway>();
    final hasLease = context.select<ControlGateway, bool>((g) => g.hasLease);
    final useDogDirect = context.select<ControlGateway, bool>((g) => g.useDogDirect);
    final dogClientRef = context.select<ControlGateway, DogDirectClient?>((g) => g.dogClient);
    final provider = context.watch<RobotConnectionProvider>();
    final client = provider.client;
    final isDogConnected = provider.isDogConnected;

    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        automaticallyImplyLeading: false,
        backgroundColor: Colors.transparent,
        elevation: 0,
        title: GlassCard(
          borderRadius: 30,
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
          blurSigma: 10,
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              IconButton(
                icon: Icon(Icons.arrow_back_ios_new,
                    size: 18,
                    color: context.isDark ? Colors.white : Colors.black87),
                onPressed: () => Navigator.of(context).pop(),
                padding: EdgeInsets.zero,
                constraints: const BoxConstraints(),
              ),
              const SizedBox(width: 12),
              Text(
                context.read<LocaleProvider>().tr('控制中心', 'Control Center'),
                style: TextStyle(
                  color: context.isDark ? Colors.white : Colors.black87,
                  fontSize: 16,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
        ),
        centerTitle: true,
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 12.0),
            child: GlassCard(
              borderRadius: 20,
              padding:
                  const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              blurSigma: 10,
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(Icons.videocam, size: 14,
                      color: context.isDark ? Colors.white54 : Colors.black54),
                  const SizedBox(width: 6),
                  Text(
                    'FPV',
                    style: TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w600,
                      color: context.isDark ? Colors.white54 : Colors.black54,
                    ),
                  ),
                ],
              ),
            ),
          ),
          // Dog Direct mode toggle
          if (isDogConnected)
            Padding(
              padding: const EdgeInsets.only(right: 8.0),
              child: GestureDetector(
                onTap: () {
                  HapticFeedback.selectionClick();
                  gw.toggleDogDirect();
                },
                child: GlassCard(
                  borderRadius: 20,
                  padding:
                      const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                  blurSigma: 10,
                  color: useDogDirect
                      ? AppColors.warning.withValues(alpha: 0.2)
                      : Colors.grey.withValues(alpha: 0.1),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(
                        Icons.pets,
                        size: 14,
                        color:
                            useDogDirect ? AppColors.warning : Colors.grey,
                      ),
                      const SizedBox(width: 6),
                      Text(
                        useDogDirect ? 'DOG' : 'NAV',
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          color: useDogDirect
                              ? AppColors.warning
                              : Colors.grey,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          Padding(
            padding: const EdgeInsets.only(right: 24.0),
            child: GestureDetector(
              onTap: _toggleLease,
              child: GlassCard(
                borderRadius: 20,
                padding:
                    const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                color: hasLease
                    ? AppColors.success.withValues(alpha: 0.2)
                    : Colors.grey.withValues(alpha: 0.2),
                child: Row(
                  children: [
                    Icon(
                      hasLease ? Icons.lock_open : Icons.lock,
                      size: 16,
                      color: hasLease ? AppColors.success : Colors.grey,
                    ),
                    const SizedBox(width: 8),
                    Text(
                      hasLease ? 'LEASE ACTIVE' : 'NO LEASE',
                      style: TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.bold,
                        color: hasLease ? AppColors.success : Colors.grey,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
      // ─── Return home FAB ───
      floatingActionButton: Padding(
        padding: const EdgeInsets.only(bottom: 80),
        child: FloatingActionButton(
          heroTag: 'returnHome',
          onPressed: _returnHome,
          backgroundColor: Colors.blue,
          child: const Icon(Icons.home_rounded, color: Colors.white),
        ),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.endFloat,
      body: GestureDetector(
        // ─── Double-tap anywhere for e-stop ───
        behavior: HitTestBehavior.translucent,
        onDoubleTap: _doubleTapEmergencyStop,
        child: Stack(
          children: [
            // Camera Background (FPV Style)
          Positioned.fill(
            child: client?.dataServiceClient != null
                ? WebRTCVideoWidget(
                    client: client!,
                    cameraId: 'front',
                    autoConnect: true,
                    fit: BoxFit.cover,
                    errorWidget: CameraStreamWidget(client: client),
                  )
                : client != null
                    ? CameraStreamWidget(client: client)
                    : Container(color: Colors.black87),
          ),

          // Dark gradient overlay
          Positioned.fill(
            child: Container(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [
                    Colors.black.withValues(alpha: 0.3),
                    Colors.black.withValues(alpha: 0.1),
                    Colors.black.withValues(alpha: 0.3),
                  ],
                ),
              ),
            ),
          ),

          // Main Layout
          Padding(
            padding: EdgeInsets.only(
                top: MediaQuery.of(context).padding.top + 60,
                bottom: 20,
                left: 40,
                right: 40),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                _buildJoystickSection(
                  label: 'TRANSLATION',
                  icon: Icons.open_with,
                  listener: _onLeftJoystickChange,
                  mode: JoystickMode.all,
                  speedBar: _buildSpeedBar(gw),
                ),

                // Center Controls
                Expanded(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 24.0),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        // Telemetry HUD
                        _buildTelemetryHud(provider, gw),
                        const SizedBox(height: 10),
                        // Mode Selection / Dog Control
                        if (useDogDirect)
                          _buildDogControlButtons(dogClientRef)
                        else
                          GlassCard(
                            padding: const EdgeInsets.all(4),
                            borderRadius: 22,
                            child: Row(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                _buildModeButton('IDLE',
                                    RobotMode.ROBOT_MODE_IDLE, Colors.grey),
                                _buildModeButton('MANUAL',
                                    RobotMode.ROBOT_MODE_MANUAL, Colors.blue),
                                _buildModeButton('TELEOP',
                                    RobotMode.ROBOT_MODE_TELEOP, Colors.orange),
                                _buildModeButton(
                                    'AUTO',
                                    RobotMode.ROBOT_MODE_AUTONOMOUS,
                                    Colors.purple),
                                _buildModeButton('MAP',
                                    RobotMode.ROBOT_MODE_MAPPING, Colors.teal),
                              ],
                            ),
                          ),
                        const Spacer(),
                        // E-Stop
                        GestureDetector(
                          onTap: _emergencyStop,
                          child: Container(
                            height: 72,
                            width: 72,
                            decoration: BoxDecoration(
                              shape: BoxShape.circle,
                              color: Colors.red.withValues(alpha: 0.7),
                              border: Border.all(
                                  color: Colors.white.withValues(alpha: 0.3),
                                  width: 1.5),
                            ),
                            child: const Center(
                              child: Text(
                                'STOP',
                                style: TextStyle(
                                  color: Colors.white,
                                  fontWeight: FontWeight.w600,
                                  fontSize: 16,
                                  letterSpacing: 0.5,
                                ),
                              ),
                            ),
                          ),
                        ),
                        const Spacer(),
                        // Status Text — 独立更新，不触发整棵树重建
                        ValueListenableBuilder<({double vx, double vy, double wz})>(
                          valueListenable: gw.velocityNotifier,
                          builder: (_, vel, __) => GlassCard(
                            padding: const EdgeInsets.symmetric(
                                horizontal: 16, vertical: 8),
                            child: Text(
                              useDogDirect
                                  ? 'X: ${vel.vx.toStringAsFixed(2)}  Y: ${vel.vy.toStringAsFixed(2)}  Z: ${vel.wz.toStringAsFixed(2)}'
                                  : 'Vx: ${vel.vx.toStringAsFixed(2)}  Vy: ${vel.vy.toStringAsFixed(2)}  Wz: ${vel.wz.toStringAsFixed(2)}',
                              style: TextStyle(
                                fontSize: 12,
                                fontFamily: 'monospace',
                                fontFeatures: const [
                                  FontFeature.tabularFigures()
                                ],
                                color: context.isDark
                                    ? Colors.white70
                                    : Colors.black.withValues(alpha: 0.6),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),

                _buildJoystickSection(
                  label: 'ROTATION',
                  icon: Icons.rotate_right,
                  listener: _onRightJoystickChange,
                  mode: JoystickMode.horizontal,
                ),
              ],
            ),
          ),

          // ─── Red flash overlay for double-tap e-stop ───
          AnimatedBuilder(
            animation: _flashOpacity,
            builder: (context, _) => _flashOpacity.value > 0
                ? Positioned.fill(
                    child: IgnorePointer(
                      child: Container(
                        color: Colors.red.withValues(alpha: _flashOpacity.value),
                      ),
                    ),
                  )
                : const SizedBox.shrink(),
          ),
        ],
        ),
      ),
    );
  }

  Widget _buildJoystickSection({
    required String label,
    required IconData icon,
    required void Function(StickDragDetails) listener,
    required JoystickMode mode,
    Widget? speedBar,
  }) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        GlassCard(
          borderRadius: 20,
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(icon, size: 14,
                  color: context.isDark ? Colors.white54 : Colors.black54),
              const SizedBox(width: 6),
              Text(
                label,
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: context.isDark ? Colors.white54 : Colors.black54,
                  letterSpacing: 0.5,
                ),
              ),
            ],
          ),
        ),
        if (speedBar != null) ...[const SizedBox(height: 8), speedBar],
        const SizedBox(height: 12),
        Joystick(
          mode: mode,
          listener: listener,
          base: Container(
            width: 180,
            height: 180,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Colors.white.withValues(alpha: 0.15),
              border:
                  Border.all(color: Colors.white.withValues(alpha: 0.3), width: 1),
            ),
            child: ClipOval(
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
                child: Container(
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: Colors.white.withValues(alpha: 0.1),
                  ),
                ),
              ),
            ),
          ),
          stick: Container(
            width: 56,
            height: 56,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Colors.white.withValues(alpha: 0.3),
              border:
                  Border.all(color: Colors.white.withValues(alpha: 0.5), width: 1),
            ),
            child: ClipOval(
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 8, sigmaY: 8),
                child: Container(
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: Colors.white.withValues(alpha: 0.15),
                  ),
                  child: Center(
                    child: Container(
                      width: 16,
                      height: 16,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: Colors.white.withValues(alpha: 0.6),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
        // ─── Deadzone slider (below each joystick) ───
        const SizedBox(height: 8),
        _buildDeadzoneSlider(),
      ],
    );
  }

  Widget _buildModeButton(String label, RobotMode mode, Color color) {
    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: () => _setMode(mode),
        borderRadius: BorderRadius.circular(8),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
          ),
          child: Text(
            label,
            style: TextStyle(
              fontSize: 13,
              fontWeight: FontWeight.w500,
              color: context.titleColor,
            ),
          ),
        ),
      ),
    );
  }

  // ─── Dog Direct Control Panel ───

  Widget _buildDogControlButtons(DogDirectClient? dogClient) {
    final isStanding = dogClient?.isStanding ?? false;
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        // Motor Enable / Disable
        GlassCard(
          padding: const EdgeInsets.all(4),
          borderRadius: 22,
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              _buildDogButton(
                label: 'ENABLE',
                icon: Icons.power,
                onTap: _dogEnable,
              ),
              _buildDogButton(
                label: 'DISABLE',
                icon: Icons.power_off,
                onTap: _dogDisable,
              ),
            ],
          ),
        ),
        const SizedBox(height: 8),
        // Stand / Sit
        GlassCard(
          padding: const EdgeInsets.all(4),
          borderRadius: 22,
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              _buildDogButton(
                label: 'STAND',
                icon: Icons.arrow_upward,
                onTap: _dogStandUp,
                isActive: isStanding,
              ),
              _buildDogButton(
                label: 'SIT',
                icon: Icons.arrow_downward,
                onTap: _dogSitDown,
                isActive: !isStanding,
              ),
            ],
          ),
        ),
      ],
    );
  }

  // ─── Telemetry HUD ───

  Widget _buildTelemetryHud(RobotConnectionProvider provider, ControlGateway gw) {
    return GlassCard(
      borderRadius: 12,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      blurSigma: 12,
      child: StreamBuilder<SlowState>(
        stream: provider.slowStateStream,
        initialData: provider.latestSlowState,
        builder: (context, snapshot) {
          if (snapshot.hasError) {
            return Row(mainAxisSize: MainAxisSize.min, children: [
              Icon(Icons.cloud_off_rounded, size: 13, color: Colors.white.withValues(alpha: 0.6)),
              const SizedBox(width: 6),
              Text('--', style: TextStyle(fontSize: 11, color: Colors.white.withValues(alpha: 0.6))),
            ]);
          }
          final slow = snapshot.data;
          final battery = slow?.resources.batteryPercent;
          final rtt = provider.connectionRttMs;
          return ValueListenableBuilder<({double vx, double vy, double wz})>(
            valueListenable: gw.velocityNotifier,
            builder: (context, vel, _) {
              final speed = sqrt(vel.vx * vel.vx + vel.vy * vel.vy);
              return Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  _hudItem(Icons.battery_std_rounded,
                      battery != null ? '${battery.toStringAsFixed(0)}%' : '--'),
                  const SizedBox(width: 14),
                  _hudItem(Icons.network_check_rounded,
                      rtt != null ? '${rtt.toStringAsFixed(0)}ms' : '--'),
                  const SizedBox(width: 14),
                  _hudItem(Icons.speed_rounded, '${speed.toStringAsFixed(1)}m/s'),
                ],
              );
            },
          );
        },
      ),
    );
  }

  Widget _hudItem(IconData icon, String value) {
    final color = Colors.white.withValues(alpha: 0.75);
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(icon, size: 13, color: color),
        const SizedBox(width: 4),
        Text(value, style: TextStyle(
          fontSize: 11, color: color,
          fontFeatures: const [FontFeature.tabularFigures()],
        )),
      ],
    );
  }

  // ─── Speed bar (above translation joystick) ───

  Widget _buildSpeedBar(ControlGateway gw) {
    final profile = context.read<RobotProfileProvider>().current;
    final maxSpeed = profile.maxLinearSpeed > 0 ? profile.maxLinearSpeed : 1.0;
    return ValueListenableBuilder<({double vx, double vy, double wz})>(
      valueListenable: gw.velocityNotifier,
      builder: (context, vel, _) {
        final speed = sqrt(vel.vx * vel.vx + vel.vy * vel.vy);
        final fraction = (speed / maxSpeed).clamp(0.0, 1.0);
        final barColor = fraction < 0.5
            ? AppColors.success
            : fraction < 0.8
                ? AppColors.warning
                : AppColors.error;
        return SizedBox(
          width: 180,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              ClipRRect(
                borderRadius: BorderRadius.circular(2),
                child: LinearProgressIndicator(
                  value: fraction,
                  backgroundColor: Colors.white.withValues(alpha: 0.15),
                  valueColor: AlwaysStoppedAnimation(barColor.withValues(alpha: 0.85)),
                  minHeight: 3,
                ),
              ),
              const SizedBox(height: 2),
              Text(
                '${speed.toStringAsFixed(1)} m/s',
                style: TextStyle(
                  fontSize: 10,
                  color: barColor.withValues(alpha: 0.9),
                  fontFeatures: const [FontFeature.tabularFigures()],
                ),
              ),
              // ─── Speed limit slider ───
              const SizedBox(height: 4),
              _buildSpeedLimitSlider(),
            ],
          ),
        );
      },
    );
  }

  // ─── Speed limit slider widget ───

  Widget _buildSpeedLimitSlider() {
    final labelColor = context.isDark ? Colors.white54 : Colors.black54;
    return Row(
      children: [
        Icon(Icons.speed, size: 12, color: labelColor),
        Expanded(
          child: SliderTheme(
            data: SliderTheme.of(context).copyWith(
              trackHeight: 2,
              thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 6),
              overlayShape: const RoundSliderOverlayShape(overlayRadius: 12),
              activeTrackColor: AppColors.warning,
              inactiveTrackColor: Colors.white24,
              thumbColor: AppColors.warning,
            ),
            child: Slider(
              value: _maxSpeedLimit,
              min: 0.1,
              max: 2.0,
              onChanged: (v) => setState(() => _maxSpeedLimit = v),
            ),
          ),
        ),
        Text(
          _maxSpeedLimit.toStringAsFixed(1),
          style: TextStyle(
            fontSize: 9,
            color: labelColor,
            fontFeatures: const [FontFeature.tabularFigures()],
          ),
        ),
      ],
    );
  }

  // ─── Deadzone slider widget ───

  Widget _buildDeadzoneSlider() {
    final labelColor = context.isDark ? Colors.white54 : Colors.black54;
    return SizedBox(
      width: 180,
      child: Row(
        children: [
          Icon(Icons.adjust, size: 12, color: labelColor),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                trackHeight: 2,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 6),
                overlayShape: const RoundSliderOverlayShape(overlayRadius: 12),
                activeTrackColor: Colors.white70,
                inactiveTrackColor: Colors.white24,
                thumbColor: Colors.white,
              ),
              child: Slider(
                value: _deadzone,
                min: 0.0,
                max: 0.30,
                onChanged: (v) => setState(() => _deadzone = v),
              ),
            ),
          ),
          Text(
            '${(_deadzone * 100).toStringAsFixed(0)}%',
            style: TextStyle(
              fontSize: 9,
              color: labelColor,
              fontFeatures: const [FontFeature.tabularFigures()],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDogButton({
    required String label,
    required IconData icon,
    required VoidCallback onTap,
    bool isActive = false,
  }) {
    final textColor = context.isDark ? Colors.white70 : Colors.black87;
    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(8),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
          decoration: BoxDecoration(
            color: isActive ? Colors.white.withValues(alpha: 0.1) : Colors.transparent,
            borderRadius: BorderRadius.circular(8),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(icon, size: 14, color: textColor),
              const SizedBox(width: 6),
              Text(
                label,
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: isActive ? FontWeight.w600 : FontWeight.w400,
                  color: textColor,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
