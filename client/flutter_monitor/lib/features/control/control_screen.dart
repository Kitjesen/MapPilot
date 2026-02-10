import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/shared/widgets/glass_widgets.dart';
import 'package:flutter_monitor/features/control/camera_stream_widget.dart';
import 'package:flutter_monitor/features/control/webrtc_video_widget.dart';
import 'package:flutter_monitor/core/grpc/dog_direct_client.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    // 隐藏系统状态栏提升沉浸感
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
  }

  @override
  void dispose() {
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
    if (gw.useDogDirect) {
      gw.sendVelocity(-details.y, details.x, gw.angularZ);
    } else {
      if (!gw.hasLease) return;
      final profile = context.read<RobotProfileProvider>().current;
      gw.sendVelocity(
        -details.y * profile.maxLinearSpeed,
        details.x * profile.maxLinearSpeed,
        gw.angularZ,
      );
    }
  }

  void _onRightJoystickChange(StickDragDetails details) {
    final gw = context.read<ControlGateway>();
    if (gw.useDogDirect) {
      gw.sendVelocity(gw.linearX, gw.linearY, -details.x);
    } else {
      if (!gw.hasLease) return;
      final profile = context.read<RobotProfileProvider>().current;
      gw.sendVelocity(gw.linearX, gw.linearY, -details.x * profile.maxAngularSpeed);
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
          content: Text(gw.statusMessage ?? '无法获取控制权'),
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
          content: const Text('紧急停止已触发'),
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
        SnackBar(content: Text('使能失败: ${err ?? ""}'), behavior: SnackBarBehavior.floating),
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
        SnackBar(content: Text('站立失败: ${err ?? ""}'), behavior: SnackBarBehavior.floating),
      );
    }
  }

  Future<void> _dogSitDown() async {
    HapticFeedback.mediumImpact();
    await context.read<ControlGateway>().dogSitDown();
  }

  @override
  Widget build(BuildContext context) {
    final gw = context.watch<ControlGateway>();
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
                '控制中心',
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
                  color: gw.useDogDirect
                      ? AppColors.warning.withValues(alpha: 0.2)
                      : Colors.grey.withValues(alpha: 0.1),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(
                        Icons.pets,
                        size: 14,
                        color:
                            gw.useDogDirect ? AppColors.warning : Colors.grey,
                      ),
                      const SizedBox(width: 6),
                      Text(
                        gw.useDogDirect ? 'DOG' : 'NAV',
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          color: gw.useDogDirect
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
                color: gw.hasLease
                    ? AppColors.success.withValues(alpha: 0.2)
                    : Colors.grey.withValues(alpha: 0.2),
                child: Row(
                  children: [
                    Icon(
                      gw.hasLease ? Icons.lock_open : Icons.lock,
                      size: 16,
                      color: gw.hasLease ? AppColors.success : Colors.grey,
                    ),
                    const SizedBox(width: 8),
                    Text(
                      gw.hasLease ? 'LEASE ACTIVE' : 'NO LEASE',
                      style: TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.bold,
                        color: gw.hasLease ? AppColors.success : Colors.grey,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
      body: Stack(
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
                ),

                // Center Controls
                Expanded(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 24.0),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        // Mode Selection / Dog Control
                        if (gw.useDogDirect)
                          _buildDogControlButtons(gw.dogClient)
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
                        // Status Text
                        GlassCard(
                          padding: const EdgeInsets.symmetric(
                              horizontal: 16, vertical: 8),
                          child: Text(
                            gw.useDogDirect
                                ? 'X: ${gw.linearX.toStringAsFixed(2)}  Y: ${gw.linearY.toStringAsFixed(2)}  Z: ${gw.angularZ.toStringAsFixed(2)}'
                                : 'Vx: ${gw.linearX.toStringAsFixed(2)}  Vy: ${gw.linearY.toStringAsFixed(2)}  Wz: ${gw.angularZ.toStringAsFixed(2)}',
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
        ],
      ),
    );
  }

  Widget _buildJoystickSection({
    required String label,
    required IconData icon,
    required void Function(StickDragDetails) listener,
    required JoystickMode mode,
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
        const SizedBox(height: 20),
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
