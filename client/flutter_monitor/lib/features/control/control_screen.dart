import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
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
  final StreamController<Twist> _velocityController =
      StreamController<Twist>.broadcast();
  StreamSubscription? _teleopSubscription;
  double _linearX = 0.0;
  double _linearY = 0.0;
  double _angularZ = 0.0;
  bool _useDogDirect = false;

  // 发送节流
  DateTime _lastTwistSend = DateTime(2000);
  static const _twistSendInterval = Duration(milliseconds: 50); // 20Hz max

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    // 隐藏系统状态栏提升沉浸感
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);

    // Auto-select dog direct mode when only dog board is connected
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final provider = context.read<RobotConnectionProvider>();
      if (provider.isDogConnected && !provider.isConnected) {
        setState(() => _useDogDirect = true);
      }
    });
  }

  @override
  void dispose() {
    _velocityController.close();
    _teleopSubscription?.cancel();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    super.dispose();
  }

  Future<void> _toggleLease() async {
    final provider = context.read<RobotConnectionProvider>();
    HapticFeedback.mediumImpact();

    if (provider.hasLease) {
      await provider.releaseLease();
      _teleopSubscription?.cancel();
      _teleopSubscription = null;
    } else {
      final success = await provider.acquireLease();
      if (success) {
        _startTeleopStream();
      } else {
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text('无法获取控制权'),
              behavior: SnackBarBehavior.floating,
            ),
          );
        }
      }
    }
  }

  void _startTeleopStream() {
    try {
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) return;

      _teleopSubscription =
          client.streamTeleop(_velocityController.stream).listen(
        (feedback) {
          // Handle feedback (e.g., safety warnings)
        },
        onError: (e) {
          debugPrint('Teleop stream error: $e');
          if (mounted) {
            context.read<RobotConnectionProvider>().releaseLease();
          }
        },
      );
    } catch (e) {
      debugPrint('Failed to start teleop stream: $e');
    }
  }

  void _onLeftJoystickChange(StickDragDetails details) {
    if (_useDogDirect) {
      _linearX = -details.y; // normalized [-1, 1]
      _linearY = details.x;
    } else {
      final provider = context.read<RobotConnectionProvider>();
      if (!provider.hasLease) return;
      final profile = context.read<RobotProfileProvider>().current;
      _linearX = -details.y * profile.maxLinearSpeed;
      _linearY = details.x * profile.maxLinearSpeed;
    }
    _throttledSendCommand();
  }

  void _onRightJoystickChange(StickDragDetails details) {
    if (_useDogDirect) {
      _angularZ = -details.x;
    } else {
      final provider = context.read<RobotConnectionProvider>();
      if (!provider.hasLease) return;
      final profile = context.read<RobotProfileProvider>().current;
      _angularZ = -details.x * profile.maxAngularSpeed;
    }
    _throttledSendCommand();
  }

  void _throttledSendCommand() {
    final now = DateTime.now();
    if (now.difference(_lastTwistSend) >= _twistSendInterval) {
      _lastTwistSend = now;
      if (_useDogDirect) {
        _sendDogWalk();
      } else {
        _sendTwist();
      }
    }
  }

  void _sendDogWalk() {
    final dogClient = context.read<RobotConnectionProvider>().dogClient;
    dogClient?.walk(_linearX, _linearY, _angularZ);
    if (mounted) setState(() {});
  }

  void _sendTwist() {
    final twist = Twist()
      ..linear = (Vector3()
        ..x = _linearX
        ..y = _linearY)
      ..angular = (Vector3()..z = _angularZ);
    _velocityController.add(twist);
    if (mounted) setState(() {});
  }

  // ─── Dog Direct Control ───

  Future<void> _dogEnable() async {
    HapticFeedback.mediumImpact();
    final dogClient = context.read<RobotConnectionProvider>().dogClient;
    if (dogClient == null) return;
    final ok = await dogClient.enable();
    if (mounted && !ok) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
            content: Text('使能失败: ${dogClient.errorMessage}'),
            behavior: SnackBarBehavior.floating),
      );
    }
  }

  Future<void> _dogDisable() async {
    HapticFeedback.mediumImpact();
    final dogClient = context.read<RobotConnectionProvider>().dogClient;
    if (dogClient == null) return;
    await dogClient.disable();
  }

  Future<void> _dogStandUp() async {
    HapticFeedback.mediumImpact();
    final dogClient = context.read<RobotConnectionProvider>().dogClient;
    if (dogClient == null) return;
    final ok = await dogClient.standUp();
    if (mounted && !ok) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
            content: Text('站立失败: ${dogClient.errorMessage}'),
            behavior: SnackBarBehavior.floating),
      );
    }
  }

  Future<void> _dogSitDown() async {
    HapticFeedback.mediumImpact();
    final dogClient = context.read<RobotConnectionProvider>().dogClient;
    if (dogClient == null) return;
    await dogClient.sitDown();
  }

  Future<void> _setMode(RobotMode mode) async {
    HapticFeedback.lightImpact();
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;

    final success = await client.setMode(mode);
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(success ? '模式已切换: $mode' : '模式切换失败'),
          behavior: SnackBarBehavior.floating,
          duration: const Duration(seconds: 1),
        ),
      );
    }
  }

  Future<void> _emergencyStop() async {
    HapticFeedback.heavyImpact();
    _linearX = 0;
    _linearY = 0;
    _angularZ = 0;

    final provider = context.read<RobotConnectionProvider>();

    // Stop dog if connected
    if (provider.isDogConnected) {
      await provider.dogClient!.emergencyStop();
    }

    // Stop nav board if connected
    if (provider.isConnected && provider.client != null) {
      await provider.client!.emergencyStop(hardStop: false);
    }

    if (mounted) {
      setState(() {});
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: const Text('紧急停止已触发'),
          backgroundColor: Colors.red.shade700,
          behavior: SnackBarBehavior.floating,
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final provider = context.watch<RobotConnectionProvider>();
    final hasLease = provider.hasLease;
    final client = provider.client;
    final isDogConnected = provider.isDogConnected;
    final dogClient = provider.dogClient;

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
                  const Icon(Icons.videocam, size: 14, color: AppColors.primary),
                  const SizedBox(width: 6),
                  const Text(
                    'FPV',
                    style: TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w600,
                      color: AppColors.primary,
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
                  setState(() => _useDogDirect = !_useDogDirect);
                },
                child: GlassCard(
                  borderRadius: 20,
                  padding:
                      const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                  blurSigma: 10,
                  color: _useDogDirect
                      ? AppColors.warning.withOpacity(0.2)
                      : Colors.grey.withOpacity(0.1),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(
                        Icons.pets,
                        size: 14,
                        color:
                            _useDogDirect ? AppColors.warning : Colors.grey,
                      ),
                      const SizedBox(width: 6),
                      Text(
                        _useDogDirect ? 'DOG' : 'NAV',
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          color: _useDogDirect
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
                    ? AppColors.success.withOpacity(0.2)
                    : Colors.grey.withOpacity(0.2),
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
                    Colors.black.withOpacity(0.3),
                    Colors.black.withOpacity(0.1),
                    Colors.black.withOpacity(0.3),
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
                        if (_useDogDirect)
                          _buildDogControlButtons(dogClient)
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
                              ],
                            ),
                          ),
                        const Spacer(),
                        // E-Stop
                        GestureDetector(
                          onTap: _emergencyStop,
                          child: Container(
                            height: 80,
                            width: 80,
                            decoration: BoxDecoration(
                              shape: BoxShape.circle,
                              color: Colors.red.withOpacity(0.8),
                              boxShadow: [
                                BoxShadow(
                                  color: Colors.red.withOpacity(0.4),
                                  blurRadius: 20,
                                  offset: const Offset(0, 8),
                                ),
                              ],
                              border: Border.all(
                                  color: Colors.white.withOpacity(0.5),
                                  width: 2),
                            ),
                            child: const Center(
                              child: Text(
                                'STOP',
                                style: TextStyle(
                                  color: Colors.white,
                                  fontWeight: FontWeight.bold,
                                  fontSize: 18,
                                  letterSpacing: 1.0,
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
                            _useDogDirect
                                ? 'X: ${_linearX.toStringAsFixed(2)}  Y: ${_linearY.toStringAsFixed(2)}  Z: ${_angularZ.toStringAsFixed(2)}'
                                : 'Vx: ${_linearX.toStringAsFixed(2)}  Vy: ${_linearY.toStringAsFixed(2)}  Wz: ${_angularZ.toStringAsFixed(2)}',
                            style: TextStyle(
                              fontSize: 12,
                              fontFamily: 'monospace',
                              fontFeatures: const [
                                FontFeature.tabularFigures()
                              ],
                              color: context.isDark
                                  ? Colors.white70
                                  : Colors.black.withOpacity(0.6),
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
              color: Colors.white.withOpacity(0.2),
              border:
                  Border.all(color: Colors.white.withOpacity(0.4), width: 1),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.05),
                  blurRadius: 10,
                  spreadRadius: 2,
                ),
              ],
            ),
            child: ClipOval(
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
                child: Container(
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: Colors.white.withOpacity(0.1),
                  ),
                ),
              ),
            ),
          ),
          stick: Container(
            width: 60,
            height: 60,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Colors.white.withOpacity(0.4),
              border:
                  Border.all(color: Colors.white.withOpacity(0.6), width: 1),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.1),
                  blurRadius: 10,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: ClipOval(
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
                child: Container(
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    gradient: LinearGradient(
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                      colors: [
                        Colors.white.withOpacity(0.6),
                        Colors.white.withOpacity(0.1),
                      ],
                    ),
                  ),
                  child: Center(
                    child: Container(
                      width: 20,
                      height: 20,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: Colors.white.withOpacity(0.8),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black.withOpacity(0.1),
                            blurRadius: 4,
                          ),
                        ],
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
        borderRadius: BorderRadius.circular(16),
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
          child: Text(
            label,
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w600,
              color: color,
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
                color: AppColors.success,
                onTap: _dogEnable,
              ),
              _buildDogButton(
                label: 'DISABLE',
                icon: Icons.power_off,
                color: AppColors.error,
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
                color: AppColors.primary,
                onTap: _dogStandUp,
                isActive: isStanding,
              ),
              _buildDogButton(
                label: 'SIT',
                icon: Icons.arrow_downward,
                color: AppColors.warning,
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
    required Color color,
    required VoidCallback onTap,
    bool isActive = false,
  }) {
    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(16),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
          decoration: BoxDecoration(
            color: isActive ? color.withOpacity(0.15) : Colors.transparent,
            borderRadius: BorderRadius.circular(16),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(icon, size: 16, color: color),
              const SizedBox(width: 6),
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
        ),
      ),
    );
  }
}
