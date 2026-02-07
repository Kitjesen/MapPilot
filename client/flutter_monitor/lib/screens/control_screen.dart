import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import '../services/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import '../theme/app_theme.dart';
import '../widgets/glass_widgets.dart';
import '../widgets/camera_stream_widget.dart';
import '../widgets/webrtc_video_widget.dart';
import '../theme/app_theme.dart';

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

  DateTime _lastTwistSend = DateTime(2000);
  static const _twistSendInterval = Duration(milliseconds: 50);

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
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
            const SnackBar(content: Text('无法获取控制权')),
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
        (feedback) {},
        onError: (e) {
          debugPrint('Teleop stream error: $e');
          if (mounted) context.read<RobotConnectionProvider>().releaseLease();
        },
      );
    } catch (e) {
      debugPrint('Failed to start teleop stream: $e');
    }
  }

  void _onLeftJoystickChange(StickDragDetails details) {
    final provider = context.read<RobotConnectionProvider>();
    if (!provider.hasLease) return;
    const double maxLinearSpeed = 1.0;
    _linearX = -details.y * maxLinearSpeed;
    _linearY = details.x * maxLinearSpeed;
    _throttledSendTwist();
  }

  void _onRightJoystickChange(StickDragDetails details) {
    final provider = context.read<RobotConnectionProvider>();
    if (!provider.hasLease) return;
    const double maxAngularSpeed = 1.5;
    _angularZ = -details.x * maxAngularSpeed;
    _throttledSendTwist();
  }

  void _throttledSendTwist() {
    final now = DateTime.now();
    if (now.difference(_lastTwistSend) >= _twistSendInterval) {
      _lastTwistSend = now;
      _sendTwist();
    }
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

  Future<void> _setMode(RobotMode mode) async {
    HapticFeedback.lightImpact();
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;
    final success = await client.setMode(mode);
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(success ? '模式已切换: $mode' : '模式切换失败'),
          duration: const Duration(seconds: 1),
        ),
      );
    }
  }

  Future<void> _emergencyStop() async {
    HapticFeedback.heavyImpact();
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;
    await client.emergencyStop(hardStop: false);
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: const Text('紧急停止已触发'),
          backgroundColor: AppColors.error,
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final provider = context.watch<RobotConnectionProvider>();
    final hasLease = provider.hasLease;
    final client = provider.client;

    return Scaffold(
      backgroundColor: AppColors.bg,
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        automaticallyImplyLeading: false,
        backgroundColor: Colors.transparent,
        elevation: 0,
        title: GlassCard(
          borderRadius: 30,
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
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
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
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
          Padding(
            padding: const EdgeInsets.only(right: 24.0),
            child: GestureDetector(
              onTap: _toggleLease,
              child: GlassCard(
                borderRadius: 20,
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
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
          // Camera Background
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
                    : Container(color: AppColors.bg),
          ),

          // Dark gradient overlay
          Positioned.fill(
            child: Container(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [
                    Colors.black.withOpacity(0.4),
                    Colors.black.withOpacity(0.1),
                    Colors.black.withOpacity(0.4),
                  ],
                ),
              ),
            ),
          ),

          // Main Layout
          Padding(
            padding: EdgeInsets.only(
                top: MediaQuery.of(context).padding.top + 60,
                bottom: 20, left: 40, right: 40),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                _buildJoystickSection(
                  label: 'TRANSLATION',
                  icon: Icons.open_with,
                  listener: _onLeftJoystickChange,
                  mode: JoystickMode.all,
                ),
                Expanded(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 24.0),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        // Mode Selection
                        GlassCard(
                          padding: const EdgeInsets.all(4),
                          borderRadius: 22,
                          child: Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              _buildModeButton('IDLE',
                                  RobotMode.ROBOT_MODE_IDLE, AppColors.textTertiary),
                              _buildModeButton('MANUAL',
                                  RobotMode.ROBOT_MODE_MANUAL, AppColors.info),
                              _buildModeButton('AUTO',
                                  RobotMode.ROBOT_MODE_AUTONOMOUS, AppColors.lime),
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
                              color: AppColors.error.withOpacity(0.85),
                              boxShadow: [
                                BoxShadow(
                                  color: AppColors.error.withOpacity(0.4),
                                  blurRadius: 20,
                                  offset: const Offset(0, 8),
                                ),
                              ],
                              border: Border.all(
                                  color: Colors.white.withOpacity(0.3), width: 2),
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
                            'Vx: ${_linearX.toStringAsFixed(2)}  Vy: ${_linearY.toStringAsFixed(2)}  Wz: ${_angularZ.toStringAsFixed(2)}',
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
              color: AppColors.surface.withOpacity(0.3),
              border: Border.all(color: AppColors.border.withOpacity(0.4), width: 1),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.15),
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
                    color: AppColors.surface.withOpacity(0.15),
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
              color: AppColors.surfaceLight.withOpacity(0.6),
              border: Border.all(color: AppColors.lime.withOpacity(0.3), width: 1),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.2),
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
                    gradient: RadialGradient(
                      colors: [
                        AppColors.lime.withOpacity(0.15),
                        Colors.transparent,
                      ],
                    ),
                  ),
                  child: Center(
                    child: Container(
                      width: 18,
                      height: 18,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: AppColors.lime.withOpacity(0.6),
                        boxShadow: [
                          BoxShadow(
                            color: AppColors.lime.withOpacity(0.2),
                            blurRadius: 8,
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
}
