import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import '../services/robot_client_base.dart';
import '../generated/common.pb.dart';
import '../generated/control.pb.dart';
import '../widgets/glass_widgets.dart';

class ControlScreen extends StatefulWidget {
  final RobotClientBase client;

  const ControlScreen({super.key, required this.client});

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  bool _hasLease = false;
  final StreamController<Twist> _velocityController = StreamController<Twist>.broadcast();
  StreamSubscription? _teleopSubscription;
  double _linearX = 0.0;
  double _linearY = 0.0;
  double _angularZ = 0.0;

  @override
  void initState() {
    super.initState();
    _checkLease();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }

  @override
  void dispose() {
    _velocityController.close();
    _teleopSubscription?.cancel();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
    super.dispose();
  }

  Future<void> _checkLease() async {
    // TODO: better lease state tracking
  }

  Future<void> _toggleLease() async {
    if (_hasLease) {
      await widget.client.releaseLease();
      setState(() {
        _hasLease = false;
        _teleopSubscription?.cancel();
        _teleopSubscription = null;
      });
    } else {
      final success = await widget.client.acquireLease();
      if (success) {
        setState(() {
          _hasLease = true;
        });
        _startTeleopStream();
      } else {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Failed to acquire lease')),
        );
      }
    }
  }

  void _startTeleopStream() {
    try {
      _teleopSubscription = widget.client.streamTeleop(_velocityController.stream).listen((feedback) {
        // Handle feedback
      }, onError: (e) {
        print('Teleop stream error: $e');
        setState(() {
          _hasLease = false;
        });
      });
    } catch (e) {
      print('Failed to start teleop stream: $e');
    }
  }

  void _onLeftJoystickChange(StickDragDetails details) {
    if (!_hasLease) return;
    const double maxLinearSpeed = 1.0; 
    _linearX = -details.y * maxLinearSpeed;
    _linearY = details.x * maxLinearSpeed;
    _sendTwist();
  }

  void _onRightJoystickChange(StickDragDetails details) {
    if (!_hasLease) return;
    const double maxAngularSpeed = 1.5; 
    _angularZ = -details.x * maxAngularSpeed;
    _sendTwist();
  }

  void _sendTwist() {
    final twist = Twist()
      ..linear = (Vector3()..x = _linearX..y = _linearY)
      ..angular = (Vector3()..z = _angularZ);
    _velocityController.add(twist);
  }

  Future<void> _setMode(RobotMode mode) async {
    final success = await widget.client.setMode(mode);
    if (success) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Mode set to $mode')),
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Failed to set mode')),
      );
    }
  }

  Future<void> _emergencyStop() async {
    await widget.client.emergencyStop(hardStop: false);
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('EMERGENCY STOP TRIGGERED')),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        automaticallyImplyLeading: false, // Hide default back button
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
                icon: const Icon(Icons.arrow_back_ios_new, size: 18, color: Colors.black87),
                onPressed: () => Navigator.of(context).pop(),
                padding: EdgeInsets.zero,
                constraints: const BoxConstraints(),
              ),
              const SizedBox(width: 12),
              const Text(
                'Control Center',
                style: TextStyle(
                  color: Colors.black87,
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
            padding: const EdgeInsets.only(right: 24.0),
            child: GestureDetector(
              onTap: _toggleLease,
              child: GlassCard(
                borderRadius: 20,
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                color: _hasLease ? Colors.green.withOpacity(0.2) : Colors.grey.withOpacity(0.2),
                child: Row(
                  children: [
                    Icon(
                      _hasLease ? Icons.lock_open : Icons.lock,
                      size: 16,
                      color: _hasLease ? Colors.green[700] : Colors.grey[700],
                    ),
                    const SizedBox(width: 8),
                    Text(
                      _hasLease ? 'LEASE ACTIVE' : 'NO LEASE',
                      style: TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.bold,
                        color: _hasLease ? Colors.green[800] : Colors.grey[800],
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
          // Background gradient or elements could go here
          
          // Main Layout
          Padding(
            padding: EdgeInsets.only(top: MediaQuery.of(context).padding.top + 60, bottom: 20, left: 40, right: 40),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                // Left Joystick (Translation)
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
                        // Mode Selection
                        GlassCard(
                          padding: const EdgeInsets.all(4),
                          borderRadius: 16,
                          child: Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              _buildModeButton('IDLE', RobotMode.ROBOT_MODE_IDLE, Colors.grey),
                              _buildModeButton('MANUAL', RobotMode.ROBOT_MODE_MANUAL, Colors.blue),
                              _buildModeButton('AUTO', RobotMode.ROBOT_MODE_AUTONOMOUS, Colors.purple),
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
                              border: Border.all(color: Colors.white.withOpacity(0.5), width: 2),
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
                          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                          child: Text(
                            'Linear: ${_linearX.toStringAsFixed(2)}, ${_linearY.toStringAsFixed(2)} | Angular: ${_angularZ.toStringAsFixed(2)}',
                            style: TextStyle(
                              fontSize: 12,
                              fontFamily: 'Courier', // Monospace for numbers
                              color: Colors.black.withOpacity(0.6),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),

                // Right Joystick (Rotation)
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
              Icon(icon, size: 14, color: Colors.black54),
              const SizedBox(width: 6),
              Text(
                label,
                style: const TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: Colors.black54,
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
              color: Colors.white.withOpacity(0.2), // Frosted base
              border: Border.all(color: Colors.white.withOpacity(0.4), width: 1),
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
              border: Border.all(color: Colors.white.withOpacity(0.6), width: 1),
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
        borderRadius: BorderRadius.circular(12),
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
