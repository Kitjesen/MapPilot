import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'services/robot_client.dart';
import 'services/mock_robot_client.dart';
import 'services/robot_connection_provider.dart';
import 'screens/home_screen.dart';
import 'theme/app_theme.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const RobotMonitorApp());
}

class RobotMonitorApp extends StatelessWidget {
  const RobotMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (_) => RobotConnectionProvider(),
      child: MaterialApp(
        title: '大算机器人',
        debugShowCheckedModeBanner: false,
        theme: AppTheme.dark,
        home: const ConnectionScreen(),
      ),
    );
  }
}

class ConnectionScreen extends StatefulWidget {
  const ConnectionScreen({super.key});

  @override
  State<ConnectionScreen> createState() => _ConnectionScreenState();
}

class _ConnectionScreenState extends State<ConnectionScreen>
    with TickerProviderStateMixin {
  final _hostController = TextEditingController(text: '192.168.66.190');
  final _portController = TextEditingController(text: '50051');
  bool _isConnecting = false;
  String? _errorMessage;

  static String _lastHost = '192.168.66.190';
  static String _lastPort = '50051';

  // ── Animations ──
  late AnimationController _entranceController;
  late AnimationController _pulseController;
  late AnimationController _floatController;

  late Animation<double> _logoSlide;
  late Animation<double> _logoFade;
  late Animation<double> _titleSlide;
  late Animation<double> _titleFade;
  late Animation<double> _cardSlide;
  late Animation<double> _cardFade;
  late Animation<double> _pulse;
  late Animation<double> _float;

  @override
  void initState() {
    super.initState();
    _hostController.text = _lastHost;
    _portController.text = _lastPort;

    SystemChrome.setSystemUIOverlayStyle(AppTheme.systemOverlay);

    // Stagger entrance
    _entranceController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1400),
    );

    _logoSlide = Tween<double>(begin: -40, end: 0).animate(CurvedAnimation(
      parent: _entranceController,
      curve: const Interval(0.0, 0.5, curve: Curves.easeOutCubic),
    ));
    _logoFade = Tween<double>(begin: 0, end: 1).animate(CurvedAnimation(
      parent: _entranceController,
      curve: const Interval(0.0, 0.4, curve: Curves.easeOut),
    ));
    _titleSlide = Tween<double>(begin: 30, end: 0).animate(CurvedAnimation(
      parent: _entranceController,
      curve: const Interval(0.2, 0.6, curve: Curves.easeOutCubic),
    ));
    _titleFade = Tween<double>(begin: 0, end: 1).animate(CurvedAnimation(
      parent: _entranceController,
      curve: const Interval(0.2, 0.55, curve: Curves.easeOut),
    ));
    _cardSlide = Tween<double>(begin: 50, end: 0).animate(CurvedAnimation(
      parent: _entranceController,
      curve: const Interval(0.35, 0.8, curve: Curves.easeOutCubic),
    ));
    _cardFade = Tween<double>(begin: 0, end: 1).animate(CurvedAnimation(
      parent: _entranceController,
      curve: const Interval(0.35, 0.75, curve: Curves.easeOut),
    ));

    // Glow pulse
    _pulseController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 3),
    )..repeat(reverse: true);
    _pulse = Tween<double>(begin: 0.4, end: 1.0).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );

    // Floating bob
    _floatController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 2600),
    )..repeat(reverse: true);
    _float = Tween<double>(begin: -6, end: 6).animate(
      CurvedAnimation(parent: _floatController, curve: Curves.easeInOut),
    );

    _entranceController.forward();
  }

  @override
  void dispose() {
    _hostController.dispose();
    _portController.dispose();
    _entranceController.dispose();
    _pulseController.dispose();
    _floatController.dispose();
    super.dispose();
  }

  Future<void> _connect() async {
    if (_isConnecting) return;

    final host = _hostController.text.trim();
    final portText = _portController.text.trim();

    if (host.isEmpty) {
      setState(() => _errorMessage = '请输入机器人 IP 地址');
      return;
    }
    final port = int.tryParse(portText);
    if (port == null || port <= 0 || port > 65535) {
      setState(() => _errorMessage = '端口号无效 (1-65535)');
      return;
    }

    setState(() {
      _isConnecting = true;
      _errorMessage = null;
    });

    HapticFeedback.lightImpact();

    try {
      final client = RobotClient(host: host, port: port);
      final provider = context.read<RobotConnectionProvider>();
      final connected = await provider.connect(client);
      if (!mounted) return;

      if (connected) {
        _lastHost = host;
        _lastPort = portText;
        Navigator.of(context).pushReplacement(
          PageRouteBuilder(
            pageBuilder: (_, __, ___) => const HomeScreen(),
            transitionsBuilder: (_, a, __, child) =>
                FadeTransition(opacity: a, child: child),
            transitionDuration: const Duration(milliseconds: 500),
          ),
        );
      } else {
        setState(() {
          _errorMessage = provider.errorMessage ?? '无法连接到机器人';
          _isConnecting = false;
        });
      }
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _errorMessage = '连接错误: $e';
        _isConnecting = false;
      });
    }
  }

  Future<void> _startMock() async {
    HapticFeedback.lightImpact();
    final client = MockRobotClient();
    final provider = context.read<RobotConnectionProvider>();
    await provider.connect(client);
    if (!mounted) return;

    Navigator.of(context).pushReplacement(
      PageRouteBuilder(
        pageBuilder: (_, __, ___) => const HomeScreen(),
        transitionsBuilder: (_, a, __, child) =>
            FadeTransition(opacity: a, child: child),
        transitionDuration: const Duration(milliseconds: 500),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Container(
        decoration: const BoxDecoration(
          gradient: RadialGradient(
            center: Alignment(0, -0.3),
            radius: 1.4,
            colors: [Color(0xFF1A1A1A), Color(0xFF0D0D0D)],
          ),
        ),
        child: SafeArea(
          child: Center(
            child: SingleChildScrollView(
              padding: const EdgeInsets.symmetric(horizontal: 28),
              child: AnimatedBuilder(
                animation: Listenable.merge(
                    [_entranceController, _pulseController, _floatController]),
                builder: (context, _) => Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    // ── Robot mascot ──
                    Transform.translate(
                      offset: Offset(0, _logoSlide.value + _float.value),
                      child: Opacity(
                        opacity: _logoFade.value,
                        child: _buildRobotMascot(),
                      ),
                    ),
                    const SizedBox(height: 28),

                    // ── Title ──
                    Transform.translate(
                      offset: Offset(0, _titleSlide.value),
                      child: Opacity(
                        opacity: _titleFade.value,
                        child: Column(
                          children: [
                            const Text(
                              '大算机器人',
                              style: TextStyle(
                                fontSize: 32,
                                fontWeight: FontWeight.w800,
                                letterSpacing: -1,
                                color: AppColors.textPrimary,
                              ),
                            ),
                            const SizedBox(height: 6),
                            Text(
                              'Remote Monitor & Control',
                              style: TextStyle(
                                fontSize: 14,
                                fontWeight: FontWeight.w400,
                                color: AppColors.textTertiary,
                                letterSpacing: 0.8,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                    const SizedBox(height: 44),

                    // ── Connection card ──
                    Transform.translate(
                      offset: Offset(0, _cardSlide.value),
                      child: Opacity(
                        opacity: _cardFade.value,
                        child: _buildConnectionCard(),
                      ),
                    ),
                    const SizedBox(height: 28),

                    // ── Footer ──
                    Opacity(
                      opacity: _cardFade.value,
                      child: Text(
                        'gRPC · WebRTC · ROS 2',
                        style: TextStyle(
                          fontSize: 11,
                          color: AppColors.textTertiary.withOpacity(0.5),
                          letterSpacing: 2,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildRobotMascot() {
    return Container(
      width: 110,
      height: 110,
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        gradient: RadialGradient(
          colors: [
            AppColors.lime.withOpacity(0.15 * _pulse.value),
            Colors.transparent,
          ],
          radius: 1.2,
        ),
      ),
      child: Center(
        child: Container(
          width: 90,
          height: 90,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: AppColors.surface,
            border: Border.all(
              color: AppColors.lime.withOpacity(0.3 * _pulse.value),
              width: 2,
            ),
            boxShadow: [
              BoxShadow(
                color: AppColors.lime.withOpacity(0.15 * _pulse.value),
                blurRadius: 30,
                spreadRadius: 4,
              ),
            ],
          ),
          child: const Icon(
            Icons.smart_toy_rounded,
            size: 44,
            color: AppColors.lime,
          ),
        ),
      ),
    );
  }

  Widget _buildConnectionCard() {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(24),
        border: Border.all(color: AppColors.border, width: 0.5),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.3),
            blurRadius: 30,
            offset: const Offset(0, 12),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          _buildInputField(
            controller: _hostController,
            icon: Icons.wifi_tethering,
            label: 'Robot IP',
            onSubmitted: (_) => _connect(),
          ),
          const SizedBox(height: 12),
          _buildInputField(
            controller: _portController,
            icon: Icons.tag,
            label: 'Port',
            keyboardType: TextInputType.number,
            onSubmitted: (_) => _connect(),
          ),

          // Error
          AnimatedSize(
            duration: const Duration(milliseconds: 200),
            child: _errorMessage != null
                ? Padding(
                    padding: const EdgeInsets.only(top: 16),
                    child: Container(
                      padding: const EdgeInsets.all(12),
                      decoration: BoxDecoration(
                        color: AppColors.error.withOpacity(0.1),
                        borderRadius: BorderRadius.circular(12),
                        border: Border.all(
                          color: AppColors.error.withOpacity(0.2),
                        ),
                      ),
                      child: Row(
                        children: [
                          const Icon(Icons.error_outline,
                              size: 18, color: AppColors.error),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              _errorMessage!,
                              style: const TextStyle(
                                fontSize: 13,
                                color: AppColors.error,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  )
                : const SizedBox.shrink(),
          ),
          const SizedBox(height: 24),

          // Connect button (lime)
          _buildLimeButton(
            onPressed: _isConnecting ? null : _connect,
            isLoading: _isConnecting,
            label: _isConnecting ? '连接中...' : '连接机器人',
            icon: Icons.link,
          ),
          const SizedBox(height: 12),

          // Mock button
          GestureDetector(
            onTap: _isConnecting ? null : _startMock,
            child: Container(
              height: 52,
              decoration: BoxDecoration(
                color: AppColors.surfaceLight,
                borderRadius: BorderRadius.circular(16),
                border: Border.all(color: AppColors.border, width: 0.5),
              ),
              child: Center(
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Icon(Icons.science_outlined,
                        size: 20, color: AppColors.textSecondary),
                    const SizedBox(width: 8),
                    Text(
                      'Mock 演示模式',
                      style: TextStyle(
                        fontSize: 15,
                        fontWeight: FontWeight.w500,
                        color: AppColors.textSecondary,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildInputField({
    required TextEditingController controller,
    required IconData icon,
    required String label,
    TextInputType keyboardType = TextInputType.text,
    ValueChanged<String>? onSubmitted,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: AppColors.bgElevated,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: AppColors.border, width: 0.5),
      ),
      child: TextField(
        controller: controller,
        keyboardType: keyboardType,
        textInputAction: TextInputAction.next,
        onSubmitted: onSubmitted,
        style: const TextStyle(
          fontSize: 16,
          fontWeight: FontWeight.w500,
          color: AppColors.textPrimary,
        ),
        decoration: InputDecoration(
          prefixIcon: Icon(icon, size: 20, color: AppColors.lime),
          hintText: label,
          hintStyle: TextStyle(
            color: AppColors.textTertiary,
            fontWeight: FontWeight.w400,
          ),
          border: InputBorder.none,
          contentPadding:
              const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
        ),
      ),
    );
  }

  Widget _buildLimeButton({
    required VoidCallback? onPressed,
    required bool isLoading,
    required String label,
    required IconData icon,
  }) {
    return GestureDetector(
      onTap: onPressed,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        height: 54,
        decoration: BoxDecoration(
          color: onPressed != null
              ? AppColors.lime
              : AppColors.lime.withOpacity(0.3),
          borderRadius: BorderRadius.circular(16),
          boxShadow: onPressed != null
              ? [
                  BoxShadow(
                    color: AppColors.lime.withOpacity(0.25),
                    blurRadius: 20,
                    offset: const Offset(0, 8),
                  ),
                ]
              : [],
        ),
        child: Center(
          child: isLoading
              ? const SizedBox(
                  width: 22,
                  height: 22,
                  child: CircularProgressIndicator(
                    strokeWidth: 2.5,
                    color: Colors.black,
                  ),
                )
              : Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Icon(icon, size: 20, color: Colors.black),
                    const SizedBox(width: 8),
                    Text(
                      label,
                      style: const TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w700,
                        color: Colors.black,
                        letterSpacing: 0.3,
                      ),
                    ),
                  ],
                ),
        ),
      ),
    );
  }
}
