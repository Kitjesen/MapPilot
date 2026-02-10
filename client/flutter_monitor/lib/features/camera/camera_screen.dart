import 'dart:io' show Platform;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:window_manager/window_manager.dart';

import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/features/control/webrtc_video_widget.dart';

/// Dedicated fullscreen camera view with camera switching, connection
/// indicator, and gesture controls.
class CameraScreen extends StatefulWidget {
  const CameraScreen({super.key});

  @override
  State<CameraScreen> createState() => _CameraScreenState();
}

class _CameraScreenState extends State<CameraScreen>
    with SingleTickerProviderStateMixin {
  int _activeCameraIndex = 0;
  bool _showOverlay = true;
  bool _isFullscreen = false;

  late final AnimationController _overlayAnim;
  late final Animation<double> _overlayOpacity;

  /// Dynamic camera list from ListResources; falls back to defaults.
  List<String> get _cameraIds {
    final provider = context.read<RobotConnectionProvider>();
    final dynamic = provider.availableCameras;
    if (dynamic.isNotEmpty) return dynamic;
    return ['front', 'rear']; // fallback
  }

  String get _activeCameraId => _cameraIds[_activeCameraIndex % _cameraIds.length];

  @override
  void initState() {
    super.initState();
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);

    _overlayAnim = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 250),
      value: 1.0,
    );
    _overlayOpacity =
        CurvedAnimation(parent: _overlayAnim, curve: Curves.easeInOut);

    // Auto-hide overlay after 4 seconds
    Future.delayed(const Duration(seconds: 4), () {
      if (mounted && _showOverlay) _hideOverlay();
    });
  }

  @override
  void dispose() {
    _overlayAnim.dispose();
    // 退出时恢复窗口状态
    if (_isFullscreen) {
      _setFullscreen(false);
    }
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    super.dispose();
  }

  Future<void> _toggleFullscreen() async {
    HapticFeedback.lightImpact();
    final goFull = !_isFullscreen;
    await _setFullscreen(goFull);
    if (mounted) setState(() => _isFullscreen = goFull);
  }

  Future<void> _setFullscreen(bool fullscreen) async {
    if (Platform.isWindows || Platform.isLinux || Platform.isMacOS) {
      await windowManager.setFullScreen(fullscreen);
    } else {
      // Mobile
      if (fullscreen) {
        SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
      } else {
        SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
      }
    }
  }

  void _toggleCamera() {
    HapticFeedback.lightImpact();
    setState(() {
      _activeCameraIndex = (_activeCameraIndex + 1) % _cameraIds.length;
    });
  }

  void _showOverlayControls() {
    setState(() => _showOverlay = true);
    _overlayAnim.forward();
    // Auto-hide after 4s
    Future.delayed(const Duration(seconds: 4), () {
      if (mounted && _showOverlay) _hideOverlay();
    });
  }

  void _hideOverlay() {
    _overlayAnim.reverse().then((_) {
      if (mounted) setState(() => _showOverlay = false);
    });
  }

  void _toggleOverlay() {
    if (_showOverlay) {
      _hideOverlay();
    } else {
      _showOverlayControls();
    }
  }

  @override
  Widget build(BuildContext context) {
    final conn = context.watch<RobotConnectionProvider>();
    final client = conn.client;
    final isConnected = client != null;

    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        fit: StackFit.expand,
        children: [
          // Video feed — tap here to toggle overlay
          GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: _toggleOverlay,
            child: isConnected
                ? _CameraView(
                    key: ValueKey('camera_$_activeCameraId'),
                    client: client,
                    cameraId: _activeCameraId,
                  )
                : _buildDisconnectedView(),
          ),

          // Overlay controls with fade animation
          if (_showOverlay)
            FadeTransition(
              opacity: _overlayOpacity,
              child: Stack(
                children: [
                  // Top gradient + bar
                  Positioned(
                    top: 0,
                    left: 0,
                    right: 0,
                    child: _buildTopBar(context, isConnected),
                  ),
                  // Bottom gradient + controls
                  Positioned(
                    bottom: 0,
                    left: 0,
                    right: 0,
                    child: _buildBottomBar(context, isConnected),
                  ),
                ],
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildTopBar(BuildContext context, bool isConnected) {
    return Container(
      decoration: const BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
          colors: [Colors.black87, Colors.transparent],
        ),
      ),
      child: SafeArea(
        bottom: false,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 4),
          child: Row(
            children: [
              // Back button
              IconButton(
                onPressed: () => Navigator.of(context).pop(),
                icon: const Icon(Icons.arrow_back_ios_new,
                    color: Colors.white, size: 20),
                tooltip: '返回',
              ),
              const SizedBox(width: 4),
              // Camera label
              Expanded(
                child: Row(
                  children: [
                    Icon(
                      _activeCameraIndex == 0
                          ? Icons.videocam
                          : Icons.videocam_outlined,
                      color: Colors.white70,
                      size: 18,
                    ),
                    const SizedBox(width: 6),
                    Text(
                      _activeCameraId,
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 15,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                  ],
                ),
              ),
              // Connection quality badge
              _ConnectionBadge(isConnected: isConnected),
              const SizedBox(width: 8),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildBottomBar(BuildContext context, bool isConnected) {
    return Container(
      decoration: const BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.bottomCenter,
          end: Alignment.topCenter,
          colors: [Colors.black87, Colors.transparent],
        ),
      ),
      child: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              // Camera switch
              _ControlButton(
                icon: Icons.cameraswitch_outlined,
                label: '切换',
                onTap: _toggleCamera,
              ),
              // Reconnect button (only if disconnected)
              if (!isConnected)
                _ControlButton(
                  icon: Icons.refresh,
                  label: '重连',
                  highlight: true,
                  onTap: () {
                    HapticFeedback.mediumImpact();
                    Navigator.of(context).pushNamedAndRemoveUntil(
                      '/main',
                      (_) => false,
                    );
                  },
                ),
              // Fullscreen
              _ControlButton(
                icon: _isFullscreen ? Icons.fullscreen_exit : Icons.fullscreen,
                label: _isFullscreen ? '退出全屏' : '全屏',
                onTap: _toggleFullscreen,
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildDisconnectedView() {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 80,
            height: 80,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Colors.white.withValues(alpha: 0.06),
            ),
            child: Icon(
              Icons.videocam_off,
              size: 36,
              color: Colors.white.withValues(alpha: 0.3),
            ),
          ),
          const SizedBox(height: 20),
          Text(
            '未连接到机器人',
            style: TextStyle(
              color: Colors.white.withValues(alpha: 0.6),
              fontSize: 16,
              fontWeight: FontWeight.w600,
            ),
          ),
          const SizedBox(height: 8),
          Text(
            '请先在扫描页面连接后再查看相机',
            style: TextStyle(
              color: Colors.white.withValues(alpha: 0.3),
              fontSize: 13,
            ),
          ),
        ],
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Connection quality badge
// ─────────────────────────────────────────────────────────────────────────────

class _ConnectionBadge extends StatelessWidget {
  final bool isConnected;

  const _ConnectionBadge({required this.isConnected});

  @override
  Widget build(BuildContext context) {
    final color = isConnected ? AppColors.success : AppColors.error;
    final label = isConnected ? '已连接' : '断开';

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.2),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withValues(alpha: 0.3)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 6,
            height: 6,
            decoration: BoxDecoration(color: color, shape: BoxShape.circle),
          ),
          const SizedBox(width: 5),
          Text(
            label,
            style: TextStyle(
              color: color,
              fontSize: 11,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera view wrapper
// ─────────────────────────────────────────────────────────────────────────────

class _CameraView extends StatelessWidget {
  final dynamic client;
  final String cameraId;

  const _CameraView({
    super.key,
    required this.client,
    required this.cameraId,
  });

  @override
  Widget build(BuildContext context) {
    return WebRTCVideoWidget(
      client: client,
      cameraId: cameraId,
      autoConnect: true,
      fit: BoxFit.contain,
      placeholder: _buildPlaceholder(),
      loadingWidget: _buildLoading(),
    );
  }

  Widget _buildPlaceholder() {
    return Container(
      color: Colors.black,
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              Icons.videocam_off,
              size: 44,
              color: Colors.white.withValues(alpha: 0.15),
            ),
            const SizedBox(height: 10),
            Text(
              '等待视频流...',
              style: TextStyle(
                color: Colors.white.withValues(alpha: 0.3),
                fontSize: 14,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildLoading() {
    return Container(
      color: Colors.black,
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            SizedBox(
              width: 32,
              height: 32,
              child: CircularProgressIndicator(
                color: Colors.white.withValues(alpha: 0.4),
                strokeWidth: 2.5,
              ),
            ),
            const SizedBox(height: 16),
            Text(
              '正在连接相机...',
              style: TextStyle(
                color: Colors.white.withValues(alpha: 0.5),
                fontSize: 14,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Circular control button
// ─────────────────────────────────────────────────────────────────────────────

class _ControlButton extends StatelessWidget {
  final IconData icon;
  final String label;
  final VoidCallback onTap;
  final bool highlight;

  const _ControlButton({
    required this.icon,
    required this.label,
    required this.onTap,
    this.highlight = false,
  });

  @override
  Widget build(BuildContext context) {
    final bgColor = highlight
        ? AppColors.primary.withValues(alpha: 0.3)
        : Colors.white.withValues(alpha: 0.12);
    final borderColor = highlight
        ? AppColors.primary.withValues(alpha: 0.5)
        : Colors.white.withValues(alpha: 0.25);

    return GestureDetector(
      onTap: onTap,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 52,
            height: 52,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: bgColor,
              border: Border.all(color: borderColor, width: 1),
            ),
            child: Icon(icon, color: Colors.white, size: 22),
          ),
          const SizedBox(height: 6),
          Text(
            label,
            style: TextStyle(
              color: Colors.white.withValues(alpha: 0.7),
              fontSize: 11,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
    );
  }
}
