// webrtc_video_widget.dart
// Widget for displaying WebRTC video stream

import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_webrtc/flutter_webrtc.dart';

import '../services/robot_client_base.dart';
import '../services/webrtc_client.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';

/// WebRTC 视频显示组件
class WebRTCVideoWidget extends StatefulWidget {
  final RobotClientBase client;
  final String cameraId;
  final bool autoConnect;
  final BoxFit fit;
  final Widget? placeholder;
  final Widget? loadingWidget;
  final Widget? errorWidget;

  const WebRTCVideoWidget({
    super.key,
    required this.client,
    this.cameraId = 'front',
    this.autoConnect = true,
    this.fit = BoxFit.cover,
    this.placeholder,
    this.loadingWidget,
    this.errorWidget,
  });

  @override
  State<WebRTCVideoWidget> createState() => _WebRTCVideoWidgetState();
}

class _WebRTCVideoWidgetState extends State<WebRTCVideoWidget> {
  WebRTCClient? _webrtcClient;
  RTCVideoRenderer? _renderer;
  WebRTCConnectionState _connectionState = WebRTCConnectionState.disconnected;
  StreamSubscription<WebRTCConnectionState>? _stateSubscription;
  String? _errorMessage;
  bool _isInitialized = false;

  @override
  void initState() {
    super.initState();
    if (widget.autoConnect) {
      _initializeWebRTC();
    }
  }

  @override
  void dispose() {
    _stateSubscription?.cancel();
    _webrtcClient?.dispose();
    super.dispose();
  }

  Future<void> _initializeWebRTC() async {
    // 检查客户端是否支持 WebRTC (真实客户端才支持)
    final dataClient = widget.client.dataServiceClient;
    if (dataClient == null) {
      setState(() {
        _errorMessage = 'WebRTC not available in mock mode';
        _connectionState = WebRTCConnectionState.failed;
      });
      return;
    }

    // 生成唯一 session ID
    final sessionId = 'webrtc_${DateTime.now().millisecondsSinceEpoch}';

    _webrtcClient = WebRTCClient(
      dataClient: dataClient,
      sessionId: sessionId,
    );

    // 监听连接状态
    _stateSubscription = _webrtcClient!.connectionStateStream.listen((state) {
      if (mounted) {
        setState(() {
          _connectionState = state;
          if (state == WebRTCConnectionState.connected) {
            _renderer = _webrtcClient!.videoRenderer;
            _isInitialized = true;
          }
        });
      }
    });

    // 开始连接
    try {
      await _webrtcClient!.connect(
        videoEnabled: true,
        audioEnabled: false,
        cameraId: widget.cameraId,
      );
    } catch (e) {
      if (mounted) {
        setState(() {
          _errorMessage = e.toString();
          _connectionState = WebRTCConnectionState.failed;
        });
      }
    }
  }

  Future<void> _reconnect() async {
    setState(() {
      _connectionState = WebRTCConnectionState.disconnected;
      _errorMessage = null;
    });

    await _webrtcClient?.disconnect();
    _webrtcClient?.dispose();
    _webrtcClient = null;

    await _initializeWebRTC();
  }

  @override
  Widget build(BuildContext context) {
    switch (_connectionState) {
      case WebRTCConnectionState.connecting:
        return widget.loadingWidget ?? _buildLoadingWidget();

      case WebRTCConnectionState.connected:
        if (_renderer != null && _isInitialized) {
          return RTCVideoView(
            _renderer!,
            objectFit: _convertFit(widget.fit),
            mirror: false,
          );
        }
        return widget.placeholder ?? _buildPlaceholder();

      case WebRTCConnectionState.failed:
        return widget.errorWidget ?? _buildErrorWidget();

      case WebRTCConnectionState.disconnected:
      default:
        return widget.placeholder ?? _buildPlaceholder();
    }
  }

  RTCVideoViewObjectFit _convertFit(BoxFit fit) {
    switch (fit) {
      case BoxFit.contain:
        return RTCVideoViewObjectFit.RTCVideoViewObjectFitContain;
      case BoxFit.cover:
      default:
        return RTCVideoViewObjectFit.RTCVideoViewObjectFitCover;
    }
  }

  Widget _buildLoadingWidget() {
    return Container(
      color: Colors.black,
      child: const Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            CircularProgressIndicator(
              color: Colors.white54,
              strokeWidth: 2,
            ),
            SizedBox(height: 16),
            Text(
              'Connecting to camera...',
              style: TextStyle(
                color: Colors.white54,
                fontSize: 14,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPlaceholder() {
    return Container(
      color: Colors.black87,
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              Icons.videocam_off,
              size: 48,
              color: Colors.white.withOpacity(0.3),
            ),
            const SizedBox(height: 8),
            Text(
              'No video feed',
              style: TextStyle(
                color: Colors.white.withOpacity(0.3),
                fontSize: 14,
              ),
            ),
            const SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _initializeWebRTC,
              icon: const Icon(Icons.play_arrow),
              label: const Text('Start Video'),
              style: ElevatedButton.styleFrom(
                backgroundColor: const Color(0xFF007AFF),
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildErrorWidget() {
    return Container(
      color: Colors.black87,
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Icon(
              Icons.error_outline,
              size: 48,
              color: Colors.red,
            ),
            const SizedBox(height: 8),
            const Text(
              'Connection failed',
              style: TextStyle(
                color: Colors.white70,
                fontSize: 16,
                fontWeight: FontWeight.w500,
              ),
            ),
            if (_errorMessage != null) ...[
              const SizedBox(height: 4),
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 32),
                child: Text(
                  _errorMessage!,
                  style: const TextStyle(
                    color: Colors.white38,
                    fontSize: 12,
                  ),
                  textAlign: TextAlign.center,
                  maxLines: 2,
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
            const SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _reconnect,
              icon: const Icon(Icons.refresh),
              label: const Text('Retry'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.red.shade700,
                foregroundColor: Colors.white,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

