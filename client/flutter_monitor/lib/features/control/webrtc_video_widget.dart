// webrtc_video_widget.dart
// Widget for displaying video stream with automatic WebRTC → gRPC fallback.
// Attempts WebRTC DataChannel first; if it fails or times out, falls back
// to gRPC Subscribe (direct JPEG streaming) which is more reliable on
// platforms where WebRTC native support is limited (e.g. Windows desktop).

import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_webrtc/flutter_webrtc.dart';
import 'package:permission_handler/permission_handler.dart';

import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/features/control/webrtc_client.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/data.pb.dart';

/// Camera video widget with WebRTC + gRPC Subscribe fallback.
///
/// Strategy:
///   1. Start gRPC Subscribe immediately (reliable, works across NAT)
///   2. Try WebRTC DataChannel in parallel (low-latency, peer-to-peer)
///   3. If WebRTC delivers frames, switch to WebRTC and stop gRPC
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

enum _VideoMode { none, connecting, webrtc, grpcFallback, failed }

class _WebRTCVideoWidgetState extends State<WebRTCVideoWidget> {
  // ── WebRTC ──
  WebRTCClient? _webrtcClient;
  RTCVideoRenderer? _renderer;
  StreamSubscription<WebRTCConnectionState>? _stateSubscription;
  StreamSubscription<Uint8List>? _frameSubscription;

  // ── gRPC Subscribe fallback ──
  StreamSubscription<DataChunk>? _grpcSubscription;

  // ── Shared state ──
  Uint8List? _latestJpegFrame;
  String? _errorMessage;
  _VideoMode _mode = _VideoMode.none;
  Timer? _webrtcTimeout;
  int _grpcFrameCount = 0;

  @override
  void initState() {
    super.initState();
    if (widget.autoConnect) {
      _startVideoStream();
    }
  }

  @override
  void didUpdateWidget(covariant WebRTCVideoWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.cameraId != widget.cameraId) {
      _fullCleanup();
      _startVideoStream();
    }
  }

  @override
  void dispose() {
    _fullCleanup();
    super.dispose();
  }

  void _fullCleanup() {
    _webrtcTimeout?.cancel();
    _grpcSubscription?.cancel();
    _grpcSubscription = null;
    _cleanupWebRTC();
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Entry: try WebRTC first, fallback to gRPC on timeout
  // ════════════════════════════════════════════════════════════════════════

  Future<void> _startVideoStream() async {
    setState(() {
      _mode = _VideoMode.connecting;
      _errorMessage = null;
      _latestJpegFrame = null;
      _grpcFrameCount = 0;
    });

    // 直接启动 gRPC Subscribe（可靠路径），同时尝试 WebRTC
    // 如果 WebRTC DataChannel 先出帧则自动切换
    debugPrint('VideoWidget: Starting gRPC Subscribe as primary path');
    _startGrpcFallback();

    // 同时尝试 WebRTC (可选的低延迟路径)
    _tryWebRTC().then((ok) {
      if (!ok) {
        debugPrint('VideoWidget: WebRTC init failed, gRPC already active');
      }
    });
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Path A: WebRTC DataChannel
  // ════════════════════════════════════════════════════════════════════════

  Future<bool> _tryWebRTC() async {
    final dataClient = widget.client.dataServiceClient;
    if (dataClient == null) {
      return false;
    }

    // Runtime permissions (Android)
    try {
      await [Permission.camera, Permission.microphone].request();
    } catch (_) {}

    final sessionId = 'webrtc_${DateTime.now().millisecondsSinceEpoch}';
    _webrtcClient = WebRTCClient(
      dataClient: dataClient,
      sessionId: sessionId,
    );

    // Listen for JPEG frames (DataChannel)
    _frameSubscription = _webrtcClient!.videoFrameStream.listen((frameData) {
      if (mounted && frameData.isNotEmpty) {
        // WebRTC 出帧了，切换到 WebRTC 模式并停掉 gRPC
        if (_mode != _VideoMode.webrtc) {
          debugPrint('VideoWidget: WebRTC delivering frames, stopping gRPC');
          _grpcSubscription?.cancel();
          _grpcSubscription = null;
        }
        setState(() {
          _latestJpegFrame = frameData;
          _mode = _VideoMode.webrtc;
        });
      }
    });

    // Listen for connection state
    _stateSubscription = _webrtcClient!.connectionStateStream.listen((state) {
      if (!mounted) return;
      if (state == WebRTCConnectionState.connected) {
        _renderer = _webrtcClient!.videoRenderer;
      }
      if (state == WebRTCConnectionState.failed) {
        debugPrint('VideoWidget: WebRTC state=failed, falling back');
        _webrtcTimeout?.cancel();
        _cleanupWebRTC();
        _startGrpcFallback();
      }
    });

    try {
      await _webrtcClient!.connect(
        videoEnabled: true,
        audioEnabled: false,
        cameraId: widget.cameraId,
      );
      return true;
    } catch (e) {
      debugPrint('VideoWidget: WebRTC connect exception: $e');
      return false;
    }
  }

  void _cleanupWebRTC() {
    _frameSubscription?.cancel();
    _frameSubscription = null;
    _stateSubscription?.cancel();
    _stateSubscription = null;
    _webrtcClient?.dispose();
    _webrtcClient = null;
    _renderer = null;
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Path B: gRPC Subscribe fallback (direct JPEG stream)
  // ════════════════════════════════════════════════════════════════════════

  void _startGrpcFallback() {
    if (_grpcSubscription != null) return; // 已在运行

    debugPrint('VideoWidget: Starting gRPC Subscribe for '
        'camera "${widget.cameraId}"');

    // 使用简洁的 camera name（如 'front'），让服务端解析实际 topic
    // 与 test/data/camera.dart 测试工具保持一致
    final resourceId = ResourceId()
      ..type = ResourceType.RESOURCE_TYPE_CAMERA
      ..name = widget.cameraId;

    try {
      _grpcSubscription = widget.client.subscribeToResource(resourceId).listen(
        (chunk) {
          if (mounted && chunk.data.isNotEmpty) {
            _grpcFrameCount++;
            if (_grpcFrameCount == 1) {
              final isJpeg = chunk.data.length > 2 &&
                  chunk.data[0] == 0xFF && chunk.data[1] == 0xD8;
              debugPrint('VideoWidget: gRPC first frame! '
                  '${chunk.data.length} bytes, JPEG=$isJpeg');
            }
            setState(() {
              _latestJpegFrame = Uint8List.fromList(chunk.data);
              _mode = _VideoMode.grpcFallback;
            });
          }
        },
        onError: (error) {
          debugPrint('VideoWidget: gRPC camera error: $error');
          _grpcSubscription = null;
          // 自动重连，不直接标记 failed
          if (mounted) {
            Future.delayed(const Duration(seconds: 2), () {
              if (mounted && _grpcSubscription == null) {
                debugPrint('VideoWidget: gRPC auto-reconnecting...');
                _startGrpcFallback();
              }
            });
          }
        },
        onDone: () {
          debugPrint('VideoWidget: gRPC stream ended');
          _grpcSubscription = null;
          if (mounted) {
            Future.delayed(const Duration(seconds: 1), () {
              if (mounted && _grpcSubscription == null) {
                _startGrpcFallback();
              }
            });
          }
        },
      );
    } catch (e) {
      debugPrint('VideoWidget: gRPC Subscribe failed: $e');
      if (mounted) {
        setState(() {
          _errorMessage = 'Failed to subscribe: $e';
          _mode = _VideoMode.failed;
        });
      }
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Reconnect
  // ════════════════════════════════════════════════════════════════════════

  Future<void> _reconnect() async {
    _fullCleanup();
    setState(() {
      _latestJpegFrame = null;
      _errorMessage = null;
      _mode = _VideoMode.none;
    });
    await _startVideoStream();
  }

  // ════════════════════════════════════════════════════════════════════════
  //  UI
  // ════════════════════════════════════════════════════════════════════════

  @override
  Widget build(BuildContext context) {
    switch (_mode) {
      case _VideoMode.none:
      case _VideoMode.connecting:
        return widget.loadingWidget ?? _buildLoadingWidget();

      case _VideoMode.webrtc:
      case _VideoMode.grpcFallback:
        if (_latestJpegFrame != null) {
          return Container(
            color: Colors.black,
            child: Center(
              child: Image.memory(
                _latestJpegFrame!,
                fit: widget.fit,
                gaplessPlayback: true,
                errorBuilder: (context, error, stackTrace) {
                  return widget.placeholder ?? _buildPlaceholder();
                },
              ),
            ),
          );
        }
        // Fallback: RTCVideoView (WebRTC video track mode)
        if (_renderer != null) {
          return RTCVideoView(
            _renderer!,
            objectFit: _convertFit(widget.fit),
            mirror: false,
          );
        }
        return widget.placeholder ?? _buildPlaceholder();

      case _VideoMode.failed:
        return widget.errorWidget ?? _buildErrorWidget();
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
              '正在连接相机...',
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
              color: Colors.white.withValues(alpha: 0.3),
            ),
            const SizedBox(height: 8),
            Text(
              '无视频信号',
              style: TextStyle(
                color: Colors.white.withValues(alpha: 0.3),
                fontSize: 14,
              ),
            ),
            const SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _reconnect,
              icon: const Icon(Icons.play_arrow),
              label: const Text('开始播放'),
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
              '连接失败',
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
                  maxLines: 3,
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
            const SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _reconnect,
              icon: const Icon(Icons.refresh),
              label: const Text('重试'),
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
