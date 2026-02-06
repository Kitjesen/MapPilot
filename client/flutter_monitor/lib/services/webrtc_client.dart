// webrtc_client.dart
// WebRTC client for real-time video/audio streaming via gRPC signaling

import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_webrtc/flutter_webrtc.dart';
import 'package:grpc/grpc.dart';

import 'package:robot_proto/src/data.pbgrpc.dart';

/// WebRTC 连接状态
enum WebRTCConnectionState {
  disconnected,
  connecting,
  connected,
  failed,
}

/// WebRTC 客户端 - 通过 gRPC 信令进行 WebRTC 连接
class WebRTCClient {
  final DataServiceClient _dataClient;
  final String sessionId;
  
  RTCPeerConnection? _peerConnection;
  MediaStream? _remoteStream;
  RTCVideoRenderer? _videoRenderer;
  
  StreamController<WebRTCSignal>? _signalController;
  ResponseStream<WebRTCSignal>? _signalingStream;
  
  final _connectionStateController = StreamController<WebRTCConnectionState>.broadcast();
  WebRTCConnectionState _connectionState = WebRTCConnectionState.disconnected;
  
  // ICE 服务器配置
  final Map<String, dynamic> _iceServers = {
    'iceServers': [
      {'urls': 'stun:stun.l.google.com:19302'},
      {'urls': 'stun:stun1.l.google.com:19302'},
    ]
  };
  
  // 媒体约束
  final Map<String, dynamic> _offerSdpConstraints = {
    'mandatory': {
      'OfferToReceiveAudio': true,
      'OfferToReceiveVideo': true,
    },
    'optional': [],
  };
  
  WebRTCClient({
    required DataServiceClient dataClient,
    required this.sessionId,
  }) : _dataClient = dataClient;
  
  /// 连接状态流
  Stream<WebRTCConnectionState> get connectionStateStream => 
      _connectionStateController.stream;
  
  /// 当前连接状态
  WebRTCConnectionState get connectionState => _connectionState;
  
  /// 远程视频流
  MediaStream? get remoteStream => _remoteStream;
  
  /// 视频渲染器
  RTCVideoRenderer? get videoRenderer => _videoRenderer;
  
  /// 初始化并连接
  Future<void> connect({
    bool videoEnabled = true,
    bool audioEnabled = false,
    String cameraId = 'front',
    int width = 640,
    int height = 480,
    int fps = 30,
  }) async {
    if (_connectionState == WebRTCConnectionState.connecting ||
        _connectionState == WebRTCConnectionState.connected) {
      debugPrint('WebRTC: Already connecting or connected');
      return;
    }
    
    _setConnectionState(WebRTCConnectionState.connecting);
    
    try {
      // 1. 初始化视频渲染器
      _videoRenderer = RTCVideoRenderer();
      await _videoRenderer!.initialize();
      
      // 2. 创建 PeerConnection
      _peerConnection = await createPeerConnection(_iceServers);
      
      // 3. 设置回调
      _peerConnection!.onIceCandidate = _onIceCandidate;
      _peerConnection!.onIceConnectionState = _onIceConnectionState;
      _peerConnection!.onTrack = _onTrack;
      _peerConnection!.onAddStream = _onAddStream;
      
      // 4. 创建信令流
      _signalController = StreamController<WebRTCSignal>();
      _signalingStream = _dataClient.webRTCSignaling(_signalController!.stream);
      
      // 5. 监听服务器信令
      _listenToSignaling();
      
      // 6. 发送 Offer
      await _createAndSendOffer(
        videoEnabled: videoEnabled,
        audioEnabled: audioEnabled,
        cameraId: cameraId,
        width: width,
        height: height,
        fps: fps,
      );
      
    } catch (e) {
      debugPrint('WebRTC: Connection failed: $e');
      _setConnectionState(WebRTCConnectionState.failed);
      rethrow;
    }
  }
  
  /// 断开连接
  Future<void> disconnect() async {
    debugPrint('WebRTC: Disconnecting...');
    
    // 发送挂断信号
    if (_signalController != null && !_signalController!.isClosed) {
      final hangup = WebRTCSignal()
        ..sessionId = sessionId
        ..type = WebRTCSignalType.WEBRTC_SIGNAL_TYPE_HANGUP;
      _signalController!.add(hangup);
      
      // 等待一小段时间让信号发送
      await Future.delayed(const Duration(milliseconds: 100));
    }
    
    await _cleanup();
    _setConnectionState(WebRTCConnectionState.disconnected);
  }
  
  /// 创建并发送 Offer
  Future<void> _createAndSendOffer({
    required bool videoEnabled,
    required bool audioEnabled,
    required String cameraId,
    required int width,
    required int height,
    required int fps,
  }) async {
    // 创建 Offer
    final description = await _peerConnection!.createOffer(_offerSdpConstraints);
    await _peerConnection!.setLocalDescription(description);
    
    debugPrint('WebRTC: Created offer SDP');
    
    // 构建配置
    final config = WebRTCSessionConfig()
      ..videoEnabled = videoEnabled
      ..audioEnabled = audioEnabled
      ..cameraId = cameraId
      ..videoProfile = (VideoProfile()
        ..width = width
        ..height = height
        ..fps = fps
        ..bitrateKbps = 2000
        ..codec = 'H264');
    
    // 发送 Offer 信令
    final offerSignal = WebRTCSignal()
      ..sessionId = sessionId
      ..type = WebRTCSignalType.WEBRTC_SIGNAL_TYPE_OFFER
      ..sdp = description.sdp ?? ''
      ..config = config;
    
    _signalController!.add(offerSignal);
    debugPrint('WebRTC: Sent offer to server');
  }
  
  /// 监听服务器信令
  void _listenToSignaling() {
    _signalingStream?.listen(
      (signal) async {
        debugPrint('WebRTC: Received signal type: ${signal.type}');
        
        switch (signal.type) {
          case WebRTCSignalType.WEBRTC_SIGNAL_TYPE_ANSWER:
            await _handleAnswer(signal);
            break;
            
          case WebRTCSignalType.WEBRTC_SIGNAL_TYPE_ICE_CANDIDATE:
            await _handleRemoteIceCandidate(signal);
            break;
            
          case WebRTCSignalType.WEBRTC_SIGNAL_TYPE_HANGUP:
            debugPrint('WebRTC: Received hangup from server');
            await _cleanup();
            _setConnectionState(WebRTCConnectionState.disconnected);
            break;
            
          default:
            debugPrint('WebRTC: Unknown signal type: ${signal.type}');
        }
      },
      onError: (error) {
        debugPrint('WebRTC: Signaling error: $error');
        _setConnectionState(WebRTCConnectionState.failed);
      },
      onDone: () {
        debugPrint('WebRTC: Signaling stream closed');
        if (_connectionState == WebRTCConnectionState.connected ||
            _connectionState == WebRTCConnectionState.connecting) {
          _setConnectionState(WebRTCConnectionState.disconnected);
        }
      },
    );
  }
  
  /// 处理 Answer
  Future<void> _handleAnswer(WebRTCSignal signal) async {
    debugPrint('WebRTC: Setting remote description (answer)');
    
    final description = RTCSessionDescription(
      signal.sdp,
      'answer',
    );
    
    await _peerConnection!.setRemoteDescription(description);
    debugPrint('WebRTC: Remote description set successfully');
  }
  
  /// 处理远程 ICE candidate
  Future<void> _handleRemoteIceCandidate(WebRTCSignal signal) async {
    if (signal.iceCandidate.isEmpty) return;
    
    debugPrint('WebRTC: Adding remote ICE candidate');
    
    final candidate = RTCIceCandidate(
      signal.iceCandidate,
      signal.iceMid,
      signal.iceMlineIndex,
    );
    
    await _peerConnection!.addCandidate(candidate);
  }
  
  /// 本地 ICE candidate 回调
  void _onIceCandidate(RTCIceCandidate candidate) {
    debugPrint('WebRTC: Local ICE candidate: ${candidate.candidate}');
    
    if (_signalController == null || _signalController!.isClosed) return;
    
    final signal = WebRTCSignal()
      ..sessionId = sessionId
      ..type = WebRTCSignalType.WEBRTC_SIGNAL_TYPE_ICE_CANDIDATE
      ..iceCandidate = candidate.candidate ?? ''
      ..iceMid = candidate.sdpMid ?? ''
      ..iceMlineIndex = candidate.sdpMLineIndex ?? 0;
    
    _signalController!.add(signal);
  }
  
  /// ICE 连接状态变化回调
  void _onIceConnectionState(RTCIceConnectionState state) {
    debugPrint('WebRTC: ICE connection state: $state');
    
    switch (state) {
      case RTCIceConnectionState.RTCIceConnectionStateConnected:
      case RTCIceConnectionState.RTCIceConnectionStateCompleted:
        _setConnectionState(WebRTCConnectionState.connected);
        break;
        
      case RTCIceConnectionState.RTCIceConnectionStateFailed:
        _setConnectionState(WebRTCConnectionState.failed);
        break;
        
      case RTCIceConnectionState.RTCIceConnectionStateDisconnected:
      case RTCIceConnectionState.RTCIceConnectionStateClosed:
        _setConnectionState(WebRTCConnectionState.disconnected);
        break;
        
      default:
        break;
    }
  }
  
  /// 接收到媒体轨道回调
  void _onTrack(RTCTrackEvent event) {
    debugPrint('WebRTC: Received track: ${event.track.kind}');
    
    if (event.track.kind == 'video' && event.streams.isNotEmpty) {
      _remoteStream = event.streams[0];
      _videoRenderer?.srcObject = _remoteStream;
      debugPrint('WebRTC: Video stream attached to renderer');
    }
  }
  
  /// 接收到媒体流回调 (旧 API 兼容)
  void _onAddStream(MediaStream stream) {
    debugPrint('WebRTC: Received stream: ${stream.id}');
    
    _remoteStream = stream;
    _videoRenderer?.srcObject = stream;
    debugPrint('WebRTC: Video stream attached to renderer (legacy)');
  }
  
  /// 设置连接状态
  void _setConnectionState(WebRTCConnectionState state) {
    if (_connectionState != state) {
      _connectionState = state;
      _connectionStateController.add(state);
      debugPrint('WebRTC: Connection state changed to: $state');
    }
  }
  
  /// 清理资源
  Future<void> _cleanup() async {
    await _signalController?.close();
    _signalController = null;
    _signalingStream = null;
    
    _remoteStream?.getTracks().forEach((track) => track.stop());
    _remoteStream = null;
    
    await _peerConnection?.close();
    _peerConnection = null;
    
    await _videoRenderer?.dispose();
    _videoRenderer = null;
  }
  
  /// 释放资源
  void dispose() {
    _cleanup();
    _connectionStateController.close();
  }
}
