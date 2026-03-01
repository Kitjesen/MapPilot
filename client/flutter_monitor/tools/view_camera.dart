#!/usr/bin/env dart
/// 摄像头可视化 — 本地 HTTP 服务器，浏览器打开即可看视频流
///
/// 用法: dart run tools/view_camera.dart [robotHost] [robotPort] [localHttpPort]
///   默认: robotHost=192.168.66.190  robotPort=50051  localHttpPort=8888
///
/// 启动后浏览器打开 http://localhost:8888 即可看实时画面

import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/common.pb.dart';

late List<int> _latestFrame;
bool _hasFrame = false;
int _frameCount = 0;
int _totalBytes = 0;
final _sw = Stopwatch();
final _frameNotifier = StreamController<void>.broadcast();

void main(List<String> args) async {
  final robotHost = args.isNotEmpty ? args[0] : '192.168.66.190';
  final robotPort = args.length > 1 ? int.parse(args[1]) : 50051;
  final httpPort = args.length > 2 ? int.parse(args[2]) : 8888;

  print('══════════════════════════════════════════');
  print('  Camera Viewer');
  print('  Robot: $robotHost:$robotPort');
  print('  Open: http://localhost:$httpPort');
  print('══════════════════════════════════════════\n');

  // ── Start gRPC camera subscription ──
  final channel = ClientChannel(robotHost, port: robotPort,
    options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final client = DataServiceClient(channel);

  final rid = Random().nextInt(999999).toString().padLeft(6, '0');
  final request = SubscribeRequest()
    ..base = (RequestBase()..requestId = 'viewer-$rid')
    ..resourceId = (ResourceId()
      ..name = 'front'
      ..type = ResourceType.RESOURCE_TYPE_CAMERA)
    ..profile = (SubscribeProfile()..frequency = 30.0);

  _sw.start();

  // Subscribe in background
  _subscribeCamera(client, request);

  // ── Start HTTP server ──
  final server = await HttpServer.bind(InternetAddress.anyIPv4, httpPort);
  print('[HTTP] Listening on http://localhost:$httpPort');
  print('[HTTP] Press Ctrl+C to stop\n');

  await for (final req in server) {
    if (req.uri.path == '/stream') {
      _serveMjpeg(req);
    } else if (req.uri.path == '/snapshot') {
      _serveSnapshot(req);
    } else if (req.uri.path == '/stats') {
      _serveStats(req);
    } else {
      _serveHtml(req, httpPort);
    }
  }
}

void _subscribeCamera(DataServiceClient client, SubscribeRequest request) async {
  print('[gRPC] Subscribing to camera...');
  try {
    await for (final chunk in client.subscribe(request)) {
      _latestFrame = chunk.data;
      _hasFrame = true;
      _frameCount++;
      _totalBytes += chunk.data.length;
      _frameNotifier.add(null); // notify MJPEG clients

      if (_frameCount == 1) {
        print('[gRPC] First frame: ${chunk.data.length} bytes');
      }
      if (_frameCount % 100 == 0) {
        final elapsed = _sw.elapsedMilliseconds / 1000.0;
        print('[gRPC] $_frameCount frames | ${(_frameCount / elapsed).toStringAsFixed(1)} fps | ${(_totalBytes / 1024 / 1024).toStringAsFixed(1)} MB');
      }
    }
  } catch (e) {
    print('[gRPC] ERROR: $e');
  }
}

void _serveMjpeg(HttpRequest req) async {
  print('[HTTP] MJPEG client connected: ${req.connectionInfo?.remoteAddress}');
  req.response.headers.set('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
  req.response.headers.set('Cache-Control', 'no-cache');
  req.response.headers.set('Connection', 'keep-alive');
  req.response.headers.set('Access-Control-Allow-Origin', '*');

  try {
    await for (final _ in _frameNotifier.stream) {
      if (!_hasFrame) continue;
      final frame = _latestFrame;
      req.response.add('--frame\r\n'.codeUnits);
      req.response.add('Content-Type: image/jpeg\r\n'.codeUnits);
      req.response.add('Content-Length: ${frame.length}\r\n'.codeUnits);
      req.response.add('\r\n'.codeUnits);
      req.response.add(frame);
      req.response.add('\r\n'.codeUnits);
      await req.response.flush();
    }
  } catch (_) {
    print('[HTTP] MJPEG client disconnected');
  }
}

void _serveStats(HttpRequest req) {
  final elapsed = _sw.elapsedMilliseconds / 1000.0;
  final fps = elapsed > 0 ? _frameCount / elapsed : 0.0;
  final avgSize = _frameCount > 0 ? _totalBytes / _frameCount : 0;
  req.response.headers.set('Content-Type', 'application/json');
  req.response.headers.set('Cache-Control', 'no-cache');
  req.response.headers.set('Access-Control-Allow-Origin', '*');
  req.response.write('{"frames":$_frameCount,"fps":${fps.toStringAsFixed(1)},"avgBytes":$avgSize,"totalMB":${(_totalBytes/1024/1024).toStringAsFixed(1)}}');
  req.response.close();
}

void _serveSnapshot(HttpRequest req) {
  if (_hasFrame) {
    req.response.headers.set('Content-Type', 'image/jpeg');
    req.response.headers.set('Cache-Control', 'no-cache');
    req.response.add(_latestFrame);
  } else {
    req.response.statusCode = 503;
    req.response.write('No frame yet');
  }
  req.response.close();
}

void _serveHtml(HttpRequest req, int port) {
  req.response.headers.set('Content-Type', 'text/html; charset=utf-8');
  req.response.write('''<!DOCTYPE html>
<html>
<head>
  <title>Robot Camera</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #1a1a2e; color: #eee; font-family: system-ui, sans-serif;
      display: flex; flex-direction: column; align-items: center;
      min-height: 100vh; padding: 20px;
    }
    h1 { margin-bottom: 10px; color: #00d4aa; font-size: 24px; }
    .info { color: #888; margin-bottom: 20px; font-size: 14px; }
    .container {
      background: #16213e; border-radius: 12px; padding: 12px;
      box-shadow: 0 8px 32px rgba(0,0,0,0.3); max-width: 960px; width: 100%;
    }
    img {
      width: 100%; border-radius: 8px; display: block;
      background: #0f3460;
    }
    .stats {
      margin-top: 12px; padding: 8px 12px; background: #0f3460;
      border-radius: 6px; font-size: 13px; color: #aaa;
      display: flex; gap: 20px;
    }
    .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
      background: #00d4aa; margin-right: 6px; animation: pulse 1s infinite; }
    @keyframes pulse { 50% { opacity: 0.3; } }
  </style>
</head>
<body>
  <h1>Robot Camera Live</h1>
  <div class="info">gRPC Subscribe -> MJPEG Stream</div>
  <div class="container">
    <img id="cam" src="/stream" alt="Camera Feed" />
    <div class="stats">
      <span><span class="dot"></span>LIVE</span>
      <span id="fps">--</span>
      <span id="size">--</span>
    </div>
  </div>
  <script>
    const fpsEl = document.getElementById('fps');
    const sizeEl = document.getElementById('size');
    setInterval(async () => {
      try {
        const r = await fetch('/stats');
        if (r.ok) {
          const s = await r.json();
          fpsEl.textContent = s.fps + ' fps (gRPC source)';
          sizeEl.textContent = (s.avgBytes/1024).toFixed(0) + ' KB/frame | ' + s.totalMB + ' MB total';
        }
      } catch(_) {}
    }, 1000);
  </script>
</body>
</html>''');
  req.response.close();
}
