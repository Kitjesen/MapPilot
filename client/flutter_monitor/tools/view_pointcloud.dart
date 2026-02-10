#!/usr/bin/env dart
/// 点云可视化 — 本地 HTTP 服务器, 浏览器打开即可看实时 2D 鸟瞰点云
///
/// 用法: dart run tools/view_pointcloud.dart [robotHost] [robotPort] [httpPort]
///   默认: robotHost=192.168.66.190  robotPort=50051  httpPort=8889
///
/// 启动后浏览器打开 http://localhost:8889 即可看实时点云
///
/// 支持切换: local_cloud (实时LiDAR) / terrain (地形图)
///
/// 数据格式 (来自 data_service.cpp SerializePointCloud2WithMeta):
///   header (20 bytes LE): width(4) height(4) point_step(4) row_step(4) num_fields(4)
///   data: raw PointCloud2 bytes (XYZ floats at offset 0,4,8 per point)

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math';
import 'dart:typed_data';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';

// ─── Stream state ───
class StreamState {
  final String name;
  List<double>? pointsJson; // flattened [x,y,z, x,y,z, ...]
  int frameCount = 0;
  int totalBytes = 0;
  int numPoints = 0;
  int pointStep = 0;
  final Stopwatch sw = Stopwatch();
  StreamSubscription? sub;
  bool active = false;
  String? error;

  StreamState(this.name);

  double get fps {
    final s = sw.elapsedMilliseconds / 1000.0;
    return s > 0 ? frameCount / s : 0;
  }
}

late DataServiceClient _client;
late ClientChannel _channel;
final Map<String, StreamState> _streams = {};
final _updateNotifier = StreamController<void>.broadcast();

// Current active stream name
String _activeStream = 'local_cloud';

void main(List<String> args) async {
  final robotHost = args.isNotEmpty ? args[0] : '192.168.66.190';
  final robotPort = args.length > 1 ? int.parse(args[1]) : 50051;
  final httpPort = args.length > 2 ? int.parse(args[2]) : 8889;

  print('══════════════════════════════════════════');
  print('  PointCloud Viewer');
  print('  Robot: $robotHost:$robotPort');
  print('  Open: http://localhost:$httpPort');
  print('══════════════════════════════════════════\n');

  _channel = ClientChannel(robotHost,
      port: robotPort,
      options: const ChannelOptions(
          credentials: ChannelCredentials.insecure(),
          connectionTimeout: Duration(seconds: 5)));
  _client = DataServiceClient(_channel);

  // Initialize stream states
  _streams['local_cloud'] = StreamState('local_cloud');
  _streams['/terrain_map'] = StreamState('/terrain_map');
  _streams['terrain'] = StreamState('terrain');

  // Start subscribing to the default stream
  _startStream('local_cloud');

  // HTTP server
  final server = await HttpServer.bind(InternetAddress.anyIPv4, httpPort);
  print('[HTTP] Listening on http://localhost:$httpPort\n');

  await for (final req in server) {
    final path = req.uri.path;
    if (path == '/data') {
      _servePointData(req);
    } else if (path == '/stats') {
      _serveStats(req);
    } else if (path == '/switch') {
      _handleSwitch(req);
    } else {
      _serveHtml(req, httpPort);
    }
  }
}

void _startStream(String name) {
  final state = _streams[name];
  if (state == null) return;

  // Cancel previous subscription if any
  state.sub?.cancel();
  state.frameCount = 0;
  state.totalBytes = 0;
  state.error = null;
  state.active = true;
  state.sw.reset();
  state.sw.start();

  final rid = Random().nextInt(999999).toString().padLeft(6, '0');
  final req = SubscribeRequest()
    ..base = (RequestBase()..requestId = 'pc-view-$rid')
    ..resourceId = (ResourceId()
      ..name = name
      ..type = ResourceType.RESOURCE_TYPE_POINTCLOUD)
    ..profile = (SubscribeProfile()..frequency = 10.0);

  print('[gRPC] Subscribing to "$name"...');

  try {
    final stream = _client.subscribe(req);
    state.sub = stream.listen(
      (chunk) {
        _processPointCloud(state, chunk.data);
        state.frameCount++;
        state.totalBytes += chunk.data.length;
        _updateNotifier.add(null);

        if (state.frameCount == 1) {
          print(
              '[gRPC:$name] First frame: ${chunk.data.length} bytes, ${state.numPoints} points, point_step=${state.pointStep}');
        }
        if (state.frameCount % 50 == 0) {
          print(
              '[gRPC:$name] ${state.frameCount} frames | ${state.fps.toStringAsFixed(1)} fps | ${state.numPoints} pts');
        }
      },
      onError: (e) {
        print('[gRPC:$name] ERROR: $e');
        state.error = e.toString();
        state.active = false;
      },
      onDone: () {
        print('[gRPC:$name] Stream ended');
        state.active = false;
      },
    );
  } catch (e) {
    print('[gRPC:$name] Failed to subscribe: $e');
    state.error = e.toString();
    state.active = false;
  }
}

void _processPointCloud(StreamState state, List<int> rawData) {
  if (rawData.length < 20) return;

  final bytes = Uint8List.fromList(rawData);
  final bd = ByteData.sublistView(bytes);

  // Parse 20-byte header (little-endian)
  final width = bd.getUint32(0, Endian.little);
  final height = bd.getUint32(4, Endian.little);
  final pointStep = bd.getUint32(8, Endian.little);
  // row_step at offset 12
  // num_fields at offset 16

  state.pointStep = pointStep;
  final dataOffset = 20;
  final dataLength = bytes.length - dataOffset;

  if (pointStep == 0) return;
  final numPoints = dataLength ~/ pointStep;
  state.numPoints = numPoints;

  // Extract xyz (first 3 floats in each point)
  // Downsample if too many points (for browser performance)
  final maxPoints = 50000;
  final step = numPoints > maxPoints ? numPoints ~/ maxPoints : 1;
  final outCount = (numPoints / step).ceil();

  final points = Float64List(outCount * 3);
  int idx = 0;

  for (int i = 0; i < numPoints && idx < outCount * 3; i += step) {
    final pOffset = dataOffset + i * pointStep;
    if (pOffset + 12 > bytes.length) break;

    final x = bd.getFloat32(pOffset + 0, Endian.little);
    final y = bd.getFloat32(pOffset + 4, Endian.little);
    final z = bd.getFloat32(pOffset + 8, Endian.little);

    // Skip NaN/Inf points
    if (x.isNaN || y.isNaN || z.isNaN || x.isInfinite || y.isInfinite || z.isInfinite) {
      continue;
    }

    points[idx++] = x;
    points[idx++] = y;
    points[idx++] = z;
  }

  state.pointsJson = points.sublist(0, idx).toList();
}

void _servePointData(HttpRequest req) {
  req.response.headers.set('Content-Type', 'application/json');
  req.response.headers.set('Cache-Control', 'no-cache');
  req.response.headers.set('Access-Control-Allow-Origin', '*');

  final state = _streams[_activeStream];
  if (state == null || state.pointsJson == null || state.pointsJson!.isEmpty) {
    req.response.write('{"points":[],"numPoints":0}');
  } else {
    // Send as compact JSON array
    final buf = StringBuffer('{"points":[');
    final pts = state.pointsJson!;
    for (int i = 0; i < pts.length; i++) {
      if (i > 0) buf.write(',');
      buf.write(pts[i].toStringAsFixed(3));
    }
    buf.write('],"numPoints":${state.numPoints},"pointStep":${state.pointStep}}');
    req.response.write(buf.toString());
  }
  req.response.close();
}

void _serveStats(HttpRequest req) {
  req.response.headers.set('Content-Type', 'application/json');
  req.response.headers.set('Cache-Control', 'no-cache');
  req.response.headers.set('Access-Control-Allow-Origin', '*');

  final state = _streams[_activeStream];
  final streams = <String, dynamic>{};
  for (final e in _streams.entries) {
    streams[e.key] = {
      'frames': e.value.frameCount,
      'fps': double.parse(e.value.fps.toStringAsFixed(1)),
      'numPoints': e.value.numPoints,
      'active': e.value.active,
      'error': e.value.error,
    };
  }

  req.response.write(jsonEncode({
    'activeStream': _activeStream,
    'frames': state?.frameCount ?? 0,
    'fps': state?.fps.toStringAsFixed(1) ?? '0',
    'numPoints': state?.numPoints ?? 0,
    'pointStep': state?.pointStep ?? 0,
    'totalKB': ((state?.totalBytes ?? 0) / 1024).round(),
    'streams': streams,
  }));
  req.response.close();
}

void _handleSwitch(HttpRequest req) async {
  final name = req.uri.queryParameters['name'];
  req.response.headers.set('Content-Type', 'application/json');
  req.response.headers.set('Access-Control-Allow-Origin', '*');

  if (name != null && _streams.containsKey(name)) {
    // Stop old stream
    _streams[_activeStream]?.sub?.cancel();
    _streams[_activeStream]?.active = false;

    _activeStream = name;
    _startStream(name);
    req.response.write('{"ok":true,"activeStream":"$name"}');
  } else {
    req.response
        .write('{"ok":false,"error":"Unknown stream","available":${jsonEncode(_streams.keys.toList())}}');
  }
  req.response.close();
}

void _serveHtml(HttpRequest req, int port) {
  req.response.headers.set('Content-Type', 'text/html; charset=utf-8');
  req.response.write('''<!DOCTYPE html>
<html>
<head>
  <title>PointCloud Viewer</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #0f0e1a; color: #e8e5f5; font-family: system-ui, -apple-system, sans-serif;
      display: flex; flex-direction: column; align-items: center;
      min-height: 100vh; padding: 16px;
    }
    h1 { font-size: 22px; font-weight: 800; color: #8b83ff; margin-bottom: 8px; }
    .header { display: flex; align-items: center; gap: 16px; margin-bottom: 12px; flex-wrap: wrap; justify-content: center; }
    .btn {
      padding: 8px 18px; border-radius: 10px; border: 2px solid #6c63ff;
      background: transparent; color: #8b83ff; font-weight: 600; font-size: 13px;
      cursor: pointer; transition: all 0.2s;
    }
    .btn:hover { background: #6c63ff22; }
    .btn.active { background: #6c63ff; color: white; box-shadow: 0 4px 16px #6c63ff40; }
    .container {
      background: #1a1830; border-radius: 16px; padding: 12px;
      box-shadow: 0 8px 32px rgba(108,99,255,0.08); max-width: 1000px; width: 100%;
    }
    canvas {
      width: 100%; border-radius: 12px; display: block;
      background: #13112a; cursor: grab;
    }
    canvas:active { cursor: grabbing; }
    .stats-bar {
      margin-top: 10px; padding: 10px 14px; background: #13112a;
      border-radius: 10px; font-size: 13px; color: #9ca3af;
      display: flex; gap: 24px; flex-wrap: wrap; align-items: center;
    }
    .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
      background: #22c55e; margin-right: 6px; animation: pulse 1.5s infinite; }
    .dot.off { background: #ef4444; animation: none; }
    @keyframes pulse { 50% { opacity: 0.3; } }
    .legend {
      margin-top: 10px; font-size: 12px; color: #6b7280; text-align: center;
    }
    .val { color: #a78bfa; font-weight: 600; }
  </style>
</head>
<body>
  <h1>PointCloud Live Viewer</h1>
  <div class="header">
    <button class="btn active" id="btn-local_cloud" onclick="switchStream('local_cloud')">local_cloud (LiDAR)</button>
    <button class="btn" id="btn-/terrain_map" onclick="switchStream('/terrain_map')">terrain_map</button>
    <button class="btn" id="btn-terrain" onclick="switchStream('terrain')">terrain_ext</button>
  </div>
  <div class="container">
    <canvas id="cv" width="960" height="600"></canvas>
    <div class="stats-bar">
      <span><span class="dot" id="dot"></span><span id="status">CONNECTING</span></span>
      <span>FPS: <span class="val" id="fps">--</span></span>
      <span>Points: <span class="val" id="pts">--</span></span>
      <span>Step: <span class="val" id="step">--</span> B</span>
      <span>Data: <span class="val" id="data">--</span> KB</span>
    </div>
  </div>
  <div class="legend">
    Mouse: drag to pan, scroll to zoom | Color: height (Z) mapped blue→green→yellow→red
  </div>
<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
let W = cv.width, H = cv.height;

// View transform
let scale = 30; // pixels per meter
let offsetX = W / 2;
let offsetY = H / 2;

// Drag state
let dragging = false, lastMx = 0, lastMy = 0;

cv.addEventListener('mousedown', e => { dragging = true; lastMx = e.clientX; lastMy = e.clientY; });
cv.addEventListener('mousemove', e => {
  if (!dragging) return;
  offsetX += e.clientX - lastMx;
  offsetY += e.clientY - lastMy;
  lastMx = e.clientX; lastMy = e.clientY;
});
cv.addEventListener('mouseup', () => dragging = false);
cv.addEventListener('mouseleave', () => dragging = false);
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const factor = e.deltaY > 0 ? 0.9 : 1.1;
  // Zoom toward mouse position
  const rect = cv.getBoundingClientRect();
  const mx = (e.clientX - rect.left) / rect.width * W;
  const my = (e.clientY - rect.top) / rect.height * H;
  offsetX = mx + (offsetX - mx) * factor;
  offsetY = my + (offsetY - my) * factor;
  scale *= factor;
}, { passive: false });

// Height colormap: blue(-1m) → green(0m) → yellow(0.5m) → red(1m+)
function heightColor(z) {
  const t = Math.max(0, Math.min(1, (z + 1) / 2.5)); // -1m → 1.5m range
  let r, g, b;
  if (t < 0.33) {
    const s = t / 0.33;
    r = 30; g = Math.floor(100 + 155 * s); b = Math.floor(220 * (1 - s));
  } else if (t < 0.66) {
    const s = (t - 0.33) / 0.33;
    r = Math.floor(255 * s); g = 255; b = 30;
  } else {
    const s = (t - 0.66) / 0.34;
    r = 255; g = Math.floor(255 * (1 - s * 0.7)); b = 30;
  }
  return 'rgb(' + r + ',' + g + ',' + b + ')';
}

let currentPoints = null;

async function fetchData() {
  try {
    const r = await fetch('/data');
    if (!r.ok) return;
    const d = await r.json();
    currentPoints = d.points;
    render();
  } catch(_) {}
}

function render() {
  ctx.fillStyle = '#13112a';
  ctx.fillRect(0, 0, W, H);

  if (!currentPoints || currentPoints.length < 3) {
    ctx.fillStyle = '#6b728088';
    ctx.font = '16px system-ui';
    ctx.textAlign = 'center';
    ctx.fillText('Waiting for point cloud data...', W/2, H/2);
    return;
  }

  // Draw grid
  ctx.strokeStyle = '#1e1b4b30';
  ctx.lineWidth = 1;
  const gridStep = scale; // 1 meter grid
  const startX = offsetX % gridStep;
  const startY = offsetY % gridStep;
  for (let x = startX; x < W; x += gridStep) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
  }
  for (let y = startY; y < H; y += gridStep) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  }

  // Draw origin cross
  ctx.strokeStyle = '#6c63ff40';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(offsetX - 15, offsetY); ctx.lineTo(offsetX + 15, offsetY); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(offsetX, offsetY - 15); ctx.lineTo(offsetX, offsetY + 15); ctx.stroke();

  // Draw points (top-down: x→right, y→up)
  const pts = currentPoints;
  const len = pts.length;
  const ps = 1.5; // point size

  // Batch by color for perf — use imageData for massive point counts
  for (let i = 0; i < len; i += 3) {
    const x = pts[i];
    const y = pts[i+1];
    const z = pts[i+2];

    const px = offsetX + x * scale;
    const py = offsetY - y * scale; // Y flipped (screen Y is down)

    if (px < -5 || px > W+5 || py < -5 || py > H+5) continue;

    ctx.fillStyle = heightColor(z);
    ctx.fillRect(px - ps/2, py - ps/2, ps, ps);
  }

  // Scale indicator
  const meterPx = scale;
  ctx.strokeStyle = '#a78bfa';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(20, H - 20);
  ctx.lineTo(20 + meterPx, H - 20);
  ctx.stroke();
  ctx.fillStyle = '#a78bfa';
  ctx.font = '11px system-ui';
  ctx.textAlign = 'left';
  ctx.fillText('1m', 20 + meterPx/2 - 8, H - 8);
}

async function updateStats() {
  try {
    const r = await fetch('/stats');
    if (!r.ok) return;
    const s = await r.json();
    document.getElementById('fps').textContent = s.fps;
    document.getElementById('pts').textContent = s.numPoints.toLocaleString();
    document.getElementById('step').textContent = s.pointStep;
    document.getElementById('data').textContent = s.totalKB.toLocaleString();

    const dot = document.getElementById('dot');
    const status = document.getElementById('status');
    if (parseInt(s.fps) > 0) {
      dot.className = 'dot'; status.textContent = 'LIVE';
    } else {
      dot.className = 'dot off'; status.textContent = 'NO DATA';
    }
  } catch(_) {}
}

function switchStream(name) {
  // Update button styles
  document.querySelectorAll('.btn').forEach(b => b.classList.remove('active'));
  const btn = document.getElementById('btn-' + name);
  if (btn) btn.classList.add('active');

  currentPoints = null;
  render();

  fetch('/switch?name=' + encodeURIComponent(name))
    .then(r => r.json())
    .then(d => console.log('Switched to:', d))
    .catch(e => console.error('Switch error:', e));
}

// Poll for data at ~8 fps
setInterval(fetchData, 125);
setInterval(updateStats, 1000);
render();
</script>
</body>
</html>''');
  req.response.close();
}
