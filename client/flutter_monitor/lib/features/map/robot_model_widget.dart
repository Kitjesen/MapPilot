import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:webview_flutter/webview_flutter.dart' as wv;
import 'package:webview_windows/webview_windows.dart' as wv_win;
import 'package:robot_proto/src/common.pb.dart';
import 'package:flutter_monitor/app/theme.dart';

/// Displays the real robot 3D model (URDF/STL meshes) in a WebView
/// with full kinematic joint control.
///
/// Joint order (16 joints):
///   0-3  : FR  hip, thigh, calf, foot
///   4-7  : FL  hip, thigh, calf, foot
///   8-11 : RR  hip, thigh, calf, foot
///  12-15 : RL  hip, thigh, calf, foot
///
/// On Windows desktop, uses `webview_windows` (WebView2).
/// On mobile (Android / iOS), uses `webview_flutter`.
class RobotModelWidget extends StatefulWidget {
  final Pose? currentPose;

  /// 16 joint angles in radians. Pass null to keep idle animation.
  final List<double>? jointAngles;

  const RobotModelWidget({
    super.key,
    this.currentPose,
    this.jointAngles,
  });

  @override
  State<RobotModelWidget> createState() => _RobotModelWidgetState();
}

class _RobotModelWidgetState extends State<RobotModelWidget> {
  bool _isLoaded = false;
  DateTime? _lastPoseUpdate;
  DateTime? _lastJointUpdate;
  Timer? _poseThrottle;
  Timer? _jointThrottle;
  HttpServer? _server;
  int? _serverPort;

  // ── Platform-specific controllers ───────────────────────────
  wv.WebViewController? _mobileController;
  wv_win.WebviewController? _windowsController;
  bool _windowsReady = false;

  @override
  void initState() {
    super.initState();
    _startLocalServer().then((_) => _initializeWebView());
  }

  @override
  void dispose() {
    _poseThrottle?.cancel();
    _jointThrottle?.cancel();
    _server?.close(force: true);
    if (Platform.isWindows) {
      _windowsController?.dispose();
    }
    super.dispose();
  }

  // ==================== Local HTTP server ====================

  Future<void> _startLocalServer() async {
    if (_server != null) return;
    _server = await HttpServer.bind(InternetAddress.loopbackIPv4, 0);
    _serverPort = _server!.port;
    debugPrint('[RobotModel] HTTP server on port $_serverPort');

    _server!.listen((HttpRequest request) async {
      var path = request.uri.path;
      if (path == '/') path = '/urdf_viewer_simple.html';

      // Decode percent-encoded segments (Chinese characters, etc.)
      final decoded = Uri.decodeFull(path);
      final assetPath = 'assets$decoded';

      try {
        final data = await rootBundle.load(assetPath);
        request.response.headers.contentType = _contentType(decoded);
        request.response.headers.add('Access-Control-Allow-Origin', '*');
        request.response.add(data.buffer.asUint8List());
      } catch (_) {
        request.response.statusCode = HttpStatus.notFound;
        request.response.write('Not Found: $assetPath');
      } finally {
        await request.response.close();
      }
    });
  }

  static ContentType _contentType(String path) {
    final p = path.toLowerCase();
    if (p.endsWith('.html')) return ContentType.html;
    if (p.endsWith('.js')) {
      return ContentType('application', 'javascript', charset: 'utf-8');
    }
    if (p.endsWith('.stl')) return ContentType('application', 'octet-stream');
    if (p.endsWith('.urdf')) {
      return ContentType('application', 'xml', charset: 'utf-8');
    }
    if (p.endsWith('.png')) return ContentType('image', 'png');
    if (p.endsWith('.json')) {
      return ContentType('application', 'json', charset: 'utf-8');
    }
    return ContentType('application', 'octet-stream');
  }

  // ==================== WebView initialisation ====================

  void _initializeWebView() {
    if (_serverPort == null) return;
    final url = 'http://127.0.0.1:$_serverPort/';

    if (Platform.isWindows) {
      _initWindowsWebView(url);
    } else {
      _initMobileWebView(url);
    }
  }

  // ── Mobile (webview_flutter) ──────────────────────────────────

  void _initMobileWebView(String url) {
    debugPrint('[RobotModel] Mobile WebView → $url');
    setState(() {
      _mobileController = wv.WebViewController()
        ..setJavaScriptMode(wv.JavaScriptMode.unrestricted)
        ..setBackgroundColor(Colors.transparent)
        ..addJavaScriptChannel(
          'FlutterChannel',
          onMessageReceived: (msg) => _onJSMessage(msg.message),
        )
        ..setNavigationDelegate(
          wv.NavigationDelegate(
            onWebResourceError: (e) =>
                debugPrint('[RobotModel] WebView error: ${e.description}'),
          ),
        )
        ..loadRequest(Uri.parse(url));
    });
  }

  // ── Windows (webview_windows / WebView2) ──────────────────────

  Future<void> _initWindowsWebView(String url) async {
    debugPrint('[RobotModel] Windows WebView → $url');
    try {
      _windowsController = wv_win.WebviewController();
      await _windowsController!.initialize();

      // webMessage stream: webview_windows JSON-decodes the value
      // from get_WebMessageAsJson, so if JS sends an object via
      // chrome.webview.postMessage(obj) it arrives as a Map.
      _windowsController!.webMessage.listen(
        (dynamic decoded) {
          if (decoded is Map) {
            _onJSMessageDecoded(decoded);
          } else {
            // Fallback for string messages
            _onJSMessage(decoded.toString());
          }
        },
        onError: (e) => debugPrint('[RobotModel] webMessage error: $e'),
      );

      // Debug: log loading state changes
      _windowsController!.loadingState.listen((state) {
        debugPrint('[RobotModel] loadingState: $state');
        if (state == wv_win.LoadingState.navigationCompleted && !_isLoaded) {
          // Page loaded — give JS a moment then try triggering ready
          Future.delayed(const Duration(seconds: 2), () {
            if (!_isLoaded && mounted) {
              debugPrint('[RobotModel] Force-checking ready via JS');
              _runJS("if(typeof sendToFlutter==='function') sendToFlutter({type:'ready',message:'forced'});");
            }
          });
        }
      });

      _windowsController!.onLoadError.listen((error) {
        debugPrint('[RobotModel] WebView load error: $error');
      });

      await _windowsController!.setPopupWindowPolicy(
          wv_win.WebviewPopupWindowPolicy.deny);
      await _windowsController!.loadUrl(url);

      if (mounted) setState(() => _windowsReady = true);
    } catch (e) {
      debugPrint('[RobotModel] Windows WebView init failed: $e');
    }
  }

  // ==================== JS message handling ====================

  /// Called with a raw JSON string (mobile path).
  void _onJSMessage(String raw) {
    try {
      final data = jsonDecode(raw);
      _onJSMessageDecoded(data);
    } catch (_) {}
  }

  /// Called with an already-decoded map (Windows path or after decode).
  void _onJSMessageDecoded(dynamic data) {
    final type = data is Map ? data['type'] ?? '' : '';
    debugPrint('[RobotModel] JS $type: ${data is Map ? data['message'] : data}');

    if (type == 'ready' || type == 'info') {
      if (!_isLoaded) {
        setState(() => _isLoaded = true);
        Future.delayed(const Duration(milliseconds: 400), () {
          if (!mounted) return;
          if (widget.currentPose != null) _sendPose();
          if (widget.jointAngles != null) _sendJointAngles();
          // Sync theme
          final isDark = Theme.of(context).brightness == Brightness.dark;
          _runJS(
              "if(window.setTheme) window.setTheme('${isDark ? 'dark' : 'light'}');");
        });
      }
    }
  }

  // ==================== JS execution ====================

  void _runJS(String script) {
    if (Platform.isWindows) {
      _windowsController?.executeScript(script);
    } else {
      _mobileController?.runJavaScript(script);
    }
  }

  // ==================== didUpdateWidget ====================

  @override
  void didUpdateWidget(RobotModelWidget oldWidget) {
    super.didUpdateWidget(oldWidget);

    if (widget.currentPose != oldWidget.currentPose &&
        widget.currentPose != null) {
      _schedulePose();
    }

    if (widget.jointAngles != oldWidget.jointAngles &&
        widget.jointAngles != null) {
      _scheduleJoints();
    }
  }

  // ==================== Pose (throttled 2 Hz) ====================

  void _schedulePose() {
    final now = DateTime.now();
    if (_lastPoseUpdate != null &&
        now.difference(_lastPoseUpdate!).inMilliseconds < 500) return;
    _poseThrottle?.cancel();
    _poseThrottle = Timer(const Duration(milliseconds: 500), _sendPose);
  }

  void _sendPose() {
    if (!_isLoaded || widget.currentPose == null) return;
    final p = widget.currentPose!.position;
    final o = widget.currentPose!.orientation;

    // Viewer uses Z-up (matching ROS convention) → send raw coords
    _runJS(
      'if(window.updateRobotPose) window.updateRobotPose('
      '${p.x},${p.y},${p.z},'
      '${o.x},${o.y},${o.z},${o.w});',
    );
    _lastPoseUpdate = DateTime.now();
  }

  // ==================== Joint angles (throttled 5 Hz) ====================

  void _scheduleJoints() {
    final now = DateTime.now();
    if (_lastJointUpdate != null &&
        now.difference(_lastJointUpdate!).inMilliseconds < 200) return;
    _jointThrottle?.cancel();
    _jointThrottle = Timer(const Duration(milliseconds: 200), _sendJointAngles);
  }

  void _sendJointAngles() {
    if (!_isLoaded || widget.jointAngles == null) return;
    final a = widget.jointAngles!;
    if (a.length < 16) return;

    final csv = a.map((v) => v.toStringAsFixed(4)).join(',');
    _runJS(
      'if(window.updateJointAngles) window.updateJointAngles([$csv]);',
    );
    _lastJointUpdate = DateTime.now();
  }

  /// Manually set idle animation on/off from parent.
  void setIdleAnimation(bool enable) {
    _runJS(
      'if(window.enableIdleAnimation) window.enableIdleAnimation(${enable ? 'true' : 'false'});',
    );
  }

  // ==================== Build ====================

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    // ── Still initialising ──────────────────────────────────────
    final controllerReady = Platform.isWindows
        ? _windowsReady
        : _mobileController != null;

    if (!controllerReady) {
      return _buildPlaceholder(isDark, '正在启动 3D 查看器…');
    }

    // ── WebView + loading overlay ───────────────────────────────
    return Stack(
      children: [
        if (Platform.isWindows)
          wv_win.Webview(_windowsController!)
        else
          wv.WebViewWidget(controller: _mobileController!),
        if (!_isLoaded) _buildLoadingOverlay(isDark),
      ],
    );
  }

  Widget _buildPlaceholder(bool isDark, String text) {
    return Container(
      color: isDark ? AppColors.darkBackground : Colors.grey[100],
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            CircularProgressIndicator(
                color: context.subtitleColor, strokeWidth: 2),
            const SizedBox(height: 14),
            Text(
              text,
              style: TextStyle(
                color: isDark ? Colors.white60 : Colors.black54,
                fontSize: 12,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildLoadingOverlay(bool isDark) {
    return Container(
      color: (isDark ? Colors.black : Colors.white).withValues(alpha:0.85),
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            CircularProgressIndicator(
                color: context.subtitleColor, strokeWidth: 2),
            const SizedBox(height: 14),
            Text(
              '正在加载机器人模型…',
              style: TextStyle(
                color: isDark ? Colors.white60 : Colors.black54,
                fontSize: 12,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
