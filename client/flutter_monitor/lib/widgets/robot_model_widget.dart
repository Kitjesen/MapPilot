import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'package:robot_proto/src/common.pb.dart';

class RobotModelWidget extends StatefulWidget {
  final Pose? currentPose;

  const RobotModelWidget({
    super.key,
    this.currentPose,
  });

  @override
  State<RobotModelWidget> createState() => _RobotModelWidgetState();
}

class _RobotModelWidgetState extends State<RobotModelWidget> {
  WebViewController? _controller;
  bool _isLoaded = false;
  DateTime? _lastUpdate;
  Timer? _throttleTimer;
  final String _urdfAssetPath = 'assets/urdf/轮足狗机器人v3.urdf';
  HttpServer? _server;
  int? _serverPort;

  @override
  void initState() {
    super.initState();
    _startLocalServer().then((_) {
      _initializeWebView();
    });
  }

  @override
  void dispose() {
    _throttleTimer?.cancel();
    _server?.close(force: true);
    super.dispose();
  }

  Future<void> _startLocalServer() async {
    if (_server != null) {
      return;
    }
    _server = await HttpServer.bind(InternetAddress.loopbackIPv4, 0);
    _serverPort = _server!.port;
    debugPrint('🌐 HTTP Server started on port $_serverPort');
    
    _server!.listen((HttpRequest request) async {
      final path = request.uri.path == '/' ? '/urdf_viewer.html' : request.uri.path;
      final assetPath = 'assets$path';

      debugPrint('📥 Request: $path');

      try {
        final data = await rootBundle.load(assetPath);
        final contentType = _contentTypeForPath(assetPath);
        request.response.headers.contentType = contentType;
        request.response.add(data.buffer.asUint8List());
        debugPrint('✅ Served: $assetPath (${data.lengthInBytes} bytes)');
      } catch (e) {
        debugPrint('❌ Failed to load: $assetPath - $e');
        request.response.statusCode = HttpStatus.notFound;
        request.response.write('Not Found');
      } finally {
        await request.response.close();
      }
    });
  }

  ContentType _contentTypeForPath(String path) {
    if (path.endsWith('.html')) return ContentType.html;
    if (path.endsWith('.js')) return ContentType('application', 'javascript');
    if (path.endsWith('.stl')) return ContentType('application', 'octet-stream');
    if (path.endsWith('.urdf')) return ContentType('application', 'xml');
    if (path.endsWith('.csv')) return ContentType('text', 'csv');
    if (path.endsWith('.png')) return ContentType('image', 'png');
    return ContentType('application', 'octet-stream');
  }

  void _initializeWebView() {
    if (_serverPort == null) {
      debugPrint('❌ Server port is null, cannot init WebView');
      return;
    }
    
    final url = 'http://127.0.0.1:$_serverPort/urdf_viewer.html';
    debugPrint('🌍 Initializing WebView with URL: $url');
    
    setState(() {
      _controller = WebViewController()
        ..setJavaScriptMode(JavaScriptMode.unrestricted)
        ..setBackgroundColor(Colors.transparent)
        ..addJavaScriptChannel(
          'FlutterChannel',
          onMessageReceived: (JavaScriptMessage message) {
            _handleMessageFromJS(message.message);
          },
        )
        ..setNavigationDelegate(
          NavigationDelegate(
            onPageStarted: (String url) {
              debugPrint('📄 Page started loading: $url');
            },
            onPageFinished: (String url) {
              debugPrint('✅ Page finished loading: $url');
              setState(() {
                _isLoaded = true;
              });
              
              // Delay to ensure JavaScript is fully initialized
              Future.delayed(const Duration(milliseconds: 500), () {
                debugPrint('⏰ JavaScript should be ready now');
                // Send initial pose if available
                if (widget.currentPose != null) {
                  _updateRobotInWebView();
                }
              });
            },
            onWebResourceError: (WebResourceError error) {
              debugPrint('❌ WebView error: ${error.description}');
            },
          ),
        )
        ..loadRequest(Uri.parse('http://127.0.0.1:$_serverPort/urdf_viewer_simple.html'));
    });
  }

  Future<void> _loadUrdfModel() async {
    if (_controller == null) {
      debugPrint('❌ Controller is null, cannot load URDF');
      return;
    }
    
    debugPrint('🤖 Loading URDF from: $_urdfAssetPath');
    
    try {
      final urdfContent = await rootBundle.loadString(_urdfAssetPath);
      debugPrint('✅ URDF loaded, length: ${urdfContent.length} chars');
      
      final encodedContent = jsonEncode(urdfContent);
      final jsCode = '''
        console.log('📦 Flutter injecting URDF...');
        if (typeof window.loadURDF === 'function') {
          window.loadURDF($encodedContent);
        } else {
          console.error('❌ window.loadURDF is not defined!');
        }
      ''';
      await _controller!.runJavaScript(jsCode);
      debugPrint('✅ URDF injection JS executed');
    } catch (e) {
      debugPrint('❌ Failed to load URDF asset: $e');
    }
  }

  void _handleMessageFromJS(String message) {
    try {
      final data = jsonDecode(message);
      final type = data['type'];
      
      if (type == 'ready') {
        debugPrint('3D viewer ready: ${data['message']}');
      } else if (type == 'info') {
        debugPrint('3D viewer info: ${data['message']}');
      } else if (type == 'error') {
        debugPrint('3D viewer error: ${data['message']}');
      }
    } catch (e) {
      debugPrint('Failed to parse message from JS: $e');
    }
  }

  @override
  void didUpdateWidget(RobotModelWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    
    // Update robot pose when it changes (with throttling)
    if (widget.currentPose != oldWidget.currentPose && 
        widget.currentPose != null) {
      _scheduleUpdate();
    }
  }

  void _scheduleUpdate() {
    // Throttle updates to 2Hz (500ms) to reduce WebView load significantly
    final now = DateTime.now();
    if (_lastUpdate != null && 
        now.difference(_lastUpdate!).inMilliseconds < 500) {
      return;
    }

    _throttleTimer?.cancel();
    _throttleTimer = Timer(const Duration(milliseconds: 500), () {
      _updateRobotInWebView();
    });
  }

  void _updateRobotInWebView() {
    if (_controller == null || !_isLoaded || widget.currentPose == null) return;

    final pose = widget.currentPose!;
    final position = pose.position;
    final orientation = pose.orientation;

    // Convert ROS coordinates to Three.js coordinates
    // ROS: X forward, Y left, Z up
    // Three.js: X right, Y up, Z forward
    final x = position.x;
    final y = position.z; // Z becomes Y (height)
    final z = -position.y; // Y becomes -Z (forward)

    // Quaternion conversion (ROS to Three.js coordinate system)
    final qx = orientation.x;
    final qy = orientation.z;
    final qz = -orientation.y;
    final qw = orientation.w;

    final jsCode = '''
      if (typeof window.updateRobotPose === 'function') {
        window.updateRobotPose($x, $y, $z, $qx, $qy, $qz, $qw);
      }
    ''';

    _controller!.runJavaScript(jsCode);
    _lastUpdate = DateTime.now();
  }

  @override
  Widget build(BuildContext context) {
    if (_controller == null) {
      return Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              Colors.grey[200]!,
              Colors.grey[100]!,
            ],
          ),
        ),
        child: const Center(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              CircularProgressIndicator(color: Color(0xFF007AFF)),
              SizedBox(height: 16),
              Text(
                'Starting 3D Viewer...',
                style: TextStyle(
                  color: Colors.black54,
                  fontSize: 12,
                ),
              ),
            ],
          ),
        ),
      );
    }

    return Stack(
      children: [
        // WebView
        WebViewWidget(controller: _controller!),
        
        // Loading indicator
        if (!_isLoaded)
          Container(
            color: Colors.white.withOpacity(0.9),
            child: const Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  CircularProgressIndicator(),
                  SizedBox(height: 16),
                  Text(
                    'Loading 3D Viewer...',
                    style: TextStyle(
                      color: Colors.black54,
                      fontSize: 12,
                    ),
                  ),
                ],
              ),
            ),
          ),
      ],
    );
  }
}
