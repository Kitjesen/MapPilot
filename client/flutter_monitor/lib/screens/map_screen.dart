import 'dart:ui';
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter/services.dart';
import '../services/robot_client_base.dart';
import '../generated/telemetry.pb.dart';
import '../generated/common.pb.dart';
import '../generated/data.pb.dart';
import '../widgets/glass_widgets.dart';

class MapScreen extends StatefulWidget {
  final RobotClientBase client;

  const MapScreen({super.key, required this.client});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  final List<Offset> _path = [];
  List<Offset> _globalMapPoints = [];
  List<Offset> _localCloudPoints = [];
  
  Pose? _currentPose;
  StreamSubscription? _subscription;
  StreamSubscription? _mapSubscription;
  StreamSubscription? _pclSubscription;
  
  final TransformationController _transformController = TransformationController();
  double _currentYaw = 0.0;
  bool _showGlobalMap = true;

  @override
  void initState() {
    super.initState();
    _startListening();
    _subscribeToMaps();
    
    // Initial view
    final matrix = Matrix4.identity()
      ..translate(200.0, 300.0) 
      ..scale(20.0); 
    _transformController.value = matrix;
  }

  @override
  void dispose() {
    _subscription?.cancel();
    _mapSubscription?.cancel();
    _pclSubscription?.cancel();
    _transformController.dispose();
    super.dispose();
  }

  void _subscribeToMaps() {
    // Subscribe to Global Map
    _mapSubscription = widget.client.subscribeToResource(
      ResourceId()..type = ResourceType.RESOURCE_TYPE_MAP
    ).listen((chunk) {
      if (!mounted) return;
      _parseAndSetPoints(chunk.data, isGlobal: true);
    });

    // Subscribe to Local Cloud
    _pclSubscription = widget.client.subscribeToResource(
      ResourceId()..type = ResourceType.RESOURCE_TYPE_POINTCLOUD
    ).listen((chunk) {
      if (!mounted) return;
      _parseAndSetPoints(chunk.data, isGlobal: false);
    });
  }

  void _parseAndSetPoints(List<int> data, {required bool isGlobal}) {
    // Basic parsing assuming float32 fields
    // Global Map (XYZ): 12 bytes
    // Local Cloud (XYZI): 16 bytes
    final pointStep = isGlobal ? 12 : 16;
    final points = <Offset>[];
    
    final byteData = Uint8List.fromList(data).buffer.asByteData();
    final count = data.length ~/ pointStep;
    
    // Downsample for performance
    final stride = isGlobal ? 10 : 2; // Show 1/10th of global, 1/2 of local
    
    for (var i = 0; i < count; i += stride) {
      final offset = i * pointStep;
      if (offset + 8 <= data.length) {
        final x = byteData.getFloat32(offset, Endian.little);
        final y = byteData.getFloat32(offset + 4, Endian.little);
        points.add(Offset(x, y));
      }
    }

    setState(() {
      if (isGlobal) {
        _globalMapPoints = points;
      } else {
        _localCloudPoints = points;
      }
    });
  }

  void _startListening() {
    _subscription = widget.client.streamFastState(desiredHz: 5.0).listen((state) {
      if (!mounted) return;
      setState(() {
        _currentPose = state.pose;
        
        // Calculate Yaw
        final q = state.pose.orientation;
        final siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        final cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        _currentYaw = math.atan2(siny_cosp, cosy_cosp);

        final point = Offset(state.pose.position.x, state.pose.position.y);
        
        if (_path.isEmpty || (_path.last - point).distance > 0.05) {
          _path.add(point);
          if (_path.length > 5000) {
            _path.removeRange(0, 1000);
          }
        }
      });
    });
  }

  void _clearPath() {
    setState(() {
      _path.clear();
    });
  }

  void _recenter() {
    if (_currentPose != null) {
      // Simple recenter reset
      // Ideally needs to calculate center of screen vs robot pos
      final matrix = Matrix4.identity()
      ..translate(200.0, 400.0) 
      ..scale(20.0)
      ..translate(-_currentPose!.position.x, -_currentPose!.position.y); 
      _transformController.value = matrix;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        title: const Text('Trajectory'),
        backgroundColor: Colors.transparent,
        elevation: 0,
        actions: [
           Padding(
             padding: const EdgeInsets.only(right: 16.0),
             child: IconButton(
              icon: const Icon(Icons.delete_outline, color: Colors.black54),
              onPressed: _clearPath,
              tooltip: 'Clear Path',
             ),
           ),
        ],
      ),
      body: Stack(
        children: [
          // Map Background
          Positioned.fill(
            child: GridPaper(
              color: Colors.black12,
              interval: 100, // 100 pixels at scale 1.0 (5m at scale 20)
              divisions: 1,
              subdivisions: 5,
              child: InteractiveViewer(
                transformationController: _transformController,
                boundaryMargin: const EdgeInsets.all(5000),
                minScale: 0.1,
                maxScale: 100.0,
                child: Container(
                  width: 10000,
                  height: 10000,
                  child: CustomPaint(
                    painter: TrajectoryPainter(
                      path: _path,
                      currentPose: _currentPose,
                      globalPoints: _showGlobalMap ? _globalMapPoints : [],
                      localPoints: _localCloudPoints,
                    ),
                    size: const Size(10000, 10000),
                  ),
                ),
              ),
            ),
          ),
          
          // Floating Compass
          Positioned(
            left: 20,
            top: MediaQuery.of(context).padding.top + 60,
            child: Column(
              children: [
                GlassCard(
                  borderRadius: 30,
                  padding: const EdgeInsets.all(8),
                  child: Column(
                    children: [
                      Transform.rotate(
                        angle: -_currentYaw, 
                        child: const Icon(Icons.navigation, color: Colors.blue, size: 32),
                      ),
                      const SizedBox(height: 4),
                      Text(
                        '${(_currentYaw * 180 / math.pi).toStringAsFixed(0)}°',
                        style: const TextStyle(fontSize: 10, fontWeight: FontWeight.bold),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 12),
                GlassButton(
                  onPressed: () {
                    setState(() {
                      _showGlobalMap = !_showGlobalMap;
                    });
                  },
                  icon: Icon(_showGlobalMap ? Icons.layers : Icons.layers_clear, size: 20),
                  label: 'MAP',
                  backgroundColor: _showGlobalMap ? Colors.blue : Colors.grey,
                ),
              ],
            ),
          ),

          // Floating Controls
          Positioned(
            right: 20,
            bottom: 40,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                FloatingActionButton.small(
                  heroTag: 'zoom_in',
                  backgroundColor: Colors.white.withOpacity(0.9),
                  foregroundColor: Colors.black87,
                  elevation: 2,
                  onPressed: () {
                    final matrix = _transformController.value.clone();
                    matrix.scale(1.2);
                    _transformController.value = matrix;
                  },
                  child: const Icon(Icons.add),
                ),
                const SizedBox(height: 12),
                FloatingActionButton.small(
                  heroTag: 'zoom_out',
                  backgroundColor: Colors.white.withOpacity(0.9),
                  foregroundColor: Colors.black87,
                  elevation: 2,
                  onPressed: () {
                    final matrix = _transformController.value.clone();
                    matrix.scale(1/1.2);
                    _transformController.value = matrix;
                  },
                  child: const Icon(Icons.remove),
                ),
                const SizedBox(height: 24),
                GlassButton(
                  onPressed: _recenter,
                  icon: const Icon(Icons.my_location, color: Colors.white),
                  label: 'LOCATE',
                  backgroundColor: const Color(0xFF007AFF),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class TrajectoryPainter extends CustomPainter {
  final List<Offset> path;
  final Pose? currentPose;
  final List<Offset> globalPoints;
  final List<Offset> localPoints;

  TrajectoryPainter({
    required this.path, 
    this.currentPose,
    this.globalPoints = const [],
    this.localPoints = const [],
  });

  @override
  void paint(Canvas canvas, Size size) {
    canvas.translate(size.width / 2, size.height / 2);
    canvas.scale(1.0, -1.0);

    // Draw Global Map
    if (globalPoints.isNotEmpty) {
      final mapPaint = Paint()
        ..color = Colors.black12
        ..strokeWidth = 0.1
        ..strokeCap = StrokeCap.round;
      
      canvas.drawPoints(PointMode.points, globalPoints, mapPaint);
    }

    // Draw Local Cloud
    if (localPoints.isNotEmpty) {
      final cloudPaint = Paint()
        ..color = Colors.red.withOpacity(0.3)
        ..strokeWidth = 0.05
        ..strokeCap = StrokeCap.round;
      
      canvas.drawPoints(PointMode.points, localPoints, cloudPaint);
    }

    final paint = Paint()
      ..color = const Color(0xFF007AFF).withOpacity(0.6)
      ..strokeWidth = 0.1
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    if (path.isNotEmpty) {
      final pathObj = Path();
      pathObj.moveTo(path.first.dx, path.first.dy);
      for (var i = 1; i < path.length; i++) {
        pathObj.lineTo(path[i].dx, path[i].dy);
      }
      canvas.drawPath(pathObj, paint);
    }

    if (currentPose != null) {
      final p = currentPose!.position;
      final q = currentPose!.orientation;
      
      final siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      final cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      final yaw = math.atan2(siny_cosp, cosy_cosp);

      canvas.save();
      canvas.translate(p.x, p.y);
      canvas.rotate(yaw);

      final robotPaint = Paint()
        ..color = const Color(0xFFFF3B30)
        ..style = PaintingStyle.fill;
      
      const double robotLen = 0.5;
      const double robotWidth = 0.3;

      final robotPath = Path()
        ..moveTo(robotLen / 2, 0)
        ..lineTo(-robotLen / 2, robotWidth / 2)
        ..lineTo(-robotLen / 2, -robotWidth / 2)
        ..close();
      
      canvas.drawPath(robotPath, robotPaint);
      
      // Draw simple heading line
      canvas.drawLine(Offset.zero, const Offset(1.0, 0), Paint()..color = Colors.black.withOpacity(0.5)..strokeWidth=0.05);

      canvas.restore();
    }
    
    // Origin
    final originPaint = Paint()..strokeWidth = 0.05;
    canvas.drawLine(const Offset(-1, 0), const Offset(1, 0), originPaint..color = Colors.grey.withOpacity(0.5));
    canvas.drawLine(const Offset(0, -1), const Offset(0, 1), originPaint..color = Colors.grey.withOpacity(0.5));
  }

  @override
  bool shouldRepaint(covariant TrajectoryPainter oldDelegate) {
    return true;
  }
}
