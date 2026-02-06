import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../services/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import '../widgets/glass_widgets.dart';
import '../widgets/robot_model_widget.dart';

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen>
    with AutomaticKeepAliveClientMixin {
  final List<Offset> _path = [];
  List<Offset> _globalMapPoints = [];
  List<Offset> _localCloudPoints = [];

  Pose? _currentPose;
  StreamSubscription<FastState>? _fastSub;
  StreamSubscription? _mapSubscription;
  StreamSubscription? _pclSubscription;

  final TransformationController _transformController =
      TransformationController();
  double _currentYaw = 0.0;
  bool _showGlobalMap = true;
  bool _show3DModel = true;

  // 节流：地图位姿更新限制在 5Hz
  DateTime _lastPoseUpdate = DateTime(2000);
  static const _poseUpdateInterval = Duration(milliseconds: 200);

  // 地图点数据版本号，用于优化 shouldRepaint
  int _mapDataVersion = 0;

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startListening();
      _subscribeToMaps();
    });

    // Initial view
    final matrix = Matrix4.identity()
      ..translate(200.0, 300.0)
      ..scale(20.0);
    _transformController.value = matrix;
  }

  @override
  void dispose() {
    _fastSub?.cancel();
    _mapSubscription?.cancel();
    _pclSubscription?.cancel();
    _transformController.dispose();
    super.dispose();
  }

  void _subscribeToMaps() {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;

    // Subscribe to Global Map
    _mapSubscription = client
        .subscribeToResource(ResourceId()..type = ResourceType.RESOURCE_TYPE_MAP)
        .listen((chunk) {
      if (!mounted) return;
      _parseAndSetPoints(chunk.data, isGlobal: true);
    }, onError: (e) {
      debugPrint('[MapScreen] Map subscription error: $e');
    });

    // Subscribe to Local Cloud
    _pclSubscription = client
        .subscribeToResource(
            ResourceId()..type = ResourceType.RESOURCE_TYPE_POINTCLOUD)
        .listen((chunk) {
      if (!mounted) return;
      _parseAndSetPoints(chunk.data, isGlobal: false);
    }, onError: (e) {
      debugPrint('[MapScreen] Pointcloud subscription error: $e');
    });
  }

  void _parseAndSetPoints(List<int> data, {required bool isGlobal}) {
    if (data.isEmpty) return;

    final pointStep = isGlobal ? 12 : 16;
    final points = <Offset>[];

    final byteData = Uint8List.fromList(data).buffer.asByteData();
    final count = data.length ~/ pointStep;

    // Downsample for performance
    final stride = isGlobal ? 10 : 2;

    for (var i = 0; i < count; i += stride) {
      final offset = i * pointStep;
      if (offset + 8 <= data.length) {
        final x = byteData.getFloat32(offset, Endian.little);
        final y = byteData.getFloat32(offset + 4, Endian.little);
        if (x.isFinite && y.isFinite) {
          points.add(Offset(x, y));
        }
      }
    }

    setState(() {
      _mapDataVersion++;
      if (isGlobal) {
        _globalMapPoints = points;
      } else {
        _localCloudPoints = points;
      }
    });
  }

  void _startListening() {
    final provider = context.read<RobotConnectionProvider>();

    _fastSub = provider.fastStateStream.listen((state) {
      if (!mounted) return;

      final now = DateTime.now();
      if (now.difference(_lastPoseUpdate) < _poseUpdateInterval) return;
      _lastPoseUpdate = now;

      final newPose = state.pose;

      // Calculate Yaw
      final q = newPose.orientation;
      final siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      final cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      final newYaw = math.atan2(siny_cosp, cosy_cosp);

      final point = Offset(newPose.position.x, newPose.position.y);

      setState(() {
        _currentPose = newPose;
        _currentYaw = newYaw;

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
    HapticFeedback.lightImpact();
    setState(() {
      _path.clear();
    });
  }

  void _recenter() {
    HapticFeedback.lightImpact();
    if (_currentPose != null) {
      final size = MediaQuery.of(context).size;
      final matrix = Matrix4.identity()
        ..translate(size.width / 2, size.height / 2)
        ..scale(20.0)
        ..translate(-_currentPose!.position.x, -_currentPose!.position.y);
      _transformController.value = matrix;
    }
  }

  @override
  Widget build(BuildContext context) {
    super.build(context); // Required by AutomaticKeepAliveClientMixin

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
          // 3D Ground + Robot (full screen)
          if (_show3DModel)
            Positioned.fill(
              child: RepaintBoundary(
                child: RobotModelWidget(
                  currentPose: _currentPose,
                ),
              ),
            ),

          // 2D Map Background (fallback)
          if (!_show3DModel)
            Positioned.fill(
              child: GridPaper(
                color: Colors.black12,
                interval: 100,
                divisions: 1,
                subdivisions: 5,
                child: InteractiveViewer(
                  transformationController: _transformController,
                  boundaryMargin: const EdgeInsets.all(5000),
                  minScale: 0.1,
                  maxScale: 100.0,
                  child: SizedBox(
                    width: 10000,
                    height: 10000,
                    child: RepaintBoundary(
                      child: CustomPaint(
                        painter: TrajectoryPainter(
                          path: _path,
                          currentPose: _currentPose,
                          globalPoints:
                              _showGlobalMap ? _globalMapPoints : const [],
                          localPoints: _localCloudPoints,
                          dataVersion: _mapDataVersion,
                        ),
                        size: const Size(10000, 10000),
                      ),
                    ),
                  ),
                ),
              ),
            ),

          // Top Left: Robot Name Panel
          Positioned(
            left: 16,
            top: MediaQuery.of(context).padding.top + 16,
            child: GlassCard(
              borderRadius: 12,
              padding:
                  const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  // 使用 Selector 精确监听连接状态
                  Selector<RobotConnectionProvider, bool>(
                    selector: (_, p) => p.isConnected,
                    builder: (_, isConnected, __) {
                      return Container(
                        width: 8,
                        height: 8,
                        decoration: BoxDecoration(
                          color: isConnected ? Colors.green : Colors.red,
                          shape: BoxShape.circle,
                        ),
                      );
                    },
                  ),
                  const SizedBox(width: 8),
                  const Text(
                    'my_robot',
                    style: TextStyle(
                      fontSize: 14,
                      fontWeight: FontWeight.w600,
                      color: Colors.black87,
                    ),
                  ),
                ],
              ),
            ),
          ),

          // Top Right: Scene Options Panel
          Positioned(
            right: 16,
            top: MediaQuery.of(context).padding.top + 16,
            child: GlassCard(
              borderRadius: 12,
              padding: const EdgeInsets.all(8),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  _buildIconButton(
                    icon: _show3DModel ? Icons.view_in_ar : Icons.map,
                    tooltip: _show3DModel ? 'Switch to 2D' : 'Switch to 3D',
                    isSelected: true,
                    onPressed: () {
                      HapticFeedback.selectionClick();
                      setState(() => _show3DModel = !_show3DModel);
                    },
                  ),
                  const SizedBox(height: 8),
                  _buildIconButton(
                    icon: Icons.layers,
                    tooltip: 'Toggle Map Layer',
                    isSelected: _showGlobalMap,
                    onPressed: () {
                      HapticFeedback.selectionClick();
                      setState(() => _showGlobalMap = !_showGlobalMap);
                    },
                  ),
                  const SizedBox(height: 8),
                  _buildIconButton(
                    icon: Icons.my_location,
                    tooltip: 'Recenter',
                    onPressed: _recenter,
                  ),
                  const SizedBox(height: 8),
                  _buildIconButton(
                    icon: Icons.settings,
                    tooltip: 'Settings',
                    onPressed: () => _showSettingsDialog(context),
                  ),
                ],
              ),
            ),
          ),

          // Bottom Left: Compass
          Positioned(
            left: 16,
            bottom: 100,
            child: GlassCard(
              borderRadius: 30,
              padding: const EdgeInsets.all(12),
              child: Transform.rotate(
                angle: -_currentYaw,
                child: const Icon(Icons.navigation,
                    color: Color(0xFF007AFF), size: 28),
              ),
            ),
          ),

          // Bottom right: path info
          if (_path.isNotEmpty)
            Positioned(
              right: 16,
              bottom: 100,
              child: GlassCard(
                borderRadius: 12,
                padding:
                    const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                child: Text(
                  '${_path.length} pts',
                  style: TextStyle(
                    fontSize: 11,
                    fontWeight: FontWeight.w600,
                    color: Colors.black.withOpacity(0.4),
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildIconButton({
    required IconData icon,
    required VoidCallback onPressed,
    String? tooltip,
    bool isSelected = false,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: isSelected
            ? const Color(0xFF007AFF).withOpacity(0.1)
            : Colors.transparent,
        borderRadius: BorderRadius.circular(8),
      ),
      child: IconButton(
        icon: Icon(icon),
        color: isSelected ? const Color(0xFF007AFF) : Colors.black54,
        iconSize: 20,
        tooltip: tooltip,
        onPressed: onPressed,
        constraints: const BoxConstraints(minWidth: 40, minHeight: 40),
        padding: EdgeInsets.zero,
      ),
    );
  }

  void _showSettingsDialog(BuildContext context) {
    showDialog(
      context: context,
      builder: (context) => Dialog(
        backgroundColor: Colors.transparent,
        child: GlassCard(
          borderRadius: 16,
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  const Text(
                    'Settings',
                    style:
                        TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  IconButton(
                    icon: const Icon(Icons.close, size: 20),
                    onPressed: () => Navigator.pop(context),
                    padding: EdgeInsets.zero,
                    constraints: const BoxConstraints(),
                  ),
                ],
              ),
              const SizedBox(height: 20),
              const Text('Theme',
                  style:
                      TextStyle(fontSize: 14, fontWeight: FontWeight.w500)),
              const SizedBox(height: 12),
              Row(
                children: [
                  _buildThemeOption('Light', false),
                  const SizedBox(width: 8),
                  _buildThemeOption('Dark', true),
                  const SizedBox(width: 8),
                  _buildThemeOption('System', false),
                ],
              ),
              const SizedBox(height: 20),
              const Text('Visualization',
                  style:
                      TextStyle(fontSize: 14, fontWeight: FontWeight.w500)),
              const SizedBox(height: 12),
              _buildSwitch('Show Grid', true),
              _buildSwitch('Show Shadows', true),
              _buildSwitch('Show Robot Name', true),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildThemeOption(String label, bool isSelected) {
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 8),
        decoration: BoxDecoration(
          color: isSelected ? Colors.white : Colors.grey.withOpacity(0.1),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(
            color:
                isSelected ? const Color(0xFF007AFF) : Colors.transparent,
            width: 1.5,
          ),
          boxShadow: isSelected
              ? [
                  BoxShadow(
                    color: Colors.black.withOpacity(0.05),
                    blurRadius: 4,
                    offset: const Offset(0, 2),
                  )
                ]
              : null,
        ),
        child: Center(
          child: Text(
            label,
            style: TextStyle(
              fontSize: 12,
              fontWeight:
                  isSelected ? FontWeight.w600 : FontWeight.normal,
              color: isSelected
                  ? const Color(0xFF007AFF)
                  : Colors.black87,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildSwitch(String label, bool value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(fontSize: 13)),
          Switch.adaptive(
            value: value,
            onChanged: (v) {},
            activeColor: const Color(0xFF007AFF),
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
  final int dataVersion;

  // 缓存上次绘制参数，用于 shouldRepaint 优化
  final int _pathLength;
  final double? _poseX;
  final double? _poseY;

  TrajectoryPainter({
    required this.path,
    this.currentPose,
    this.globalPoints = const [],
    this.localPoints = const [],
    this.dataVersion = 0,
  })  : _pathLength = path.length,
        _poseX = currentPose?.position.x,
        _poseY = currentPose?.position.y;

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

      canvas.drawPoints(ui.PointMode.points, globalPoints, mapPaint);
    }

    // Draw Local Cloud
    if (localPoints.isNotEmpty) {
      final cloudPaint = Paint()
        ..color = Colors.red.withOpacity(0.3)
        ..strokeWidth = 0.05
        ..strokeCap = StrokeCap.round;

      canvas.drawPoints(ui.PointMode.points, localPoints, cloudPaint);
    }

    // Draw trajectory path
    if (path.isNotEmpty) {
      final paint = Paint()
        ..color = const Color(0xFF007AFF).withOpacity(0.6)
        ..strokeWidth = 0.1
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round;

      final pathObj = Path();
      pathObj.moveTo(path.first.dx, path.first.dy);
      for (var i = 1; i < path.length; i++) {
        pathObj.lineTo(path[i].dx, path[i].dy);
      }
      canvas.drawPath(pathObj, paint);
    }

    // Draw robot
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

      // Heading line
      canvas.drawLine(
          Offset.zero,
          const Offset(1.0, 0),
          Paint()
            ..color = Colors.black.withOpacity(0.5)
            ..strokeWidth = 0.05);

      canvas.restore();
    }

    // Origin
    final originPaint = Paint()..strokeWidth = 0.05;
    canvas.drawLine(const Offset(-1, 0), const Offset(1, 0),
        originPaint..color = Colors.grey.withOpacity(0.5));
    canvas.drawLine(const Offset(0, -1), const Offset(0, 1),
        originPaint..color = Colors.grey.withOpacity(0.5));
  }

  @override
  bool shouldRepaint(covariant TrajectoryPainter oldDelegate) {
    // 只在数据实际变化时重绘
    return dataVersion != oldDelegate.dataVersion ||
        _pathLength != oldDelegate._pathLength ||
        _poseX != oldDelegate._poseX ||
        _poseY != oldDelegate._poseY;
  }
}
