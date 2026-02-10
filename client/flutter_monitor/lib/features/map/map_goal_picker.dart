import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/features/map/map_screen.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:robot_proto/src/control.pb.dart' as ctrl;
import 'package:robot_proto/src/common.pb.dart' as common;

/// 地图选点页面 — 用户点击地图选择一个目标点，确认后返回 NavigationGoal
class MapGoalPicker extends StatefulWidget {
  const MapGoalPicker({super.key});

  @override
  State<MapGoalPicker> createState() => _MapGoalPickerState();
}

class _MapGoalPickerState extends State<MapGoalPicker> {
  final List<Offset> _path = [];
  List<Offset> _globalMapPoints = [];

  Pose? _currentPose;
  StreamSubscription<FastState>? _fastSub;
  StreamSubscription? _mapSubscription;

  final TransformationController _transformController =
      TransformationController();

  int _mapDataVersion = 0;
  Offset? _selectedPoint; // 选中的地图坐标

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startListening();
      _subscribeToMaps();
    });
    final matrix = Matrix4.identity()
      ..translate(180.0, 350.0)
      ..scale(3.0, 3.0);
    _transformController.value = matrix;
  }

  @override
  void dispose() {
    _fastSub?.cancel();
    _mapSubscription?.cancel();
    super.dispose();
  }

  void _startListening() {
    final provider = context.read<RobotConnectionProvider>();
    _fastSub = provider.fastStateStream.listen((state) {
      if (!mounted) return;
      final point = Offset(state.pose.position.x, state.pose.position.y);
      setState(() {
        _currentPose = state.pose;
        if (_path.isEmpty || (_path.last - point).distance > 0.05) {
          _path.add(point);
          _mapDataVersion++;
        }
      });
    });
  }

  void _subscribeToMaps() {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;
    _mapSubscription = client
        .subscribeToResource(ResourceId()..type = ResourceType.RESOURCE_TYPE_MAP)
        .listen((chunk) {
      if (!mounted) return;
      // Parse simple x,y point pairs from binary data
      _parseMapChunk(chunk.data);
    }, onError: (e) {
      debugPrint('[MapGoalPicker] Map subscription error: $e');
    });
  }

  void _parseMapChunk(List<int> data) {
    if (data.length < 12) return;
    final byteData = ByteData.sublistView(Uint8List.fromList(data));
    final pts = <Offset>[];
    // Assume float32 x,y,z triplets
    for (int i = 0; i + 11 < data.length; i += 12) {
      final x = byteData.getFloat32(i, Endian.little);
      final y = byteData.getFloat32(i + 4, Endian.little);
      pts.add(Offset(x, y));
    }
    if (pts.isNotEmpty) {
      setState(() {
        _globalMapPoints = pts;
        _mapDataVersion++;
      });
    }
  }

  void _handleMapTap(TapDownDetails details) {
    final matrix = _transformController.value.clone();
    final inverted = Matrix4.tryInvert(matrix);
    if (inverted == null) return;

    final screenPoint = details.localPosition;
    final mapX = inverted[0] * screenPoint.dx +
        inverted[4] * screenPoint.dy +
        inverted[12];
    final mapY = inverted[1] * screenPoint.dx +
        inverted[5] * screenPoint.dy +
        inverted[13];

    HapticFeedback.mediumImpact();
    setState(() {
      _selectedPoint = Offset(mapX, -mapY);
    });
  }

  void _confirmSelection() {
    if (_selectedPoint == null) return;
    HapticFeedback.mediumImpact();

    final goal = ctrl.NavigationGoal()
      ..position = (common.Vector3()
        ..x = _selectedPoint!.dx
        ..y = _selectedPoint!.dy
        ..z = 0)
      ..arrivalRadius = 1.0;

    Navigator.of(context).pop(goal);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: context.isDark ? AppColors.darkBackground : AppColors.lightBackground,
      appBar: AppBar(
        title: const Text('选择目标点'),
        leading: IconButton(
          icon: const Icon(Icons.close, size: 20),
          onPressed: () => Navigator.of(context).pop(),
        ),
        actions: [
          if (_selectedPoint != null)
            TextButton(
              onPressed: _confirmSelection,
              child: Text(
                '确认',
                style: TextStyle(
                  fontSize: 14,
                  fontWeight: FontWeight.w600,
                  color: context.titleColor,
                ),
              ),
            ),
        ],
      ),
      body: Stack(
        children: [
          // 地图画布
          GestureDetector(
            onTapDown: _handleMapTap,
            child: InteractiveViewer(
              transformationController: _transformController,
              boundaryMargin: const EdgeInsets.all(double.infinity),
              minScale: 0.1,
              maxScale: 50.0,
              child: CustomPaint(
                size: Size.infinite,
                painter: TrajectoryPainter(
                  path: _path,
                  currentPose: _currentPose,
                  globalPoints: _globalMapPoints,
                  localPoints: const [],
                  navGoalPoint: _selectedPoint,
                  dataVersion: _mapDataVersion,
                ),
              ),
            ),
          ),

          // 选中点标记
          if (_selectedPoint != null)
            Positioned.fill(
              child: IgnorePointer(
                child: CustomPaint(
                  painter: _NavGoalPainter(
                    goalPoint: _selectedPoint!,
                    transform: _transformController.value,
                  ),
                ),
              ),
            ),

          // 提示条
          Positioned(
            top: 12,
            left: 0,
            right: 0,
            child: Center(
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 7),
                decoration: BoxDecoration(
                  color: context.isDark ? AppColors.darkCard : Colors.white,
                  borderRadius: BorderRadius.circular(AppRadius.card),
                  boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
                ),
                child: Text(
                  _selectedPoint == null
                      ? '点击地图选择目标点'
                      : '已选择 (${_selectedPoint!.dx.toStringAsFixed(1)}, ${_selectedPoint!.dy.toStringAsFixed(1)})',
                  style: TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w500,
                    color: _selectedPoint == null
                        ? context.subtitleColor
                        : context.titleColor,
                  ),
                ),
              ),
            ),
          ),

          // 确认按钮（底部）
          if (_selectedPoint != null)
            Positioned(
              bottom: MediaQuery.of(context).padding.bottom + 20,
              left: 20,
              right: 20,
              child: SizedBox(
                height: 44,
                child: TextButton(
                  onPressed: _confirmSelection,
                  style: TextButton.styleFrom(
                    foregroundColor: context.titleColor,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(AppRadius.card),
                      side: BorderSide(color: context.borderColor),
                    ),
                    backgroundColor: context.isDark
                        ? Colors.white.withValues(alpha:0.05)
                        : Colors.black.withValues(alpha:0.03),
                  ),
                  child: Text(
                    '确认选择 (${_selectedPoint!.dx.toStringAsFixed(1)}, ${_selectedPoint!.dy.toStringAsFixed(1)})',
                    style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w500),
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}

/// 导航目标标记绘制（复用 map_screen 的同名类）
class _NavGoalPainter extends CustomPainter {
  final Offset goalPoint;
  final Matrix4 transform;

  _NavGoalPainter({required this.goalPoint, required this.transform});

  @override
  void paint(Canvas canvas, Size size) {
    final m = transform;
    final screenX = m[0] * goalPoint.dx + m[4] * (-goalPoint.dy) + m[12];
    final screenY = m[1] * goalPoint.dx + m[5] * (-goalPoint.dy) + m[13];
    final center = Offset(screenX, screenY);

    canvas.drawCircle(
      center, 16,
      Paint()
        ..color = const Color(0xFFFF3B30).withValues(alpha:0.2)
        ..style = PaintingStyle.fill,
    );
    canvas.drawCircle(
      center, 16,
      Paint()
        ..color = const Color(0xFFFF3B30)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2,
    );
    canvas.drawCircle(
      center, 5,
      Paint()
        ..color = const Color(0xFFFF3B30)
        ..style = PaintingStyle.fill,
    );
  }

  @override
  bool shouldRepaint(covariant _NavGoalPainter oldDelegate) {
    return goalPoint != oldDelegate.goalPoint ||
        transform != oldDelegate.transform;
  }
}
