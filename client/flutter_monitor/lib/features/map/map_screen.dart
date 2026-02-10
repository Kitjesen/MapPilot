import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/shared/widgets/glass_widgets.dart';
import 'package:flutter_monitor/features/map/robot_model_widget.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';

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
  List<double>? _currentJointAngles;
  StreamSubscription<FastState>? _fastSub;
  StreamSubscription? _mapSubscription;
  StreamSubscription? _pclSubscription;
  StreamSubscription? _dogJointSub;

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

  // ─── Navigation goal ───
  Offset? _navGoalPoint;  // 地图坐标
  bool _isSettingGoal = false;  // 是否处于点击设置目标模式

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
    _dogJointSub?.cancel();
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

    // Dog board joint stream → update robot model joint angles
    final dogClient = provider.dogClient;
    if (dogClient != null && dogClient.isConnected) {
      _dogJointSub = dogClient.jointStream.listen((_) {
        if (!mounted) return;
        final angles = dogClient.jointPositions;
        if (angles != null) {
          setState(() => _currentJointAngles = angles);
        }
      });
    }

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

      // 关节角度: 优先从 Dog Board 直连获取, 否则从 FastState proto
      List<double>? angles;
      final dogClient =
          context.read<RobotConnectionProvider>().dogClient;
      if (dogClient != null && dogClient.isConnected) {
        angles = dogClient.jointPositions;
      }
      if (angles == null && state.jointAngles.isNotEmpty) {
        angles = state.jointAngles.map((a) => a.toDouble()).toList();
      }

      setState(() {
        _currentPose = newPose;
        _currentYaw = newYaw;
        _currentJointAngles = angles;

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
        title: const Text('轨迹地图'),
        backgroundColor: Colors.transparent,
        elevation: 0,
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 16.0),
            child: IconButton(
              icon: Icon(Icons.delete_outline,
                  color: context.isDark ? Colors.white54 : Colors.black54),
              onPressed: _clearPath,
              tooltip: '清除路径',
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
                  jointAngles: _currentJointAngles,
                ),
              ),
            ),

          // 2D Map Background (fallback)
          if (!_show3DModel)
            Positioned.fill(
              child: GestureDetector(
                onTapDown: _isSettingGoal ? _handleMapTap : null,
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
                            navGoalPoint: _navGoalPoint,
                            dataVersion: _mapDataVersion,
                          ),
                          size: const Size(10000, 10000),
                        ),
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
              borderRadius: 18,
              padding:
                  const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Selector<RobotConnectionProvider, bool>(
                    selector: (_, p) => p.isConnected,
                    builder: (_, isConnected, __) {
                      return Container(
                        width: 8,
                        height: 8,
                        decoration: BoxDecoration(
                          color: isConnected ? AppColors.success : AppColors.error,
                          shape: BoxShape.circle,
                        ),
                      );
                    },
                  ),
                  const SizedBox(width: 8),
                  Text(
                    context.watch<RobotProfileProvider>().current.name,
                    style: TextStyle(
                      fontSize: 14,
                      fontWeight: FontWeight.w600,
                      color: context.isDark ? Colors.white : Colors.black87,
                    ),
                  ),
                ],
              ),
            ),
          ),

          // Top Right: Scene Options Panel
          Positioned(
            right: 12,
            top: MediaQuery.of(context).padding.top + 12,
            child: GlassCard(
              borderRadius: 12,
              padding: const EdgeInsets.all(6),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  _buildIconButton(
                    icon: _show3DModel ? Icons.view_in_ar : Icons.map,
                    tooltip: _show3DModel ? '切换2D' : '切换3D',
                    isSelected: true,
                    onPressed: () {
                      HapticFeedback.selectionClick();
                      setState(() => _show3DModel = !_show3DModel);
                    },
                  ),
                  const SizedBox(height: 4),
                  _buildIconButton(
                    icon: Icons.layers,
                    tooltip: '地图图层',
                    isSelected: _showGlobalMap,
                    onPressed: () {
                      HapticFeedback.selectionClick();
                      setState(() => _showGlobalMap = !_showGlobalMap);
                    },
                  ),
                  const SizedBox(height: 4),
                  _buildIconButton(
                    icon: Icons.my_location,
                    tooltip: '居中',
                    onPressed: _recenter,
                  ),
                  const SizedBox(height: 4),
                  _buildIconButton(
                    icon: Icons.settings_outlined,
                    tooltip: '设置',
                    onPressed: () => _showSettingsDialog(context),
                  ),
                ],
              ),
            ),
          ),

          // Bottom Left: Compass + Path info
          Positioned(
            left: 16,
            bottom: 100,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                GlassCard(
                  borderRadius: 24,
                  padding: const EdgeInsets.all(10),
                  child: Transform.rotate(
                    angle: -_currentYaw,
                    child: const Icon(Icons.navigation,
                        color: Color(0xFF007AFF), size: 22),
                  ),
                ),
                if (_path.isNotEmpty) ...[
                  const SizedBox(height: 6),
                  GlassCard(
                    borderRadius: 12,
                    padding:
                        const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                    child: Text(
                      '${_path.length} pts',
                      style: TextStyle(
                        fontSize: 10,
                        fontWeight: FontWeight.w500,
                        color: context.subtitleColor,
                      ),
                    ),
                  ),
                ],
              ],
            ),
          ),

          // ─── Navigation Goal marker ───
          if (_navGoalPoint != null && !_show3DModel)
            Positioned.fill(
              child: IgnorePointer(
                child: CustomPaint(
                  painter: _NavGoalPainter(
                    goalPoint: _navGoalPoint!,
                    transform: _transformController.value,
                  ),
                ),
              ),
            ),

          // ─── Goal Setting Mode Indicator ───
          if (_isSettingGoal)
            Positioned(
              top: MediaQuery.of(context).padding.top + 60,
              left: 0,
              right: 0,
              child: Center(
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                  decoration: BoxDecoration(
                    color: AppColors.warning.withValues(alpha: 0.9),
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: const Text(
                    '点击地图设置导航目标',
                    style: TextStyle(
                      color: Colors.white,
                      fontSize: 13,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ),
              ),
            ),

          // ─── FAB Action Buttons ───
          Positioned(
            right: 16,
            bottom: 90,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                // Set Goal mode toggle
                _MapFab(
                  icon: _isSettingGoal ? Icons.close : Icons.add_location_alt_outlined,
                  tooltip: '设置导航目标',
                  onPressed: () {
                    HapticFeedback.selectionClick();
                    setState(() {
                      _isSettingGoal = !_isSettingGoal;
                      if (!_isSettingGoal) _navGoalPoint = null;
                    });
                  },
                ),
                const SizedBox(height: 6),
                _MapFab(
                  icon: Icons.navigation_outlined,
                  tooltip: '启动导航',
                  onPressed: _navGoalPoint != null
                      ? () => _startNavigation()
                      : () {
                          Navigator.of(context).pushNamed('/task-panel');
                        },
                ),
                const SizedBox(height: 6),
                _MapFab(
                  icon: Icons.folder_outlined,
                  tooltip: '地图管理',
                  onPressed: () => Navigator.of(context).pushNamed('/map-manager'),
                ),
                const SizedBox(height: 6),
                _MapFab(
                  icon: Icons.save_outlined,
                  tooltip: '保存地图',
                  onPressed: () => _saveMap(),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  /// Convert screen tap to map coordinates (2D mode only)
  void _handleMapTap(TapDownDetails details) {
    if (!_isSettingGoal || _show3DModel) return;

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

    // Map coords are y-flipped in our painter
    HapticFeedback.mediumImpact();
    setState(() {
      _navGoalPoint = Offset(mapX, -mapY);
      _isSettingGoal = false;
    });
  }

  Future<void> _startNavigation() async {
    if (_navGoalPoint == null) return;
    final taskGateway = context.read<TaskGateway>();
    if (taskGateway.isRunning) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: const Text('已有任务执行中，请先取消或等待完成'),
            backgroundColor: AppColors.warning,
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
          ),
        );
      }
      return;
    }

    HapticFeedback.mediumImpact();
    final ok = await taskGateway.startSingleGoalNavigation(
      x: _navGoalPoint!.dx,
      y: _navGoalPoint!.dy,
    );
    final msg = ok ? '导航任务已启动' : (taskGateway.statusMessage ?? '启动失败');

    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(ok ? msg : UiErrorMapper.fromMessage(msg)),
          backgroundColor: ok ? AppColors.success : AppColors.error,
          behavior: SnackBarBehavior.floating,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        ),
      );
    }
  }

  Future<void> _saveMap() async {
    HapticFeedback.mediumImpact();
    final name = 'map_${DateTime.now().millisecondsSinceEpoch}';
    final (ok, msg) = await context.read<MapGateway>().saveMap('/maps/$name.pcd');
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(ok ? '地图已保存: $name.pcd' : UiErrorMapper.fromMessage(msg)),
          backgroundColor: ok ? AppColors.success : AppColors.error,
          behavior: SnackBarBehavior.floating,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        ),
      );
    }
  }

  Widget _buildIconButton({
    required IconData icon,
    required VoidCallback onPressed,
    String? tooltip,
    bool isSelected = false,
  }) {
    final isDark = context.isDark;
    return Container(
      decoration: BoxDecoration(
        color: isSelected
            ? (isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.05))
            : Colors.transparent,
        borderRadius: BorderRadius.circular(8),
      ),
      child: IconButton(
        icon: Icon(icon),
        color: isSelected
            ? context.titleColor
            : context.subtitleColor,
        iconSize: 18,
        tooltip: tooltip,
        onPressed: onPressed,
        constraints: const BoxConstraints(minWidth: 38, minHeight: 38),
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
          borderRadius: 14,
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    'Settings',
                    style: TextStyle(
                        fontSize: 15,
                        fontWeight: FontWeight.w600,
                        color: context.titleColor),
                  ),
                  IconButton(
                    icon: Icon(Icons.close, size: 18, color: context.subtitleColor),
                    onPressed: () => Navigator.pop(context),
                    padding: EdgeInsets.zero,
                    constraints: const BoxConstraints(),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              Text('Theme',
                  style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w500,
                      color: context.subtitleColor)),
              const SizedBox(height: 10),
              Row(
                children: [
                  _buildThemeOption('Light', false),
                  const SizedBox(width: 8),
                  _buildThemeOption('Dark', true),
                  const SizedBox(width: 8),
                  _buildThemeOption('System', false),
                ],
              ),
              const SizedBox(height: 16),
              Text('Visualization',
                  style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w500,
                      color: context.subtitleColor)),
              const SizedBox(height: 10),
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
    final isDark = context.isDark;
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 10),
        decoration: BoxDecoration(
          color: isSelected
              ? (isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.04))
              : Colors.transparent,
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: context.borderColor),
        ),
        child: Center(
          child: Text(
            label,
            style: TextStyle(
              fontSize: 12,
              fontWeight: isSelected ? FontWeight.w600 : FontWeight.w400,
              color: isSelected ? context.titleColor : context.subtitleColor,
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
            activeColor: context.titleColor,
          ),
        ],
      ),
    );
  }
}

/// Small floating action button for map actions
class _MapFab extends StatelessWidget {
  final IconData icon;
  final String tooltip;
  final VoidCallback onPressed;

  const _MapFab({
    required this.icon,
    required this.tooltip,
    required this.onPressed,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return Tooltip(
      message: tooltip,
      child: GestureDetector(
        onTap: onPressed,
        child: Container(
          width: 40,
          height: 40,
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: context.borderColor),
          ),
          child: Icon(icon, size: 18, color: context.subtitleColor),
        ),
      ),
    );
  }
}

/// Paints a navigation goal marker on the map
class _NavGoalPainter extends CustomPainter {
  final Offset goalPoint;
  final Matrix4 transform;

  _NavGoalPainter({required this.goalPoint, required this.transform});

  @override
  void paint(Canvas canvas, Size size) {
    // Transform goal map coords → screen coords
    final m = transform;
    final screenX = m[0] * goalPoint.dx + m[4] * (-goalPoint.dy) + m[12];
    final screenY = m[1] * goalPoint.dx + m[5] * (-goalPoint.dy) + m[13];

    final center = Offset(screenX, screenY);

    // Outer ring
    canvas.drawCircle(
      center,
      16,
      Paint()
        ..color = const Color(0xFFFF3B30).withValues(alpha: 0.2)
        ..style = PaintingStyle.fill,
    );
    canvas.drawCircle(
      center,
      16,
      Paint()
        ..color = const Color(0xFFFF3B30)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2,
    );
    // Inner dot
    canvas.drawCircle(
      center,
      5,
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

class TrajectoryPainter extends CustomPainter {
  final List<Offset> path;
  final Pose? currentPose;
  final List<Offset> globalPoints;
  final List<Offset> localPoints;
  final Offset? navGoalPoint;
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
    this.navGoalPoint,
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
        ..color = Colors.red.withValues(alpha: 0.3)
        ..strokeWidth = 0.05
        ..strokeCap = StrokeCap.round;

      canvas.drawPoints(ui.PointMode.points, localPoints, cloudPaint);
    }

    // Draw trajectory path
    if (path.isNotEmpty) {
      final paint = Paint()
        ..color = const Color(0xFF007AFF).withValues(alpha: 0.6)
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
            ..color = Colors.black.withValues(alpha: 0.5)
            ..strokeWidth = 0.05);

      canvas.restore();
    }

    // Draw navigation goal marker
    if (navGoalPoint != null) {
      canvas.save();
      canvas.translate(navGoalPoint!.dx, navGoalPoint!.dy);

      // Outer ring
      final goalPaint = Paint()
        ..color = const Color(0xFF34C759)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 0.08;
      canvas.drawCircle(Offset.zero, 0.5, goalPaint);

      // Inner dot
      final dotPaint = Paint()
        ..color = const Color(0xFF34C759)
        ..style = PaintingStyle.fill;
      canvas.drawCircle(Offset.zero, 0.15, dotPaint);

      // Crosshair
      canvas.drawLine(const Offset(-0.3, 0), const Offset(0.3, 0), goalPaint);
      canvas.drawLine(const Offset(0, -0.3), const Offset(0, 0.3), goalPaint);

      canvas.restore();
    }

    // Origin
    final originPaint = Paint()..strokeWidth = 0.05;
    canvas.drawLine(const Offset(-1, 0), const Offset(1, 0),
        originPaint..color = Colors.grey.withValues(alpha: 0.5));
    canvas.drawLine(const Offset(0, -1), const Offset(0, 1),
        originPaint..color = Colors.grey.withValues(alpha: 0.5));
  }

  @override
  bool shouldRepaint(covariant TrajectoryPainter oldDelegate) {
    // 只在数据实际变化时重绘
    return dataVersion != oldDelegate.dataVersion ||
        _pathLength != oldDelegate._pathLength ||
        _poseX != oldDelegate._poseX ||
        _poseY != oldDelegate._poseY ||
        navGoalPoint != oldDelegate.navGoalPoint;
  }
}
