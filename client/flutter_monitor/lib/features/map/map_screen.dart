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
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/shared/widgets/glass_widgets.dart';
import 'package:flutter_monitor/features/map/robot_model_widget.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';

// ═══════════════════════════════════════════════════════════════
//  Mission Planner — Map Screen
// ═══════════════════════════════════════════════════════════════

enum TaskMode {
  navigation,
  mapping,
  patrol,
}

extension TaskModeX on TaskMode {
  String get displayName => switch (this) {
    TaskMode.navigation => 'Navigation',
    TaskMode.mapping => 'Mapping',
    TaskMode.patrol => 'Patrol',
  };

  TaskType get taskType => switch (this) {
    TaskMode.navigation => TaskType.TASK_TYPE_NAVIGATION,
    TaskMode.mapping => TaskType.TASK_TYPE_MAPPING,
    TaskMode.patrol => TaskType.TASK_TYPE_INSPECTION,
  };
}

class MapScreen extends StatefulWidget {
  const MapScreen({super.key});
  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen>
    with AutomaticKeepAliveClientMixin {
  // ─── Existing data ───
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
  int _mapDataVersion = 0;

  DateTime _lastPoseUpdate = DateTime(2000);
  static const _poseUpdateInterval = Duration(milliseconds: 200);

  // ─── Navigation goal ───
  Offset? _navGoalPoint;
  bool _isSettingGoal = false;

  // ─── Mission Planner state ───
  TaskMode _selectedMode = TaskMode.navigation;
  final _missionNameCtrl = TextEditingController();
  double _speedLimit = 1.5;
  // TODO(protocol): Add priority to task proto (or task metadata) and pass through startTask.
  int _priority = 0; // 0=Normal, 1=High, 2=Critical
  // TODO(protocol): Add obstacleOverride to navigation/mapping params once proto supports it.
  bool _obstacleOverride = false;

  // ─── Waypoints ───
  final List<_Waypoint> _waypoints = [];

  bool get _modeUsesGoalPoint => _selectedMode != TaskMode.mapping;
  String get _goalPointLabel =>
      _selectedMode == TaskMode.patrol ? '巡检目标点' : '导航目标点';

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startListening();
      _subscribeToMaps();
    });
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
    _missionNameCtrl.dispose();
    super.dispose();
  }

  // ─── Data subscriptions (unchanged) ───
  void _subscribeToMaps() {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;
    _mapSubscription = client
        .subscribeToResource(ResourceId()..type = ResourceType.RESOURCE_TYPE_MAP)
        .listen((chunk) {
      if (!mounted) return;
      _parseAndSetPoints(chunk.data, isGlobal: true);
    }, onError: (e) => debugPrint('[MapScreen] Map sub error: $e'));

    _pclSubscription = client
        .subscribeToResource(ResourceId()..type = ResourceType.RESOURCE_TYPE_POINTCLOUD)
        .listen((chunk) {
      if (!mounted) return;
      _parseAndSetPoints(chunk.data, isGlobal: false);
    }, onError: (e) => debugPrint('[MapScreen] PCL sub error: $e'));
  }

  void _parseAndSetPoints(List<int> data, {required bool isGlobal}) {
    if (data.isEmpty) return;
    final pointStep = isGlobal ? 12 : 16;
    final points = <Offset>[];
    final byteData = Uint8List.fromList(data).buffer.asByteData();
    final count = data.length ~/ pointStep;
    final stride = isGlobal ? 10 : 2;
    for (var i = 0; i < count; i += stride) {
      final offset = i * pointStep;
      if (offset + 8 <= data.length) {
        final x = byteData.getFloat32(offset, Endian.little);
        final y = byteData.getFloat32(offset + 4, Endian.little);
        if (x.isFinite && y.isFinite) points.add(Offset(x, y));
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
    final dogClient = provider.dogClient;
    if (dogClient != null && dogClient.isConnected) {
      _dogJointSub = dogClient.jointStream.listen((_) {
        if (!mounted) return;
        final angles = dogClient.jointPositions;
        if (angles != null) setState(() => _currentJointAngles = angles);
      });
    }
    _fastSub = provider.fastStateStream.listen((state) {
      if (!mounted) return;
      final now = DateTime.now();
      if (now.difference(_lastPoseUpdate) < _poseUpdateInterval) return;
      _lastPoseUpdate = now;
      final newPose = state.pose;
      final q = newPose.orientation;
      final siny = 2 * (q.w * q.z + q.x * q.y);
      final cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
      final newYaw = math.atan2(siny, cosy);
      final point = Offset(newPose.position.x, newPose.position.y);
      List<double>? angles;
      final dc = context.read<RobotConnectionProvider>().dogClient;
      if (dc != null && dc.isConnected) angles = dc.jointPositions;
      if (angles == null && state.jointAngles.isNotEmpty) {
        angles = state.jointAngles.map((a) => a.toDouble()).toList();
      }
      setState(() {
        _currentPose = newPose;
        _currentYaw = newYaw;
        _currentJointAngles = angles;
        if (_path.isEmpty || (_path.last - point).distance > 0.05) {
          _path.add(point);
          if (_path.length > 5000) _path.removeRange(0, 1000);
        }
      });
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

  void _handleMapTap(TapDownDetails details) {
    if (!_isSettingGoal || _show3DModel) return;
    final matrix = _transformController.value.clone();
    final inv = Matrix4.tryInvert(matrix);
    if (inv == null) return;
    final sp = details.localPosition;
    final mx = inv[0] * sp.dx + inv[4] * sp.dy + inv[12];
    final my = inv[1] * sp.dx + inv[5] * sp.dy + inv[13];
    HapticFeedback.mediumImpact();
    final point = Offset(mx, -my);
    setState(() {
      _navGoalPoint = point;
      if (_selectedMode == TaskMode.patrol) {
        final idx = _waypoints.length + 1;
        _waypoints.add(_Waypoint(
          'Patrol #$idx',
          'x=${point.dx.toStringAsFixed(2)}, y=${point.dy.toStringAsFixed(2)}',
          null,
          point: point,
        ));
      } else {
        _isSettingGoal = false;
      }
    });
  }

  Future<void> _startSelectedTask() async {
    final tg = context.read<TaskGateway>();
    if (tg.isRunning) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
          content: const Text('已有任务执行中'),
          backgroundColor: AppColors.warning,
          behavior: SnackBarBehavior.floating,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        ));
      }
      return;
    }

    if (_selectedMode == TaskMode.navigation && _navGoalPoint == null) {
      if (mounted) Navigator.of(context).pushNamed('/task-panel');
      return;
    }

    HapticFeedback.mediumImpact();
    final missionName = _missionNameCtrl.text.trim();
    late final bool ok;
    late final String okMsg;
    switch (_selectedMode) {
      case TaskMode.navigation:
        // TODO(protocol): Wire _priority/_obstacleOverride into task request after proto update.
        ok = await tg.startSingleGoalNavigation(
          x: _navGoalPoint!.dx,
          y: _navGoalPoint!.dy,
          label: missionName,
          maxSpeed: _speedLimit,
        );
        okMsg = '导航任务已启动';
        break;
      case TaskMode.mapping:
        final mapName = missionName.isNotEmpty
            ? missionName
            : 'map_${DateTime.now().millisecondsSinceEpoch}';
        // TODO(protocol): Wire _priority/_obstacleOverride into mapping request when supported.
        ok = await tg.startTask(
          _selectedMode.taskType,
          mappingParams: MappingParams()
            ..mapName = mapName
            ..saveOnComplete = true,
        );
        okMsg = '建图任务已启动';
        break;
      case TaskMode.patrol:
        final patrolGoals = _waypoints
            .where((wp) => wp.point != null)
            .map((wp) => NavigationGoal()
              ..position = (Vector3()
                ..x = wp.point!.dx
                ..y = wp.point!.dy
                ..z = 0)
              ..yaw = 0
              ..arrivalRadius = 1.0
              ..label = wp.name)
            .toList();
        if (patrolGoals.isEmpty) {
          if (mounted) {
            ScaffoldMessenger.of(context).showSnackBar(SnackBar(
              content: const Text('请先添加至少一个巡检航点'),
              backgroundColor: AppColors.warning,
              behavior: SnackBarBehavior.floating,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
            ));
          }
          return;
        }
        // TODO(protocol): Wire _priority/_obstacleOverride into inspection request when supported.
        ok = await tg.startTask(
          _selectedMode.taskType,
          navigationParams: NavigationParams()
            ..waypoints.addAll(patrolGoals)
            ..loop = true
            ..maxSpeed = _speedLimit,
        );
        okMsg = '巡检任务已启动';
        break;
    }

    final msg = ok ? okMsg : (tg.statusMessage ?? '启动失败');
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok ? msg : UiErrorMapper.fromMessage(msg)),
        backgroundColor: ok ? AppColors.success : AppColors.error,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ));
    }
  }

  Future<void> _saveMap() async {
    HapticFeedback.mediumImpact();
    final name = 'map_${DateTime.now().millisecondsSinceEpoch}';
    final (ok, msg) = await context.read<MapGateway>().saveMap('/maps/$name.pcd');
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok ? '地图已保存: $name.pcd' : UiErrorMapper.fromMessage(msg)),
        backgroundColor: ok ? AppColors.success : AppColors.error,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ));
    }
  }

  // ═══════════════════════════════════════════════════════════
  //  BUILD
  // ═══════════════════════════════════════════════════════════
  @override
  Widget build(BuildContext context) {
    super.build(context);
    final isDesktop = !context.isMobile;
    final conn = context.watch<RobotConnectionProvider>();
    final online = conn.isConnected;

    return Scaffold(
      backgroundColor: Colors.transparent,
      body: isDesktop
          ? _buildDesktopLayout(context, online)
          : _buildMobileLayout(context, online),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  DESKTOP — Mission Planner split layout
  // ═══════════════════════════════════════════════════════════
  Widget _buildDesktopLayout(BuildContext context, bool online) {
    return Row(
      children: [
        // ── Left Panel ──
        SizedBox(
          width: 280,
          child: _buildLeftPanel(context, online),
        ),

        // ── Map + Waypoint area ──
        Expanded(
          child: Column(
            children: [
              // Map toolbar
              _buildMapToolbar(context, online),
              // Map view
              Expanded(child: _buildMapArea(context)),
              // Waypoint timeline
              _buildWaypointTimeline(context),
            ],
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  MOBILE — Full screen map with overlays
  // ═══════════════════════════════════════════════════════════
  Widget _buildMobileLayout(BuildContext context, bool online) {
    return Stack(
      children: [
        Positioned.fill(child: _buildMapArea(context)),
        // Top controls
        Positioned(
          left: 16, top: MediaQuery.of(context).padding.top + 16,
          child: GlassCard(
            borderRadius: 18,
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Container(width: 8, height: 8, decoration: BoxDecoration(
                color: online ? AppColors.success : AppColors.error,
                shape: BoxShape.circle,
              )),
              const SizedBox(width: 8),
              Text(context.watch<RobotProfileProvider>().current.name,
                style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600, color: context.titleColor)),
            ]),
          ),
        ),
        // Right side controls
        Positioned(
          right: 12, top: MediaQuery.of(context).padding.top + 12,
          child: _buildMapControls(context),
        ),
        // FABs
        Positioned(
          right: 16, bottom: 100,
          child: _buildFabColumn(context),
        ),
        // Goal setting indicator
        if (_isSettingGoal) _buildGoalSettingBanner(context),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  LEFT PANEL — Task Modes + Parameters
  // ═══════════════════════════════════════════════════════════
  Widget _buildLeftPanel(BuildContext context, bool online) {
    final dark = context.isDark;
    return Container(
      decoration: BoxDecoration(
        color: dark ? Colors.white.withValues(alpha: 0.03) : Colors.white.withValues(alpha: 0.5),
        border: Border(right: BorderSide(
          color: dark ? Colors.white.withValues(alpha: 0.06) : Colors.black.withValues(alpha: 0.06),
        )),
      ),
      child: ListView(
        padding: const EdgeInsets.all(20),
        children: [
          // ── TASK MODE ──
          Text('TASK MODE', style: TextStyle(
            fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 1.2,
            color: context.subtitleColor,
          )),
          const SizedBox(height: 14),
          _buildTaskModeGrid(context),

          const SizedBox(height: 24),
          Divider(color: context.borderColor.withValues(alpha: 0.5)),
          const SizedBox(height: 16),

          // ── PARAMETERS ──
          Row(
            children: [
              Text('PARAMETERS', style: TextStyle(
                fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 1.2,
                color: context.subtitleColor,
              )),
              const Spacer(),
              GestureDetector(
                onTap: () => setState(() {
                  _missionNameCtrl.clear();
                  _speedLimit = 1.5;
                  _priority = 0;
                  _obstacleOverride = false;
                }),
                child: Text('Reset\nDefaults', textAlign: TextAlign.center,
                  style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600, color: AppColors.primary, height: 1.2)),
              ),
            ],
          ),
          const SizedBox(height: 16),

          // Mission Name
          _paramLabel('Mission Name'),
          const SizedBox(height: 6),
          _paramInput(_missionNameCtrl, 'e.g. Warehouse Alpha'),
          const SizedBox(height: 16),

          // Robot Selection
          _paramLabel('Robot Selection'),
          const SizedBox(height: 6),
          _buildRobotDropdown(context),
          const SizedBox(height: 16),

          // Speed + Priority row
          Row(children: [
            Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              _paramLabel('Speed Limit'),
              const SizedBox(height: 6),
              _buildSpeedField(context),
            ])),
            const SizedBox(width: 12),
            Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              _paramLabel('Priority'),
              const SizedBox(height: 6),
              _buildPriorityDropdown(context),
            ])),
          ]),
          const SizedBox(height: 16),

          // Obstacle avoidance
          Row(children: [
            SizedBox(
              width: 20, height: 20,
              child: Checkbox(
                value: _obstacleOverride,
                onChanged: (v) => setState(() => _obstacleOverride = v ?? false),
                materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
                visualDensity: VisualDensity.compact,
              ),
            ),
            const SizedBox(width: 10),
            Expanded(child: Text('Enable obstacle\navoidance override',
              style: TextStyle(fontSize: 12, color: context.subtitleColor, height: 1.3))),
          ]),
        ],
      ),
    );
  }

  Widget _buildTaskModeGrid(BuildContext context) {
    const modes = [
      (TaskMode.navigation, Icons.navigation_rounded, Color(0xFF6366F1)),
      (TaskMode.mapping, Icons.map_rounded, Color(0xFFEA580C)),
      (TaskMode.patrol, Icons.shield_rounded, Color(0xFF0EA5E9)),
    ];
    return Wrap(
      spacing: 10,
      runSpacing: 10,
      children: [
        for (final mode in modes)
          _TaskModeButton(
            icon: mode.$2,
            label: mode.$1.displayName,
            color: mode.$3,
            isSelected: _selectedMode == mode.$1,
            onTap: () => setState(() {
              _selectedMode = mode.$1;
              if (!_modeUsesGoalPoint) _isSettingGoal = false;
            }),
          ),
      ],
    );
  }

  Widget _paramLabel(String text) => Text(text, style: TextStyle(
    fontSize: 12, fontWeight: FontWeight.w600, color: context.titleColor,
  ));

  Widget _paramInput(TextEditingController ctrl, String hint) {
    return TextField(
      controller: ctrl,
      style: TextStyle(fontSize: 13, color: context.titleColor),
      decoration: InputDecoration(
        hintText: hint,
        hintStyle: TextStyle(color: context.hintColor, fontSize: 13),
        contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
        isDense: true,
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(10),
          borderSide: BorderSide(color: context.borderColor),
        ),
        enabledBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(10),
          borderSide: BorderSide(color: context.borderColor),
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(10),
          borderSide: const BorderSide(color: AppColors.primary, width: 1.5),
        ),
      ),
    );
  }

  Widget _buildRobotDropdown(BuildContext context) {
    final name = context.watch<RobotProfileProvider>().current.name;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: context.borderColor),
      ),
      child: Row(children: [
        Expanded(child: Text('Unit 734 - "$name"',
          style: TextStyle(fontSize: 13, color: context.titleColor),
          overflow: TextOverflow.ellipsis,
        )),
        Icon(Icons.keyboard_arrow_down_rounded, size: 18, color: context.subtitleColor),
      ]),
    );
  }

  Widget _buildSpeedField(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: context.borderColor),
      ),
      child: Row(children: [
        Text('${_speedLimit.toStringAsFixed(1)}',
          style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: context.titleColor)),
        const SizedBox(width: 4),
        Text('m/s', style: TextStyle(fontSize: 11, color: context.subtitleColor)),
      ]),
    );
  }

  Widget _buildPriorityDropdown(BuildContext context) {
    const labels = ['Normal', 'High', 'Critical'];
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: context.borderColor),
      ),
      child: Row(children: [
        Text(labels[_priority],
          style: TextStyle(fontSize: 13, color: context.titleColor)),
        const Spacer(),
        Icon(Icons.keyboard_arrow_down_rounded, size: 18, color: context.subtitleColor),
      ]),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  MAP TOOLBAR — Live feed indicator + view toggles
  // ═══════════════════════════════════════════════════════════
  Widget _buildMapToolbar(BuildContext context, bool online) {
    final dark = context.isDark;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 12),
      decoration: BoxDecoration(
        border: Border(bottom: BorderSide(
          color: dark ? Colors.white.withValues(alpha: 0.06) : Colors.black.withValues(alpha: 0.06),
        )),
      ),
      child: Row(
        children: [
          // View toggle icons
          _toolbarIconBtn(Icons.navigation_rounded, tooltip: '导航视图',
            isActive: _show3DModel, onTap: () => setState(() => _show3DModel = true)),
          const SizedBox(width: 4),
          _toolbarIconBtn(Icons.location_on_rounded, tooltip: '2D地图',
            isActive: !_show3DModel, onTap: () => setState(() => _show3DModel = false)),
          const SizedBox(width: 4),
          _toolbarIconBtn(Icons.route_rounded, tooltip: '轨迹',
            isActive: _showGlobalMap, onTap: () => setState(() => _showGlobalMap = !_showGlobalMap)),

          const SizedBox(width: 16),
          // Live Feed indicator
          if (online) Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            decoration: BoxDecoration(
              color: dark ? Colors.white.withValues(alpha: 0.06) : Colors.white,
              borderRadius: BorderRadius.circular(20),
              border: Border.all(color: context.borderColor),
            ),
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Container(width: 6, height: 6, decoration: const BoxDecoration(
                color: AppColors.success, shape: BoxShape.circle,
              )),
              const SizedBox(width: 8),
              Text('Live Feed Active', style: TextStyle(
                fontSize: 11, fontWeight: FontWeight.w600, color: context.titleColor,
              )),
            ]),
          ),

          const Spacer(),

          // Status badge
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            decoration: BoxDecoration(
              color: online
                  ? AppColors.success.withValues(alpha: 0.08)
                  : AppColors.error.withValues(alpha: 0.08),
              borderRadius: BorderRadius.circular(20),
            ),
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Container(width: 6, height: 6, decoration: BoxDecoration(
                color: online ? AppColors.success : AppColors.error,
                shape: BoxShape.circle,
              )),
              const SizedBox(width: 6),
              Text(online ? 'System Ready' : 'Offline', style: TextStyle(
                fontSize: 11, fontWeight: FontWeight.w600,
                color: online ? AppColors.success : AppColors.error,
              )),
            ]),
          ),
          const SizedBox(width: 8),
          Icon(Icons.help_outline_rounded, size: 20, color: context.subtitleColor),
        ],
      ),
    );
  }

  Widget _toolbarIconBtn(IconData icon, {required String tooltip,
      required bool isActive, required VoidCallback onTap}) {
    final dark = context.isDark;
    return Tooltip(
      message: tooltip,
      child: GestureDetector(
        onTap: onTap,
        child: Container(
          width: 36, height: 36,
          decoration: BoxDecoration(
            color: isActive
                ? (dark ? Colors.white.withValues(alpha: 0.1) : Colors.black.withValues(alpha: 0.06))
                : Colors.transparent,
            borderRadius: BorderRadius.circular(10),
          ),
          child: Icon(icon, size: 18, color: isActive ? context.titleColor : context.subtitleColor),
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  MAP AREA — 3D / 2D rendering + controls overlay
  // ═══════════════════════════════════════════════════════════
  Widget _buildMapArea(BuildContext context) {
    return Stack(
      children: [
        // 3D Model
        if (_show3DModel)
          Positioned.fill(
            child: RepaintBoundary(child: RobotModelWidget(
              currentPose: _currentPose, jointAngles: _currentJointAngles,
            )),
          ),
        // 2D Map
        if (!_show3DModel)
          Positioned.fill(
            child: GestureDetector(
              onTapDown: (_isSettingGoal && _modeUsesGoalPoint)
                  ? _handleMapTap
                  : null,
              child: GridPaper(
                color: Colors.black12, interval: 100, divisions: 1, subdivisions: 5,
                child: InteractiveViewer(
                  transformationController: _transformController,
                  boundaryMargin: const EdgeInsets.all(5000),
                  minScale: 0.1, maxScale: 100.0,
                  child: SizedBox(
                    width: 10000, height: 10000,
                    child: RepaintBoundary(child: CustomPaint(
                      painter: TrajectoryPainter(
                        path: _path, currentPose: _currentPose,
                        globalPoints: _showGlobalMap ? _globalMapPoints : const [],
                        localPoints: _localCloudPoints,
                        navGoalPoint: _navGoalPoint,
                        dataVersion: _mapDataVersion,
                      ),
                      size: const Size(10000, 10000),
                    )),
                  ),
                ),
              ),
            ),
          ),
        // Nav goal overlay
        if (_navGoalPoint != null && !_show3DModel && _modeUsesGoalPoint)
          Positioned.fill(child: IgnorePointer(child: CustomPaint(
            painter: _NavGoalPainter(goalPoint: _navGoalPoint!, transform: _transformController.value),
          ))),
        // Goal setting banner
        if (_isSettingGoal) _buildGoalSettingBanner(context),

        // Desktop map controls (right side)
        if (!context.isMobile)
          Positioned(
            right: 16, top: 16,
            child: _buildMapControls(context),
          ),

        // Desktop FABs
        if (!context.isMobile)
          Positioned(
            right: 16, bottom: 16,
            child: _buildFabColumn(context),
          ),
      ],
    );
  }

  Widget _buildMapControls(BuildContext context) {
    return GlassCard(
      borderRadius: 12,
      padding: const EdgeInsets.all(6),
      child: Column(mainAxisSize: MainAxisSize.min, children: [
        _mapCtrlBtn(Icons.add, onTap: () {
          final cur = _transformController.value.clone();
          cur.scale(1.2);
          _transformController.value = cur;
        }),
        const SizedBox(height: 4),
        _mapCtrlBtn(Icons.remove, onTap: () {
          final cur = _transformController.value.clone();
          cur.scale(0.8);
          _transformController.value = cur;
        }),
        const SizedBox(height: 4),
        _mapCtrlBtn(Icons.layers_rounded, onTap: () {
          HapticFeedback.selectionClick();
          setState(() => _showGlobalMap = !_showGlobalMap);
        }),
        const SizedBox(height: 4),
        // Compass / recenter
        Tooltip(
          message: 'Yaw: ${(_currentYaw * 180 / math.pi).toStringAsFixed(0)}°',
          child: GestureDetector(
            onTap: _recenter,
            child: Container(
              width: 36, height: 36,
              decoration: BoxDecoration(borderRadius: BorderRadius.circular(8)),
              child: Transform.rotate(
                angle: -_currentYaw,
                child: Icon(Icons.navigation_rounded, size: 18, color: AppColors.primary),
              ),
            ),
          ),
        ),
      ]),
    );
  }

  Widget _mapCtrlBtn(IconData icon, {required VoidCallback onTap}) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 36, height: 36,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(8),
        ),
        child: Icon(icon, size: 18, color: context.subtitleColor),
      ),
    );
  }

  Widget _buildFabColumn(BuildContext context) {
    return Column(mainAxisSize: MainAxisSize.min, children: [
      _MapFab(
        icon: _isSettingGoal ? Icons.close : Icons.add_location_alt_outlined,
        tooltip: '设置$_goalPointLabel',
        onPressed: () {
          HapticFeedback.selectionClick();
          if (!_modeUsesGoalPoint) {
            ScaffoldMessenger.of(context).showSnackBar(SnackBar(
              content: const Text('建图模式无需设置目标点'),
              backgroundColor: AppColors.warning,
              behavior: SnackBarBehavior.floating,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
            ));
            return;
          }
          setState(() {
            _isSettingGoal = !_isSettingGoal;
            if (!_isSettingGoal && _selectedMode == TaskMode.navigation) {
              _navGoalPoint = null;
            }
          });
        },
      ),
      const SizedBox(height: 6),
      _MapFab(
        icon: Icons.navigation_outlined,
        tooltip: '启动任务',
        onPressed: _startSelectedTask,
      ),
      const SizedBox(height: 6),
      _MapFab(icon: Icons.folder_outlined, tooltip: '地图管理',
        onPressed: () => Navigator.of(context).pushNamed('/map-manager')),
      const SizedBox(height: 6),
      _MapFab(icon: Icons.save_outlined, tooltip: '保存地图',
        onPressed: _saveMap),
    ]);
  }

  Widget _buildGoalSettingBanner(BuildContext context) {
    return Positioned(
      top: MediaQuery.of(context).padding.top + 60,
      left: 0, right: 0,
      child: Center(child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: AppColors.warning.withValues(alpha: 0.9),
          borderRadius: BorderRadius.circular(20),
        ),
        child: Text('点击地图设置$_goalPointLabel', style: const TextStyle(
          color: Colors.white, fontSize: 13, fontWeight: FontWeight.w600,
        )),
      )),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  WAYPOINT TIMELINE — bottom bar
  // ═══════════════════════════════════════════════════════════
  Widget _buildWaypointTimeline(BuildContext context) {
    final dark = context.isDark;
    return Container(
      padding: const EdgeInsets.fromLTRB(20, 14, 20, 14),
      decoration: BoxDecoration(
        color: dark ? Colors.white.withValues(alpha: 0.03) : Colors.white.withValues(alpha: 0.6),
        border: Border(top: BorderSide(
          color: dark ? Colors.white.withValues(alpha: 0.06) : Colors.black.withValues(alpha: 0.06),
        )),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: [
          // Header
          Row(children: [
            Text('WAYPOINT SEQUENCE', style: TextStyle(
              fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 1.2,
              color: context.subtitleColor,
            )),
            const Spacer(),
            GestureDetector(
              onTap: () {},
              child: Icon(Icons.edit_rounded, size: 16, color: context.subtitleColor),
            ),
            const SizedBox(width: 12),
            GestureDetector(
              onTap: () => setState(() => _waypoints.clear()),
              child: Icon(Icons.delete_outline_rounded, size: 16, color: context.subtitleColor),
            ),
          ]),
          const SizedBox(height: 14),

          // Timeline
          SizedBox(
            height: 60,
            child: ListView.separated(
              scrollDirection: Axis.horizontal,
              itemCount: _waypoints.length + 1, // +1 for add button
              separatorBuilder: (_, i) => i < _waypoints.length
                  ? _waypointArrow(_waypoints[math.min(i, _waypoints.length - 1)].eta)
                  : const SizedBox.shrink(),
              itemBuilder: (_, i) {
                if (i == _waypoints.length) {
                  return GestureDetector(
                    onTap: () => setState(() => _isSettingGoal = true),
                    child: Container(
                      width: 40, height: 40,
                      margin: const EdgeInsets.symmetric(vertical: 10),
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        border: Border.all(color: context.borderColor, width: 1.5),
                      ),
                      child: Icon(Icons.add, size: 18, color: context.subtitleColor),
                    ),
                  );
                }
                final wp = _waypoints[i];
                final isFirst = i == 0;
                final isLast = i == _waypoints.length - 1;
                return _WaypointChip(
                  index: i + 1, name: wp.name, action: wp.action,
                  isFirst: isFirst, isLast: isLast,
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _waypointArrow(String? eta) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 18),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          if (eta != null)
            Text(eta, style: TextStyle(fontSize: 9, color: context.subtitleColor)),
          Icon(Icons.arrow_forward_rounded, size: 16, color: context.subtitleColor),
        ],
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Helper types & widgets
// ═══════════════════════════════════════════════════════════════

class _Waypoint {
  final String name;
  final String action;
  final String? eta;
  final Offset? point;
  _Waypoint(this.name, this.action, this.eta, {this.point});
}

class _TaskModeButton extends StatelessWidget {
  final IconData icon;
  final String label;
  final Color color;
  final bool isSelected;
  final VoidCallback onTap;

  const _TaskModeButton({
    required this.icon, required this.label, required this.color,
    required this.isSelected, required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 115,
        padding: const EdgeInsets.symmetric(vertical: 14),
        decoration: BoxDecoration(
          color: isSelected
              ? color.withValues(alpha: dark ? 0.2 : 0.1)
              : (dark ? Colors.white.withValues(alpha: 0.04) : Colors.white),
          borderRadius: BorderRadius.circular(16),
          border: Border.all(
            color: isSelected ? color.withValues(alpha: 0.4) : context.borderColor,
            width: isSelected ? 1.5 : 1,
          ),
          boxShadow: isSelected
              ? [BoxShadow(color: color.withValues(alpha: 0.12), blurRadius: 8, offset: const Offset(0, 2))]
              : null,
        ),
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          Container(
            width: 40, height: 40,
            decoration: BoxDecoration(
              color: isSelected ? color : color.withValues(alpha: 0.1),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(icon, size: 20,
              color: isSelected ? Colors.white : color),
          ),
          const SizedBox(height: 8),
          Text(label, style: TextStyle(
            fontSize: 11, fontWeight: FontWeight.w600,
            color: isSelected ? color : context.subtitleColor,
          )),
        ]),
      ),
    );
  }
}

class _WaypointChip extends StatelessWidget {
  final int index;
  final String name;
  final String action;
  final bool isFirst;
  final bool isLast;

  const _WaypointChip({
    required this.index, required this.name, required this.action,
    required this.isFirst, required this.isLast,
  });

  @override
  Widget build(BuildContext context) {
    return Row(mainAxisSize: MainAxisSize.min, children: [
      // Number circle
      Container(
        width: 28, height: 28,
        decoration: BoxDecoration(
          color: isLast ? Colors.transparent : AppColors.primary,
          shape: BoxShape.circle,
          border: isLast ? Border.all(color: AppColors.primary, width: 2) : null,
        ),
        child: Center(child: Text('$index', style: TextStyle(
          fontSize: 12, fontWeight: FontWeight.w700,
          color: isLast ? AppColors.primary : Colors.white,
        ))),
      ),
      const SizedBox(width: 10),
      // Info
      Column(crossAxisAlignment: CrossAxisAlignment.start, mainAxisSize: MainAxisSize.min, children: [
        Text(name, style: TextStyle(
          fontSize: 13, fontWeight: FontWeight.w600, color: context.titleColor,
        )),
        Text(action, style: TextStyle(fontSize: 11, color: context.subtitleColor)),
      ]),
      if (isLast) ...[
        const SizedBox(width: 10),
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
          decoration: BoxDecoration(
            color: AppColors.primary.withValues(alpha: 0.1),
            borderRadius: BorderRadius.circular(6),
          ),
          child: Text('Target', style: TextStyle(
            fontSize: 10, fontWeight: FontWeight.w600, color: AppColors.primary,
          )),
        ),
      ],
    ]);
  }
}

class _MapFab extends StatelessWidget {
  final IconData icon;
  final String tooltip;
  final VoidCallback onPressed;

  const _MapFab({required this.icon, required this.tooltip, required this.onPressed});

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return Tooltip(
      message: tooltip,
      child: GestureDetector(
        onTap: onPressed,
        child: Container(
          width: 40, height: 40,
          decoration: BoxDecoration(
            color: dark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: context.borderColor),
          ),
          child: Icon(icon, size: 18, color: context.subtitleColor),
        ),
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Custom Painters (unchanged)
// ═══════════════════════════════════════════════════════════════

class _NavGoalPainter extends CustomPainter {
  final Offset goalPoint;
  final Matrix4 transform;
  _NavGoalPainter({required this.goalPoint, required this.transform});

  @override
  void paint(Canvas canvas, Size size) {
    final m = transform;
    final sx = m[0] * goalPoint.dx + m[4] * (-goalPoint.dy) + m[12];
    final sy = m[1] * goalPoint.dx + m[5] * (-goalPoint.dy) + m[13];
    final center = Offset(sx, sy);
    canvas.drawCircle(center, 16, Paint()..color = const Color(0xFFFF3B30).withValues(alpha: 0.2)..style = PaintingStyle.fill);
    canvas.drawCircle(center, 16, Paint()..color = const Color(0xFFFF3B30)..style = PaintingStyle.stroke..strokeWidth = 2);
    canvas.drawCircle(center, 5, Paint()..color = const Color(0xFFFF3B30)..style = PaintingStyle.fill);
  }

  @override
  bool shouldRepaint(covariant _NavGoalPainter old) =>
      goalPoint != old.goalPoint || transform != old.transform;
}

class TrajectoryPainter extends CustomPainter {
  final List<Offset> path;
  final Pose? currentPose;
  final List<Offset> globalPoints;
  final List<Offset> localPoints;
  final Offset? navGoalPoint;
  final int dataVersion;
  final int _pathLength;
  final double? _poseX;
  final double? _poseY;

  TrajectoryPainter({
    required this.path, this.currentPose,
    this.globalPoints = const [], this.localPoints = const [],
    this.navGoalPoint, this.dataVersion = 0,
  })  : _pathLength = path.length,
        _poseX = currentPose?.position.x,
        _poseY = currentPose?.position.y;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.translate(size.width / 2, size.height / 2);
    canvas.scale(1.0, -1.0);
    if (globalPoints.isNotEmpty) {
      canvas.drawPoints(ui.PointMode.points, globalPoints, Paint()
        ..color = Colors.black12..strokeWidth = 0.1..strokeCap = StrokeCap.round);
    }
    if (localPoints.isNotEmpty) {
      canvas.drawPoints(ui.PointMode.points, localPoints, Paint()
        ..color = Colors.red.withValues(alpha: 0.3)..strokeWidth = 0.05..strokeCap = StrokeCap.round);
    }
    if (path.isNotEmpty) {
      final p = Path()..moveTo(path.first.dx, path.first.dy);
      for (var i = 1; i < path.length; i++) p.lineTo(path[i].dx, path[i].dy);
      canvas.drawPath(p, Paint()
        ..color = const Color(0xFF007AFF).withValues(alpha: 0.6)
        ..strokeWidth = 0.1..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round..strokeJoin = StrokeJoin.round);
    }
    if (currentPose != null) {
      final pos = currentPose!.position;
      final q = currentPose!.orientation;
      final yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      canvas.save();
      canvas.translate(pos.x, pos.y);
      canvas.rotate(yaw);
      canvas.drawPath(
        Path()..moveTo(0.25, 0)..lineTo(-0.25, 0.15)..lineTo(-0.25, -0.15)..close(),
        Paint()..color = const Color(0xFFFF3B30)..style = PaintingStyle.fill,
      );
      canvas.drawLine(Offset.zero, const Offset(1.0, 0), Paint()
        ..color = Colors.black.withValues(alpha: 0.5)..strokeWidth = 0.05);
      canvas.restore();
    }
    if (navGoalPoint != null) {
      canvas.save();
      canvas.translate(navGoalPoint!.dx, navGoalPoint!.dy);
      final gp = Paint()..color = const Color(0xFF34C759)..style = PaintingStyle.stroke..strokeWidth = 0.08;
      canvas.drawCircle(Offset.zero, 0.5, gp);
      canvas.drawCircle(Offset.zero, 0.15, Paint()..color = const Color(0xFF34C759)..style = PaintingStyle.fill);
      canvas.drawLine(const Offset(-0.3, 0), const Offset(0.3, 0), gp);
      canvas.drawLine(const Offset(0, -0.3), const Offset(0, 0.3), gp);
      canvas.restore();
    }
    final op = Paint()..strokeWidth = 0.05;
    canvas.drawLine(const Offset(-1, 0), const Offset(1, 0), op..color = Colors.grey.withValues(alpha: 0.5));
    canvas.drawLine(const Offset(0, -1), const Offset(0, 1), op..color = Colors.grey.withValues(alpha: 0.5));
  }

  @override
  bool shouldRepaint(covariant TrajectoryPainter old) =>
      dataVersion != old.dataVersion || _pathLength != old._pathLength ||
      _poseX != old._poseX || _poseY != old._poseY || navGoalPoint != old.navGoalPoint;
}
