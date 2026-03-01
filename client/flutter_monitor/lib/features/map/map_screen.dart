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
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/gateway/runtime_config_gateway.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/shared/widgets/glass_widgets.dart';
import 'package:flutter_monitor/features/map/robot_model_widget.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';

// ═══════════════════════════════════════════════════════════════
//  Mission Planner — Map Screen
// ═══════════════════════════════════════════════════════════════

enum TaskMode {
  navigation,
  mapping,
  patrol,
  semanticNav,
  followPerson,
}

extension TaskModeX on TaskMode {
  String localizedName(LocaleProvider locale) => switch (this) {
    TaskMode.navigation => locale.tr('导航', 'Navigation'),
    TaskMode.mapping => locale.tr('建图', 'Mapping'),
    TaskMode.patrol => locale.tr('巡检', 'Patrol'),
    TaskMode.semanticNav => locale.tr('语义', 'Semantic'),
    TaskMode.followPerson => locale.tr('跟随', 'Follow'),
  };

  String get displayName => switch (this) {
    TaskMode.navigation => 'Navigation',
    TaskMode.mapping => 'Mapping',
    TaskMode.patrol => 'Patrol',
    TaskMode.semanticNav => 'Semantic',
    TaskMode.followPerson => 'Follow',
  };

  TaskType get taskType => switch (this) {
    TaskMode.navigation => TaskType.TASK_TYPE_NAVIGATION,
    TaskMode.mapping => TaskType.TASK_TYPE_MAPPING,
    TaskMode.patrol => TaskType.TASK_TYPE_INSPECTION,
    TaskMode.semanticNav => TaskType.TASK_TYPE_SEMANTIC_NAV,
    TaskMode.followPerson => TaskType.TASK_TYPE_FOLLOW_PERSON,
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
  List<List<Offset>> _globalMapBuckets = List.generate(5, (_) => []);
  List<List<Offset>> _localCloudBuckets = List.generate(5, (_) => []);

  // ─── Global path & occupancy grid ───
  List<Offset> _globalPathPoints = [];
  _OccupancyGrid? _occupancyGrid;
  StreamSubscription? _pathSubscription;
  StreamSubscription? _gridSubscription;
  bool _showGlobalPath = true;
  bool _showOccupancyGrid = true;

  // ─── Frontier markers ───
  List<_FrontierMarker> _frontierMarkers = [];
  StreamSubscription? _frontierSubscription;
  bool _showFrontiers = true;

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
  final _semanticInstructionCtrl = TextEditingController();
  bool _exploreIfUnknown = true;
  double _speedLimit = 1.5;
  // TODO(protocol): Add priority to task proto (or task metadata) and pass through startTask.
  int _priority = 0; // 0=Normal, 1=High, 2=Critical
  // TODO(protocol): Add obstacleOverride to navigation/mapping params once proto supports it.
  bool _obstacleOverride = false;

  // ─── Follow Person state ───
  final _followPersonTargetCtrl = TextEditingController();
  double _followPersonDistance = 1.5;

  // ─── Waypoints ───
  final List<_Waypoint> _waypoints = [];

  // ─── Long-press waypoints (temporary markers on map) ───
  final List<Offset> _longPressMarkers = [];

  // ─── Measurement tool ───
  bool _isMeasuring = false;
  Offset? _measureStart; // world coordinates
  Offset? _measureEnd;   // world coordinates

  // ─── Rotation lock ───
  bool _rotationLocked = false;

  // ─── Active waypoints (from backend) ───
  List<ActiveWaypoint> _activeWaypoints = [];
  WaypointSource _waypointSource = WaypointSource.WAYPOINT_SOURCE_NONE;
  int _activeWaypointCurrentIndex = 0;
  Timer? _waypointPollTimer;
  bool _wasRunning = false;

  // ─── Geofence alert ───
  StreamSubscription? _geofenceSub;
  String _geofenceState = 'NO_FENCE';
  double _geofenceMargin = 0;

  bool get _modeUsesGoalPoint =>
      _selectedMode != TaskMode.mapping &&
      _selectedMode != TaskMode.semanticNav &&
      _selectedMode != TaskMode.followPerson;
  String _goalPointLabel(LocaleProvider locale) =>
      _selectedMode == TaskMode.patrol
          ? locale.tr('巡检目标点', 'Patrol target')
          : locale.tr('导航目标点', 'Nav target');

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startListening();
      _subscribeToMaps();
      // 从 RuntimeConfigGateway 同步速度限制
      final rcGw = context.read<RuntimeConfigGateway>();
      setState(() => _speedLimit = rcGw.config.maxSpeed);
    });
    final matrix = Matrix4.identity()
      ..translate(200.0, 300.0)
      ..scale(20.0);
    _transformController.value = matrix;
  }

  @override
  void dispose() {
    _waypointPollTimer?.cancel();
    _geofenceSub?.cancel();
    _fastSub?.cancel();
    _mapSubscription?.cancel();
    _pclSubscription?.cancel();
    _pathSubscription?.cancel();
    _gridSubscription?.cancel();
    _frontierSubscription?.cancel();
    _dogJointSub?.cancel();
    _transformController.dispose();
    _missionNameCtrl.dispose();
    _semanticInstructionCtrl.dispose();
    _followPersonTargetCtrl.dispose();
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

    _pathSubscription = client
        .subscribeToResource(ResourceId()
          ..type = ResourceType.RESOURCE_TYPE_PATH
          ..name = 'global_path')
        .listen((chunk) {
      if (!mounted) return;
      _parseGlobalPath(chunk.data);
    }, onError: (e) => debugPrint('[MapScreen] Path sub error: $e'));

    _gridSubscription = client
        .subscribeToResource(ResourceId()..type = ResourceType.RESOURCE_TYPE_OCCUPANCY_GRID)
        .listen((chunk) {
      if (!mounted) return;
      _parseOccupancyGrid(chunk.data);
    }, onError: (e) => debugPrint('[MapScreen] Grid sub error: $e'));

    _frontierSubscription = client
        .subscribeToResource(ResourceId()..type = ResourceType.RESOURCE_TYPE_FRONTIER_MARKERS)
        .listen((chunk) {
      if (!mounted) return;
      _parseFrontierMarkers(chunk.data);
    }, onError: (_) {/* frontier not available — silent */});
  }

  void _parseAndSetPoints(List<int> data, {required bool isGlobal}) {
    if (data.isEmpty) return;
    final pointStep = isGlobal ? 12 : 16;
    final bd = Uint8List.fromList(data).buffer.asByteData();
    final count = data.length ~/ pointStep;
    final stride = isGlobal ? 10 : 2;
    final pts = <Offset>[];
    final zs = <double>[];
    for (var i = 0; i < count; i += stride) {
      final off = i * pointStep;
      if (off + 12 <= data.length) {
        final x = bd.getFloat32(off, Endian.little);
        final y = bd.getFloat32(off + 4, Endian.little);
        final z = bd.getFloat32(off + 8, Endian.little);
        if (x.isFinite && y.isFinite && z.isFinite) {
          pts.add(Offset(x, y));
          zs.add(z);
        }
      }
    }
    // Z-range for viridis bucketing
    var zMin = double.maxFinite, zMax = double.negativeInfinity;
    for (final z in zs) {
      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;
    }
    final range = (zMax - zMin).clamp(0.01, double.infinity);
    final buckets = List.generate(5, (_) => <Offset>[]);
    for (var i = 0; i < pts.length; i++) {
      final t = ((zs[i] - zMin) / range).clamp(0.0, 1.0);
      buckets[(t * 4.99).floor().clamp(0, 4)].add(pts[i]);
    }
    setState(() {
      _mapDataVersion++;
      if (isGlobal) {
        _globalMapBuckets = buckets;
      } else {
        _localCloudBuckets = buckets;
      }
    });
  }

  /// Global path wire format (from SerializePathWithMeta in data_service.cpp):
  ///   Bytes 0-3:   num_poses      (uint32 LE)
  ///   Bytes 4-7:   frame_id_len   (uint32 LE)
  ///   Bytes 8..(8+frame_id_len-1): frame_id string (skip)
  ///   Per pose × 56 bytes: 7×float64 LE  (x, y, z, qx, qy, qz, qw)
  void _parseGlobalPath(List<int> data) {
    if (data.length < 8) return;
    final bd = Uint8List.fromList(data).buffer.asByteData();
    final numPoses = bd.getUint32(0, Endian.little);
    final frameIdLen = bd.getUint32(4, Endian.little);
    final poseStart = 8 + frameIdLen;
    if (numPoses == 0 || data.length < poseStart + numPoses * 56) return;
    final points = <Offset>[];
    for (var i = 0; i < numPoses; i++) {
      final off = poseStart + i * 56;
      final x = bd.getFloat64(off, Endian.little);
      final y = bd.getFloat64(off + 8, Endian.little);
      if (x.isFinite && y.isFinite) points.add(Offset(x, y));
    }
    setState(() {
      _globalPathPoints = points;
      _mapDataVersion++;
    });
  }

  /// Occupancy grid binary format:
  ///   Bytes 0-3:  width (int32 LE)
  ///   Bytes 4-7:  height (int32 LE)
  ///   Bytes 8-11: resolution m/cell (float32 LE)
  ///   Bytes 12-15: origin_x (float32 LE)
  ///   Bytes 16-19: origin_y (float32 LE)
  ///   Bytes 20+:  width*height uint8 (0=free, 128=unknown, 255=occupied)
  void _parseOccupancyGrid(List<int> data) {
    if (data.length < 20) return;
    final bd = Uint8List.fromList(data).buffer.asByteData();
    final w = bd.getInt32(0, Endian.little);
    final h = bd.getInt32(4, Endian.little);
    final res = bd.getFloat32(8, Endian.little);
    final ox = bd.getFloat32(12, Endian.little);
    final oy = bd.getFloat32(16, Endian.little);
    if (w <= 0 || h <= 0 || res <= 0 || data.length < 20 + w * h) return;
    final gridData = Uint8List.fromList(data.sublist(20, 20 + w * h));
    setState(() {
      _occupancyGrid = _OccupancyGrid(
          width: w, height: h, resolution: res, originX: ox, originY: oy, data: gridData);
      _mapDataVersion++;
    });
  }

  /// Frontier markers binary format:
  ///   Bytes 0-3:   count (uint32 LE)
  ///   Per marker × 12 bytes: float32 x, float32 y, float32 score (LE)
  ///   score range [0,1] — higher = more informative frontier
  void _parseFrontierMarkers(List<int> data) {
    if (data.length < 4) return;
    final bd = Uint8List.fromList(data).buffer.asByteData();
    final count = bd.getUint32(0, Endian.little);
    if (count == 0 || data.length < 4 + count * 12) return;
    final markers = <_FrontierMarker>[];
    for (var i = 0; i < count; i++) {
      final off = 4 + i * 12;
      final x = bd.getFloat32(off, Endian.little);
      final y = bd.getFloat32(off + 4, Endian.little);
      final score = bd.getFloat32(off + 8, Endian.little);
      if (x.isFinite && y.isFinite && score.isFinite) {
        markers.add(_FrontierMarker(x: x, y: y, score: score.clamp(0.0, 1.0)));
      }
    }
    setState(() {
      _frontierMarkers = markers;
      _mapDataVersion++;
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
    // Subscribe to geofence data from SlowState
    _geofenceSub = provider.slowStateStream.listen((ss) {
      if (!mounted) return;
      final g = ss.geofence;
      final newState = g.state.isEmpty ? 'NO_FENCE' : g.state;
      if (newState != _geofenceState || g.marginDistance != _geofenceMargin) {
        setState(() {
          _geofenceState = newState;
          _geofenceMargin = g.marginDistance;
        });
      }
    });

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
    if (_show3DModel) return;

    // ── Measurement mode ──
    if (_isMeasuring) {
      final matrix = _transformController.value.clone();
      final inv = Matrix4.tryInvert(matrix);
      if (inv == null) return;
      final sp = details.localPosition;
      final mx = inv[0] * sp.dx + inv[4] * sp.dy + inv[12];
      final my = inv[1] * sp.dx + inv[5] * sp.dy + inv[13];
      HapticFeedback.lightImpact();
      final point = Offset(mx, -my);
      setState(() {
        if (_measureStart == null || _measureEnd != null) {
          // First click or restart: set start
          _measureStart = point;
          _measureEnd = null;
        } else {
          // Second click: set end
          _measureEnd = point;
        }
      });
      return;
    }

    if (!_isSettingGoal) return;
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

  void _handleMapLongPress(LongPressStartDetails details) {
    if (_show3DModel) return;
    final matrix = _transformController.value.clone();
    final inv = Matrix4.tryInvert(matrix);
    if (inv == null) return;
    final sp = details.localPosition;
    final mx = inv[0] * sp.dx + inv[4] * sp.dy + inv[12];
    final my = inv[1] * sp.dx + inv[5] * sp.dy + inv[13];
    HapticFeedback.heavyImpact();
    final worldPoint = Offset(mx, -my);
    setState(() => _longPressMarkers.add(worldPoint));
    final locale = context.read<LocaleProvider>();
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(locale.tr(
        '长按添加航点: (${worldPoint.dx.toStringAsFixed(2)}, ${worldPoint.dy.toStringAsFixed(2)})',
        'Long-press waypoint: (${worldPoint.dx.toStringAsFixed(2)}, ${worldPoint.dy.toStringAsFixed(2)})',
      )),
      behavior: SnackBarBehavior.floating,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      duration: const Duration(seconds: 2),
    ));
  }

  Future<void> _startSelectedTask() async {
    final tg = context.read<TaskGateway>();
    final locale = context.read<LocaleProvider>();
    if (tg.isRunning) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
          content: Text(locale.tr('已有任务执行中', 'A task is already running')),
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

    if (_selectedMode == TaskMode.semanticNav &&
        _semanticInstructionCtrl.text.trim().isEmpty) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
          content: Text(locale.tr('请输入导航指令', 'Please enter a navigation instruction')),
          backgroundColor: AppColors.warning,
          behavior: SnackBarBehavior.floating,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        ));
      }
      return;
    }

    // Check for active waypoints before starting a new task
    final active = await tg.getActiveWaypoints();
    if (active != null &&
        active.source != WaypointSource.WAYPOINT_SOURCE_NONE &&
        active.source != WaypointSource.WAYPOINT_SOURCE_UNSPECIFIED &&
        active.totalCount > 0) {
      if (!mounted) return;
      final proceed = await _showActiveWaypointDialog(active);
      if (!proceed) return;
      await tg.clearWaypoints();
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
        okMsg = locale.tr('导航任务已启动', 'Navigation task started');
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
        okMsg = locale.tr('建图任务已启动', 'Mapping task started');
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
              content: Text(locale.tr('请先添加至少一个巡检航点', 'Please add at least one patrol waypoint')),
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
        okMsg = locale.tr('巡检任务已启动', 'Patrol task started');
        break;
      case TaskMode.semanticNav:
        ok = await tg.startSemanticNav(
          _semanticInstructionCtrl.text,
          exploreIfUnknown: _exploreIfUnknown,
        );
        okMsg = locale.tr('语义导航任务已启动', 'Semantic nav task started');
        if (ok) _semanticInstructionCtrl.clear();
        break;
      case TaskMode.followPerson:
        ok = await tg.startFollowPerson(
          _followPersonTargetCtrl.text.trim().isEmpty
              ? 'person'
              : _followPersonTargetCtrl.text.trim(),
          followDistance: _followPersonDistance,
        );
        okMsg = locale.tr('跟随任务已启动', 'Follow task started');
        break;
    }

    final msg = ok ? okMsg : (tg.statusMessage ?? locale.tr('启动失败', 'Failed to start'));
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok ? msg : UiErrorMapper.fromMessage(msg)),
        backgroundColor: ok ? AppColors.success : AppColors.error,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ));
    }
  }

  // ─── Active waypoint polling ───
  void _syncWaypointPolling(bool isRunning) {
    if (isRunning && !_wasRunning) {
      _startWaypointPolling();
    } else if (!isRunning && _wasRunning) {
      _stopWaypointPolling();
    }
    _wasRunning = isRunning;
  }

  void _startWaypointPolling() {
    _waypointPollTimer?.cancel();
    _pollActiveWaypoints(); // immediate first poll
    _waypointPollTimer = Timer.periodic(const Duration(seconds: 2), (_) {
      _pollActiveWaypoints();
    });
  }

  void _stopWaypointPolling() {
    _waypointPollTimer?.cancel();
    _waypointPollTimer = null;
    if (mounted) {
      setState(() {
        _activeWaypoints = [];
        _waypointSource = WaypointSource.WAYPOINT_SOURCE_NONE;
        _activeWaypointCurrentIndex = 0;
      });
    }
  }

  Future<void> _pollActiveWaypoints() async {
    final tg = context.read<TaskGateway>();
    final resp = await tg.getActiveWaypoints();
    if (resp != null && mounted) {
      setState(() {
        _activeWaypoints = resp.waypoints.toList();
        _waypointSource = resp.source;
        _activeWaypointCurrentIndex = resp.currentIndex;
      });
    }
  }

  // ─── Active waypoint check dialog ───
  Future<bool> _showActiveWaypointDialog(GetActiveWaypointsResponse active) async {
    final locale = context.read<LocaleProvider>();
    final sourceLabel = switch (active.source) {
      WaypointSource.WAYPOINT_SOURCE_APP => locale.tr('App 任务', 'App task'),
      WaypointSource.WAYPOINT_SOURCE_PLANNER => locale.tr('全局规划器', 'Global planner'),
      _ => locale.tr('未知', 'Unknown'),
    };
    final progressPct = (active.progressPercent * 100).toStringAsFixed(0);
    final result = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(locale.tr('存在活跃航点', 'Active waypoints exist')),
        content: Text(
          locale.tr(
            '当前有 ${active.totalCount} 个来自「$sourceLabel」的航点正在执行。\n'
            '进度: $progressPct%\n\n'
            '是否清除当前航点并启动新任务？',
            '${active.totalCount} waypoints from "$sourceLabel" are running.\n'
            'Progress: $progressPct%\n\n'
            'Clear current waypoints and start a new task?',
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(false),
            child: Text(locale.tr('取消', 'Cancel')),
          ),
          FilledButton(
            onPressed: () => Navigator.of(ctx).pop(true),
            child: Text(locale.tr('清除并继续', 'Clear & continue')),
          ),
        ],
      ),
    );
    return result ?? false;
  }

  Future<void> _saveMap() async {
    HapticFeedback.mediumImpact();
    final locale = context.read<LocaleProvider>();
    final name = 'map_${DateTime.now().millisecondsSinceEpoch}';
    final (ok, msg) = await context.read<MapGateway>().saveMap('/maps/$name.pcd');
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok
            ? locale.tr('地图已保存: $name.pcd', 'Map saved: $name.pcd')
            : UiErrorMapper.fromMessage(msg)),
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
    final locale = context.watch<LocaleProvider>();
    final isDesktop = !context.isMobile;
    final online = context.select<RobotConnectionProvider, bool>((p) => p.isConnected);

    // Sync waypoint polling lifecycle outside of build tree
    final isRunning = context.watch<TaskGateway>().isRunning;
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      _syncWaypointPolling(isRunning);
    });

    return Scaffold(
      backgroundColor: Colors.transparent,
      body: isDesktop
          ? _buildDesktopLayout(context, online, locale)
          : _buildMobileLayout(context, online, locale),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  DESKTOP — Mission Planner split layout
  // ═══════════════════════════════════════════════════════════
  Widget _buildDesktopLayout(BuildContext context, bool online, LocaleProvider locale) {
    return Row(
      children: [
        // ── Left Panel ──
        SizedBox(
          width: 280,
          child: _buildLeftPanel(context, online, locale),
        ),

        // ── Map + Waypoint area ──
        Expanded(
          child: Column(
            children: [
              // Map toolbar
              _buildMapToolbar(context, online, locale),
              // Map view + task status overlay
              Expanded(child: Stack(children: [
                Positioned.fill(child: _buildMapArea(context)),
                _buildTaskStatusBar(context),
              ])),
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
  Widget _buildMobileLayout(BuildContext context, bool online, LocaleProvider locale) {
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
              Text(context.select<RobotProfileProvider, String>((p) => p.current.name),
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
        // Task status bar (mobile)
        _buildTaskStatusBar(context),
        // Goal setting indicator
        if (_isSettingGoal) _buildGoalSettingBanner(context),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  LEFT PANEL — Task Modes + Parameters
  // ═══════════════════════════════════════════════════════════
  Widget _buildLeftPanel(BuildContext context, bool online, LocaleProvider locale) {
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
          Text(locale.tr('任务模式', 'TASK MODE'), style: TextStyle(
            fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 1.2,
            color: context.subtitleColor,
          )),
          const SizedBox(height: 14),
          _buildTaskModeGrid(context, locale),

          const SizedBox(height: 24),
          Divider(color: context.borderColor.withValues(alpha: 0.5)),
          const SizedBox(height: 16),

          // ── PARAMETERS ──
          Row(
            children: [
              Text(locale.tr('参数设置', 'PARAMETERS'), style: TextStyle(
                fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 1.2,
                color: context.subtitleColor,
              )),
              const Spacer(),
              GestureDetector(
                onTap: () {
                  final defaultSpeed = context.read<RuntimeConfigGateway>().config.maxSpeed;
                  setState(() {
                    _missionNameCtrl.clear();
                    _speedLimit = defaultSpeed;
                    _priority = 0;
                    _obstacleOverride = false;
                  });
                },
                child: Text(locale.tr('恢复\n默认', 'Reset\nDefaults'), textAlign: TextAlign.center,
                  style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600, color: AppColors.primary, height: 1.2)),
              ),
            ],
          ),
          const SizedBox(height: 16),

          // Semantic Navigation Instruction (visible only in semanticNav mode)
          if (_selectedMode == TaskMode.semanticNav) ...[
            _paramLabel(locale.tr('语义导航指令', 'Semantic instruction')),
            const SizedBox(height: 6),
            _buildSemanticInstructionInput(context, locale),
            const SizedBox(height: 10),
            Row(children: [
              SizedBox(
                width: 20, height: 20,
                child: Checkbox(
                  value: _exploreIfUnknown,
                  onChanged: (v) => setState(() => _exploreIfUnknown = v ?? true),
                  materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
                  visualDensity: VisualDensity.compact,
                ),
              ),
              const SizedBox(width: 10),
              Expanded(child: Text(locale.tr('未知目标自动探索', 'Auto-explore unknown targets'),
                style: TextStyle(fontSize: 12, color: context.subtitleColor, height: 1.3))),
            ]),
            const SizedBox(height: 16),
          ],

          // Follow Person params (visible only in followPerson mode)
          if (_selectedMode == TaskMode.followPerson) ...[
            _paramLabel(locale.tr('跟随目标描述', 'Follow target description')),
            const SizedBox(height: 6),
            _paramInput(_followPersonTargetCtrl, locale.tr('例: "穿红衣服的人" 或 "person"', 'e.g. "person in red" or "person"')),
            const SizedBox(height: 16),
            _paramLabel(locale.tr('跟随距离 (${_followPersonDistance.toStringAsFixed(1)} m)', 'Follow distance (${_followPersonDistance.toStringAsFixed(1)} m)')),
            const SizedBox(height: 6),
            Slider(
              value: _followPersonDistance,
              min: 0.5,
              max: 4.0,
              divisions: 7,
              label: '${_followPersonDistance.toStringAsFixed(1)} m',
              onChanged: (v) => setState(() => _followPersonDistance = v),
            ),
            const SizedBox(height: 8),
          ],

          // Mission Name
          if (_selectedMode != TaskMode.semanticNav && _selectedMode != TaskMode.followPerson) ...[
            _paramLabel(locale.tr('任务名称', 'Mission Name')),
            const SizedBox(height: 6),
            _paramInput(_missionNameCtrl, locale.tr('例: 仓库A区', 'e.g. Warehouse Alpha')),
            const SizedBox(height: 16),
          ],

          // Robot Selection
          _paramLabel(locale.tr('机器人选择', 'Robot Selection')),
          const SizedBox(height: 6),
          _buildRobotDropdown(context),
          const SizedBox(height: 16),

          // Speed + Priority row
          Row(children: [
            Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              _paramLabel(locale.tr('速度限制', 'Speed Limit')),
              const SizedBox(height: 6),
              _buildSpeedField(context),
            ])),
            const SizedBox(width: 12),
            Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              _paramLabel(locale.tr('优先级', 'Priority')),
              const SizedBox(height: 6),
              _buildPriorityDropdown(context, locale),
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
            Expanded(child: Text(locale.tr('启用避障覆盖', 'Enable obstacle\navoidance override'),
              style: TextStyle(fontSize: 12, color: context.subtitleColor, height: 1.3))),
          ]),
        ],
      ),
    );
  }

  Widget _buildTaskModeGrid(BuildContext context, LocaleProvider locale) {
    const modes = [
      (TaskMode.navigation, Icons.navigation_rounded, Color(0xFF6366F1)),
      (TaskMode.mapping, Icons.map_rounded, Color(0xFFEA580C)),
      (TaskMode.patrol, Icons.shield_rounded, Color(0xFF0EA5E9)),
      (TaskMode.semanticNav, Icons.chat_rounded, Color(0xFF10B981)),
      (TaskMode.followPerson, Icons.directions_walk_rounded, Color(0xFFF59E0B)),
    ];
    return Wrap(
      spacing: 10,
      runSpacing: 10,
      children: [
        for (final mode in modes)
          _TaskModeButton(
            icon: mode.$2,
            label: mode.$1.localizedName(locale),
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

  Widget _buildSemanticInstructionInput(BuildContext context, LocaleProvider locale) {
    final dark = context.isDark;
    return Container(
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: const Color(0xFF10B981).withValues(alpha: 0.3)),
        color: dark
            ? const Color(0xFF10B981).withValues(alpha: 0.06)
            : const Color(0xFF10B981).withValues(alpha: 0.04),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          TextField(
            controller: _semanticInstructionCtrl,
            maxLines: 3,
            minLines: 2,
            style: TextStyle(fontSize: 13, color: context.titleColor),
            decoration: InputDecoration(
              hintText: locale.tr(
                '输入自然语言指令...\n例: "去红色灭火器旁边" 或 "go to the door"',
                'Enter natural language instruction...\ne.g. "go to the red extinguisher" or "find the door"',
              ),
              hintStyle: TextStyle(color: context.hintColor, fontSize: 12),
              contentPadding: const EdgeInsets.fromLTRB(14, 12, 14, 8),
              border: InputBorder.none,
              suffixIcon: IconButton(
                icon: Icon(Icons.mic_none_rounded,
                    size: 20, color: context.subtitleColor),
                onPressed: () {
                  // TODO: Voice input (future)
                  ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                    content: Text(locale.tr('语音输入即将支持', 'Voice input coming soon')),
                    behavior: SnackBarBehavior.floating,
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8)),
                  ));
                },
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(14, 0, 14, 10),
            child: Row(
              children: [
                Icon(Icons.auto_awesome, size: 14,
                    color: const Color(0xFF10B981)),
                const SizedBox(width: 6),
                Text(locale.tr('AI 语义理解', 'AI Semantic'),
                    style: TextStyle(
                        fontSize: 10,
                        fontWeight: FontWeight.w600,
                        color: const Color(0xFF10B981))),
                const Spacer(),
                Text('Cloud LLM',
                    style: TextStyle(
                        fontSize: 10, color: context.hintColor)),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRobotDropdown(BuildContext context) {
    final name = context.select<RobotProfileProvider, String>((p) => p.current.name);
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
    final dark = context.isDark;
    return Container(
      padding: const EdgeInsets.fromLTRB(14, 8, 14, 4),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: context.borderColor),
        color: dark
            ? Colors.white.withValues(alpha: 0.04)
            : Colors.white.withValues(alpha: 0.6),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisSize: MainAxisSize.min,
        children: [
          // Value display
          Row(children: [
            Text(
              _speedLimit.toStringAsFixed(1),
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.w700,
                color: context.titleColor,
                letterSpacing: -0.3,
              ),
            ),
            const SizedBox(width: 4),
            Text('m/s',
              style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w500,
                color: context.subtitleColor,
              ),
            ),
            const Spacer(),
            // Speed icon
            Icon(
              _speedLimit > 1.2
                  ? Icons.speed_rounded
                  : _speedLimit > 0.6
                      ? Icons.directions_walk_rounded
                      : Icons.accessibility_new_rounded,
              size: 16,
              color: _speedLimit > 1.5
                  ? AppColors.warning
                  : AppColors.primary,
            ),
          ]),
          // Slider
          SliderTheme(
            data: SliderThemeData(
              trackHeight: 4,
              thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 7),
              overlayShape: const RoundSliderOverlayShape(overlayRadius: 16),
              activeTrackColor: AppColors.primary,
              inactiveTrackColor: dark
                  ? Colors.white.withValues(alpha: 0.1)
                  : const Color(0xFFE8E5F5),
              thumbColor: Colors.white,
              overlayColor: AppColors.primary.withValues(alpha: 0.12),
            ),
            child: Slider(
              value: _speedLimit,
              min: 0.3,
              max: 2.0,
              divisions: 17,
              onChanged: (v) {
                setState(() => _speedLimit = v);
                // 同步到 RuntimeConfigGateway
                context.read<RuntimeConfigGateway>().updateParam('max_speed', v);
              },
            ),
          ),
          // Range labels
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 2),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text('0.3', style: TextStyle(fontSize: 9, color: context.hintColor)),
                Text('2.0', style: TextStyle(fontSize: 9, color: context.hintColor)),
              ],
            ),
          ),
          const SizedBox(height: 2),
        ],
      ),
    );
  }

  Widget _buildPriorityDropdown(BuildContext context, LocaleProvider locale) {
    final labels = [
      locale.tr('普通', 'Normal'),
      locale.tr('高', 'High'),
      locale.tr('紧急', 'Critical'),
    ];
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
  Widget _buildMapToolbar(BuildContext context, bool online, LocaleProvider locale) {
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
          _toolbarIconBtn(Icons.navigation_rounded, tooltip: locale.tr('导航视图', 'Navigation view'),
            isActive: _show3DModel, onTap: () => setState(() => _show3DModel = true)),
          const SizedBox(width: 4),
          _toolbarIconBtn(Icons.location_on_rounded, tooltip: locale.tr('2D地图', '2D Map'),
            isActive: !_show3DModel, onTap: () => setState(() => _show3DModel = false)),
          const SizedBox(width: 4),
          _toolbarIconBtn(Icons.route_rounded, tooltip: locale.tr('轨迹', 'Trajectory'),
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
              Text(locale.tr('实时数据', 'Live Feed Active'), style: TextStyle(
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
              Text(online
                  ? locale.tr('系统就绪', 'System Ready')
                  : locale.tr('离线', 'Offline'), style: TextStyle(
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
              onTapDown: (_isMeasuring || (_isSettingGoal && _modeUsesGoalPoint))
                  ? _handleMapTap
                  : null,
              onLongPressStart: _handleMapLongPress,
              child: GridPaper(
                color: Colors.black12, interval: 100, divisions: 1, subdivisions: 5,
                child: InteractiveViewer(
                  transformationController: _transformController,
                  boundaryMargin: const EdgeInsets.all(5000),
                  minScale: 0.1, maxScale: 100.0,
                  panEnabled: !_rotationLocked,
                  scaleEnabled: !_rotationLocked,
                  child: SizedBox(
                    width: 10000, height: 10000,
                    child: RepaintBoundary(child: CustomPaint(
                      painter: TrajectoryPainter(
                        path: _path, currentPose: _currentPose,
                        globalBuckets: _showGlobalMap
                            ? _globalMapBuckets
                            : const [[], [], [], [], []],
                        localBuckets: _localCloudBuckets,
                        navGoalPoint: _navGoalPoint,
                        dataVersion: _mapDataVersion,
                        globalPathPoints: _showGlobalPath ? _globalPathPoints : const [],
                        occupancyGrid: _showOccupancyGrid ? _occupancyGrid : null,
                        frontierMarkers: _showFrontiers ? _frontierMarkers : const [],
                        longPressMarkers: _longPressMarkers,
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

        // Active waypoints overlay (while task runs)
        if (_activeWaypoints.isNotEmpty && !_show3DModel)
          Positioned.fill(child: IgnorePointer(child: CustomPaint(
            painter: _ActiveWaypointPainter(
              waypoints: _activeWaypoints,
              currentIndex: _activeWaypointCurrentIndex,
              transform: _transformController.value,
            ),
          ))),

        // Measurement overlay
        if (_measureStart != null && !_show3DModel)
          Positioned.fill(child: IgnorePointer(child: CustomPaint(
            painter: _MeasurePainter(
              start: _measureStart!,
              end: _measureEnd,
              transform: _transformController.value,
            ),
          ))),

        // Crosshair overlay (2D mode only)
        if (!_show3DModel)
          Positioned.fill(child: IgnorePointer(child: CustomPaint(
            painter: _CrosshairPainter(),
          ))),

        // Center coordinate display (2D mode only)
        if (!_show3DModel)
          Positioned(
            bottom: 60, left: 0, right: 0,
            child: Center(child: AnimatedBuilder(
              animation: _transformController,
              builder: (_, __) {
                final matrix = _transformController.value.clone();
                final inv = Matrix4.tryInvert(matrix);
                if (inv == null) return const SizedBox.shrink();
                final size = MediaQuery.of(context).size;
                final cx = size.width / 2;
                final cy = size.height / 2;
                final wx = inv[0] * cx + inv[4] * cy + inv[12];
                final wy = -(inv[1] * cx + inv[5] * cy + inv[13]);
                return Container(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                  decoration: BoxDecoration(
                    color: Colors.black.withValues(alpha: 0.6),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: Text(
                    'x: ${wx.toStringAsFixed(2)}, y: ${wy.toStringAsFixed(2)}',
                    style: const TextStyle(
                      color: Colors.white, fontSize: 11, fontFamily: 'monospace',
                    ),
                  ),
                );
              },
            )),
          ),

        // Geofence alert banner
        if (_geofenceState == 'WARNING' || _geofenceState == 'VIOLATION')
          _buildGeofenceAlertBanner(),

        // Low battery warning banner
        _buildLowBatteryBanner(context),

        // Map legend + scale ruler (bottom-left, 2D mode only)
        if (!_show3DModel)
          Positioned(
            left: 16, bottom: 16,
            child: IgnorePointer(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  _buildMapLegend(),
                  const SizedBox(height: 6),
                  _buildScaleRuler(),
                ],
              ),
            ),
          ),

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

  // ─── Map legend ───────────────────────────────────────────
  Widget _buildMapLegend() {
    return Container(
      width: 120,
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.70),
        borderRadius: BorderRadius.circular(10),
      ),
      child: DefaultTextStyle(
        style: const TextStyle(color: Colors.white, fontSize: 12, height: 1.6),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisSize: MainAxisSize.min,
          children: [
            // Z-height gradient indicator (blue→cyan→green→yellow→red)
            Row(mainAxisSize: MainAxisSize.min, children: [
              for (final c in [Colors.blue, Colors.cyan, Colors.green, Colors.yellow, Colors.red])
                Container(
                  width: 8, height: 8,
                  margin: const EdgeInsets.only(right: 1),
                  decoration: BoxDecoration(color: c, shape: BoxShape.circle),
                ),
              const SizedBox(width: 4),
              const Text('Z高度'),
            ]),
            _legendRow(const Color(0xFF00AA44), '自由'),
            _legendRow(const Color(0xFF888888), '未知'),
            _legendRow(const Color(0xFFCC2222), '障碍'),
            _legendRow(const Color(0xFF7C3AED), '路径'),
            _legendRow(const Color(0xFFFF6B00), '机器人'),
          ],
        ),
      ),
    );
  }

  Row _legendRow(Color color, String label) => Row(
    mainAxisSize: MainAxisSize.min,
    children: [
      Container(
        width: 10, height: 10,
        decoration: BoxDecoration(color: color, shape: BoxShape.circle),
      ),
      const SizedBox(width: 6),
      Text(label),
    ],
  );

  // ─── Scale ruler ─────────────────────────────────────────
  Widget _buildScaleRuler() {
    return AnimatedBuilder(
      animation: _transformController,
      builder: (_, __) {
        final scale = _transformController.value.getMaxScaleOnAxis();
        final rawM = 80.0 / scale.clamp(0.1, 10000.0);
        final niceM = _niceDistance(rawM);
        final rulerPx = (niceM * scale).clamp(10.0, 200.0);
        final label = niceM >= 1.0 ? '${niceM.round()}m' : '${(niceM * 100).round()}cm';
        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 5),
          decoration: BoxDecoration(
            color: Colors.black.withValues(alpha: 0.60),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              SizedBox(
                width: rulerPx, height: 10,
                child: Stack(
                  children: [
                    Positioned(left: 0, right: 0, top: 4, child: Container(height: 2, color: Colors.white70)),
                    Positioned(left: 0, top: 0, child: Container(width: 2, height: 10, color: Colors.white70)),
                    Positioned(right: 0, top: 0, child: Container(width: 2, height: 10, color: Colors.white70)),
                  ],
                ),
              ),
              const SizedBox(width: 6),
              Text(label, style: const TextStyle(color: Colors.white70, fontSize: 9)),
            ],
          ),
        );
      },
    );
  }

  double _niceDistance(double meters) {
    const nice = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0];
    for (final v in nice) {
      if (v >= meters) return v;
    }
    return 500.0;
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
        _mapCtrlBtn(Icons.layers_rounded,
          color: _showGlobalMap ? AppColors.primary : null,
          onTap: () {
            HapticFeedback.selectionClick();
            setState(() => _showGlobalMap = !_showGlobalMap);
          },
          tooltip: 'Point cloud map',
        ),
        const SizedBox(height: 4),
        _mapCtrlBtn(Icons.route_rounded,
          color: _showGlobalPath ? const Color(0xFF7C3AED) : null,
          onTap: () {
            HapticFeedback.selectionClick();
            setState(() => _showGlobalPath = !_showGlobalPath);
          },
          tooltip: 'Global path',
        ),
        const SizedBox(height: 4),
        _mapCtrlBtn(Icons.grid_on_rounded,
          color: _showOccupancyGrid ? AppColors.warning : null,
          onTap: () {
            HapticFeedback.selectionClick();
            setState(() => _showOccupancyGrid = !_showOccupancyGrid);
          },
          tooltip: 'Occupancy grid',
        ),
        const SizedBox(height: 4),
        _mapCtrlBtn(Icons.explore_rounded,
          color: _showFrontiers && _frontierMarkers.isNotEmpty
              ? const Color(0xFFFF6D00) : null,
          onTap: () {
            HapticFeedback.selectionClick();
            setState(() => _showFrontiers = !_showFrontiers);
          },
          tooltip: 'Frontier markers',
        ),
        const SizedBox(height: 4),
        // Rotation / pan lock
        _mapCtrlBtn(
          _rotationLocked ? Icons.screen_lock_rotation : Icons.screen_rotation,
          color: _rotationLocked ? AppColors.warning : null,
          onTap: () {
            HapticFeedback.selectionClick();
            setState(() => _rotationLocked = !_rotationLocked);
          },
          tooltip: _rotationLocked ? 'Unlock view' : 'Lock view',
        ),
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

  Widget _mapCtrlBtn(IconData icon, {
    required VoidCallback onTap,
    Color? color,
    String? tooltip,
  }) {
    final btn = GestureDetector(
      onTap: onTap,
      child: Container(
        width: 36, height: 36,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(8),
          color: color != null ? color.withValues(alpha: 0.15) : null,
        ),
        child: Icon(icon, size: 18, color: color ?? context.subtitleColor),
      ),
    );
    if (tooltip != null) return Tooltip(message: tooltip, child: btn);
    return btn;
  }

  Widget _buildFabColumn(BuildContext context) {
    final locale = context.read<LocaleProvider>();
    final tg = context.watch<TaskGateway>();
    final cg = context.watch<ControlGateway>();
    final isRunning = tg.isRunning;
    final isMapping = isRunning && tg.activeTaskType == TaskType.TASK_TYPE_MAPPING;
    final isEstop = cg.currentMode == RobotMode.ROBOT_MODE_ESTOP;

    return Column(mainAxisSize: MainAxisSize.min, children: [
      // Goal setting (hidden when task running)
      if (!isRunning)
        _MapFab(
          icon: _isSettingGoal ? Icons.close : Icons.add_location_alt_outlined,
          tooltip: locale.tr('设置${_goalPointLabel(locale)}', 'Set ${_goalPointLabel(locale)}'),
          onPressed: () {
            HapticFeedback.selectionClick();
            if (!_modeUsesGoalPoint) {
              ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                content: Text(locale.tr('建图模式无需设置目标点', 'No goal needed for mapping mode')),
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
      if (!isRunning) const SizedBox(height: 6),

      // Start / Cancel task button
      isRunning
          ? _MapFab(
              icon: isMapping ? Icons.save_rounded : Icons.stop_rounded,
              tooltip: isMapping
                  ? locale.tr('停止建图', 'Stop mapping')
                  : locale.tr('取消任务', 'Cancel task'),
              color: isMapping ? AppColors.success : AppColors.error,
              onPressed: () async {
                final confirm = await showDialog<bool>(
                  context: context,
                  builder: (ctx) => AlertDialog(
                    title: Text(isMapping
                        ? locale.tr('停止建图', 'Stop mapping')
                        : locale.tr('取消任务', 'Cancel task')),
                    content: Text(isMapping
                        ? locale.tr('停止建图后将自动保存地图。', 'Map will be saved automatically after stopping.')
                        : locale.tr('确定要取消当前任务吗？', 'Are you sure you want to cancel the current task?')),
                    actions: [
                      TextButton(onPressed: () => Navigator.of(ctx).pop(false),
                        child: Text(locale.tr('返回', 'Back'))),
                      FilledButton(
                        style: FilledButton.styleFrom(
                          backgroundColor: isMapping ? AppColors.success : AppColors.error),
                        onPressed: () => Navigator.of(ctx).pop(true),
                        child: Text(isMapping
                            ? locale.tr('停止并保存', 'Stop & save')
                            : locale.tr('取消任务', 'Cancel task')),
                      ),
                    ],
                  ),
                );
                if (confirm == true) {
                  await tg.cancelTask();
                  _stopWaypointPolling();
                }
              },
            )
          : _MapFab(
              icon: Icons.navigation_outlined,
              tooltip: isEstop
                  ? locale.tr('急停中，无法启动', 'E-stop active, cannot start')
                  : locale.tr('启动任务', 'Start task'),
              color: isEstop ? Colors.grey : null,
              onPressed: isEstop ? () {
                ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                  content: Text(locale.tr('急停状态，请先解除急停', 'E-stop active, please release first')),
                  backgroundColor: AppColors.error,
                  behavior: SnackBarBehavior.floating,
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                ));
              } : _startSelectedTask,
            ),

      // Emergency stop (visible when task is running)
      if (isRunning) ...[
        const SizedBox(height: 6),
        _MapFab(
          icon: Icons.emergency_rounded,
          tooltip: locale.tr('急停', 'E-Stop'),
          color: Colors.red,
          onPressed: () async {
            HapticFeedback.heavyImpact();
            await cg.emergencyStop();
          },
        ),
      ],
      const SizedBox(height: 6),
      // Measurement tool
      _MapFab(
        icon: Icons.straighten,
        tooltip: locale.tr('测距工具', 'Measure distance'),
        color: _isMeasuring ? AppColors.primary : null,
        onPressed: () {
          HapticFeedback.selectionClick();
          setState(() {
            _isMeasuring = !_isMeasuring;
            if (!_isMeasuring) {
              _measureStart = null;
              _measureEnd = null;
            }
          });
        },
      ),
      const SizedBox(height: 6),
      _MapFab(icon: Icons.folder_outlined, tooltip: locale.tr('地图管理', 'Map manager'),
        onPressed: () => Navigator.of(context).pushNamed('/map-manager')),
      const SizedBox(height: 6),
      _MapFab(icon: Icons.save_outlined, tooltip: locale.tr('保存地图', 'Save map'),
        onPressed: _saveMap),
    ]);
  }

  // ═══════════════════════════════════════════════════════════
  //  TASK STATUS BAR — overlay when task is running
  // ═══════════════════════════════════════════════════════════
  Widget _buildTaskStatusBar(BuildContext context) {
    final locale = context.read<LocaleProvider>();
    final tg = context.watch<TaskGateway>();
    if (!tg.isRunning) return const SizedBox.shrink();

    final dark = context.isDark;
    final taskLabel = switch (tg.activeTaskType) {
      TaskType.TASK_TYPE_NAVIGATION => locale.tr('导航', 'NAVIGATION'),
      TaskType.TASK_TYPE_MAPPING => locale.tr('建图', 'MAPPING'),
      TaskType.TASK_TYPE_INSPECTION => locale.tr('巡检', 'PATROL'),
      TaskType.TASK_TYPE_FOLLOW_PATH => locale.tr('循迹', 'FOLLOW'),
      TaskType.TASK_TYPE_SEMANTIC_NAV => locale.tr('语义导航', 'SEMANTIC NAV'),
      _ => locale.tr('任务', 'TASK'),
    };
    final progress = tg.progress;
    final sourceLabel = switch (_waypointSource) {
      WaypointSource.WAYPOINT_SOURCE_APP => 'App',
      WaypointSource.WAYPOINT_SOURCE_PLANNER => 'Planner',
      _ => null,
    };
    final waypointInfo = _activeWaypoints.isNotEmpty
        ? '${sourceLabel != null ? '[$sourceLabel] ' : ''}Waypoint ${_activeWaypointCurrentIndex + 1}/${_activeWaypoints.length}'
        : null;

    return Positioned(
      left: 16, right: 16, top: MediaQuery.of(context).padding.top + 8,
      child: Container(
        padding: const EdgeInsets.fromLTRB(16, 10, 8, 10),
        decoration: BoxDecoration(
          color: dark
              ? const Color(0xE0222222)
              : const Color(0xE0FFFFFF),
          borderRadius: BorderRadius.circular(14),
          border: Border.all(
            color: tg.isPaused
                ? AppColors.warning.withValues(alpha: 0.5)
                : AppColors.primary.withValues(alpha: 0.3),
          ),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withValues(alpha: 0.12),
              blurRadius: 12, offset: const Offset(0, 4),
            ),
          ],
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // Top row: label + waypoint info + buttons
            Row(
              children: [
                // Task type badge
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                  decoration: BoxDecoration(
                    color: tg.isPaused
                        ? AppColors.warning.withValues(alpha: 0.15)
                        : AppColors.primary.withValues(alpha: 0.15),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: Text(
                    tg.isPaused ? '$taskLabel (${locale.tr('已暂停', 'PAUSED')})' : taskLabel,
                    style: TextStyle(
                      fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 0.5,
                      color: tg.isPaused ? AppColors.warning : AppColors.primary,
                    ),
                  ),
                ),
                const SizedBox(width: 10),
                // Waypoint progress
                if (waypointInfo != null)
                  Text(waypointInfo, style: TextStyle(
                    fontSize: 12, fontWeight: FontWeight.w500, color: context.titleColor,
                  )),
                const Spacer(),
                // Pause / Resume
                _statusBarBtn(
                  icon: tg.isPaused ? Icons.play_arrow_rounded : Icons.pause_rounded,
                  tooltip: tg.isPaused ? locale.tr('恢复', 'Resume') : locale.tr('暂停', 'Pause'),
                  onTap: () => tg.isPaused ? tg.resumeTask() : tg.pauseTask(),
                ),
                const SizedBox(width: 4),
                // Cancel / Stop Mapping
                Builder(builder: (ctx) {
                  final isMappingTask = tg.activeTaskType == TaskType.TASK_TYPE_MAPPING;
                  return _statusBarBtn(
                    icon: isMappingTask ? Icons.save_rounded : Icons.close_rounded,
                    tooltip: isMappingTask
                        ? locale.tr('停止建图', 'Stop mapping')
                        : locale.tr('取消任务', 'Cancel task'),
                    color: isMappingTask ? AppColors.success : AppColors.error,
                    onTap: () async {
                      final confirm = await showDialog<bool>(
                        context: context,
                        builder: (ctx) => AlertDialog(
                          title: Text(isMappingTask
                              ? locale.tr('停止建图', 'Stop mapping')
                              : locale.tr('取消任务', 'Cancel task')),
                          content: Text(isMappingTask
                              ? locale.tr('停止建图后将自动保存地图。', 'Map will be saved automatically after stopping.')
                              : locale.tr('确定要取消当前任务吗？', 'Are you sure you want to cancel the current task?')),
                          actions: [
                            TextButton(onPressed: () => Navigator.of(ctx).pop(false),
                              child: Text(locale.tr('返回', 'Back'))),
                            FilledButton(
                              style: FilledButton.styleFrom(
                                backgroundColor: isMappingTask ? AppColors.success : AppColors.error),
                              onPressed: () => Navigator.of(ctx).pop(true),
                              child: Text(isMappingTask
                                  ? locale.tr('停止并保存', 'Stop & save')
                                  : locale.tr('取消任务', 'Cancel task')),
                            ),
                          ],
                        ),
                      );
                      if (confirm == true) {
                        await tg.cancelTask();
                        _stopWaypointPolling();
                      }
                    },
                  );
                }),
                const SizedBox(width: 4),
                // Clear waypoints
                _statusBarBtn(
                  icon: Icons.delete_sweep_rounded,
                  tooltip: locale.tr('清除航点', 'Clear waypoints'),
                  color: AppColors.warning,
                  onTap: () async {
                    final confirm = await showDialog<bool>(
                      context: context,
                      builder: (ctx) => AlertDialog(
                        title: Text(locale.tr('清除航点', 'Clear waypoints')),
                        content: Text(locale.tr('清除所有航点并立即停车？', 'Clear all waypoints and stop immediately?')),
                        actions: [
                          TextButton(onPressed: () => Navigator.of(ctx).pop(false),
                            child: Text(locale.tr('返回', 'Back'))),
                          FilledButton(
                            style: FilledButton.styleFrom(backgroundColor: AppColors.warning),
                            onPressed: () => Navigator.of(ctx).pop(true),
                            child: Text(locale.tr('清除并停车', 'Clear & stop')),
                          ),
                        ],
                      ),
                    );
                    if (confirm == true) {
                      await tg.clearWaypoints();
                    }
                  },
                ),
              ],
            ),
            const SizedBox(height: 8),
            // Progress bar
            ClipRRect(
              borderRadius: BorderRadius.circular(4),
              child: LinearProgressIndicator(
                value: progress.clamp(0.0, 1.0),
                minHeight: 4,
                backgroundColor: dark
                    ? Colors.white.withValues(alpha: 0.08)
                    : Colors.black.withValues(alpha: 0.06),
                valueColor: AlwaysStoppedAnimation(
                  tg.isPaused ? AppColors.warning : AppColors.primary,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _statusBarBtn({
    required IconData icon, required String tooltip,
    Color? color, required VoidCallback onTap,
  }) {
    return Tooltip(
      message: tooltip,
      child: GestureDetector(
        onTap: onTap,
        child: Container(
          width: 32, height: 32,
          decoration: BoxDecoration(
            color: (color ?? context.subtitleColor).withValues(alpha: 0.1),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Icon(icon, size: 18, color: color ?? context.subtitleColor),
        ),
      ),
    );
  }

  Widget _buildGoalSettingBanner(BuildContext context) {
    final locale = context.read<LocaleProvider>();
    return Positioned(
      top: MediaQuery.of(context).padding.top + 60,
      left: 0, right: 0,
      child: Center(child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: AppColors.warning.withValues(alpha: 0.9),
          borderRadius: BorderRadius.circular(20),
        ),
        child: Text(locale.tr('点击地图设置${_goalPointLabel(locale)}', 'Tap map to set ${_goalPointLabel(locale)}'), style: const TextStyle(
          color: Colors.white, fontSize: 13, fontWeight: FontWeight.w600,
        )),
      )),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  GEOFENCE ALERT — top banner when WARNING or VIOLATION
  // ═══════════════════════════════════════════════════════════
  Widget _buildGeofenceAlertBanner() {
    final locale = context.read<LocaleProvider>();
    final isViolation = _geofenceState == 'VIOLATION';
    final color = isViolation ? AppColors.error : AppColors.warning;
    final label = isViolation
        ? locale.tr('围栏越界', 'Geofence violation')
        : locale.tr('围栏警告', 'Geofence warning');
    final marginText = _geofenceMargin > 0
        ? locale.tr('  (余量: ${_geofenceMargin.toStringAsFixed(1)}m)',
                     '  (margin: ${_geofenceMargin.toStringAsFixed(1)}m)')
        : '';

    return Positioned(
      top: 8, left: 16, right: 16,
      child: SafeArea(
        child: Material(
          elevation: 4,
          borderRadius: BorderRadius.circular(10),
          color: color.withValues(alpha: 0.92),
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
            child: Row(children: [
              Icon(
                isViolation ? Icons.error_outline : Icons.warning_amber_rounded,
                color: Colors.white, size: 20,
              ),
              const SizedBox(width: 8),
              Expanded(child: Text(
                '$label$marginText',
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 13,
                  fontWeight: FontWeight.w600,
                ),
              )),
            ]),
          ),
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  LOW BATTERY WARNING BANNER
  // ═══════════════════════════════════════════════════════════
  Widget _buildLowBatteryBanner(BuildContext context) {
    final provider = context.read<RobotConnectionProvider>();
    return StreamBuilder<SlowState>(
      stream: provider.slowStateStream,
      initialData: provider.latestSlowState,
      builder: (context, snapshot) {
        final battery = snapshot.data?.resources.batteryPercent;
        if (battery == null || battery >= 15) return const SizedBox.shrink();
        final locale = context.read<LocaleProvider>();
        return Positioned(
          top: 44, left: 16, right: 16,
          child: SafeArea(
            child: Material(
              elevation: 4,
              borderRadius: BorderRadius.circular(10),
              color: AppColors.warning.withValues(alpha: 0.92),
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
                child: Row(children: [
                  const Icon(Icons.battery_alert_rounded,
                    color: Colors.white, size: 20),
                  const SizedBox(width: 8),
                  Expanded(child: Text(
                    locale.tr(
                      '\u26A0 电量不足 (${battery.toStringAsFixed(0)}%) \u2014 请及时充电或返回',
                      '\u26A0 Low battery (${battery.toStringAsFixed(0)}%) \u2014 charge or return soon',
                    ),
                    style: const TextStyle(
                      color: Colors.white,
                      fontSize: 13,
                      fontWeight: FontWeight.w600,
                    ),
                  )),
                ]),
              ),
            ),
          ),
        );
      },
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
            Text(context.read<LocaleProvider>().tr('航点序列', 'WAYPOINT SEQUENCE'), style: TextStyle(
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
  final Color? color;

  const _MapFab({required this.icon, required this.tooltip, required this.onPressed, this.color});

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    final c = color;
    return Tooltip(
      message: tooltip,
      child: GestureDetector(
        onTap: onPressed,
        child: Container(
          width: 40, height: 40,
          decoration: BoxDecoration(
            color: c != null
                ? c.withValues(alpha: dark ? 0.2 : 0.1)
                : (dark ? AppColors.darkCard : Colors.white),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(
              color: c?.withValues(alpha: 0.5) ?? context.borderColor,
            ),
          ),
          child: Icon(icon, size: 18, color: c ?? context.subtitleColor),
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

class _ActiveWaypointPainter extends CustomPainter {
  final List<ActiveWaypoint> waypoints;
  final int currentIndex;
  final Matrix4 transform;

  _ActiveWaypointPainter({
    required this.waypoints, required this.currentIndex, required this.transform,
  });

  Offset _toScreen(double x, double y) {
    final m = transform;
    return Offset(
      m[0] * x + m[4] * (-y) + m[12],
      m[1] * x + m[5] * (-y) + m[13],
    );
  }

  // 缓存所有 Paint 对象，避免每帧 ~15-60 次堆分配
  static final Paint _linePaint = Paint()
    ..color = const Color(0xFF007AFF).withValues(alpha: 0.4)
    ..strokeWidth = 2
    ..style = PaintingStyle.stroke;
  // Current waypoint
  static final Paint _curFill = Paint()
    ..color = const Color(0xFF34C759).withValues(alpha: 0.25);
  static final Paint _curStroke = Paint()
    ..color = const Color(0xFF34C759)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 2.5;
  static final Paint _curDot = Paint()
    ..color = const Color(0xFF34C759);
  // Completed waypoint
  static final Paint _doneFill = Paint()
    ..color = const Color(0xFF999999).withValues(alpha: 0.3);
  static final Paint _doneDot = Paint()
    ..color = const Color(0xFF999999);
  // Future waypoint
  static final Paint _futureFill = Paint()
    ..color = const Color(0xFF007AFF).withValues(alpha: 0.15);
  static final Paint _futureStroke = Paint()
    ..color = const Color(0xFF007AFF)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 1.5;
  static final Paint _futureDot = Paint()
    ..color = const Color(0xFF007AFF);

  @override
  void paint(Canvas canvas, Size size) {
    if (waypoints.isEmpty) return;

    // Convert waypoint positions to screen coordinates
    final points = <Offset>[];
    for (final wp in waypoints) {
      points.add(_toScreen(wp.position.x, wp.position.y));
    }

    // Draw connecting lines
    if (points.length >= 2) {
      final path = Path()..moveTo(points.first.dx, points.first.dy);
      for (var i = 1; i < points.length; i++) {
        path.lineTo(points[i].dx, points[i].dy);
      }
      canvas.drawPath(path, _linePaint);
    }

    // Draw waypoint markers
    for (var i = 0; i < points.length; i++) {
      final center = points[i];
      final isCurrent = i == currentIndex;

      if (isCurrent) {
        canvas.drawCircle(center, 14, _curFill);
        canvas.drawCircle(center, 14, _curStroke);
        canvas.drawCircle(center, 5, _curDot);
      } else if (i < currentIndex) {
        canvas.drawCircle(center, 8, _doneFill);
        canvas.drawCircle(center, 3, _doneDot);
      } else {
        canvas.drawCircle(center, 10, _futureFill);
        canvas.drawCircle(center, 10, _futureStroke);
        canvas.drawCircle(center, 4, _futureDot);
      }

      // Draw index number
      final tp = TextPainter(
        text: TextSpan(text: '${i + 1}', style: TextStyle(
          color: isCurrent ? const Color(0xFF34C759) : const Color(0xFF007AFF),
          fontSize: 10, fontWeight: FontWeight.w700,
        )),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, center + Offset(-tp.width / 2, isCurrent ? 18 : 14));
    }
  }

  @override
  bool shouldRepaint(covariant _ActiveWaypointPainter old) =>
      waypoints.length != old.waypoints.length ||
      currentIndex != old.currentIndex ||
      transform != old.transform;
}

class TrajectoryPainter extends CustomPainter {
  final List<Offset> path;
  final Pose? currentPose;
  final List<List<Offset>> globalBuckets;  // 5 Z-buckets (blue→cyan→green→yellow→red)
  final List<List<Offset>> localBuckets;   // 5 Z-buckets (blue→cyan→green→yellow→red)
  final Offset? navGoalPoint;
  final int dataVersion;
  final List<Offset> globalPathPoints;
  final _OccupancyGrid? occupancyGrid;
  final List<_FrontierMarker> frontierMarkers;
  final List<Offset> longPressMarkers;
  final int _pathLength;
  final double? _poseX;
  final double? _poseY;

  // ── Z-height 5-bucket paints for point clouds (blue→cyan→green→yellow→red) ──
  static final _viridisGlobal = [
    Paint()..color = Colors.blue..strokeWidth = 0.12..strokeCap = StrokeCap.round,
    Paint()..color = Colors.cyan..strokeWidth = 0.12..strokeCap = StrokeCap.round,
    Paint()..color = Colors.green..strokeWidth = 0.12..strokeCap = StrokeCap.round,
    Paint()..color = Colors.yellow..strokeWidth = 0.12..strokeCap = StrokeCap.round,
    Paint()..color = Colors.red..strokeWidth = 0.12..strokeCap = StrokeCap.round,
  ];
  static final _viridisLocal = [
    Paint()..color = Colors.blue.withValues(alpha: 0.53)..strokeWidth = 0.06..strokeCap = StrokeCap.round,
    Paint()..color = Colors.cyan.withValues(alpha: 0.53)..strokeWidth = 0.06..strokeCap = StrokeCap.round,
    Paint()..color = Colors.green.withValues(alpha: 0.53)..strokeWidth = 0.06..strokeCap = StrokeCap.round,
    Paint()..color = Colors.yellow.withValues(alpha: 0.53)..strokeWidth = 0.06..strokeCap = StrokeCap.round,
    Paint()..color = Colors.red.withValues(alpha: 0.53)..strokeWidth = 0.06..strokeCap = StrokeCap.round,
  ];
  static final _trajectoryPaint = Paint()
    ..color = const Color(0xFF007AFF).withValues(alpha: 0.6)
    ..strokeWidth = 0.1..style = PaintingStyle.stroke
    ..strokeCap = StrokeCap.round..strokeJoin = StrokeJoin.round;
  static final _robotPaint = Paint()
    ..color = const Color(0xFFFF3B30)..style = PaintingStyle.fill;
  static final _headingPaint = Paint()
    ..color = Colors.black.withValues(alpha: 0.5)..strokeWidth = 0.05;
  static final _goalStrokePaint = Paint()
    ..color = const Color(0xFF34C759)..style = PaintingStyle.stroke..strokeWidth = 0.08;
  static final _goalFillPaint = Paint()
    ..color = const Color(0xFF34C759)..style = PaintingStyle.fill;
  static final _axisPaint = Paint()
    ..strokeWidth = 0.05..color = Colors.grey.withValues(alpha: 0.5);
  // 全局路径
  static final _globalPathPaint = Paint()
    ..color = const Color(0xFF7C3AED)
    ..strokeWidth = 0.18
    ..style = PaintingStyle.stroke
    ..strokeCap = StrokeCap.round
    ..strokeJoin = StrokeJoin.round;
  static final _globalPathDotPaint = Paint()
    ..color = const Color(0xFF7C3AED)
    ..style = PaintingStyle.fill;
  static final _globalPathEndStroke = Paint()
    ..color = const Color(0xFF7C3AED)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 0.06;
  // 占用栅格
  final _gridPaint = Paint()..style = PaintingStyle.fill;
  // 长按航点标记（橙色）
  static final _longPressStrokePaint = Paint()
    ..color = const Color(0xFFFF6B00)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 0.08;
  static final _longPressFillPaint = Paint()
    ..color = const Color(0x44FF6B00)
    ..style = PaintingStyle.fill;
  static final _longPressDotPaint = Paint()
    ..color = const Color(0xFFFF6B00)
    ..style = PaintingStyle.fill;

  TrajectoryPainter({
    required this.path, this.currentPose,
    this.globalBuckets = const [[], [], [], [], []],
    this.localBuckets = const [[], [], [], [], []],
    this.navGoalPoint, this.dataVersion = 0,
    this.globalPathPoints = const [], this.occupancyGrid,
    this.frontierMarkers = const [],
    this.longPressMarkers = const [],
  })  : _pathLength = path.length,
        _poseX = currentPose?.position.x,
        _poseY = currentPose?.position.y;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.translate(size.width / 2, size.height / 2);
    canvas.scale(1.0, -1.0);

    // ── 占用栅格（底层，最先绘制）── 三色: 空闲绿/未知灰/障碍红
    final g = occupancyGrid;
    if (g != null) {
      final cellSz = g.resolution * 1.05;
      for (var row = 0; row < g.height; row++) {
        for (var col = 0; col < g.width; col++) {
          final val = g.data[row * g.width + col];
          if (val == 0) continue; // 完全空闲 — 跳过提升性能
          final wx = g.originX + (col + 0.5) * g.resolution;
          final wy = g.originY + (row + 0.5) * g.resolution;
          if (val < 50) {
            _gridPaint.color = const Color(0x3300AA44); // 自由空间 — 绿
          } else if (val < 128) {
            _gridPaint.color = const Color(0x55888888); // 未知 — 灰
          } else {
            _gridPaint.color = const Color(0xAACC2222); // 障碍 — 深红
          }
          canvas.drawRect(
            Rect.fromCenter(center: Offset(wx, wy), width: cellSz, height: cellSz),
            _gridPaint,
          );
        }
      }
    }

    // ── 地图点云 — viridis Z 分段着色 ──
    for (var b = 0; b < 5; b++) {
      if (globalBuckets[b].isNotEmpty) {
        canvas.drawPoints(ui.PointMode.points, globalBuckets[b], _viridisGlobal[b]);
      }
      if (localBuckets[b].isNotEmpty) {
        canvas.drawPoints(ui.PointMode.points, localBuckets[b], _viridisLocal[b]);
      }
    }

    // ── 全局路径（紫色线） ──
    if (globalPathPoints.length >= 2) {
      final gp = Path()
        ..moveTo(globalPathPoints.first.dx, globalPathPoints.first.dy);
      for (var i = 1; i < globalPathPoints.length; i++) {
        gp.lineTo(globalPathPoints[i].dx, globalPathPoints[i].dy);
      }
      canvas.drawPath(gp, _globalPathPaint);
      // 起始点
      canvas.drawCircle(globalPathPoints.first, 0.2, _globalPathDotPaint);
      // 终点圆环
      final end = globalPathPoints.last;
      canvas.drawCircle(end, 0.4, Paint()
        ..color = const Color(0xFF7C3AED).withValues(alpha: 0.2)
        ..style = PaintingStyle.fill);
      canvas.drawCircle(end, 0.4, _globalPathEndStroke);
      canvas.drawCircle(end, 0.15, _globalPathDotPaint);
    }

    // ── 前沿探索标记（橙色渐变圆圈） ──
    for (final fm in frontierMarkers) {
      final r = 0.4 + fm.score * 0.8; // radius 0.4..1.2 m
      // inner fill: score-based orange
      final alpha = (0.12 + fm.score * 0.25).clamp(0.0, 0.37);
      canvas.drawCircle(
        Offset(fm.x, fm.y), r,
        Paint()
          ..color = Color.fromRGBO(255, 109, 0, alpha)
          ..style = PaintingStyle.fill,
      );
      // stroke
      canvas.drawCircle(
        Offset(fm.x, fm.y), r,
        Paint()
          ..color = Color.fromRGBO(255, 109, 0, (0.5 + fm.score * 0.45).clamp(0.0, 1.0))
          ..style = PaintingStyle.stroke
          ..strokeWidth = 0.05,
      );
    }

    // ── 历史轨迹（蓝色线） ──
    if (path.isNotEmpty) {
      final p = Path()..moveTo(path.first.dx, path.first.dy);
      for (var i = 1; i < path.length; i++) { p.lineTo(path[i].dx, path[i].dy); }
      canvas.drawPath(p, _trajectoryPaint);
    }

    // ── 机器人位姿 ──
    if (currentPose != null) {
      final pos = currentPose!.position;
      final q = currentPose!.orientation;
      final yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      canvas.save();
      canvas.translate(pos.x, pos.y);
      canvas.rotate(yaw);
      canvas.drawPath(
        Path()..moveTo(0.25, 0)..lineTo(-0.25, 0.15)..lineTo(-0.25, -0.15)..close(),
        _robotPaint,
      );
      canvas.drawLine(Offset.zero, const Offset(1.0, 0), _headingPaint);
      canvas.restore();
    }

    // ── 导航目标点 ──
    if (navGoalPoint != null) {
      canvas.save();
      canvas.translate(navGoalPoint!.dx, navGoalPoint!.dy);
      canvas.drawCircle(Offset.zero, 0.5, _goalStrokePaint);
      canvas.drawCircle(Offset.zero, 0.15, _goalFillPaint);
      canvas.drawLine(const Offset(-0.3, 0), const Offset(0.3, 0), _goalStrokePaint);
      canvas.drawLine(const Offset(0, -0.3), const Offset(0, 0.3), _goalStrokePaint);
      canvas.restore();
    }

    // ── 长按航点标记（橙色圆圈 + 序号标签） ──
    for (var i = 0; i < longPressMarkers.length; i++) {
      final pt = longPressMarkers[i];
      canvas.drawCircle(pt, 0.35, _longPressFillPaint);
      canvas.drawCircle(pt, 0.35, _longPressStrokePaint);
      canvas.drawCircle(pt, 0.1, _longPressDotPaint);
      // Waypoint number label (white text, black stroke, above marker)
      canvas.save();
      canvas.translate(pt.dx, pt.dy);
      canvas.scale(0.04, -0.04); // flip Y back for text, ~10sp at world scale
      final label = '${i + 1}';
      // Black outline
      final outlineTp = TextPainter(
        text: TextSpan(text: label, style: TextStyle(
          fontSize: 10,
          fontWeight: FontWeight.w700,
          foreground: Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 2.5
            ..color = Colors.black,
        )),
        textDirection: TextDirection.ltr,
      )..layout();
      outlineTp.paint(canvas, Offset(-outlineTp.width / 2, -outlineTp.height - 8));
      // White fill
      final fillTp = TextPainter(
        text: TextSpan(text: label, style: const TextStyle(
          fontSize: 10, fontWeight: FontWeight.w700, color: Colors.white,
        )),
        textDirection: TextDirection.ltr,
      )..layout();
      fillTp.paint(canvas, Offset(-fillTp.width / 2, -fillTp.height - 8));
      canvas.restore();
    }

    // ── 坐标轴 ──
    canvas.drawLine(const Offset(-1, 0), const Offset(1, 0), _axisPaint);
    canvas.drawLine(const Offset(0, -1), const Offset(0, 1), _axisPaint);
  }

  @override
  bool shouldRepaint(covariant TrajectoryPainter old) =>
      dataVersion != old.dataVersion || _pathLength != old._pathLength ||
      _poseX != old._poseX || _poseY != old._poseY || navGoalPoint != old.navGoalPoint ||
      globalPathPoints.length != old.globalPathPoints.length ||
      occupancyGrid != old.occupancyGrid ||
      frontierMarkers.length != old.frontierMarkers.length ||
      longPressMarkers.length != old.longPressMarkers.length ||
      !identical(globalBuckets, old.globalBuckets) ||
      !identical(localBuckets, old.localBuckets);
}

// ═══════════════════════════════════════════════════════════════
//  Crosshair Painter — center crosshair overlay
// ═══════════════════════════════════════════════════════════════

class _CrosshairPainter extends CustomPainter {
  static final _paint = Paint()
    ..color = Colors.white.withValues(alpha: 0.5)
    ..strokeWidth = 1;

  @override
  void paint(Canvas canvas, Size size) {
    final cx = size.width / 2;
    final cy = size.height / 2;
    const arm = 20.0;
    canvas.drawLine(Offset(cx - arm, cy), Offset(cx + arm, cy), _paint);
    canvas.drawLine(Offset(cx, cy - arm), Offset(cx, cy + arm), _paint);
  }

  @override
  bool shouldRepaint(covariant _CrosshairPainter old) => false;
}

// ═══════════════════════════════════════════════════════════════
//  Measurement Painter — dashed line + distance label
// ═══════════════════════════════════════════════════════════════

class _MeasurePainter extends CustomPainter {
  final Offset start;
  final Offset? end;
  final Matrix4 transform;

  _MeasurePainter({required this.start, this.end, required this.transform});

  Offset _toScreen(double x, double y) {
    final m = transform;
    return Offset(
      m[0] * x + m[4] * (-y) + m[12],
      m[1] * x + m[5] * (-y) + m[13],
    );
  }

  @override
  void paint(Canvas canvas, Size size) {
    final startScreen = _toScreen(start.dx, start.dy);

    // Start point — green dot
    canvas.drawCircle(startScreen, 6, Paint()..color = const Color(0xFF34C759));
    canvas.drawCircle(startScreen, 6, Paint()
      ..color = Colors.white..style = PaintingStyle.stroke..strokeWidth = 2);

    if (end == null) return;

    final endScreen = _toScreen(end!.dx, end!.dy);

    // End point — red dot
    canvas.drawCircle(endScreen, 6, Paint()..color = const Color(0xFFFF3B30));
    canvas.drawCircle(endScreen, 6, Paint()
      ..color = Colors.white..style = PaintingStyle.stroke..strokeWidth = 2);

    // Dashed line
    final dashPaint = Paint()
      ..color = Colors.white
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;
    final dx = endScreen.dx - startScreen.dx;
    final dy = endScreen.dy - startScreen.dy;
    final dist = math.sqrt(dx * dx + dy * dy);
    const dashLen = 8.0;
    const gapLen = 4.0;
    if (dist > 0) {
      final ux = dx / dist;
      final uy = dy / dist;
      double d = 0;
      while (d < dist) {
        final end2 = math.min(d + dashLen, dist);
        canvas.drawLine(
          Offset(startScreen.dx + ux * d, startScreen.dy + uy * d),
          Offset(startScreen.dx + ux * end2, startScreen.dy + uy * end2),
          dashPaint,
        );
        d += dashLen + gapLen;
      }
    }

    // Distance label (world distance in meters)
    final worldDist = (end! - start).distance;
    final label = '${worldDist.toStringAsFixed(2)} m';
    final mid = Offset((startScreen.dx + endScreen.dx) / 2,
                        (startScreen.dy + endScreen.dy) / 2);
    final tp = TextPainter(
      text: TextSpan(text: label, style: const TextStyle(
        color: Colors.white, fontSize: 14, fontWeight: FontWeight.w700,
      )),
      textDirection: TextDirection.ltr,
    )..layout();
    // Background rect
    final rect = Rect.fromCenter(
      center: mid - const Offset(0, 16),
      width: tp.width + 12,
      height: tp.height + 6,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(rect, const Radius.circular(4)),
      Paint()..color = Colors.black.withValues(alpha: 0.7),
    );
    tp.paint(canvas, Offset(rect.left + 6, rect.top + 3));
  }

  @override
  bool shouldRepaint(covariant _MeasurePainter old) =>
      start != old.start || end != old.end || transform != old.transform;
}

// ═══════════════════════════════════════════════════════════════
//  Data models
// ═══════════════════════════════════════════════════════════════

/// 占用栅格数据模型
///
/// Binary wire format (from gRPC DataChunk):
///   Bytes  0-3:  width  (int32 LE, cells)
///   Bytes  4-7:  height (int32 LE, cells)
///   Bytes  8-11: resolution (float32 LE, metres/cell)
///   Bytes 12-15: origin_x   (float32 LE, world metres)
///   Bytes 16-19: origin_y   (float32 LE, world metres)
///   Bytes 20+:   width*height uint8 values
///                  0   = free
///                  128 = unknown
///                  255 = occupied
class _OccupancyGrid {
  final int width;
  final int height;
  final double resolution;
  final double originX;
  final double originY;
  final Uint8List data;

  const _OccupancyGrid({
    required this.width,
    required this.height,
    required this.resolution,
    required this.originX,
    required this.originY,
    required this.data,
  });
}

/// 前沿探索标记数据模型
///
/// Binary wire format (from DataChunk):
///   Bytes 0-3:  count (uint32 LE)
///   Per marker × 12 bytes: float32 x, float32 y, float32 score (LE)
///   score ∈ [0, 1] — higher = more informative frontier
class _FrontierMarker {
  final double x;
  final double y;
  final double score; // [0, 1]

  const _FrontierMarker({required this.x, required this.y, required this.score});
}
