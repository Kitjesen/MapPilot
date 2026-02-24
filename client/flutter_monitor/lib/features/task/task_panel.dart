import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/control.pb.dart';

/// 任务控制面板 — 极简 Cursor 风格
class TaskPanel extends StatefulWidget {
  final List<NavigationGoal>? presetWaypoints;
  const TaskPanel({super.key, this.presetWaypoints});

  @override
  State<TaskPanel> createState() => _TaskPanelState();
}

class _TaskPanelState extends State<TaskPanel> {
  TaskType _selectedType = TaskType.TASK_TYPE_NAVIGATION;

  final List<NavigationGoal> _waypoints = [];
  bool _loop = false;

  final TextEditingController _mapNameCtrl = TextEditingController();
  bool _saveOnComplete = true;

  // ─── Semantic nav state ───
  final TextEditingController _semanticInstructionCtrl = TextEditingController();
  bool _semanticExplore = true;

  // ─── Follow person state ───
  final TextEditingController _followPersonTargetCtrl = TextEditingController();
  double _followPersonDistance = 1.5;

  // ─── Backend waypoint state ───
  GetActiveWaypointsResponse? _activeWpResp;
  bool _wpLoading = false;

  @override
  void initState() {
    super.initState();
    if (widget.presetWaypoints != null) _waypoints.addAll(widget.presetWaypoints!);
  }

  @override
  void dispose() {
    _mapNameCtrl.dispose();
    _semanticInstructionCtrl.dispose();
    _followPersonTargetCtrl.dispose();
    super.dispose();
  }

  // ── Waypoint queries ──

  Future<void> _fetchActiveWaypoints() async {
    if (_wpLoading) return;
    setState(() => _wpLoading = true);
    final resp = await context.read<TaskGateway>().getActiveWaypoints();
    if (mounted) setState(() { _activeWpResp = resp; _wpLoading = false; });
  }

  Future<void> _clearWaypoints() async {
    final ok = await context.read<TaskGateway>().clearWaypoints();
    if (ok && mounted) {
      _snack(context.read<LocaleProvider>().tr('航点已清除', 'Waypoints cleared'));
      _fetchActiveWaypoints();
    }
  }

  // ── Actions (delegate to TaskGateway) ──

  Future<void> _start() async {
    HapticFeedback.lightImpact();
    final gw = context.read<TaskGateway>();
    final locale = context.read<LocaleProvider>();

    // Pre-start: check for active waypoints from backend
    if (_selectedType != TaskType.TASK_TYPE_MAPPING) {
      final active = await gw.getActiveWaypoints();
      if (active != null && active.totalCount > 0 && mounted) {
        final sourceLabel = switch (active.source) {
          WaypointSource.WAYPOINT_SOURCE_APP => locale.tr('App 任务', 'App task'),
          WaypointSource.WAYPOINT_SOURCE_PLANNER => locale.tr('全局规划器', 'Global planner'),
          _ => locale.tr('未知', 'Unknown'),
        };
        final progressPct = (active.progressPercent * 100).toStringAsFixed(0);
        final proceed = await showDialog<bool>(
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
              TextButton(onPressed: () => Navigator.of(ctx).pop(false),
                child: Text(locale.tr('取消', 'Cancel'))),
              FilledButton(onPressed: () => Navigator.of(ctx).pop(true),
                child: Text(locale.tr('清除并继续', 'Clear & continue'))),
            ],
          ),
        );
        if (proceed != true) return;
        await gw.clearWaypoints();
      }
    }

    if (_selectedType == TaskType.TASK_TYPE_SEMANTIC_NAV) {
      final instruction = _semanticInstructionCtrl.text.trim();
      if (instruction.isEmpty) {
        _snack(locale.tr('请输入指令', 'Please enter an instruction'), err: true);
        return;
      }
      final ok = await gw.startSemanticNav(
        instruction,
        exploreIfUnknown: _semanticExplore,
      );
      _showStatus(gw, ok);
      return;
    }

    if (_selectedType == TaskType.TASK_TYPE_FOLLOW_PERSON) {
      final target = _followPersonTargetCtrl.text.trim();
      final ok = await gw.startFollowPerson(
        target.isEmpty ? 'person' : target,
        followDistance: _followPersonDistance,
      );
      _showStatus(gw, ok);
      return;
    }

    if (_selectedType == TaskType.TASK_TYPE_MAPPING) {
      final ok = await gw.startTask(
        TaskType.TASK_TYPE_MAPPING,
        mappingParams: MappingParams()
          ..mapName = _mapNameCtrl.text.isNotEmpty
              ? _mapNameCtrl.text
              : 'map_${DateTime.now().millisecondsSinceEpoch}'
          ..saveOnComplete = _saveOnComplete,
      );
      _showStatus(gw, ok);
    } else {
      if (_selectedType == TaskType.TASK_TYPE_NAVIGATION) {
        final ok = await gw.startNavigationTask(_waypoints, loop: _loop);
        _showStatus(gw, ok);
        return;
      }
      if (_waypoints.isEmpty) {
        _snack(locale.tr('请先添加航点', 'Please add waypoints first'), err: true);
        return;
      }
      final ok = await gw.startTask(
        _selectedType,
        navigationParams: NavigationParams()
          ..waypoints.addAll(_waypoints)
          ..loop = _loop,
      );
      _showStatus(gw, ok);
    }
  }

  Future<void> _pause() async {
    HapticFeedback.selectionClick();
    final gw = context.read<TaskGateway>();
    final locale = context.read<LocaleProvider>();
    final ok = await gw.pauseTask();
    if (!ok) _snack(gw.statusMessage ?? locale.tr('暂停失败', 'Pause failed'), err: true);
  }

  Future<void> _resume() async {
    HapticFeedback.selectionClick();
    final gw = context.read<TaskGateway>();
    final locale = context.read<LocaleProvider>();
    final ok = await gw.resumeTask();
    if (!ok) _snack(gw.statusMessage ?? locale.tr('恢复失败', 'Resume failed'), err: true);
  }

  Future<void> _cancel() async {
    HapticFeedback.mediumImpact();
    final gw = context.read<TaskGateway>();
    final ok = await gw.cancelTask();
    _showStatus(gw, ok);
  }

  void _showStatus(TaskGateway gw, bool ok) {
    final msg = gw.statusMessage;
    if (msg != null) _snack(msg, err: !ok);
  }

  void _snack(String msg, {bool err = false}) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(
        err ? UiErrorMapper.fromMessage(msg) : msg,
        style: const TextStyle(fontSize: 13),
      ),
      backgroundColor: err ? AppColors.error : AppColors.success,
      behavior: SnackBarBehavior.floating,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
    ));
  }

  void _addWaypoint() {
    HapticFeedback.selectionClick();
    showModalBottomSheet(
      context: context, isScrollControlled: true, backgroundColor: Colors.transparent,
      builder: (_) => _WaypointSheet(onSave: (g) => setState(() => _waypoints.add(g))),
    );
  }

  // ── Build ──

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    final gw = context.watch<TaskGateway>();

    return Scaffold(
      backgroundColor: context.isDark ? AppColors.darkBackground : AppColors.lightBackground,
      appBar: AppBar(
        title: Text(locale.tr('任务控制', 'Task Control')),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 17),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: gw.isRunning ? _runningView(gw, locale) : _setupView(locale),
      ),
    );
  }

  // ════════════════════════════════════════════
  //  Setup View
  // ════════════════════════════════════════════

  Widget _setupView(LocaleProvider locale) {
    return ListView(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 8),
      children: [
        _label(locale.tr('任务类型', 'Task type')),
        const SizedBox(height: 6),
        _typeSelector(locale),
        const SizedBox(height: 24),
        if (_selectedType == TaskType.TASK_TYPE_SEMANTIC_NAV)
          _semanticNavSection(locale)
        else if (_selectedType == TaskType.TASK_TYPE_FOLLOW_PERSON)
          _followPersonSection(locale)
        else if (_selectedType == TaskType.TASK_TYPE_MAPPING)
          _mappingSection(locale)
        else
          _navSection(locale),
        const SizedBox(height: 32),
        _startBtn(locale),
        const SizedBox(height: 80),
      ],
    );
  }

  Widget _label(String t) => Text(t, style: TextStyle(fontSize: 12, color: context.subtitleColor, fontWeight: FontWeight.w500));

  Widget _typeSelector(LocaleProvider locale) {
    final items = [
      (TaskType.TASK_TYPE_NAVIGATION, locale.tr('导航', 'Nav')),
      (TaskType.TASK_TYPE_MAPPING, locale.tr('建图', 'Map')),
      (TaskType.TASK_TYPE_INSPECTION, locale.tr('巡检', 'Patrol')),
      (TaskType.TASK_TYPE_RETURN_HOME, locale.tr('回家', 'Home')),
      (TaskType.TASK_TYPE_FOLLOW_PATH, locale.tr('循迹', 'Follow')),
      (TaskType.TASK_TYPE_SEMANTIC_NAV, locale.tr('语义', 'Semantic')),
      (TaskType.TASK_TYPE_FOLLOW_PERSON, locale.tr('跟随', 'Track')),
    ];
    return _card(
      child: GridView.count(
        crossAxisCount: 4,
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        childAspectRatio: 2.8,
        children: items.map((t) {
          final sel = _selectedType == t.$1;
          return GestureDetector(
            onTap: () { HapticFeedback.selectionClick(); setState(() => _selectedType = t.$1); },
            child: Container(
              padding: const EdgeInsets.symmetric(vertical: 10),
              decoration: BoxDecoration(
                color: sel ? (context.isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.04)) : Colors.transparent,
                borderRadius: BorderRadius.circular(6),
              ),
              child: Center(
                child: Text(t.$2, style: TextStyle(
                  fontSize: 13,
                  fontWeight: sel ? FontWeight.w600 : FontWeight.w400,
                  color: sel ? context.titleColor : context.subtitleColor,
                )),
              ),
            ),
          );
        }).toList(),
      ),
    );
  }

  // ── Semantic nav config ──

  static const _quickInstructions = [
    '找灭火器', '门在哪里', '带我去会议室', '哪里有打印机',
    '找最近的椅子', '看看垃圾桶', '去电梯', '找楼梯',
  ];

  Widget _semanticNavSection(LocaleProvider locale) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _label(locale.tr('自然语言指令', 'Natural language instruction')),
        const SizedBox(height: 6),
        _card(
          child: Padding(
            padding: const EdgeInsets.all(4),
            child: TextField(
              controller: _semanticInstructionCtrl,
              decoration: InputDecoration(
                hintText: locale.tr('例: 看一下灭火器在哪', 'e.g. Find the fire extinguisher'),
                hintStyle: TextStyle(color: context.subtitleColor.withValues(alpha: 0.5)),
                border: InputBorder.none,
                contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                suffixIcon: _semanticInstructionCtrl.text.isNotEmpty
                    ? IconButton(
                        icon: Icon(Icons.clear, size: 18, color: context.subtitleColor),
                        onPressed: () => setState(() => _semanticInstructionCtrl.clear()),
                      )
                    : null,
              ),
              style: TextStyle(fontSize: 14, color: context.titleColor),
              maxLines: 2,
              textInputAction: TextInputAction.done,
              onChanged: (_) => setState(() {}),
            ),
          ),
        ),
        const SizedBox(height: 12),
        _label(locale.tr('快捷指令', 'Quick commands')),
        const SizedBox(height: 6),
        Wrap(
          spacing: 8,
          runSpacing: 6,
          children: _quickInstructions.map((text) {
            return GestureDetector(
              onTap: () {
                HapticFeedback.selectionClick();
                setState(() => _semanticInstructionCtrl.text = text);
              },
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
                decoration: BoxDecoration(
                  color: context.isDark
                      ? Colors.white.withValues(alpha: 0.06)
                      : Colors.black.withValues(alpha: 0.04),
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(
                    color: context.isDark
                        ? Colors.white.withValues(alpha: 0.08)
                        : Colors.black.withValues(alpha: 0.06),
                  ),
                ),
                child: Text(
                  text,
                  style: TextStyle(fontSize: 12, color: context.subtitleColor),
                ),
              ),
            );
          }).toList(),
        ),
        const SizedBox(height: 16),
        Row(
          children: [
            SizedBox(
              height: 28,
              child: Switch(
                value: _semanticExplore,
                onChanged: (v) => setState(() => _semanticExplore = v),
              ),
            ),
            const SizedBox(width: 8),
            Text(locale.tr('未知目标自动探索', 'Auto-explore unknown targets'), style: TextStyle(fontSize: 13, color: context.subtitleColor)),
          ],
        ),
      ],
    );
  }

  // ── Follow person config ──

  Widget _followPersonSection(LocaleProvider locale) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _label(locale.tr('跟随目标描述', 'Follow target description')),
        const SizedBox(height: 6),
        _card(
          child: Padding(
            padding: const EdgeInsets.all(4),
            child: TextField(
              controller: _followPersonTargetCtrl,
              decoration: InputDecoration(
                hintText: locale.tr('例: "穿红衣服的人" 或 "person"（留空默认跟随最近的人）',
                    'e.g. "person in red" or "person" (empty = follow nearest)'),
                hintStyle: TextStyle(color: context.subtitleColor.withValues(alpha: 0.5), fontSize: 12),
                border: InputBorder.none,
                contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                suffixIcon: _followPersonTargetCtrl.text.isNotEmpty
                    ? IconButton(
                        icon: Icon(Icons.clear, size: 18, color: context.subtitleColor),
                        onPressed: () => setState(() => _followPersonTargetCtrl.clear()),
                      )
                    : null,
              ),
              style: TextStyle(fontSize: 14, color: context.titleColor),
              maxLines: 1,
              textInputAction: TextInputAction.done,
              onChanged: (_) => setState(() {}),
            ),
          ),
        ),
        const SizedBox(height: 16),
        _label(locale.tr('跟随距离: ${_followPersonDistance.toStringAsFixed(1)} m',
            'Follow distance: ${_followPersonDistance.toStringAsFixed(1)} m')),
        const SizedBox(height: 4),
        Slider(
          value: _followPersonDistance,
          min: 0.5,
          max: 4.0,
          divisions: 7,
          label: '${_followPersonDistance.toStringAsFixed(1)} m',
          onChanged: (v) => setState(() => _followPersonDistance = v),
        ),
      ],
    );
  }

  // ── Navigation config ──

  Widget _navSection(LocaleProvider locale) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _label(locale.tr('航点', 'Waypoints')),
        const SizedBox(height: 6),
        if (_waypoints.isEmpty)
          _card(
            child: Padding(
              padding: const EdgeInsets.symmetric(vertical: 20),
              child: Center(
                child: Text(locale.tr('暂无航点', 'No waypoints'), style: TextStyle(fontSize: 13, color: context.subtitleColor)),
              ),
            ),
          )
        else
          _card(
            child: Column(
              children: [
                for (int i = 0; i < _waypoints.length; i++) ...[
                  if (i > 0) Divider(height: 1, color: context.dividerColor),
                  _waypointRow(i),
                ],
              ],
            ),
          ),
        const SizedBox(height: 8),
        Row(
          children: [
            _outlineBtn(locale.tr('添加航点', 'Add waypoint'), onTap: _addWaypoint),
            const SizedBox(width: 8),
            _outlineBtn(locale.tr('从地图选择', 'Select from map'), onTap: () {
              Navigator.of(context).pushNamed('/map-select-goal').then((r) {
                if (r is NavigationGoal) setState(() => _waypoints.add(r));
              });
            }),
          ],
        ),
        const SizedBox(height: 20),
        _label(locale.tr('选项', 'Options')),
        const SizedBox(height: 6),
        _card(
          child: _switchRow(
            locale.tr('循环执行', 'Loop'),
            locale.tr('到达最后航点后返回起点重复', 'Return to start and repeat after reaching last waypoint'),
            _loop, (v) => setState(() => _loop = v)),
        ),
      ],
    );
  }

  Widget _waypointRow(int i) {
    final wp = _waypoints[i];
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      child: Row(
        children: [
          Text('${i + 1}', style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: context.subtitleColor)),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  wp.label.isNotEmpty ? wp.label : context.read<LocaleProvider>().tr('航点 ${i + 1}', 'Waypoint ${i + 1}'),
                  style: TextStyle(fontSize: 14, color: context.titleColor),
                ),
                Text(
                  '${wp.position.x.toStringAsFixed(1)}, ${wp.position.y.toStringAsFixed(1)}, ${wp.position.z.toStringAsFixed(1)}',
                  style: TextStyle(fontSize: 12, color: context.subtitleColor),
                ),
              ],
            ),
          ),
          GestureDetector(
            onTap: () => setState(() => _waypoints.removeAt(i)),
            child: Icon(Icons.close, size: 16, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  // ── Mapping config ──

  Widget _mappingSection(LocaleProvider locale) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _label(locale.tr('建图参数', 'Mapping parameters')),
        const SizedBox(height: 6),
        _card(
          child: Column(
            children: [
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
                child: Row(
                  children: [
                    Text(locale.tr('地图名称', 'Map name'), style: TextStyle(fontSize: 14, color: context.titleColor)),
                    const SizedBox(width: 16),
                    Expanded(
                      child: TextField(
                        controller: _mapNameCtrl,
                        textAlign: TextAlign.end,
                        style: TextStyle(fontSize: 14, color: context.titleColor),
                        decoration: InputDecoration(
                          hintText: locale.tr('自动生成', 'Auto-generated'),
                          hintStyle: TextStyle(fontSize: 14, color: context.subtitleColor.withValues(alpha: 0.5)),
                          border: InputBorder.none,
                          isDense: true,
                          contentPadding: EdgeInsets.zero,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              Divider(height: 1, color: context.dividerColor),
              _switchRow(
                locale.tr('完成后自动保存', 'Auto-save on complete'),
                locale.tr('停止建图时保存地图文件', 'Save map file when mapping stops'),
                _saveOnComplete, (v) => setState(() => _saveOnComplete = v)),
            ],
          ),
        ),
        const SizedBox(height: 12),
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 2),
          child: Text(
            locale.tr(
              '建图模式将启动 SLAM，遥控机器人移动来构建环境地图。',
              'Mapping mode starts SLAM. Drive the robot around to build an environment map.',
            ),
            style: TextStyle(fontSize: 12, color: context.subtitleColor),
          ),
        ),
      ],
    );
  }

  Widget _startBtn(LocaleProvider locale) {
    final label = _selectedType == TaskType.TASK_TYPE_MAPPING
        ? locale.tr('启动建图', 'Start mapping')
        : locale.tr('启动任务', 'Start task');
    return SizedBox(
      width: double.infinity,
      height: 44,
      child: TextButton(
        onPressed: _start,
        style: TextButton.styleFrom(
          backgroundColor: context.isDark ? Colors.white.withValues(alpha: 0.08) : Colors.black.withValues(alpha: 0.05),
          foregroundColor: context.titleColor,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(8),
            side: BorderSide(color: context.borderColor),
          ),
        ),
        child: Text(label, style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
      ),
    );
  }

  // ════════════════════════════════════════════
  //  Running View
  // ════════════════════════════════════════════

  Widget _runningView(TaskGateway gw, LocaleProvider locale) {
    // Auto-fetch backend waypoints on first build
    if (_activeWpResp == null && !_wpLoading) {
      WidgetsBinding.instance.addPostFrameCallback((_) => _fetchActiveWaypoints());
    }

    final name = switch (_selectedType) {
      TaskType.TASK_TYPE_NAVIGATION => locale.tr('导航', 'Navigation'),
      TaskType.TASK_TYPE_MAPPING => locale.tr('建图', 'Mapping'),
      TaskType.TASK_TYPE_INSPECTION => locale.tr('巡检', 'Patrol'),
      TaskType.TASK_TYPE_RETURN_HOME => locale.tr('回家', 'Return home'),
      TaskType.TASK_TYPE_FOLLOW_PATH => locale.tr('循迹', 'Follow path'),
      TaskType.TASK_TYPE_SEMANTIC_NAV => locale.tr('语义导航', 'Semantic nav'),
      _ => locale.tr('任务', 'Task'),
    };

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 20),
      child: Column(
        children: [
          const Spacer(flex: 2),
          // Status
          Text(
            gw.isPaused
                ? locale.tr('$name 已暂停', '$name paused')
                : locale.tr('$name 执行中', '$name running'),
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600, color: context.titleColor),
          ),
          const SizedBox(height: 4),
          if (gw.activeTaskId != null)
            Text(
              gw.activeTaskId!.length > 20 ? '${gw.activeTaskId!.substring(0, 20)}…' : gw.activeTaskId!,
              style: TextStyle(fontSize: 12, color: context.subtitleColor),
            ),
          const SizedBox(height: 24),
          // Progress
          _card(
            child: Padding(
              padding: const EdgeInsets.all(14),
              child: Column(
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      Text(locale.tr('进度', 'Progress'), style: TextStyle(fontSize: 13, color: context.subtitleColor)),
                      Text('${(gw.progress * 100).toStringAsFixed(0)}%', style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: context.titleColor)),
                    ],
                  ),
                  const SizedBox(height: 10),
                  ClipRRect(
                    borderRadius: BorderRadius.circular(3),
                    child: LinearProgressIndicator(
                      value: gw.progress,
                      backgroundColor: context.isDark ? Colors.white.withValues(alpha: 0.06) : Colors.black.withValues(alpha: 0.04),
                      valueColor: AlwaysStoppedAnimation(context.isDark ? Colors.white.withValues(alpha: 0.5) : Colors.black.withValues(alpha: 0.3)),
                      minHeight: 4,
                    ),
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 16),
          // Backend waypoint info
          _backendWaypointCard(),
          const Spacer(flex: 3),
          // Controls
          Row(
            children: [
              Expanded(
                child: _outlineBtn(
                  gw.isPaused ? locale.tr('恢复', 'Resume') : locale.tr('暂停', 'Pause'),
                  onTap: gw.isPaused ? _resume : _pause,
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: SizedBox(
                  height: 40,
                  child: TextButton(
                    onPressed: _cancel,
                    style: TextButton.styleFrom(
                      foregroundColor: AppColors.error,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8),
                        side: BorderSide(color: AppColors.error.withValues(alpha: 0.3)),
                      ),
                    ),
                    child: Text(locale.tr('取消', 'Cancel'), style: const TextStyle(fontSize: 14)),
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 40),
        ],
      ),
    );
  }

  Widget _backendWaypointCard() {
    final locale = context.read<LocaleProvider>();
    final resp = _activeWpResp;
    if (_wpLoading) {
      return _card(
        child: const Padding(
          padding: EdgeInsets.all(14),
          child: Center(child: SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))),
        ),
      );
    }
    if (resp == null || resp.totalCount == 0) {
      return const SizedBox.shrink();
    }
    final sourceLabel = switch (resp.source) {
      WaypointSource.WAYPOINT_SOURCE_APP => 'App',
      WaypointSource.WAYPOINT_SOURCE_PLANNER => locale.tr('规划器', 'Planner'),
      _ => locale.tr('未知', 'Unknown'),
    };
    return _card(
      child: Padding(
        padding: const EdgeInsets.all(14),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text(locale.tr('航点 ($sourceLabel)', 'Waypoints ($sourceLabel)'), style: TextStyle(fontSize: 13, color: context.subtitleColor)),
                Text('${resp.currentIndex + 1} / ${resp.totalCount}',
                  style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: context.titleColor)),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  child: _outlineBtn(locale.tr('刷新', 'Refresh'), onTap: _fetchActiveWaypoints),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: SizedBox(
                    height: 36,
                    child: TextButton(
                      onPressed: _clearWaypoints,
                      style: TextButton.styleFrom(
                        foregroundColor: AppColors.error,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(8),
                          side: BorderSide(color: AppColors.error.withValues(alpha: 0.3)),
                        ),
                      ),
                      child: Text(locale.tr('清除航点', 'Clear waypoints'), style: const TextStyle(fontSize: 12)),
                    ),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  // ════════════════════════════════════════════
  //  Shared widgets
  // ════════════════════════════════════════════

  Widget _card({required Widget child}) {
    return Container(
      width: double.infinity,
      decoration: BoxDecoration(
        color: context.isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: child,
    );
  }

  Widget _switchRow(String title, String sub, bool value, ValueChanged<bool> onChanged) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(title, style: TextStyle(fontSize: 14, color: context.titleColor)),
                const SizedBox(height: 2),
                Text(sub, style: TextStyle(fontSize: 12, color: context.subtitleColor)),
              ],
            ),
          ),
          SizedBox(
            height: 24,
            child: Switch.adaptive(value: value, onChanged: onChanged),
          ),
        ],
      ),
    );
  }

  Widget _outlineBtn(String label, {required VoidCallback onTap}) {
    return SizedBox(
      height: 40,
      child: TextButton(
        onPressed: onTap,
        style: TextButton.styleFrom(
          foregroundColor: context.titleColor,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(8),
            side: BorderSide(color: context.borderColor),
          ),
        ),
        child: Text(label, style: const TextStyle(fontSize: 13)),
      ),
    );
  }
}

// ════════════════════════════════════════════
//  Waypoint Editor Sheet
// ════════════════════════════════════════════

class _WaypointSheet extends StatefulWidget {
  final void Function(NavigationGoal) onSave;
  const _WaypointSheet({required this.onSave});
  @override
  State<_WaypointSheet> createState() => _WaypointSheetState();
}

class _WaypointSheetState extends State<_WaypointSheet> {
  final _x = TextEditingController(text: '0.0');
  final _y = TextEditingController(text: '0.0');
  final _z = TextEditingController(text: '0.0');
  final _yaw = TextEditingController(text: '0.0');
  final _lbl = TextEditingController();
  final _rad = TextEditingController(text: '1.0');

  @override
  void dispose() { _x.dispose(); _y.dispose(); _z.dispose(); _yaw.dispose(); _lbl.dispose(); _rad.dispose(); super.dispose(); }

  void _save() {
    widget.onSave(NavigationGoal()
      ..position = (Vector3()
        ..x = double.tryParse(_x.text) ?? 0
        ..y = double.tryParse(_y.text) ?? 0
        ..z = double.tryParse(_z.text) ?? 0)
      ..yaw = double.tryParse(_yaw.text) ?? 0
      ..label = _lbl.text
      ..arrivalRadius = double.tryParse(_rad.text) ?? 1.0);
    Navigator.pop(context);
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return Container(
      margin: const EdgeInsets.all(16),
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(context.read<LocaleProvider>().tr('添加航点', 'Add waypoint'), style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600, color: context.titleColor)),
          const SizedBox(height: 16),
          _field(context.read<LocaleProvider>().tr('标签', 'Label'), _lbl, context.read<LocaleProvider>().tr('可选', 'Optional')),
          const SizedBox(height: 10),
          Row(children: [
            Expanded(child: _field('X', _x, '0.0', num: true)),
            const SizedBox(width: 8),
            Expanded(child: _field('Y', _y, '0.0', num: true)),
            const SizedBox(width: 8),
            Expanded(child: _field('Z', _z, '0.0', num: true)),
          ]),
          const SizedBox(height: 10),
          Row(children: [
            Expanded(child: _field('Yaw (rad)', _yaw, '0.0', num: true)),
            const SizedBox(width: 8),
            Expanded(child: _field(context.read<LocaleProvider>().tr('半径 (m)', 'Radius (m)'), _rad, '1.0', num: true)),
          ]),
          const SizedBox(height: 18),
          SizedBox(
            width: double.infinity,
            height: 40,
            child: TextButton(
              onPressed: _save,
              style: TextButton.styleFrom(
                foregroundColor: context.titleColor,
                backgroundColor: isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.04),
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8), side: BorderSide(color: context.borderColor)),
              ),
              child: Text(context.read<LocaleProvider>().tr('确定', 'OK'), style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
            ),
          ),
        ],
      ),
    );
  }

  Widget _field(String label, TextEditingController c, String hint, {bool num = false}) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: TextStyle(fontSize: 11, color: context.subtitleColor)),
        const SizedBox(height: 4),
        TextField(
          controller: c,
          keyboardType: num ? const TextInputType.numberWithOptions(decimal: true, signed: true) : TextInputType.text,
          style: TextStyle(fontSize: 14, color: context.titleColor),
          decoration: InputDecoration(
            hintText: hint,
            hintStyle: TextStyle(fontSize: 14, color: context.subtitleColor.withValues(alpha: 0.4)),
            filled: true,
            fillColor: context.inputFillColor,
            border: OutlineInputBorder(borderRadius: BorderRadius.circular(6), borderSide: BorderSide.none),
            isDense: true,
            contentPadding: const EdgeInsets.symmetric(horizontal: 10, vertical: 10),
          ),
        ),
      ],
    );
  }
}
