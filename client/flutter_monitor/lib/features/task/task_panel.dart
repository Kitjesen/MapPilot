import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';
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

  @override
  void initState() {
    super.initState();
    if (widget.presetWaypoints != null) _waypoints.addAll(widget.presetWaypoints!);
  }

  @override
  void dispose() {
    _mapNameCtrl.dispose();
    super.dispose();
  }

  // ── Actions (delegate to TaskGateway) ──

  Future<void> _start() async {
    HapticFeedback.lightImpact();
    final gw = context.read<TaskGateway>();

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
        _snack('请先添加航点', err: true);
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
    final ok = await gw.pauseTask();
    if (!ok) _snack(gw.statusMessage ?? '暂停失败', err: true);
  }

  Future<void> _resume() async {
    HapticFeedback.selectionClick();
    final gw = context.read<TaskGateway>();
    final ok = await gw.resumeTask();
    if (!ok) _snack(gw.statusMessage ?? '恢复失败', err: true);
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
    final gw = context.watch<TaskGateway>();

    return Scaffold(
      backgroundColor: context.isDark ? AppColors.darkBackground : AppColors.lightBackground,
      appBar: AppBar(
        title: const Text('任务控制'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 17),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: gw.isRunning ? _runningView(gw) : _setupView(),
      ),
    );
  }

  // ════════════════════════════════════════════
  //  Setup View
  // ════════════════════════════════════════════

  Widget _setupView() {
    return ListView(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 8),
      children: [
        _label('任务类型'),
        const SizedBox(height: 6),
        _typeSelector(),
        const SizedBox(height: 24),
        if (_selectedType == TaskType.TASK_TYPE_MAPPING) _mappingSection() else _navSection(),
        const SizedBox(height: 32),
        _startBtn(),
        const SizedBox(height: 80),
      ],
    );
  }

  Widget _label(String t) => Text(t, style: TextStyle(fontSize: 12, color: context.subtitleColor, fontWeight: FontWeight.w500));

  Widget _typeSelector() {
    const items = [
      (TaskType.TASK_TYPE_NAVIGATION, '导航'),
      (TaskType.TASK_TYPE_MAPPING, '建图'),
      (TaskType.TASK_TYPE_INSPECTION, '巡检'),
      (TaskType.TASK_TYPE_RETURN_HOME, '回家'),
      (TaskType.TASK_TYPE_FOLLOW_PATH, '循迹'),
    ];
    return _card(
      child: Row(
        children: items.map((t) {
          final sel = _selectedType == t.$1;
          return Expanded(
            child: GestureDetector(
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
            ),
          );
        }).toList(),
      ),
    );
  }

  // ── Navigation config ──

  Widget _navSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _label('航点'),
        const SizedBox(height: 6),
        if (_waypoints.isEmpty)
          _card(
            child: Padding(
              padding: const EdgeInsets.symmetric(vertical: 20),
              child: Center(
                child: Text('暂无航点', style: TextStyle(fontSize: 13, color: context.subtitleColor)),
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
            _outlineBtn('添加航点', onTap: _addWaypoint),
            const SizedBox(width: 8),
            _outlineBtn('从地图选择', onTap: () {
              Navigator.of(context).pushNamed('/map-select-goal').then((r) {
                if (r is NavigationGoal) setState(() => _waypoints.add(r));
              });
            }),
          ],
        ),
        const SizedBox(height: 20),
        _label('选项'),
        const SizedBox(height: 6),
        _card(
          child: _switchRow('循环执行', '到达最后航点后返回起点重复', _loop, (v) => setState(() => _loop = v)),
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
                  wp.label.isNotEmpty ? wp.label : '航点 ${i + 1}',
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

  Widget _mappingSection() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _label('建图参数'),
        const SizedBox(height: 6),
        _card(
          child: Column(
            children: [
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
                child: Row(
                  children: [
                    Text('地图名称', style: TextStyle(fontSize: 14, color: context.titleColor)),
                    const SizedBox(width: 16),
                    Expanded(
                      child: TextField(
                        controller: _mapNameCtrl,
                        textAlign: TextAlign.end,
                        style: TextStyle(fontSize: 14, color: context.titleColor),
                        decoration: InputDecoration(
                          hintText: '自动生成',
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
              _switchRow('完成后自动保存', '停止建图时保存地图文件', _saveOnComplete, (v) => setState(() => _saveOnComplete = v)),
            ],
          ),
        ),
        const SizedBox(height: 12),
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 2),
          child: Text(
            '建图模式将启动 SLAM，遥控机器人移动来构建环境地图。',
            style: TextStyle(fontSize: 12, color: context.subtitleColor),
          ),
        ),
      ],
    );
  }

  Widget _startBtn() {
    final label = _selectedType == TaskType.TASK_TYPE_MAPPING ? '启动建图' : '启动任务';
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

  Widget _runningView(TaskGateway gw) {
    final name = switch (_selectedType) {
      TaskType.TASK_TYPE_NAVIGATION => '导航',
      TaskType.TASK_TYPE_MAPPING => '建图',
      TaskType.TASK_TYPE_INSPECTION => '巡检',
      TaskType.TASK_TYPE_RETURN_HOME => '回家',
      TaskType.TASK_TYPE_FOLLOW_PATH => '循迹',
      _ => '任务',
    };

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 20),
      child: Column(
        children: [
          const Spacer(flex: 2),
          // Status
          Text(
            gw.isPaused ? '$name 已暂停' : '$name 执行中',
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
                      Text('进度', style: TextStyle(fontSize: 13, color: context.subtitleColor)),
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
          const Spacer(flex: 3),
          // Controls
          Row(
            children: [
              Expanded(
                child: _outlineBtn(
                  gw.isPaused ? '恢复' : '暂停',
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
                    child: const Text('取消', style: TextStyle(fontSize: 14)),
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
          Text('添加航点', style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600, color: context.titleColor)),
          const SizedBox(height: 16),
          _field('标签', _lbl, '可选'),
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
            Expanded(child: _field('半径 (m)', _rad, '1.0', num: true)),
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
              child: const Text('确定', style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
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
