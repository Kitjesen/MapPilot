import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:math' as math;
import 'package:fixnum/fixnum.dart';
import 'package:path_provider/path_provider.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';
import 'package:robot_proto/src/system.pb.dart';

/// 地图管理 — 极简 Cursor 风格
class MapManagerPage extends StatefulWidget {
  const MapManagerPage({super.key});
  @override
  State<MapManagerPage> createState() => _MapManagerPageState();
}

class _MapManagerPageState extends State<MapManagerPage> {
  final _xC = TextEditingController(text: '0.0');
  final _yC = TextEditingController(text: '0.0');
  final _zC = TextEditingController(text: '0.0');
  final _yawC = TextEditingController(text: '0.0');

  @override
  void initState() {
    super.initState();
    // 首次加载地图列表
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<MapGateway>().refreshMaps();
    });
  }

  @override
  void dispose() { _xC.dispose(); _yC.dispose(); _zC.dispose(); _yawC.dispose(); super.dispose(); }

  Future<void> _save() async {
    final name = await _inputDialog('保存地图', '名称', 'my_map');
    if (name == null) return;
    final saveName = _normalizeMapSaveName(name);
    if (saveName == null) return;
    HapticFeedback.lightImpact();
    final (ok, msg) =
        await context.read<MapGateway>().saveMap('/maps/$saveName');
    _snack(ok ? '已保存 $saveName' : msg, err: !ok);
  }

  Future<void> _loadMap(MapInfo m) async {
    final ok = await _relocalizeDialog(m);
    if (ok != true) return;
    final validationError = _validateRelocalizeInputs();
    if (validationError != null) {
      _snack(validationError, err: true);
      return;
    }
    HapticFeedback.lightImpact();
    final (success, msg) = await context.read<MapGateway>().relocalize(
      m.path,
      x: double.tryParse(_xC.text) ?? 0,
      y: double.tryParse(_yC.text) ?? 0,
      z: double.tryParse(_zC.text) ?? 0,
      yaw: double.tryParse(_yawC.text) ?? 0,
    );
    _snack(success ? '已加载 ${m.name}' : msg, err: !success);
  }

  Future<void> _delete(MapInfo m) async {
    final ok = await showDialog<bool>(context: context, builder: (ctx) => AlertDialog(
      title: const Text('删除地图'), content: Text('确定删除 "${m.name}"？'),
      actions: [
        TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('取消')),
        TextButton(onPressed: () => Navigator.pop(ctx, true), child: const Text('删除', style: TextStyle(color: AppColors.error))),
      ],
    ));
    if (ok != true) return;
    HapticFeedback.lightImpact();
    final (success, msg) = await context.read<MapGateway>().deleteMap(m.path);
    _snack(msg, err: !success);
  }

  Future<void> _rename(MapInfo m) async {
    final n = await _inputDialog('重命名', '新名称', m.name.replaceAll(RegExp(r'\.[^.]+$'), ''));
    if (n == null) return;
    final normalized = _normalizeMapNameInput(n);
    if (normalized == null) return;
    final ext = m.name.contains('.') ? '.${m.name.split('.').last}' : '';
    final full = normalized.endsWith(ext) ? normalized : '$normalized$ext';
    final (success, msg) = await context.read<MapGateway>().renameMap(m.path, full);
    _snack(msg, err: !success);
  }

  Future<void> _download(MapInfo m) async {
    HapticFeedback.lightImpact();
    _snack('正在下载 ${m.name}...');
    final gateway = context.read<MapGateway>();
    final bytes = await gateway.downloadMap(m.path);
    if (bytes == null || !mounted) {
      _snack('下载失败', err: true);
      return;
    }
    try {
      final dir = await getApplicationDocumentsDirectory();
      final file = File('${dir.path}/${m.name}');
      await file.writeAsBytes(bytes);
      if (mounted) {
        final sizeMB = (bytes.length / (1024 * 1024)).toStringAsFixed(1);
        _snack('已下载 ${m.name} (${sizeMB} MB) → ${file.path}');
      }
    } catch (e) {
      _snack('保存失败: $e', err: true);
    }
  }

  Future<String?> _inputDialog(String title, String label, String init) {
    final ctrl = TextEditingController(text: init);
    return showDialog<String>(context: context, builder: (ctx) => AlertDialog(
      title: Text(title), content: TextField(controller: ctrl, autofocus: true, decoration: InputDecoration(labelText: label)),
      actions: [
        TextButton(onPressed: () => Navigator.pop(ctx), child: const Text('取消')),
        TextButton(onPressed: () => Navigator.pop(ctx, ctrl.text), child: const Text('确定')),
      ],
    ));
  }

  Future<bool?> _relocalizeDialog(MapInfo m) => showDialog<bool>(context: context, builder: (ctx) => AlertDialog(
    title: Text('加载 ${m.name}'),
    content: Column(mainAxisSize: MainAxisSize.min, children: [
      Text('初始位姿（可选）', style: TextStyle(fontSize: 13, color: context.subtitleColor)),
      const SizedBox(height: 12),
      Row(children: [Expanded(child: _numField('X', _xC)), const SizedBox(width: 8), Expanded(child: _numField('Y', _yC))]),
      const SizedBox(height: 8),
      Row(children: [Expanded(child: _numField('Z', _zC)), const SizedBox(width: 8), Expanded(child: _numField('Yaw', _yawC))]),
    ]),
    actions: [
      TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('取消')),
      TextButton(
        onPressed: () {
          final validationError = _validateRelocalizeInputs();
          if (validationError != null) {
            _snack(validationError, err: true);
            return;
          }
          Navigator.pop(ctx, true);
        },
        child: const Text('加载'),
      ),
    ],
  ));

  Widget _numField(String l, TextEditingController c) => TextField(
    controller: c, keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
    decoration: InputDecoration(labelText: l, isDense: true, border: const OutlineInputBorder()),
  );

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

  String? _normalizeMapNameInput(String raw) {
    final name = raw.trim();
    if (name.isEmpty) {
      _snack('地图名称不能为空', err: true);
      return null;
    }
    if (name.contains('/') || name.contains('\\')) {
      _snack('地图名称不能包含路径分隔符', err: true);
      return null;
    }
    if (name.contains('..')) {
      _snack('地图名称不能包含 ".."', err: true);
      return null;
    }
    if (RegExp(r'[<>:"|?*]').hasMatch(name)) {
      _snack('地图名称包含非法字符', err: true);
      return null;
    }
    return name;
  }

  String? _normalizeMapSaveName(String raw) {
    final name = _normalizeMapNameInput(raw);
    if (name == null) return null;
    final withoutExt = name.toLowerCase().endsWith('.pcd')
        ? name.substring(0, name.length - 4)
        : name;
    if (withoutExt.trim().isEmpty) {
      _snack('地图名称不能为空', err: true);
      return null;
    }
    return '$withoutExt.pcd';
  }

  String? _validateRelocalizeInputs() {
    final x = double.tryParse(_xC.text);
    final y = double.tryParse(_yC.text);
    final z = double.tryParse(_zC.text);
    final yaw = double.tryParse(_yawC.text);
    if (x == null || y == null || z == null || yaw == null) {
      return '参数错误，请检查输入';
    }
    if (!x.isFinite || !y.isFinite || !z.isFinite || !yaw.isFinite) {
      return '参数错误，请检查输入';
    }
    if (yaw < -math.pi || yaw > math.pi) {
      return 'Yaw 超出范围，请输入 -pi 到 pi';
    }
    return null;
  }

  // ── Build ──

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final gateway = context.watch<MapGateway>();

    return Scaffold(
      backgroundColor: isDark ? AppColors.darkBackground : AppColors.lightBackground,
      appBar: AppBar(
        title: const Text('地图管理'),
        leading: IconButton(icon: const Icon(Icons.arrow_back_ios_new, size: 17), onPressed: () => Navigator.pop(context)),
        actions: [IconButton(icon: const Icon(Icons.refresh, size: 18), onPressed: () => gateway.refreshMaps())],
      ),
      body: gateway.isLoading
          ? const Center(child: CircularProgressIndicator.adaptive())
          : gateway.error != null
              ? _errView(gateway)
              : _listView(gateway),
      bottomNavigationBar: SafeArea(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(20, 8, 20, 12),
          child: SizedBox(
            height: 44,
            child: TextButton(
              onPressed: _save,
              style: TextButton.styleFrom(
                foregroundColor: context.titleColor,
                backgroundColor: isDark ? Colors.white.withValues(alpha:0.07) : Colors.black.withValues(alpha:0.04),
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8), side: BorderSide(color: context.borderColor)),
              ),
              child: const Text('保存当前地图', style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
            ),
          ),
        ),
      ),
    );
  }

  Widget _errView(MapGateway gateway) => Center(
    child: Column(mainAxisSize: MainAxisSize.min, children: [
      Text(gateway.error!, style: TextStyle(fontSize: 13, color: context.subtitleColor)),
      const SizedBox(height: 12),
      TextButton(onPressed: () => gateway.refreshMaps(), child: const Text('重试')),
    ]),
  );

  Widget _listView(MapGateway gateway) {
    final maps = gateway.maps;
    if (maps.isEmpty) {
      return Center(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          Text('暂无地图', style: TextStyle(fontSize: 15, color: context.subtitleColor)),
          const SizedBox(height: 4),
          Text('建图后保存即可在此管理', style: TextStyle(fontSize: 12, color: context.subtitleColor)),
        ]),
      );
    }
    return RefreshIndicator(
      onRefresh: () => gateway.refreshMaps(),
      child: ListView.builder(
        padding: const EdgeInsets.fromLTRB(20, 4, 20, 20),
        itemCount: maps.length + 1, // +1 for header
        itemBuilder: (_, i) {
          if (i == 0) {
            return Padding(
              padding: const EdgeInsets.only(bottom: 6, top: 4),
              child: Text('${maps.length} 个地图', style: TextStyle(fontSize: 12, color: context.subtitleColor)),
            );
          }
          final m = maps[i - 1];
          return _mapRow(m, isLast: i == maps.length);
        },
      ),
    );
  }

  Widget _mapRow(MapInfo m, {bool isLast = false}) {
    return Container(
      decoration: BoxDecoration(
        color: context.isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      margin: const EdgeInsets.only(bottom: 8),
      child: InkWell(
        onTap: () => _loadMap(m),
        borderRadius: BorderRadius.circular(AppRadius.card),
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
          child: Row(
            children: [
              // Info
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(m.name, style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500, color: context.titleColor)),
                    const SizedBox(height: 3),
                    Text(
                      [
                        _fmtSize(m.sizeBytes),
                        if (m.pointCount > 0) '${_fmtPts(m.pointCount)} 点',
                        if (m.modifiedAt.isNotEmpty) m.modifiedAt.split('T').first,
                      ].join('  ·  '),
                      style: TextStyle(fontSize: 12, color: context.subtitleColor),
                    ),
                  ],
                ),
              ),
              // Actions
              PopupMenuButton<String>(
                iconSize: 18,
                padding: EdgeInsets.zero,
                icon: Icon(Icons.more_horiz, size: 18, color: context.subtitleColor),
                onSelected: (v) {
                  if (v == 'download') _download(m);
                  if (v == 'rename') _rename(m);
                  if (v == 'delete') _delete(m);
                },
                itemBuilder: (_) => [
                  const PopupMenuItem(value: 'download', child: Text('下载到本机')),
                  const PopupMenuItem(value: 'rename', child: Text('重命名')),
                  const PopupMenuItem(value: 'delete', child: Text('删除', style: TextStyle(color: AppColors.error))),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }

  String _fmtSize(Int64 b) { final mb = b.toDouble() / (1024 * 1024); return mb >= 1024 ? '${(mb / 1024).toStringAsFixed(1)} GB' : mb >= 1 ? '${mb.toStringAsFixed(1)} MB' : '${(b.toDouble() / 1024).toStringAsFixed(0)} KB'; }
  String _fmtPts(int c) => c >= 1000000 ? '${(c / 1000000).toStringAsFixed(1)}M' : c >= 1000 ? '${(c / 1000).toStringAsFixed(0)}K' : '$c';
}
