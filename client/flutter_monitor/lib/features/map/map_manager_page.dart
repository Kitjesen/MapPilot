import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:math' as math;
import 'package:fixnum/fixnum.dart';
import 'package:path_provider/path_provider.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:flutter_monitor/shared/widgets/skeleton_loader.dart';
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
    final locale = context.read<LocaleProvider>();
    final name = await _inputDialog(locale.tr('保存地图', 'Save Map'), locale.tr('名称', 'Name'), 'my_map');
    if (name == null) return;
    final saveName = _normalizeMapSaveName(name);
    if (saveName == null) return;

    // 覆盖保护: 检查同名是否已存在
    final gateway = context.read<MapGateway>();
    if (gateway.mapNameExists(saveName)) {
      final overwrite = await showDialog<bool>(
        context: context,
        builder: (ctx) => AlertDialog(
          title: Text(locale.tr('地图已存在', 'Map Already Exists')),
          content: Text(locale.tr('"$saveName" 已存在，要覆盖吗？', '"$saveName" exists. Overwrite?')),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(ctx, false),
              child: Text(locale.tr('取消', 'Cancel')),
            ),
            TextButton(
              onPressed: () => Navigator.pop(ctx, true),
              child: Text(locale.tr('覆盖', 'Overwrite'), style: const TextStyle(color: AppColors.error)),
            ),
          ],
        ),
      );
      if (overwrite != true) return;
      // 先删除旧的，再保存新的
      final (delOk, delMsg) = await gateway.deleteMap('/maps/$saveName');
      if (!delOk) {
        _snack(locale.tr('无法覆盖: $delMsg', 'Cannot overwrite: $delMsg'), err: true);
        return;
      }
    }

    HapticFeedback.lightImpact();
    final (ok, msg) = await gateway.saveMap('/maps/$saveName');
    _snack(ok ? locale.tr('已保存 $saveName', 'Saved $saveName') : msg, err: !ok);
  }

  Future<void> _loadMap(MapInfo m) async {
    final locale = context.read<LocaleProvider>();
    // 格式感知: 只有 PCD 可用于 relocalize
    if (!MapGateway.canRelocalize(m)) {
      _snack(locale.tr('${_fmtExt(m.name)} 格式不支持加载定位，仅 PCD 可用', '${_fmtExt(m.name)} format not supported, only PCD'), err: true);
      return;
    }
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
    _snack(success ? locale.tr('已加载 ${m.name}', 'Loaded ${m.name}') : msg, err: !success);
  }

  Future<void> _delete(MapInfo m) async {
    final locale = context.read<LocaleProvider>();
    final ok = await showDialog<bool>(context: context, builder: (ctx) => AlertDialog(
      title: Text(locale.tr('删除地图', 'Delete Map')), content: Text(locale.tr('确定删除 "${m.name}"？', 'Delete "${m.name}"?')),
      actions: [
        TextButton(onPressed: () => Navigator.pop(ctx, false), child: Text(locale.tr('取消', 'Cancel'))),
        TextButton(onPressed: () => Navigator.pop(ctx, true), child: Text(locale.tr('删除', 'Delete'), style: const TextStyle(color: AppColors.error))),
      ],
    ));
    if (ok != true) return;
    HapticFeedback.lightImpact();
    final (success, msg) = await context.read<MapGateway>().deleteMap(m.path);
    _snack(msg, err: !success);
  }

  Future<void> _rename(MapInfo m) async {
    final locale = context.read<LocaleProvider>();
    final n = await _inputDialog(locale.tr('重命名', 'Rename'), locale.tr('新名称', 'New Name'), m.name.replaceAll(RegExp(r'\.[^.]+$'), ''));
    if (n == null) return;
    final normalized = _normalizeMapNameInput(n);
    if (normalized == null) return;
    final ext = m.name.contains('.') ? '.${m.name.split('.').last}' : '';
    final full = normalized.endsWith(ext) ? normalized : '$normalized$ext';
    final (success, msg) = await context.read<MapGateway>().renameMap(m.path, full);
    _snack(msg, err: !success);
  }

  Future<void> _download(MapInfo m) async {
    final locale = context.read<LocaleProvider>();
    HapticFeedback.lightImpact();
    final gateway = context.read<MapGateway>();

    // 显示进度对话框
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (ctx) => AlertDialog(
        title: Text(locale.tr('下载 ${m.name}', 'Download ${m.name}')),
        content: ValueListenableBuilder<double>(
          valueListenable: gateway.downloadProgressNotifier,
          builder: (_, progress, __) => Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              LinearProgressIndicator(value: progress > 0 ? progress : null),
              const SizedBox(height: 8),
              Text(
                progress > 0
                    ? '${(progress * 100).toStringAsFixed(0)}%'
                    : locale.tr('准备中...', 'Preparing...'),
                style: TextStyle(fontSize: 12, color: context.subtitleColor),
              ),
            ],
          ),
        ),
      ),
    );

    try {
      final dir = await getApplicationDocumentsDirectory();
      final file = File('${dir.path}/${m.name}');
      final bytes = await gateway.downloadMapToFile(m.path, file);
      if (mounted) Navigator.of(context).pop();
      if (bytes == null || !mounted) {
        _snack(locale.tr('下载失败', 'Download failed'), err: true);
        return;
      }
      if (mounted) {
        final sizeMB = (bytes / (1024 * 1024)).toStringAsFixed(1);
        _snack(locale.tr('已下载 ${m.name} ($sizeMB MB) → ${file.path}', 'Downloaded ${m.name} ($sizeMB MB) → ${file.path}'));
      }
    } catch (e) {
      if (mounted) Navigator.of(context).pop();
      _snack(locale.tr('下载失败: $e', 'Download failed: $e'), err: true);
    }
  }

  Future<String?> _inputDialog(String title, String label, String init) {
    final locale = context.read<LocaleProvider>();
    final ctrl = TextEditingController(text: init);
    return showDialog<String>(context: context, builder: (ctx) => AlertDialog(
      title: Text(title), content: TextField(controller: ctrl, autofocus: true, decoration: InputDecoration(labelText: label)),
      actions: [
        TextButton(onPressed: () => Navigator.pop(ctx), child: Text(locale.tr('取消', 'Cancel'))),
        TextButton(onPressed: () => Navigator.pop(ctx, ctrl.text), child: Text(locale.tr('确定', 'OK'))),
      ],
    ));
  }

  Future<bool?> _relocalizeDialog(MapInfo m) {
    final locale = context.read<LocaleProvider>();
    return showDialog<bool>(context: context, builder: (ctx) => AlertDialog(
      title: Text(locale.tr('加载 ${m.name}', 'Load ${m.name}')),
      content: Column(mainAxisSize: MainAxisSize.min, children: [
        Text(locale.tr('初始位姿（可选）', 'Initial pose (optional)'), style: TextStyle(fontSize: 13, color: context.subtitleColor)),
        const SizedBox(height: 12),
        Row(children: [Expanded(child: _numField('X', _xC)), const SizedBox(width: 8), Expanded(child: _numField('Y', _yC))]),
        const SizedBox(height: 8),
        Row(children: [Expanded(child: _numField('Z', _zC)), const SizedBox(width: 8), Expanded(child: _numField('Yaw', _yawC))]),
      ]),
      actions: [
        TextButton(onPressed: () => Navigator.pop(ctx, false), child: Text(locale.tr('取消', 'Cancel'))),
        TextButton(
          onPressed: () {
            final validationError = _validateRelocalizeInputs();
            if (validationError != null) {
              _snack(validationError, err: true);
              return;
            }
            Navigator.pop(ctx, true);
          },
          child: Text(locale.tr('加载', 'Load')),
        ),
      ],
    ));
  }

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
    final locale = context.read<LocaleProvider>();
    final name = raw.trim();
    if (name.isEmpty) {
      _snack(locale.tr('地图名称不能为空', 'Map name cannot be empty'), err: true);
      return null;
    }
    if (name.contains('/') || name.contains('\\')) {
      _snack(locale.tr('地图名称不能包含路径分隔符', 'Map name cannot contain path separators'), err: true);
      return null;
    }
    if (name.contains('..')) {
      _snack(locale.tr('地图名称不能包含 ".."', 'Map name cannot contain ".."'), err: true);
      return null;
    }
    if (RegExp(r'[<>:"|?*]').hasMatch(name)) {
      _snack(locale.tr('地图名称包含非法字符', 'Map name contains invalid characters'), err: true);
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
      final locale = context.read<LocaleProvider>();
      _snack(locale.tr('地图名称不能为空', 'Map name cannot be empty'), err: true);
      return null;
    }
    return '$withoutExt.pcd';
  }

  String? _validateRelocalizeInputs() {
    final locale = context.read<LocaleProvider>();
    final x = double.tryParse(_xC.text);
    final y = double.tryParse(_yC.text);
    final z = double.tryParse(_zC.text);
    final yaw = double.tryParse(_yawC.text);
    if (x == null || y == null || z == null || yaw == null) {
      return locale.tr('参数错误，请检查输入', 'Invalid parameters, check input');
    }
    if (!x.isFinite || !y.isFinite || !z.isFinite || !yaw.isFinite) {
      return locale.tr('参数错误，请检查输入', 'Invalid parameters, check input');
    }
    if (yaw < -math.pi || yaw > math.pi) {
      return locale.tr('Yaw 超出范围，请输入 -pi 到 pi', 'Yaw out of range, enter -pi to pi');
    }
    return null;
  }

  // ── Build ──

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final locale = context.watch<LocaleProvider>();
    final gateway = context.watch<MapGateway>();

    return Scaffold(
      backgroundColor: isDark ? AppColors.darkBackground : AppColors.lightBackground,
      appBar: AppBar(
        title: Text(locale.tr('地图管理', 'Map Manager')),
        leading: IconButton(icon: const Icon(Icons.arrow_back_ios_new, size: 17), onPressed: () => Navigator.pop(context)),
        actions: [IconButton(icon: const Icon(Icons.refresh, size: 18), tooltip: locale.tr('刷新', 'Refresh'), onPressed: () => gateway.refreshMaps())],
      ),
      body: gateway.isLoading
          ? const Padding(
              padding: EdgeInsets.all(20),
              child: Column(children: [
                SkeletonListTile(),
                SizedBox(height: 8),
                SkeletonListTile(),
                SizedBox(height: 8),
                SkeletonListTile(),
              ]),
            )
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
              child: Text(locale.tr('保存当前地图', 'Save Current Map'), style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
            ),
          ),
        ),
      ),
    );
  }

  Widget _errView(MapGateway gateway) {
    final locale = context.watch<LocaleProvider>();
    return Center(
      child: Column(mainAxisSize: MainAxisSize.min, children: [
        Text(gateway.error!, style: TextStyle(fontSize: 13, color: context.subtitleColor)),
        const SizedBox(height: 12),
        TextButton(onPressed: () => gateway.refreshMaps(), child: Text(locale.tr('重试', 'Retry'))),
      ]),
    );
  }

  Widget _listView(MapGateway gateway) {
    final locale = context.watch<LocaleProvider>();
    final maps = gateway.maps;
    if (maps.isEmpty) {
      return Center(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          Text(locale.tr('暂无地图', 'No maps yet'), style: TextStyle(fontSize: 15, color: context.subtitleColor)),
          const SizedBox(height: 4),
          Text(locale.tr('建图后保存即可在此管理', 'Save a map after mapping to manage here'), style: TextStyle(fontSize: 12, color: context.subtitleColor)),
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
              child: Text(locale.tr('${maps.length} 个地图', '${maps.length} maps'), style: TextStyle(fontSize: 12, color: context.subtitleColor)),
            );
          }
          final m = maps[i - 1];
          return _mapRow(m, isLast: i == maps.length);
        },
      ),
    );
  }

  Widget _mapRow(MapInfo m, {bool isLast = false}) {
    final locale = context.watch<LocaleProvider>();
    final canLoad = MapGateway.canRelocalize(m);
    final ext = _fmtExt(m.name);

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
                    Row(children: [
                      Flexible(
                        child: Text(
                          m.name,
                          style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w500,
                            color: canLoad
                                ? context.titleColor
                                : context.subtitleColor,
                          ),
                          overflow: TextOverflow.ellipsis,
                        ),
                      ),
                      if (!canLoad) ...[
                        const SizedBox(width: 6),
                        Container(
                          padding: const EdgeInsets.symmetric(
                              horizontal: 5, vertical: 1),
                          decoration: BoxDecoration(
                            color: context.isDark
                                ? Colors.white.withValues(alpha: 0.08)
                                : Colors.black.withValues(alpha: 0.05),
                            borderRadius: BorderRadius.circular(4),
                          ),
                          child: Text(
                            ext.toUpperCase(),
                            style: TextStyle(
                              fontSize: 9,
                              fontWeight: FontWeight.w600,
                              color: context.subtitleColor,
                            ),
                          ),
                        ),
                      ],
                    ]),
                    const SizedBox(height: 3),
                    Text(
                      [
                        _fmtSize(m.sizeBytes),
                        if (m.pointCount > 0) locale.tr('${_fmtPts(m.pointCount)} 点', '${_fmtPts(m.pointCount)} pts'),
                        if (m.modifiedAt.isNotEmpty)
                          m.modifiedAt.split('T').first,
                      ].join('  ·  '),
                      style: TextStyle(
                          fontSize: 12, color: context.subtitleColor),
                    ),
                  ],
                ),
              ),
              // Actions
              PopupMenuButton<String>(
                iconSize: 18,
                padding: EdgeInsets.zero,
                icon: Icon(Icons.more_horiz, size: 18,
                    color: context.subtitleColor),
                onSelected: (v) {
                  if (v == 'load') _loadMap(m);
                  if (v == 'download') _download(m);
                  if (v == 'rename') _rename(m);
                  if (v == 'delete') _delete(m);
                },
                itemBuilder: (_) => [
                  if (canLoad)
                    PopupMenuItem(
                        value: 'load', child: Text(locale.tr('加载定位', 'Load & Localize'))),
                  PopupMenuItem(
                      value: 'download', child: Text(locale.tr('下载到本机', 'Download'))),
                  PopupMenuItem(
                      value: 'rename', child: Text(locale.tr('重命名', 'Rename'))),
                  PopupMenuItem(
                      value: 'delete',
                      child: Text(locale.tr('删除', 'Delete'),
                          style: const TextStyle(color: AppColors.error))),
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
  String _fmtExt(String name) { final dot = name.lastIndexOf('.'); return dot >= 0 ? name.substring(dot + 1) : ''; }
}
