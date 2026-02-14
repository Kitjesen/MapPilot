import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/gateway/runtime_config_gateway.dart';
import 'package:flutter_monitor/core/models/runtime_config.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:flutter_monitor/core/locale/param_strings.dart' as S;

// ═══════════════════════════════════════════════════════════════
//  RuntimeParamsPage — 全参数在线配置页 (中英双语)
//
//  8 个归类 Tab，~130 个参数
//  三种输入控件: Slider / Stepper / TextField
//  基础参数 + 可折叠「高级」区
//  右上角语言切换按钮 🌐
// ═══════════════════════════════════════════════════════════════

class RuntimeParamsPage extends StatelessWidget {
  const RuntimeParamsPage({super.key});

  static const _tabIcons = <String, IconData>{
    'common': Icons.speed_rounded,
    'safety': Icons.security_rounded,
    'navigation': Icons.navigation_rounded,
    'path_follow': Icons.route_rounded,
    'terrain': Icons.terrain_rounded,
    'local_planner': Icons.alt_route_rounded,
    'perception': Icons.radar_rounded,
    'system': Icons.settings_rounded,
  };

  @override
  Widget build(BuildContext context) {
    final gw = context.watch<RuntimeConfigGateway>();
    final locale = context.watch<LocaleProvider>();
    final isEn = locale.isEn;

    return DefaultTabController(
      length: allParamGroups.length,
      child: Scaffold(
        appBar: AppBar(
          title: Text(locale.tr('运行参数', 'Runtime Config')),
          actions: [
            // ── Language toggle ──
            _LanguageToggle(locale: locale),
            // Sync indicator
            if (gw.isSyncing)
              const Padding(
                padding: EdgeInsets.only(right: 12),
                child: Center(
                  child: SizedBox(
                    width: 18, height: 18,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  ),
                ),
              ),
            if (gw.isDirty && !gw.isSyncing)
              IconButton(
                icon: const Icon(Icons.cloud_upload_rounded, size: 22),
                tooltip: locale.tr('推送到机器人', 'Push to Robot'),
                onPressed: () => _pushConfig(context, gw, locale),
              ),
            PopupMenuButton<String>(
              icon: Icon(Icons.more_vert_rounded, color: context.subtitleColor),
              onSelected: (v) {
                if (v == 'reset') _showResetDialog(context, gw, locale);
                if (v == 'revert') gw.revertToServer();
                if (v == 'export') _exportConfig(context, gw, locale);
              },
              itemBuilder: (_) => [
                PopupMenuItem(value: 'reset', child: Text(locale.tr('恢复默认值', 'Reset to Defaults'))),
                if (gw.serverConfig != null)
                  PopupMenuItem(value: 'revert', child: Text(locale.tr('回退到服务端值', 'Revert to Server'))),
                PopupMenuItem(value: 'export', child: Text(locale.tr('导出配置 JSON', 'Export Config JSON'))),
              ],
            ),
          ],
          bottom: TabBar(
            isScrollable: true,
            tabAlignment: TabAlignment.start,
            labelPadding: const EdgeInsets.symmetric(horizontal: 14),
            indicatorSize: TabBarIndicatorSize.label,
            tabs: [
              for (final g in allParamGroups)
                Tab(
                  height: 56,
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(_tabIcons[g.key] ?? Icons.tune, size: 18),
                      const SizedBox(width: 6),
                      Text(
                        S.groupLabel(g.key, g.label, isEn: isEn),
                        style: const TextStyle(fontSize: 13),
                      ),
                    ],
                  ),
                ),
            ],
          ),
        ),
        body: Column(
          children: [
            _StatusBanner(gw: gw, locale: locale),
            Expanded(
              child: TabBarView(
                children: [
                  for (final group in allParamGroups)
                    _GroupView(group: group, gw: gw, isEn: isEn, locale: locale),
                ],
              ),
            ),
          ],
        ),
        floatingActionButton: gw.isDirty
            ? FloatingActionButton.extended(
                onPressed: gw.isSyncing ? null : () => _pushConfig(context, gw, locale),
                icon: gw.isSyncing
                    ? const SizedBox(
                        width: 20, height: 20,
                        child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white),
                      )
                    : const Icon(Icons.cloud_upload_rounded),
                label: Text(locale.tr('推送到机器人', 'Push to Robot')),
                backgroundColor: AppColors.primary,
                foregroundColor: Colors.white,
              )
            : null,
      ),
    );
  }

  Future<void> _pushConfig(BuildContext context, RuntimeConfigGateway gw, LocaleProvider locale) async {
    HapticFeedback.heavyImpact();
    final ok = await gw.pushToServer();
    if (context.mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok
            ? locale.tr('参数已同步到机器人', 'Config synced to robot')
            : locale.tr('同步失败: ${gw.error ?? "未知错误"}', 'Sync failed: ${gw.error ?? "unknown"}')),
        backgroundColor: ok ? AppColors.success : AppColors.error,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ));
    }
  }

  void _showResetDialog(BuildContext context, RuntimeConfigGateway gw, LocaleProvider locale) {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(locale.tr('恢复默认值', 'Reset to Defaults')),
        content: Text(locale.tr(
          '将所有参数重置为出厂默认值。此操作不可撤销。',
          'Reset all parameters to factory defaults. This cannot be undone.',
        )),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx), child: Text(locale.tr('取消', 'Cancel'))),
          TextButton(
            onPressed: () { gw.resetToDefaults(); Navigator.pop(ctx); },
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: Text(locale.tr('确认重置', 'Confirm Reset')),
          ),
        ],
      ),
    );
  }

  void _exportConfig(BuildContext context, RuntimeConfigGateway gw, LocaleProvider locale) {
    final json = gw.config.toJsonString();
    Clipboard.setData(ClipboardData(text: json));
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(locale.tr('配置 JSON 已复制到剪贴板', 'Config JSON copied to clipboard')),
      behavior: SnackBarBehavior.floating,
    ));
  }
}

// ═══════════════════════════════════════════════════════════════
//  Language Toggle Button — ZH / EN
// ═══════════════════════════════════════════════════════════════

class _LanguageToggle extends StatelessWidget {
  final LocaleProvider locale;
  const _LanguageToggle({required this.locale});

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return Padding(
      padding: const EdgeInsets.only(right: 4),
      child: GestureDetector(
        onTap: () {
          HapticFeedback.selectionClick();
          locale.toggle();
        },
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          decoration: BoxDecoration(
            color: AppColors.primary.withValues(alpha: dark ? 0.15 : 0.08),
            borderRadius: BorderRadius.circular(6),
            border: Border.all(color: AppColors.primary.withValues(alpha: 0.25)),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.language_rounded, size: 16, color: AppColors.primary),
              const SizedBox(width: 4),
              Text(
                locale.isZh ? 'EN' : '中',
                style: const TextStyle(
                  fontSize: 12, fontWeight: FontWeight.w700, color: AppColors.primary,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Status Banner
// ═══════════════════════════════════════════════════════════════

class _StatusBanner extends StatelessWidget {
  final RuntimeConfigGateway gw;
  final LocaleProvider locale;
  const _StatusBanner({required this.gw, required this.locale});

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    String text;
    Color color;
    IconData icon;

    if (gw.error != null) {
      text = gw.error!;
      color = AppColors.error;
      icon = Icons.error_outline_rounded;
    } else if (!gw.isOnline) {
      text = locale.tr('未连接机器人', 'Robot not connected');
      color = context.subtitleColor;
      icon = Icons.cloud_off_rounded;
    } else if (gw.isDirty) {
      text = locale.tr('有未同步的修改', 'Unsaved changes');
      color = AppColors.warning;
      icon = Icons.cloud_off_rounded;
    } else {
      text = locale.tr('已同步 (v${gw.configVersion})', 'Synced (v${gw.configVersion})');
      color = AppColors.success;
      icon = Icons.cloud_done_rounded;
    }

    return Container(
      margin: const EdgeInsets.fromLTRB(16, 8, 16, 4),
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        color: color.withValues(alpha: dark ? 0.12 : 0.08),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: color.withValues(alpha: 0.2)),
      ),
      child: Row(children: [
        Icon(icon, size: 18, color: color),
        const SizedBox(width: 8),
        Expanded(
          child: Text(text,
            style: TextStyle(fontSize: 12, fontWeight: FontWeight.w500, color: color),
          ),
        ),
      ]),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Group View — one tab's content, with basic/advanced split
// ═══════════════════════════════════════════════════════════════

class _GroupView extends StatefulWidget {
  final ParamGroup group;
  final RuntimeConfigGateway gw;
  final bool isEn;
  final LocaleProvider locale;

  const _GroupView({required this.group, required this.gw, required this.isEn, required this.locale});

  @override
  State<_GroupView> createState() => _GroupViewState();
}

class _GroupViewState extends State<_GroupView> with AutomaticKeepAliveClientMixin {
  bool _showAdvanced = false;

  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);

    final basicParams = widget.group.params.where((p) => !p.isAdvanced).toList();
    final advancedParams = widget.group.params.where((p) => p.isAdvanced).toList();
    final basicToggles = widget.group.toggles.where((t) => !t.isAdvanced).toList();
    final advancedToggles = widget.group.toggles.where((t) => t.isAdvanced).toList();

    final hasAdvanced = advancedParams.isNotEmpty || advancedToggles.isNotEmpty;

    return ListView(
      physics: const BouncingScrollPhysics(),
      padding: const EdgeInsets.fromLTRB(16, 12, 16, 100),
      children: [
        if (basicParams.isNotEmpty || basicToggles.isNotEmpty)
          _buildCard(context, basicParams, basicToggles),

        if (hasAdvanced) ...[
          const SizedBox(height: 12),
          _AdvancedHeader(
            expanded: _showAdvanced,
            count: advancedParams.length + advancedToggles.length,
            isEn: widget.isEn,
            onTap: () => setState(() => _showAdvanced = !_showAdvanced),
          ),
          if (_showAdvanced) ...[
            const SizedBox(height: 8),
            _buildCard(context, advancedParams, advancedToggles),
          ],
        ],
      ],
    );
  }

  Widget _buildCard(BuildContext context, List<ParamConstraint> params, List<BoolParam> toggles) {
    return Container(
      decoration: context.cardDecoration,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(AppRadius.card),
        child: Column(
          children: [
            for (int i = 0; i < params.length; i++) ...[
              _buildParamTile(params[i]),
              if (i < params.length - 1 || toggles.isNotEmpty)
                Divider(height: 0.5, indent: 16, endIndent: 16, color: context.dividerColor),
            ],
            for (int i = 0; i < toggles.length; i++) ...[
              _ToggleTile(
                param: toggles[i],
                value: widget.gw.getParamValue(toggles[i].fieldName) as bool? ?? false,
                onChanged: (v) => widget.gw.updateParam(toggles[i].fieldName, v),
                isEn: widget.isEn,
              ),
              if (i < toggles.length - 1)
                Divider(height: 0.5, indent: 16, endIndent: 16, color: context.dividerColor),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildParamTile(ParamConstraint c) {
    final value = _getDouble(widget.gw, c.fieldName);
    return switch (c.inputType) {
      ParamInputType.slider => _SliderParamTile(
        constraint: c, value: value, isEn: widget.isEn, locale: widget.locale,
        onChanged: (v) => widget.gw.updateParam(c.fieldName, v),
      ),
      ParamInputType.stepper => _StepperParamTile(
        constraint: c, value: value, isEn: widget.isEn, locale: widget.locale,
        onChanged: (v) => widget.gw.updateParam(c.fieldName, v),
      ),
      ParamInputType.field => _FieldParamTile(
        constraint: c, value: value, isEn: widget.isEn,
        onChanged: (v) => widget.gw.updateParam(c.fieldName, v),
      ),
    };
  }

  double _getDouble(RuntimeConfigGateway gw, String fieldName) {
    final v = gw.getParamValue(fieldName);
    if (v is num) return v.toDouble();
    return 0.0;
  }
}

// ═══════════════════════════════════════════════════════════════
//  Advanced Section Header
// ═══════════════════════════════════════════════════════════════

class _AdvancedHeader extends StatelessWidget {
  final bool expanded;
  final int count;
  final bool isEn;
  final VoidCallback onTap;

  const _AdvancedHeader({required this.expanded, required this.count, required this.isEn, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 6),
        child: Row(children: [
          Icon(
            expanded ? Icons.expand_less_rounded : Icons.expand_more_rounded,
            size: 20, color: context.subtitleColor,
          ),
          const SizedBox(width: 4),
          Text(
            expanded
                ? (isEn ? 'Hide Advanced' : '收起高级参数')
                : (isEn ? 'Show Advanced ($count)' : '展开高级参数 ($count)'),
            style: TextStyle(fontSize: 12, fontWeight: FontWeight.w500, color: context.subtitleColor),
          ),
          const Spacer(),
          if (!expanded)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              decoration: BoxDecoration(
                color: context.hintColor.withValues(alpha: 0.08),
                borderRadius: BorderRadius.circular(4),
              ),
              child: Text(
                isEn ? '$count items' : '$count 项',
                style: TextStyle(fontSize: 10, color: context.hintColor),
              ),
            ),
        ]),
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  1. Slider Tile
// ═══════════════════════════════════════════════════════════════

class _SliderParamTile extends StatelessWidget {
  final ParamConstraint constraint;
  final double value;
  final ValueChanged<double> onChanged;
  final bool isEn;
  final LocaleProvider locale;

  const _SliderParamTile({
    required this.constraint, required this.value, required this.onChanged,
    required this.isEn, required this.locale,
  });

  bool get _isInt => constraint.step >= 1;
  String _fmtValue(double v) => _isInt ? v.round().toString() : v.toStringAsFixed(2);
  String get _name => S.paramName(constraint.fieldName, constraint.displayName, isEn: isEn);

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    final divisions = constraint.step > 0
        ? ((constraint.maxValue - constraint.minValue) / constraint.step).round()
        : 20;

    return Padding(
      padding: const EdgeInsets.fromLTRB(16, 10, 16, 6),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Expanded(
              child: Text(_name,
                style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500, color: context.titleColor),
              ),
            ),
            GestureDetector(
              onTap: () => _showNumberDialog(context, constraint, value, onChanged, isEn: isEn, locale: locale),
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                decoration: BoxDecoration(
                  color: AppColors.primary.withValues(alpha: dark ? 0.15 : 0.08),
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: AppColors.primary.withValues(alpha: 0.2)),
                ),
                child: Row(mainAxisSize: MainAxisSize.min, children: [
                  Text('${_fmtValue(value)} ${constraint.unit}',
                    style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w700, color: AppColors.primary),
                  ),
                  const SizedBox(width: 4),
                  Icon(Icons.edit_rounded, size: 12, color: AppColors.primary.withValues(alpha: 0.6)),
                ]),
              ),
            ),
          ]),
          SliderTheme(
            data: SliderThemeData(
              trackHeight: 3,
              thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 6),
              overlayShape: const RoundSliderOverlayShape(overlayRadius: 14),
              activeTrackColor: AppColors.primary,
              inactiveTrackColor: dark ? Colors.white.withValues(alpha: 0.08) : const Color(0xFFE8E5F5),
              thumbColor: Colors.white,
              overlayColor: AppColors.primary.withValues(alpha: 0.1),
            ),
            child: Slider(
              value: value.clamp(constraint.minValue, constraint.maxValue),
              min: constraint.minValue,
              max: constraint.maxValue,
              divisions: divisions > 0 ? divisions : 20,
              onChanged: (v) { HapticFeedback.selectionClick(); onChanged(v); },
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 2),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text(_isInt ? constraint.minValue.round().toString() : constraint.minValue.toStringAsFixed(1),
                  style: TextStyle(fontSize: 9, color: context.hintColor)),
                Text(_isInt ? constraint.maxValue.round().toString() : constraint.maxValue.toStringAsFixed(1),
                  style: TextStyle(fontSize: 9, color: context.hintColor)),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  2. Stepper Tile
// ═══════════════════════════════════════════════════════════════

class _StepperParamTile extends StatelessWidget {
  final ParamConstraint constraint;
  final double value;
  final ValueChanged<double> onChanged;
  final bool isEn;
  final LocaleProvider locale;

  const _StepperParamTile({
    required this.constraint, required this.value, required this.onChanged,
    required this.isEn, required this.locale,
  });

  bool get _isInt => constraint.step >= 1;
  String get _displayValue => _isInt ? value.round().toString() : value.toStringAsFixed(1);
  double get _step => constraint.step > 0 ? constraint.step : 1;
  String get _name => S.paramName(constraint.fieldName, constraint.displayName, isEn: isEn);

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    final clamped = value.clamp(constraint.minValue, constraint.maxValue);
    final canDec = clamped > constraint.minValue;
    final canInc = clamped < constraint.maxValue;

    return InkWell(
      onTap: () => _showNumberDialog(context, constraint, value, onChanged, isEn: isEn, locale: locale),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        child: Row(children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(_name,
                  style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500, color: context.titleColor),
                ),
                const SizedBox(height: 2),
                Text(
                  '${constraint.minValue.round()}${constraint.unit.isNotEmpty ? " ${constraint.unit}" : ""}'
                  ' ~ ${constraint.maxValue.round()}${constraint.unit.isNotEmpty ? " ${constraint.unit}" : ""}',
                  style: TextStyle(fontSize: 11, color: context.hintColor),
                ),
              ],
            ),
          ),
          _StepButton(
            icon: Icons.remove_rounded, enabled: canDec, dark: dark,
            onTap: () {
              HapticFeedback.selectionClick();
              onChanged((clamped - _step).clamp(constraint.minValue, constraint.maxValue));
            },
          ),
          GestureDetector(
            onTap: () => _showNumberDialog(context, constraint, value, onChanged, isEn: isEn, locale: locale),
            child: Container(
              constraints: const BoxConstraints(minWidth: 56),
              alignment: Alignment.center,
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
              decoration: BoxDecoration(
                color: dark ? Colors.white.withValues(alpha: 0.05) : const Color(0xFFF5F3FA),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Text(
                '${_displayValue}${constraint.unit.isNotEmpty ? " ${constraint.unit}" : ""}',
                style: TextStyle(fontSize: 14, fontWeight: FontWeight.w700, color: context.titleColor),
              ),
            ),
          ),
          _StepButton(
            icon: Icons.add_rounded, enabled: canInc, dark: dark,
            onTap: () {
              HapticFeedback.selectionClick();
              onChanged((clamped + _step).clamp(constraint.minValue, constraint.maxValue));
            },
          ),
        ]),
      ),
    );
  }
}

class _StepButton extends StatelessWidget {
  final IconData icon;
  final bool enabled;
  final bool dark;
  final VoidCallback onTap;

  const _StepButton({required this.icon, required this.enabled, required this.dark, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return Material(
      color: Colors.transparent,
      child: InkWell(
        borderRadius: BorderRadius.circular(8),
        onTap: enabled ? onTap : null,
        child: Container(
          width: 36, height: 36,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(8),
            color: enabled
                ? AppColors.primary.withValues(alpha: dark ? 0.12 : 0.08)
                : (dark ? Colors.white.withValues(alpha: 0.03) : Colors.grey.withValues(alpha: 0.05)),
          ),
          child: Icon(icon, size: 18,
            color: enabled ? AppColors.primary : (dark ? Colors.white24 : Colors.grey.shade300),
          ),
        ),
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  3. Field Tile
// ═══════════════════════════════════════════════════════════════

class _FieldParamTile extends StatefulWidget {
  final ParamConstraint constraint;
  final double value;
  final ValueChanged<double> onChanged;
  final bool isEn;

  const _FieldParamTile({
    required this.constraint, required this.value, required this.onChanged, required this.isEn,
  });

  @override
  State<_FieldParamTile> createState() => _FieldParamTileState();
}

class _FieldParamTileState extends State<_FieldParamTile> {
  late TextEditingController _controller;
  late FocusNode _focusNode;
  bool _editing = false;

  bool get _isInt => widget.constraint.step >= 1;
  String _fmtValue(double v) => _isInt ? v.round().toString() : v.toStringAsFixed(2);
  String get _name => S.paramName(widget.constraint.fieldName, widget.constraint.displayName, isEn: widget.isEn);

  @override
  void initState() {
    super.initState();
    _controller = TextEditingController(text: _fmtValue(widget.value));
    _focusNode = FocusNode();
    _focusNode.addListener(() {
      if (!_focusNode.hasFocus && _editing) _submit();
      setState(() => _editing = _focusNode.hasFocus);
    });
  }

  @override
  void didUpdateWidget(covariant _FieldParamTile old) {
    super.didUpdateWidget(old);
    if (!_editing && old.value != widget.value) {
      _controller.text = _fmtValue(widget.value);
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    _focusNode.dispose();
    super.dispose();
  }

  void _submit() {
    final parsed = double.tryParse(_controller.text.trim());
    if (parsed != null) {
      final clamped = parsed.clamp(widget.constraint.minValue, widget.constraint.maxValue);
      widget.onChanged(clamped);
      _controller.text = _fmtValue(clamped);
    } else {
      _controller.text = _fmtValue(widget.value);
    }
    setState(() => _editing = false);
  }

  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    final c = widget.constraint;

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      child: Row(children: [
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(_name,
                style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500, color: context.titleColor),
              ),
              const SizedBox(height: 2),
              Text(
                '${_isInt ? c.minValue.round() : c.minValue} ~ ${_isInt ? c.maxValue.round() : c.maxValue}'
                '${c.unit.isNotEmpty ? " ${c.unit}" : ""}',
                style: TextStyle(fontSize: 11, color: context.hintColor),
              ),
            ],
          ),
        ),
        SizedBox(
          width: 100, height: 38,
          child: TextField(
            controller: _controller,
            focusNode: _focusNode,
            textAlign: TextAlign.center,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            style: TextStyle(
              fontSize: 14, fontWeight: FontWeight.w700,
              color: _editing ? AppColors.primary : context.titleColor,
            ),
            decoration: InputDecoration(
              suffixText: c.unit,
              suffixStyle: TextStyle(fontSize: 12, color: context.hintColor),
              filled: true,
              fillColor: _editing
                  ? AppColors.primary.withValues(alpha: 0.08)
                  : (dark ? Colors.white.withValues(alpha: 0.05) : const Color(0xFFF5F3FA)),
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: _editing
                    ? const BorderSide(color: AppColors.primary, width: 1.5)
                    : BorderSide.none,
              ),
              enabledBorder: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8), borderSide: BorderSide.none,
              ),
              focusedBorder: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: const BorderSide(color: AppColors.primary, width: 1.5),
              ),
              contentPadding: const EdgeInsets.symmetric(horizontal: 8, vertical: 8),
              isDense: true,
            ),
            onSubmitted: (_) => _submit(),
          ),
        ),
      ]),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Toggle Tile
// ═══════════════════════════════════════════════════════════════

class _ToggleTile extends StatelessWidget {
  final BoolParam param;
  final bool value;
  final ValueChanged<bool> onChanged;
  final bool isEn;

  const _ToggleTile({required this.param, required this.value, required this.onChanged, required this.isEn});

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: () { HapticFeedback.selectionClick(); onChanged(!value); },
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        child: Row(children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  S.toggleName(param.fieldName, param.displayName, isEn: isEn),
                  style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500, color: context.titleColor),
                ),
                if (param.subtitle != null)
                  Text(
                    S.toggleSubtitle(param.fieldName, param.subtitle, isEn: isEn) ?? '',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor),
                  ),
              ],
            ),
          ),
          Switch(
            value: value,
            onChanged: (v) { HapticFeedback.selectionClick(); onChanged(v); },
          ),
        ]),
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Shared: Number input dialog
// ═══════════════════════════════════════════════════════════════

void _showNumberDialog(
  BuildContext context,
  ParamConstraint constraint,
  double currentValue,
  ValueChanged<double> onChanged, {
  required bool isEn,
  required LocaleProvider locale,
}) {
  final isInt = constraint.step >= 1;
  String fmtValue(double v) => isInt ? v.round().toString() : v.toStringAsFixed(2);
  final name = S.paramName(constraint.fieldName, constraint.displayName, isEn: isEn);

  final controller = TextEditingController(text: fmtValue(currentValue));

  showDialog(
    context: context,
    builder: (ctx) => AlertDialog(
      title: Text(name),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            '${locale.tr("范围", "Range")}: ${isInt ? constraint.minValue.round() : constraint.minValue} '
            '~ ${isInt ? constraint.maxValue.round() : constraint.maxValue} ${constraint.unit}'
            '${constraint.step > 0 ? "  (${locale.tr("步长", "step")} ${isInt ? constraint.step.round() : constraint.step})" : ""}',
            style: TextStyle(fontSize: 12, color: Theme.of(ctx).hintColor),
          ),
          const SizedBox(height: 12),
          TextField(
            controller: controller,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: InputDecoration(
              suffixText: constraint.unit,
              border: const OutlineInputBorder(),
              contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
            ),
            autofocus: true,
            onSubmitted: (s) {
              final parsed = double.tryParse(s.trim());
              if (parsed != null) {
                HapticFeedback.mediumImpact();
                onChanged(parsed.clamp(constraint.minValue, constraint.maxValue));
              }
              Navigator.pop(ctx);
            },
          ),
        ],
      ),
      actions: [
        TextButton(onPressed: () => Navigator.pop(ctx), child: Text(locale.tr('取消', 'Cancel'))),
        FilledButton(
          onPressed: () {
            final parsed = double.tryParse(controller.text.trim());
            if (parsed != null) {
              HapticFeedback.mediumImpact();
              onChanged(parsed.clamp(constraint.minValue, constraint.maxValue));
            }
            Navigator.pop(ctx);
          },
          child: Text(locale.tr('确认', 'OK')),
        ),
      ],
    ),
  );
}
