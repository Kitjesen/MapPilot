import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:flutter_monitor/core/models/scheduled_task.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/shared/widgets/feature_card.dart';

/// 定时任务管理页面 — 创建/编辑/删除定时巡逻任务。
///
/// TODO: 后续接 flutter_local_notifications 实现真实后台调度。
class ScheduledTasksPage extends StatelessWidget {
  const ScheduledTasksPage({super.key});

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    final prefs = context.watch<SettingsPreferences>();
    final tasks = prefs.scheduledTasks;

    return Scaffold(
      appBar: AppBar(
        title: Text(locale.tr('定时任务', 'Scheduled Tasks')),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          IconButton(
            icon: const Icon(Icons.add, size: 22),
            tooltip: locale.tr('添加定时任务', 'Add scheduled task'),
            onPressed: () => _showCreateDialog(context),
          ),
        ],
      ),
      body: tasks.isEmpty
          ? _buildEmpty(context, locale)
          : _buildList(context, locale, prefs, tasks),
    );
  }

  Widget _buildEmpty(BuildContext context, LocaleProvider locale) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            Icons.schedule_outlined,
            size: 64,
            color: context.subtitleColor.withValues(alpha: 0.3),
          ),
          const SizedBox(height: 16),
          Text(
            locale.tr('暂无定时任务', 'No scheduled tasks'),
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.w600,
              color: context.subtitleColor,
            ),
          ),
          const SizedBox(height: 8),
          Text(
            locale.tr('点击右上角 + 添加', 'Tap + to add one'),
            style: TextStyle(
              fontSize: 13,
              color: context.subtitleColor.withValues(alpha: 0.7),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildList(
    BuildContext context,
    LocaleProvider locale,
    SettingsPreferences prefs,
    List<ScheduledTask> tasks,
  ) {
    return ListView(
      physics: const BouncingScrollPhysics(),
      padding: const EdgeInsets.only(top: 8, bottom: 40),
      children: [
        // Hint
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 8),
          child: Text(
            locale.tr(
              '调度器将在指定时间发送任务启动通知（App 在前台时自动执行）',
              'The scheduler sends task-start notifications at the specified time (auto-executes when app is in foreground)',
            ),
            style: TextStyle(fontSize: 12, color: context.subtitleColor),
          ),
        ),
        SettingsSection(
          children: tasks.map((task) {
            return Dismissible(
              key: ValueKey(task.id),
              direction: DismissDirection.endToStart,
              background: Container(
                alignment: Alignment.centerRight,
                padding: const EdgeInsets.only(right: 20),
                color: AppColors.error,
                child: const Icon(Icons.delete_outline,
                    color: Colors.white, size: 22),
              ),
              confirmDismiss: (_) => _confirmDelete(context, locale),
              onDismissed: (_) => prefs.deleteScheduledTask(task.id),
              child: _ScheduledTaskTile(task: task),
            );
          }).toList(),
        ),
      ],
    );
  }

  Future<bool?> _confirmDelete(
      BuildContext context, LocaleProvider locale) async {
    return showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(locale.tr('删除定时任务', 'Delete Scheduled Task')),
        content: Text(locale.tr('确定要删除此定时任务吗？', 'Delete this scheduled task?')),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: Text(locale.tr('取消', 'Cancel')),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: Text(locale.tr('删除', 'Delete')),
          ),
        ],
      ),
    );
  }

  void _showCreateDialog(BuildContext context) {
    showDialog(
      context: context,
      builder: (ctx) => const _CreateScheduledTaskDialog(),
    );
  }
}

// ---------------------------------------------------------------------------
// Tile for a single scheduled task
// ---------------------------------------------------------------------------

class _ScheduledTaskTile extends StatelessWidget {
  final ScheduledTask task;
  const _ScheduledTaskTile({required this.task});

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    final prefs = context.watch<SettingsPreferences>();

    return SettingsTile(
      icon: Icons.schedule_outlined,
      title: task.templateName,
      subtitle: '${_weekdaysLabel(task.weekdays, locale)}  '
          '${task.time.hour.toString().padLeft(2, '0')}:'
          '${task.time.minute.toString().padLeft(2, '0')}',
      trailing: Switch(
        value: task.enabled,
        onChanged: (v) => prefs.toggleScheduledTask(task.id, v),
      ),
    );
  }

  String _weekdaysLabel(List<int> weekdays, LocaleProvider locale) {
    if (weekdays.isEmpty) return locale.tr('每天', 'Daily');
    final zhNames = ['日', '一', '二', '三', '四', '五', '六'];
    final enNames = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];
    final names = locale.locale == AppLocale.zh ? zhNames : enNames;
    final sorted = List<int>.from(weekdays)..sort();
    return sorted.map((d) => names[d % 7]).join(locale.locale == AppLocale.zh ? ' ' : ' ');
  }
}

// ---------------------------------------------------------------------------
// Create dialog
// ---------------------------------------------------------------------------

class _CreateScheduledTaskDialog extends StatefulWidget {
  const _CreateScheduledTaskDialog();

  @override
  State<_CreateScheduledTaskDialog> createState() =>
      _CreateScheduledTaskDialogState();
}

class _CreateScheduledTaskDialogState
    extends State<_CreateScheduledTaskDialog> {
  String? _selectedTemplateName;
  TimeOfDay _selectedTime = TimeOfDay.now();
  final Set<int> _selectedWeekdays = {};

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    final prefs = context.watch<SettingsPreferences>();
    final templates = prefs.taskTemplates;

    return AlertDialog(
      title: Text(locale.tr('新建定时任务', 'New Scheduled Task')),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
      content: SizedBox(
        width: 320,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Template selector
            Text(
              locale.tr('选择模板', 'Select Template'),
              style: TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.w600,
                color: context.subtitleColor,
              ),
            ),
            const SizedBox(height: 6),
            if (templates.isEmpty)
              Text(
                locale.tr('暂无模板，请先在任务面板保存模板',
                    'No templates yet. Save one from the task panel first.'),
                style: TextStyle(fontSize: 13, color: context.subtitleColor),
              )
            else
              InputDecorator(
                decoration: InputDecoration(
                  contentPadding:
                      const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
                  border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(10),
                  ),
                ),
                child: DropdownButtonHideUnderline(
                  child: DropdownButton<String>(
                    value: _selectedTemplateName,
                    isExpanded: true,
                    isDense: true,
                    hint: Text(locale.tr('选择模板', 'Select template')),
                    items: templates
                        .map((t) => DropdownMenuItem(
                              value: t.name,
                              child:
                                  Text(t.name, overflow: TextOverflow.ellipsis),
                            ))
                        .toList(),
                    onChanged: (v) => setState(() => _selectedTemplateName = v),
                  ),
                ),
              ),

            const SizedBox(height: 16),

            // Time picker
            Text(
              locale.tr('执行时间', 'Execution Time'),
              style: TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.w600,
                color: context.subtitleColor,
              ),
            ),
            const SizedBox(height: 6),
            InkWell(
              borderRadius: BorderRadius.circular(10),
              onTap: () async {
                final picked = await showTimePicker(
                  context: context,
                  initialTime: _selectedTime,
                );
                if (picked != null) setState(() => _selectedTime = picked);
              },
              child: Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                decoration: BoxDecoration(
                  borderRadius: BorderRadius.circular(10),
                  border: Border.all(color: context.dividerColor),
                ),
                child: Row(
                  children: [
                    Icon(Icons.access_time,
                        size: 18, color: context.subtitleColor),
                    const SizedBox(width: 8),
                    Text(
                      '${_selectedTime.hour.toString().padLeft(2, '0')}:'
                      '${_selectedTime.minute.toString().padLeft(2, '0')}',
                      style: TextStyle(
                        fontSize: 15,
                        fontWeight: FontWeight.w500,
                        color: context.titleColor,
                      ),
                    ),
                  ],
                ),
              ),
            ),

            const SizedBox(height: 16),

            // Weekday picker
            Text(
              locale.tr('重复 (不选 = 每天)', 'Repeat (none = daily)'),
              style: TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.w600,
                color: context.subtitleColor,
              ),
            ),
            const SizedBox(height: 6),
            _buildWeekdayChips(locale),
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: Text(locale.tr('取消', 'Cancel')),
        ),
        TextButton(
          onPressed: _selectedTemplateName == null ? null : _save,
          child: Text(locale.tr('保存', 'Save')),
        ),
      ],
    );
  }

  Widget _buildWeekdayChips(LocaleProvider locale) {
    final zhLabels = ['日', '一', '二', '三', '四', '五', '六'];
    final enLabels = ['S', 'M', 'T', 'W', 'T', 'F', 'S'];
    final labels = locale.locale == AppLocale.zh ? zhLabels : enLabels;

    return Wrap(
      spacing: 6,
      runSpacing: 4,
      children: List.generate(7, (i) {
        final selected = _selectedWeekdays.contains(i);
        return FilterChip(
          label: Text(labels[i]),
          selected: selected,
          onSelected: (v) {
            HapticFeedback.selectionClick();
            setState(() {
              if (v) {
                _selectedWeekdays.add(i);
              } else {
                _selectedWeekdays.remove(i);
              }
            });
          },
          visualDensity: VisualDensity.compact,
          materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
        );
      }),
    );
  }

  void _save() {
    final prefs = context.read<SettingsPreferences>();
    final task = ScheduledTask(
      id: DateTime.now().microsecondsSinceEpoch.toRadixString(36),
      templateName: _selectedTemplateName!,
      weekdays: _selectedWeekdays.toList()..sort(),
      time: _selectedTime,
      createdAt: DateTime.now().toIso8601String(),
    );
    prefs.saveScheduledTask(task);
    Navigator.pop(context);
  }
}
