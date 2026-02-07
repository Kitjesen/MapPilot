import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/core/services/alert_monitor_service.dart';
import 'package:flutter_monitor/features/settings/alert_history_page.dart';
import 'package:flutter_monitor/shared/widgets/feature_card.dart';

class AlertSettingsPage extends StatelessWidget {
  const AlertSettingsPage({super.key});

  @override
  Widget build(BuildContext context) {
    final prefs = context.watch<SettingsPreferences>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('告警与提醒'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.only(top: 8, bottom: 40),
        children: [
          // ===== 提醒方式 =====
          SettingsSection(
            title: '提醒方式',
            children: [
              SettingsTile(
                icon: Icons.notifications_active_outlined,
                iconColor: AppColors.primary,
                title: 'APP 内提醒',
                subtitle: '在 APP 内显示告警横幅',
                trailing: Switch(
                  value: prefs.alertInApp,
                  onChanged: (v) => prefs.setAlertInApp(v),
                ),
              ),
              SettingsTile(
                icon: Icons.circle_notifications_outlined,
                iconColor: AppColors.secondary,
                title: '系统通知',
                subtitle: '允许推送系统级通知（需授权）',
                trailing: Switch(
                  value: prefs.alertSystem,
                  onChanged: (v) => prefs.setAlertSystem(v),
                ),
              ),
            ],
          ),

          // ===== 告警类型 =====
          SettingsSection(
            title: '告警类型',
            children: [
              SettingsTile(
                icon: Icons.battery_alert_outlined,
                iconColor: AppColors.error,
                title: '电量低',
                subtitle: '电池电量低于 20% 时告警',
                trailing: Switch(
                  value: prefs.alertBatteryLow,
                  onChanged: (v) => prefs.setAlertBatteryLow(v),
                ),
              ),
              SettingsTile(
                icon: Icons.thermostat,
                iconColor: AppColors.warning,
                title: '温度过高',
                subtitle: 'CPU 温度超过 70°C 时告警',
                trailing: Switch(
                  value: prefs.alertTempHigh,
                  onChanged: (v) => prefs.setAlertTempHigh(v),
                ),
              ),
              SettingsTile(
                icon: Icons.signal_wifi_off,
                iconColor: AppColors.error,
                title: '通信异常',
                subtitle: '与机器人失去连接时告警',
                trailing: Switch(
                  value: prefs.alertCommLost,
                  onChanged: (v) => prefs.setAlertCommLost(v),
                ),
              ),
            ],
          ),

          // ===== 历史记录 =====
          SettingsSection(
            title: '历史记录',
            children: [
              SettingsTile(
                icon: Icons.history,
                iconColor: AppColors.info,
                title: '告警历史',
                subtitle: '${context.watch<AlertMonitorService>().history.length} 条记录',
                trailing: const Icon(Icons.chevron_right, size: 20),
                onTap: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                        builder: (_) => const AlertHistoryPage()),
                  );
                },
              ),
            ],
          ),

          // ===== 说明 =====
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
            child: Text(
              '告警基于机器人状态数据实时判断。APP 内提醒不需要额外权限；'
              '系统通知需要在系统设置中允许本 APP 的通知权限。',
              style: TextStyle(
                fontSize: 12,
                color: context.subtitleColor,
                height: 1.5,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
