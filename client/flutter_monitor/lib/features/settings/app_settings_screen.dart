import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:path_provider/path_provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/shared/widgets/feature_card.dart';
import 'package:flutter_monitor/features/connection/splash_screen.dart' show kAppVersion;
import 'package:flutter_monitor/features/settings/saved_devices_page.dart';
import 'package:flutter_monitor/features/settings/alert_settings_page.dart';
import 'package:flutter_monitor/features/settings/firmware_ota_page.dart';
import 'package:flutter_monitor/features/settings/log_export_page.dart';
import 'package:flutter_monitor/features/settings/support_page.dart';
import 'package:flutter_monitor/features/settings/version_detail_page.dart';
import 'package:flutter_monitor/features/settings/cloud_config_page.dart';
import 'package:flutter_monitor/features/settings/device_info_page.dart';
import 'package:flutter_monitor/features/settings/runtime_params_page.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/gateway/runtime_config_gateway.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';

/// Settings screen — every entry is a real, functional feature.
class AppSettingsScreen extends StatelessWidget {
  const AppSettingsScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    final themeProvider = context.watch<ThemeProvider>();
    final settingsPrefs = context.watch<SettingsPreferences>();
    final otaGw = context.watch<OtaGateway>();

    return Scaffold(
      appBar: AppBar(
        title: Text(locale.tr('设置', 'Settings')),
        leading: Navigator.canPop(context)
            ? IconButton(
                icon: const Icon(Icons.arrow_back_ios_new, size: 20),
                onPressed: () => Navigator.pop(context),
              )
            : null,
        automaticallyImplyLeading: false,
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.only(top: 8, bottom: 40),
        children: [
          // ==================== 语言 ====================
          SettingsSection(
            title: locale.tr('语言', 'Language'),
            children: [
              SettingsTile(
                icon: Icons.language_outlined,
                title: locale.tr('界面语言', 'Interface Language'),
                subtitle: locale.tr('中文', 'English'),
                trailing: SegmentedButton<AppLocale>(
                  segments: const [
                    ButtonSegment(value: AppLocale.zh, label: Text('中文')),
                    ButtonSegment(value: AppLocale.en, label: Text('EN')),
                  ],
                  selected: {locale.locale},
                  onSelectionChanged: (v) => locale.setLocale(v.first),
                  style: ButtonStyle(visualDensity: VisualDensity.compact, tapTargetSize: MaterialTapTargetSize.shrinkWrap),
                ),
              ),
            ],
          ),

          // ==================== 外观 ====================
          SettingsSection(
            title: locale.tr('外观', 'Appearance'),
            children: [
              SettingsTile(
                icon: Icons.contrast_outlined,
                title: locale.tr('深色模式', 'Dark Mode'),
                subtitle: _themeModeLabel(themeProvider.mode, locale),
                trailing: _buildThemeSelector(context, themeProvider),
              ),
            ],
          ),

          // ==================== 连接 ====================
          SettingsSection(
            title: locale.tr('连接', 'Connection'),
            children: [
              SettingsTile(
                icon: Icons.devices_outlined,
                title: locale.tr('已保存的设备', 'Saved Devices'),
                subtitle: '${settingsPrefs.savedDevices.length} ${locale.tr('个设备', 'devices')}',
                trailing: SettingsActionButton(
                  label: locale.tr('管理', 'Manage'),
                  onTap: () => _push(context, const SavedDevicesPage()),
                ),
                onTap: () => _push(context, const SavedDevicesPage()),
              ),
              SettingsTile(
                icon: Icons.timer_outlined,
                title: locale.tr('连接超时', 'Connection Timeout'),
                subtitle: '${settingsPrefs.connectionTimeoutSec} ${locale.tr('秒', 'sec')}',
                onTap: () => _showTimeoutPicker(context, settingsPrefs, locale),
              ),
              SettingsTile(
                icon: Icons.sync_outlined,
                title: locale.tr('自动重连', 'Auto Reconnect'),
                subtitle: settingsPrefs.autoReconnect
                    ? locale.tr('断连后自动尝试重连', 'Auto retry after disconnection')
                    : locale.tr('已关闭', 'Disabled'),
                trailing: Switch(
                  value: settingsPrefs.autoReconnect,
                  onChanged: (v) => settingsPrefs.setAutoReconnect(v),
                ),
              ),
            ],
          ),

          // ==================== 运行参数 ====================
          SettingsSection(
            title: locale.tr('机器人参数', 'Robot Parameters'),
            children: [
              Builder(builder: (context) {
                final rcGw = context.watch<RuntimeConfigGateway>();
                return SettingsTile(
                  icon: Icons.tune_rounded,
                  title: locale.tr('运行参数', 'Runtime Parameters'),
                  subtitle: rcGw.isDirty
                      ? locale.tr('有未同步的修改', 'Unsaved changes')
                      : '速度 ${rcGw.config.maxSpeed.toStringAsFixed(1)} m/s '
                        '| 急停 ${rcGw.config.stopDistance.toStringAsFixed(1)} m',
                  trailing: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      if (rcGw.isDirty)
                        Container(
                          width: 8, height: 8,
                          margin: const EdgeInsets.only(right: 8),
                          decoration: const BoxDecoration(
                            color: AppColors.warning,
                            shape: BoxShape.circle,
                          ),
                        ),
                      SettingsActionButton(
                        label: locale.tr('配置', 'Configure'),
                        onTap: () => _push(context, const RuntimeParamsPage()),
                      ),
                    ],
                  ),
                  onTap: () => _push(context, const RuntimeParamsPage()),
                );
              }),
            ],
          ),

          // ==================== 通知 ====================
          SettingsSection(
            title: locale.tr('通知', 'Notifications'),
            children: [
              SettingsTile(
                icon: Icons.vibration_outlined,
                title: locale.tr('操作震动反馈', 'Haptic Feedback'),
                subtitle: settingsPrefs.hapticFeedback
                    ? locale.tr('控制操作时触觉反馈', 'Vibration on control actions')
                    : locale.tr('已关闭', 'Disabled'),
                trailing: Switch(
                  value: settingsPrefs.hapticFeedback,
                  onChanged: (v) => settingsPrefs.setHapticFeedback(v),
                ),
              ),
              SettingsTile(
                icon: Icons.notifications_outlined,
                title: locale.tr('异常告警提醒', 'Alert Notifications'),
                subtitle: locale.tr('电量/温度/通信异常', 'Battery / Temp / Comm alerts'),
                trailing: SettingsActionButton(
                  label: locale.tr('配置', 'Configure'),
                  onTap: () => _push(context, const AlertSettingsPage()),
                ),
                onTap: () => _push(context, const AlertSettingsPage()),
              ),
            ],
          ),

          // ==================== 设备与固件 ====================
          SettingsSection(
            title: locale.tr('设备与固件', 'Device & Firmware'),
            children: [
              SettingsTile(
                icon: Icons.system_update_outlined,
                title: locale.tr('固件升级 (OTA)', 'Firmware Update (OTA)'),
                subtitle: locale.tr('查看/上传机器人固件', 'View / upload robot firmware'),
                trailing: SettingsActionButton(
                  label: locale.tr('打开', 'Open'),
                  onTap: () => _push(context, const FirmwareOtaPage()),
                ),
                onTap: () => _push(context, const FirmwareOtaPage()),
              ),
              SettingsTile(
                icon: Icons.cloud_outlined,
                title: locale.tr('云端更新源', 'Cloud Update Source'),
                subtitle: otaGw.cloud.useCustomUrl
                    ? locale.tr('自定义 URL', 'Custom URL')
                    : '${otaGw.cloud.owner}/${otaGw.cloud.repo}',
                trailing: SettingsActionButton(
                  label: locale.tr('配置', 'Configure'),
                  onTap: () => _push(context, const CloudConfigPage()),
                ),
                onTap: () => _push(context, const CloudConfigPage()),
              ),
              SettingsTile(
                icon: Icons.description_outlined,
                title: locale.tr('导出机器人日志', 'Export Robot Logs'),
                subtitle: locale.tr('状态/事件/通信日志', 'Status / events / comm logs'),
                trailing: SettingsActionButton(
                  label: locale.tr('导出', 'Export'),
                  onTap: () => _push(context, const LogExportPage()),
                ),
                onTap: () => _push(context, const LogExportPage()),
              ),
            ],
          ),

          // ==================== 数据与存储 ====================
          SettingsSection(
            title: locale.tr('数据与存储', 'Data & Storage'),
            children: [
              _ClearCacheTile(),
            ],
          ),

          // ==================== 关于 ====================
          SettingsSection(
            title: locale.tr('关于', 'About'),
            children: [
              // ── 品牌信息卡 ──
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: Container(
                  padding: const EdgeInsets.all(16),
                  decoration: context.softCardDecoration,
                  child: Row(
                    children: [
                      Container(
                        width: 48,
                        height: 48,
                        decoration: BoxDecoration(
                          gradient: AppColors.brandGradient,
                          borderRadius: BorderRadius.circular(14),
                        ),
                        child: const Center(
                          child: Text(
                            'DS',
                            style: TextStyle(
                              fontSize: 20,
                              fontWeight: FontWeight.w900,
                              color: Colors.white,
                              letterSpacing: 1,
                              height: 1,
                            ),
                          ),
                        ),
                      ),
                      const SizedBox(width: 14),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              locale.tr('大算 3D NAV', 'Dasuan 3D NAV'),
                              style: TextStyle(
                                fontSize: 16,
                                fontWeight: FontWeight.w700,
                                color: context.titleColor,
                              ),
                            ),
                            const SizedBox(height: 2),
                            Text(
                              'v$kAppVersion',
                              style: TextStyle(
                                fontSize: 13,
                                color: context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              SettingsTile(
                icon: Icons.developer_board_outlined,
                title: locale.tr('设备管理', 'Device Management'),
                subtitle: locale.tr('设备信息、系统服务', 'Device info, system services'),
                onTap: () => _push(context, const DeviceInfoPage()),
              ),
              SettingsTile(
                icon: Icons.info_outlined,
                title: locale.tr('版本信息', 'Version Info'),
                subtitle: 'v$kAppVersion'
                    '${const String.fromEnvironment('BUILD_NUMBER').isNotEmpty ? ' (${const String.fromEnvironment('BUILD_NUMBER')})' : ''}',
                onTap: () => _push(context, const VersionDetailPage()),
              ),
              SettingsTile(
                icon: Icons.article_outlined,
                title: locale.tr('开源许可', 'Open Source Licenses'),
                subtitle: locale.tr('第三方库许可证', 'Third-party library licenses'),
                onTap: () => showLicensePage(
                  context: context,
                  applicationName: locale.tr('大算 3D NAV', 'Dasuan 3D NAV'),
                  applicationVersion: 'v$kAppVersion',
                ),
              ),
              SettingsTile(
                icon: Icons.help_outline,
                title: locale.tr('反馈与支持', 'Feedback & Support'),
                subtitle: locale.tr('技术支持/文档/FAQ', 'Tech support / Docs / FAQ'),
                onTap: () => _push(context, const SupportPage()),
              ),
            ],
          ),
        ],
      ),
    );
  }

  void _push(BuildContext context, Widget page) {
    Navigator.of(context).push(
      PageRouteBuilder(
        pageBuilder: (_, __, ___) => page,
        transitionsBuilder: (_, a, __, child) {
          return FadeTransition(
            opacity: CurvedAnimation(parent: a, curve: Curves.easeOut),
            child: child,
          );
        },
        transitionDuration: const Duration(milliseconds: 250),
      ),
    );
  }

  Widget _buildThemeSelector(BuildContext context, ThemeProvider provider) {
    return SegmentedButton<ThemeMode>(
      segments: const [
        ButtonSegment(
          value: ThemeMode.light,
          icon: Icon(Icons.light_mode, size: 16),
        ),
        ButtonSegment(
          value: ThemeMode.system,
          icon: Icon(Icons.phone_android, size: 16),
        ),
        ButtonSegment(
          value: ThemeMode.dark,
          icon: Icon(Icons.dark_mode, size: 16),
        ),
      ],
      selected: {provider.mode},
      onSelectionChanged: (modes) {
        HapticFeedback.selectionClick();
        provider.setMode(modes.first);
      },
      style: ButtonStyle(
        visualDensity: VisualDensity.compact,
        tapTargetSize: MaterialTapTargetSize.shrinkWrap,
      ),
    );
  }

  String _themeModeLabel(ThemeMode mode, LocaleProvider locale) {
    switch (mode) {
      case ThemeMode.light:
        return locale.tr('浅色', 'Light');
      case ThemeMode.dark:
        return locale.tr('深色', 'Dark');
      case ThemeMode.system:
        return locale.tr('跟随系统', 'System');
    }
  }

  void _showTimeoutPicker(BuildContext context, SettingsPreferences prefs, LocaleProvider locale) {
    final isDark = context.isDark;
    final options = [3, 5, 10, 15];

    showModalBottomSheet(
      context: context,
      backgroundColor: isDark ? AppColors.darkCard : Colors.white,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (ctx) {
        return SafeArea(
          child: Padding(
            padding: const EdgeInsets.symmetric(vertical: 16),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Container(
                  width: 36,
                  height: 4,
                  decoration: BoxDecoration(
                    color: context.subtitleColor.withValues(alpha:0.3),
                    borderRadius: BorderRadius.circular(2),
                  ),
                ),
                const SizedBox(height: 16),
                Text(
                  locale.tr('连接超时', 'Connection Timeout'),
                  style: TextStyle(
                    fontSize: 17,
                    fontWeight: FontWeight.w700,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 8),
                Text(
                  locale.tr('弱网环境下可调大超时以减少误判', 'Increase timeout for weak networks to reduce false alarms'),
                  style: TextStyle(fontSize: 13, color: context.subtitleColor),
                ),
                const SizedBox(height: 16),
                ...options.map((sec) {
                  final isSelected = sec == prefs.connectionTimeoutSec;
                  return ListTile(
                    title: Text(
                      '$sec ${locale.tr('秒', 'sec')}',
                      style: TextStyle(
                        fontWeight:
                            isSelected ? FontWeight.w700 : FontWeight.normal,
                        color: isSelected
                            ? context.titleColor
                            : (isDark ? Colors.white : Colors.black87),
                      ),
                    ),
                    trailing: isSelected
                        ? Icon(Icons.check_circle,
                            color: context.titleColor, size: 20)
                        : null,
                    onTap: () {
                      HapticFeedback.selectionClick();
                      prefs.setConnectionTimeout(sec);
                      Navigator.pop(ctx);
                    },
                  );
                }),
                const SizedBox(height: 8),
              ],
            ),
          ),
        );
      },
    );
  }
}

/// Real cache clearing tile with actual size calculation.
class _ClearCacheTile extends StatefulWidget {
  @override
  State<_ClearCacheTile> createState() => _ClearCacheTileState();
}

class _ClearCacheTileState extends State<_ClearCacheTile> {
  static const _calculating = '计算中...';
  static const _unknown = '未知';
  String _cacheSize = _calculating;
  bool _isClearing = false;

  @override
  void initState() {
    super.initState();
    _calculateCacheSize();
  }

  Future<void> _calculateCacheSize() async {
    try {
      final cacheDir = await getTemporaryDirectory();
      final size = await _dirSize(cacheDir);
      if (mounted) {
        setState(() => _cacheSize = _formatBytes(size));
      }
    } catch (_) {
      if (mounted) setState(() => _cacheSize = _unknown);
    }
  }

  Future<int> _dirSize(Directory dir) async {
    int total = 0;
    try {
      if (await dir.exists()) {
        await for (final entity in dir.list(recursive: true, followLinks: false)) {
          if (entity is File) {
            total += await entity.length();
          }
        }
      }
    } catch (_) {}
    return total;
  }

  String _formatBytes(int bytes) {
    if (bytes < 1024) return '$bytes B';
    if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
    if (bytes < 1024 * 1024 * 1024) {
      return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
    }
    return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(1)} GB';
  }

  Future<void> _clearCache() async {
    final locale = context.read<LocaleProvider>();
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(locale.tr('清除缓存', 'Clear Cache')),
        content: Text('${locale.tr('确定要清除本地缓存数据吗？', 'Clear local cache data?')}\n\n${locale.tr('当前缓存大小:', 'Current cache size:')} ${_displayCacheSize(locale)}\n'
            '${locale.tr('（不会清除已保存的设备和设置）', '(Saved devices and settings will not be cleared)')}'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx, false),
              child: Text(locale.tr('取消', 'Cancel'))),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: Text(locale.tr('清除', 'Clear')),
          ),
        ],
      ),
    );

    if (confirmed != true || !mounted) return;

    setState(() => _isClearing = true);

    try {
      final cacheDir = await getTemporaryDirectory();
      if (await cacheDir.exists()) {
        await for (final entity
            in cacheDir.list(recursive: false, followLinks: false)) {
          try {
            await entity.delete(recursive: true);
          } catch (_) {}
        }
      }

      // Recalculate
      await _calculateCacheSize();

      if (mounted) {
        final loc = context.read<LocaleProvider>();
        setState(() => _isClearing = false);
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(loc.tr('缓存已清除', 'Cache cleared')),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(10)),
          ),
        );
      }
    } catch (e) {
      if (mounted) {
        final loc = context.read<LocaleProvider>();
        setState(() => _isClearing = false);
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('${loc.tr('清除失败:', 'Clear failed:')} $e'),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(10)),
          ),
        );
      }
    }
  }

  String _displayCacheSize(LocaleProvider locale) {
    if (_cacheSize == _calculating) return locale.tr('计算中...', 'Calculating...');
    if (_cacheSize == _unknown) return locale.tr('未知', 'Unknown');
    return _cacheSize;
  }

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    return SettingsTile(
      icon: Icons.delete_outline,
      title: locale.tr('清除缓存', 'Clear Cache'),
      subtitle: _isClearing ? locale.tr('正在清除...', 'Clearing...') : _displayCacheSize(locale),
      trailing: _isClearing
          ? const SizedBox(
              width: 18,
              height: 18,
              child: CircularProgressIndicator(strokeWidth: 1.5),
            )
          : SettingsActionButton(
              label: locale.tr('清除', 'Clear'),
              onTap: _clearCache,
            ),
      onTap: _isClearing ? null : _clearCache,
    );
  }
}
