import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:path_provider/path_provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/shared/widgets/feature_card.dart';
import 'package:flutter_monitor/features/settings/saved_devices_page.dart';
import 'package:flutter_monitor/features/settings/alert_settings_page.dart';
import 'package:flutter_monitor/features/settings/firmware_ota_page.dart';
import 'package:flutter_monitor/features/settings/log_export_page.dart';
import 'package:flutter_monitor/features/settings/support_page.dart';
import 'package:flutter_monitor/features/settings/version_detail_page.dart';
import 'package:flutter_monitor/features/settings/cloud_config_page.dart';
import 'package:flutter_monitor/features/settings/device_info_page.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';

/// Settings screen — every entry is a real, functional feature.
class AppSettingsScreen extends StatelessWidget {
  const AppSettingsScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final themeProvider = context.watch<ThemeProvider>();
    final settingsPrefs = context.watch<SettingsPreferences>();
    final otaGw = context.watch<OtaGateway>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('设置'),
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
          // ==================== 外观 ====================
          SettingsSection(
            title: '外观',
            children: [
              SettingsTile(
                icon: Icons.contrast_outlined,
                title: '深色模式',
                subtitle: _themeModeLabel(themeProvider.mode),
                trailing: _buildThemeSelector(context, themeProvider),
              ),
            ],
          ),

          // ==================== 连接 ====================
          SettingsSection(
            title: '连接',
            children: [
              SettingsTile(
                icon: Icons.devices_outlined,
                title: '已保存的设备',
                subtitle: '${settingsPrefs.savedDevices.length} 个设备',
                trailing: SettingsActionButton(
                  label: '管理',
                  onTap: () => _push(context, const SavedDevicesPage()),
                ),
                onTap: () => _push(context, const SavedDevicesPage()),
              ),
              SettingsTile(
                icon: Icons.timer_outlined,
                title: '连接超时',
                subtitle: '${settingsPrefs.connectionTimeoutSec} 秒',
                onTap: () => _showTimeoutPicker(context, settingsPrefs),
              ),
              SettingsTile(
                icon: Icons.sync_outlined,
                title: '自动重连',
                subtitle: settingsPrefs.autoReconnect
                    ? '断连后自动尝试重连'
                    : '已关闭',
                trailing: Switch(
                  value: settingsPrefs.autoReconnect,
                  onChanged: (v) => settingsPrefs.setAutoReconnect(v),
                ),
              ),
            ],
          ),

          // ==================== 通知 ====================
          SettingsSection(
            title: '通知',
            children: [
              SettingsTile(
                icon: Icons.vibration_outlined,
                title: '操作震动反馈',
                subtitle: settingsPrefs.hapticFeedback
                    ? '控制操作时触觉反馈'
                    : '已关闭',
                trailing: Switch(
                  value: settingsPrefs.hapticFeedback,
                  onChanged: (v) => settingsPrefs.setHapticFeedback(v),
                ),
              ),
              SettingsTile(
                icon: Icons.notifications_outlined,
                title: '异常告警提醒',
                subtitle: '电量/温度/通信异常',
                trailing: SettingsActionButton(
                  label: '配置',
                  onTap: () => _push(context, const AlertSettingsPage()),
                ),
                onTap: () => _push(context, const AlertSettingsPage()),
              ),
            ],
          ),

          // ==================== 设备与固件 ====================
          SettingsSection(
            title: '设备与固件',
            children: [
              SettingsTile(
                icon: Icons.system_update_outlined,
                title: '固件升级 (OTA)',
                subtitle: '查看/上传机器人固件',
                trailing: SettingsActionButton(
                  label: '打开',
                  onTap: () => _push(context, const FirmwareOtaPage()),
                ),
                onTap: () => _push(context, const FirmwareOtaPage()),
              ),
              SettingsTile(
                icon: Icons.cloud_outlined,
                title: '云端更新源',
                subtitle: otaGw.cloud.useCustomUrl
                    ? '自定义 URL'
                    : '${otaGw.cloud.owner}/${otaGw.cloud.repo}',
                trailing: SettingsActionButton(
                  label: '配置',
                  onTap: () => _push(context, const CloudConfigPage()),
                ),
                onTap: () => _push(context, const CloudConfigPage()),
              ),
              SettingsTile(
                icon: Icons.description_outlined,
                title: '导出机器人日志',
                subtitle: '状态/事件/通信日志',
                trailing: SettingsActionButton(
                  label: '导出',
                  onTap: () => _push(context, const LogExportPage()),
                ),
                onTap: () => _push(context, const LogExportPage()),
              ),
            ],
          ),

          // ==================== 数据与存储 ====================
          SettingsSection(
            title: '数据与存储',
            children: [
              _ClearCacheTile(),
            ],
          ),

          // ==================== 关于 ====================
          SettingsSection(
            title: '关于',
            children: [
              SettingsTile(
                icon: Icons.developer_board_outlined,
                title: '设备管理',
                subtitle: '设备信息、系统服务',
                onTap: () => _push(context, const DeviceInfoPage()),
              ),
              SettingsTile(
                icon: Icons.info_outlined,
                title: '版本信息',
                subtitle: 'v${const String.fromEnvironment('APP_VERSION', defaultValue: '1.0.0')}'
                    '${const String.fromEnvironment('BUILD_NUMBER').isNotEmpty ? ' (${const String.fromEnvironment('BUILD_NUMBER')})' : ''}',
                onTap: () => _push(context, const VersionDetailPage()),
              ),
              SettingsTile(
                icon: Icons.article_outlined,
                title: '开源许可',
                subtitle: '第三方库许可证',
                onTap: () => showLicensePage(
                  context: context,
                  applicationName: '大算机器人',
                  applicationVersion: 'v${const String.fromEnvironment('APP_VERSION', defaultValue: '1.0.0')}',
                ),
              ),
              SettingsTile(
                icon: Icons.help_outline,
                title: '反馈与支持',
                subtitle: '技术支持/文档/FAQ',
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

  String _themeModeLabel(ThemeMode mode) {
    switch (mode) {
      case ThemeMode.light:
        return '浅色';
      case ThemeMode.dark:
        return '深色';
      case ThemeMode.system:
        return '跟随系统';
    }
  }

  void _showTimeoutPicker(BuildContext context, SettingsPreferences prefs) {
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
                  '连接超时',
                  style: TextStyle(
                    fontSize: 17,
                    fontWeight: FontWeight.w700,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 8),
                Text(
                  '弱网环境下可调大超时以减少误判',
                  style: TextStyle(fontSize: 13, color: context.subtitleColor),
                ),
                const SizedBox(height: 16),
                ...options.map((sec) {
                  final isSelected = sec == prefs.connectionTimeoutSec;
                  return ListTile(
                    title: Text(
                      '$sec 秒',
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
  String _cacheSize = '计算中...';
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
      if (mounted) setState(() => _cacheSize = '未知');
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
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('清除缓存'),
        content: Text('确定要清除本地缓存数据吗？\n\n当前缓存大小: $_cacheSize\n'
            '（不会清除已保存的设备和设置）'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx, false),
              child: const Text('取消')),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: const Text('清除'),
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
        setState(() => _isClearing = false);
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: const Text('缓存已清除'),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(10)),
          ),
        );
      }
    } catch (e) {
      if (mounted) {
        setState(() => _isClearing = false);
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('清除失败: $e'),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(10)),
          ),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return SettingsTile(
      icon: Icons.delete_outline,
      title: '清除缓存',
      subtitle: _isClearing ? '正在清除...' : _cacheSize,
      trailing: _isClearing
          ? const SizedBox(
              width: 18,
              height: 18,
              child: CircularProgressIndicator(strokeWidth: 1.5),
            )
          : SettingsActionButton(
              label: '清除',
              onTap: _clearCache,
            ),
      onTap: _isClearing ? null : _clearCache,
    );
  }
}
