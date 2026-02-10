import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/src/data.pb.dart';

/// Page that displays robot device information and manages systemd services.
class DeviceInfoPage extends StatefulWidget {
  const DeviceInfoPage({super.key});

  @override
  State<DeviceInfoPage> createState() => _DeviceInfoPageState();
}

class _DeviceInfoPageState extends State<DeviceInfoPage> {
  DeviceInfoResponse? _info;
  bool _loading = true;
  String? _error;

  @override
  void initState() {
    super.initState();
    _fetchDeviceInfo();
  }

  Future<void> _fetchDeviceInfo() async {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) {
      setState(() {
        _loading = false;
        _error = '未连接到机器人';
      });
      return;
    }

    setState(() {
      _loading = true;
      _error = null;
    });

    try {
      final info = await client.getDeviceInfo();
      if (mounted) setState(() { _info = info; _loading = false; });
    } catch (e) {
      AppLogger.system.error('Failed to fetch device info: $e');
      if (mounted) setState(() { _loading = false; _error = '获取设备信息失败: $e'; });
    }
  }

  Future<void> _manageService(String serviceName, ServiceAction action) async {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;

    HapticFeedback.mediumImpact();

    try {
      final resp = await client.manageService(
        serviceName: serviceName,
        action: action,
      );
      if (mounted) {
        final msg = resp.success
            ? '${_actionLabel(action)} $serviceName 成功'
            : '${_actionLabel(action)} $serviceName 失败: ${resp.message}';
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(msg),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
          ),
        );
        // Refresh to get updated status
        await _fetchDeviceInfo();
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('操作失败: $e'),
            behavior: SnackBarBehavior.floating,
            backgroundColor: AppColors.error,
          ),
        );
      }
    }
  }

  String _actionLabel(ServiceAction action) {
    switch (action) {
      case ServiceAction.SERVICE_ACTION_START: return '启动';
      case ServiceAction.SERVICE_ACTION_STOP: return '停止';
      case ServiceAction.SERVICE_ACTION_RESTART: return '重启';
      default: return '操作';
    }
  }

  String _formatUptime(int seconds) {
    if (seconds < 60) return '${seconds}s';
    if (seconds < 3600) return '${seconds ~/ 60}m ${seconds % 60}s';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h < 24) return '${h}h ${m}m';
    final d = h ~/ 24;
    return '${d}d ${h % 24}h';
  }

  String _formatBytes(int bytes) {
    if (bytes < 1024) return '$bytes B';
    if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
    if (bytes < 1024 * 1024 * 1024) return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
    return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(1)} GB';
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: const Text('设备信息'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh, size: 20),
            onPressed: _fetchDeviceInfo,
          ),
        ],
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator(strokeWidth: 2))
          : _error != null
              ? Center(
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.error_outline, size: 48,
                          color: context.subtitleColor),
                      const SizedBox(height: 12),
                      Text(_error!,
                          style: TextStyle(
                              fontSize: 14, color: context.subtitleColor)),
                      const SizedBox(height: 16),
                      TextButton(
                        onPressed: _fetchDeviceInfo,
                        child: const Text('重试'),
                      ),
                    ],
                  ),
                )
              : RefreshIndicator(
                  onRefresh: _fetchDeviceInfo,
                  child: ListView(
                    physics: const AlwaysScrollableScrollPhysics(),
                    padding: const EdgeInsets.all(20),
                    children: [
                      _buildDeviceHeader(isDark),
                      const SizedBox(height: 20),
                      _buildInfoSection(isDark),
                      const SizedBox(height: 20),
                      _buildDiskSection(isDark),
                      const SizedBox(height: 20),
                      if (_info!.services.isNotEmpty) ...[
                        _buildServicesSection(isDark),
                        const SizedBox(height: 20),
                      ],
                    ],
                  ),
                ),
    );
  }

  Widget _buildDeviceHeader(bool isDark) {
    final info = _info!;
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        children: [
          Icon(Icons.developer_board, size: 36, color: AppColors.primary),
          const SizedBox(height: 12),
          Text(
            info.hostname.isNotEmpty ? info.hostname : 'Robot',
            style: TextStyle(
              fontSize: 18, fontWeight: FontWeight.w700,
              color: context.titleColor,
            ),
          ),
          if (info.robotId.isNotEmpty) ...[
            const SizedBox(height: 4),
            Text(
              info.robotId,
              style: TextStyle(fontSize: 12, color: context.subtitleColor),
            ),
          ],
          const SizedBox(height: 12),
          Wrap(
            spacing: 6,
            runSpacing: 6,
            alignment: WrapAlignment.center,
            children: info.ipAddresses.map((ip) => Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                color: AppColors.primary.withValues(alpha: isDark ? 0.15 : 0.08),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Text(ip,
                  style: const TextStyle(fontSize: 12, fontFamily: 'monospace',
                      color: AppColors.primary)),
            )).toList(),
          ),
        ],
      ),
    );
  }

  Widget _buildInfoSection(bool isDark) {
    final info = _info!;
    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        children: [
          _buildInfoRow('硬件 ID', info.hwId.isNotEmpty ? info.hwId : 'N/A'),
          Divider(height: 0.5, indent: 16, color: context.dividerColor),
          _buildInfoRow('操作系统', info.osVersion.isNotEmpty ? info.osVersion : 'N/A'),
          Divider(height: 0.5, indent: 16, color: context.dividerColor),
          _buildInfoRow('OTA Daemon', info.otaDaemonVersion.isNotEmpty ? info.otaDaemonVersion : 'N/A'),
          Divider(height: 0.5, indent: 16, color: context.dividerColor),
          _buildInfoRow('运行时间', _formatUptime(info.uptimeSeconds.toInt())),
          if (info.batteryPercent >= 0) ...[
            Divider(height: 0.5, indent: 16, color: context.dividerColor),
            _buildInfoRow('电池电量', '${info.batteryPercent}%'),
          ],
        ],
      ),
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
      child: Row(
        children: [
          Expanded(
            flex: 2,
            child: Text(label,
                style: TextStyle(fontSize: 14, color: context.subtitleColor)),
          ),
          Expanded(
            flex: 3,
            child: Text(value,
                textAlign: TextAlign.end,
                style: TextStyle(
                    fontSize: 14, fontWeight: FontWeight.w600,
                    color: context.titleColor)),
          ),
        ],
      ),
    );
  }

  Widget _buildDiskSection(bool isDark) {
    final info = _info!;
    final total = info.diskTotalBytes.toInt();
    final free = info.diskFreeBytes.toInt();
    final used = total - free;
    final ratio = total > 0 ? used / total : 0.0;

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('磁盘空间',
              style: TextStyle(
                  fontSize: 13, fontWeight: FontWeight.w600,
                  color: context.subtitleColor)),
          const SizedBox(height: 12),
          Row(
            children: [
              Text('${_formatBytes(used)} / ${_formatBytes(total)}',
                  style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600,
                      color: context.titleColor)),
              const Spacer(),
              Text('${_formatBytes(free)} 可用',
                  style: TextStyle(fontSize: 12, color: context.subtitleColor)),
            ],
          ),
          const SizedBox(height: 8),
          ClipRRect(
            borderRadius: BorderRadius.circular(4),
            child: LinearProgressIndicator(
              value: ratio.clamp(0.0, 1.0),
              minHeight: 6,
              color: ratio > 0.9
                  ? AppColors.error
                  : ratio > 0.7
                      ? AppColors.warning
                      : AppColors.primary,
              backgroundColor: isDark
                  ? Colors.white.withValues(alpha: 0.06)
                  : Colors.black.withValues(alpha: 0.04),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildServicesSection(bool isDark) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Padding(
          padding: const EdgeInsets.only(left: 4, bottom: 8),
          child: Text('系统服务',
              style: TextStyle(
                  fontSize: 13, fontWeight: FontWeight.w600,
                  color: context.subtitleColor)),
        ),
        ...(_info!.services.map((svc) => _buildServiceTile(svc, isDark)).toList()),
      ],
    );
  }

  Widget _buildServiceTile(ServiceStatus svc, bool isDark) {
    Color stateColor() {
      switch (svc.state.toLowerCase()) {
        case 'active': return AppColors.success;
        case 'inactive': return context.subtitleColor;
        case 'failed': return AppColors.error;
        case 'activating': return AppColors.warning;
        default: return context.subtitleColor;
      }
    }

    return Container(
      margin: const EdgeInsets.only(bottom: 8),
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 8, height: 8,
                decoration: BoxDecoration(
                    color: stateColor(), shape: BoxShape.circle),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(svc.name,
                    style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600,
                        color: context.titleColor)),
              ),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                decoration: BoxDecoration(
                  color: stateColor().withValues(alpha: 0.12),
                  borderRadius: BorderRadius.circular(6),
                ),
                child: Text(
                  '${svc.state}${svc.subState.isNotEmpty ? ' / ${svc.subState}' : ''}',
                  style: TextStyle(fontSize: 11, fontWeight: FontWeight.w500,
                      color: stateColor()),
                ),
              ),
            ],
          ),
          if (svc.uptimeSeconds.toInt() > 0 || svc.restartCount > 0) ...[
            const SizedBox(height: 8),
            Row(
              children: [
                if (svc.uptimeSeconds.toInt() > 0) ...[
                  Icon(Icons.timer_outlined, size: 12, color: context.subtitleColor),
                  const SizedBox(width: 4),
                  Text(_formatUptime(svc.uptimeSeconds.toInt()),
                      style: TextStyle(fontSize: 11, color: context.subtitleColor)),
                ],
                if (svc.restartCount > 0) ...[
                  const SizedBox(width: 12),
                  Icon(Icons.replay, size: 12, color: context.subtitleColor),
                  const SizedBox(width: 4),
                  Text('重启 ${svc.restartCount} 次',
                      style: TextStyle(fontSize: 11, color: context.subtitleColor)),
                ],
              ],
            ),
          ],
          const SizedBox(height: 10),
          Row(
            mainAxisAlignment: MainAxisAlignment.end,
            children: [
              _buildServiceAction(
                svc.name,
                Icons.play_arrow,
                '启动',
                ServiceAction.SERVICE_ACTION_START,
                enabled: svc.state.toLowerCase() != 'active',
              ),
              const SizedBox(width: 8),
              _buildServiceAction(
                svc.name,
                Icons.stop,
                '停止',
                ServiceAction.SERVICE_ACTION_STOP,
                enabled: svc.state.toLowerCase() == 'active',
              ),
              const SizedBox(width: 8),
              _buildServiceAction(
                svc.name,
                Icons.refresh,
                '重启',
                ServiceAction.SERVICE_ACTION_RESTART,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildServiceAction(
    String serviceName,
    IconData icon,
    String label,
    ServiceAction action, {
    bool enabled = true,
  }) {
    return InkWell(
      onTap: enabled ? () => _manageService(serviceName, action) : null,
      borderRadius: BorderRadius.circular(6),
      child: Opacity(
        opacity: enabled ? 1.0 : 0.35,
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
          decoration: BoxDecoration(
            border: Border.all(color: context.borderColor),
            borderRadius: BorderRadius.circular(6),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(icon, size: 14, color: context.subtitleColor),
              const SizedBox(width: 4),
              Text(label,
                  style: TextStyle(fontSize: 11, color: context.subtitleColor)),
            ],
          ),
        ),
      ),
    );
  }
}
