import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/ble/ble_protocol.dart';
import 'package:flutter_monitor/core/ble/ble_robot_client.dart';
import 'package:flutter_monitor/features/connection/bluetooth_service.dart';

/// BLE 机器人控制面板
///
/// 提供通过 BLE 连接与机器人交互的 UI:
/// - 状态监控（电量、温度、模式、运行时间）
/// - 紧急停止
/// - 模式切换
/// - WiFi 配置
/// - 连接测试 (Ping)
class BleControlScreen extends StatefulWidget {
  final BluetoothService bleService;

  const BleControlScreen({super.key, required this.bleService});

  @override
  State<BleControlScreen> createState() => _BleControlScreenState();
}

class _BleControlScreenState extends State<BleControlScreen> {
  final _ssidController = TextEditingController();
  final _passwordController = TextEditingController();
  bool _passwordVisible = false;
  bool _isSendingWifi = false;
  StreamSubscription<BleStatusData>? _statusSub;
  BleStatusData? _lastStatus;
  int _pingLatencyMs = -1;

  BleRobotClient get _client => widget.bleService.robotClient;

  @override
  void initState() {
    super.initState();
    _statusSub = _client.statusStream.listen((status) {
      if (mounted) setState(() => _lastStatus = status);
    });
    _lastStatus = _client.latestStatus;
  }

  @override
  void dispose() {
    _statusSub?.cancel();
    _ssidController.dispose();
    _passwordController.dispose();
    super.dispose();
  }

  Future<void> _doPing() async {
    final sw = Stopwatch()..start();
    await _client.sendPing();
    // 等待 Pong 回来 (最多 2 秒)
    for (int i = 0; i < 20; i++) {
      await Future.delayed(const Duration(milliseconds: 100));
      if (_client.lastPongTime != null &&
          _client.lastPongTime!.isAfter(
              DateTime.now().subtract(const Duration(seconds: 2)))) {
        sw.stop();
        if (mounted) setState(() => _pingLatencyMs = sw.elapsedMilliseconds);
        return;
      }
    }
    if (mounted) setState(() => _pingLatencyMs = -1);
  }

  Future<void> _sendWifiConfig() async {
    final ssid = _ssidController.text.trim();
    final pass = _passwordController.text;
    if (ssid.isEmpty) return;
    setState(() => _isSendingWifi = true);
    try {
      await _client.sendWifiConfig(ssid, pass);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('WiFi 配置已发送')),
        );
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('发送失败: $e')),
        );
      }
    } finally {
      if (mounted) setState(() => _isSendingWifi = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final device = widget.bleService.connectedDevice;

    return Scaffold(
      appBar: AppBar(
        title: Text(device?.platformName ?? 'BLE 控制'),
        actions: [
          // Ping 按钮
          IconButton(
            icon: const Icon(Icons.network_ping, size: 22),
            tooltip: 'Ping',
            onPressed: _client.isReady ? _doPing : null,
          ),
          // 断开连接
          IconButton(
            icon: const Icon(Icons.link_off, size: 22),
            tooltip: '断开连接',
            onPressed: () async {
              await widget.bleService.disconnectDevice();
              if (mounted) Navigator.of(context).pop();
            },
          ),
        ],
      ),
      body: !_client.isReady
          ? _buildNotReady(isDark)
          : ListView(
              padding: const EdgeInsets.all(16),
              children: [
                // 连接信息
                _buildConnectionCard(isDark),
                const SizedBox(height: 16),

                // 状态面板
                _buildStatusCard(isDark),
                const SizedBox(height: 16),

                // 紧急停止
                _buildEstopButton(),
                const SizedBox(height: 16),

                // 模式切换
                _buildModeCard(isDark),
                const SizedBox(height: 16),

                // WiFi 配置
                _buildWifiCard(isDark),
              ],
            ),
    );
  }

  Widget _buildNotReady(bool isDark) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(Icons.bluetooth_disabled,
              size: 64, color: context.subtitleColor),
          const SizedBox(height: 16),
          Text(
            'BLE 服务未就绪',
            style: TextStyle(
              fontSize: 17,
              fontWeight: FontWeight.w600,
              color: isDark ? Colors.white70 : Colors.black54,
            ),
          ),
          const SizedBox(height: 8),
          Text(
            '设备已连接但未发现机器人 BLE 服务',
            style: TextStyle(fontSize: 13, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  Widget _buildConnectionCard(bool isDark) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(color: context.cardShadowColor, blurRadius: 10),
        ],
      ),
      child: Row(
        children: [
          Container(
            width: 48,
            height: 48,
            decoration: BoxDecoration(
              color: AppColors.success.withOpacity(0.15),
              borderRadius: BorderRadius.circular(14),
            ),
            child: const Icon(Icons.bluetooth_connected,
                color: AppColors.success, size: 24),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'BLE 已连接',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.w600,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  widget.bleService.connectedDevice?.remoteId.str ?? '-',
                  style:
                      TextStyle(fontSize: 12, color: context.subtitleColor),
                ),
              ],
            ),
          ),
          if (_pingLatencyMs >= 0)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
              decoration: BoxDecoration(
                color: AppColors.primary.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                '${_pingLatencyMs}ms',
                style: const TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w600,
                    color: AppColors.primary),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildStatusCard(bool isDark) {
    final s = _lastStatus;
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(color: context.cardShadowColor, blurRadius: 10),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('机器人状态',
              style: TextStyle(
                fontSize: 15,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white : Colors.black87,
              )),
          const SizedBox(height: 12),
          Row(
            children: [
              _statusTile(
                Icons.battery_std,
                '电量',
                s != null ? '${s.batteryPercent}%' : '--',
                _batteryColor(s?.batteryPercent),
              ),
              _statusTile(
                Icons.thermostat,
                '温度',
                s != null ? '${s.cpuTemp}°C' : '--',
                s != null && s.cpuTemp > 70
                    ? AppColors.error
                    : AppColors.info,
              ),
              _statusTile(
                Icons.gamepad,
                '模式',
                s?.modeString ?? '--',
                AppColors.secondary,
              ),
              _statusTile(
                Icons.timer,
                '运行',
                s != null ? _formatUptime(s.uptimeSeconds) : '--',
                AppColors.primary,
              ),
            ],
          ),
          if (s != null && s.hasError) ...[
            const SizedBox(height: 12),
            Container(
              padding: const EdgeInsets.all(10),
              decoration: BoxDecoration(
                color: AppColors.error.withOpacity(0.08),
                borderRadius: BorderRadius.circular(10),
              ),
              child: Row(
                children: [
                  const Icon(Icons.error_outline,
                      size: 16, color: AppColors.error),
                  const SizedBox(width: 8),
                  Text(
                    '错误码: 0x${s.errorCode.toRadixString(16).toUpperCase()}',
                    style:
                        const TextStyle(fontSize: 13, color: AppColors.error),
                  ),
                ],
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _statusTile(IconData icon, String label, String value, Color color) {
    return Expanded(
      child: Column(
        children: [
          Icon(icon, size: 20, color: color),
          const SizedBox(height: 4),
          Text(
            value,
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w700,
              color: color,
            ),
          ),
          const SizedBox(height: 2),
          Text(
            label,
            style: TextStyle(fontSize: 11, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  Widget _buildEstopButton() {
    return SizedBox(
      height: 56,
      child: ElevatedButton.icon(
        onPressed: () async {
          HapticFeedback.heavyImpact();
          await _client.sendEmergencyStop();
          if (mounted) {
            ScaffoldMessenger.of(context).showSnackBar(
              const SnackBar(
                content: Text('紧急停止已发送'),
                backgroundColor: AppColors.error,
              ),
            );
          }
        },
        icon: const Icon(Icons.emergency, size: 24),
        label: const Text('紧急停止',
            style: TextStyle(fontSize: 16, fontWeight: FontWeight.w700)),
        style: ElevatedButton.styleFrom(
          backgroundColor: AppColors.error,
          foregroundColor: Colors.white,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
        ),
      ),
    );
  }

  Widget _buildModeCard(bool isDark) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(color: context.cardShadowColor, blurRadius: 10),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('模式切换',
              style: TextStyle(
                fontSize: 15,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white : Colors.black87,
              )),
          const SizedBox(height: 12),
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: [
              _modeChip('待机', BleProtocol.modeIdle, Icons.pause_circle_outline),
              _modeChip('手动', BleProtocol.modeManual, Icons.sports_esports),
              _modeChip('遥控', BleProtocol.modeTeleop, Icons.gamepad),
              _modeChip('自主', BleProtocol.modeAutonomous, Icons.smart_toy),
              _modeChip('建图', BleProtocol.modeMapping, Icons.map),
            ],
          ),
        ],
      ),
    );
  }

  Widget _modeChip(String label, int mode, IconData icon) {
    final isActive = _lastStatus?.mode == mode;
    return ActionChip(
      avatar: Icon(icon,
          size: 16,
          color: isActive ? Colors.white : AppColors.primary),
      label: Text(label),
      backgroundColor: isActive ? AppColors.primary : null,
      labelStyle: TextStyle(
        fontSize: 13,
        fontWeight: FontWeight.w500,
        color: isActive ? Colors.white : null,
      ),
      side: BorderSide(
        color: isActive
            ? AppColors.primary
            : AppColors.primary.withOpacity(0.2),
      ),
      onPressed: () async {
        HapticFeedback.lightImpact();
        await _client.sendModeSwitch(mode);
        // 稍等一下再刷新状态
        await Future.delayed(const Duration(milliseconds: 300));
        await _client.requestStatus();
      },
    );
  }

  Widget _buildWifiCard(bool isDark) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(color: context.cardShadowColor, blurRadius: 10),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('WiFi 配置',
              style: TextStyle(
                fontSize: 15,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white : Colors.black87,
              )),
          const SizedBox(height: 4),
          Text('通过 BLE 为机器人配置 WiFi（首次连接时使用）',
              style: TextStyle(fontSize: 12, color: context.subtitleColor)),
          const SizedBox(height: 12),
          Container(
            decoration: BoxDecoration(
              color: context.inputFillColor,
              borderRadius: BorderRadius.circular(12),
            ),
            child: TextField(
              controller: _ssidController,
              style: TextStyle(
                  fontSize: 15,
                  color: isDark ? Colors.white : Colors.black87),
              decoration: InputDecoration(
                prefixIcon: const Icon(Icons.wifi, size: 20,
                    color: AppColors.primary),
                hintText: 'WiFi SSID',
                hintStyle: TextStyle(color: context.subtitleColor),
                border: InputBorder.none,
                contentPadding: const EdgeInsets.symmetric(
                    horizontal: 12, vertical: 14),
              ),
            ),
          ),
          const SizedBox(height: 8),
          Container(
            decoration: BoxDecoration(
              color: context.inputFillColor,
              borderRadius: BorderRadius.circular(12),
            ),
            child: TextField(
              controller: _passwordController,
              obscureText: !_passwordVisible,
              style: TextStyle(
                  fontSize: 15,
                  color: isDark ? Colors.white : Colors.black87),
              decoration: InputDecoration(
                prefixIcon: const Icon(Icons.lock_outline,
                    size: 20, color: AppColors.primary),
                suffixIcon: IconButton(
                  icon: Icon(
                    _passwordVisible
                        ? Icons.visibility_off
                        : Icons.visibility,
                    size: 20,
                    color: context.subtitleColor,
                  ),
                  onPressed: () =>
                      setState(() => _passwordVisible = !_passwordVisible),
                ),
                hintText: '密码',
                hintStyle: TextStyle(color: context.subtitleColor),
                border: InputBorder.none,
                contentPadding: const EdgeInsets.symmetric(
                    horizontal: 12, vertical: 14),
              ),
            ),
          ),
          const SizedBox(height: 12),
          SizedBox(
            width: double.infinity,
            height: 44,
            child: ElevatedButton.icon(
              onPressed: _isSendingWifi ? null : _sendWifiConfig,
              icon: _isSendingWifi
                  ? const SizedBox(
                      width: 16,
                      height: 16,
                      child: CircularProgressIndicator(
                          strokeWidth: 2, color: Colors.white),
                    )
                  : const Icon(Icons.send, size: 18),
              label: Text(_isSendingWifi ? '发送中...' : '发送配置'),
              style: ElevatedButton.styleFrom(
                backgroundColor: AppColors.primary,
                foregroundColor: Colors.white,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Color _batteryColor(int? percent) {
    if (percent == null) return AppColors.info;
    if (percent <= 15) return AppColors.error;
    if (percent <= 30) return AppColors.warning;
    return AppColors.success;
  }

  String _formatUptime(int seconds) {
    if (seconds < 60) return '${seconds}s';
    if (seconds < 3600) return '${seconds ~/ 60}m';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    return '${h}h${m}m';
  }
}
