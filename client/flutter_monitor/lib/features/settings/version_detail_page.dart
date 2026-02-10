import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

class VersionDetailPage extends StatefulWidget {
  const VersionDetailPage({super.key});

  @override
  State<VersionDetailPage> createState() => _VersionDetailPageState();
}

class _VersionDetailPageState extends State<VersionDetailPage> {
  // Build-time injected constants (via --dart-define)
  static const _appVersion = String.fromEnvironment('APP_VERSION', defaultValue: '1.0.0');
  static const _buildNumber = String.fromEnvironment('BUILD_NUMBER', defaultValue: '1');
  static const _gitHash = String.fromEnvironment('GIT_HASH', defaultValue: 'dev');

  // Robot info from GetRobotInfo RPC
  String _robotId = '';
  String _robotDisplayName = '';
  String _firmwareVersion = '';
  String _softwareVersion = '';
  bool _loadingInfo = false;

  @override
  void initState() {
    super.initState();
    _fetchAllInfo();
  }

  Future<void> _fetchAllInfo() async {
    await Future.wait([
      _fetchRobotInfo(),
      _fetchInstalledVersions(),
    ]);
  }

  Future<void> _fetchRobotInfo() async {
    final conn = context.read<RobotConnectionProvider>();
    final client = conn.client;
    if (client == null) return;

    setState(() => _loadingInfo = true);
    try {
      final info = await client.getRobotInfo();
      if (mounted) {
        setState(() {
          _robotId = info.robotId;
          _robotDisplayName = info.displayName;
          _firmwareVersion = info.firmwareVersion;
          _softwareVersion = info.softwareVersion;
          _loadingInfo = false;
        });
      }
    } catch (_) {
      if (mounted) setState(() => _loadingInfo = false);
    }
  }

  Future<void> _fetchInstalledVersions() async {
    final ota = context.read<OtaGateway>();
    // Only fetch if not already loaded
    if (ota.installedVersions.isEmpty && ota.robotSystemVersion == null) {
      try {
        await ota.fetchInstalledVersions();
      } catch (_) {
        // Non-critical — gracefully handle missing OTA daemon
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final ota = context.watch<OtaGateway>();
    final ctrl = context.watch<ControlGateway>();
    final conn = context.watch<RobotConnectionProvider>();
    final isConnected = conn.client != null;

    final robotVersion = ota.robotSystemVersion ?? (isConnected ? '查询中...' : '未连接');
    final dogVersion = ctrl.dogClient?.isConnected == true
        ? '已连接'
        : '未连接';

    return Scaffold(
      appBar: AppBar(
        title: const Text('版本信息'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== Logo & Version =====
          Container(
            padding: const EdgeInsets.all(28),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              children: [
                Icon(Icons.smart_toy_outlined,
                    size: 36, color: context.subtitleColor),
                const SizedBox(height: 16),
                Text(
                  '大算机器人',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.w700,
                    color: context.titleColor,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  'Robot Monitor & Control',
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 20),

          // ===== App Info =====
          _SectionHeader(title: 'App', isDark: isDark),
          const SizedBox(height: 8),
          Container(
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              children: [
                _buildRow(context, 'App 版本', 'v$_appVersion'),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, 'Build 号', _buildNumber),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, 'Git 提交', _gitHash),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, '协议版本', 'gRPC v1.0'),
              ],
            ),
          ),
          const SizedBox(height: 20),

          // ===== Robot Info =====
          _SectionHeader(title: '机器人', isDark: isDark),
          const SizedBox(height: 8),
          Container(
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              children: [
                if (_robotId.isNotEmpty) ...[
                  _buildRow(context, '机器人 ID', _robotId),
                  Divider(height: 0.5, indent: 16, color: context.dividerColor),
                ],
                if (_robotDisplayName.isNotEmpty) ...[
                  _buildRow(context, '机器人名称', _robotDisplayName),
                  Divider(height: 0.5, indent: 16, color: context.dividerColor),
                ],
                if (_firmwareVersion.isNotEmpty) ...[
                  _buildRow(context, '固件版本', _firmwareVersion),
                  Divider(height: 0.5, indent: 16, color: context.dividerColor),
                ],
                if (_softwareVersion.isNotEmpty) ...[
                  _buildRow(context, '软件版本', _softwareVersion),
                  Divider(height: 0.5, indent: 16, color: context.dividerColor),
                ],
                _buildRow(context, 'Nav Board 系统', robotVersion),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, 'Dog Board 状态', dogVersion),
                if (_loadingInfo)
                  const Padding(
                    padding: EdgeInsets.all(12),
                    child: SizedBox(
                      width: 16, height: 16,
                      child: CircularProgressIndicator(strokeWidth: 1.5),
                    ),
                  ),
              ],
            ),
          ),
          const SizedBox(height: 20),

          // ===== Installed Components =====
          if (ota.installedVersions.isNotEmpty) ...[
            _SectionHeader(title: '已安装组件', isDark: isDark),
            const SizedBox(height: 8),
            Container(
              decoration: BoxDecoration(
                color: isDark ? AppColors.darkCard : Colors.white,
                borderRadius: BorderRadius.circular(AppRadius.card),
                boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
              ),
              child: Column(
                children: [
                  for (var i = 0; i < ota.installedVersions.length; i++) ...[
                    if (i > 0)
                      Divider(height: 0.5, indent: 16, color: context.dividerColor),
                    _buildRow(
                      context,
                      ota.installedVersions[i].name,
                      ota.installedVersions[i].version,
                    ),
                  ],
                ],
              ),
            ),
            const SizedBox(height: 20),
          ],

          // ===== Copy All Button =====
          SizedBox(
            height: 48,
            child: TextButton(
              onPressed: () => _copyAll(context, robotVersion, dogVersion),
              style: TextButton.styleFrom(
                foregroundColor: context.titleColor,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                  side: BorderSide(color: context.borderColor),
                ),
              ),
              child: const Text('一键复制版本信息',
                  style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500)),
            ),
          ),

          // ===== Footer =====
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 24),
            child: Text(
              '© 2026 大算科技\ngRPC · WebRTC · ROS 2 · BLE',
              textAlign: TextAlign.center,
              style: TextStyle(
                fontSize: 12,
                color: context.subtitleColor,
                height: 1.6,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRow(BuildContext context, String label, String value) {
    final isDark = context.isDark;
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
      child: Row(
        children: [
          Expanded(
            flex: 2,
            child: Text(
              label,
              style: TextStyle(fontSize: 14, color: context.subtitleColor),
            ),
          ),
          Expanded(
            flex: 3,
            child: Text(
              value,
              textAlign: TextAlign.end,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white : Colors.black87,
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _copyAll(BuildContext context, String robotVersion, String dogVersion) {
    final ota = context.read<OtaGateway>();
    final components = ota.installedVersions
        .map((c) => '  ${c.name}: ${c.version}')
        .join('\n');

    final info = 'App: v$_appVersion (Build $_buildNumber, $_gitHash)\n'
        'Nav Board: $robotVersion\n'
        'Dog Board: $dogVersion\n'
        '${components.isNotEmpty ? 'Components:\n$components' : ''}';

    Clipboard.setData(ClipboardData(text: info));
    HapticFeedback.lightImpact();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: const Text('版本信息已复制到剪贴板'),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        duration: const Duration(seconds: 1),
      ),
    );
  }
}

class _SectionHeader extends StatelessWidget {
  final String title;
  final bool isDark;

  const _SectionHeader({required this.title, required this.isDark});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(left: 4),
      child: Text(
        title,
        style: TextStyle(
          fontSize: 13,
          fontWeight: FontWeight.w600,
          color: isDark ? Colors.white54 : Colors.black45,
        ),
      ),
    );
  }
}
