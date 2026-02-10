import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:shimmer/shimmer.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/features/connection/network_scanner.dart';
import 'package:flutter_monitor/features/connection/bluetooth_service.dart';
import 'package:flutter_monitor/core/grpc/robot_client.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/features/connection/ble_control_screen.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

class ScanScreen extends StatefulWidget {
  const ScanScreen({super.key});

  @override
  State<ScanScreen> createState() => _ScanScreenState();
}

class _ScanScreenState extends State<ScanScreen>
    with SingleTickerProviderStateMixin {
  late TabController _tabController;
  final NetworkScanner _networkScanner = NetworkScanner();
  final List<DiscoveredRobot> _networkDevices = [];
  double _scanProgress = 0.0;
  bool _isNetworkScanning = false;

  // Manual entry — defaults loaded from settings/profile in initState
  final _hostController = TextEditingController();
  final _portController = TextEditingController(text: '50051');
  final _dogHostController = TextEditingController();
  final _dogPortController = TextEditingController(text: '13145');
  bool _isConnecting = false;
  bool _isDogConnecting = false;
  String? _errorMessage;

  StreamSubscription? _resultsSub;
  StreamSubscription? _progressSub;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);

    // Set defaults from saved devices / robot profile
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final prefs = context.read<SettingsPreferences>();
      final profile = context.read<RobotProfileProvider>().current;

      // Pre-fill from last connected device, or leave empty for scan discovery
      final defaultDevice = prefs.defaultDevice;
      if (defaultDevice != null && defaultDevice.host.isNotEmpty) {
        _hostController.text = defaultDevice.host;
        _portController.text = defaultDevice.port.toString();
      }
      _dogHostController.text = profile.defaultHost;
      _dogPortController.text = profile.defaultPort.toString();
    });

    // Start network scan automatically
    _startNetworkScan();
  }

  @override
  void dispose() {
    _tabController.dispose();
    _networkScanner.dispose();
    _hostController.dispose();
    _portController.dispose();
    _dogHostController.dispose();
    _dogPortController.dispose();
    _resultsSub?.cancel();
    _progressSub?.cancel();
    super.dispose();
  }

  Future<void> _startNetworkScan() async {
    setState(() {
      _isNetworkScanning = true;
      _networkDevices.clear();
      _scanProgress = 0.0;
    });

    _resultsSub?.cancel();
    _progressSub?.cancel();

    _resultsSub = _networkScanner.results.listen((robot) {
      if (mounted) {
        setState(() {
          if (!_networkDevices.contains(robot)) {
            _networkDevices.add(robot);
          }
        });
      }
    });

    _progressSub = _networkScanner.progress.listen((p) {
      if (mounted) setState(() => _scanProgress = p);
    });

    await _networkScanner.scan();
    if (mounted) setState(() => _isNetworkScanning = false);
  }

  Future<void> _connectToRobot(String host, int port) async {
    if (_isConnecting) return;
    setState(() {
      _isConnecting = true;
      _errorMessage = null;
    });
    HapticFeedback.lightImpact();

    // Show connecting overlay
    _showConnectingOverlay(context, host, port);

    try {
      final client = RobotClient(host: host, port: port);
      final provider = context.read<RobotConnectionProvider>();
      final connected = await provider.connect(client);
      if (!mounted) return;

      if (connected) {
        // Save device to history
        context.read<SettingsPreferences>().addOrUpdateDevice(SavedDevice(
          name: context.read<RobotProfileProvider>().current.name,
          host: host,
          port: port,
          lastConnected: DateTime.now(),
        ));
        // Dismiss overlay then navigate
        if (Navigator.canPop(context)) Navigator.pop(context);
        await Future.delayed(const Duration(milliseconds: 150));
        if (!mounted) return;
        Navigator.of(context).pushReplacementNamed('/robot-detail');
      } else {
        if (Navigator.canPop(context)) Navigator.pop(context);
        setState(() {
          _errorMessage = provider.errorMessage ?? '无法连接到机器人';
          _isConnecting = false;
        });
      }
    } catch (e) {
      if (!mounted) return;
      if (Navigator.canPop(context)) Navigator.pop(context);
      setState(() {
        _errorMessage = '连接错误: $e';
        _isConnecting = false;
      });
    }
  }

  void _showConnectingOverlay(BuildContext ctx, String host, int port) {
    final isDark = Theme.of(ctx).brightness == Brightness.dark;
    showDialog(
      context: ctx,
      barrierDismissible: false,
      barrierColor: Colors.black54,
      builder: (_) => Center(
        child: Container(
          width: 260,
          padding: const EdgeInsets.symmetric(vertical: 32, horizontal: 24),
          decoration: BoxDecoration(
            color: isDark ? const Color(0xFF1C1C1E) : Colors.white,
            borderRadius: BorderRadius.circular(16),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withValues(alpha: 0.2),
                blurRadius: 20,
                offset: const Offset(0, 4),
              ),
            ],
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              SizedBox(
                width: 48,
                height: 48,
                child: CircularProgressIndicator(
                  strokeWidth: 3,
                  color: AppColors.primary,
                  backgroundColor: isDark
                      ? Colors.white.withValues(alpha: 0.08)
                      : Colors.black.withValues(alpha: 0.06),
                ),
              ),
              const SizedBox(height: 20),
              Text(
                '正在连接...',
                style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.w600,
                  color: isDark ? Colors.white : Colors.black87,
                ),
              ),
              const SizedBox(height: 6),
              Text(
                '$host:$port',
                style: TextStyle(
                  fontSize: 13,
                  color: isDark ? Colors.white54 : Colors.black45,
                  fontFamily: 'monospace',
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  /// 直连 Dog Board (han_dog CMS gRPC)
  Future<void> _connectToDogDirect(String host, int port) async {
    if (_isDogConnecting) return;
    setState(() {
      _isDogConnecting = true;
      _errorMessage = null;
    });
    HapticFeedback.lightImpact();

    try {
      final provider = context.read<RobotConnectionProvider>();
      final connected = await provider.connectDog(host: host, port: port);
      if (!mounted) return;

      if (connected) {
        // 如果没有 Nav Board 连接, 也跳转到详情页
        if (!provider.isConnected) {
          Navigator.of(context).pushReplacementNamed('/robot-detail');
        } else {
          // 已有 Nav Board 连接, 仅提示成功
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('Dog Board 已连接')),
          );
        }
        setState(() => _isDogConnecting = false);
      } else {
        setState(() {
          _errorMessage = provider.dogClient?.errorMessage ?? '无法连接到机器狗';
          _isDogConnecting = false;
        });
      }
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _errorMessage = 'Dog Board 连接错误: $e';
        _isDogConnecting = false;
      });
    }
  }

  Future<void> _startMock() async {
    HapticFeedback.lightImpact();
    final client = MockRobotClient();
    final provider = context.read<RobotConnectionProvider>();
    await provider.connect(client);
    if (!mounted) return;
    Navigator.of(context).pushReplacementNamed('/robot-detail');
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: const Text('发现设备'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: Column(
        children: [
          // Tab bar
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
            decoration: BoxDecoration(
              color: isDark
                  ? Colors.white.withValues(alpha: 0.06)
                  : Colors.black.withValues(alpha: 0.04),
              borderRadius: BorderRadius.circular(12),
            ),
            child: TabBar(
              controller: _tabController,
              labelColor: context.titleColor,
              unselectedLabelColor: context.subtitleColor,
              indicatorSize: TabBarIndicatorSize.tab,
              dividerHeight: 0,
              indicator: BoxDecoration(
                color: isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.05),
                borderRadius: BorderRadius.circular(8),
              ),
              labelStyle: const TextStyle(
                  fontSize: 13, fontWeight: FontWeight.w600),
              unselectedLabelStyle: const TextStyle(
                  fontSize: 13, fontWeight: FontWeight.w400),
              tabs: const [
                Tab(text: '网络扫描'),
                Tab(text: '蓝牙'),
                Tab(text: '手动输入'),
              ],
            ),
          ),

          // Tab content
          Expanded(
            child: TabBarView(
              controller: _tabController,
              children: [
                _buildNetworkTab(),
                _buildBluetoothTab(),
                _buildManualTab(),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ==================== Network Tab ====================

  Widget _buildNetworkTab() {
    final isDark = context.isDark;

    return Column(
      children: [
        // Scan progress
        if (_isNetworkScanning)
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Column(
              children: [
                ClipRRect(
                  borderRadius: BorderRadius.circular(4),
                  child: LinearProgressIndicator(
                    value: _scanProgress,
                    backgroundColor: isDark
                        ? Colors.white.withValues(alpha: 0.06)
                        : Colors.black.withValues(alpha: 0.05),
                    valueColor:
                        AlwaysStoppedAnimation(context.subtitleColor),
                    minHeight: 3,
                  ),
                ),
                const SizedBox(height: 6),
                Text(
                  '扫描中... ${(_scanProgress * 100).toInt()}%',
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                  ),
                ),
              ],
            ),
          ),

        // Results list
        Expanded(
          child: _networkDevices.isEmpty && !_isNetworkScanning
              ? _buildEmptyState(
                  Icons.wifi_find,
                  '未发现设备',
                  '请确保机器人在同一网络',
                )
              : ListView.builder(
                  padding: const EdgeInsets.all(16),
                  itemCount: _networkDevices.length +
                      (_isNetworkScanning ? 3 : 0),
                  itemBuilder: (context, index) {
                    if (index >= _networkDevices.length) {
                      return _buildShimmerItem();
                    }
                    final robot = _networkDevices[index];
                    return _buildDeviceItem(
                      icon: Icons.wifi,
                      iconColor: context.subtitleColor,
                      title: robot.displayName,
                      subtitle: robot.address,
                      onTap: () =>
                          _connectToRobot(robot.ip, robot.port),
                    );
                  },
                ),
        ),

        // Rescan button
        Padding(
          padding: const EdgeInsets.all(16),
          child: SizedBox(
            width: double.infinity,
            height: 48,
            child: OutlinedButton.icon(
              onPressed: _isNetworkScanning ? null : _startNetworkScan,
              icon: Icon(
                _isNetworkScanning ? Icons.hourglass_top : Icons.refresh,
                size: 16,
              ),
              label: Text(_isNetworkScanning ? '扫描中...' : '重新扫描'),
              style: OutlinedButton.styleFrom(
                foregroundColor: context.titleColor,
                side: BorderSide(color: context.borderColor),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }

  // ==================== Bluetooth Tab ====================

  Widget _buildBluetoothTab() {
    return ChangeNotifierProvider(
      create: (_) => BluetoothService(),
      child: Consumer<BluetoothService>(
        builder: (context, ble, _) {
          return Column(
            children: [
              // Scanning indicator
              if (ble.state == BleConnectionState.scanning)
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Column(
                    children: [
                      const LinearProgressIndicator(minHeight: 3),
                      const SizedBox(height: 6),
                      Text(
                        '正在搜索蓝牙设备...',
                        style: TextStyle(
                          fontSize: 12,
                          color: context.subtitleColor,
                        ),
                      ),
                    ],
                  ),
                ),

              // Error message
              if (ble.errorMessage != null)
                Container(
                  margin: const EdgeInsets.all(16),
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: AppColors.error.withValues(alpha: 0.08),
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: Row(
                    children: [
                      const Icon(Icons.error_outline,
                          size: 18, color: AppColors.error),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(
                          ble.errorMessage!,
                          style: const TextStyle(
                              fontSize: 13, color: AppColors.error),
                        ),
                      ),
                    ],
                  ),
                ),

              // Devices list
              Expanded(
                child: ble.discoveredDevices.isEmpty &&
                        ble.state != BleConnectionState.scanning
                    ? _buildEmptyState(
                        Icons.bluetooth_searching,
                        '未发现蓝牙设备',
                        '请确保机器人蓝牙已开启',
                      )
                    : ListView.builder(
                        padding: const EdgeInsets.all(16),
                        itemCount: ble.discoveredDevices.length,
                        itemBuilder: (context, index) {
                          final device = ble.discoveredDevices[index];
                          return _buildDeviceItem(
                            icon: Icons.bluetooth,
                            iconColor: AppColors.secondary,
                            title: device.displayName,
                            subtitle:
                                'RSSI: ${device.rssi} dBm${device.hasRobotService ? ' · 机器人服务' : ''}',
                            trailing: _buildSignalBars(
                                device.signalQuality),
                            onTap: () async {
                              HapticFeedback.lightImpact();
                              final connected = await ble.connectDevice(device);
                              if (connected && mounted) {
                                Navigator.of(context).push(
                                  MaterialPageRoute(
                                    builder: (_) => BleControlScreen(bleService: ble),
                                  ),
                                );
                              }
                            },
                          );
                        },
                      ),
              ),

              // Scan button
              Padding(
                padding: const EdgeInsets.all(16),
                child: SizedBox(
                  width: double.infinity,
                  height: 48,
                  child: OutlinedButton.icon(
                    onPressed: ble.state == BleConnectionState.scanning
                        ? null
                        : () => ble.startScan(),
                    icon: Icon(
                      ble.state == BleConnectionState.scanning
                          ? Icons.hourglass_top
                          : Icons.bluetooth_searching,
                      size: 18,
                    ),
                    label: Text(ble.state == BleConnectionState.scanning
                        ? '搜索中...'
                        : '搜索蓝牙'),
                    style: OutlinedButton.styleFrom(
                      foregroundColor: context.titleColor,
                      side: BorderSide(color: context.borderColor),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8),
                      ),
                    ),
                  ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }

  // ==================== Manual Tab ====================

  Widget _buildManualTab() {
    final isDark = context.isDark;

    return SingleChildScrollView(
      padding: const EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const SizedBox(height: 16),
          Icon(
            Icons.link,
            size: 36,
            color: context.subtitleColor,
          ),
          const SizedBox(height: 16),
          Text(
            '手动输入机器人地址',
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: 17,
              fontWeight: FontWeight.w600,
              color: isDark ? Colors.white : Colors.black87,
            ),
          ),
          const SizedBox(height: 32),

          // IP field
          _buildInputField(
            controller: _hostController,
            icon: Icons.wifi_tethering,
            label: 'Robot IP',
          ),
          const SizedBox(height: 12),

          // Port field
          _buildInputField(
            controller: _portController,
            icon: Icons.tag,
            label: 'Port',
            keyboardType: TextInputType.number,
          ),

          // Error
          AnimatedSize(
            duration: const Duration(milliseconds: 200),
            child: _errorMessage != null
                ? Container(
                    margin: const EdgeInsets.only(top: 16),
                    padding: const EdgeInsets.all(12),
                    decoration: BoxDecoration(
                      color: AppColors.error.withValues(alpha: 0.08),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Row(
                      children: [
                        const Icon(Icons.error_outline,
                            size: 18, color: AppColors.error),
                        const SizedBox(width: 8),
                        Expanded(
                          child: Text(
                            _errorMessage!,
                            style: const TextStyle(
                                fontSize: 13, color: AppColors.error),
                          ),
                        ),
                      ],
                    ),
                  )
                : const SizedBox.shrink(),
          ),

          const SizedBox(height: 24),

          // Connect button
          _buildGradientButton(
            onPressed: _isConnecting
                ? null
                : () {
                    final host = _hostController.text.trim();
                    final port =
                        int.tryParse(_portController.text.trim()) ?? 50051;
                    _connectToRobot(host, port);
                  },
            isLoading: _isConnecting,
            icon: Icons.link,
            label: _isConnecting ? '连接中...' : '连接机器人',
          ),

          const SizedBox(height: 32),

          // ─── Dog Board 直连区域 ───
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: isDark
                  ? Colors.orange.withValues(alpha: 0.08)
                  : Colors.orange.withValues(alpha: 0.05),
              borderRadius: BorderRadius.circular(10),
              border: Border.all(
                color: Colors.orange.withValues(alpha: 0.2),
              ),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Row(
                  children: [
                    Icon(Icons.pets, size: 18, color: Colors.orange.shade700),
                    const SizedBox(width: 8),
                    Text(
                      'Dog Board 直连',
                      style: TextStyle(
                        fontSize: 15,
                        fontWeight: FontWeight.w600,
                        color: Colors.orange.shade700,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 4),
                Text(
                  '绕过导航板，直接连接机器狗控制板 (CMS)',
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                  ),
                ),
                const SizedBox(height: 12),
                _buildInputField(
                  controller: _dogHostController,
                  icon: Icons.pets,
                  label: 'Dog IP',
                ),
                const SizedBox(height: 8),
                _buildInputField(
                  controller: _dogPortController,
                  icon: Icons.tag,
                  label: 'CMS Port',
                  keyboardType: TextInputType.number,
                ),
                const SizedBox(height: 12),
                SizedBox(
                  height: 44,
                  child: OutlinedButton.icon(
                    onPressed: _isDogConnecting
                        ? null
                        : () {
                            final host = _dogHostController.text.trim();
                            final port = int.tryParse(
                                    _dogPortController.text.trim()) ??
                                13145;
                            _connectToDogDirect(host, port);
                          },
                    icon: Icon(
                      _isDogConnecting
                          ? Icons.hourglass_top
                          : Icons.link,
                      size: 16,
                      color: Colors.orange.shade700,
                    ),
                    label: Text(
                      _isDogConnecting ? '连接中...' : '直连 Dog Board',
                      style: TextStyle(color: Colors.orange.shade700),
                    ),
                    style: OutlinedButton.styleFrom(
                      side: BorderSide(
                          color: Colors.orange.withValues(alpha: 0.4)),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),

          const SizedBox(height: 16),

          // Mock button
          SizedBox(
            height: 50,
            child: OutlinedButton.icon(
              onPressed: _isConnecting ? null : _startMock,
              icon: const Icon(Icons.science_outlined, size: 18),
              label: const Text('Mock 演示模式'),
              style: OutlinedButton.styleFrom(
                foregroundColor: context.subtitleColor,
                side: BorderSide(color: context.dividerColor),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }

  // ==================== Shared Widgets ====================

  Widget _buildDeviceItem({
    required IconData icon,
    required Color iconColor,
    required String title,
    required String subtitle,
    Widget? trailing,
    VoidCallback? onTap,
  }) {
    final isDark = context.isDark;

    return Container(
      margin: const EdgeInsets.only(bottom: 8),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: context.borderColor),
      ),
      child: ListTile(
        onTap: onTap,
        contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 4),
        leading: Icon(icon, color: context.subtitleColor, size: 18),
        title: Text(
          title,
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w500,
            color: context.titleColor,
          ),
        ),
        subtitle: Text(
          subtitle,
          style: TextStyle(fontSize: 12, color: context.subtitleColor),
        ),
        trailing: trailing ??
            Icon(Icons.chevron_right, size: 16, color: context.subtitleColor),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      ),
    );
  }

  Widget _buildShimmerItem() {
    final isDark = context.isDark;
    return Shimmer.fromColors(
      baseColor: isDark ? Colors.white.withValues(alpha: 0.06) : Colors.grey.shade200,
      highlightColor:
          isDark ? Colors.white.withValues(alpha: 0.12) : Colors.grey.shade100,
      child: Container(
        height: 70,
        margin: const EdgeInsets.only(bottom: 10),
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(10),
        ),
      ),
    );
  }

  Widget _buildEmptyState(IconData icon, String title, String subtitle) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(icon, size: 56, color: context.subtitleColor),
          const SizedBox(height: 16),
          Text(
            title,
            style: TextStyle(
              fontSize: 17,
              fontWeight: FontWeight.w600,
              color: context.isDark ? Colors.white70 : Colors.black54,
            ),
          ),
          const SizedBox(height: 6),
          Text(
            subtitle,
            style: TextStyle(fontSize: 13, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  Widget _buildSignalBars(double quality) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: List.generate(4, (i) {
        final threshold = (i + 1) / 4.0;
        final active = quality >= threshold;
        return Container(
          width: 4,
          height: 6.0 + i * 4.0,
          margin: const EdgeInsets.only(right: 2),
          decoration: BoxDecoration(
            color: active ? context.titleColor : Colors.grey.withValues(alpha: 0.2),
            borderRadius: BorderRadius.circular(2),
          ),
        );
      }),
    );
  }

  Widget _buildInputField({
    required TextEditingController controller,
    required IconData icon,
    required String label,
    TextInputType keyboardType = TextInputType.text,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: context.inputFillColor,
        borderRadius: BorderRadius.circular(14),
      ),
      child: TextField(
        controller: controller,
        keyboardType: keyboardType,
        textInputAction: TextInputAction.next,
        style: TextStyle(
          fontSize: 16,
          fontWeight: FontWeight.w500,
          color: context.isDark ? Colors.white : Colors.black87,
        ),
        decoration: InputDecoration(
          prefixIcon: Icon(icon, size: 18, color: context.subtitleColor),
          hintText: label,
          hintStyle: TextStyle(color: context.subtitleColor),
          border: InputBorder.none,
          contentPadding:
              const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
        ),
      ),
    );
  }

  Widget _buildGradientButton({
    required VoidCallback? onPressed,
    required bool isLoading,
    required IconData icon,
    required String label,
  }) {
    return SizedBox(
      height: 44,
      width: double.infinity,
      child: TextButton(
        onPressed: onPressed,
        style: TextButton.styleFrom(
          foregroundColor: context.titleColor,
          backgroundColor: context.isDark ? Colors.white.withValues(alpha: 0.07) : Colors.black.withValues(alpha: 0.04),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(8),
            side: BorderSide(color: context.borderColor),
          ),
        ),
        child: isLoading
            ? SizedBox(
                width: 18,
                height: 18,
                child: CircularProgressIndicator(
                  strokeWidth: 2,
                  color: context.subtitleColor,
                ),
              )
            : Text(label, style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
      ),
    );
  }
}
