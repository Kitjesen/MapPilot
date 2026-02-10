import 'dart:async';
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

  // Manual entry
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

    WidgetsBinding.instance.addPostFrameCallback((_) {
      final prefs = context.read<SettingsPreferences>();
      final profile = context.read<RobotProfileProvider>().current;

      final defaultDevice = prefs.defaultDevice;
      if (defaultDevice != null && defaultDevice.host.isNotEmpty) {
        _hostController.text = defaultDevice.host;
        _portController.text = defaultDevice.port.toString();
      }
      _dogHostController.text = profile.defaultHost;
      _dogPortController.text = profile.defaultPort.toString();
    });

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

    _showConnectingOverlay(context, host, port);

    try {
      final client = RobotClient(host: host, port: port);
      final provider = context.read<RobotConnectionProvider>();
      final connected = await provider.connect(client);
      if (!mounted) return;

      if (connected) {
        context.read<SettingsPreferences>().addOrUpdateDevice(SavedDevice(
          name: context.read<RobotProfileProvider>().current.name,
          host: host,
          port: port,
          lastConnected: DateTime.now(),
        ));
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
    showDialog(
      context: ctx,
      barrierDismissible: false,
      barrierColor: Colors.black38,
      builder: (_) => Center(
        child: Container(
          width: 280,
          padding: const EdgeInsets.symmetric(vertical: 36, horizontal: 28),
          decoration: BoxDecoration(
            color: ctx.cardColor,
            borderRadius: BorderRadius.circular(AppRadius.xl),
            boxShadow: AppShadows.elevated(isDark: ctx.isDark),
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
                  backgroundColor: AppColors.primary.withValues(alpha: 0.1),
                ),
              ),
              const SizedBox(height: 24),
              Text(
                '正在连接...',
                style: TextStyle(
                  fontSize: 17,
                  fontWeight: FontWeight.w700,
                  color: ctx.titleColor,
                ),
              ),
              const SizedBox(height: 6),
              Text(
                '$host:$port',
                style: TextStyle(
                  fontSize: 13,
                  color: ctx.subtitleColor,
                  fontFamily: 'monospace',
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

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
        if (!provider.isConnected) {
          Navigator.of(context).pushReplacementNamed('/robot-detail');
        } else {
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
    return Scaffold(
      body: Container(
        decoration: BoxDecoration(gradient: context.bgGradient),
        child: SafeArea(
          child: Column(
            children: [
              // Header
              _buildHeader(context),
              // Title section
              _buildTitle(context),
              // Tab bar
              _buildTabBar(context),
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
        ),
      ),
    );
  }

  Widget _buildHeader(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(8, 8, 16, 0),
      child: Row(
        children: [
          IconButton(
            icon: const Icon(Icons.arrow_back_rounded, size: 22),
            color: context.titleColor,
            onPressed: () => Navigator.pop(context),
          ),
          const Spacer(),
          // Close button
          GestureDetector(
            onTap: () => Navigator.pop(context),
            child: Container(
              width: 36,
              height: 36,
              decoration: BoxDecoration(
                color: context.cardColor,
                shape: BoxShape.circle,
                boxShadow: [AppShadows.light()],
              ),
              child: Icon(Icons.close_rounded, size: 18, color: context.subtitleColor),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTitle(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(24, 12, 24, 16),
      child: Column(
        children: [
          Text(
            'Connect Robot',
            style: TextStyle(
              fontSize: 24,
              fontWeight: FontWeight.w800,
              color: context.titleColor,
              letterSpacing: -0.5,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            'Select a connection method to pair a new unit',
            style: TextStyle(
              fontSize: 14,
              color: context.subtitleColor,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTabBar(BuildContext context) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 24),
      decoration: BoxDecoration(
        color: context.surfaceColor,
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
      child: TabBar(
        controller: _tabController,
        labelColor: AppColors.primary,
        unselectedLabelColor: context.subtitleColor,
        indicatorSize: TabBarIndicatorSize.tab,
        dividerHeight: 0,
        indicator: BoxDecoration(
          color: context.cardColor,
          borderRadius: BorderRadius.circular(AppRadius.sm),
          boxShadow: [AppShadows.light()],
        ),
        indicatorPadding: const EdgeInsets.all(4),
        labelStyle: const TextStyle(fontSize: 13, fontWeight: FontWeight.w600),
        unselectedLabelStyle: const TextStyle(fontSize: 13, fontWeight: FontWeight.w400),
        tabs: const [
          Tab(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.wifi_rounded, size: 16),
                SizedBox(width: 6),
                Text('Network Scan'),
              ],
            ),
          ),
          Tab(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.bluetooth_rounded, size: 16),
                SizedBox(width: 6),
                Text('Bluetooth'),
              ],
            ),
          ),
          Tab(
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.edit_rounded, size: 16),
                SizedBox(width: 6),
                Text('Manual'),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ==================== Network Tab ====================

  Widget _buildNetworkTab() {
    return Column(
      children: [
        const SizedBox(height: 16),
        // Scanning status bar
        if (_isNetworkScanning)
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 24),
            child: Row(
              children: [
                Container(
                  width: 8,
                  height: 8,
                  decoration: const BoxDecoration(
                    color: AppColors.success,
                    shape: BoxShape.circle,
                  ),
                ),
                const SizedBox(width: 8),
                Text(
                  'SCANNING LOCAL NETWORK...',
                  style: TextStyle(
                    fontSize: 11,
                    fontWeight: FontWeight.w700,
                    color: AppColors.success,
                    letterSpacing: 1,
                  ),
                ),
                const Spacer(),
                GestureDetector(
                  onTap: _startNetworkScan,
                  child: Text(
                    'Refresh',
                    style: TextStyle(
                      fontSize: 13,
                      fontWeight: FontWeight.w600,
                      color: AppColors.primary,
                    ),
                  ),
                ),
              ],
            ),
          ),

        if (_isNetworkScanning)
          Padding(
            padding: const EdgeInsets.fromLTRB(24, 8, 24, 0),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(4),
              child: LinearProgressIndicator(
                value: _scanProgress,
                backgroundColor: AppColors.primary.withValues(alpha: 0.08),
                valueColor: const AlwaysStoppedAnimation(AppColors.primary),
                minHeight: 3,
              ),
            ),
          ),

        // Results list
        Expanded(
          child: _networkDevices.isEmpty && !_isNetworkScanning
              ? _buildEmptyState(
                  Icons.wifi_find_rounded,
                  '未发现设备',
                  '请确保机器人在同一网络',
                )
              : ListView.builder(
                  padding: const EdgeInsets.all(24),
                  itemCount: _networkDevices.length +
                      (_isNetworkScanning ? 2 : 0),
                  itemBuilder: (context, index) {
                    if (index >= _networkDevices.length) {
                      return _buildShimmerItem();
                    }
                    final robot = _networkDevices[index];
                    return _buildDeviceCard(
                      icon: Icons.smart_toy_rounded,
                      iconColor: AppColors.primary,
                      title: robot.displayName,
                      subtitle: robot.address,
                      signalLabel: 'Strong Signal',
                      signalColor: AppColors.success,
                      onConnect: () =>
                          _connectToRobot(robot.ip, robot.port),
                    );
                  },
                ),
        ),

        // Troubleshoot link
        Padding(
          padding: const EdgeInsets.only(bottom: 16),
          child: TextButton.icon(
            onPressed: () {},
            icon: Icon(Icons.help_outline_rounded, size: 16, color: AppColors.primary),
            label: Text(
              'Troubleshoot connection issues',
              style: TextStyle(
                fontSize: 13,
                color: AppColors.primary,
                fontWeight: FontWeight.w500,
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
              const SizedBox(height: 16),
              // Scanning indicator
              if (ble.state == BleConnectionState.scanning)
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 24),
                  child: Row(
                    children: [
                      Container(
                        width: 8,
                        height: 8,
                        decoration: const BoxDecoration(
                          color: AppColors.success,
                          shape: BoxShape.circle,
                        ),
                      ),
                      const SizedBox(width: 8),
                      Text(
                        'Scanning Active',
                        style: TextStyle(
                          fontSize: 12,
                          fontWeight: FontWeight.w600,
                          color: AppColors.success,
                        ),
                      ),
                      const Spacer(),
                      GestureDetector(
                        onTap: () => ble.startScan(),
                        child: Text(
                          'Rescan',
                          style: TextStyle(
                            fontSize: 13,
                            fontWeight: FontWeight.w600,
                            color: AppColors.primary,
                          ),
                        ),
                      ),
                    ],
                  ),
                ),

              if (ble.state == BleConnectionState.scanning)
                Padding(
                  padding: const EdgeInsets.fromLTRB(24, 8, 24, 0),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(4),
                    child: LinearProgressIndicator(
                      backgroundColor: AppColors.primary.withValues(alpha: 0.08),
                      valueColor: const AlwaysStoppedAnimation(AppColors.primary),
                      minHeight: 3,
                    ),
                  ),
                ),

              // Error
              if (ble.errorMessage != null)
                Container(
                  margin: const EdgeInsets.all(24),
                  padding: const EdgeInsets.all(14),
                  decoration: BoxDecoration(
                    color: AppColors.errorLight,
                    borderRadius: BorderRadius.circular(AppRadius.md),
                  ),
                  child: Row(
                    children: [
                      const Icon(Icons.error_outline, size: 18, color: AppColors.error),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Text(
                          ble.errorMessage!,
                          style: const TextStyle(fontSize: 13, color: AppColors.error),
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
                        Icons.bluetooth_searching_rounded,
                        '未发现蓝牙设备',
                        '请确保机器人蓝牙已开启',
                      )
                    : ListView.builder(
                        padding: const EdgeInsets.all(24),
                        itemCount: ble.discoveredDevices.length,
                        itemBuilder: (context, index) {
                          final device = ble.discoveredDevices[index];
                          final isStrong = device.signalQuality > 0.7;
                          final isWeak = device.signalQuality < 0.3;
                          return _buildDeviceCard(
                            icon: Icons.bluetooth_rounded,
                            iconColor: AppColors.secondary,
                            title: device.displayName,
                            subtitle:
                                '${device.id}${device.hasRobotService ? ' · Robot Service' : ''}',
                            signalLabel: isStrong ? 'STRONG SIGNAL' : isWeak ? 'WEAK SIGNAL' : 'MEDIUM',
                            signalColor: isStrong ? AppColors.success : isWeak ? AppColors.error : AppColors.warning,
                            onConnect: () async {
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

              // Search button
              Padding(
                padding: const EdgeInsets.fromLTRB(24, 0, 24, 16),
                child: SizedBox(
                  width: double.infinity,
                  height: 48,
                  child: ElevatedButton.icon(
                    onPressed: ble.state == BleConnectionState.scanning
                        ? null
                        : () => ble.startScan(),
                    icon: Icon(
                      ble.state == BleConnectionState.scanning
                          ? Icons.hourglass_top_rounded
                          : Icons.bluetooth_searching_rounded,
                      size: 18,
                    ),
                    label: Text(ble.state == BleConnectionState.scanning
                        ? '搜索中...'
                        : '搜索蓝牙'),
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
    return SingleChildScrollView(
      padding: const EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const SizedBox(height: 16),
          // Icon
          Container(
            width: 64,
            height: 64,
            alignment: Alignment.center,
            decoration: BoxDecoration(
              color: AppColors.primary.withValues(alpha: 0.08),
              shape: BoxShape.circle,
            ),
            child: const Icon(Icons.link_rounded, size: 32, color: AppColors.primary),
          ),
          const SizedBox(height: 20),
          Text(
            '手动输入机器人地址',
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.w700,
              color: context.titleColor,
            ),
          ),
          const SizedBox(height: 32),

          // IP field
          _buildInputField(
            controller: _hostController,
            icon: Icons.wifi_tethering_rounded,
            label: 'Robot IP',
          ),
          const SizedBox(height: 12),

          // Port field
          _buildInputField(
            controller: _portController,
            icon: Icons.tag_rounded,
            label: 'Port',
            keyboardType: TextInputType.number,
          ),

          // Error
          AnimatedSize(
            duration: const Duration(milliseconds: 200),
            child: _errorMessage != null
                ? Container(
                    margin: const EdgeInsets.only(top: 16),
                    padding: const EdgeInsets.all(14),
                    decoration: BoxDecoration(
                      color: AppColors.errorLight,
                      borderRadius: BorderRadius.circular(AppRadius.md),
                    ),
                    child: Row(
                      children: [
                        const Icon(Icons.error_outline, size: 18, color: AppColors.error),
                        const SizedBox(width: 10),
                        Expanded(
                          child: Text(
                            _errorMessage!,
                            style: const TextStyle(fontSize: 13, color: AppColors.error),
                          ),
                        ),
                      ],
                    ),
                  )
                : const SizedBox.shrink(),
          ),

          const SizedBox(height: 24),

          // Connect button
          SizedBox(
            height: 50,
            child: ElevatedButton.icon(
              onPressed: _isConnecting
                  ? null
                  : () {
                      final host = _hostController.text.trim();
                      final port =
                          int.tryParse(_portController.text.trim()) ?? 50051;
                      _connectToRobot(host, port);
                    },
              icon: _isConnecting
                  ? const SizedBox(
                      width: 18,
                      height: 18,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        color: Colors.white,
                      ),
                    )
                  : const Icon(Icons.link_rounded, size: 18),
              label: Text(_isConnecting ? '连接中...' : '连接机器人'),
            ),
          ),

          const SizedBox(height: 32),

          // ─── Dog Board 直连 ───
          Container(
            padding: const EdgeInsets.all(20),
            decoration: BoxDecoration(
              color: AppColors.warningLight,
              borderRadius: BorderRadius.circular(AppRadius.card),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Row(
                  children: [
                    Container(
                      width: 36,
                      height: 36,
                      decoration: BoxDecoration(
                        color: AppColors.warning.withValues(alpha: 0.15),
                        borderRadius: BorderRadius.circular(10),
                      ),
                      child: Icon(Icons.pets_rounded, size: 18, color: AppColors.warning),
                    ),
                    const SizedBox(width: 12),
                    Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          'Dog Board 直连',
                          style: TextStyle(
                            fontSize: 15,
                            fontWeight: FontWeight.w700,
                            color: AppColors.textPrimary,
                          ),
                        ),
                        Text(
                          '绕过导航板直接连接 CMS',
                          style: TextStyle(
                            fontSize: 12,
                            color: AppColors.textSecondary,
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
                const SizedBox(height: 16),
                _buildInputField(
                  controller: _dogHostController,
                  icon: Icons.pets_rounded,
                  label: 'Dog IP',
                ),
                const SizedBox(height: 8),
                _buildInputField(
                  controller: _dogPortController,
                  icon: Icons.tag_rounded,
                  label: 'CMS Port',
                  keyboardType: TextInputType.number,
                ),
                const SizedBox(height: 14),
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
                          ? Icons.hourglass_top_rounded
                          : Icons.link_rounded,
                      size: 16,
                    ),
                    label: Text(
                      _isDogConnecting ? '连接中...' : '直连 Dog Board',
                    ),
                    style: OutlinedButton.styleFrom(
                      foregroundColor: AppColors.warning,
                      side: BorderSide(color: AppColors.warning.withValues(alpha: 0.5)),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(AppRadius.md),
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
            height: 48,
            child: OutlinedButton.icon(
              onPressed: _isConnecting ? null : _startMock,
              icon: const Icon(Icons.science_outlined, size: 18),
              label: const Text('Mock 演示模式'),
            ),
          ),
        ],
      ),
    );
  }

  // ==================== Shared Widgets ====================

  Widget _buildDeviceCard({
    required IconData icon,
    required Color iconColor,
    required String title,
    required String subtitle,
    String? signalLabel,
    Color? signalColor,
    VoidCallback? onConnect,
  }) {
    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      padding: const EdgeInsets.all(16),
      decoration: context.cardDecoration,
      child: Row(
        children: [
          Container(
            width: 44,
            height: 44,
            decoration: BoxDecoration(
              color: iconColor.withValues(alpha: 0.1),
              borderRadius: BorderRadius.circular(14),
            ),
            child: Icon(icon, color: iconColor, size: 22),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: TextStyle(
                    fontSize: 15,
                    fontWeight: FontWeight.w600,
                    color: context.titleColor,
                  ),
                ),
                const SizedBox(height: 2),
                Row(
                  children: [
                    Flexible(
                      child: Text(
                        subtitle,
                        style: TextStyle(fontSize: 12, color: context.subtitleColor),
                        overflow: TextOverflow.ellipsis,
                      ),
                    ),
                    if (signalLabel != null) ...[
                      const SizedBox(width: 8),
                      Text(
                        '· ',
                        style: TextStyle(color: context.subtitleColor, fontSize: 12),
                      ),
                      Text(
                        signalLabel,
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          color: signalColor ?? AppColors.success,
                        ),
                      ),
                    ],
                  ],
                ),
              ],
            ),
          ),
          const SizedBox(width: 12),
          SizedBox(
            height: 36,
            child: ElevatedButton(
              onPressed: onConnect,
              style: ElevatedButton.styleFrom(
                padding: const EdgeInsets.symmetric(horizontal: 16),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(AppRadius.sm),
                ),
              ),
              child: const Text('Connect', style: TextStyle(fontSize: 13)),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildShimmerItem() {
    final isDark = context.isDark;
    return Shimmer.fromColors(
      baseColor: isDark ? Colors.white.withValues(alpha: 0.06) : const Color(0xFFEEEBFF),
      highlightColor: isDark ? Colors.white.withValues(alpha: 0.12) : const Color(0xFFF8F6FF),
      child: Container(
        height: 76,
        margin: const EdgeInsets.only(bottom: 12),
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
        ),
      ),
    );
  }

  Widget _buildEmptyState(IconData icon, String title, String subtitle) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Container(
            width: 80,
            height: 80,
            decoration: BoxDecoration(
              color: AppColors.primary.withValues(alpha: 0.06),
              shape: BoxShape.circle,
            ),
            child: Icon(icon, size: 40, color: AppColors.primary.withValues(alpha: 0.4)),
          ),
          const SizedBox(height: 20),
          Text(
            title,
            style: TextStyle(
              fontSize: 17,
              fontWeight: FontWeight.w700,
              color: context.titleColor,
            ),
          ),
          const SizedBox(height: 6),
          Text(
            subtitle,
            style: TextStyle(fontSize: 14, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  Widget _buildInputField({
    required TextEditingController controller,
    required IconData icon,
    required String label,
    TextInputType keyboardType = TextInputType.text,
  }) {
    return TextField(
      controller: controller,
      keyboardType: keyboardType,
      textInputAction: TextInputAction.next,
      style: TextStyle(
        fontSize: 15,
        fontWeight: FontWeight.w500,
        color: context.titleColor,
      ),
      decoration: InputDecoration(
        prefixIcon: Icon(icon, size: 18, color: context.subtitleColor),
        hintText: label,
        hintStyle: TextStyle(color: context.hintColor),
      ),
    );
  }
}
