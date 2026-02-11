import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:shimmer/shimmer.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/features/connection/network_scanner.dart';
import 'package:flutter_monitor/features/connection/bluetooth_service.dart';
import 'package:flutter_monitor/core/grpc/robot_client.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/features/connection/ble_control_screen.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

// ═══════════════════════════════════════════════════════════════
//  Robot Connection Manager — Network / Bluetooth / Manual
// ═══════════════════════════════════════════════════════════════

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
  bool _useSecure = false;
  int _protocol = 0; // 0=gRPC, 1=WebSocket

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

  // ─── Network scan ───
  Future<void> _startNetworkScan() async {
    setState(() { _isNetworkScanning = true; _networkDevices.clear(); _scanProgress = 0.0; });
    _resultsSub?.cancel();
    _progressSub?.cancel();
    _resultsSub = _networkScanner.results.listen((robot) {
      if (mounted) setState(() { if (!_networkDevices.contains(robot)) _networkDevices.add(robot); });
    });
    _progressSub = _networkScanner.progress.listen((p) {
      if (mounted) setState(() => _scanProgress = p);
    });
    await _networkScanner.scan();
    if (mounted) setState(() => _isNetworkScanning = false);
  }

  // ─── Connect to robot ───
  Future<void> _connectToRobot(String host, int port) async {
    if (_isConnecting) return;
    setState(() { _isConnecting = true; _errorMessage = null; });
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
          host: host, port: port, lastConnected: DateTime.now(),
        ));
        if (Navigator.canPop(context)) Navigator.pop(context);
        await Future.delayed(const Duration(milliseconds: 150));
        if (!mounted) return;
        Navigator.of(context).pushReplacementNamed('/robot-detail');
      } else {
        if (Navigator.canPop(context)) Navigator.pop(context);
        setState(() { _errorMessage = provider.errorMessage ?? '无法连接到机器人'; _isConnecting = false; });
      }
    } catch (e) {
      if (!mounted) return;
      if (Navigator.canPop(context)) Navigator.pop(context);
      setState(() { _errorMessage = '连接错误: $e'; _isConnecting = false; });
    }
  }

  void _showConnectingOverlay(BuildContext ctx, String host, int port) {
    showDialog(
      context: ctx, barrierDismissible: false, barrierColor: Colors.black38,
      builder: (_) => Center(child: Container(
        width: 280,
        padding: const EdgeInsets.symmetric(vertical: 36, horizontal: 28),
        decoration: BoxDecoration(
          color: ctx.cardColor,
          borderRadius: BorderRadius.circular(AppRadius.xl),
          boxShadow: AppShadows.elevated(isDark: ctx.isDark),
        ),
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          SizedBox(width: 48, height: 48, child: CircularProgressIndicator(
            strokeWidth: 3, color: AppColors.primary,
            backgroundColor: AppColors.primary.withValues(alpha: 0.1),
          )),
          const SizedBox(height: 24),
          Text('Connecting...', style: TextStyle(fontSize: 17, fontWeight: FontWeight.w700, color: ctx.titleColor)),
          const SizedBox(height: 6),
          Text('$host:$port', style: TextStyle(fontSize: 13, color: ctx.subtitleColor, fontFamily: 'monospace')),
        ]),
      )),
    );
  }

  Future<void> _connectToDogDirect(String host, int port) async {
    if (_isDogConnecting) return;
    setState(() { _isDogConnecting = true; _errorMessage = null; });
    HapticFeedback.lightImpact();
    try {
      final provider = context.read<RobotConnectionProvider>();
      final connected = await provider.connectDog(host: host, port: port);
      if (!mounted) return;
      if (connected) {
        if (!provider.isConnected) {
          Navigator.of(context).pushReplacementNamed('/robot-detail');
        } else {
          ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Dog Board 已连接')));
        }
        setState(() => _isDogConnecting = false);
      } else {
        setState(() { _errorMessage = provider.dogClient?.errorMessage ?? '无法连接到机器狗'; _isDogConnecting = false; });
      }
    } catch (e) {
      if (!mounted) return;
      setState(() { _errorMessage = 'Dog Board 连接错误: $e'; _isDogConnecting = false; });
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

  // ═══════════════════════════════════════════════════════════
  //  BUILD
  // ═══════════════════════════════════════════════════════════
  @override
  Widget build(BuildContext context) {
    final dark = context.isDark;
    return Scaffold(
      body: Container(
        decoration: BoxDecoration(gradient: context.bgGradient),
        child: SafeArea(
          child: Column(
            children: [
              _buildHeader(context),
              const SizedBox(height: 8),
              _buildTabBar(context),
              const SizedBox(height: 8),
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
              // Bottom actions
              _buildBottomBar(context, dark),
            ],
          ),
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  HEADER — Title + status badge + action icons
  // ═══════════════════════════════════════════════════════════
  Widget _buildHeader(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(24, 12, 16, 0),
      child: Row(
        children: [
          // Back arrow
          GestureDetector(
            onTap: () => Navigator.pop(context),
            child: Icon(Icons.arrow_back_rounded, size: 22, color: context.titleColor),
          ),
          const SizedBox(width: 16),
          // Title
          Expanded(child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(children: [
                Text('Robot Connection Manager', style: TextStyle(
                  fontSize: 20, fontWeight: FontWeight.w800, color: context.titleColor, letterSpacing: -0.3,
                )),
                const SizedBox(width: 10),
                if (_isNetworkScanning)
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 3),
                    decoration: BoxDecoration(
                      color: context.isDark ? Colors.white.withValues(alpha: 0.08) : const Color(0xFFF3F4F6),
                      borderRadius: BorderRadius.circular(10),
                    ),
                    child: Text('Scanning', style: TextStyle(
                      fontSize: 11, fontWeight: FontWeight.w600, color: context.subtitleColor,
                    )),
                  ),
              ]),
              Text('Searching for devices', style: TextStyle(
                fontSize: 13, color: AppColors.primary,
              )),
            ],
          )),
          // Action icons
          GestureDetector(
            onTap: _startNetworkScan,
            child: Icon(Icons.refresh_rounded, size: 22, color: context.subtitleColor),
          ),
          const SizedBox(width: 12),
          Icon(Icons.help_outline_rounded, size: 22, color: context.subtitleColor),
          const SizedBox(width: 8),
          GestureDetector(
            onTap: () => Navigator.pop(context),
            child: Icon(Icons.close_rounded, size: 22, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  TAB BAR — Network / Bluetooth / Manual
  // ═══════════════════════════════════════════════════════════
  Widget _buildTabBar(BuildContext context) {
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 24),
      child: TabBar(
        controller: _tabController,
        labelColor: AppColors.primary,
        unselectedLabelColor: context.subtitleColor,
        indicatorSize: TabBarIndicatorSize.label,
        indicatorColor: AppColors.primary,
        indicatorWeight: 2.5,
        dividerHeight: 0.5,
        dividerColor: context.borderColor,
        labelStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
        unselectedLabelStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w400),
        tabs: const [
          Tab(text: 'Network'),
          Tab(text: 'Bluetooth'),
          Tab(text: 'Manual'),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  NETWORK TAB
  // ═══════════════════════════════════════════════════════════
  Widget _buildNetworkTab() {
    return Column(
      children: [
        const SizedBox(height: 16),
        if (_isNetworkScanning)
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 24),
            child: Row(children: [
              Container(width: 8, height: 8, decoration: const BoxDecoration(
                color: AppColors.success, shape: BoxShape.circle)),
              const SizedBox(width: 8),
              Text('SCANNING LOCAL NETWORK...', style: TextStyle(
                fontSize: 11, fontWeight: FontWeight.w700, color: AppColors.success, letterSpacing: 1)),
              const Spacer(),
              GestureDetector(
                onTap: _startNetworkScan,
                child: Row(children: [
                  Icon(Icons.refresh_rounded, size: 14, color: AppColors.primary),
                  const SizedBox(width: 4),
                  Text('Refresh', style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: AppColors.primary)),
                ]),
              ),
            ]),
          ),
        if (_isNetworkScanning) Padding(
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
        Expanded(
          child: _networkDevices.isEmpty && !_isNetworkScanning
              ? _buildEmptyState(Icons.wifi_find_rounded, '未发现设备', '请确保机器人在同一网络')
              : ListView.builder(
                  padding: const EdgeInsets.all(24),
                  itemCount: _networkDevices.length + (_isNetworkScanning ? 2 : 0),
                  itemBuilder: (_, i) {
                    if (i >= _networkDevices.length) return _buildShimmerItem();
                    final robot = _networkDevices[i];
                    return _DeviceCard(
                      icon: Icons.smart_toy_rounded,
                      iconBg: AppColors.primary.withValues(alpha: 0.1),
                      iconColor: AppColors.primary,
                      title: robot.displayName,
                      subtitle: robot.address,
                      signalLabel: 'Strong Signal',
                      signalColor: AppColors.success,
                      actionLabel: 'Connect',
                      actionFilled: i == 0,
                      onAction: () => _connectToRobot(robot.ip, robot.port),
                    );
                  },
                ),
        ),
        // Troubleshoot
        Padding(
          padding: const EdgeInsets.only(bottom: 8),
          child: TextButton.icon(
            onPressed: () {},
            icon: Icon(Icons.help_outline_rounded, size: 14, color: AppColors.primary),
            label: Text('Troubleshoot connection issues',
              style: TextStyle(fontSize: 12, color: AppColors.primary, fontWeight: FontWeight.w500)),
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  BLUETOOTH TAB — Radar animation + device list
  // ═══════════════════════════════════════════════════════════
  Widget _buildBluetoothTab() {
    return ChangeNotifierProvider(
      create: (_) => BluetoothService(),
      child: Consumer<BluetoothService>(
        builder: (context, ble, _) {
          final isScanning = ble.state == BleConnectionState.scanning;
          final isDesktop = !context.isMobile;

          return Column(
            children: [
              Expanded(
                child: isDesktop
                    ? _buildBluetoothDesktop(context, ble, isScanning)
                    : _buildBluetoothMobile(context, ble, isScanning),
              ),
              // Bottom bar
              _buildBleBottomBar(context, ble, isScanning),
            ],
          );
        },
      ),
    );
  }

  Widget _buildBluetoothDesktop(BuildContext context, BluetoothService ble, bool isScanning) {
    return Row(
      children: [
        // Left: Radar animation
        Expanded(
          flex: 2,
          child: _BluetoothRadar(isScanning: isScanning),
        ),
        // Right: Device list
        Expanded(
          flex: 3,
          child: _buildBleDeviceList(context, ble, isScanning),
        ),
      ],
    );
  }

  Widget _buildBluetoothMobile(BuildContext context, BluetoothService ble, bool isScanning) {
    return Column(
      children: [
        // Radar (smaller)
        SizedBox(height: 200, child: _BluetoothRadar(isScanning: isScanning)),
        // Device list
        Expanded(child: _buildBleDeviceList(context, ble, isScanning)),
      ],
    );
  }

  Widget _buildBleDeviceList(BuildContext context, BluetoothService ble, bool isScanning) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        // Header
        Padding(
          padding: const EdgeInsets.fromLTRB(24, 20, 24, 0),
          child: Row(children: [
            Text('Available Devices (${ble.discoveredDevices.length})', style: TextStyle(
              fontSize: 16, fontWeight: FontWeight.w700, color: context.titleColor)),
            const Spacer(),
            GestureDetector(
              onTap: () => ble.startScan(),
              child: Row(children: [
                Icon(Icons.refresh_rounded, size: 14, color: AppColors.primary),
                const SizedBox(width: 4),
                Text('Rescan', style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: AppColors.primary)),
              ]),
            ),
          ]),
        ),

        // Error
        if (ble.errorMessage != null)
          Container(
            margin: const EdgeInsets.fromLTRB(24, 12, 24, 0),
            padding: const EdgeInsets.all(14),
            decoration: BoxDecoration(
              color: AppColors.errorLight, borderRadius: BorderRadius.circular(AppRadius.md),
            ),
            child: Row(children: [
              const Icon(Icons.error_outline, size: 18, color: AppColors.error),
              const SizedBox(width: 10),
              Expanded(child: Text(ble.errorMessage!, style: const TextStyle(fontSize: 13, color: AppColors.error))),
            ]),
          ),

        // Device list
        Expanded(
          child: ble.discoveredDevices.isEmpty && !isScanning
              ? Center(child: Text('No devices found', style: TextStyle(color: context.subtitleColor)))
              : ListView.builder(
                  padding: const EdgeInsets.all(24),
                  itemCount: ble.discoveredDevices.length,
                  itemBuilder: (_, i) {
                    final d = ble.discoveredDevices[i];
                    final strong = d.signalQuality > 0.7;
                    final weak = d.signalQuality < 0.3;
                    return _DeviceCard(
                      icon: Icons.smart_toy_rounded,
                      iconBg: AppColors.primary.withValues(alpha: 0.1),
                      iconColor: AppColors.primary,
                      title: d.displayName,
                      subtitle: 'MAC: ${d.id}${d.hasRobotService ? ' · Robot Service' : ''}',
                      signalLabel: strong ? 'STRONG SIGNAL' : weak ? 'WEAK SIGNAL' : 'MEDIUM',
                      signalColor: strong ? AppColors.success : weak ? AppColors.error : AppColors.warning,
                      actionLabel: 'Connect',
                      actionIcon: Icons.link_rounded,
                      onAction: () async {
                        HapticFeedback.lightImpact();
                        final connected = await ble.connectDevice(d);
                        if (connected && mounted) {
                          Navigator.of(context).push(MaterialPageRoute(
                            builder: (_) => BleControlScreen(bleService: ble),
                          ));
                        }
                      },
                    );
                  },
                ),
        ),

        // Manually enter MAC
        Padding(
          padding: const EdgeInsets.fromLTRB(24, 0, 24, 8),
          child: Column(children: [
            Text("Don't see your device?", style: TextStyle(fontSize: 12, color: context.subtitleColor)),
            const SizedBox(height: 8),
            OutlinedButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add, size: 16),
              label: const Text('Manually Enter MAC Address'),
              style: OutlinedButton.styleFrom(
                foregroundColor: context.titleColor,
                side: BorderSide(color: context.borderColor),
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(24)),
                padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
              ),
            ),
          ]),
        ),
      ],
    );
  }

  Widget _buildBleBottomBar(BuildContext context, BluetoothService ble, bool isScanning) {
    return Container(
      padding: const EdgeInsets.fromLTRB(24, 12, 24, 12),
      decoration: BoxDecoration(
        border: Border(top: BorderSide(color: context.borderColor.withValues(alpha: 0.5))),
      ),
      child: Row(
        children: [
          // BLE version info
          Row(children: [
            Icon(Icons.info_outline_rounded, size: 14, color: context.hintColor),
            const SizedBox(width: 6),
            Text('Bluetooth 5.0 Low Energy required', style: TextStyle(
              fontSize: 11, color: context.hintColor)),
          ]),
          const Spacer(),
          // Cancel
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: Text('Cancel', style: TextStyle(color: context.subtitleColor, fontWeight: FontWeight.w500)),
          ),
          const SizedBox(width: 8),
          // Done/Scan button
          ElevatedButton.icon(
            onPressed: isScanning ? null : () => ble.startScan(),
            icon: Icon(isScanning ? Icons.hourglass_top_rounded : Icons.refresh_rounded, size: 16),
            label: Text(isScanning ? 'Scanning...' : 'Scan Again'),
            style: ElevatedButton.styleFrom(
              backgroundColor: AppColors.primary,
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
              padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
            ),
          ),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  MANUAL TAB — Device Configuration form
  // ═══════════════════════════════════════════════════════════
  Widget _buildManualTab() {
    final dark = context.isDark;
    return SingleChildScrollView(
      padding: const EdgeInsets.all(24),
      child: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 480),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const SizedBox(height: 16),
              // Glass card container
              Container(
                padding: const EdgeInsets.all(28),
                decoration: BoxDecoration(
                  color: dark ? Colors.white.withValues(alpha: 0.04) : Colors.white.withValues(alpha: 0.7),
                  borderRadius: BorderRadius.circular(24),
                  border: Border.all(color: context.borderColor.withValues(alpha: 0.5)),
                  boxShadow: [BoxShadow(
                    color: Colors.black.withValues(alpha: dark ? 0.1 : 0.04),
                    blurRadius: 20, offset: const Offset(0, 4),
                  )],
                ),
                child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                  // Header
                  Row(children: [
                    Container(
                      width: 48, height: 48,
                      decoration: BoxDecoration(
                        color: AppColors.primary.withValues(alpha: 0.08),
                        borderRadius: BorderRadius.circular(14),
                      ),
                      child: const Icon(Icons.wifi_tethering_rounded, size: 24, color: AppColors.primary),
                    ),
                    const SizedBox(width: 14),
                    Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                      Text('Manual Input', style: TextStyle(
                        fontSize: 18, fontWeight: FontWeight.w700, color: context.titleColor)),
                      Text('Enter robot network parameters directly', style: TextStyle(
                        fontSize: 12, color: context.subtitleColor)),
                    ]),
                  ]),
                  const SizedBox(height: 28),

                  // Robot IP Address
                  _formLabel('Robot IP Address'),
                  const SizedBox(height: 8),
                  _buildIpField(context),
                  const SizedBox(height: 4),
                  Text('Static IP recommended for stable connection', style: TextStyle(
                    fontSize: 11, color: context.hintColor)),
                  const SizedBox(height: 20),

                  // Port + Protocol row
                  Row(children: [
                    Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                      _formLabel('Port'),
                      const SizedBox(height: 8),
                      _buildFormField(_portController, Icons.tag_rounded, '50051',
                        keyboardType: TextInputType.number),
                    ])),
                    const SizedBox(width: 16),
                    Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                      _formLabel('Protocol'),
                      const SizedBox(height: 8),
                      _buildProtocolDropdown(context),
                    ])),
                  ]),
                  const SizedBox(height: 20),

                  // SSL/TLS option
                  Container(
                    padding: const EdgeInsets.all(14),
                    decoration: BoxDecoration(
                      color: dark ? Colors.white.withValues(alpha: 0.03) : const Color(0xFFF9FAFB),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Row(children: [
                      SizedBox(width: 20, height: 20, child: Checkbox(
                        value: _useSecure,
                        onChanged: (v) => setState(() => _useSecure = v ?? false),
                        materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
                        visualDensity: VisualDensity.compact,
                      )),
                      const SizedBox(width: 12),
                      Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                        Text('Use Secure Connection (SSL/TLS)', style: TextStyle(
                          fontSize: 13, fontWeight: FontWeight.w600, color: context.titleColor)),
                        Text('Required for remote cloud operations', style: TextStyle(
                          fontSize: 11, color: context.subtitleColor)),
                      ])),
                    ]),
                  ),
                  const SizedBox(height: 24),

                  // Error
                  if (_errorMessage != null)
                    Container(
                      margin: const EdgeInsets.only(bottom: 16),
                      padding: const EdgeInsets.all(14),
                      decoration: BoxDecoration(
                        color: AppColors.errorLight, borderRadius: BorderRadius.circular(12)),
                      child: Row(children: [
                        const Icon(Icons.error_outline, size: 18, color: AppColors.error),
                        const SizedBox(width: 10),
                        Expanded(child: Text(_errorMessage!, style: const TextStyle(fontSize: 13, color: AppColors.error))),
                      ]),
                    ),

                  // Link Device button (gradient)
                  SizedBox(
                    height: 52,
                    width: double.infinity,
                    child: Container(
                      decoration: BoxDecoration(
                        gradient: AppColors.brandGradient,
                        borderRadius: BorderRadius.circular(14),
                        boxShadow: [BoxShadow(
                          color: AppColors.primary.withValues(alpha: 0.3),
                          blurRadius: 12, offset: const Offset(0, 4),
                        )],
                      ),
                      child: ElevatedButton.icon(
                        onPressed: _isConnecting ? null : () {
                          final host = _hostController.text.trim();
                          final port = int.tryParse(_portController.text.trim()) ?? 50051;
                          _connectToRobot(host, port);
                        },
                        icon: _isConnecting
                            ? const SizedBox(width: 18, height: 18, child: CircularProgressIndicator(
                                strokeWidth: 2, color: Colors.white))
                            : const Icon(Icons.link_rounded, size: 18),
                        label: Text(_isConnecting ? 'Connecting...' : 'Link Device'),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.transparent,
                          shadowColor: Colors.transparent,
                          foregroundColor: Colors.white,
                          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(height: 12),

                  // Cancel Setup
                  Center(child: TextButton(
                    onPressed: () => Navigator.pop(context),
                    child: Text('Cancel Setup', style: TextStyle(
                      fontSize: 13, color: context.subtitleColor, fontWeight: FontWeight.w500)),
                  )),
                ]),
              ),

              const SizedBox(height: 20),

              // Status indicator
              Center(child: Row(mainAxisSize: MainAxisSize.min, children: [
                Container(width: 6, height: 6, decoration: BoxDecoration(
                  color: _isConnecting ? AppColors.warning : context.hintColor,
                  shape: BoxShape.circle,
                )),
                const SizedBox(width: 6),
                Text(_isConnecting ? 'CONNECTING' : 'IDLE', style: TextStyle(
                  fontSize: 10, fontWeight: FontWeight.w600, letterSpacing: 1,
                  color: context.hintColor,
                )),
              ])),

              const SizedBox(height: 16),

              // Dog Board section
              _buildDogBoardSection(context),

              const SizedBox(height: 12),

              // Scan Network link
              Center(child: TextButton(
                onPressed: () {
                  _tabController.animateTo(0);
                },
                child: Text.rich(TextSpan(children: [
                  TextSpan(text: "Can't find your robot IP? ", style: TextStyle(
                    fontSize: 12, color: context.subtitleColor)),
                  TextSpan(text: 'Scan Network', style: TextStyle(
                    fontSize: 12, color: AppColors.primary, fontWeight: FontWeight.w600)),
                ])),
              )),

              const SizedBox(height: 8),

              // Mock button
              Center(child: OutlinedButton.icon(
                onPressed: _isConnecting ? null : _startMock,
                icon: const Icon(Icons.science_outlined, size: 16),
                label: const Text('Mock 演示模式', style: TextStyle(fontSize: 12)),
                style: OutlinedButton.styleFrom(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                ),
              )),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildDogBoardSection(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: AppColors.warningLight,
        borderRadius: BorderRadius.circular(16),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
        Row(children: [
          Container(
            width: 36, height: 36,
            decoration: BoxDecoration(
              color: AppColors.warning.withValues(alpha: 0.15),
              borderRadius: BorderRadius.circular(10),
            ),
            child: Icon(Icons.pets_rounded, size: 18, color: AppColors.warning),
          ),
          const SizedBox(width: 12),
          Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text('Dog Board 直连', style: TextStyle(
              fontSize: 15, fontWeight: FontWeight.w700, color: AppColors.textPrimary)),
            Text('绕过导航板直接连接 CMS', style: TextStyle(
              fontSize: 12, color: AppColors.textSecondary)),
          ]),
        ]),
        const SizedBox(height: 16),
        _buildFormField(_dogHostController, Icons.pets_rounded, 'Dog IP'),
        const SizedBox(height: 8),
        _buildFormField(_dogPortController, Icons.tag_rounded, 'CMS Port', keyboardType: TextInputType.number),
        const SizedBox(height: 14),
        SizedBox(height: 44, child: OutlinedButton.icon(
          onPressed: _isDogConnecting ? null : () {
            final host = _dogHostController.text.trim();
            final port = int.tryParse(_dogPortController.text.trim()) ?? 13145;
            _connectToDogDirect(host, port);
          },
          icon: Icon(_isDogConnecting ? Icons.hourglass_top_rounded : Icons.link_rounded, size: 16),
          label: Text(_isDogConnecting ? '连接中...' : '直连 Dog Board'),
          style: OutlinedButton.styleFrom(
            foregroundColor: AppColors.warning,
            side: BorderSide(color: AppColors.warning.withValues(alpha: 0.5)),
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          ),
        )),
      ]),
    );
  }

  Widget _formLabel(String text) => Text(text, style: TextStyle(
    fontSize: 13, fontWeight: FontWeight.w600, color: context.titleColor));

  Widget _buildIpField(BuildContext context) {
    final hasText = _hostController.text.isNotEmpty;
    return TextField(
      controller: _hostController,
      keyboardType: TextInputType.number,
      style: TextStyle(fontSize: 15, fontWeight: FontWeight.w500, color: context.titleColor, fontFamily: 'monospace'),
      onChanged: (_) => setState(() {}),
      decoration: InputDecoration(
        prefixIcon: Icon(Icons.computer_rounded, size: 18, color: context.subtitleColor),
        hintText: '192.168.1.45',
        hintStyle: TextStyle(color: context.hintColor, fontFamily: 'monospace'),
        suffixIcon: hasText ? Container(
          width: 12, height: 12,
          margin: const EdgeInsets.all(16),
          decoration: const BoxDecoration(color: AppColors.success, shape: BoxShape.circle),
        ) : null,
      ),
    );
  }

  Widget _buildFormField(TextEditingController ctrl, IconData icon, String hint,
      {TextInputType keyboardType = TextInputType.text}) {
    return TextField(
      controller: ctrl,
      keyboardType: keyboardType,
      textInputAction: TextInputAction.next,
      style: TextStyle(fontSize: 15, fontWeight: FontWeight.w500, color: context.titleColor),
      decoration: InputDecoration(
        prefixIcon: Icon(icon, size: 18, color: context.subtitleColor),
        hintText: hint,
        hintStyle: TextStyle(color: context.hintColor),
      ),
    );
  }

  Widget _buildProtocolDropdown(BuildContext context) {
    const labels = ['gRPC', 'WebSocket'];
    const icons = [Icons.hub_rounded, Icons.cable_rounded];
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: context.borderColor),
        color: context.isDark ? Colors.white.withValues(alpha: 0.04) : Colors.white,
      ),
      child: GestureDetector(
        onTap: () => setState(() => _protocol = (_protocol + 1) % 2),
        child: Row(children: [
          Icon(icons[_protocol], size: 16, color: context.subtitleColor),
          const SizedBox(width: 8),
          Text(labels[_protocol], style: TextStyle(fontSize: 13, color: context.titleColor)),
          const Spacer(),
          Icon(Icons.keyboard_arrow_down_rounded, size: 18, color: context.subtitleColor),
        ]),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  BOTTOM BAR — global actions
  // ═══════════════════════════════════════════════════════════
  Widget _buildBottomBar(BuildContext context, bool dark) {
    return Container(
      padding: const EdgeInsets.fromLTRB(24, 8, 24, 8),
      child: Row(children: [
        // Cancel
        TextButton.icon(
          onPressed: () => Navigator.pop(context),
          icon: Icon(Icons.arrow_back_rounded, size: 16, color: context.subtitleColor),
          label: Text('Cancel', style: TextStyle(color: context.subtitleColor)),
        ),
        const Spacer(),
        TextButton(
          onPressed: () {},
          child: Text('Troubleshoot', style: TextStyle(color: context.subtitleColor, fontWeight: FontWeight.w500)),
        ),
        const SizedBox(width: 8),
        ElevatedButton.icon(
          onPressed: _startNetworkScan,
          icon: const Icon(Icons.refresh_rounded, size: 16),
          label: const Text('Scan Again'),
          style: ElevatedButton.styleFrom(
            backgroundColor: AppColors.primary,
            foregroundColor: Colors.white,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
            padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
          ),
        ),
      ]),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  SHARED WIDGETS
  // ═══════════════════════════════════════════════════════════

  Widget _buildShimmerItem() {
    final dark = context.isDark;
    return Shimmer.fromColors(
      baseColor: dark ? Colors.white.withValues(alpha: 0.06) : const Color(0xFFEEEBFF),
      highlightColor: dark ? Colors.white.withValues(alpha: 0.12) : const Color(0xFFF8F6FF),
      child: Container(
        height: 76, margin: const EdgeInsets.only(bottom: 12),
        decoration: BoxDecoration(color: Colors.white, borderRadius: BorderRadius.circular(16)),
      ),
    );
  }

  Widget _buildEmptyState(IconData icon, String title, String subtitle) {
    return Center(child: Column(mainAxisAlignment: MainAxisAlignment.center, children: [
      Container(
        width: 80, height: 80,
        decoration: BoxDecoration(
          color: AppColors.primary.withValues(alpha: 0.06), shape: BoxShape.circle),
        child: Icon(icon, size: 40, color: AppColors.primary.withValues(alpha: 0.4)),
      ),
      const SizedBox(height: 20),
      Text(title, style: TextStyle(fontSize: 17, fontWeight: FontWeight.w700, color: context.titleColor)),
      const SizedBox(height: 6),
      Text(subtitle, style: TextStyle(fontSize: 14, color: context.subtitleColor)),
    ]));
  }
}

// ═══════════════════════════════════════════════════════════════
//  Device Card
// ═══════════════════════════════════════════════════════════════

class _DeviceCard extends StatelessWidget {
  final IconData icon;
  final Color iconBg;
  final Color iconColor;
  final String title;
  final String subtitle;
  final String? signalLabel;
  final Color? signalColor;
  final String actionLabel;
  final IconData? actionIcon;
  final bool actionFilled;
  final VoidCallback? onAction;

  const _DeviceCard({
    required this.icon, required this.iconBg, required this.iconColor,
    required this.title, required this.subtitle,
    this.signalLabel, this.signalColor,
    required this.actionLabel, this.actionIcon,
    this.actionFilled = false, this.onAction,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: context.isDark ? Colors.white.withValues(alpha: 0.04) : Colors.white.withValues(alpha: 0.7),
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: context.borderColor.withValues(alpha: 0.5)),
      ),
      child: Row(children: [
        Container(
          width: 44, height: 44,
          decoration: BoxDecoration(color: iconBg, borderRadius: BorderRadius.circular(14)),
          child: Icon(icon, color: iconColor, size: 22),
        ),
        const SizedBox(width: 14),
        Expanded(child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Text(title, style: TextStyle(fontSize: 15, fontWeight: FontWeight.w600, color: context.titleColor)),
          const SizedBox(height: 2),
          Row(children: [
            Flexible(child: Text(subtitle,
              style: TextStyle(fontSize: 12, color: context.subtitleColor),
              overflow: TextOverflow.ellipsis)),
            if (signalLabel != null) ...[
              const SizedBox(width: 8),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                decoration: BoxDecoration(
                  color: (signalColor ?? AppColors.success).withValues(alpha: 0.1),
                  borderRadius: BorderRadius.circular(6),
                ),
                child: Text(signalLabel!, style: TextStyle(
                  fontSize: 10, fontWeight: FontWeight.w700,
                  color: signalColor ?? AppColors.success,
                )),
              ),
            ],
          ]),
        ])),
        const SizedBox(width: 12),
        if (actionFilled)
          ElevatedButton(
            onPressed: onAction,
            style: ElevatedButton.styleFrom(
              backgroundColor: AppColors.primary,
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(horizontal: 16),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
              minimumSize: const Size(0, 36),
            ),
            child: Text(actionLabel, style: const TextStyle(fontSize: 13)),
          )
        else
          OutlinedButton(
            onPressed: onAction,
            style: OutlinedButton.styleFrom(
              foregroundColor: context.titleColor,
              side: BorderSide(color: context.borderColor),
              padding: const EdgeInsets.symmetric(horizontal: 16),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
              minimumSize: const Size(0, 36),
            ),
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Text(actionLabel, style: const TextStyle(fontSize: 13)),
              if (actionIcon != null) ...[
                const SizedBox(width: 4),
                Icon(actionIcon!, size: 14),
              ],
            ]),
          ),
      ]),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Bluetooth Radar Animation
// ═══════════════════════════════════════════════════════════════

class _BluetoothRadar extends StatefulWidget {
  final bool isScanning;
  const _BluetoothRadar({required this.isScanning});

  @override
  State<_BluetoothRadar> createState() => _BluetoothRadarState();
}

class _BluetoothRadarState extends State<_BluetoothRadar>
    with SingleTickerProviderStateMixin {
  late AnimationController _ctrl;

  @override
  void initState() {
    super.initState();
    _ctrl = AnimationController(vsync: this, duration: const Duration(seconds: 3))
      ..repeat();
  }

  @override
  void dispose() {
    _ctrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: AnimatedBuilder(
        animation: _ctrl,
        builder: (_, __) {
          return CustomPaint(
            size: const Size(280, 280),
            painter: _RadarPainter(
              progress: _ctrl.value,
              isScanning: widget.isScanning,
              isDark: context.isDark,
            ),
            child: SizedBox(
              width: 280, height: 280,
              child: Center(child: Container(
                width: 72, height: 72,
                decoration: BoxDecoration(
                  gradient: AppColors.brandGradient,
                  shape: BoxShape.circle,
                  boxShadow: [BoxShadow(
                    color: AppColors.primary.withValues(alpha: 0.3),
                    blurRadius: 20,
                  )],
                ),
                child: const Icon(Icons.bluetooth_rounded, color: Colors.white, size: 36),
              )),
            ),
          );
        },
      ),
    );
  }
}

class _RadarPainter extends CustomPainter {
  final double progress;
  final bool isScanning;
  final bool isDark;

  _RadarPainter({required this.progress, required this.isScanning, required this.isDark});

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final maxRadius = size.width / 2;

    // Static rings
    for (int i = 1; i <= 3; i++) {
      final r = maxRadius * (i / 3);
      canvas.drawCircle(center, r, Paint()
        ..color = isDark
            ? Colors.white.withValues(alpha: 0.04)
            : AppColors.primary.withValues(alpha: 0.06)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1);
    }

    // Animated pulse rings
    if (isScanning) {
      for (int i = 0; i < 3; i++) {
        final p = (progress + i * 0.33) % 1.0;
        final r = maxRadius * p;
        final alpha = (1.0 - p) * 0.25;
        canvas.drawCircle(center, r, Paint()
          ..color = AppColors.primary.withValues(alpha: alpha)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2);
      }
    }

    // Small device dots
    if (isScanning) {
      final dotPositions = [
        Offset(center.dx + maxRadius * 0.4, center.dy - maxRadius * 0.25),
        Offset(center.dx - maxRadius * 0.3, center.dy + maxRadius * 0.35),
      ];
      for (final pos in dotPositions) {
        canvas.drawCircle(pos, 4, Paint()
          ..color = AppColors.primary.withValues(alpha: 0.4 + 0.3 * math.sin(progress * math.pi * 2))
          ..style = PaintingStyle.fill);
      }
    }
  }

  @override
  bool shouldRepaint(covariant _RadarPainter old) =>
      progress != old.progress || isScanning != old.isScanning;
}
