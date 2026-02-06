import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../services/robot_connection_provider.dart';
import 'status_screen.dart';
import 'control_screen.dart';
import 'map_screen.dart';
import 'events_screen.dart';
import '../main.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  int _currentIndex = 0;

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
  }

  void _onTabSelected(int index) {
    if (index == 1) {
      // Control 页面以独立路由打开
      HapticFeedback.selectionClick();
      Navigator.of(context).push(
        MaterialPageRoute(
          builder: (context) => const ControlScreen(),
        ),
      );
    } else {
      HapticFeedback.selectionClick();
      setState(() {
        _currentIndex = index;
      });
    }
  }

  Future<void> _handleDisconnect() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('断开连接'),
        content: const Text('确定要断开与机器人的连接吗？'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('取消'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            style: TextButton.styleFrom(foregroundColor: Colors.red),
            child: const Text('断开'),
          ),
        ],
      ),
    );

    if (confirmed == true && mounted) {
      final provider = context.read<RobotConnectionProvider>();
      await provider.disconnect();
      if (mounted) {
        Navigator.of(context).pushAndRemoveUntil(
          MaterialPageRoute(builder: (_) => const ConnectionScreen()),
          (_) => false,
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBody: true,
      body: Stack(
        children: [
          // 使用 IndexedStack 保持屏幕状态
          IndexedStack(
            index: _currentIndex > 1 ? _currentIndex - 1 : _currentIndex,
            children: const [
              StatusScreen(),
              MapScreen(),
              EventsScreen(),
            ],
          ),

          // 连接状态横幅（重连中时显示）
          _buildConnectionBanner(),

          // 浮动导航栏
          _buildFloatingNavBar(context),
        ],
      ),
    );
  }

  /// 连接状态横幅
  Widget _buildConnectionBanner() {
    return Selector<RobotConnectionProvider, ConnectionStatus>(
      selector: (_, p) => p.status,
      builder: (context, status, _) {
        if (status == ConnectionStatus.connected) {
          return const SizedBox.shrink();
        }

        Color bgColor;
        String text;
        IconData icon;
        switch (status) {
          case ConnectionStatus.reconnecting:
            bgColor = const Color(0xFFFF9500);
            text = '正在重新连接...';
            icon = Icons.sync;
          case ConnectionStatus.error:
            bgColor = const Color(0xFFFF3B30);
            text = context.read<RobotConnectionProvider>().errorMessage ?? '连接错误';
            icon = Icons.error_outline;
          default:
            bgColor = Colors.grey;
            text = '未连接';
            icon = Icons.cloud_off;
        }

        return Positioned(
          top: MediaQuery.of(context).padding.top,
          left: 0,
          right: 0,
          child: Material(
            color: Colors.transparent,
            child: Container(
              margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
              decoration: BoxDecoration(
                color: bgColor.withOpacity(0.9),
                borderRadius: BorderRadius.circular(12),
                boxShadow: [
                  BoxShadow(
                    color: bgColor.withOpacity(0.3),
                    blurRadius: 12,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: Row(
                children: [
                  Icon(icon, size: 18, color: Colors.white),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Text(
                      text,
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 13,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                  ),
                  if (status == ConnectionStatus.reconnecting)
                    const SizedBox(
                      width: 16,
                      height: 16,
                      child: CircularProgressIndicator(
                        strokeWidth: 2,
                        color: Colors.white,
                      ),
                    ),
                ],
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _buildFloatingNavBar(BuildContext context) {
    return Positioned(
      left: 24,
      right: 24,
      bottom: MediaQuery.of(context).padding.bottom + 12,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(24),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 24, sigmaY: 24),
          child: Container(
            height: 64,
            decoration: BoxDecoration(
              color: Colors.white.withOpacity(0.82),
              borderRadius: BorderRadius.circular(24),
              border: Border.all(
                color: Colors.white.withOpacity(0.4),
                width: 0.5,
              ),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.12),
                  blurRadius: 24,
                  offset: const Offset(0, 8),
                ),
              ],
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                _buildNavItem(
                    0, Icons.dashboard_outlined, Icons.dashboard, 'Status'),
                _buildNavItem(
                    1, Icons.gamepad_outlined, Icons.gamepad, 'Control'),
                _buildNavItem(2, Icons.map_outlined, Icons.map, 'Map'),
                _buildNavItem(3, Icons.notifications_outlined,
                    Icons.notifications, 'Events'),
                // 断开连接按钮
                _buildDisconnectButton(),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildNavItem(
      int index, IconData icon, IconData activeIcon, String label) {
    // Control tab 永远不会 "selected"
    final isSelected = index != 1 && _currentIndex == index;
    final isControl = index == 1;

    return GestureDetector(
      onTap: () => _onTabSelected(index),
      behavior: HitTestBehavior.opaque,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
        decoration: isSelected
            ? BoxDecoration(
                color: const Color(0xFF007AFF).withOpacity(0.12),
                borderRadius: BorderRadius.circular(16),
              )
            : null,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              isSelected ? activeIcon : icon,
              size: 22,
              color: isSelected
                  ? const Color(0xFF007AFF)
                  : isControl
                      ? const Color(0xFFAF52DE)
                      : Colors.black.withOpacity(0.35),
            ),
            if (isSelected) ...[
              const SizedBox(height: 2),
              Container(
                width: 4,
                height: 4,
                decoration: const BoxDecoration(
                  color: Color(0xFF007AFF),
                  shape: BoxShape.circle,
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildDisconnectButton() {
    return GestureDetector(
      onTap: _handleDisconnect,
      behavior: HitTestBehavior.opaque,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 8),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              Icons.power_settings_new,
              size: 22,
              color: Colors.red.withOpacity(0.6),
            ),
          ],
        ),
      ),
    );
  }
}
