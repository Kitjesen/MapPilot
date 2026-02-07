import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../services/robot_connection_provider.dart';
import '../theme/app_theme.dart';
import 'status_screen.dart';
import 'control_screen.dart';
import 'map_screen.dart';
import 'events_screen.dart';
import 'settings_screen.dart';
import '../main.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen>
    with SingleTickerProviderStateMixin {
  int _currentIndex = 0;

  late AnimationController _navAnimController;
  late Animation<double> _navSlide;

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
    SystemChrome.setSystemUIOverlayStyle(AppTheme.systemOverlay);

    // Nav bar entrance animation
    _navAnimController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 600),
    );
    _navSlide = Tween<double>(begin: 100, end: 0).animate(
      CurvedAnimation(parent: _navAnimController, curve: Curves.easeOutCubic),
    );
    _navAnimController.forward();
  }

  @override
  void dispose() {
    _navAnimController.dispose();
    super.dispose();
  }

  // index: 0=Status, 1=Control(push), 2=Map, 3=Events, 4=Settings
  int _indexedStackIndex() {
    if (_currentIndex == 0) return 0;
    if (_currentIndex == 2) return 1;
    if (_currentIndex == 3) return 2;
    if (_currentIndex == 4) return 3;
    return 0;
  }

  void _onTabSelected(int index) {
    if (index == 1) {
      HapticFeedback.selectionClick();
      Navigator.of(context).push(
        MaterialPageRoute(builder: (context) => const ControlScreen()),
      );
    } else {
      HapticFeedback.selectionClick();
      setState(() => _currentIndex = index);
    }
  }

  Future<void> _handleDisconnect() async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('断开连接'),
        content: const Text('确定要断开与机器人的连接吗？'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('取消'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
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
          IndexedStack(
            index: _indexedStackIndex(),
            children: const [
              StatusScreen(),
              MapScreen(),
              EventsScreen(),
              SettingsScreen(),
            ],
          ),
          _buildConnectionBanner(),
          _buildFloatingNavBar(context),
        ],
      ),
    );
  }

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
            bgColor = AppColors.warning;
            text = '正在重新连接...';
            icon = Icons.sync;
          case ConnectionStatus.error:
            bgColor = AppColors.error;
            text = context.read<RobotConnectionProvider>().errorMessage ??
                '连接错误';
            icon = Icons.error_outline;
          default:
            bgColor = AppColors.textTertiary;
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
              padding:
                  const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
              decoration: BoxDecoration(
                color: bgColor.withOpacity(0.9),
                borderRadius: BorderRadius.circular(14),
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
    return AnimatedBuilder(
      animation: _navSlide,
      builder: (context, child) => Positioned(
        left: 24,
        right: 24,
        bottom: MediaQuery.of(context).padding.bottom + 12 + _navSlide.value,
        child: Opacity(
          opacity: (1 - _navSlide.value / 100).clamp(0, 1),
          child: child!,
        ),
      ),
      child: Container(
        height: 68,
        decoration: BoxDecoration(
          color: AppColors.surface.withOpacity(0.95),
          borderRadius: BorderRadius.circular(28),
          border: Border.all(color: AppColors.border.withOpacity(0.3)),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.3),
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
            _buildNavItem(
                3, Icons.notifications_outlined, Icons.notifications, 'Events'),
            _buildNavItem(
                4, Icons.settings_outlined, Icons.settings, 'Settings'),
            _buildDisconnectButton(),
          ],
        ),
      ),
    );
  }

  Widget _buildNavItem(
      int index, IconData icon, IconData activeIcon, String label) {
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
                color: AppColors.lime.withOpacity(0.12),
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
                  ? AppColors.lime
                  : isControl
                      ? AppColors.lime.withOpacity(0.6)
                      : AppColors.textTertiary,
            ),
            if (isSelected) ...[
              const SizedBox(height: 2),
              Container(
                width: 4,
                height: 4,
                decoration: const BoxDecoration(
                  color: AppColors.lime,
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
        child: Icon(
          Icons.power_settings_new,
          size: 22,
          color: AppColors.error.withOpacity(0.6),
        ),
      ),
    );
  }
}
