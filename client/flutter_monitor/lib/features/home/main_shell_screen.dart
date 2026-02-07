import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/features/home/home_screen.dart';
import 'package:flutter_monitor/features/status/status_screen.dart';
import 'package:flutter_monitor/features/map/map_screen.dart';
import 'package:flutter_monitor/features/events/events_screen.dart';
import 'package:flutter_monitor/features/settings/app_settings_screen.dart';

/// Main app shell with bottom navigation bar.
/// Wraps Home, Status, Map, Events, and Settings tabs.
class MainShellScreen extends StatefulWidget {
  const MainShellScreen({super.key});

  @override
  State<MainShellScreen> createState() => _MainShellScreenState();
}

class _MainShellScreenState extends State<MainShellScreen> {
  int _currentIndex = 0;

  final List<Widget> _screens = const [
    HomeScreen(),
    StatusScreen(),
    MapScreen(),
    EventsScreen(),
    AppSettingsScreen(),
  ];

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final isConnected = provider.isConnected;
    final isDogConnected = provider.isDogConnected;

    return Scaffold(
      body: IndexedStack(
        index: _currentIndex,
        children: _screens,
      ),
      bottomNavigationBar: Container(
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkSurface : Colors.white,
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(isDark ? 0.3 : 0.06),
              blurRadius: 20,
              offset: const Offset(0, -4),
            ),
          ],
          borderRadius: const BorderRadius.vertical(top: Radius.circular(24)),
        ),
        child: ClipRRect(
          borderRadius: const BorderRadius.vertical(top: Radius.circular(24)),
          child: NavigationBar(
            selectedIndex: _currentIndex,
            onDestinationSelected: (index) {
              HapticFeedback.selectionClick();
              setState(() => _currentIndex = index);
            },
            backgroundColor: Colors.transparent,
            surfaceTintColor: Colors.transparent,
            elevation: 0,
            height: 60,
            labelBehavior: NavigationDestinationLabelBehavior.alwaysHide,
            indicatorColor: AppColors.primary.withOpacity(isDark ? 0.15 : 0.1),
            indicatorShape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(16),
            ),
            destinations: [
              const NavigationDestination(
                icon: Icon(Icons.home_outlined),
                selectedIcon: Icon(Icons.home_rounded),
                label: '首页',
              ),
              NavigationDestination(
                icon: Badge(
                  isLabelVisible: isConnected || isDogConnected,
                  smallSize: 8,
                  backgroundColor:
                      isConnected ? AppColors.success : AppColors.warning,
                  child: const Icon(Icons.dashboard_outlined),
                ),
                selectedIcon: Badge(
                  isLabelVisible: isConnected || isDogConnected,
                  smallSize: 8,
                  backgroundColor:
                      isConnected ? AppColors.success : AppColors.warning,
                  child: const Icon(Icons.dashboard_rounded),
                ),
                label: '状态',
              ),
              const NavigationDestination(
                icon: Icon(Icons.map_outlined),
                selectedIcon: Icon(Icons.map_rounded),
                label: '地图',
              ),
              const NavigationDestination(
                icon: Icon(Icons.notifications_outlined),
                selectedIcon: Icon(Icons.notifications_rounded),
                label: '事件',
              ),
              const NavigationDestination(
                icon: Icon(Icons.settings_outlined),
                selectedIcon: Icon(Icons.settings_rounded),
                label: '设置',
              ),
            ],
          ),
        ),
      ),
    );
  }
}
