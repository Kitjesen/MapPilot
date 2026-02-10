import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/features/home/home_screen.dart';
import 'package:flutter_monitor/features/status/status_screen.dart';
import 'package:flutter_monitor/features/map/map_screen.dart';
import 'package:flutter_monitor/features/files/file_browser_screen.dart';
import 'package:flutter_monitor/features/settings/app_settings_screen.dart';

/// 自适应主导航: 手机→底部导航栏, 平板/桌面→侧边栏
///
/// Tab 结构:
///   0 — 首页 (Dashboard): 连接状态 + 快速操作 + 事件摘要
///   1 — 监控 (Monitor):  实时遥测 (StatusScreen)
///   2 — 控制 (Control):  地图导航 + 遥控 (MapScreen)
///   3 — 文件 (Files):    远程文件管理 (FileBrowserScreen)
///   4 — 设置 (Settings): 应用设置
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
    FileBrowserScreen(embedded: true),
    AppSettingsScreen(),
  ];

  static const _navItems = [
    _NavDef(Icons.dashboard_outlined, Icons.dashboard_rounded, '首页'),
    _NavDef(Icons.monitor_heart_outlined, Icons.monitor_heart_rounded, '监控'),
    _NavDef(Icons.map_outlined, Icons.map_rounded, '控制'),
    _NavDef(Icons.folder_outlined, Icons.folder_rounded, '文件'),
    _NavDef(Icons.settings_outlined, Icons.settings_rounded, '设置'),
  ];

  @override
  void initState() {
    super.initState();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }

  void _onTap(int index) {
    HapticFeedback.selectionClick();
    setState(() => _currentIndex = index);
  }

  @override
  Widget build(BuildContext context) {
    final provider = context.watch<RobotConnectionProvider>();
    final isConnected = provider.isConnected;
    final isDogConnected = provider.isDogConnected;
    final useSide = context.useSideNav;

    // Listen for tab-switch notifications from child widgets (e.g. HomeScreen quick actions)
    return NotificationListener<MainShellTabNotification>(
      onNotification: (notification) {
        _onTap(notification.tabIndex);
        return true;
      },
      child: Scaffold(
        extendBody: !useSide,
        body: useSide
            ? _buildSideNavLayout(context, isConnected, isDogConnected)
            : IndexedStack(index: _currentIndex, children: _screens),
        bottomNavigationBar: useSide
            ? null
            : _buildBottomNav(context, isConnected, isDogConnected),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════
  //  桌面/平板: 侧边栏 — 紫色主题
  // ═══════════════════════════════════════════════════════════════
  Widget _buildSideNavLayout(
      BuildContext context, bool isConnected, bool isDogConnected) {
    final isDark = context.isDark;

    return Row(
      children: [
        // Custom sidebar
        Container(
          width: 72,
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkSurface : AppColors.lightSidebar,
            border: Border(
              right: BorderSide(
                color: isDark ? AppColors.borderDark : Colors.transparent,
                width: 1,
              ),
            ),
          ),
          child: SafeArea(
            child: Column(
              children: [
                const SizedBox(height: 16),
                // Robot avatar
                _buildRobotAvatar(isConnected, isDogConnected),
                const SizedBox(height: 24),
                // Nav items
                Expanded(
                  child: Column(
                    children: [
                      for (int i = 0; i < _navItems.length; i++)
                        _SideNavItem(
                          icon: _navItems[i].icon,
                          activeIcon: _navItems[i].activeIcon,
                          label: _navItems[i].label,
                          isActive: _currentIndex == i,
                          onTap: () => _onTap(i),
                        ),
                    ],
                  ),
                ),
                // Bottom icons
                _SideNavItem(
                  icon: Icons.settings_outlined,
                  activeIcon: Icons.settings_rounded,
                  label: '设置',
                  isActive: _currentIndex == 4,
                  onTap: () => _onTap(4),
                ),
                const SizedBox(height: 8),
                // Theme toggle dot
                Container(
                  width: 28,
                  height: 28,
                  decoration: BoxDecoration(
                    color: isDark ? AppColors.darkElevated : Colors.white,
                    shape: BoxShape.circle,
                    boxShadow: [AppShadows.light()],
                  ),
                  child: Icon(
                    isDark ? Icons.dark_mode : Icons.light_mode,
                    size: 14,
                    color: AppColors.primary,
                  ),
                ),
                const SizedBox(height: 16),
              ],
            ),
          ),
        ),
        // Content area
        Expanded(
          child: IndexedStack(
            index: _currentIndex,
            children: _screens,
          ),
        ),
      ],
    );
  }

  Widget _buildRobotAvatar(bool isConnected, bool isDogConnected) {
    return Stack(
      alignment: Alignment.bottomRight,
      children: [
        Container(
          width: 44,
          height: 44,
          decoration: BoxDecoration(
            gradient: AppColors.brandGradient,
            borderRadius: BorderRadius.circular(14),
            boxShadow: [
              AppShadows.glow(AppColors.primary, blur: 12),
            ],
          ),
          child: const Icon(Icons.smart_toy, color: Colors.white, size: 22),
        ),
        // Status dot
        Positioned(
          right: -2,
          bottom: -2,
          child: Container(
            width: 14,
            height: 14,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: isConnected
                  ? AppColors.online
                  : isDogConnected
                      ? AppColors.connecting
                      : AppColors.offline,
              border: Border.all(color: Colors.white, width: 2),
            ),
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════════
  //  手机: 底部导航栏 — 浮动胶囊样式
  // ═══════════════════════════════════════════════════════════════
  Widget _buildBottomNav(
      BuildContext context, bool isConnected, bool isDogConnected) {
    final isDark = context.isDark;

    return SafeArea(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(24, 0, 24, 12),
        child: Container(
          height: 64,
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(32),
            boxShadow: AppShadows.elevated(isDark: isDark),
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              for (int i = 0; i < _navItems.length; i++)
                _BottomNavItem(
                  icon: _navItems[i].icon,
                  activeIcon: _navItems[i].activeIcon,
                  label: _navItems[i].label,
                  isActive: _currentIndex == i,
                  badge: i == 1 && (isConnected || isDogConnected),
                  badgeColor: isConnected ? AppColors.online : AppColors.connecting,
                  onTap: () => _onTap(i),
                ),
            ],
          ),
        ),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────
// Helper types
// ─────────────────────────────────────────────────────────────────

class _NavDef {
  final IconData icon;
  final IconData activeIcon;
  final String label;
  const _NavDef(this.icon, this.activeIcon, this.label);
}

/// Sidebar nav item with purple active indicator
class _SideNavItem extends StatelessWidget {
  final IconData icon;
  final IconData activeIcon;
  final String label;
  final bool isActive;
  final VoidCallback onTap;

  const _SideNavItem({
    required this.icon,
    required this.activeIcon,
    required this.label,
    required this.isActive,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: GestureDetector(
        onTap: onTap,
        behavior: HitTestBehavior.opaque,
        child: Tooltip(
          message: label,
          child: AnimatedContainer(
            duration: AppDurations.fast,
            width: 48,
            height: 48,
            decoration: BoxDecoration(
              color: isActive
                  ? (isDark
                      ? AppColors.primary.withValues(alpha: 0.15)
                      : AppColors.primary.withValues(alpha: 0.1))
                  : Colors.transparent,
              borderRadius: BorderRadius.circular(14),
            ),
            child: Icon(
              isActive ? activeIcon : icon,
              size: 22,
              color: isActive
                  ? AppColors.primary
                  : (isDark ? AppColors.textSecondaryDark : AppColors.textSecondary),
            ),
          ),
        ),
      ),
    );
  }
}

/// Bottom nav item with active pill indicator
class _BottomNavItem extends StatelessWidget {
  final IconData icon;
  final IconData activeIcon;
  final String label;
  final bool isActive;
  final bool badge;
  final Color? badgeColor;
  final VoidCallback onTap;

  const _BottomNavItem({
    required this.icon,
    required this.activeIcon,
    required this.label,
    required this.isActive,
    this.badge = false,
    this.badgeColor,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      behavior: HitTestBehavior.opaque,
      child: SizedBox(
        width: 56,
        height: 56,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Stack(
              alignment: Alignment.center,
              children: [
                AnimatedContainer(
                  duration: AppDurations.fast,
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
                  decoration: BoxDecoration(
                    color: isActive
                        ? AppColors.primary.withValues(alpha: 0.1)
                        : Colors.transparent,
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: Icon(
                    isActive ? activeIcon : icon,
                    size: 22,
                    color: isActive
                        ? AppColors.primary
                        : context.subtitleColor,
                  ),
                ),
                if (badge)
                  Positioned(
                    top: 0,
                    right: 4,
                    child: Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        color: badgeColor ?? AppColors.online,
                        shape: BoxShape.circle,
                        border: Border.all(color: context.cardColor, width: 1.5),
                      ),
                    ),
                  ),
              ],
            ),
            const SizedBox(height: 2),
            Text(
              label,
              style: TextStyle(
                fontSize: 10,
                fontWeight: isActive ? FontWeight.w600 : FontWeight.w400,
                color: isActive ? AppColors.primary : context.hintColor,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
