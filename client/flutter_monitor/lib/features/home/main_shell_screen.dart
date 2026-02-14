import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:flutter_monitor/features/home/home_screen.dart';
import 'package:flutter_monitor/features/status/status_screen.dart';
import 'package:flutter_monitor/features/map/map_screen.dart';
import 'package:flutter_monitor/features/events/events_screen.dart';
import 'package:flutter_monitor/features/settings/app_settings_screen.dart';

/// Adaptive main navigation: mobile→bottom nav, tablet/desktop→sidebar
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

  // Nav items — generated per locale
  static List<_NavDef> _mainNavItems(LocaleProvider l) => [
    _NavDef(Icons.dashboard_outlined, Icons.dashboard_rounded, l.tr('首页', 'Dashboard')),
    _NavDef(Icons.grid_view_outlined, Icons.grid_view_rounded, l.tr('模块', 'Modules')),
    _NavDef(Icons.map_outlined, Icons.map_rounded, l.tr('地图', 'Map')),
    _NavDef(Icons.history_rounded, Icons.history_rounded, l.tr('历史', 'History')),
  ];

  static List<_NavDef> _allNavItems(LocaleProvider l) => [
    _NavDef(Icons.dashboard_outlined, Icons.dashboard_rounded, l.tr('首页', 'Home')),
    _NavDef(Icons.grid_view_outlined, Icons.grid_view_rounded, l.tr('状态', 'Status')),
    _NavDef(Icons.map_outlined, Icons.map_rounded, l.tr('地图', 'Map')),
    _NavDef(Icons.history_rounded, Icons.history_rounded, l.tr('事件', 'Events')),
    _NavDef(Icons.settings_outlined, Icons.settings_rounded, l.tr('设置', 'Settings')),
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
    // 收窄 select: 仅在连接状态变化时重建，心跳 RTT 等不触发
    final isConnected = context.select<RobotConnectionProvider, bool>(
      (p) => p.isConnected,
    );
    final isDogConnected = context.select<RobotConnectionProvider, bool>(
      (p) => p.isDogConnected,
    );
    final locale = context.watch<LocaleProvider>();
    final useSide = context.useSideNav;

    // Listen for tab-switch notifications from children
    return NotificationListener<MainShellTabNotification>(
      onNotification: (n) {
        _onTap(n.tabIndex);
        return true;
      },
        child: Scaffold(
        extendBody: !useSide,
        body: useSide
            ? _buildDesktopShell(context, isConnected, isDogConnected, locale)
            : IndexedStack(index: _currentIndex, children: _screens),
        bottomNavigationBar: useSide
            ? null
            : _buildBottomNav(context, isConnected, isDogConnected, locale),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════
  //  DESKTOP SHELL — Glass panel with sidebar
  // ═══════════════════════════════════════════════════════════════
  Widget _buildDesktopShell(
      BuildContext context, bool isConnected, bool isDogConnected, LocaleProvider locale) {
    final dark = context.isDark;

    return Stack(
      children: [
        // ── Background gradient ──
        Positioned.fill(
          child: Container(
            decoration: BoxDecoration(gradient: context.bgGradient),
          ),
        ),

        // ── Decorative blobs ──
        if (!dark) ...[
          _posBlob(top: -120, left: -100, color: const Color(0x30C4B5FD), size: 450),
          _posBlob(bottom: -120, right: -100, color: const Color(0x2893C5FD), size: 450),
          _posBlob(top: 300, left: 400, color: const Color(0x22FBCFE8), size: 300),
        ] else ...[
          _posBlob(top: -120, left: -100, color: const Color(0x10C4B5FD), size: 450),
          _posBlob(bottom: -120, right: -100, color: const Color(0x1093C5FD), size: 450),
        ],

        // ── Floating glass panel ──
        Positioned.fill(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(40),
              child: BackdropFilter(
                filter: ImageFilter.blur(sigmaX: 24, sigmaY: 24),
                child: Container(
                  decoration: BoxDecoration(
                    color: dark
                        ? Colors.white.withValues(alpha: 0.06)
                        : Colors.white.withValues(alpha: 0.5),
                    borderRadius: BorderRadius.circular(40),
                    border: Border.all(
                      color: dark
                          ? Colors.white.withValues(alpha: 0.08)
                          : Colors.white.withValues(alpha: 0.65),
                    ),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.black.withValues(alpha: dark ? 0.2 : 0.08),
                        blurRadius: 40,
                        offset: const Offset(0, 10),
                      ),
                    ],
                  ),
                  child: Row(
                    children: [
                      // ── Sidebar ──
                      _buildSidebar(context, isConnected, isDogConnected, dark, locale),

                      // ── Content (transparent scaffold bg inside shell) ──
                      Expanded(
                        child: ClipRRect(
                          borderRadius: const BorderRadius.only(
                            topRight: Radius.circular(40),
                            bottomRight: Radius.circular(40),
                          ),
                          child: Theme(
                            data: Theme.of(context).copyWith(
                              scaffoldBackgroundColor: Colors.transparent,
                              appBarTheme: Theme.of(context).appBarTheme.copyWith(
                                backgroundColor: Colors.transparent,
                                surfaceTintColor: Colors.transparent,
                                elevation: 0,
                                scrolledUnderElevation: 0,
                              ),
                            ),
                            child: IndexedStack(
                              index: _currentIndex,
                              children: _screens,
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════════
  //  SIDEBAR — Glass, 96px wide, matching HTML reference
  // ═══════════════════════════════════════════════════════════════
  Widget _buildSidebar(
      BuildContext context, bool isConnected, bool isDogConnected, bool dark, LocaleProvider locale) {
    return Container(
      width: 96,
      decoration: BoxDecoration(
        color: dark
            ? Colors.white.withValues(alpha: 0.03)
            : Colors.white.withValues(alpha: 0.2),
        border: Border(
          right: BorderSide(
            color: dark
                ? Colors.white.withValues(alpha: 0.06)
                : Colors.white.withValues(alpha: 0.3),
          ),
        ),
      ),
      child: SafeArea(
        child: Column(
          children: [
            const SizedBox(height: 32),

            // ── Brand Logo — "DS" monogram ──
            Container(
              width: 56,
              height: 56,
              decoration: BoxDecoration(
                gradient: AppColors.brandGradient,
                borderRadius: BorderRadius.circular(20),
                boxShadow: [
                  BoxShadow(
                    color: AppColors.primary.withValues(alpha: 0.35),
                    blurRadius: 16,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: const Center(
                child: Text(
                  'DS',
                  style: TextStyle(
                    fontSize: 22,
                    fontWeight: FontWeight.w900,
                    color: Colors.white,
                    letterSpacing: 1,
                    height: 1,
                  ),
                ),
              ),
            ),
            const SizedBox(height: 4),
            Text(
              'NAV',
              style: TextStyle(
                fontSize: 9,
                fontWeight: FontWeight.w700,
                letterSpacing: 3,
                color: dark
                    ? AppColors.textSecondaryDark
                    : AppColors.textSecondary,
              ),
            ),

            const SizedBox(height: 32),

            // ── Main nav items (0-3) ──
            Expanded(
              child: Builder(builder: (context) {
                final items = _mainNavItems(locale);
                return Column(
                  children: [
                    for (int i = 0; i < items.length; i++) ...[
                      _SideNavItem(
                        icon: items[i].icon,
                        activeIcon: items[i].activeIcon,
                        label: items[i].label,
                        isActive: _currentIndex == i,
                        onTap: () => _onTap(i),
                      ),
                      SizedBox(height: i < items.length - 1 ? 20 : 0),
                    ],
                  ],
                );
              }),
            ),

            // ── Settings (bottom, separated) ──
            _SideNavItem(
              icon: Icons.settings_outlined,
              activeIcon: Icons.settings_rounded,
              label: locale.tr('设置', 'Settings'),
              isActive: _currentIndex == 4,
              onTap: () => _onTap(4),
            ),
            const SizedBox(height: 16),

            // ── Language toggle ──
            GestureDetector(
              onTap: () {
                HapticFeedback.selectionClick();
                locale.toggle();
              },
              child: Tooltip(
                message: locale.tr('切换至英文', 'Switch to Chinese'),
                child: Container(
                  width: 40,
                  height: 40,
                  decoration: BoxDecoration(
                    color: dark
                        ? Colors.white.withValues(alpha: 0.08)
                        : Colors.white.withValues(alpha: 0.5),
                    borderRadius: BorderRadius.circular(12),
                    border: Border.all(
                      color: dark
                          ? Colors.white.withValues(alpha: 0.1)
                          : Colors.white.withValues(alpha: 0.4),
                    ),
                  ),
                  child: Center(
                    child: Text(
                      locale.isEn ? 'EN' : '中',
                      style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w700,
                        color: AppColors.primary,
                      ),
                    ),
                  ),
                ),
              ),
            ),
            const SizedBox(height: 16),

            // ── User avatar with status dot ──
            Stack(
              alignment: Alignment.bottomRight,
              children: [
                Container(
                  width: 48,
                  height: 48,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    color: dark
                        ? Colors.white.withValues(alpha: 0.08)
                        : const Color(0xFFE8E5F0),
                    border: Border.all(color: Colors.white, width: 2),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.black.withValues(alpha: 0.06),
                        blurRadius: 8,
                      ),
                    ],
                  ),
                  child: Icon(
                    Icons.person_rounded,
                    size: 24,
                    color: dark ? Colors.white54 : context.subtitleColor,
                  ),
                ),
                Positioned(
                  right: 0,
                  bottom: 0,
                  child: Container(
                    width: 14,
                    height: 14,
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: isConnected
                          ? AppColors.success
                          : isDogConnected
                              ? AppColors.connecting
                              : AppColors.offline,
                      border: Border.all(color: Colors.white, width: 2.5),
                    ),
                  ),
                ),
              ],
            ),

            const SizedBox(height: 24),
          ],
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════
  //  BOTTOM NAV — Mobile floating pill
  // ═══════════════════════════════════════════════════════════════
  Widget _buildBottomNav(
      BuildContext context, bool isConnected, bool isDogConnected, LocaleProvider locale) {
    final dark = context.isDark;
    final items = _allNavItems(locale);
    return SafeArea(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(24, 0, 24, 12),
        child: Container(
          height: 64,
          decoration: BoxDecoration(
            color: dark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(32),
            boxShadow: AppShadows.elevated(isDark: dark),
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              for (int i = 0; i < items.length; i++)
                _BottomNavItem(
                  icon: items[i].icon,
                  activeIcon: items[i].activeIcon,
                  label: items[i].label,
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

  // ── Blob helper ──
  Widget _posBlob({double? top, double? bottom, double? left, double? right,
      required Color color, required double size}) {
    return Positioned(
      top: top, bottom: bottom, left: left, right: right,
      child: Container(
        width: size, height: size,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          gradient: RadialGradient(colors: [color, color.withValues(alpha: 0)]),
        ),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────
//  Helper types
// ─────────────────────────────────────────────────────────────────

class _NavDef {
  final IconData icon;
  final IconData activeIcon;
  final String label;
  const _NavDef(this.icon, this.activeIcon, this.label);
}

// ─────────────────────────────────────────────────────────────────
//  Sidebar nav item — glass active state (white bg + shadow)
// ─────────────────────────────────────────────────────────────────

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
    final dark = context.isDark;
    return GestureDetector(
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
                ? (dark ? Colors.white.withValues(alpha: 0.12) : Colors.white.withValues(alpha: 0.8))
                : Colors.transparent,
            borderRadius: BorderRadius.circular(14),
            boxShadow: isActive
                ? [BoxShadow(
                    color: Colors.black.withValues(alpha: dark ? 0.15 : 0.08),
                    blurRadius: 10,
                    offset: const Offset(0, 2),
                  )]
                : null,
          ),
          child: Icon(
            isActive ? activeIcon : icon,
            size: 24,
            color: isActive
                ? AppColors.primary
                : (dark ? AppColors.textSecondaryDark : AppColors.textSecondary),
          ),
        ),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────
//  Bottom nav item (mobile)
// ─────────────────────────────────────────────────────────────────

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
                    color: isActive ? AppColors.primary : context.subtitleColor,
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
