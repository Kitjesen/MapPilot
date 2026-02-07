import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:animations/animations.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/core/services/alert_monitor_service.dart';
import 'package:flutter_monitor/core/services/state_logger_service.dart';
import 'package:flutter_monitor/features/connection/splash_screen.dart';
import 'package:flutter_monitor/features/home/main_shell_screen.dart';
import 'package:flutter_monitor/features/connection/scan_screen.dart';
import 'package:flutter_monitor/features/home/robot_detail_screen.dart';
import 'package:flutter_monitor/features/settings/app_settings_screen.dart';
import 'package:flutter_monitor/features/control/control_screen.dart';

import 'package:flutter_monitor/core/services/notification_service.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
    ),
  );

  // 初始化系统通知服务
  await NotificationService().initialize();

  runApp(const RobotMonitorApp());
}

class RobotMonitorApp extends StatelessWidget {
  const RobotMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => RobotConnectionProvider()),
        ChangeNotifierProvider(create: (_) => ThemeProvider()),
        ChangeNotifierProvider(create: (_) => SettingsPreferences()),
        ChangeNotifierProvider(create: (_) => AlertMonitorService()),
        ChangeNotifierProvider(create: (_) => StateLoggerService()),
      ],
      child: _AppWithBindings(),
    );
  }
}

/// Binds SettingsPreferences -> RobotConnectionProvider after both are created.
class _AppWithBindings extends StatefulWidget {
  @override
  State<_AppWithBindings> createState() => _AppWithBindingsState();
}

class _AppWithBindingsState extends State<_AppWithBindings> {
  bool _bound = false;
  StreamSubscription<AlertRecord>? _alertSub;

  /// 全局 ScaffoldMessenger key，用于从任意位置弹 SnackBar
  final GlobalKey<ScaffoldMessengerState> _messengerKey =
      GlobalKey<ScaffoldMessengerState>();

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_bound) {
      final connProvider = context.read<RobotConnectionProvider>();
      final settingsPrefs = context.read<SettingsPreferences>();
      final alertService = context.read<AlertMonitorService>();
      final stateLogger = context.read<StateLoggerService>();

      connProvider.bindSettings(settingsPrefs);
      alertService.bind(
        connProvider: connProvider,
        settingsPrefs: settingsPrefs,
      );
      stateLogger.bind(connProvider);

      // 监听告警流，展示 SnackBar
      _alertSub = alertService.alertStream.listen(_showAlertSnackBar);

      _bound = true;
    }
  }

  void _showAlertSnackBar(AlertRecord alert) {
    final prefs = context.read<SettingsPreferences>();
    if (!prefs.alertInApp) return; // 用户关闭了 APP 内提醒

    final Color bgColor;
    final IconData icon;
    switch (alert.level) {
      case AlertLevel.critical:
        bgColor = AppColors.error;
        icon = Icons.error_outline;
      case AlertLevel.warning:
        bgColor = AppColors.warning;
        icon = Icons.warning_amber_rounded;
      case AlertLevel.info:
        bgColor = AppColors.info;
        icon = Icons.info_outline;
    }

    _messengerKey.currentState?.showSnackBar(
      SnackBar(
        content: Row(
          children: [
            Icon(icon, color: Colors.white, size: 20),
            const SizedBox(width: 10),
            Expanded(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    alert.title,
                    style: const TextStyle(
                      fontWeight: FontWeight.w700,
                      fontSize: 14,
                      color: Colors.white,
                    ),
                  ),
                  Text(
                    alert.message,
                    style: const TextStyle(fontSize: 12, color: Colors.white70),
                  ),
                ],
              ),
            ),
          ],
        ),
        backgroundColor: bgColor,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
        margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        duration: const Duration(seconds: 4),
        action: SnackBarAction(
          label: '关闭',
          textColor: Colors.white,
          onPressed: () {},
        ),
      ),
    );
  }

  @override
  void dispose() {
    _alertSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<ThemeProvider>(
      builder: (context, themeProvider, _) {
        return MaterialApp(
          scaffoldMessengerKey: _messengerKey,
          title: '大算机器人',
          debugShowCheckedModeBanner: false,
          theme: lightTheme,
          darkTheme: darkTheme,
          themeMode: themeProvider.mode,
          initialRoute: '/',
          onGenerateRoute: _generateRoute,
        );
      },
    );
  }

  Route<dynamic>? _generateRoute(RouteSettings settings) {
    Widget page;
    switch (settings.name) {
      case '/':
        page = const SplashScreen();
      case '/main':
        page = const MainShellScreen();
      case '/scan':
        page = const ScanScreen();
      case '/robot-detail':
        page = const RobotDetailScreen();
      case '/settings':
        page = const AppSettingsScreen();
      case '/control':
        page = const ControlScreen();
      default:
        page = const MainShellScreen();
    }

    return PageRouteBuilder(
      settings: settings,
      pageBuilder: (_, __, ___) => page,
      transitionsBuilder: (context, animation, secondaryAnimation, child) {
        return SharedAxisTransition(
          animation: animation,
          secondaryAnimation: secondaryAnimation,
          transitionType: SharedAxisTransitionType.horizontal,
          child: child,
        );
      },
      transitionDuration: const Duration(milliseconds: 300),
    );
  }
}
