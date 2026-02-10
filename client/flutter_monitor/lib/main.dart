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
import 'package:flutter_monitor/features/robot_select/robot_select_screen.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/gateway/file_gateway.dart';
import 'package:flutter_monitor/features/task/task_panel.dart';
import 'package:flutter_monitor/features/map/map_manager_page.dart';
import 'package:flutter_monitor/features/map/map_goal_picker.dart';
import 'package:flutter_monitor/features/camera/camera_screen.dart';
import 'package:flutter_monitor/features/files/file_browser_screen.dart';
import 'package:flutter_monitor/features/events/events_screen.dart';

import 'package:flutter_monitor/core/services/notification_service.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:flutter_monitor/shared/widgets/global_safety_overlay.dart';
import 'package:flutter_monitor/shared/widgets/error_boundary.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
    ),
  );

  // Capture unhandled Flutter errors
  FlutterError.onError = (details) {
    AppLogger.system.error(
      'Flutter framework error: ${details.exceptionAsString()}',
      error: details.exception,
      stackTrace: details.stack,
    );
  };

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
        ChangeNotifierProvider(create: (_) => RobotProfileProvider()),
        ChangeNotifierProvider(create: (_) => OtaGateway()),
        ChangeNotifierProvider(create: (_) => MapGateway()),
        ChangeNotifierProvider(create: (_) => TaskGateway()),
        ChangeNotifierProvider(create: (_) => ControlGateway()),
        ChangeNotifierProvider(create: (_) => FileGateway()),
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

      final profileProvider = context.read<RobotProfileProvider>();
      connProvider.bindSettings(settingsPrefs);
      connProvider.bindProfileProvider(profileProvider);
      alertService.bind(
        connProvider: connProvider,
        settingsPrefs: settingsPrefs,
      );
      stateLogger.bind(connProvider);

      // 绑定 Gateway（延迟到下一帧，避免 build 阶段 notifyListeners 报错）
      final otaGateway = context.read<OtaGateway>();
      final mapGateway = context.read<MapGateway>();
      final taskGateway = context.read<TaskGateway>();
      final fileGateway = context.read<FileGateway>();
      final controlGateway = context.read<ControlGateway>();

      WidgetsBinding.instance.addPostFrameCallback((_) {
        otaGateway.updateClient(connProvider.client);
        otaGateway.cloud.loadConfig();
        mapGateway.updateClient(connProvider.client);
        taskGateway.updateClient(connProvider.client);
        fileGateway.updateClient(connProvider.client);
        controlGateway.updateClients(
          client: connProvider.client,
          dogClient: connProvider.dogClient,
        );
      });

      // 监听连接变化，自动同步所有 Gateway 的 client 引用
      connProvider.addListener(() {
        otaGateway.updateClient(connProvider.client);
        mapGateway.updateClient(connProvider.client);
        taskGateway.updateClient(connProvider.client);
        fileGateway.updateClient(connProvider.client);
        controlGateway.updateClients(
          client: connProvider.client,
          dogClient: connProvider.dogClient,
        );
      });

      // ── Cross-Gateway Orchestration ──

      // 1. Mapping auto-save: when mapping task completes → save map
      taskGateway.onMappingComplete = () {
        AppLogger.system.info('Mapping complete → auto-saving map');
        mapGateway.saveMap('auto_${DateTime.now().millisecondsSinceEpoch}.pcd');
      };

      // 2. Disconnect recovery: on reconnect → re-query gateway states
      connProvider.onReconnected = () {
        AppLogger.system.info('Reconnected → refreshing gateway states');
        // Re-query installed versions (and check deployment status)
        otaGateway.fetchInstalledVersions().catchError((_) {});
        // Re-query maps
        mapGateway.refreshMaps().catchError((_) {});
      };

      // 2.1 Task terminal callback: refresh OTA-relevant state after task ends.
      taskGateway.onTaskTerminated = (type, status) {
        AppLogger.system.info('Task terminated: type=$type status=$status');
        otaGateway.fetchInstalledVersions().catchError((_) {});
      };

      // 3. Safety interlock: before COLD OTA apply → ensure robot safe
      otaGateway.safetyCheck = () async {
        // Check if robot is currently executing a task
        if (taskGateway.isRunning) {
          AppLogger.system.warning('Safety check: task running, blocking COLD update');
          return false;
        }
        // Check if dog is standing — sit down + disable first
        final dog = controlGateway.dogClient;
        if (dog != null && dog.isConnected && dog.isStanding) {
          AppLogger.system.info('Safety check: dog standing, sitting down + disabling');
          await controlGateway.dogSitDown();
          await Future.delayed(const Duration(seconds: 2));
          await controlGateway.dogDisable();
          await Future.delayed(const Duration(seconds: 1));
        }
        // Release teleop lease if held
        if (controlGateway.hasLease) {
          AppLogger.system.info('Safety check: releasing control lease');
          await controlGateway.toggleLease();
        }
        return true;
      };

      // 监听告警流，展示 SnackBar
      _alertSub = alertService.alertStream.listen(_showAlertSnackBar);

      _bound = true;
    }
  }

  void _showAlertSnackBar(AlertRecord alert) {
    final prefs = context.read<SettingsPreferences>();
    if (!prefs.alertInApp) return; // 用户关闭了 APP 内提醒

    final Color accentColor;
    switch (alert.level) {
      case AlertLevel.critical:
        accentColor = AppColors.error;
      case AlertLevel.warning:
        accentColor = AppColors.warning;
      case AlertLevel.info:
        accentColor = AppColors.primary;
    }

    final brightness = Theme.of(context).brightness;
    final isDark = brightness == Brightness.dark;

    _messengerKey.currentState?.showSnackBar(
      SnackBar(
        content: Row(
          children: [
            // Left accent strip
            Container(
              width: 3,
              height: 36,
              decoration: BoxDecoration(
                color: accentColor,
                borderRadius: BorderRadius.circular(2),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    alert.title,
                    style: TextStyle(
                      fontWeight: FontWeight.w600,
                      fontSize: 13,
                      color: isDark ? Colors.white : const Color(0xFF1A1A1A),
                    ),
                  ),
                  const SizedBox(height: 2),
                  Text(
                    alert.message,
                    style: TextStyle(
                      fontSize: 12,
                      color: isDark
                          ? Colors.white.withValues(alpha: 0.5)
                          : Colors.black.withValues(alpha: 0.45),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(width: 8),
            // Dismiss button
            GestureDetector(
              onTap: () {
                _messengerKey.currentState?.hideCurrentSnackBar();
              },
              child: Icon(
                Icons.close,
                size: 16,
                color: isDark
                    ? Colors.white.withValues(alpha: 0.35)
                    : Colors.black.withValues(alpha: 0.3),
              ),
            ),
          ],
        ),
        backgroundColor: isDark ? AppColors.darkCard : Colors.white,
        behavior: SnackBarBehavior.floating,
        elevation: 0,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(AppRadius.card),
          side: BorderSide(
            color: isDark
                ? AppColors.borderDark
                : AppColors.borderLight,
          ),
        ),
        margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
        duration: const Duration(seconds: 5),
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
          builder: (context, child) {
            // Global error boundary + safety overlay wrapping all screens
            return ErrorBoundary(
              child: GlobalSafetyOverlay(
                child: child ?? const SizedBox.shrink(),
              ),
            );
          },
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
      case '/robot-select':
        page = const RobotSelectScreen();
      case '/task-panel':
        page = const TaskPanel();
      case '/map-manager':
        page = const MapManagerPage();
      case '/map-select-goal':
        page = const MapGoalPicker();
      case '/camera':
        page = const CameraScreen();
      case '/files':
        page = const FileBrowserScreen();
      case '/events':
        page = const EventsScreen();
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
