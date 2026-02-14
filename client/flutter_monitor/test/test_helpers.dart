/// Shared test utilities for flutter_monitor widget tests.
///
/// Provides a [pumpApp] helper that wraps a widget with all required providers
/// and a connected or disconnected [MockRobotClient].
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/gateway/file_gateway.dart';
import 'package:flutter_monitor/core/services/alert_monitor_service.dart';
import 'package:flutter_monitor/core/services/state_logger_service.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';

/// Creates a [MockRobotClient] and optionally connects it.
Future<MockRobotClient> createMockClient({bool connect = true}) async {
  final client = MockRobotClient();
  if (connect) await client.connect();
  return client;
}

/// Pumps [child] wrapped with all required providers + MaterialApp.
///
/// Pass [connected] = true to inject a connected MockRobotClient into
/// [RobotConnectionProvider] so streams work.
///
/// [routes] — additional named routes (e.g. {'/main': (_) => ...}).
Future<void> pumpApp(
  WidgetTester tester,
  Widget child, {
  bool connected = false,
  NavigatorObserver? observer,
  Map<String, WidgetBuilder>? routes,
}) async {
  await tester.pumpWidget(
    MultiProvider(
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
      child: MaterialApp(
        title: 'Test',
        theme: lightTheme,
        darkTheme: darkTheme,
        home: child,
        navigatorObservers: observer != null ? [observer] : [],
        onGenerateRoute: (settings) {
          final builder = routes?[settings.name];
          if (builder != null) {
            return MaterialPageRoute(builder: builder, settings: settings);
          }
          // Fallback: return an empty scaffold for unknown routes
          return MaterialPageRoute(
            builder: (_) => const Scaffold(body: SizedBox.shrink()),
            settings: settings,
          );
        },
      ),
    ),
  );
  // Let the first frame settle.
  await tester.pump();
}
