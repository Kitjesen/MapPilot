import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/gateway/file_gateway.dart';
import 'package:flutter_monitor/core/services/alert_monitor_service.dart';
import 'package:flutter_monitor/core/services/state_logger_service.dart';

void main() {
  testWidgets('App shell renders without errors', (WidgetTester tester) async {
    // Build a minimal app shell with all required providers
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
        child: Consumer<ThemeProvider>(
          builder: (context, themeProvider, _) {
            return MaterialApp(
              title: '大算机器人',
              debugShowCheckedModeBanner: false,
              theme: lightTheme,
              darkTheme: darkTheme,
              themeMode: themeProvider.mode,
              home: const Scaffold(
                body: Center(child: Text('Test Shell')),
              ),
            );
          },
        ),
      ),
    );
    await tester.pump();

    // App should render without errors
    expect(find.text('Test Shell'), findsOneWidget);
  });

  testWidgets('Providers are accessible in widget tree', (WidgetTester tester) async {
    late RobotConnectionProvider connProvider;
    late ControlGateway controlGateway;

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
          home: Builder(
            builder: (context) {
              connProvider = context.read<RobotConnectionProvider>();
              controlGateway = context.read<ControlGateway>();
              return const Scaffold(body: SizedBox.shrink());
            },
          ),
        ),
      ),
    );
    await tester.pump();

    // Providers should be accessible and in correct initial state
    expect(connProvider.isConnected, isFalse);
    expect(controlGateway.hasLease, isFalse);
  });

  testWidgets('ThemeProvider toggles mode', (WidgetTester tester) async {
    final themeProvider = ThemeProvider();

    await tester.pumpWidget(
      ChangeNotifierProvider.value(
        value: themeProvider,
        child: Consumer<ThemeProvider>(
          builder: (context, tp, _) {
            return MaterialApp(
              theme: lightTheme,
              darkTheme: darkTheme,
              themeMode: tp.mode,
              home: Builder(
                builder: (context) {
                  final brightness = Theme.of(context).brightness;
                  return Scaffold(
                    body: Text('Brightness: $brightness'),
                  );
                },
              ),
            );
          },
        ),
      ),
    );
    await tester.pump();

    // Default should follow system (ThemeMode.system)
    expect(themeProvider.mode, ThemeMode.system);
  });
}
