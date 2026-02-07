import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:animations/animations.dart';
import 'theme/app_theme.dart';
import 'services/robot_connection_provider.dart';
import 'services/settings_preferences.dart';
import 'screens/splash_screen.dart';
import 'screens/main_shell_screen.dart';
import 'screens/scan_screen.dart';
import 'screens/robot_detail_screen.dart';
import 'screens/app_settings_screen.dart';
import 'screens/control_screen.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
    ),
  );
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

  // ── Animations ──
  late AnimationController _entranceController;
  late AnimationController _pulseController;
  late AnimationController _floatController;

  late Animation<double> _logoSlide;
  late Animation<double> _logoFade;
  late Animation<double> _titleSlide;
  late Animation<double> _titleFade;
  late Animation<double> _cardSlide;
  late Animation<double> _cardFade;
  late Animation<double> _pulse;
  late Animation<double> _float;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_bound) {
      final connProvider = context.read<RobotConnectionProvider>();
      final settingsPrefs = context.read<SettingsPreferences>();
      connProvider.bindSettings(settingsPrefs);
      _bound = true;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<ThemeProvider>(
      builder: (context, themeProvider, _) {
        return MaterialApp(
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
