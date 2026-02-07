import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';

// ============================================================
// Design tokens — spacing, radius, elevation
// ============================================================

class AppSpacing {
  static const double xs = 4;
  static const double sm = 8;
  static const double md = 12;
  static const double lg = 16;
  static const double xl = 20;
  static const double xxl = 24;
  static const double xxxl = 32;

  /// Standard screen horizontal padding.
  static const double screenH = 20;
}

class AppRadius {
  static const double sm = 8;
  static const double md = 12;
  static const double lg = 16;
  static const double xl = 20;
  static const double xxl = 24;

  /// Default card radius.
  static const double card = 20;
  /// Icon container radius.
  static const double icon = 12;
  /// Pill / badge radius.
  static const double pill = 30;
}

class AppShadows {
  /// Subtle card shadow (light mode).
  static BoxShadow light({Color? color}) => BoxShadow(
        color: color ?? Colors.black.withOpacity(0.06),
        blurRadius: 16,
        offset: const Offset(0, 6),
      );

  /// Deeper card shadow (dark mode — subtle).
  static BoxShadow dark({Color? color}) => BoxShadow(
        color: color ?? Colors.black.withOpacity(0.35),
        blurRadius: 20,
        offset: const Offset(0, 8),
      );

  /// Colored glow (for accent icons / buttons).
  static BoxShadow glow(Color color, {double blur = 16}) => BoxShadow(
        color: color.withOpacity(0.30),
        blurRadius: blur,
        offset: const Offset(0, 4),
      );
}

class AppDurations {
  static const fast = Duration(milliseconds: 150);
  static const normal = Duration(milliseconds: 250);
  static const slow = Duration(milliseconds: 400);
  static const stagger = Duration(milliseconds: 800);
}

// ============================================================
// App color constants
// ============================================================

class AppColors {
  // Brand
  static const primary = Color(0xFF007AFF);
  static const secondary = Color(0xFF5856D6);
  static const accent = Color(0xFFAF52DE);

  // Semantic
  static const success = Color(0xFF34C759);
  static const warning = Color(0xFFFF9500);
  static const error = Color(0xFFFF3B30);
  static const info = Color(0xFF5AC8FA);

  // Light surfaces
  static const lightBackground = Color(0xFFF2F2F7);
  static const lightSurface = Colors.white;
  static const lightCard = Color(0xFFFFFFFF);

  // Dark surfaces — refined tech-dark palette (OLED-friendly)
  static const darkBackground = Color(0xFF0A0A0F);
  static const darkSurface = Color(0xFF141418);
  static const darkCard = Color(0xFF1C1C24);
  static const darkElevated = Color(0xFF26262E);

  // Glass
  static Color glassLight = Colors.white.withOpacity(0.82);
  static Color glassDark = const Color(0xFF1C1C24).withOpacity(0.75);
  static Color glassBorderLight = Colors.white.withOpacity(0.45);
  static Color glassBorderDark = Colors.white.withOpacity(0.06);

  // Gradients
  static const brandGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [primary, secondary],
  );

  /// Background gradient for dark mode.
  static const darkBgGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [
      Color(0xFF08080E),
      Color(0xFF0E0E18),
      Color(0xFF08080E),
    ],
  );

  /// Background gradient for light mode.
  static const lightBgGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [
      Color(0xFFF2F2F7),
      Color(0xFFE8ECF4),
      Color(0xFFF2F2F7),
    ],
  );
}

// ============================================================
// Theme mode provider (ChangeNotifier)
// ============================================================

class ThemeProvider extends ChangeNotifier {
  static const _key = 'theme_mode';
  ThemeMode _mode = ThemeMode.system;

  ThemeMode get mode => _mode;
  bool get isDark => _mode == ThemeMode.dark;

  ThemeProvider() {
    _load();
  }

  Future<void> _load() async {
    final prefs = await SharedPreferences.getInstance();
    final value = prefs.getString(_key);
    if (value == 'light') {
      _mode = ThemeMode.light;
    } else if (value == 'dark') {
      _mode = ThemeMode.dark;
    } else {
      _mode = ThemeMode.system;
    }
    notifyListeners();
  }

  Future<void> setMode(ThemeMode mode) async {
    _mode = mode;
    notifyListeners();
    final prefs = await SharedPreferences.getInstance();
    switch (mode) {
      case ThemeMode.light:
        await prefs.setString(_key, 'light');
      case ThemeMode.dark:
        await prefs.setString(_key, 'dark');
      case ThemeMode.system:
        await prefs.remove(_key);
    }
  }
}

// ============================================================
// Light theme
// ============================================================

final ThemeData lightTheme = ThemeData(
  useMaterial3: true,
  brightness: Brightness.light,
  scaffoldBackgroundColor: AppColors.lightBackground,
  colorScheme: ColorScheme.fromSeed(
    seedColor: AppColors.primary,
    surface: AppColors.lightSurface,
    brightness: Brightness.light,
  ),
  appBarTheme: const AppBarTheme(
    backgroundColor: Colors.transparent,
    elevation: 0,
    scrolledUnderElevation: 0,
    centerTitle: true,
    systemOverlayStyle: SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.dark,
    ),
    titleTextStyle: TextStyle(
      color: Colors.black87,
      fontSize: 17,
      fontWeight: FontWeight.w600,
      letterSpacing: -0.5,
    ),
    iconTheme: IconThemeData(color: Colors.black87),
  ),
  cardTheme: CardThemeData(
    color: AppColors.lightCard,
    elevation: 0,
    shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(AppRadius.card)),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: Colors.black.withOpacity(0.06),
    thickness: 0.5,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return AppColors.primary;
      return Colors.white;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return AppColors.primary.withOpacity(0.4);
      }
      return Colors.black.withOpacity(0.1);
    }),
  ),
);

// ============================================================
// Dark theme
// ============================================================

final ThemeData darkTheme = ThemeData(
  useMaterial3: true,
  brightness: Brightness.dark,
  scaffoldBackgroundColor: AppColors.darkBackground,
  colorScheme: ColorScheme.fromSeed(
    seedColor: AppColors.primary,
    surface: AppColors.darkSurface,
    brightness: Brightness.dark,
  ),
  appBarTheme: const AppBarTheme(
    backgroundColor: Colors.transparent,
    elevation: 0,
    scrolledUnderElevation: 0,
    centerTitle: true,
    systemOverlayStyle: SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ),
    titleTextStyle: TextStyle(
      color: Color(0xFFF0F0F5),
      fontSize: 17,
      fontWeight: FontWeight.w600,
      letterSpacing: -0.5,
    ),
    iconTheme: IconThemeData(color: Color(0xFFF0F0F5)),
  ),
  cardTheme: CardThemeData(
    color: AppColors.darkCard,
    elevation: 0,
    shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(AppRadius.card)),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: Colors.white.withOpacity(0.07),
    thickness: 0.5,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return AppColors.primary;
      return Colors.grey;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return AppColors.primary.withOpacity(0.4);
      }
      return Colors.white.withOpacity(0.1);
    }),
  ),
);

// ============================================================
// Theme helper extensions
// ============================================================

extension ThemeContextExtension on BuildContext {
  bool get isDark => Theme.of(this).brightness == Brightness.dark;

  // ── Surface colors ──
  Color get glassColor => isDark ? AppColors.glassDark : AppColors.glassLight;
  Color get glassBorder =>
      isDark ? AppColors.glassBorderDark : AppColors.glassBorderLight;
  Color get cardColor => isDark ? AppColors.darkCard : Colors.white;

  // ── Text colors ──
  Color get titleColor =>
      isDark ? const Color(0xFFF0F0F5) : const Color(0xFF1A1A1A);
  Color get subtitleColor =>
      isDark ? Colors.white.withOpacity(0.48) : Colors.black.withOpacity(0.42);

  // ── Decorative ──
  Color get cardShadowColor => isDark
      ? Colors.black.withOpacity(0.35)
      : Colors.black.withOpacity(0.06);
  Color get dividerColor =>
      isDark ? Colors.white.withOpacity(0.07) : Colors.black.withOpacity(0.06);
  Color get inputFillColor =>
      isDark ? Colors.white.withOpacity(0.06) : Colors.black.withOpacity(0.04);

  // ── Convenience decorations ──
  BoxDecoration get cardDecoration => BoxDecoration(
        color: cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [
          isDark ? AppShadows.dark() : AppShadows.light(),
        ],
      );

  LinearGradient get bgGradient =>
      isDark ? AppColors.darkBgGradient : AppColors.lightBgGradient;
}
