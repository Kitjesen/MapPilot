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
  static const double sm = 6;
  static const double md = 8;
  static const double lg = 12;
  static const double xl = 16;
  static const double xxl = 20;

  /// Default card radius.
  static const double card = 12;
  /// Icon container radius.
  static const double icon = 8;
  /// Pill / badge radius.
  static const double pill = 20;
}

class AppShadows {
  /// Barely visible shadow (light mode) — Cursor/Apple style.
  static BoxShadow light({Color? color}) => BoxShadow(
        color: color ?? Colors.black.withValues(alpha: 0.04),
        blurRadius: 6,
        offset: const Offset(0, 1),
      );

  /// Subtle shadow (dark mode).
  static BoxShadow dark({Color? color}) => BoxShadow(
        color: color ?? Colors.black.withValues(alpha: 0.40),
        blurRadius: 8,
        offset: const Offset(0, 2),
      );

  /// Colored glow — very subtle.
  static BoxShadow glow(Color color, {double blur = 10}) => BoxShadow(
        color: color.withValues(alpha: 0.15),
        blurRadius: blur,
        offset: const Offset(0, 1),
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

  // Dark surfaces — Apple/URDF-Studio style
  static const darkBackground = Color(0xFF000000);
  static const darkSurface = Color(0xFF1C1C1E);
  static const darkCard = Color(0xFF1C1C1E);
  static const darkElevated = Color(0xFF2C2C2E);

  // Borders — clean & subtle
  static Color borderLight = const Color(0xFFE5E5E5);
  static Color borderDark = const Color(0xFF38383A);

  // Glass — now cleaner, less frosted
  static Color glassLight = Colors.white.withValues(alpha: 0.95);
  static Color glassDark = const Color(0xFF1C1C1E).withValues(alpha: 0.92);
  static Color glassBorderLight = const Color(0xFFE5E5E5);
  static Color glassBorderDark = const Color(0xFF38383A);

  // Gradients
  static const brandGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [primary, secondary],
  );

  /// Background gradient for dark mode — flat, no gradient noise.
  static const darkBgGradient = LinearGradient(
    begin: Alignment.topCenter,
    end: Alignment.bottomCenter,
    colors: [
      Color(0xFF000000),
      Color(0xFF000000),
    ],
  );

  /// Background gradient for light mode — clean flat.
  static const lightBgGradient = LinearGradient(
    begin: Alignment.topCenter,
    end: Alignment.bottomCenter,
    colors: [
      Color(0xFFF2F2F7),
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
      borderRadius: BorderRadius.circular(AppRadius.card),
      side: BorderSide(color: AppColors.borderLight),
    ),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: Colors.black.withValues(alpha: 0.05),
    thickness: 0.5,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return Colors.white;
      return Colors.white;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return AppColors.primary;
      }
      return const Color(0xFFE9E9EA);
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
      borderRadius: BorderRadius.circular(AppRadius.card),
      side: BorderSide(color: AppColors.borderDark),
    ),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: Colors.white.withValues(alpha: 0.06),
    thickness: 0.5,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return Colors.white;
      return Colors.white;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return const Color(0xFF0A84FF); // iOS dark blue
      }
      return const Color(0xFF1C1C1E);
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

  // ── Border color ──
  Color get borderColor =>
      isDark ? AppColors.borderDark : AppColors.borderLight;

  // ── Text colors ──
  Color get titleColor =>
      isDark ? const Color(0xFFE5E5E5) : const Color(0xFF1A1A1A);
  Color get subtitleColor =>
      isDark ? const Color(0xFF86868B) : const Color(0xFF86868B);

  // ── Decorative ──
  Color get cardShadowColor => isDark
      ? Colors.black.withValues(alpha: 0.50)
      : Colors.black.withValues(alpha: 0.08);
  Color get dividerColor =>
      isDark ? const Color(0xFF38383A) : const Color(0xFFE5E5E5);
  Color get inputFillColor =>
      isDark ? Colors.white.withValues(alpha: 0.06) : Colors.black.withValues(alpha: 0.04);

  // ── Convenience decorations — clean card with border ──
  BoxDecoration get cardDecoration => BoxDecoration(
        color: cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(color: borderColor),
        boxShadow: [
          isDark ? AppShadows.dark() : AppShadows.light(),
        ],
      );

  LinearGradient get bgGradient =>
      isDark ? AppColors.darkBgGradient : AppColors.lightBgGradient;
}
