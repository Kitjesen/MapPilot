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
  static const double card = 16;
  /// Icon container radius.
  static const double icon = 12;
  /// Pill / badge radius.
  static const double pill = 20;
}

class AppShadows {
  /// Soft card shadow (light mode).
  static BoxShadow light({Color? color}) => BoxShadow(
        color: color ?? const Color(0xFF6C63FF).withValues(alpha: 0.06),
        blurRadius: 24,
        spreadRadius: 0,
        offset: const Offset(0, 4),
      );

  /// Subtle shadow (dark mode).
  static BoxShadow dark({Color? color}) => BoxShadow(
        color: color ?? Colors.black.withValues(alpha: 0.30),
        blurRadius: 16,
        offset: const Offset(0, 4),
      );

  /// Colored glow — soft purple.
  static BoxShadow glow(Color color, {double blur = 16}) => BoxShadow(
        color: color.withValues(alpha: 0.18),
        blurRadius: blur,
        offset: const Offset(0, 2),
      );

  /// Elevated card shadow.
  static List<BoxShadow> elevated({bool isDark = false}) => [
        BoxShadow(
          color: isDark
              ? Colors.black.withValues(alpha: 0.4)
              : const Color(0xFF6C63FF).withValues(alpha: 0.08),
          blurRadius: 32,
          spreadRadius: 0,
          offset: const Offset(0, 8),
        ),
        BoxShadow(
          color: isDark
              ? Colors.black.withValues(alpha: 0.2)
              : const Color(0xFF6C63FF).withValues(alpha: 0.04),
          blurRadius: 12,
          spreadRadius: 0,
          offset: const Offset(0, 2),
        ),
      ];
}

class AppDurations {
  static const fast = Duration(milliseconds: 150);
  static const normal = Duration(milliseconds: 250);
  static const slow = Duration(milliseconds: 400);
  static const stagger = Duration(milliseconds: 800);
}

// ============================================================
// App color constants — Purple/Indigo Design System
// ============================================================

class AppColors {
  // Brand — Purple / Indigo
  static const primary = Color(0xFF6C63FF);
  static const primaryLight = Color(0xFF8B83FF);
  static const primaryDark = Color(0xFF5046E5);
  static const secondary = Color(0xFF8B5CF6);
  static const accent = Color(0xFFA78BFA);

  // Semantic
  static const success = Color(0xFF22C55E);
  static const successLight = Color(0xFFDCFCE7);
  static const warning = Color(0xFFF59E0B);
  static const warningLight = Color(0xFFFEF3C7);
  static const error = Color(0xFFEF4444);
  static const errorLight = Color(0xFFFEE2E2);
  static const info = Color(0xFF3B82F6);
  static const infoLight = Color(0xFFDBEAFE);

  // Light surfaces — soft lavender tones
  static const lightBackground = Color(0xFFF5F3FF);
  static const lightSurface = Color(0xFFFAF9FF);
  static const lightCard = Color(0xFFFFFFFF);
  static const lightSidebar = Color(0xFFF0EDFF);

  // Dark surfaces — deep purple-tinted
  static const darkBackground = Color(0xFF0F0E1A);
  static const darkSurface = Color(0xFF1A1830);
  static const darkCard = Color(0xFF221F3A);
  static const darkElevated = Color(0xFF2D2950);

  // Borders — subtle, barely visible
  static Color borderLight = const Color(0xFFE8E5F5);
  static Color borderDark = const Color(0xFF2D2950);

  // Text colors
  static const textPrimary = Color(0xFF1E1B4B);
  static const textSecondary = Color(0xFF6B7280);
  static const textTertiary = Color(0xFF9CA3AF);

  static const textPrimaryDark = Color(0xFFE8E5F5);
  static const textSecondaryDark = Color(0xFF9CA3AF);
  static const textTertiaryDark = Color(0xFF6B7280);

  // Glass — now with purple tint
  static Color glassLight = Colors.white.withValues(alpha: 0.92);
  static Color glassDark = const Color(0xFF1A1830).withValues(alpha: 0.90);
  static Color glassBorderLight = const Color(0xFFE8E5F5);
  static Color glassBorderDark = const Color(0xFF2D2950);

  // Gradients
  static const brandGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [Color(0xFF6C63FF), Color(0xFF8B5CF6)],
  );

  static const cardGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [Color(0xFFFFFFFF), Color(0xFFFAF9FF)],
  );

  /// Background gradient for light mode — soft lavender.
  static const lightBgGradient = LinearGradient(
    begin: Alignment.topLeft,
    end: Alignment.bottomRight,
    colors: [
      Color(0xFFF5F3FF),
      Color(0xFFEEF2FF),
      Color(0xFFF0F0FF),
    ],
  );

  /// Background gradient for dark mode — deep purple.
  static const darkBgGradient = LinearGradient(
    begin: Alignment.topCenter,
    end: Alignment.bottomCenter,
    colors: [
      Color(0xFF0F0E1A),
      Color(0xFF131127),
    ],
  );

  /// Status badge colors
  static const online = Color(0xFF22C55E);
  static const offline = Color(0xFFEF4444);
  static const connecting = Color(0xFFF59E0B);
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
  fontFamily: 'Inter',
  colorScheme: ColorScheme.fromSeed(
    seedColor: AppColors.primary,
    primary: AppColors.primary,
    surface: AppColors.lightSurface,
    brightness: Brightness.light,
  ),
  appBarTheme: const AppBarTheme(
    backgroundColor: Colors.transparent,
    elevation: 0,
    scrolledUnderElevation: 0,
    centerTitle: false,
    systemOverlayStyle: SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.dark,
    ),
    titleTextStyle: TextStyle(
      color: AppColors.textPrimary,
      fontSize: 18,
      fontWeight: FontWeight.w700,
      letterSpacing: -0.3,
    ),
    iconTheme: IconThemeData(color: AppColors.textPrimary),
  ),
  cardTheme: CardThemeData(
    color: AppColors.lightCard,
    elevation: 0,
    shape: RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(AppRadius.card),
    ),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: AppColors.borderLight,
    thickness: 1,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  elevatedButtonTheme: ElevatedButtonThemeData(
    style: ElevatedButton.styleFrom(
      backgroundColor: AppColors.primary,
      foregroundColor: Colors.white,
      elevation: 0,
      padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 14),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
      textStyle: const TextStyle(
        fontSize: 14,
        fontWeight: FontWeight.w600,
      ),
    ),
  ),
  outlinedButtonTheme: OutlinedButtonThemeData(
    style: OutlinedButton.styleFrom(
      foregroundColor: AppColors.primary,
      side: const BorderSide(color: AppColors.primary, width: 1.5),
      padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 14),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
      textStyle: const TextStyle(
        fontSize: 14,
        fontWeight: FontWeight.w600,
      ),
    ),
  ),
  inputDecorationTheme: InputDecorationTheme(
    filled: true,
    fillColor: Colors.white,
    contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
    border: OutlineInputBorder(
      borderRadius: BorderRadius.circular(AppRadius.md),
      borderSide: BorderSide(color: AppColors.borderLight),
    ),
    enabledBorder: OutlineInputBorder(
      borderRadius: BorderRadius.circular(AppRadius.md),
      borderSide: BorderSide(color: AppColors.borderLight),
    ),
    focusedBorder: OutlineInputBorder(
      borderRadius: BorderRadius.circular(AppRadius.md),
      borderSide: const BorderSide(color: AppColors.primary, width: 1.5),
    ),
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
      return const Color(0xFFE5E7EB);
    }),
  ),
  tabBarTheme: TabBarThemeData(
    labelColor: AppColors.primary,
    unselectedLabelColor: AppColors.textSecondary,
    indicatorColor: AppColors.primary,
    labelStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
    unselectedLabelStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w400),
  ),
  chipTheme: ChipThemeData(
    backgroundColor: AppColors.lightSurface,
    labelStyle: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
    shape: RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(AppRadius.pill),
    ),
    side: BorderSide.none,
  ),
);

// ============================================================
// Dark theme
// ============================================================

final ThemeData darkTheme = ThemeData(
  useMaterial3: true,
  brightness: Brightness.dark,
  scaffoldBackgroundColor: AppColors.darkBackground,
  fontFamily: 'Inter',
  colorScheme: ColorScheme.fromSeed(
    seedColor: AppColors.primary,
    primary: AppColors.primaryLight,
    surface: AppColors.darkSurface,
    brightness: Brightness.dark,
  ),
  appBarTheme: const AppBarTheme(
    backgroundColor: Colors.transparent,
    elevation: 0,
    scrolledUnderElevation: 0,
    centerTitle: false,
    systemOverlayStyle: SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ),
    titleTextStyle: TextStyle(
      color: AppColors.textPrimaryDark,
      fontSize: 18,
      fontWeight: FontWeight.w700,
      letterSpacing: -0.3,
    ),
    iconTheme: IconThemeData(color: AppColors.textPrimaryDark),
  ),
  cardTheme: CardThemeData(
    color: AppColors.darkCard,
    elevation: 0,
    shape: RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(AppRadius.card),
    ),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: AppColors.borderDark,
    thickness: 1,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  elevatedButtonTheme: ElevatedButtonThemeData(
    style: ElevatedButton.styleFrom(
      backgroundColor: AppColors.primaryLight,
      foregroundColor: Colors.white,
      elevation: 0,
      padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 14),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
    ),
  ),
  outlinedButtonTheme: OutlinedButtonThemeData(
    style: OutlinedButton.styleFrom(
      foregroundColor: AppColors.primaryLight,
      side: const BorderSide(color: AppColors.primaryLight, width: 1.5),
      padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 14),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(AppRadius.md),
      ),
    ),
  ),
  inputDecorationTheme: InputDecorationTheme(
    filled: true,
    fillColor: AppColors.darkElevated,
    contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
    border: OutlineInputBorder(
      borderRadius: BorderRadius.circular(AppRadius.md),
      borderSide: BorderSide(color: AppColors.borderDark),
    ),
    enabledBorder: OutlineInputBorder(
      borderRadius: BorderRadius.circular(AppRadius.md),
      borderSide: BorderSide(color: AppColors.borderDark),
    ),
    focusedBorder: OutlineInputBorder(
      borderRadius: BorderRadius.circular(AppRadius.md),
      borderSide: const BorderSide(color: AppColors.primaryLight, width: 1.5),
    ),
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return Colors.white;
      return Colors.white;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return AppColors.primaryLight;
      }
      return AppColors.darkElevated;
    }),
  ),
  tabBarTheme: TabBarThemeData(
    labelColor: AppColors.primaryLight,
    unselectedLabelColor: AppColors.textSecondaryDark,
    indicatorColor: AppColors.primaryLight,
    labelStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
    unselectedLabelStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w400),
  ),
  chipTheme: ChipThemeData(
    backgroundColor: AppColors.darkElevated,
    labelStyle: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
    shape: RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(AppRadius.pill),
    ),
    side: BorderSide.none,
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
  Color get surfaceColor =>
      isDark ? AppColors.darkSurface : AppColors.lightSurface;

  // ── Border color ──
  Color get borderColor =>
      isDark ? AppColors.borderDark : AppColors.borderLight;

  // ── Text colors ──
  Color get titleColor =>
      isDark ? AppColors.textPrimaryDark : AppColors.textPrimary;
  Color get subtitleColor =>
      isDark ? AppColors.textSecondaryDark : AppColors.textSecondary;
  Color get hintColor =>
      isDark ? AppColors.textTertiaryDark : AppColors.textTertiary;

  // ── Decorative ──
  Color get cardShadowColor => isDark
      ? Colors.black.withValues(alpha: 0.40)
      : const Color(0xFF6C63FF).withValues(alpha: 0.06);
  Color get dividerColor =>
      isDark ? AppColors.borderDark : AppColors.borderLight;
  Color get inputFillColor =>
      isDark ? AppColors.darkElevated : Colors.white;

  // ── Card decoration — clean with soft shadow, NO border in light mode ──
  BoxDecoration get cardDecoration => BoxDecoration(
        color: cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: isDark ? Border.all(color: borderColor) : null,
        boxShadow: [
          isDark ? AppShadows.dark() : AppShadows.light(),
        ],
      );

  /// Elevated card decoration with stronger shadow.
  BoxDecoration get elevatedCardDecoration => BoxDecoration(
        color: cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: isDark ? Border.all(color: borderColor) : null,
        boxShadow: AppShadows.elevated(isDark: isDark),
      );

  LinearGradient get bgGradient =>
      isDark ? AppColors.darkBgGradient : AppColors.lightBgGradient;
}
