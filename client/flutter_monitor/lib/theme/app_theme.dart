import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// 大算机器人 App — Dark Lime Theme
/// Inspired by modern dark UI with chartreuse/lime accent.
class AppColors {
  AppColors._();

  // ── Background layers ──
  static const Color bg = Color(0xFF0D0D0D);
  static const Color bgElevated = Color(0xFF161616);
  static const Color surface = Color(0xFF1C1C1E);
  static const Color surfaceLight = Color(0xFF2C2C2E);

  // ── Accent ──
  static const Color lime = Color(0xFFD4FF2B);
  static const Color limeDim = Color(0xFF8FA61E);

  // ── Text ──
  static const Color textPrimary = Color(0xFFF5F5F7);
  static const Color textSecondary = Color(0xFF98989D);
  static const Color textTertiary = Color(0xFF636366);

  // ── Semantic ──
  static const Color success = Color(0xFF34C759);
  static const Color warning = Color(0xFFFF9F0A);
  static const Color error = Color(0xFFFF453A);
  static const Color info = Color(0xFF64D2FF);

  // ── Borders & Dividers ──
  static const Color border = Color(0xFF38383A);
  static const Color divider = Color(0xFF2C2C2E);

  // ── Card variants ──
  static const Color cardDark = Color(0xFF1C1C1E);
  static const Color cardLight = Color(0xFF2C2C2E);
  static const Color cardLime = Color(0xFFD4FF2B);
}

class AppTheme {
  AppTheme._();

  static ThemeData get dark {
    return ThemeData(
      useMaterial3: true,
      brightness: Brightness.dark,
      scaffoldBackgroundColor: AppColors.bg,
      colorScheme: const ColorScheme.dark(
        primary: AppColors.lime,
        onPrimary: Colors.black,
        secondary: AppColors.limeDim,
        surface: AppColors.surface,
        onSurface: AppColors.textPrimary,
        error: AppColors.error,
        onError: Colors.white,
      ),
      appBarTheme: const AppBarTheme(
        backgroundColor: Colors.transparent,
        elevation: 0,
        centerTitle: true,
        systemOverlayStyle: SystemUiOverlayStyle.light,
        titleTextStyle: TextStyle(
          color: AppColors.textPrimary,
          fontSize: 17,
          fontWeight: FontWeight.w600,
          letterSpacing: -0.5,
        ),
        iconTheme: IconThemeData(color: AppColors.textPrimary),
      ),
      dividerTheme: const DividerThemeData(
        color: AppColors.divider,
        thickness: 0.5,
      ),
      snackBarTheme: SnackBarThemeData(
        backgroundColor: AppColors.surfaceLight,
        contentTextStyle: const TextStyle(color: AppColors.textPrimary),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        behavior: SnackBarBehavior.floating,
      ),
      dialogTheme: DialogTheme(
        backgroundColor: AppColors.surface,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        titleTextStyle: const TextStyle(
          color: AppColors.textPrimary,
          fontSize: 18,
          fontWeight: FontWeight.w700,
        ),
        contentTextStyle: const TextStyle(
          color: AppColors.textSecondary,
          fontSize: 14,
        ),
      ),
      switchTheme: SwitchThemeData(
        thumbColor: WidgetStateProperty.resolveWith((states) {
          if (states.contains(WidgetState.selected)) return AppColors.lime;
          return AppColors.textTertiary;
        }),
        trackColor: WidgetStateProperty.resolveWith((states) {
          if (states.contains(WidgetState.selected)) {
            return AppColors.lime.withOpacity(0.3);
          }
          return AppColors.surfaceLight;
        }),
      ),
      filledButtonTheme: FilledButtonThemeData(
        style: FilledButton.styleFrom(
          backgroundColor: AppColors.lime,
          foregroundColor: Colors.black,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
          textStyle: const TextStyle(
            fontWeight: FontWeight.w700,
            fontSize: 15,
          ),
        ),
      ),
      textButtonTheme: TextButtonThemeData(
        style: TextButton.styleFrom(
          foregroundColor: AppColors.lime,
        ),
      ),
    );
  }

  /// Overlay style for status bar
  static const systemOverlay = SystemUiOverlayStyle(
    statusBarColor: Colors.transparent,
    statusBarIconBrightness: Brightness.light,
    systemNavigationBarColor: AppColors.bg,
    systemNavigationBarIconBrightness: Brightness.light,
  );
}
