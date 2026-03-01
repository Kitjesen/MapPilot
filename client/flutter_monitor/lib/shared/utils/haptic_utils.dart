import 'package:flutter/services.dart';

/// Unified haptic feedback utility.
///
/// Provides semantic haptic methods so the app uses consistent
/// vibration intensities for each interaction category.
class HapticUtils {
  HapticUtils._();

  /// Light tap — UI selection, minor actions.
  static void light() => HapticFeedback.lightImpact();

  /// Medium tap — confirmations, mode switches, task start.
  static void medium() => HapticFeedback.mediumImpact();

  /// Heavy tap — emergency stop, destructive actions.
  static void heavy() => HapticFeedback.heavyImpact();

  /// Task/mission completed successfully.
  static void success() => HapticFeedback.mediumImpact();

  /// Warning — low battery, high temperature, alerts.
  static void warning() => HapticFeedback.heavyImpact();

  /// Selection click — toggling chips, radio buttons.
  static void selection() => HapticFeedback.selectionClick();
}
