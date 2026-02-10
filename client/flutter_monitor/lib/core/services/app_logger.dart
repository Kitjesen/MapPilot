import 'dart:collection';
import 'package:flutter/foundation.dart';

/// Severity levels for log entries.
enum LogLevel { debug, info, warning, error }

/// A single log entry.
class LogEntry {
  final DateTime timestamp;
  final LogLevel level;
  final String tag;
  final String message;
  final Object? error;
  final StackTrace? stackTrace;

  LogEntry({
    required this.timestamp,
    required this.level,
    required this.tag,
    required this.message,
    this.error,
    this.stackTrace,
  });

  @override
  String toString() {
    final ts = timestamp.toIso8601String().substring(11, 23);
    final lvl = level.name.toUpperCase().padRight(5);
    final err = error != null ? ' | $error' : '';
    return '[$ts] $lvl [$tag] $message$err';
  }
}

/// Centralized logging service with ring buffer for log export.
///
/// Usage:
/// ```dart
/// AppLogger.ota.info('Upload started');
/// AppLogger.grpc.error('Connection failed', error: e, stackTrace: st);
/// ```
class AppLogger {
  // Named loggers for each subsystem
  static final ota = AppLogger._('OTA');
  static final grpc = AppLogger._('gRPC');
  static final control = AppLogger._('CTRL');
  static final map = AppLogger._('MAP');
  static final task = AppLogger._('TASK');
  static final ui = AppLogger._('UI');
  static final cloud = AppLogger._('CLOUD');
  static final system = AppLogger._('SYS');

  /// Maximum entries in the ring buffer.
  static const int maxBufferSize = 2000;

  /// Shared ring buffer across all loggers.
  static final Queue<LogEntry> _buffer = Queue<LogEntry>();

  /// Minimum level to output to debugPrint (only in debug mode).
  static LogLevel consoleLevel = LogLevel.debug;

  /// Optional external handler for crash reporting integration.
  /// Set this to forward errors to Sentry/Crashlytics/etc.
  static void Function(LogEntry entry)? onError;

  final String tag;

  AppLogger._(this.tag);

  /// Create a custom named logger.
  factory AppLogger.named(String tag) => AppLogger._(tag);

  void debug(String message) => _log(LogLevel.debug, message);
  void info(String message) => _log(LogLevel.info, message);
  void warning(String message, {Object? error, StackTrace? stackTrace}) =>
      _log(LogLevel.warning, message, error: error, stackTrace: stackTrace);
  void error(String message, {Object? error, StackTrace? stackTrace}) =>
      _log(LogLevel.error, message, error: error, stackTrace: stackTrace);

  void _log(
    LogLevel level,
    String message, {
    Object? error,
    StackTrace? stackTrace,
  }) {
    final entry = LogEntry(
      timestamp: DateTime.now(),
      level: level,
      tag: tag,
      message: message,
      error: error,
      stackTrace: stackTrace,
    );

    // Ring buffer
    _buffer.addLast(entry);
    while (_buffer.length > maxBufferSize) {
      _buffer.removeFirst();
    }

    // Debug console output
    if (kDebugMode && level.index >= consoleLevel.index) {
      debugPrint(entry.toString());
      if (stackTrace != null && level == LogLevel.error) {
        debugPrint(stackTrace.toString());
      }
    }

    // Forward errors to external crash reporting
    if (level == LogLevel.error) {
      onError?.call(entry);
    }
  }

  /// Export all buffered logs as a single string (for log file export).
  static String exportLogs() {
    return _buffer.map((e) => e.toString()).join('\n');
  }

  /// Export recent N entries.
  static List<LogEntry> recentEntries([int count = 100]) {
    final list = _buffer.toList();
    if (list.length <= count) return list;
    return list.sublist(list.length - count);
  }

  /// Clear the log buffer.
  static void clear() => _buffer.clear();
}
