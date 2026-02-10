import 'package:flutter/material.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';

/// Global error boundary that catches widget tree errors and displays
/// a user-friendly fallback instead of crashing the app.
///
/// Wraps child widgets and intercepts errors from the Flutter framework,
/// showing a recovery screen with retry capability.
class ErrorBoundary extends StatefulWidget {
  final Widget child;

  const ErrorBoundary({super.key, required this.child});

  @override
  State<ErrorBoundary> createState() => _ErrorBoundaryState();
}

class _ErrorBoundaryState extends State<ErrorBoundary> {
  bool _hasError = false;
  FlutterErrorDetails? _errorDetails;

  @override
  void initState() {
    super.initState();

    // Override the default error widget builder globally
    ErrorWidget.builder = (FlutterErrorDetails details) {
      // Log the error
      AppLogger.system.error(
        'Widget error: ${details.exceptionAsString()}',
        error: details.exception,
        stackTrace: details.stack,
      );

      // Return a minimal error indicator instead of the red/yellow screen
      return _InlineErrorWidget(details: details);
    };
  }

  @override
  Widget build(BuildContext context) {
    if (_hasError) {
      return _FullScreenErrorWidget(
        details: _errorDetails,
        onRetry: () {
          setState(() {
            _hasError = false;
            _errorDetails = null;
          });
        },
      );
    }

    return widget.child;
  }
}

/// Inline error indicator — replaces a single broken widget without
/// destroying the entire screen. Shows a compact red indicator.
class _InlineErrorWidget extends StatelessWidget {
  final FlutterErrorDetails details;

  const _InlineErrorWidget({required this.details});

  @override
  Widget build(BuildContext context) {
    final isDark = Theme.of(context).brightness == Brightness.dark;

    return Container(
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.all(4),
      decoration: BoxDecoration(
        color: AppColors.error.withValues(alpha: isDark ? 0.15 : 0.08),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: AppColors.error.withValues(alpha: 0.3)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          const Icon(Icons.error_outline, color: AppColors.error, size: 18),
          const SizedBox(width: 8),
          Flexible(
            child: Text(
              '组件渲染错误',
              style: TextStyle(
                color: AppColors.error,
                fontSize: 12,
                fontWeight: FontWeight.w500,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

/// Full-screen error recovery page — displayed when an unrecoverable
/// error occurs in the widget tree.
class _FullScreenErrorWidget extends StatelessWidget {
  final FlutterErrorDetails? details;
  final VoidCallback onRetry;

  const _FullScreenErrorWidget({
    this.details,
    required this.onRetry,
  });

  @override
  Widget build(BuildContext context) {
    return Material(
      child: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(32),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const Icon(
                Icons.warning_amber_rounded,
                size: 64,
                color: AppColors.warning,
              ),
              const SizedBox(height: 24),
              Text(
                '出了点问题',
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.w700,
                  color: context.titleColor,
                ),
              ),
              const SizedBox(height: 8),
              Text(
                '应用遇到了意外错误，您可以尝试重试。',
                style: TextStyle(
                  fontSize: 14,
                  color: context.subtitleColor,
                ),
                textAlign: TextAlign.center,
              ),
              if (details != null) ...[
                const SizedBox(height: 16),
                Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: context.isDark ? AppColors.darkCard : AppColors.lightSurface,
                    borderRadius: BorderRadius.circular(AppRadius.sm),
                    border: context.isDark ? Border.all(color: AppColors.borderDark) : null,
                    boxShadow: context.isDark ? null : [AppShadows.light()],
                  ),
                  child: Text(
                    details!.exceptionAsString(),
                    style: TextStyle(
                      fontSize: 11,
                      fontFamily: 'monospace',
                      color: context.subtitleColor,
                    ),
                    maxLines: 4,
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
              ],
              const SizedBox(height: 32),
              ElevatedButton.icon(
                onPressed: onRetry,
                icon: const Icon(Icons.refresh),
                label: const Text('重试'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: AppColors.primary,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(
                      horizontal: 32, vertical: 14),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(AppRadius.md),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
