import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/shared/utils/haptic_utils.dart';

/// Global safety overlay that provides:
/// 1. A persistent floating emergency stop button (visible when connected)
/// 2. A connection status banner (visible when reconnecting/disconnected)
///
/// This widget should wrap the main app content so it appears above all screens.
class GlobalSafetyOverlay extends StatelessWidget {
  final Widget child;

  const GlobalSafetyOverlay({super.key, required this.child});

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        child,
        const _ConnectionStatusBanner(),
        const _EmergencyStopFAB(),
      ],
    );
  }
}

// ============================================================
// Connection Status Banner
// ============================================================

class _ConnectionStatusBanner extends StatelessWidget {
  const _ConnectionStatusBanner();

  @override
  Widget build(BuildContext context) {
    final (:status, :errorMessage) = context.select<RobotConnectionProvider,
        ({ConnectionStatus status, String? errorMessage})>(
      (p) => (status: p.status, errorMessage: p.errorMessage),
    );

    // Only show for non-normal states when previously connected
    final shouldShow = status == ConnectionStatus.reconnecting ||
        status == ConnectionStatus.error;

    if (!shouldShow) return const SizedBox.shrink();

    final isReconnecting = status == ConnectionStatus.reconnecting;
    final isDark = context.isDark;
    final color = isReconnecting ? AppColors.warning : AppColors.error;

    return Positioned(
      top: 0,
      left: 0,
      right: 0,
      child: SafeArea(
        bottom: false,
        child: TweenAnimationBuilder<double>(
          tween: Tween(begin: -1.0, end: 0.0),
          duration: const Duration(milliseconds: 350),
          curve: Curves.easeOutCubic,
          builder: (context, value, child) => Transform.translate(
            offset: Offset(0, value * 60),
            child: child,
          ),
          child: Container(
            margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
            decoration: BoxDecoration(
              color: isDark
                  ? color.withValues(alpha: 0.2)
                  : color.withValues(alpha: 0.08),
              borderRadius: BorderRadius.circular(AppRadius.card),
              border: isDark ? Border.all(color: color.withValues(alpha: 0.3)) : null,
              boxShadow: isDark ? null : [AppShadows.light()],
            ),
            child: Row(
              children: [
                if (isReconnecting) ...[
                  SizedBox(
                    width: 14,
                    height: 14,
                    child: CircularProgressIndicator(
                      strokeWidth: 2,
                      color: color,
                    ),
                  ),
                ] else ...[
                  Icon(Icons.link_off, size: 16, color: color),
                ],
                const SizedBox(width: 10),
                Expanded(
                  child: Text(
                    isReconnecting
                        ? '正在重新连接…'
                        : '连接已断开: ${errorMessage ?? "未知错误"}',
                    style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                      color: color,
                    ),
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
                if (!isReconnecting) ...[
                  const SizedBox(width: 8),
                  GestureDetector(
                    onTap: () {
                      // Navigate back to scan screen
                      Navigator.of(context).pushNamedAndRemoveUntil(
                        '/scan',
                        (route) => false,
                      );
                    },
                    child: Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 10, vertical: 4),
                      decoration: BoxDecoration(
                        color: color.withValues(alpha: 0.15),
                        borderRadius: BorderRadius.circular(6),
                      ),
                      child: Text(
                        '重连',
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w600,
                          color: color,
                        ),
                      ),
                    ),
                  ),
                ],
              ],
            ),
          ),
        ),
      ),
    );
  }
}

// ============================================================
// Emergency Stop Floating Action Button
// ============================================================

class _EmergencyStopFAB extends StatefulWidget {
  const _EmergencyStopFAB();

  @override
  State<_EmergencyStopFAB> createState() => _EmergencyStopFABState();
}

class _EmergencyStopFABState extends State<_EmergencyStopFAB>
    with SingleTickerProviderStateMixin {
  bool _triggered = false;
  late final AnimationController _pulseController;
  late final Animation<double> _pulseAnimation;

  @override
  void initState() {
    super.initState();
    _pulseController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1500),
    )..repeat(reverse: true);
    _pulseAnimation = Tween<double>(begin: 1.0, end: 1.08).animate(
      CurvedAnimation(parent: _pulseController, curve: Curves.easeInOut),
    );
  }

  @override
  void dispose() {
    _pulseController.dispose();
    super.dispose();
  }

  Future<void> _onEmergencyStop() async {
    HapticUtils.heavy();

    setState(() => _triggered = true);

    final controlGateway = context.read<ControlGateway>();
    try {
      await controlGateway.emergencyStop();
    } catch (_) {
      // E-stop must never throw to the user
    }

    // Visual feedback: flash red for 2 seconds
    await Future.delayed(const Duration(seconds: 2));
    if (mounted) setState(() => _triggered = false);
  }

  Future<void> _onClearEmergencyStop() async {
    HapticFeedback.mediumImpact();

    final reason = await showDialog<String>(
      context: context,
      builder: (_) => const _ClearEstopDialog(),
    );
    if (reason == null || !mounted) return;

    final controlGateway = context.read<ControlGateway>();
    final (ok, msg) = await controlGateway.clearEmergencyStop(reason: reason);
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok ? '急停已解除' : msg),
        backgroundColor: ok ? AppColors.success : AppColors.error,
        behavior: SnackBarBehavior.floating,
      ));
    }
  }

  @override
  Widget build(BuildContext context) {
    final isConnected = context.select<RobotConnectionProvider, bool>(
      (p) => p.isConnected || p.isDogConnected,
    );

    // Don't show if not connected
    if (!isConnected) return const SizedBox.shrink();

    final isEstop = context.select<ControlGateway, bool>(
      (g) => g.currentMode == RobotMode.ROBOT_MODE_ESTOP,
    );

    // In ESTOP mode: show amber "clear" button instead of red "stop" button
    if (isEstop) {
      return Positioned(
        right: 16,
        bottom: 90,
        child: ScaleTransition(
          scale: _pulseAnimation,
          child: GestureDetector(
            onTap: _onClearEmergencyStop,
            child: Container(
              width: 52,
              height: 52,
              decoration: BoxDecoration(
                color: AppColors.warning.withValues(alpha: 0.9),
                shape: BoxShape.circle,
                boxShadow: [
                  BoxShadow(
                    color: AppColors.warning.withValues(alpha: 0.4),
                    blurRadius: 12,
                  ),
                ],
              ),
              child: const Center(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Icon(Icons.lock_open, color: Colors.white, size: 18),
                    Text(
                      'CLEAR',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 7,
                        fontWeight: FontWeight.w900,
                        letterSpacing: 0.5,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      );
    }

    return Positioned(
      right: 16,
      bottom: 90, // above the bottom nav bar
      child: ScaleTransition(
        scale: _pulseAnimation,
        child: GestureDetector(
          onTap: _onEmergencyStop,
          child: AnimatedContainer(
            duration: const Duration(milliseconds: 200),
            width: 52,
            height: 52,
            decoration: BoxDecoration(
              color: _triggered
                  ? AppColors.error
                  : AppColors.error.withValues(alpha: 0.85),
              shape: BoxShape.circle,
              boxShadow: [
                BoxShadow(
                  color: AppColors.error.withValues(alpha: _triggered ? 0.6 : 0.3),
                  blurRadius: _triggered ? 20 : 12,
                  spreadRadius: _triggered ? 4 : 0,
                ),
              ],
            ),
            child: Center(
              child: _triggered
                  ? const Icon(Icons.check, color: Colors.white, size: 24)
                  : const Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(Icons.front_hand, color: Colors.white, size: 18),
                        Text(
                          'STOP',
                          style: TextStyle(
                            color: Colors.white,
                            fontSize: 8,
                            fontWeight: FontWeight.w900,
                            letterSpacing: 1.0,
                          ),
                        ),
                      ],
                    ),
            ),
          ),
        ),
      ),
    );
  }
}

// ============================================================
// E-stop Clear Confirmation Dialog
// ============================================================

class _ClearEstopDialog extends StatefulWidget {
  const _ClearEstopDialog();

  @override
  State<_ClearEstopDialog> createState() => _ClearEstopDialogState();
}

class _ClearEstopDialogState extends State<_ClearEstopDialog> {
  static const _reasons = [
    '检查完毕',
    '误触急停',
    '障碍已清除',
    '其他',
  ];

  String? _selectedReason;

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('确认危险已消除？'),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            '解除急停后机器人将恢复运动能力，请确认周围环境安全。',
            style: TextStyle(fontSize: 13, color: Colors.grey.shade700),
          ),
          const SizedBox(height: 16),
          DropdownButtonFormField<String>(
            value: _selectedReason,
            decoration: const InputDecoration(
              labelText: '解除原因',
              border: OutlineInputBorder(),
              contentPadding: EdgeInsets.symmetric(horizontal: 12, vertical: 10),
            ),
            items: _reasons
                .map((r) => DropdownMenuItem(value: r, child: Text(r)))
                .toList(),
            onChanged: (v) => setState(() => _selectedReason = v),
          ),
        ],
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.of(context).pop(),
          child: const Text('取消'),
        ),
        FilledButton(
          onPressed: _selectedReason != null
              ? () => Navigator.of(context).pop(_selectedReason)
              : null,
          style: FilledButton.styleFrom(
            backgroundColor: AppColors.warning,
          ),
          child: const Text('确认解除'),
        ),
      ],
    );
  }
}
