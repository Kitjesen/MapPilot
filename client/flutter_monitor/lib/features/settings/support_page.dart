import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

class SupportPage extends StatelessWidget {
  const SupportPage({super.key});

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: const Text('反馈与支持'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== 联系方式 =====
          _SupportCard(
            icon: Icons.email_outlined,
            title: '技术支持邮箱',
            subtitle: 'support@dasuan-robot.com',
            action: '复制邮箱',
            onAction: () => _copyToClipboard(context, 'support@dasuan-robot.com', '邮箱已复制'),
          ),
          const SizedBox(height: 12),
          _SupportCard(
            icon: Icons.language,
            title: '文档中心',
            subtitle: 'docs.dasuan-robot.com',
            action: '复制链接',
            onAction: () => _copyToClipboard(context, 'https://docs.dasuan-robot.com', '链接已复制'),
          ),
          const SizedBox(height: 12),
          _SupportCard(
            icon: Icons.forum_outlined,
            title: '社区论坛',
            subtitle: '与其他用户交流',
            action: '复制链接',
            onAction: () => _copyToClipboard(context, 'https://community.dasuan-robot.com', '链接已复制'),
          ),
          const SizedBox(height: 12),
          _SupportCard(
            icon: Icons.question_answer_outlined,
            title: '常见问题 FAQ',
            subtitle: '查看常见问题及解答',
            action: '复制链接',
            onAction: () => _copyToClipboard(context, 'https://docs.dasuan-robot.com/faq', '链接已复制'),
          ),
          const SizedBox(height: 24),

          // ===== 一键提交设备信息 =====
          Container(
            padding: const EdgeInsets.all(20),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  '一键复制设备信息',
                  style: TextStyle(
                    fontSize: 15,
                    fontWeight: FontWeight.w600,
                    color: context.titleColor,
                  ),
                ),
                const SizedBox(height: 6),
                Text(
                  '反馈问题时附带设备信息，可以帮助我们更快定位问题。',
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                    height: 1.4,
                  ),
                ),
                const SizedBox(height: 14),
                SizedBox(
                  width: double.infinity,
                  height: 40,
                  child: TextButton(
                    onPressed: () => _copyDeviceInfo(context),
                    style: TextButton.styleFrom(
                      foregroundColor: context.titleColor,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8),
                        side: BorderSide(color: context.borderColor),
                      ),
                    ),
                    child: const Text('复制设备信息', style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500)),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  void _copyToClipboard(BuildContext context, String text, String message) {
    Clipboard.setData(ClipboardData(text: text));
    HapticFeedback.lightImpact();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        duration: const Duration(seconds: 1),
      ),
    );
  }

  void _copyDeviceInfo(BuildContext context) {
    final provider = context.read<RobotConnectionProvider>();
    final slow = provider.latestSlowState;

    final info = StringBuffer();
    info.writeln('=== 大算机器人 设备信息 ===');
    info.writeln('APP 版本: v1.0.0 (Build 1)');
    info.writeln('协议: gRPC + WebRTC');
    info.writeln('连接状态: ${provider.isConnected ? "已连接" : "未连接"}');
    if (slow != null) {
      info.writeln('电池: ${slow.resources.batteryPercent.toStringAsFixed(0)}%');
      info.writeln('CPU: ${slow.resources.cpuPercent.toStringAsFixed(0)}%');
      info.writeln('温度: ${slow.resources.cpuTemp.toStringAsFixed(1)}°C');
      info.writeln('模式: ${slow.currentMode}');
    }
    info.writeln('时间: ${DateTime.now().toIso8601String()}');

    _copyToClipboard(context, info.toString(), '设备信息已复制到剪贴板');
  }
}

class _SupportCard extends StatelessWidget {
  final IconData icon;
  final String title;
  final String subtitle;
  final String action;
  final VoidCallback onAction;

  const _SupportCard({
    required this.icon,
    required this.title,
    required this.subtitle,
    required this.action,
    required this.onAction,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Row(
        children: [
          Icon(icon, color: context.subtitleColor, size: 18),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w500,
                    color: context.titleColor,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  subtitle,
                  style: TextStyle(fontSize: 12, color: context.subtitleColor),
                ),
              ],
            ),
          ),
          TextButton(
            onPressed: onAction,
            style: TextButton.styleFrom(
              foregroundColor: context.titleColor,
              visualDensity: VisualDensity.compact,
            ),
            child: Text(action, style: const TextStyle(fontSize: 12)),
          ),
        ],
      ),
    );
  }
}
