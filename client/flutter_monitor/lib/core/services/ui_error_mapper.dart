import 'package:flutter_monitor/core/services/grpc_error_formatter.dart';

/// Maps raw errors/messages to user-facing Chinese hints.
class UiErrorMapper {
  UiErrorMapper._();

  static String fromMessage(String? message) {
    final raw = (message ?? '').trim();
    if (raw.isEmpty) return '操作失败，请稍后重试';

    final lower = raw.toLowerCase();
    if (lower.contains('error_code_lease_conflict') ||
        lower.contains('lease_conflict') ||
        lower.contains('租约冲突') ||
        lower.contains('2001')) {
      return '控制权冲突：当前有其他客户端占用控制租约';
    }
    if (lower.contains('error_code_mode_conflict') ||
        lower.contains('mode_conflict') ||
        lower.contains('模式冲突') ||
        lower.contains('2004')) {
      return '模式冲突：请先切换到允许当前操作的机器人模式';
    }
    if (lower.contains('未连接') || lower.contains('not connected')) {
      return '未连接机器人';
    }
    if (lower.contains('timeout') || lower.contains('deadline exceeded')) {
      return '请求超时，请重试';
    }
    if (lower.contains('not found') || lower.contains('不存在')) {
      return '资源不存在或已删除';
    }
    if (lower.contains('invalid argument') ||
        lower.contains('参数错误') ||
        lower.contains('参数非法')) {
      return '参数错误，请检查输入';
    }
    if (lower.contains('busy') ||
        lower.contains('resource exhausted') ||
        lower.contains('任务执行中')) {
      return '机器人忙，请稍后重试';
    }

    // Best-effort fallback for grpc/exception formatted strings.
    return GrpcErrorFormatter.format(raw);
  }
}
