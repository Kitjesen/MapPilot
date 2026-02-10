import 'package:grpc/grpc.dart';

/// Converts gRPC errors into user-friendly Chinese messages.
///
/// Usage:
/// ```dart
/// try {
///   await client.someRpc();
/// } catch (e) {
///   final msg = GrpcErrorFormatter.format(e);
///   showSnackBar(msg);
/// }
/// ```
class GrpcErrorFormatter {
  GrpcErrorFormatter._();

  /// Format any error into a user-friendly message.
  /// If the error is a [GrpcError], map the status code.
  /// Otherwise, return the error's toString().
  static String format(Object error) {
    if (error is GrpcError) {
      return _formatGrpcError(error);
    }
    final msg = error.toString();
    // Strip common Dart exception prefixes
    if (msg.startsWith('Exception: ')) {
      return msg.substring('Exception: '.length);
    }
    return msg;
  }

  static String _formatGrpcError(GrpcError error) {
    switch (error.code) {
      case StatusCode.ok:
        return '操作成功';
      case StatusCode.cancelled:
        return '操作已取消';
      case StatusCode.unknown:
        return '未知错误：${error.message ?? '请重试'}';
      case StatusCode.invalidArgument:
        return '参数错误：${error.message ?? '请检查输入'}';
      case StatusCode.deadlineExceeded:
        return '操作超时，请检查网络后重试';
      case StatusCode.notFound:
        return '未找到请求的资源';
      case StatusCode.alreadyExists:
        return '资源已存在';
      case StatusCode.permissionDenied:
        return '权限不足，请检查设备配对';
      case StatusCode.resourceExhausted:
        return '资源不足（磁盘/内存），请清理后重试';
      case StatusCode.failedPrecondition:
        return '前置条件不满足，请检查机器人状态';
      case StatusCode.aborted:
        return '操作被中断，请重试';
      case StatusCode.outOfRange:
        return '参数超出范围';
      case StatusCode.unimplemented:
        return '机器人不支持此功能（固件版本过低）';
      case StatusCode.internal:
        return '机器人内部错误，请查看日志';
      case StatusCode.unavailable:
        return '机器人未响应，请检查网络连接';
      case StatusCode.dataLoss:
        return '数据损坏，请重试';
      case StatusCode.unauthenticated:
        return '未认证，请重新连接';
      default:
        return '连接错误 (${error.code}): ${error.message ?? ''}';
    }
  }

  /// Whether this error is recoverable (user can retry).
  static bool isRetryable(Object error) {
    if (error is GrpcError) {
      return const [
        StatusCode.deadlineExceeded,
        StatusCode.unavailable,
        StatusCode.aborted,
        StatusCode.resourceExhausted,
        StatusCode.unknown,
      ].contains(error.code);
    }
    return false;
  }

  /// Whether this error means the connection is likely dead.
  static bool isConnectionError(Object error) {
    if (error is GrpcError) {
      return const [
        StatusCode.unavailable,
        StatusCode.deadlineExceeded,
      ].contains(error.code);
    }
    return false;
  }
}
