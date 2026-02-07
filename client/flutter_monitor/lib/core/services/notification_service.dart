import 'dart:io';
import 'package:flutter/foundation.dart';
import 'package:flutter_local_notifications/flutter_local_notifications.dart';

/// 系统级通知服务
///
/// 封装 [flutter_local_notifications] 插件，提供 OS 级推送通知。
/// 由 [AlertMonitorService] 在检测到告警时调用。
class NotificationService {
  static final NotificationService _instance = NotificationService._();
  factory NotificationService() => _instance;
  NotificationService._();

  final FlutterLocalNotificationsPlugin _plugin =
      FlutterLocalNotificationsPlugin();

  bool _initialized = false;
  bool get isInitialized => _initialized;

  /// 初始化通知插件（在 main() 中调用一次）
  Future<void> initialize() async {
    if (_initialized) return;

    const androidSettings =
        AndroidInitializationSettings('@mipmap/ic_launcher');
    const iosSettings = DarwinInitializationSettings(
      requestAlertPermission: true,
      requestBadgePermission: true,
      requestSoundPermission: true,
    );
    const initSettings = InitializationSettings(
      android: androidSettings,
      iOS: iosSettings,
    );

    await _plugin.initialize(
      initSettings,
      onDidReceiveNotificationResponse: _onNotificationTapped,
    );

    // 请求 Android 13+ 通知权限
    if (Platform.isAndroid) {
      await _plugin
          .resolvePlatformSpecificImplementation<
              AndroidFlutterLocalNotificationsPlugin>()
          ?.requestNotificationsPermission();
    }

    _initialized = true;
    debugPrint('[NotificationService] Initialized');
  }

  void _onNotificationTapped(NotificationResponse response) {
    debugPrint(
        '[NotificationService] Tapped: id=${response.id}, payload=${response.payload}');
    // 可以在这里处理点击通知后的导航
  }

  /// 通知渠道定义
  static const _robotChannel = AndroidNotificationDetails(
    'robot_alerts',
    '机器人告警',
    channelDescription: '来自机器人的状态告警通知',
    importance: Importance.high,
    priority: Priority.high,
    showWhen: true,
    enableVibration: true,
  );

  static const _criticalChannel = AndroidNotificationDetails(
    'robot_critical',
    '紧急告警',
    channelDescription: '紧急告警（通信断开、电量极低）',
    importance: Importance.max,
    priority: Priority.max,
    showWhen: true,
    enableVibration: true,
    playSound: true,
  );

  /// 显示普通告警通知
  Future<void> showAlert({
    required int id,
    required String title,
    required String body,
    String? payload,
  }) async {
    if (!_initialized) return;
    const details = NotificationDetails(
      android: _robotChannel,
      iOS: DarwinNotificationDetails(),
    );
    await _plugin.show(id, title, body, details, payload: payload);
  }

  /// 显示紧急告警通知
  Future<void> showCriticalAlert({
    required int id,
    required String title,
    required String body,
    String? payload,
  }) async {
    if (!_initialized) return;
    const details = NotificationDetails(
      android: _criticalChannel,
      iOS: DarwinNotificationDetails(
        presentAlert: true,
        presentBadge: true,
        presentSound: true,
      ),
    );
    await _plugin.show(id, title, body, details, payload: payload);
  }

  /// 取消特定通知
  Future<void> cancel(int id) async {
    await _plugin.cancel(id);
  }

  /// 取消所有通知
  Future<void> cancelAll() async {
    await _plugin.cancelAll();
  }
}
