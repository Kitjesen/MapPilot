import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_monitor/core/models/task_template.dart';
import 'package:flutter_monitor/core/models/scheduled_task.dart';

/// Centralized settings preferences with real persistence.
/// All values are read/written via SharedPreferences.
class SettingsPreferences extends ChangeNotifier {
  static const _keyTaskTemplates = 'task_templates';
  static const _maxTemplates = 20;
  static const _keyScheduledTasks = 'scheduled_tasks';
  static const _maxScheduledTasks = 10;

  static const _keyAutoReconnect = 'auto_reconnect';
  static const _keyConnectionTimeout = 'connection_timeout';
  static const _keyHapticFeedback = 'haptic_feedback';
  static const _keySavedDevices = 'saved_devices';
  static const _keyAlertInApp = 'alert_in_app';
  static const _keyAlertSystem = 'alert_system';
  static const _keyAlertBatteryLow = 'alert_battery_low';
  static const _keyAlertTempHigh = 'alert_temp_high';
  static const _keyAlertCommLost = 'alert_comm_lost';

  SharedPreferences? _prefs;

  // ====== Defaults ======
  List<ScheduledTask> _scheduledTasks = [];
  List<TaskTemplate> _taskTemplates = [];
  bool _autoReconnect = true;
  int _connectionTimeoutSec = 5;
  bool _hapticFeedback = true;
  List<SavedDevice> _savedDevices = [];
  bool _alertInApp = true;
  bool _alertSystem = false;
  bool _alertBatteryLow = true;
  bool _alertTempHigh = true;
  bool _alertCommLost = true;

  // ====== Getters ======
  List<ScheduledTask> get scheduledTasks => List.unmodifiable(_scheduledTasks);
  List<TaskTemplate> get taskTemplates => List.unmodifiable(_taskTemplates);
  bool get autoReconnect => _autoReconnect;
  int get connectionTimeoutSec => _connectionTimeoutSec;
  bool get hapticFeedback => _hapticFeedback;
  List<SavedDevice> get savedDevices => List.unmodifiable(_savedDevices);
  bool get alertInApp => _alertInApp;
  bool get alertSystem => _alertSystem;
  bool get alertBatteryLow => _alertBatteryLow;
  bool get alertTempHigh => _alertTempHigh;
  bool get alertCommLost => _alertCommLost;

  SettingsPreferences() {
    _load();
  }

  Future<void> _load() async {
    _prefs = await SharedPreferences.getInstance();
    _autoReconnect = _prefs!.getBool(_keyAutoReconnect) ?? true;
    _connectionTimeoutSec = _prefs!.getInt(_keyConnectionTimeout) ?? 5;
    _hapticFeedback = _prefs!.getBool(_keyHapticFeedback) ?? true;
    _alertInApp = _prefs!.getBool(_keyAlertInApp) ?? true;
    _alertSystem = _prefs!.getBool(_keyAlertSystem) ?? false;
    _alertBatteryLow = _prefs!.getBool(_keyAlertBatteryLow) ?? true;
    _alertTempHigh = _prefs!.getBool(_keyAlertTempHigh) ?? true;
    _alertCommLost = _prefs!.getBool(_keyAlertCommLost) ?? true;

    // Load task templates
    final templatesJson = _prefs!.getString(_keyTaskTemplates);
    if (templatesJson != null) {
      try {
        final list = jsonDecode(templatesJson) as List;
        _taskTemplates = list
            .map((e) => TaskTemplate.fromJson(e as Map<String, dynamic>))
            .toList();
      } catch (_) {
        _taskTemplates = [];
      }
    }

    // Load scheduled tasks
    final scheduledJson = _prefs!.getString(_keyScheduledTasks);
    if (scheduledJson != null) {
      try {
        final list = jsonDecode(scheduledJson) as List;
        _scheduledTasks = list
            .map((e) => ScheduledTask.fromJson(e as Map<String, dynamic>))
            .toList();
      } catch (_) {
        _scheduledTasks = [];
      }
    }

    // Load saved devices
    final devicesJson = _prefs!.getString(_keySavedDevices);
    if (devicesJson != null) {
      try {
        final list = jsonDecode(devicesJson) as List;
        _savedDevices = list.map((e) => SavedDevice.fromJson(e)).toList();
      } catch (_) {
        _savedDevices = [];
      }
    }

    notifyListeners();
  }

  // ====== Setters ======

  Future<void> setAutoReconnect(bool value) async {
    _autoReconnect = value;
    await _prefs?.setBool(_keyAutoReconnect, value);
    notifyListeners();
  }

  Future<void> setConnectionTimeout(int seconds) async {
    _connectionTimeoutSec = seconds;
    await _prefs?.setInt(_keyConnectionTimeout, seconds);
    notifyListeners();
  }

  Future<void> setHapticFeedback(bool value) async {
    _hapticFeedback = value;
    await _prefs?.setBool(_keyHapticFeedback, value);
    notifyListeners();
  }

  Future<void> setAlertInApp(bool value) async {
    _alertInApp = value;
    await _prefs?.setBool(_keyAlertInApp, value);
    notifyListeners();
  }

  Future<void> setAlertSystem(bool value) async {
    _alertSystem = value;
    await _prefs?.setBool(_keyAlertSystem, value);
    notifyListeners();
  }

  Future<void> setAlertBatteryLow(bool value) async {
    _alertBatteryLow = value;
    await _prefs?.setBool(_keyAlertBatteryLow, value);
    notifyListeners();
  }

  Future<void> setAlertTempHigh(bool value) async {
    _alertTempHigh = value;
    await _prefs?.setBool(_keyAlertTempHigh, value);
    notifyListeners();
  }

  Future<void> setAlertCommLost(bool value) async {
    _alertCommLost = value;
    await _prefs?.setBool(_keyAlertCommLost, value);
    notifyListeners();
  }

  // ====== Saved Devices ======

  Future<void> addOrUpdateDevice(SavedDevice device) async {
    final idx = _savedDevices.indexWhere(
        (d) => d.host == device.host && d.port == device.port);
    if (idx >= 0) {
      _savedDevices[idx] = device;
    } else {
      _savedDevices.insert(0, device);
    }
    await _persistDevices();
    notifyListeners();
  }

  Future<void> removeDevice(SavedDevice device) async {
    _savedDevices.removeWhere(
        (d) => d.host == device.host && d.port == device.port);
    await _persistDevices();
    notifyListeners();
  }

  Future<void> clearAllDevices() async {
    _savedDevices.clear();
    await _persistDevices();
    notifyListeners();
  }

  Future<void> setDefaultDevice(SavedDevice device) async {
    for (var d in _savedDevices) {
      d.isDefault = (d.host == device.host && d.port == device.port);
    }
    await _persistDevices();
    notifyListeners();
  }

  SavedDevice? get defaultDevice {
    try {
      return _savedDevices.firstWhere((d) => d.isDefault);
    } catch (_) {
      return _savedDevices.isNotEmpty ? _savedDevices.first : null;
    }
  }

  // ====== Task Templates ======

  Future<void> saveTemplate(TaskTemplate template) async {
    // 重名则覆盖
    final idx = _taskTemplates.indexWhere((t) => t.name == template.name);
    if (idx >= 0) {
      _taskTemplates[idx] = template;
    } else {
      _taskTemplates.insert(0, template);
      if (_taskTemplates.length > _maxTemplates) {
        _taskTemplates = _taskTemplates.take(_maxTemplates).toList();
      }
    }
    await _persistTemplates();
    notifyListeners();
  }

  Future<void> deleteTemplate(String name) async {
    _taskTemplates.removeWhere((t) => t.name == name);
    await _persistTemplates();
    notifyListeners();
  }

  // ====== Scheduled Tasks ======

  Future<void> saveScheduledTask(ScheduledTask task) async {
    final idx = _scheduledTasks.indexWhere((t) => t.id == task.id);
    if (idx >= 0) {
      _scheduledTasks[idx] = task;
    } else {
      _scheduledTasks.insert(0, task);
      if (_scheduledTasks.length > _maxScheduledTasks) {
        _scheduledTasks = _scheduledTasks.take(_maxScheduledTasks).toList();
      }
    }
    await _persistScheduledTasks();
    notifyListeners();
  }

  Future<void> deleteScheduledTask(String id) async {
    _scheduledTasks.removeWhere((t) => t.id == id);
    await _persistScheduledTasks();
    notifyListeners();
  }

  Future<void> toggleScheduledTask(String id, bool enabled) async {
    final idx = _scheduledTasks.indexWhere((t) => t.id == id);
    if (idx >= 0) {
      _scheduledTasks[idx] = _scheduledTasks[idx].copyWith(enabled: enabled);
      await _persistScheduledTasks();
      notifyListeners();
    }
  }

  Future<void> _persistScheduledTasks() async {
    final json =
        jsonEncode(_scheduledTasks.map((t) => t.toJson()).toList());
    await _prefs?.setString(_keyScheduledTasks, json);
  }

  Future<void> _persistTemplates() async {
    final json = jsonEncode(_taskTemplates.map((t) => t.toJson()).toList());
    await _prefs?.setString(_keyTaskTemplates, json);
  }

  Future<void> _persistDevices() async {
    final json = jsonEncode(_savedDevices.map((d) => d.toJson()).toList());
    await _prefs?.setString(_keySavedDevices, json);
  }

  /// Clear non-critical data (templates, scheduled tasks).
  /// Keeps saved devices and alert settings.
  Future<void> clearNonCriticalData() async {
    _taskTemplates.clear();
    _scheduledTasks.clear();
    await _prefs?.remove(_keyTaskTemplates);
    await _prefs?.remove(_keyScheduledTasks);
    notifyListeners();
  }

  /// Estimated size of stored preferences data (rough byte count).
  int get estimatedStorageBytes {
    int total = 0;
    final p = _prefs;
    if (p == null) return 0;
    for (final key in [
      _keyTaskTemplates,
      _keyScheduledTasks,
      _keySavedDevices,
    ]) {
      final v = p.getString(key);
      if (v != null) total += v.length * 2; // UTF-16 estimate
    }
    return total;
  }
}

/// Represents a saved robot device connection.
class SavedDevice {
  String name;
  String host;
  int port;
  DateTime lastConnected;
  bool isDefault;

  SavedDevice({
    required this.name,
    required this.host,
    required this.port,
    required this.lastConnected,
    this.isDefault = false,
  });

  factory SavedDevice.fromJson(Map<String, dynamic> json) {
    return SavedDevice(
      name: json['name'] ?? '',
      host: json['host'] ?? '',
      port: json['port'] ?? 50051,
      lastConnected: DateTime.tryParse(json['lastConnected'] ?? '') ?? DateTime.now(),
      isDefault: json['isDefault'] ?? false,
    );
  }

  Map<String, dynamic> toJson() => {
        'name': name,
        'host': host,
        'port': port,
        'lastConnected': lastConnected.toIso8601String(),
        'isDefault': isDefault,
      };
}
