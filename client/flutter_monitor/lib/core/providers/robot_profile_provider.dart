import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_monitor/core/models/robot_profile.dart';

/// 管理当前选中的机器人型号
///
/// 持久化存储选中的 profile id，下次启动自动恢复。
/// 当 DogDirectClient 连接成功获取 GetParams 返回的 RobotType 后，
/// 可调用 [autoDetect] 自动切换到对应型号。
class RobotProfileProvider extends ChangeNotifier {
  static const _key = 'selected_robot_profile_id';

  RobotProfile _current = RobotProfiles.thunder;
  bool _loaded = false;

  RobotProfile get current => _current;
  bool get loaded => _loaded;

  /// 所有可用型号
  List<RobotProfile> get allProfiles => RobotProfiles.all;

  RobotProfileProvider() {
    _load();
  }

  Future<void> _load() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final savedId = prefs.getString(_key);
      if (savedId != null) {
        final found = RobotProfiles.byId(savedId);
        if (found != null) {
          _current = found;
        }
      }
    } catch (_) {
      // 首次运行或读取失败, 使用默认值
    }
    _loaded = true;
    notifyListeners();
  }

  /// 手动选择型号
  Future<void> select(RobotProfile profile) async {
    if (_current == profile) return;
    _current = profile;
    notifyListeners();
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_key, profile.id);
    } catch (_) {}
  }

  /// 根据 gRPC GetParams 返回的 RobotType 名自动匹配
  /// 返回 true 表示切换了型号
  Future<bool> autoDetect(String protoTypeName) async {
    final matched = RobotProfiles.byProtoType(protoTypeName);
    if (matched != null && matched != _current) {
      await select(matched);
      return true;
    }
    return false;
  }
}
