import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// 应用语言
enum AppLocale { zh, en }

/// 全局语言切换 Provider
///
/// 存储在 SharedPreferences，重启后记忆上次选择。
class LocaleProvider extends ChangeNotifier {
  static const _key = 'app_locale';

  AppLocale _locale = AppLocale.zh;
  AppLocale get locale => _locale;
  bool get isEn => _locale == AppLocale.en;
  bool get isZh => _locale == AppLocale.zh;

  LocaleProvider() {
    _load();
  }

  Future<void> _load() async {
    final prefs = await SharedPreferences.getInstance();
    final v = prefs.getString(_key);
    if (v == 'en') _locale = AppLocale.en;
    notifyListeners();
  }

  Future<void> setLocale(AppLocale l) async {
    if (_locale == l) return;
    _locale = l;
    notifyListeners();
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_key, l.name);
  }

  void toggle() => setLocale(isZh ? AppLocale.en : AppLocale.zh);

  /// 双语快捷选择
  String tr(String zh, String en) => isEn ? en : zh;
}
