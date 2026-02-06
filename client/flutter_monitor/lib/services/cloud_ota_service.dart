import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';

/// GitHub Release 资产信息
class CloudAsset {
  final String name;
  final int size;
  final String downloadUrl;
  final String contentType;
  final int downloadCount;

  /// 从文件扩展名推断的分类
  String get category {
    final ext = name.split('.').last.toLowerCase();
    if (['pt', 'pth', 'onnx', 'tflite', 'engine'].contains(ext)) {
      return 'model';
    } else if (['pcd', 'ply', 'pickle', 'pgm'].contains(ext)) {
      return 'map';
    } else if (['yaml', 'yml', 'json', 'xml', 'cfg', 'ini'].contains(ext)) {
      return 'config';
    } else if (['bin', 'hex', 'img', 'deb'].contains(ext)) {
      return 'firmware';
    } else if (['apk'].contains(ext)) {
      return 'apk';
    }
    return 'other';
  }

  String get formattedSize {
    if (size < 1024) return '$size B';
    if (size < 1024 * 1024) return '${(size / 1024).toStringAsFixed(1)} KB';
    if (size < 1024 * 1024 * 1024) {
      return '${(size / (1024 * 1024)).toStringAsFixed(1)} MB';
    }
    return '${(size / (1024 * 1024 * 1024)).toStringAsFixed(2)} GB';
  }

  CloudAsset({
    required this.name,
    required this.size,
    required this.downloadUrl,
    required this.contentType,
    required this.downloadCount,
  });

  factory CloudAsset.fromJson(Map<String, dynamic> json) {
    return CloudAsset(
      name: json['name'] ?? '',
      size: json['size'] ?? 0,
      downloadUrl: json['browser_download_url'] ?? '',
      contentType: json['content_type'] ?? '',
      downloadCount: json['download_count'] ?? 0,
    );
  }
}

/// GitHub Release 信息
class CloudRelease {
  final String tagName;
  final String name;
  final String body;
  final String publishedAt;
  final String htmlUrl;
  final bool prerelease;
  final bool draft;
  final List<CloudAsset> assets;

  /// 发布时间的友好显示
  String get publishedDate {
    try {
      final dt = DateTime.parse(publishedAt);
      final now = DateTime.now();
      final diff = now.difference(dt);
      if (diff.inMinutes < 60) return '${diff.inMinutes} 分钟前';
      if (diff.inHours < 24) return '${diff.inHours} 小时前';
      if (diff.inDays < 30) return '${diff.inDays} 天前';
      return '${dt.year}-${dt.month.toString().padLeft(2, '0')}-${dt.day.toString().padLeft(2, '0')}';
    } catch (_) {
      return publishedAt;
    }
  }

  /// 按分类过滤的资产（排除 APK 和 source code）
  List<CloudAsset> get deployableAssets => assets
      .where((a) => a.category != 'apk' && a.category != 'other')
      .toList();

  /// 所有资产（包括 APK）
  List<CloudAsset> get allAssets => assets;

  CloudRelease({
    required this.tagName,
    required this.name,
    required this.body,
    required this.publishedAt,
    required this.htmlUrl,
    required this.prerelease,
    required this.draft,
    required this.assets,
  });

  factory CloudRelease.fromJson(Map<String, dynamic> json) {
    final assetList = (json['assets'] as List<dynamic>?)
            ?.map((a) => CloudAsset.fromJson(a as Map<String, dynamic>))
            .toList() ??
        [];
    return CloudRelease(
      tagName: json['tag_name'] ?? '',
      name: json['name'] ?? '',
      body: json['body'] ?? '',
      publishedAt: json['published_at'] ?? '',
      htmlUrl: json['html_url'] ?? '',
      prerelease: json['prerelease'] ?? false,
      draft: json['draft'] ?? false,
      assets: assetList,
    );
  }
}

/// 云端 OTA 服务 - 通过 GitHub Releases API 获取最新版本
class CloudOtaService {
  // SharedPreferences keys
  static const _keyRepoOwner = 'cloud_ota_repo_owner';
  static const _keyRepoName = 'cloud_ota_repo_name';
  static const _keyCustomUrl = 'cloud_ota_custom_url';
  static const _keyUseCustomUrl = 'cloud_ota_use_custom';

  // 默认 GitHub 仓库
  static const defaultOwner = 'Kitjesen';
  static const defaultRepo = '3d_NAV';

  String _owner = defaultOwner;
  String _repo = defaultRepo;
  String _customUrl = '';
  bool _useCustomUrl = false;

  String get owner => _owner;
  String get repo => _repo;
  String get customUrl => _customUrl;
  bool get useCustomUrl => _useCustomUrl;
  String get repoDisplay => _useCustomUrl ? _customUrl : '$_owner/$_repo';

  CloudOtaService();

  /// 从 SharedPreferences 加载配置
  Future<void> loadConfig() async {
    final prefs = await SharedPreferences.getInstance();
    _owner = prefs.getString(_keyRepoOwner) ?? defaultOwner;
    _repo = prefs.getString(_keyRepoName) ?? defaultRepo;
    _customUrl = prefs.getString(_keyCustomUrl) ?? '';
    _useCustomUrl = prefs.getBool(_keyUseCustomUrl) ?? false;
  }

  /// 保存配置
  Future<void> saveConfig({
    String? owner,
    String? repo,
    String? customUrl,
    bool? useCustomUrl,
  }) async {
    if (owner != null) _owner = owner;
    if (repo != null) _repo = repo;
    if (customUrl != null) _customUrl = customUrl;
    if (useCustomUrl != null) _useCustomUrl = useCustomUrl;

    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_keyRepoOwner, _owner);
    await prefs.setString(_keyRepoName, _repo);
    await prefs.setString(_keyCustomUrl, _customUrl);
    await prefs.setBool(_keyUseCustomUrl, _useCustomUrl);
  }

  /// 获取 API base URL
  String get _apiBaseUrl {
    if (_useCustomUrl && _customUrl.isNotEmpty) {
      // 自定义 URL 直接作为 releases API endpoint
      return _customUrl;
    }
    return 'https://api.github.com/repos/$_owner/$_repo/releases';
  }

  Map<String, String> get _headers => {
        'Accept': 'application/vnd.github+json',
        'X-GitHub-Api-Version': '2022-11-28',
      };

  /// 获取最新 Release
  Future<CloudRelease?> fetchLatestRelease() async {
    try {
      final url = '$_apiBaseUrl/latest';
      final response = await http.get(Uri.parse(url), headers: _headers)
          .timeout(const Duration(seconds: 15));

      if (response.statusCode == 200) {
        final json = jsonDecode(response.body) as Map<String, dynamic>;
        return CloudRelease.fromJson(json);
      } else if (response.statusCode == 404) {
        return null; // 没有 Release
      } else {
        throw Exception('GitHub API error: ${response.statusCode}');
      }
    } catch (e) {
      rethrow;
    }
  }

  /// 获取所有 Releases（最多 count 条）
  Future<List<CloudRelease>> fetchReleases({int count = 10}) async {
    try {
      final url = '$_apiBaseUrl?per_page=$count';
      final response = await http.get(Uri.parse(url), headers: _headers)
          .timeout(const Duration(seconds: 15));

      if (response.statusCode == 200) {
        final list = jsonDecode(response.body) as List<dynamic>;
        return list
            .map((j) => CloudRelease.fromJson(j as Map<String, dynamic>))
            .toList();
      } else {
        throw Exception('GitHub API error: ${response.statusCode}');
      }
    } catch (e) {
      rethrow;
    }
  }

  /// 下载资产文件到内存（返回字节）
  /// [onProgress] 下载进度回调 (0.0 ~ 1.0)
  Future<List<int>> downloadAsset(
    CloudAsset asset, {
    void Function(double progress)? onProgress,
  }) async {
    final request = http.Request('GET', Uri.parse(asset.downloadUrl));
    final streamResponse = await http.Client().send(request);

    if (streamResponse.statusCode != 200) {
      throw Exception('Download failed: HTTP ${streamResponse.statusCode}');
    }

    final totalBytes = streamResponse.contentLength ?? asset.size;
    final bytes = <int>[];
    int received = 0;

    await for (final chunk in streamResponse.stream) {
      bytes.addAll(chunk);
      received += chunk.length;
      if (totalBytes > 0) {
        onProgress?.call(received / totalBytes);
      }
    }

    return bytes;
  }
}
