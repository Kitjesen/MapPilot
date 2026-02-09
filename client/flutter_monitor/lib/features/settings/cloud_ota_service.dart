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

  /// manifest 中指定的分类 (优先)
  String? _manifestCategory;

  /// manifest 中的 OTA 元数据 (如果有 manifest.json)
  CloudOtaArtifactMeta? otaMeta;

  /// 从文件扩展名推断的分类
  String get category {
    if (_manifestCategory != null) return _manifestCategory!;
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

  set manifestCategory(String? cat) => _manifestCategory = cat;

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
    this.otaMeta,
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

/// 云端 manifest.json 中每个制品的元数据
class CloudOtaArtifactMeta {
  final String name;
  final String category;    // "model", "firmware", "map", "config"
  final String version;
  final String filename;
  final String sha256;
  final String targetPath;
  final String targetBoard; // "nav" or "dog"
  final List<String> hwCompat;
  final String applyAction; // "copy_only", "reload_model", "install_deb", "flash_mcu", "install_script"
  final bool requiresReboot;
  final int minBatteryPercent;
  final String changelog;
  final bool rollbackSafe;

  CloudOtaArtifactMeta({
    required this.name,
    required this.category,
    required this.version,
    required this.filename,
    required this.sha256,
    required this.targetPath,
    this.targetBoard = 'nav',
    this.hwCompat = const ['*'],
    this.applyAction = 'copy_only',
    this.requiresReboot = false,
    this.minBatteryPercent = 0,
    this.changelog = '',
    this.rollbackSafe = true,
  });

  factory CloudOtaArtifactMeta.fromJson(Map<String, dynamic> json) {
    return CloudOtaArtifactMeta(
      name: json['name'] ?? '',
      category: json['category'] ?? 'other',
      version: json['version'] ?? '',
      filename: json['filename'] ?? '',
      sha256: json['sha256'] ?? '',
      targetPath: json['target_path'] ?? '',
      targetBoard: json['target_board'] ?? 'nav',
      hwCompat: (json['hw_compat'] as List<dynamic>?)
              ?.map((e) => e.toString())
              .toList() ??
          const ['*'],
      applyAction: json['apply_action'] ?? 'copy_only',
      requiresReboot: json['requires_reboot'] ?? false,
      minBatteryPercent: json['min_battery_percent'] ?? 0,
      changelog: json['changelog'] ?? '',
      rollbackSafe: json['rollback_safe'] ?? true,
    );
  }
}

/// 云端 manifest.json 完整结构
class CloudOtaManifest {
  final String schemaVersion;
  final String releaseVersion;
  final String releaseDate;
  final String channel; // "dev", "canary", "stable"
  final String minSystemVersion;
  final List<CloudOtaArtifactMeta> artifacts;

  CloudOtaManifest({
    required this.schemaVersion,
    required this.releaseVersion,
    required this.releaseDate,
    this.channel = 'stable',
    this.minSystemVersion = '',
    required this.artifacts,
  });

  factory CloudOtaManifest.fromJson(Map<String, dynamic> json) {
    final artifactList = (json['artifacts'] as List<dynamic>?)
            ?.map((a) => CloudOtaArtifactMeta.fromJson(a as Map<String, dynamic>))
            .toList() ??
        [];
    return CloudOtaManifest(
      schemaVersion: json['schema_version'] ?? '1',
      releaseVersion: json['release_version'] ?? '',
      releaseDate: json['release_date'] ?? '',
      channel: json['channel'] ?? 'stable',
      minSystemVersion: json['min_system_version'] ?? '',
      artifacts: artifactList,
    );
  }

  /// 按 category 过滤
  List<CloudOtaArtifactMeta> byCategory(String category) =>
      artifacts.where((a) => a.category == category).toList();
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

  /// 解析后的 OTA manifest（如果 Release 中包含 manifest.json）
  CloudOtaManifest? manifest;

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

  /// 是否包含 OTA manifest
  bool get hasManifest => manifest != null;

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
    this.manifest,
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
  static const _keyChannel = 'cloud_ota_channel';

  // 默认 GitHub 仓库
  static const defaultOwner = 'Kitjesen';
  static const defaultRepo = 'MapPilot';

  String _owner = defaultOwner;
  String _repo = defaultRepo;
  String _customUrl = '';
  bool _useCustomUrl = false;
  String _channel = 'stable'; // "dev", "canary", "stable"

  String get owner => _owner;
  String get repo => _repo;
  String get customUrl => _customUrl;
  bool get useCustomUrl => _useCustomUrl;
  String get channel => _channel;
  String get repoDisplay => _useCustomUrl ? _customUrl : '$_owner/$_repo';

  CloudOtaService();

  /// 从 SharedPreferences 加载配置
  Future<void> loadConfig() async {
    final prefs = await SharedPreferences.getInstance();
    _owner = prefs.getString(_keyRepoOwner) ?? defaultOwner;
    _repo = prefs.getString(_keyRepoName) ?? defaultRepo;
    _customUrl = prefs.getString(_keyCustomUrl) ?? '';
    _useCustomUrl = prefs.getBool(_keyUseCustomUrl) ?? false;
    _channel = prefs.getString(_keyChannel) ?? 'stable';
  }

  /// 保存配置
  Future<void> saveConfig({
    String? owner,
    String? repo,
    String? customUrl,
    bool? useCustomUrl,
    String? channel,
  }) async {
    if (owner != null) _owner = owner;
    if (repo != null) _repo = repo;
    if (customUrl != null) _customUrl = customUrl;
    if (useCustomUrl != null) _useCustomUrl = useCustomUrl;
    if (channel != null) _channel = channel;

    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_keyRepoOwner, _owner);
    await prefs.setString(_keyRepoName, _repo);
    await prefs.setString(_keyCustomUrl, _customUrl);
    await prefs.setBool(_keyUseCustomUrl, _useCustomUrl);
    await prefs.setString(_keyChannel, _channel);
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

  /// 获取最新 Release（并自动拉取 manifest.json）
  /// 根据 channel 过滤:
  ///   - "stable": 仅非 prerelease
  ///   - "canary"/"dev": 包含 prerelease，再按 manifest channel 匹配
  Future<CloudRelease?> fetchLatestRelease() async {
    try {
      if (_channel == 'stable') {
        // /latest 只返回非 prerelease 的最新版
        final url = '$_apiBaseUrl/latest';
        final response = await http.get(Uri.parse(url), headers: _headers)
            .timeout(const Duration(seconds: 15));

        if (response.statusCode == 200) {
          final json = jsonDecode(response.body) as Map<String, dynamic>;
          final release = CloudRelease.fromJson(json);
          await _enrichReleaseWithManifest(release);
          return release;
        } else if (response.statusCode == 404) {
          return null;
        } else {
          throw Exception('GitHub API error: ${response.statusCode}');
        }
      } else {
        // dev/canary: 获取所有 releases，按 channel 过滤
        final releases = await fetchReleases(count: 20);
        for (final release in releases) {
          // If manifest has channel, match exactly
          if (release.manifest != null &&
              release.manifest!.channel == _channel) {
            return release;
          }
          // Fallback: prerelease = dev/canary, non-prerelease = stable
          if (_channel == 'dev' && release.prerelease) return release;
          if (_channel == 'canary' && release.prerelease) return release;
        }
        // No match for channel — fall back to latest regardless
        return releases.isNotEmpty ? releases.first : null;
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
        final releases = list
            .map((j) => CloudRelease.fromJson(j as Map<String, dynamic>))
            .toList();
        // 尝试为每个 release 拉取 manifest
        for (final release in releases) {
          await _enrichReleaseWithManifest(release);
        }
        return releases;
      } else {
        throw Exception('GitHub API error: ${response.statusCode}');
      }
    } catch (e) {
      rethrow;
    }
  }

  /// 如果 Release 中包含 manifest.json，下载并解析
  /// 同时将 manifest 中的元数据关联到对应的 CloudAsset
  Future<void> _enrichReleaseWithManifest(CloudRelease release) async {
    final manifestAsset = release.assets
        .where((a) => a.name == 'manifest.json')
        .toList();

    if (manifestAsset.isEmpty) return;

    try {
      final bytes = await downloadAsset(manifestAsset.first);
      final jsonStr = utf8.decode(bytes);
      final json = jsonDecode(jsonStr) as Map<String, dynamic>;
      final manifest = CloudOtaManifest.fromJson(json);
      release.manifest = manifest;

      // 将 manifest 元数据关联到对应的 asset
      for (final artifactMeta in manifest.artifacts) {
        final matchingAssets = release.assets
            .where((a) => a.name == artifactMeta.filename)
            .toList();
        for (final asset in matchingAssets) {
          asset.otaMeta = artifactMeta;
          asset.manifestCategory = artifactMeta.category;
        }
      }
    } catch (e) {
      // manifest 解析失败不阻塞，fallback 到文件扩展名推断
      // ignore
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
