import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/src/data.pb.dart';

/// Gateway for remote file management on the robot.
///
/// Wraps [RobotClientBase] file operations: list, upload, download, delete.
/// Runs on the OTA daemon port (50052).
class FileGateway extends ChangeNotifier {
  RobotClientBase? _client;

  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      _files = [];
      _currentDirectory = defaultDirectory;
      _totalSize = 0;
      _freeSpace = 0;
      _isLoading = false;
      _errorMessage = null;
      notifyListeners();
    }
  }

  // ── State ──

  /// 默认初始目录为地图目录（OTA allowed_directories 白名单内）
  static const String defaultDirectory = '/home/sunrise/data/SLAM/navigation/maps';

  List<RemoteFileInfo> _files = [];
  String _currentDirectory = defaultDirectory;
  int _totalSize = 0;
  int _freeSpace = 0;
  bool _isLoading = false;
  bool _isUploading = false;
  double _uploadProgress = 0.0;
  String? _errorMessage;

  List<RemoteFileInfo> get files => _files;
  String get currentDirectory => _currentDirectory;
  int get totalSize => _totalSize;
  int get freeSpace => _freeSpace;
  bool get isLoading => _isLoading;
  bool get isUploading => _isUploading;
  double get uploadProgress => _uploadProgress;
  String? get errorMessage => _errorMessage;

  /// Breadcrumb segments for the current path.
  List<String> get breadcrumbs {
    final parts = _currentDirectory.split('/').where((p) => p.isNotEmpty).toList();
    return ['/', ...parts];
  }

  // ── List Files ──

  Future<void> listFiles([String? directory]) async {
    final client = _client;
    if (client == null) return;

    final dir = directory ?? _currentDirectory;

    _isLoading = true;
    _errorMessage = null;
    notifyListeners();

    try {
      final response = await client.listRemoteFiles(directory: dir);
      _files = response.files.toList();
      _currentDirectory = dir;
      _totalSize = response.totalSize.toInt();
      _freeSpace = response.freeSpace.toInt();
      _isLoading = false;
      notifyListeners();
    } catch (e) {
      AppLogger.system.error('Failed to list files: $e');
      _isLoading = false;
      _errorMessage = '无法列出文件: $e';
      notifyListeners();
    }
  }

  /// Navigate into a subdirectory.
  Future<void> openDirectory(String name) async {
    final newPath = _currentDirectory.endsWith('/')
        ? '$_currentDirectory$name'
        : '$_currentDirectory/$name';
    await listFiles(newPath);
  }

  /// Navigate up one level.
  Future<void> goUp() async {
    if (_currentDirectory == '/' || _currentDirectory.isEmpty) return;
    final parts = _currentDirectory.split('/').where((p) => p.isNotEmpty).toList();
    if (parts.isEmpty) return;
    parts.removeLast();
    final parentPath = parts.isEmpty ? '/' : '/${parts.join('/')}';
    await listFiles(parentPath);
  }

  /// Navigate to a specific breadcrumb index.
  Future<void> navigateToBreadcrumb(int index) async {
    if (index == 0) {
      await listFiles('/');
      return;
    }
    final parts = breadcrumbs.skip(1).take(index).toList();
    final path = '/${parts.join('/')}';
    await listFiles(path);
  }

  // ── Upload File ──

  /// Infer OTA category from filename extension or current directory.
  static String inferCategory(String filename, String currentDir) {
    final lower = filename.toLowerCase();
    // By extension
    if (lower.endsWith('.onnx') || lower.endsWith('.pt') ||
        lower.endsWith('.tflite') || lower.endsWith('.engine')) return 'model';
    if (lower.endsWith('.pcd') || lower.endsWith('.pgm') ||
        lower.endsWith('.yaml') && lower.contains('map')) return 'map';
    if (lower.endsWith('.bin') || lower.endsWith('.hex') ||
        lower.endsWith('.deb') || lower.endsWith('.img')) return 'firmware';
    if (lower.endsWith('.json') || lower.endsWith('.yaml') ||
        lower.endsWith('.toml') || lower.endsWith('.cfg')) return 'config';
    // By current directory name
    final dirLower = currentDir.toLowerCase();
    if (dirLower.contains('model')) return 'model';
    if (dirLower.contains('map')) return 'map';
    if (dirLower.contains('firmware')) return 'firmware';
    if (dirLower.contains('config')) return 'config';
    return 'model'; // fallback
  }

  Future<bool> uploadFile({
    required List<int> bytes,
    required String filename,
    String? category,
  }) async {
    final client = _client;
    if (client == null) return false;

    _isUploading = true;
    _uploadProgress = 0.0;
    _errorMessage = null;
    notifyListeners();

    try {
      final remotePath = _currentDirectory.endsWith('/')
          ? '$_currentDirectory$filename'
          : '$_currentDirectory/$filename';

      final effectiveCategory = category ?? inferCategory(filename, _currentDirectory);

      await client.uploadFile(
        localBytes: bytes,
        remotePath: remotePath,
        filename: filename,
        category: effectiveCategory,
        onProgress: (p) {
          _uploadProgress = p;
          notifyListeners();
        },
      );

      _isUploading = false;
      _uploadProgress = 1.0;
      notifyListeners();

      // Refresh file list
      await listFiles();
      return true;
    } catch (e) {
      AppLogger.system.error('Failed to upload file: $e');
      _isUploading = false;
      _errorMessage = '上传失败: $e';
      notifyListeners();
      return false;
    }
  }

  // ── Delete File ──

  Future<bool> deleteFile(String remotePath) async {
    final client = _client;
    if (client == null) return false;

    try {
      await client.deleteRemoteFile(remotePath: remotePath);
      // Remove from local list immediately for snappy UX
      _files.removeWhere((f) => f.path == remotePath);
      notifyListeners();
      return true;
    } catch (e) {
      AppLogger.system.error('Failed to delete file: $e');
      _errorMessage = '删除失败: $e';
      notifyListeners();
      return false;
    }
  }

  // ── Download File ──

  bool _isDownloading = false;
  double _downloadProgress = 0.0;
  bool get isDownloading => _isDownloading;
  double get downloadProgress => _downloadProgress;

  /// Download a file from the robot. Returns the collected bytes, or null on error.
  Future<List<int>?> downloadFile(String remotePath) async {
    final client = _client;
    if (client == null) return null;

    _isDownloading = true;
    _downloadProgress = 0.0;
    _errorMessage = null;
    notifyListeners();

    try {
      final chunks = <int>[];
      int totalSize = 0;

      await for (final chunk in client.downloadFile(filePath: remotePath)) {
        chunks.addAll(chunk.data);
        if (chunk.totalSize.toInt() > 0) totalSize = chunk.totalSize.toInt();
        if (totalSize > 0) {
          _downloadProgress = chunks.length / totalSize;
          notifyListeners();
        }
        if (chunk.isLast) break;
      }

      _isDownloading = false;
      _downloadProgress = 1.0;
      notifyListeners();
      return chunks;
    } catch (e) {
      AppLogger.system.error('Failed to download file: $e');
      _isDownloading = false;
      _errorMessage = '下载失败: $e';
      notifyListeners();
      return null;
    }
  }

  // ── Helpers ──

  void clearError() {
    _errorMessage = null;
    notifyListeners();
  }
}
