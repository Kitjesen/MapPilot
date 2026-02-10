import 'dart:io' show File;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:file_picker/file_picker.dart';
import 'package:path_provider/path_provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/gateway/file_gateway.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/shared/widgets/status_placeholder.dart';
import 'package:robot_proto/src/data.pb.dart';

/// Remote file browser for the robot's filesystem.
///
/// When [embedded] is true the screen is used as a tab inside MainShell
/// — no back button, and a disconnected / OTA-unavailable placeholder
/// is shown instead of the file list.
class FileBrowserScreen extends StatefulWidget {
  final bool embedded;
  const FileBrowserScreen({super.key, this.embedded = false});

  @override
  State<FileBrowserScreen> createState() => _FileBrowserScreenState();
}

class _FileBrowserScreenState extends State<FileBrowserScreen> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final connProvider = context.read<RobotConnectionProvider>();
      if (connProvider.isConnected && connProvider.otaAvailable) {
        context.read<FileGateway>().listFiles('/');
      }
    });
  }

  Future<void> _pickAndUpload() async {
    final result = await FilePicker.platform.pickFiles();
    if (result == null || result.files.isEmpty) return;
    final file = result.files.first;
    if (file.bytes == null && file.path == null) return;

    final bytes = file.bytes ?? await _readFileBytes(file.path!);
    if (bytes == null || !mounted) return;

    final gw = context.read<FileGateway>();
    final success = await gw.uploadFile(
      bytes: bytes,
      filename: file.name,
    );

    if (mounted && success) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('已上传: ${file.name}'),
          behavior: SnackBarBehavior.floating,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        ),
      );
    }
  }

  Future<List<int>?> _readFileBytes(String path) async {
    try {
      return await File(path).readAsBytes();
    } catch (_) {
      return null;
    }
  }

  Future<void> _downloadFile(RemoteFileInfo file) async {
    final gw = context.read<FileGateway>();
    final bytes = await gw.downloadFile(file.path);
    if (bytes == null || !mounted) return;

    try {
      final dir = await getApplicationDocumentsDirectory();
      final localFile = File('${dir.path}/${file.filename}');
      await localFile.writeAsBytes(bytes);

      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('已下载: ${file.filename}'),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
            action: SnackBarAction(
              label: '路径',
              onPressed: () {
                Clipboard.setData(ClipboardData(text: localFile.path));
              },
            ),
          ),
        );
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('保存失败: $e'),
            behavior: SnackBarBehavior.floating,
          ),
        );
      }
    }
  }

  Future<void> _confirmDelete(RemoteFileInfo file) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('删除文件'),
        content: Text('确定要删除 "${file.filename}" 吗？\n此操作不可撤销。'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: const Text('取消'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: const Text('删除'),
          ),
        ],
      ),
    );

    if (confirmed == true && mounted) {
      final success = await context.read<FileGateway>().deleteFile(file.path);
      if (mounted && success) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('已删除: ${file.filename}'),
            behavior: SnackBarBehavior.floating,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
          ),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final connProvider = context.watch<RobotConnectionProvider>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('文件管理'),
        leading: widget.embedded
            ? null
            : IconButton(
                icon: const Icon(Icons.arrow_back_ios_new, size: 20),
                onPressed: () => Navigator.pop(context),
              ),
        automaticallyImplyLeading: !widget.embedded,
        actions: widget.embedded && connProvider.isConnected
            ? [
                IconButton(
                  icon: Icon(Icons.refresh_rounded, size: 20, color: context.subtitleColor),
                  onPressed: () => context.read<FileGateway>().listFiles(),
                ),
              ]
            : null,
      ),
      body: !connProvider.isConnected
          ? _buildDisconnectedPlaceholder(context)
          : !connProvider.otaAvailable
              ? _buildOtaUnavailablePlaceholder(context)
              : Consumer<FileGateway>(
        builder: (context, gw, _) {
          return Column(
            children: [
              // ── Breadcrumb bar ──
              _BreadcrumbBar(
                breadcrumbs: gw.breadcrumbs,
                onTap: (index) => gw.navigateToBreadcrumb(index),
              ),

              // ── Storage info ──
              if (gw.totalSize > 0)
                _StorageInfoBar(
                  totalSize: gw.totalSize,
                  freeSpace: gw.freeSpace,
                  isDark: isDark,
                ),

              // ── Upload progress ──
              if (gw.isUploading)
                Padding(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text('上传中...',
                          style: TextStyle(
                              fontSize: 12, color: context.subtitleColor)),
                      const SizedBox(height: 4),
                      ClipRRect(
                        borderRadius: BorderRadius.circular(3),
                        child: LinearProgressIndicator(
                          value: gw.uploadProgress,
                          minHeight: 3,
                          color: AppColors.primary,
                          backgroundColor: isDark
                              ? Colors.white.withValues(alpha: 0.06)
                              : Colors.black.withValues(alpha: 0.04),
                        ),
                      ),
                    ],
                  ),
                ),

              // ── Download progress ──
              if (gw.isDownloading)
                Padding(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text('下载中...',
                          style: TextStyle(
                              fontSize: 12, color: context.subtitleColor)),
                      const SizedBox(height: 4),
                      ClipRRect(
                        borderRadius: BorderRadius.circular(3),
                        child: LinearProgressIndicator(
                          value: gw.downloadProgress,
                          minHeight: 3,
                          color: AppColors.success,
                          backgroundColor: isDark
                              ? Colors.white.withValues(alpha: 0.06)
                              : Colors.black.withValues(alpha: 0.04),
                        ),
                      ),
                    ],
                  ),
                ),

              // ── Error banner ──
              if (gw.errorMessage != null)
                Container(
                  margin:
                      const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                  padding: const EdgeInsets.all(10),
                  decoration: BoxDecoration(
                    color: AppColors.error.withValues(alpha: isDark ? 0.1 : 0.05),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Row(
                    children: [
                      const Icon(Icons.error_outline,
                          size: 16, color: AppColors.error),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(gw.errorMessage!,
                            style: const TextStyle(
                                fontSize: 12, color: AppColors.error)),
                      ),
                      IconButton(
                        icon: const Icon(Icons.close, size: 14),
                        onPressed: () => gw.clearError(),
                        padding: EdgeInsets.zero,
                        constraints: const BoxConstraints(),
                      ),
                    ],
                  ),
                ),

              // ── File list ──
              Expanded(
                child: gw.isLoading
                    ? const Center(
                        child: CircularProgressIndicator(strokeWidth: 2))
                    : gw.files.isEmpty
                        ? StatusPlaceholder.empty(
                            icon: Icons.folder_open_outlined,
                            title: '此目录为空',
                            subtitle: gw.currentDirectory,
                          )
                        : RefreshIndicator(
                            onRefresh: () => gw.listFiles(),
                            child: ListView.separated(
                              physics: const AlwaysScrollableScrollPhysics(),
                              padding: const EdgeInsets.symmetric(
                                  horizontal: 16, vertical: 8),
                              itemCount: gw.files.length,
                              separatorBuilder: (_, __) =>
                                  const SizedBox(height: 2),
                              itemBuilder: (context, index) {
                                final file = gw.files[index];
                                return _FileListTile(
                                  file: file,
                                  isDark: isDark,
                                  onTap: () {
                                    // If it looks like a directory (size == 0 and no extension)
                                    if (file.size.toInt() == 0 &&
                                        !file.filename.contains('.')) {
                                      gw.openDirectory(file.filename);
                                    }
                                  },
                                  onDelete: () => _confirmDelete(file),
                                  onDownload: () => _downloadFile(file),
                                );
                              },
                            ),
                          ),
              ),
            ],
          );
        },
      ),
      floatingActionButton: connProvider.isConnected && connProvider.otaAvailable
          ? FloatingActionButton(
              onPressed: _pickAndUpload,
              backgroundColor: AppColors.primary,
              child: const Icon(Icons.upload_file, color: Colors.white),
            )
          : null,
    );
  }

  Widget _buildDisconnectedPlaceholder(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(32),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              width: 72,
              height: 72,
              decoration: BoxDecoration(
                color: AppColors.primary.withValues(alpha: 0.08),
                borderRadius: BorderRadius.circular(20),
              ),
              child: const Icon(Icons.folder_off_outlined, size: 36, color: AppColors.primary),
            ),
            const SizedBox(height: 20),
            Text(
              '未连接机器人',
              style: TextStyle(
                fontSize: 17,
                fontWeight: FontWeight.w600,
                color: context.titleColor,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              '连接机器人后可在此管理远程文件',
              style: TextStyle(
                fontSize: 14,
                color: context.subtitleColor,
              ),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 24),
            ElevatedButton.icon(
              onPressed: () => Navigator.of(context).pushNamed('/scan'),
              icon: const Icon(Icons.wifi_find_rounded, size: 18),
              label: const Text('扫描设备'),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildOtaUnavailablePlaceholder(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(32),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              width: 72,
              height: 72,
              decoration: BoxDecoration(
                color: AppColors.warning.withValues(alpha: 0.1),
                borderRadius: BorderRadius.circular(20),
              ),
              child: const Icon(Icons.warning_amber_rounded, size: 36, color: AppColors.warning),
            ),
            const SizedBox(height: 20),
            Text(
              'OTA 服务不可用',
              style: TextStyle(
                fontSize: 17,
                fontWeight: FontWeight.w600,
                color: context.titleColor,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              '文件管理需要 OTA daemon (端口 50052)\n请确认机器人上的 ota_daemon 服务已启动',
              style: TextStyle(
                fontSize: 14,
                color: context.subtitleColor,
              ),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 24),
            OutlinedButton.icon(
              onPressed: () {
                // Try refreshing files to re-test OTA connection
                context.read<FileGateway>().listFiles('/');
              },
              icon: const Icon(Icons.refresh_rounded, size: 18),
              label: const Text('重试'),
            ),
          ],
        ),
      ),
    );
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sub-widgets
// ─────────────────────────────────────────────────────────────────────────────

class _BreadcrumbBar extends StatelessWidget {
  final List<String> breadcrumbs;
  final void Function(int index) onTap;

  const _BreadcrumbBar({required this.breadcrumbs, required this.onTap});

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 40,
      padding: const EdgeInsets.symmetric(horizontal: 16),
      decoration: BoxDecoration(
        border: Border(
          bottom: BorderSide(color: context.borderColor, width: 0.5),
        ),
      ),
      child: ListView.separated(
        scrollDirection: Axis.horizontal,
        itemCount: breadcrumbs.length,
        separatorBuilder: (_, __) => Padding(
          padding: const EdgeInsets.symmetric(horizontal: 2),
          child: Icon(Icons.chevron_right,
              size: 16, color: context.subtitleColor),
        ),
        itemBuilder: (ctx, i) {
          final isLast = i == breadcrumbs.length - 1;
          return GestureDetector(
            onTap: isLast ? null : () => onTap(i),
            child: Center(
              child: Text(
                breadcrumbs[i],
                style: TextStyle(
                  fontSize: 13,
                  fontWeight: isLast ? FontWeight.w600 : FontWeight.w400,
                  color: isLast
                      ? context.titleColor
                      : AppColors.primary,
                ),
              ),
            ),
          );
        },
      ),
    );
  }
}

class _StorageInfoBar extends StatelessWidget {
  final int totalSize;
  final int freeSpace;
  final bool isDark;

  const _StorageInfoBar({
    required this.totalSize,
    required this.freeSpace,
    required this.isDark,
  });

  String _fmt(int bytes) {
    if (bytes < 1024) return '$bytes B';
    if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
    if (bytes < 1024 * 1024 * 1024) {
      return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
    }
    return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(1)} GB';
  }

  @override
  Widget build(BuildContext context) {
    final used = totalSize - freeSpace;
    final ratio = totalSize > 0 ? used / totalSize : 0.0;

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      child: Row(
        children: [
          Icon(Icons.storage, size: 14, color: context.subtitleColor),
          const SizedBox(width: 6),
          Text(
            '${_fmt(used)} / ${_fmt(totalSize)}',
            style: TextStyle(fontSize: 11, color: context.subtitleColor),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: ClipRRect(
              borderRadius: BorderRadius.circular(2),
              child: LinearProgressIndicator(
                value: ratio.clamp(0.0, 1.0),
                minHeight: 3,
                color: ratio > 0.9
                    ? AppColors.error
                    : ratio > 0.7
                        ? AppColors.warning
                        : AppColors.primary,
                backgroundColor: isDark
                    ? Colors.white.withValues(alpha: 0.06)
                    : Colors.black.withValues(alpha: 0.04),
              ),
            ),
          ),
          const SizedBox(width: 8),
          Text(
            '${_fmt(freeSpace)} 可用',
            style: TextStyle(fontSize: 11, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }
}

class _FileListTile extends StatelessWidget {
  final RemoteFileInfo file;
  final bool isDark;
  final VoidCallback onTap;
  final VoidCallback onDelete;
  final VoidCallback onDownload;

  const _FileListTile({
    required this.file,
    required this.isDark,
    required this.onTap,
    required this.onDelete,
    required this.onDownload,
  });

  IconData _fileIcon(String filename) {
    final ext = filename.split('.').last.toLowerCase();
    switch (ext) {
      case 'pcd':
      case 'ply':
      case 'pgm':
        return Icons.map_outlined;
      case 'onnx':
      case 'pt':
      case 'pth':
        return Icons.model_training;
      case 'yaml':
      case 'yml':
      case 'json':
      case 'toml':
        return Icons.settings_outlined;
      case 'deb':
      case 'bin':
      case 'hex':
        return Icons.system_update_outlined;
      case 'log':
      case 'txt':
        return Icons.description_outlined;
      case 'png':
      case 'jpg':
      case 'jpeg':
        return Icons.image_outlined;
      default:
        // If no extension, likely a directory
        if (!filename.contains('.')) return Icons.folder_outlined;
        return Icons.insert_drive_file_outlined;
    }
  }

  Color _fileColor(String filename) {
    final ext = filename.split('.').last.toLowerCase();
    switch (ext) {
      case 'pcd':
      case 'ply':
      case 'pgm':
        return AppColors.primary;
      case 'onnx':
      case 'pt':
      case 'pth':
        return const Color(0xFFAF52DE);
      case 'yaml':
      case 'yml':
      case 'json':
        return AppColors.warning;
      case 'deb':
      case 'bin':
        return AppColors.error;
      default:
        if (!filename.contains('.')) return AppColors.primary;
        return const Color(0xFF86868B);
    }
  }

  String _formatSize(int bytes) {
    if (bytes <= 0) return '--';
    if (bytes < 1024) return '$bytes B';
    if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
    if (bytes < 1024 * 1024 * 1024) {
      return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
    }
    return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(1)} GB';
  }

  @override
  Widget build(BuildContext context) {
    final color = _fileColor(file.filename);
    final isDir = file.size.toInt() == 0 && !file.filename.contains('.');

    return InkWell(
      onTap: onTap,
      borderRadius: BorderRadius.circular(AppRadius.card),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
        child: Row(
          children: [
            // Icon
            Container(
              width: 36,
              height: 36,
              decoration: BoxDecoration(
                color: color.withValues(alpha: isDark ? 0.15 : 0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Icon(_fileIcon(file.filename), size: 18, color: color),
            ),
            const SizedBox(width: 12),
            // Name + meta
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    file.filename,
                    style: TextStyle(
                      fontSize: 14,
                      fontWeight: FontWeight.w500,
                      color: context.titleColor,
                    ),
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                  ),
                  const SizedBox(height: 2),
                  Text(
                    isDir
                        ? '目录'
                        : '${_formatSize(file.size.toInt())}${file.modifiedTime.isNotEmpty ? '  •  ${file.modifiedTime}' : ''}',
                    style: TextStyle(
                      fontSize: 11,
                      color: context.subtitleColor,
                    ),
                  ),
                ],
              ),
            ),
            // Category chip
            if (file.category.isNotEmpty)
              Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                decoration: BoxDecoration(
                  color: color.withValues(alpha: 0.1),
                  borderRadius: BorderRadius.circular(6),
                ),
                child: Text(
                  file.category,
                  style: TextStyle(fontSize: 10, color: color),
                ),
              ),
            // Actions
            if (!isDir)
              PopupMenuButton<String>(
                icon: Icon(Icons.more_vert,
                    size: 18, color: context.subtitleColor),
                shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(10)),
                onSelected: (action) {
                  HapticFeedback.mediumImpact();
                  if (action == 'delete') {
                    onDelete();
                  } else if (action == 'download') {
                    onDownload();
                  }
                },
                itemBuilder: (ctx) => [
                  const PopupMenuItem(
                    value: 'download',
                    child: Row(
                      children: [
                        Icon(Icons.download_outlined,
                            size: 16, color: AppColors.primary),
                        SizedBox(width: 8),
                        Text('下载', style: TextStyle(fontSize: 13)),
                      ],
                    ),
                  ),
                  const PopupMenuItem(
                    value: 'delete',
                    child: Row(
                      children: [
                        Icon(Icons.delete_outline,
                            size: 16, color: AppColors.error),
                        SizedBox(width: 8),
                        Text('删除', style: TextStyle(fontSize: 13)),
                      ],
                    ),
                  ),
                ],
              )
            else
              Icon(Icons.chevron_right,
                  size: 18, color: context.subtitleColor),
          ],
        ),
      ),
    );
  }
}
