import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/features/settings/cloud_ota_service.dart';

class FirmwareOtaPage extends StatefulWidget {
  const FirmwareOtaPage({super.key});

  @override
  State<FirmwareOtaPage> createState() => _FirmwareOtaPageState();
}

class _FirmwareOtaPageState extends State<FirmwareOtaPage> {
  // ---- 本地文件上传 ----
  bool _isUploading = false;
  double _uploadProgress = 0.0;
  String? _statusMessage;
  String? _selectedFileName;
  String? _lastUploadedRemotePath;

  // ---- 云端更新检查 ----
  final CloudOtaService _cloudService = CloudOtaService();
  bool _isCheckingCloud = false;
  CloudRelease? _latestRelease;
  String? _cloudError;
  bool _isDownloadingAsset = false;
  double _downloadProgress = 0.0;
  String? _downloadingAssetName;

  // ---- 应用固件 ----
  bool _isApplying = false;

  @override
  void initState() {
    super.initState();
    _cloudService.loadConfig();
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final isConnected = provider.isConnected;
    final slowState = provider.latestSlowState;
    final currentVersion = slowState?.currentMode ?? '未知';
    final battery = slowState?.resources.batteryPercent ?? 0.0;

    return Scaffold(
      appBar: AppBar(
        title: const Text('固件升级'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== 当前固件信息卡片 =====
          _buildCurrentFirmwareCard(
              isDark, isConnected, currentVersion, battery),
          const SizedBox(height: 24),

          // ===== 升级前检查 =====
          if (!isConnected) _buildWarningBanner('请先连接机器人', AppColors.warning),
          if (isConnected && battery < 30)
            _buildWarningBanner('电量低于 30%，建议充电后再升级', AppColors.error),
          if (_statusMessage != null)
            Padding(
              padding: const EdgeInsets.only(bottom: 16),
              child: _buildWarningBanner(
                  _statusMessage!,
                  _statusMessage!.contains('成功')
                      ? AppColors.success
                      : _statusMessage!.contains('失败')
                          ? AppColors.error
                          : AppColors.primary),
            ),

          // ===== 上传进度 =====
          if (_isUploading) ...[
            _buildProgressCard(isDark, '正在上传固件...', _uploadProgress,
                subtitle: _selectedFileName),
            const SizedBox(height: 24),
          ],

          // ===== 本地文件上传按钮 =====
          SizedBox(
            width: double.infinity,
            height: 52,
            child: ElevatedButton.icon(
              onPressed: isConnected && !_isUploading && battery >= 30
                  ? () => _selectAndUploadFirmware(context)
                  : null,
              icon: const Icon(Icons.upload_file),
              label: const Text('选择固件文件并上传'),
              style: _primaryButtonStyle(isDark),
            ),
          ),

          // ===== 应用固件按钮 (上传成功后出现) =====
          if (_lastUploadedRemotePath != null && !_isUploading) ...[
            const SizedBox(height: 12),
            SizedBox(
              width: double.infinity,
              height: 52,
              child: ElevatedButton.icon(
                onPressed:
                    isConnected && !_isApplying ? () => _applyFirmware() : null,
                icon: _isApplying
                    ? const SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                            strokeWidth: 2, color: Colors.white))
                    : const Icon(Icons.system_update),
                label: Text(_isApplying ? '正在触发刷写...' : '应用固件到机器人'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: AppColors.warning,
                  foregroundColor: Colors.white,
                  disabledBackgroundColor:
                      AppColors.warning.withOpacity(isDark ? 0.15 : 0.1),
                  disabledForegroundColor: context.subtitleColor,
                  shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(16)),
                  elevation: 0,
                ),
              ),
            ),
          ],

          const SizedBox(height: 32),

          // ===== 云端更新检查 =====
          _buildCloudOtaSection(isDark, isConnected, battery),

          // ===== 说明 =====
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 20),
            child: Text(
              '升级须知：\n'
              '• 升级前确保机器人电量 >= 30%\n'
              '• 升级过程中请勿断开连接\n'
              '• 确保机器人处于静止状态\n'
              '• 支持 .bin / .fw / .zip / .deb 格式固件包\n'
              '• 上传完成后需点击"应用固件"触发刷写',
              style: TextStyle(
                fontSize: 12,
                color: context.subtitleColor,
                height: 1.6,
              ),
            ),
          ),
        ],
      ),
    );
  }

  // ================================================================
  // 子组件
  // ================================================================

  Widget _buildCurrentFirmwareCard(
      bool isDark, bool isConnected, String currentVersion, double battery) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        children: [
          Container(
            width: 64,
            height: 64,
            decoration: BoxDecoration(
              color: AppColors.primary.withOpacity(isDark ? 0.15 : 0.08),
              shape: BoxShape.circle,
            ),
            child:
                const Icon(Icons.memory, size: 32, color: AppColors.primary),
          ),
          const SizedBox(height: 16),
          Text('当前固件',
              style: TextStyle(fontSize: 13, color: context.subtitleColor)),
          const SizedBox(height: 4),
          Text(
            currentVersion,
            style: TextStyle(
              fontSize: 20,
              fontWeight: FontWeight.w700,
              color: isDark ? Colors.white : Colors.black87,
            ),
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              _buildStatusChip(
                icon: Icons.power,
                label: isConnected ? '已连接' : '未连接',
                color: isConnected ? AppColors.success : AppColors.error,
              ),
              const SizedBox(width: 10),
              _buildStatusChip(
                icon: Icons.battery_charging_full,
                label: '${battery.toStringAsFixed(0)}%',
                color: battery > 30 ? AppColors.success : AppColors.error,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildCloudOtaSection(
      bool isDark, bool isConnected, double battery) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Icon(Icons.cloud_download_outlined,
                size: 18, color: context.subtitleColor),
            const SizedBox(width: 6),
            Text(
              '云端检查更新',
              style: TextStyle(
                fontSize: 15,
                fontWeight: FontWeight.w700,
                color: isDark ? Colors.white : Colors.black87,
              ),
            ),
            const Spacer(),
            Text(
              _cloudService.repoDisplay,
              style: TextStyle(fontSize: 11, color: context.subtitleColor),
            ),
          ],
        ),
        const SizedBox(height: 12),

        // 检查按钮
        SizedBox(
          width: double.infinity,
          height: 44,
          child: OutlinedButton.icon(
            onPressed: _isCheckingCloud ? null : _checkCloudUpdate,
            icon: _isCheckingCloud
                ? const SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(strokeWidth: 2))
                : const Icon(Icons.refresh, size: 18),
            label: Text(_isCheckingCloud ? '检查中...' : '检查最新版本'),
            style: OutlinedButton.styleFrom(
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(14)),
            ),
          ),
        ),

        if (_cloudError != null) ...[
          const SizedBox(height: 10),
          _buildWarningBanner(_cloudError!, AppColors.error),
        ],

        // 最新版本信息
        if (_latestRelease != null) ...[
          const SizedBox(height: 16),
          _buildReleaseCard(isDark, isConnected, battery),
        ],
      ],
    );
  }

  Widget _buildReleaseCard(bool isDark, bool isConnected, double battery) {
    final release = _latestRelease!;
    final firmwareAssets = release.assets
        .where((a) => a.category == 'firmware')
        .toList();

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 14,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: AppColors.primary.withOpacity(0.1),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Text(
                  release.tagName,
                  style: const TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w700,
                    color: AppColors.primary,
                  ),
                ),
              ),
              if (release.prerelease)
                Container(
                  margin: const EdgeInsets.only(left: 6),
                  padding:
                      const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                  decoration: BoxDecoration(
                    color: AppColors.warning.withOpacity(0.1),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: const Text(
                    'Pre-release',
                    style: TextStyle(fontSize: 10, color: AppColors.warning),
                  ),
                ),
              const Spacer(),
              Text(
                release.publishedDate,
                style: TextStyle(fontSize: 11, color: context.subtitleColor),
              ),
            ],
          ),
          if (release.name.isNotEmpty) ...[
            const SizedBox(height: 10),
            Text(
              release.name,
              style: TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white : Colors.black87,
              ),
            ),
          ],
          if (release.body.isNotEmpty) ...[
            const SizedBox(height: 8),
            Text(
              release.body.length > 300
                  ? '${release.body.substring(0, 300)}...'
                  : release.body,
              style: TextStyle(
                fontSize: 12,
                color: context.subtitleColor,
                height: 1.4,
              ),
            ),
          ],

          // 固件资产列表
          if (firmwareAssets.isEmpty) ...[
            const SizedBox(height: 12),
            Text(
              '此版本无固件文件',
              style: TextStyle(fontSize: 12, color: context.subtitleColor),
            ),
          ] else ...[
            const SizedBox(height: 16),
            Text(
              '固件文件（${firmwareAssets.length}）',
              style: TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w600,
                color: context.subtitleColor,
              ),
            ),
            const SizedBox(height: 8),
            ...firmwareAssets.map((asset) => _buildAssetTile(
                  asset,
                  isDark,
                  canUpload: isConnected && battery >= 30,
                )),
          ],

          // 显示其他可部署资产
          if (release.deployableAssets.length > firmwareAssets.length) ...[
            const SizedBox(height: 12),
            Text(
              '其他资源',
              style: TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w600,
                color: context.subtitleColor,
              ),
            ),
            const SizedBox(height: 8),
            ...release.deployableAssets
                .where((a) => a.category != 'firmware')
                .map((asset) => _buildAssetTile(
                      asset,
                      isDark,
                      canUpload: isConnected,
                    )),
          ],
        ],
      ),
    );
  }

  Widget _buildAssetTile(CloudAsset asset, bool isDark,
      {required bool canUpload}) {
    final isThisDownloading =
        _isDownloadingAsset && _downloadingAssetName == asset.name;

    return Container(
      margin: const EdgeInsets.only(bottom: 8),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: isDark
            ? Colors.white.withOpacity(0.04)
            : Colors.black.withOpacity(0.02),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Column(
        children: [
          Row(
            children: [
              Icon(
                _categoryIcon(asset.category),
                size: 18,
                color: _categoryColor(asset.category),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      asset.name,
                      style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w600,
                        color: isDark ? Colors.white : Colors.black87,
                      ),
                      overflow: TextOverflow.ellipsis,
                    ),
                    Text(
                      '${asset.formattedSize} · ${asset.category}',
                      style: TextStyle(
                          fontSize: 11, color: context.subtitleColor),
                    ),
                  ],
                ),
              ),
              const SizedBox(width: 8),
              SizedBox(
                height: 32,
                child: ElevatedButton(
                  onPressed: canUpload && !_isDownloadingAsset
                      ? () => _downloadAndUploadAsset(asset)
                      : null,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: AppColors.primary,
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(horizontal: 12),
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8)),
                    elevation: 0,
                    textStyle: const TextStyle(fontSize: 12),
                  ),
                  child: isThisDownloading
                      ? const SizedBox(
                          width: 14,
                          height: 14,
                          child: CircularProgressIndicator(
                              strokeWidth: 2, color: Colors.white))
                      : const Text('部署'),
                ),
              ),
            ],
          ),
          if (isThisDownloading) ...[
            const SizedBox(height: 8),
            ClipRRect(
              borderRadius: BorderRadius.circular(4),
              child: LinearProgressIndicator(
                value: _downloadProgress,
                minHeight: 4,
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildProgressCard(bool isDark, String title, double progress,
      {String? subtitle}) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(22),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 16,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        children: [
          Text(title,
              style: TextStyle(
                fontSize: 15,
                fontWeight: FontWeight.w600,
                color: isDark ? Colors.white : Colors.black87,
              )),
          if (subtitle != null) ...[
            const SizedBox(height: 4),
            Text(subtitle,
                style: TextStyle(fontSize: 12, color: context.subtitleColor)),
          ],
          const SizedBox(height: 16),
          ClipRRect(
            borderRadius: BorderRadius.circular(8),
            child: LinearProgressIndicator(
              value: progress,
              minHeight: 8,
              backgroundColor: isDark
                  ? Colors.white.withOpacity(0.06)
                  : Colors.black.withOpacity(0.05),
            ),
          ),
          const SizedBox(height: 8),
          Text(
            '${(progress * 100).toStringAsFixed(0)}%',
            style: const TextStyle(
                fontSize: 14, fontWeight: FontWeight.w600, color: AppColors.primary),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusChip(
      {required IconData icon, required String label, required Color color}) {
    final isDark = context.isDark;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: color.withOpacity(isDark ? 0.12 : 0.08),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 14, color: color),
          const SizedBox(width: 5),
          Text(label,
              style: TextStyle(
                  fontSize: 12, fontWeight: FontWeight.w600, color: color)),
        ],
      ),
    );
  }

  Widget _buildWarningBanner(String text, Color color) {
    return Container(
      margin: const EdgeInsets.only(bottom: 16),
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(14),
      ),
      child: Row(
        children: [
          Icon(Icons.info_outline, size: 18, color: color),
          const SizedBox(width: 10),
          Expanded(
            child: Text(text,
                style: TextStyle(
                    fontSize: 13, fontWeight: FontWeight.w600, color: color)),
          ),
        ],
      ),
    );
  }

  ButtonStyle _primaryButtonStyle(bool isDark) {
    return ElevatedButton.styleFrom(
      backgroundColor: AppColors.primary,
      foregroundColor: Colors.white,
      disabledBackgroundColor:
          AppColors.primary.withOpacity(isDark ? 0.15 : 0.1),
      disabledForegroundColor: context.subtitleColor,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      elevation: 0,
    );
  }

  IconData _categoryIcon(String category) {
    switch (category) {
      case 'firmware':
        return Icons.memory;
      case 'model':
        return Icons.psychology;
      case 'map':
        return Icons.map;
      case 'config':
        return Icons.settings;
      default:
        return Icons.insert_drive_file;
    }
  }

  Color _categoryColor(String category) {
    switch (category) {
      case 'firmware':
        return AppColors.warning;
      case 'model':
        return AppColors.secondary;
      case 'map':
        return AppColors.success;
      case 'config':
        return AppColors.info;
      default:
        return AppColors.primary;
    }
  }

  // ================================================================
  // 业务逻辑
  // ================================================================

  /// 从本地选择文件并上传到机器人
  Future<void> _selectAndUploadFirmware(BuildContext context) async {
    HapticFeedback.mediumImpact();

    final result = await FilePicker.platform.pickFiles(
      type: FileType.any,
      withData: true,
    );

    if (result == null || result.files.isEmpty) return;

    final file = result.files.first;
    if (file.bytes == null && file.path == null) {
      _showError('无法读取文件');
      return;
    }

    final bytes = file.bytes ?? await File(file.path!).readAsBytes();
    final filename = file.name;

    setState(() {
      _isUploading = true;
      _uploadProgress = 0.0;
      _statusMessage = null;
      _selectedFileName = filename;
      _lastUploadedRemotePath = null;
    });

    try {
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final remotePath = '/firmware/$filename';
      final response = await client.uploadFile(
        localBytes: bytes,
        remotePath: remotePath,
        filename: filename,
        category: 'firmware',
        overwrite: true,
        onProgress: (progress) {
          if (mounted) setState(() => _uploadProgress = progress);
        },
      );

      setState(() {
        _isUploading = false;
        if (response.success) {
          _statusMessage = '固件上传成功！可点击下方按钮应用到机器人。';
          _lastUploadedRemotePath = remotePath;
        } else {
          _statusMessage = '上传失败: ${response.message}';
        }
      });
    } catch (e) {
      setState(() {
        _isUploading = false;
        _statusMessage = '升级失败: $e';
      });
    }
  }

  /// 触发机器人端固件应用（刷写脚本）
  Future<void> _applyFirmware() async {
    if (_lastUploadedRemotePath == null) return;
    HapticFeedback.mediumImpact();

    setState(() => _isApplying = true);

    try {
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final response =
          await client.applyFirmware(firmwarePath: _lastUploadedRemotePath!);

      setState(() {
        _isApplying = false;
        _statusMessage = response.success
            ? '固件应用指令已发送，机器人可能即将重启。'
            : '应用失败: ${response.message}';
        if (response.success) _lastUploadedRemotePath = null;
      });
    } catch (e) {
      setState(() {
        _isApplying = false;
        _statusMessage = '应用固件失败: $e';
      });
    }
  }

  /// 从 GitHub 检查最新版本
  Future<void> _checkCloudUpdate() async {
    setState(() {
      _isCheckingCloud = true;
      _cloudError = null;
      _latestRelease = null;
    });

    try {
      final release = await _cloudService.fetchLatestRelease();
      setState(() {
        _isCheckingCloud = false;
        _latestRelease = release;
        if (release == null) {
          _cloudError = '未找到任何 Release';
        }
      });
    } catch (e) {
      setState(() {
        _isCheckingCloud = false;
        _cloudError = '检查更新失败: $e';
      });
    }
  }

  /// 从云端下载资产并自动上传到机器人
  Future<void> _downloadAndUploadAsset(CloudAsset asset) async {
    HapticFeedback.mediumImpact();

    setState(() {
      _isDownloadingAsset = true;
      _downloadProgress = 0.0;
      _downloadingAssetName = asset.name;
      _statusMessage = null;
      _lastUploadedRemotePath = null;
    });

    try {
      // 1. 下载到内存
      final bytes = await _cloudService.downloadAsset(
        asset,
        onProgress: (p) {
          if (mounted) setState(() => _downloadProgress = p * 0.5); // 0~50%
        },
      );

      // 2. 上传到机器人
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final remotePath = '/${asset.category}/${asset.name}';
      final response = await client.uploadFile(
        localBytes: bytes,
        remotePath: remotePath,
        filename: asset.name,
        category: asset.category,
        overwrite: true,
        onProgress: (p) {
          if (mounted) {
            setState(() => _downloadProgress = 0.5 + p * 0.5); // 50~100%
          }
        },
      );

      setState(() {
        _isDownloadingAsset = false;
        _downloadingAssetName = null;
        if (response.success) {
          _statusMessage = '${asset.name} 已成功部署到机器人';
          if (asset.category == 'firmware') {
            _lastUploadedRemotePath = remotePath;
          }
        } else {
          _statusMessage = '部署失败: ${response.message}';
        }
      });
    } catch (e) {
      setState(() {
        _isDownloadingAsset = false;
        _downloadingAssetName = null;
        _statusMessage = '部署失败: $e';
      });
    }
  }

  void _showError(String msg) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(msg),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      ),
    );
  }
}
