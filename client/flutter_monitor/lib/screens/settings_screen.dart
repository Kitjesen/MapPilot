import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:file_picker/file_picker.dart';
import '../services/robot_connection_provider.dart';
import '../services/cloud_ota_service.dart';
import 'package:robot_proto/robot_proto.dart';
import '../theme/app_theme.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen>
    with AutomaticKeepAliveClientMixin {
  @override
  bool get wantKeepAlive => true;

  // --- File Manager State ---
  String _currentCategory = 'model';
  String _remoteDirectory = '/home/sunrise/models';
  List<RemoteFileInfo> _remoteFiles = [];
  bool _isLoadingFiles = false;
  bool _isUploading = false;
  double _uploadProgress = 0.0;
  String? _uploadStatus;

  // --- Cloud OTA State ---
  final CloudOtaService _cloudOta = CloudOtaService();
  List<CloudRelease> _releases = [];
  bool _isCheckingCloud = false;
  String? _cloudError;
  String? _deployingAssetName;
  double _deployProgress = 0.0;
  String? _deployStatus;

  // --- Category config ---
  static const Map<String, _CategoryConfig> _categories = {
    'model': _CategoryConfig(
      label: '模型',
      icon: Icons.psychology,
      color: Color(0xFF8E8AFF),
      defaultDir: '/home/sunrise/models',
      extensions: ['pt', 'pth', 'onnx', 'tflite', 'engine', 'bin'],
    ),
    'map': _CategoryConfig(
      label: '地图',
      icon: Icons.map,
      color: AppColors.success,
      defaultDir: '/home/sunrise/maps',
      extensions: ['pcd', 'ply', 'pickle', 'pgm', 'yaml'],
    ),
    'config': _CategoryConfig(
      label: '配置',
      icon: Icons.settings,
      color: AppColors.warning,
      defaultDir: '/home/sunrise/config',
      extensions: ['yaml', 'yml', 'json', 'xml', 'cfg', 'ini'],
    ),
    'firmware': _CategoryConfig(
      label: '固件',
      icon: Icons.memory,
      color: AppColors.error,
      defaultDir: '/home/sunrise/firmware',
      extensions: ['bin', 'hex', 'img', 'deb'],
    ),
  };

  @override
  void initState() {
    super.initState();
    _loadRemoteFiles();
    _initCloudOta();
  }

  Future<void> _initCloudOta() async {
    await _cloudOta.loadConfig();
    if (mounted) setState(() {});
  }

  Future<void> _loadRemoteFiles() async {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null || !client.isConnected) return;
    setState(() => _isLoadingFiles = true);
    try {
      final response = await client.listRemoteFiles(
        directory: _remoteDirectory,
        category: _currentCategory,
      );
      if (mounted) {
        setState(() {
          _remoteFiles = response.files.toList();
          _isLoadingFiles = false;
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _isLoadingFiles = false;
          _remoteFiles = [];
        });
      }
    }
  }

  Future<void> _uploadFile() async {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null || !client.isConnected) return;
    final result = await FilePicker.platform.pickFiles(type: FileType.any, withData: true);
    if (result == null || result.files.isEmpty) return;
    final file = result.files.first;
    if (file.bytes == null) {
      _showSnackBar('无法读取文件', isError: true);
      return;
    }
    final remotePath = '$_remoteDirectory/${file.name}';
    setState(() {
      _isUploading = true;
      _uploadProgress = 0.0;
      _uploadStatus = '正在上传 ${file.name}...';
    });
    try {
      final response = await client.uploadFile(
        localBytes: file.bytes!,
        remotePath: remotePath,
        filename: file.name,
        category: _currentCategory,
        overwrite: true,
        onProgress: (progress) {
          if (mounted) setState(() => _uploadProgress = progress);
        },
      );
      if (mounted) {
        setState(() { _isUploading = false; _uploadStatus = null; });
        if (response.success) {
          _showSnackBar('上传成功: ${file.name}');
          _loadRemoteFiles();
        } else {
          _showSnackBar('上传失败: ${response.message}', isError: true);
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() { _isUploading = false; _uploadStatus = null; });
        _showSnackBar('上传出错: $e', isError: true);
      }
    }
  }

  Future<void> _deleteFile(RemoteFileInfo file) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('删除文件'),
        content: Text('确定要删除 ${file.filename} 吗？\n\n此操作不可撤销。'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('取消')),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: const Text('删除'),
          ),
        ],
      ),
    );
    if (confirmed != true) return;
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;
    try {
      final response = await client.deleteRemoteFile(remotePath: file.path);
      if (response.success) {
        _showSnackBar('已删除: ${file.filename}');
        _loadRemoteFiles();
      } else {
        _showSnackBar('删除失败: ${response.message}', isError: true);
      }
    } catch (e) {
      _showSnackBar('删除出错: $e', isError: true);
    }
  }

  Future<void> _checkCloudUpdates() async {
    setState(() { _isCheckingCloud = true; _cloudError = null; _releases = []; });
    try {
      final releases = await _cloudOta.fetchReleases(count: 5);
      if (mounted) {
        setState(() { _releases = releases; _isCheckingCloud = false; });
        if (releases.isEmpty) _showSnackBar('暂无可用版本');
      }
    } catch (e) {
      if (mounted) {
        setState(() { _isCheckingCloud = false; _cloudError = e.toString(); });
        _showSnackBar('检查更新失败: $e', isError: true);
      }
    }
  }

  Future<void> _downloadAndDeploy(CloudAsset asset) async {
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null || !client.isConnected) {
      _showSnackBar('请先连接机器人', isError: true);
      return;
    }
    final targetDir = _resolveTargetDir(asset.category);
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('部署确认'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _DeployInfoLine(label: '文件', value: asset.name),
            const SizedBox(height: 6),
            _DeployInfoLine(label: '大小', value: asset.formattedSize),
            const SizedBox(height: 6),
            _DeployInfoLine(label: '类型', value: asset.category),
            const SizedBox(height: 6),
            _DeployInfoLine(label: '目标', value: '$targetDir/${asset.name}'),
            const SizedBox(height: 12),
            const Text('将从云端下载并部署到机器人',
                style: TextStyle(fontSize: 12, color: AppColors.textTertiary)),
          ],
        ),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('取消')),
          FilledButton(onPressed: () => Navigator.pop(ctx, true), child: const Text('下载并部署')),
        ],
      ),
    );
    if (confirmed != true) return;

    setState(() {
      _deployingAssetName = asset.name;
      _deployProgress = 0.0;
      _deployStatus = '正在从云端下载...';
    });
    try {
      final bytes = await _cloudOta.downloadAsset(asset, onProgress: (p) {
        if (mounted) setState(() { _deployProgress = p * 0.5; _deployStatus = '下载中 ${(p * 100).toStringAsFixed(0)}%'; });
      });
      if (!mounted) return;
      setState(() { _deployProgress = 0.5; _deployStatus = '正在部署到机器人...'; });

      final remotePath = '$targetDir/${asset.name}';
      final response = await client.uploadFile(
        localBytes: bytes,
        remotePath: remotePath,
        filename: asset.name,
        category: asset.category,
        overwrite: true,
        onProgress: (p) {
          if (mounted) setState(() { _deployProgress = 0.5 + p * 0.5; _deployStatus = '部署中 ${(p * 100).toStringAsFixed(0)}%'; });
        },
      );
      if (mounted) {
        setState(() { _deployingAssetName = null; _deployStatus = null; });
        if (response.success) {
          _showSnackBar('部署成功: ${asset.name} → $remotePath');
          _loadRemoteFiles();
        } else {
          _showSnackBar('部署失败: ${response.message}', isError: true);
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() { _deployingAssetName = null; _deployStatus = null; });
        _showSnackBar('部署出错: $e', isError: true);
      }
    }
  }

  String _resolveTargetDir(String category) {
    switch (category) {
      case 'model': return '/home/sunrise/models';
      case 'map': return '/home/sunrise/maps';
      case 'config': return '/home/sunrise/config';
      case 'firmware': return '/home/sunrise/firmware';
      default: return '/home/sunrise/downloads';
    }
  }

  Future<void> _editCloudSource() async {
    final ownerCtl = TextEditingController(text: _cloudOta.owner);
    final repoCtl = TextEditingController(text: _cloudOta.repo);
    final customCtl = TextEditingController(text: _cloudOta.customUrl);
    bool useCustom = _cloudOta.useCustomUrl;

    final saved = await showDialog<bool>(
      context: context,
      builder: (ctx) => StatefulBuilder(
        builder: (ctx, setDialogState) => AlertDialog(
          title: const Text('云端源配置'),
          content: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                SwitchListTile(
                  title: const Text('使用自定义 URL', style: TextStyle(fontSize: 14)),
                  value: useCustom,
                  contentPadding: EdgeInsets.zero,
                  onChanged: (v) => setDialogState(() => useCustom = v),
                ),
                const SizedBox(height: 8),
                if (!useCustom) ...[
                  _buildDialogField(ownerCtl, 'GitHub Owner', 'Kitjesen'),
                  const SizedBox(height: 12),
                  _buildDialogField(repoCtl, 'GitHub Repo', '3d_NAV'),
                ] else
                  _buildDialogField(customCtl, 'Releases API URL',
                      'https://api.github.com/repos/.../releases',
                      maxLines: 2),
              ],
            ),
          ),
          actions: [
            TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('取消')),
            FilledButton(onPressed: () => Navigator.pop(ctx, true), child: const Text('保存')),
          ],
        ),
      ),
    );

    if (saved == true) {
      await _cloudOta.saveConfig(
        owner: ownerCtl.text,
        repo: repoCtl.text,
        customUrl: customCtl.text,
        useCustomUrl: useCustom,
      );
      if (mounted) setState(() {});
    }
  }

  Widget _buildDialogField(TextEditingController ctl, String label, String hint,
      {int maxLines = 1}) {
    return TextField(
      controller: ctl,
      maxLines: maxLines,
      style: const TextStyle(color: AppColors.textPrimary, fontSize: 14),
      decoration: InputDecoration(
        labelText: label,
        labelStyle: const TextStyle(color: AppColors.textSecondary),
        hintText: hint,
        hintStyle: const TextStyle(color: AppColors.textTertiary),
        filled: true,
        fillColor: AppColors.bgElevated,
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.border),
        ),
        enabledBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.border),
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.lime),
        ),
        isDense: true,
      ),
    );
  }

  void _showSnackBar(String msg, {bool isError = false}) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(msg),
        backgroundColor: isError ? AppColors.error : AppColors.success,
      ),
    );
  }

  void _switchCategory(String category) {
    if (_currentCategory == category) return;
    setState(() {
      _currentCategory = category;
      _remoteDirectory = _categories[category]!.defaultDir;
      _remoteFiles = [];
    });
    _loadRemoteFiles();
  }

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Scaffold(
      backgroundColor: AppColors.bg,
      body: CustomScrollView(
        slivers: [
          SliverAppBar(
            expandedHeight: 100,
            floating: true,
            pinned: true,
            backgroundColor: AppColors.bg.withOpacity(0.9),
            flexibleSpace: const FlexibleSpaceBar(
              title: Text(
                '设置',
                style: TextStyle(
                  color: AppColors.textPrimary,
                  fontWeight: FontWeight.w700,
                  fontSize: 20,
                ),
              ),
              titlePadding: EdgeInsets.only(left: 20, bottom: 16),
            ),
          ),
          SliverPadding(
            padding: const EdgeInsets.all(16),
            sliver: SliverList(
              delegate: SliverChildListDelegate([
                _buildSectionTitle('连接信息'),
                _buildConnectionInfoCard(),
                const SizedBox(height: 24),

                _buildSectionTitle('文件管理 (OTA 部署)'),
                const SizedBox(height: 8),
                _buildCategorySelector(),
                const SizedBox(height: 12),
                _buildDirectoryHeader(),
                const SizedBox(height: 8),
                _buildRemoteFileList(),
                const SizedBox(height: 24),

                _buildSectionTitle('云端更新 (OTA)'),
                _buildCloudSourceCard(),
                const SizedBox(height: 8),
                _buildCloudDeployProgress(),
                _buildCloudReleaseList(),
                const SizedBox(height: 24),

                _buildSectionTitle('关于'),
                _buildAboutCard(),
                const SizedBox(height: 100),
              ]),
            ),
          ),
        ],
      ),
    );
  }

  // ==================== Connection Info ====================

  Widget _buildConnectionInfoCard() {
    return Consumer<RobotConnectionProvider>(
      builder: (context, provider, _) {
        final isConnected = provider.isConnected;
        final slow = provider.latestSlowState;

        return _DarkCard(
          child: Column(
            children: [
              _InfoRow(
                icon: Icons.wifi,
                label: '状态',
                value: isConnected ? '已连接' : '未连接',
                valueColor: isConnected ? AppColors.success : AppColors.error,
              ),
              if (slow != null) ...[
                const _CardDivider(),
                _InfoRow(icon: Icons.developer_board, label: 'CPU',
                    value: '${slow.resources.cpuPercent.toStringAsFixed(1)}%'),
                const _CardDivider(),
                _InfoRow(icon: Icons.memory, label: '内存',
                    value: '${slow.resources.memPercent.toStringAsFixed(1)}%'),
                const _CardDivider(),
                _InfoRow(icon: Icons.thermostat, label: '温度',
                    value: '${slow.resources.cpuTemp.toStringAsFixed(1)}°C'),
                const _CardDivider(),
                _InfoRow(icon: Icons.battery_charging_full, label: '电池',
                    value: '${slow.resources.batteryPercent.toStringAsFixed(0)}% (${slow.resources.batteryVoltage.toStringAsFixed(1)}V)'),
                const _CardDivider(),
                _InfoRow(icon: Icons.route, label: '模式', value: slow.currentMode),
              ],
            ],
          ),
        );
      },
    );
  }

  // ==================== Category Selector ====================

  Widget _buildCategorySelector() {
    return SizedBox(
      height: 44,
      child: ListView.separated(
        scrollDirection: Axis.horizontal,
        itemCount: _categories.length,
        separatorBuilder: (_, __) => const SizedBox(width: 8),
        itemBuilder: (context, index) {
          final entry = _categories.entries.elementAt(index);
          final isSelected = _currentCategory == entry.key;
          final config = entry.value;

          return GestureDetector(
            onTap: () => _switchCategory(entry.key),
            child: AnimatedContainer(
              duration: const Duration(milliseconds: 200),
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
              decoration: BoxDecoration(
                color: isSelected ? config.color : AppColors.surface,
                borderRadius: BorderRadius.circular(14),
                border: Border.all(
                  color: isSelected ? config.color : AppColors.border,
                  width: 0.5,
                ),
              ),
              child: Row(
                children: [
                  Icon(config.icon, size: 18,
                      color: isSelected ? Colors.black : config.color),
                  const SizedBox(width: 6),
                  Text(
                    config.label,
                    style: TextStyle(
                      color: isSelected ? Colors.black : AppColors.textPrimary,
                      fontWeight: FontWeight.w600,
                      fontSize: 13,
                    ),
                  ),
                ],
              ),
            ),
          );
        },
      ),
    );
  }

  // ==================== Directory Header ====================

  Widget _buildDirectoryHeader() {
    return _DarkCard(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      child: Row(
        children: [
          Icon(Icons.folder_open,
              size: 20, color: _categories[_currentCategory]!.color),
          const SizedBox(width: 10),
          Expanded(
            child: GestureDetector(
              onTap: _editDirectory,
              child: Text(
                _remoteDirectory,
                style: const TextStyle(
                  fontSize: 13,
                  fontFamily: 'monospace',
                  color: AppColors.textSecondary,
                ),
              ),
            ),
          ),
          IconButton(
            icon: _isLoadingFiles
                ? SizedBox(
                    width: 18, height: 18,
                    child: CircularProgressIndicator(
                        strokeWidth: 2, color: AppColors.lime))
                : const Icon(Icons.refresh, size: 20, color: AppColors.textSecondary),
            onPressed: _isLoadingFiles ? null : _loadRemoteFiles,
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(minWidth: 32, minHeight: 32),
          ),
          IconButton(
            icon: Icon(Icons.upload_file,
                size: 20, color: _categories[_currentCategory]!.color),
            onPressed: _isUploading ? null : _uploadFile,
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(minWidth: 32, minHeight: 32),
          ),
        ],
      ),
    );
  }

  Future<void> _editDirectory() async {
    final controller = TextEditingController(text: _remoteDirectory);
    final result = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('远程目录'),
        content: TextField(
          controller: controller,
          style: const TextStyle(fontFamily: 'monospace', fontSize: 14, color: AppColors.textPrimary),
          decoration: InputDecoration(
            hintText: '/home/sunrise/models',
            hintStyle: const TextStyle(color: AppColors.textTertiary),
            filled: true,
            fillColor: AppColors.bgElevated,
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(12),
              borderSide: const BorderSide(color: AppColors.border),
            ),
          ),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx), child: const Text('取消')),
          TextButton(onPressed: () => Navigator.pop(ctx, controller.text), child: const Text('确定')),
        ],
      ),
    );
    if (result != null && result.isNotEmpty) {
      setState(() => _remoteDirectory = result);
      _loadRemoteFiles();
    }
  }

  // ==================== File List ====================

  Widget _buildRemoteFileList() {
    if (_isUploading) {
      return _DarkCard(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                SizedBox(width: 18, height: 18,
                    child: CircularProgressIndicator(strokeWidth: 2, color: AppColors.lime)),
                const SizedBox(width: 12),
                Expanded(child: Text(_uploadStatus ?? '上传中...',
                    style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w500, color: AppColors.textPrimary))),
                Text('${(_uploadProgress * 100).toStringAsFixed(0)}%',
                    style: TextStyle(fontSize: 13, fontWeight: FontWeight.w700,
                        color: _categories[_currentCategory]!.color)),
              ],
            ),
            const SizedBox(height: 10),
            ClipRRect(
              borderRadius: BorderRadius.circular(4),
              child: LinearProgressIndicator(
                value: _uploadProgress,
                backgroundColor: AppColors.surfaceLight,
                valueColor: AlwaysStoppedAnimation(_categories[_currentCategory]!.color),
                minHeight: 6,
              ),
            ),
          ],
        ),
      );
    }

    if (_isLoadingFiles) {
      return _DarkCard(
        child: Center(
          child: Padding(
            padding: const EdgeInsets.all(24),
            child: CircularProgressIndicator(color: AppColors.lime),
          ),
        ),
      );
    }

    if (_remoteFiles.isEmpty) {
      return _DarkCard(
        child: Center(
          child: Padding(
            padding: const EdgeInsets.all(32),
            child: Column(
              children: [
                Icon(Icons.folder_off, size: 48, color: AppColors.textTertiary.withOpacity(0.4)),
                const SizedBox(height: 12),
                const Text('暂无文件',
                    style: TextStyle(fontSize: 15, color: AppColors.textSecondary, fontWeight: FontWeight.w500)),
                const SizedBox(height: 8),
                const Text('点击右上角上传按钮部署文件到机器人',
                    style: TextStyle(fontSize: 12, color: AppColors.textTertiary)),
              ],
            ),
          ),
        ),
      );
    }

    return _DarkCard(
      padding: EdgeInsets.zero,
      child: ListView.separated(
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        itemCount: _remoteFiles.length,
        separatorBuilder: (_, __) => const Divider(height: 1, indent: 56, color: AppColors.divider),
        itemBuilder: (context, index) => _buildFileItem(_remoteFiles[index]),
      ),
    );
  }

  Widget _buildFileItem(RemoteFileInfo file) {
    final ext = file.filename.split('.').last.toLowerCase();
    final iconData = _getFileIcon(ext);
    final config = _categories[_currentCategory]!;

    return ListTile(
      contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
      leading: Container(
        width: 40, height: 40,
        decoration: BoxDecoration(
          color: config.color.withOpacity(0.12),
          borderRadius: BorderRadius.circular(10),
        ),
        child: Icon(iconData, color: config.color, size: 22),
      ),
      title: Text(file.filename,
          style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600, color: AppColors.textPrimary),
          overflow: TextOverflow.ellipsis),
      subtitle: Text('${_formatSize(file.size.toInt())} · ${file.modifiedTime}',
          style: const TextStyle(fontSize: 11, color: AppColors.textTertiary)),
      trailing: IconButton(
        icon: Icon(Icons.delete_outline, size: 20, color: AppColors.error.withOpacity(0.7)),
        onPressed: () => _deleteFile(file),
      ),
    );
  }

  IconData _getFileIcon(String ext) {
    switch (ext) {
      case 'pt': case 'pth': case 'onnx': case 'tflite': case 'engine':
        return Icons.psychology;
      case 'pcd': case 'ply': case 'pickle':
        return Icons.terrain;
      case 'yaml': case 'yml': case 'json': case 'xml':
        return Icons.description;
      case 'bin': case 'hex': case 'img': case 'deb':
        return Icons.memory;
      default:
        return Icons.insert_drive_file;
    }
  }

  String _formatSize(int bytes) {
    if (bytes < 1024) return '$bytes B';
    if (bytes < 1024 * 1024) return '${(bytes / 1024).toStringAsFixed(1)} KB';
    if (bytes < 1024 * 1024 * 1024) return '${(bytes / (1024 * 1024)).toStringAsFixed(1)} MB';
    return '${(bytes / (1024 * 1024 * 1024)).toStringAsFixed(2)} GB';
  }

  // ==================== Cloud OTA UI ====================

  Widget _buildCloudSourceCard() {
    return _DarkCard(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      child: Column(
        children: [
          Row(
            children: [
              Container(
                width: 36, height: 36,
                decoration: BoxDecoration(
                  color: AppColors.info.withOpacity(0.12),
                  borderRadius: BorderRadius.circular(10),
                ),
                child: const Icon(Icons.cloud_outlined, size: 20, color: AppColors.info),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('GitHub Releases',
                        style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600, color: AppColors.textPrimary)),
                    const SizedBox(height: 2),
                    GestureDetector(
                      onTap: _editCloudSource,
                      child: Row(
                        children: [
                          Flexible(
                            child: Text(_cloudOta.repoDisplay,
                                style: const TextStyle(fontSize: 12, color: AppColors.textTertiary, fontFamily: 'monospace'),
                                overflow: TextOverflow.ellipsis),
                          ),
                          const SizedBox(width: 4),
                          const Icon(Icons.edit, size: 12, color: AppColors.textTertiary),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
              _isCheckingCloud
                  ? SizedBox(width: 36, height: 36,
                      child: Padding(padding: const EdgeInsets.all(8),
                          child: CircularProgressIndicator(strokeWidth: 2, color: AppColors.lime)))
                  : FilledButton.icon(
                      onPressed: _checkCloudUpdates,
                      icon: const Icon(Icons.refresh, size: 16),
                      label: const Text('检查', style: TextStyle(fontSize: 12)),
                      style: FilledButton.styleFrom(
                        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                        minimumSize: Size.zero,
                      ),
                    ),
            ],
          ),
          if (_cloudError != null) ...[
            const SizedBox(height: 10),
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(10),
              decoration: BoxDecoration(
                color: AppColors.error.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(_cloudError!,
                  style: const TextStyle(fontSize: 12, color: AppColors.error),
                  maxLines: 3, overflow: TextOverflow.ellipsis),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildCloudDeployProgress() {
    if (_deployingAssetName == null) return const SizedBox.shrink();
    return Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: _DarkCard(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                SizedBox(width: 18, height: 18,
                    child: CircularProgressIndicator(strokeWidth: 2, color: AppColors.lime)),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(_deployingAssetName!,
                          style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: AppColors.textPrimary),
                          overflow: TextOverflow.ellipsis),
                      const SizedBox(height: 2),
                      Text(_deployStatus ?? '',
                          style: const TextStyle(fontSize: 11, color: AppColors.textTertiary)),
                    ],
                  ),
                ),
                Text('${(_deployProgress * 100).toStringAsFixed(0)}%',
                    style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w700, color: AppColors.lime)),
              ],
            ),
            const SizedBox(height: 10),
            ClipRRect(
              borderRadius: BorderRadius.circular(4),
              child: LinearProgressIndicator(
                value: _deployProgress,
                backgroundColor: AppColors.surfaceLight,
                valueColor: const AlwaysStoppedAnimation(AppColors.lime),
                minHeight: 6,
              ),
            ),
            const SizedBox(height: 6),
            Row(
              children: [
                _PhaseIndicator(label: '云端下载', active: _deployProgress <= 0.5, done: _deployProgress > 0.5),
                Expanded(child: Container(height: 1,
                    color: _deployProgress > 0.5 ? AppColors.success : AppColors.surfaceLight)),
                _PhaseIndicator(label: '部署到机器人',
                    active: _deployProgress > 0.5 && _deployProgress < 1.0, done: _deployProgress >= 1.0),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildCloudReleaseList() {
    if (_releases.isEmpty) return const SizedBox.shrink();
    return Column(children: _releases.map((r) => _buildReleaseCard(r)).toList());
  }

  Widget _buildReleaseCard(CloudRelease release) {
    return Padding(
      padding: const EdgeInsets.only(top: 8),
      child: _DarkCard(
        padding: EdgeInsets.zero,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 14, 16, 8),
              child: Row(
                children: [
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                    decoration: BoxDecoration(
                      color: release.prerelease
                          ? AppColors.warning.withOpacity(0.15)
                          : AppColors.success.withOpacity(0.15),
                      borderRadius: BorderRadius.circular(6),
                    ),
                    child: Text(release.tagName,
                        style: TextStyle(fontSize: 12, fontWeight: FontWeight.w700,
                            color: release.prerelease ? AppColors.warning : AppColors.success,
                            fontFamily: 'monospace')),
                  ),
                  if (release.prerelease) ...[
                    const SizedBox(width: 6),
                    Container(
                      padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                      decoration: BoxDecoration(
                        color: AppColors.warning.withOpacity(0.1),
                        borderRadius: BorderRadius.circular(4),
                      ),
                      child: const Text('Pre-release',
                          style: TextStyle(fontSize: 10, color: AppColors.warning)),
                    ),
                  ],
                  const Spacer(),
                  Text(release.publishedDate,
                      style: const TextStyle(fontSize: 11, color: AppColors.textTertiary)),
                ],
              ),
            ),
            if (release.name.isNotEmpty)
              Padding(padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Text(release.name,
                      style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600, color: AppColors.textPrimary),
                      maxLines: 2, overflow: TextOverflow.ellipsis)),
            if (release.body.isNotEmpty)
              Padding(
                padding: const EdgeInsets.fromLTRB(16, 4, 16, 0),
                child: Text(
                  release.body.length > 150 ? '${release.body.substring(0, 150)}...' : release.body,
                  style: const TextStyle(fontSize: 12, color: AppColors.textSecondary, height: 1.4),
                  maxLines: 3, overflow: TextOverflow.ellipsis,
                ),
              ),
            if (release.assets.isNotEmpty) ...[
              Padding(
                padding: const EdgeInsets.fromLTRB(16, 10, 16, 4),
                child: Text('资源文件 (${release.assets.length})',
                    style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600, color: AppColors.textTertiary)),
              ),
              ...release.assets.map((a) => _buildCloudAssetItem(a)),
            ],
            const SizedBox(height: 8),
          ],
        ),
      ),
    );
  }

  Widget _buildCloudAssetItem(CloudAsset asset) {
    final isDeploying = _deployingAssetName == asset.name;
    final canDeploy = asset.category != 'other' && asset.category != 'apk';
    final catColor = _categoryColor(asset.category);

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: Row(
        children: [
          Container(
            width: 34, height: 34,
            decoration: BoxDecoration(
              color: catColor.withOpacity(0.12),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Icon(_categoryIcon(asset.category), size: 18, color: catColor),
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(asset.name,
                    style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w500, color: AppColors.textPrimary),
                    overflow: TextOverflow.ellipsis),
                const SizedBox(height: 2),
                Text('${asset.formattedSize} · ${asset.category} · ${asset.downloadCount} 次下载',
                    style: const TextStyle(fontSize: 11, color: AppColors.textTertiary)),
              ],
            ),
          ),
          if (canDeploy)
            isDeploying
                ? SizedBox(width: 24, height: 24,
                    child: CircularProgressIndicator(strokeWidth: 2, color: AppColors.lime))
                : IconButton(
                    icon: const Icon(Icons.cloud_download, size: 22, color: AppColors.lime),
                    tooltip: '下载并部署到机器人',
                    onPressed: _deployingAssetName != null ? null : () => _downloadAndDeploy(asset),
                    padding: EdgeInsets.zero,
                    constraints: const BoxConstraints(minWidth: 36, minHeight: 36),
                  )
          else if (asset.category == 'apk')
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                color: AppColors.surfaceLight,
                borderRadius: BorderRadius.circular(6),
              ),
              child: const Text('APK',
                  style: TextStyle(fontSize: 10, color: AppColors.textTertiary)),
            ),
        ],
      ),
    );
  }

  Color _categoryColor(String category) {
    switch (category) {
      case 'model': return const Color(0xFF8E8AFF);
      case 'map': return AppColors.success;
      case 'config': return AppColors.warning;
      case 'firmware': return AppColors.error;
      case 'apk': return AppColors.info;
      default: return AppColors.textTertiary;
    }
  }

  IconData _categoryIcon(String category) {
    switch (category) {
      case 'model': return Icons.psychology;
      case 'map': return Icons.terrain;
      case 'config': return Icons.tune;
      case 'firmware': return Icons.memory;
      case 'apk': return Icons.android;
      default: return Icons.insert_drive_file;
    }
  }

  // ==================== About Card ====================

  Widget _buildAboutCard() {
    return _DarkCard(
      child: Column(
        children: const [
          _InfoRow(icon: Icons.info_outline, label: '应用', value: 'MapPilot Monitor'),
          _CardDivider(),
          _InfoRow(icon: Icons.tag, label: '版本', value: '1.0.0'),
          _CardDivider(),
          _InfoRow(icon: Icons.architecture, label: '框架', value: 'Flutter + gRPC'),
          _CardDivider(),
          _InfoRow(icon: Icons.code, label: 'Proto', value: 'robot.v1'),
        ],
      ),
    );
  }

  Widget _buildSectionTitle(String title) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 8, left: 4),
      child: Text(
        title,
        style: const TextStyle(
          fontSize: 16,
          fontWeight: FontWeight.w700,
          color: AppColors.textPrimary,
          letterSpacing: -0.3,
        ),
      ),
    );
  }
}

// ==================== Reusable Widgets ====================

class _CategoryConfig {
  final String label;
  final IconData icon;
  final Color color;
  final String defaultDir;
  final List<String> extensions;

  const _CategoryConfig({
    required this.label,
    required this.icon,
    required this.color,
    required this.defaultDir,
    required this.extensions,
  });
}

class _DarkCard extends StatelessWidget {
  final Widget child;
  final EdgeInsetsGeometry? padding;

  const _DarkCard({required this.child, this.padding});

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: padding ?? const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: AppColors.border.withOpacity(0.3), width: 0.5),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.15),
            blurRadius: 12,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: child,
    );
  }
}

class _CardDivider extends StatelessWidget {
  const _CardDivider();

  @override
  Widget build(BuildContext context) {
    return const Divider(height: 24, color: AppColors.divider, thickness: 0.5);
  }
}

class _DeployInfoLine extends StatelessWidget {
  final String label;
  final String value;

  const _DeployInfoLine({required this.label, required this.value});

  @override
  Widget build(BuildContext context) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        SizedBox(width: 50,
            child: Text(label, style: const TextStyle(fontSize: 13, color: AppColors.textTertiary))),
        Expanded(child: Text(value,
            style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w500,
                fontFamily: 'monospace', color: AppColors.textPrimary))),
      ],
    );
  }
}

class _PhaseIndicator extends StatelessWidget {
  final String label;
  final bool active;
  final bool done;

  const _PhaseIndicator({required this.label, required this.active, required this.done});

  @override
  Widget build(BuildContext context) {
    final color = done
        ? AppColors.success
        : active ? AppColors.lime : AppColors.textTertiary;

    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(done ? Icons.check_circle : Icons.circle, size: 14, color: color),
        const SizedBox(width: 4),
        Text(label, style: TextStyle(fontSize: 10, color: color, fontWeight: FontWeight.w500)),
      ],
    );
  }
}

class _InfoRow extends StatelessWidget {
  final IconData icon;
  final String label;
  final String value;
  final Color? valueColor;

  const _InfoRow({required this.icon, required this.label, required this.value, this.valueColor});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Container(
          width: 32, height: 32,
          decoration: BoxDecoration(
            color: AppColors.lime.withOpacity(0.08),
            borderRadius: BorderRadius.circular(8),
          ),
          child: Icon(icon, size: 18, color: AppColors.lime),
        ),
        const SizedBox(width: 12),
        Text(label, style: const TextStyle(fontSize: 14, color: AppColors.textSecondary)),
        const Spacer(),
        Text(value, style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600,
            color: valueColor ?? AppColors.textPrimary)),
      ],
    );
  }
}
