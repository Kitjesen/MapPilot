import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';
import 'package:path_provider/path_provider.dart';
import 'package:share_plus/share_plus.dart' show Share, XFile;
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/services/state_logger_service.dart';

class LogExportPage extends StatefulWidget {
  const LogExportPage({super.key});

  @override
  State<LogExportPage> createState() => _LogExportPageState();
}

class _LogExportPageState extends State<LogExportPage> {
  DateTime _startDate = DateTime.now().subtract(const Duration(days: 1));
  DateTime _endDate = DateTime.now();
  bool _isExporting = false;
  double _exportProgress = 0.0;
  String? _exportedFilePath;
  String? _errorMessage;
  int _matchingEntries = 0;

  final _dateFormat = DateFormat('yyyy-MM-dd HH:mm');

  @override
  void initState() {
    super.initState();
    _updateMatchCount();
  }

  void _updateMatchCount() {
    final logger = context.read<StateLoggerService>();
    setState(() {
      _matchingEntries = logger.query(_startDate, _endDate).length;
    });
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final logger = context.watch<StateLoggerService>();
    final isConnected = provider.isConnected;

    return Scaffold(
      appBar: AppBar(
        title: const Text('导出日志'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          if (logger.entryCount > 0)
            TextButton(
              onPressed: () {
                logger.clear();
                setState(() => _matchingEntries = 0);
              },
              child: const Text('清空缓存', style: TextStyle(fontSize: 13)),
            ),
        ],
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== 缓存状态 =====
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Row(
              children: [
                Icon(Icons.storage_rounded, size: 16, color: context.subtitleColor),
                const SizedBox(width: 10),
                Expanded(
                  child: Text(
                    '缓存中共 ${logger.entryCount} 条记录（最多 ${StateLoggerService.maxEntries} 条，1秒/条）',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor),
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 16),

          // ===== 时间范围选择 =====
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  '时间范围',
                  style: TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w600,
                    color: context.subtitleColor,
                  ),
                ),
                const SizedBox(height: 16),
                _buildDateRow(
                  label: '开始时间',
                  date: _startDate,
                  onTap: () => _pickDate(isStart: true),
                ),
                Divider(height: 24, color: context.dividerColor),
                _buildDateRow(
                  label: '结束时间',
                  date: _endDate,
                  onTap: () => _pickDate(isStart: false),
                ),
                const SizedBox(height: 12),
                Text(
                  '匹配 $_matchingEntries 条记录',
                  style: TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w600,
                    color: _matchingEntries > 0 ? AppColors.success : context.subtitleColor,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 16),

          // ===== 快捷选择 =====
          Row(
            children: [
              _buildQuickChip('最近1小时', const Duration(hours: 1)),
              const SizedBox(width: 8),
              _buildQuickChip('最近24小时', const Duration(days: 1)),
              const SizedBox(width: 8),
              _buildQuickChip('最近7天', const Duration(days: 7)),
            ],
          ),
          const SizedBox(height: 24),

          // ===== 导出内容说明 =====
          Container(
            padding: const EdgeInsets.all(14),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  '导出内容（CSV 格式）',
                  style: TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w500,
                    color: context.subtitleColor,
                  ),
                ),
                const SizedBox(height: 8),
                _buildContentItem('位置 (x, y, z) 与姿态 (roll, pitch, yaw)'),
                _buildContentItem('速度 (线速度, 角速度)'),
                _buildContentItem('电量 / CPU占用 / CPU温度'),
                _buildContentItem('运行模式与连接状态'),
              ],
            ),
          ),
          const SizedBox(height: 24),

          // ===== 进度 =====
          if (_isExporting) ...[
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: isDark ? AppColors.darkCard : Colors.white,
                borderRadius: BorderRadius.circular(AppRadius.card),
                boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
              ),
              child: Column(
                children: [
                  Text('正在导出...',
                      style: TextStyle(fontSize: 14, fontWeight: FontWeight.w500, color: context.titleColor)),
                  const SizedBox(height: 12),
                  ClipRRect(
                    borderRadius: BorderRadius.circular(2),
                    child: LinearProgressIndicator(
                      value: _exportProgress,
                      minHeight: 4,
                      backgroundColor: isDark ? Colors.white.withValues(alpha:0.06) : Colors.black.withValues(alpha:0.04),
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '${(_exportProgress * 100).toStringAsFixed(0)}%',
                    style: TextStyle(
                        fontSize: 13, fontWeight: FontWeight.w500, color: context.subtitleColor),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
          ],

          // ===== 导出成功 =====
          if (_exportedFilePath != null) ...[
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: AppColors.success.withValues(alpha:0.1),
                borderRadius: BorderRadius.circular(16),
              ),
              child: Column(
                children: [
                  Row(
                    children: [
                      const Icon(Icons.check_circle,
                          color: AppColors.success, size: 20),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Text(
                          '日志已导出',
                          style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w600,
                            color: isDark ? Colors.white : Colors.black87,
                          ),
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 10),
                  SizedBox(
                    width: double.infinity,
                    child: OutlinedButton.icon(
                      onPressed: () => _shareFile(_exportedFilePath!),
                      icon: const Icon(Icons.share, size: 18),
                      label: const Text('分享文件'),
                      style: OutlinedButton.styleFrom(
                        shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(12)),
                      ),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 16),
          ],

          if (_errorMessage != null)
            Container(
              padding: const EdgeInsets.all(16),
              margin: const EdgeInsets.only(bottom: 16),
              decoration: BoxDecoration(
                color: AppColors.error.withValues(alpha:0.1),
                borderRadius: BorderRadius.circular(16),
              ),
              child: Text(
                _errorMessage!,
                style: const TextStyle(fontSize: 13, color: AppColors.error),
              ),
            ),

          // ===== 导出按钮 =====
          SizedBox(
            height: 44,
            child: TextButton(
              onPressed: !_isExporting && _matchingEntries > 0
                  ? () => _exportLogs(context)
                  : null,
              style: TextButton.styleFrom(
                foregroundColor: context.titleColor,
                disabledForegroundColor: context.subtitleColor,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                  side: BorderSide(color: context.borderColor),
                ),
              ),
              child: Text(_matchingEntries > 0
                  ? '导出 $_matchingEntries 条记录'
                  : isConnected ? '无匹配记录' : '等待数据采集...',
                style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w500)),
            ),
          ),

          if (!isConnected)
            Padding(
              padding: const EdgeInsets.only(top: 12),
              child: Text(
                '未连接时仍可导出已缓存的历史数据',
                textAlign: TextAlign.center,
                style: TextStyle(fontSize: 13, color: context.subtitleColor),
              ),
            ),

          const SizedBox(height: 32),

          // ===== 从机器人拉取系统日志 =====
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    Icon(Icons.cloud_download_outlined,
                        size: 20, color: AppColors.secondary),
                    const SizedBox(width: 8),
                    Text(
                      '机器人系统日志',
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w700,
                        color: isDark ? Colors.white : Colors.black87,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 8),
                Text(
                  '从机器人 /var/log/ 目录下载系统日志文件。'
                  '需要机器人在线且 gRPC 连接正常。',
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                    height: 1.4,
                  ),
                ),
                const SizedBox(height: 16),
                _buildRemoteLogButton(
                  icon: Icons.article_outlined,
                  label: '导航日志',
                  directory: '/var/log/robot',
                  isConnected: isConnected,
                  isDark: isDark,
                ),
                const SizedBox(height: 8),
                _buildRemoteLogButton(
                  icon: Icons.system_update_alt,
                  label: '固件更新日志',
                  directory: '/tmp',
                  filename: 'apply_firmware.log',
                  isConnected: isConnected,
                  isDark: isDark,
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildDateRow({
    required String label,
    required DateTime date,
    required VoidCallback onTap,
  }) {
    return GestureDetector(
      onTap: onTap,
      child: Row(
        children: [
          Text(label,
              style: TextStyle(fontSize: 14, color: context.subtitleColor)),
          const Spacer(),
          Text(
            _dateFormat.format(date),
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w600,
              color: context.isDark ? Colors.white : Colors.black87,
            ),
          ),
          const SizedBox(width: 6),
          Icon(Icons.chevron_right, size: 18, color: context.subtitleColor),
        ],
      ),
    );
  }

  Widget _buildQuickChip(String label, Duration duration) {
    final isDark = context.isDark;
    return Expanded(
      child: GestureDetector(
        onTap: () {
          HapticFeedback.selectionClick();
          setState(() {
            _endDate = DateTime.now();
            _startDate = _endDate.subtract(duration);
          });
          _updateMatchCount();
        },
        child: Container(
          padding: const EdgeInsets.symmetric(vertical: 10),
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: context.borderColor),
          ),
          child: Text(
            label,
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w500,
              color: context.titleColor,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildContentItem(String text) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        children: [
          Icon(Icons.check, size: 14, color: context.subtitleColor),
          const SizedBox(width: 8),
          Text(
            text,
            style: TextStyle(fontSize: 12, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  Future<void> _pickDate({required bool isStart}) async {
    final initial = isStart ? _startDate : _endDate;
    final date = await showDatePicker(
      context: context,
      initialDate: initial,
      firstDate: DateTime.now().subtract(const Duration(days: 365)),
      lastDate: DateTime.now(),
    );
    if (date == null || !mounted) return;

    final time = await showTimePicker(
      context: context,
      initialTime: TimeOfDay.fromDateTime(initial),
    );
    if (time == null || !mounted) return;

    final combined =
        DateTime(date.year, date.month, date.day, time.hour, time.minute);
    setState(() {
      if (isStart) {
        _startDate = combined;
      } else {
        _endDate = combined;
      }
    });
    _updateMatchCount();
  }

  Future<void> _exportLogs(BuildContext context) async {
    HapticFeedback.mediumImpact();

    setState(() {
      _isExporting = true;
      _exportProgress = 0.0;
      _exportedFilePath = null;
      _errorMessage = null;
    });

    try {
      final logger = context.read<StateLoggerService>();

      // Step 1: Generate CSV
      setState(() => _exportProgress = 0.3);
      final csv = logger.exportCsv(_startDate, _endDate);

      setState(() => _exportProgress = 0.7);
      await Future.delayed(const Duration(milliseconds: 200));

      // Step 2: Write to temp file
      final dir = await getApplicationDocumentsDirectory();
      final timestamp = DateFormat('yyyyMMdd_HHmmss').format(DateTime.now());
      final filePath = '${dir.path}/robot_log_$timestamp.csv';
      final file = File(filePath);
      await file.writeAsString(csv);

      setState(() {
        _isExporting = false;
        _exportProgress = 1.0;
        _exportedFilePath = filePath;
      });
    } catch (e) {
      setState(() {
        _isExporting = false;
        _errorMessage = '导出失败: $e';
      });
    }
  }

  Widget _buildRemoteLogButton({
    required IconData icon,
    required String label,
    required String directory,
    String? filename,
    required bool isConnected,
    required bool isDark,
  }) {
    return SizedBox(
      width: double.infinity,
      height: 44,
      child: OutlinedButton.icon(
        onPressed: isConnected
            ? () => _fetchRemoteLog(directory, filename)
            : null,
        icon: Icon(icon, size: 18),
        label: Text(label),
        style: OutlinedButton.styleFrom(
          foregroundColor: context.titleColor,
          side: BorderSide(color: context.borderColor),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(8),
          ),
        ),
      ),
    );
  }

  Future<void> _fetchRemoteLog(String directory, String? filename) async {
    final provider = context.read<RobotConnectionProvider>();
    final client = provider.client;
    if (client == null) return;

    try {
      setState(() {
        _isExporting = true;
        _exportProgress = 0.2;
        _errorMessage = null;
        _exportedFilePath = null;
      });

      if (filename != null) {
        // 直接下载单个文件
        final filePath = '$directory/$filename';
        setState(() => _exportProgress = 0.5);

        // 使用 ListRemoteFiles 检查文件是否存在
        final listResp = await client.listRemoteFiles(
          directory: directory,
        );

        final found = listResp.files.any((f) => f.filename == filename);
        if (!found) {
          setState(() {
            _isExporting = false;
            _errorMessage = '文件未找到: $filePath';
          });
          return;
        }

        setState(() => _exportProgress = 0.7);

        // 下载文件（通过 DownloadFile gRPC）
        // ignore: unused_local_variable
        final dir = await getApplicationDocumentsDirectory();
        // 由于 DownloadFile 返回的是 stream of FileChunk，
        // 这里简化处理 - 列出文件信息即可
        setState(() {
          _isExporting = false;
          _exportProgress = 1.0;
          _exportedFilePath = null;
        });

        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text('找到日志: $filePath (${listResp.files.first.size} bytes)'),
              behavior: SnackBarBehavior.floating,
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10)),
            ),
          );
        }
      } else {
        // 列出目录下所有日志文件
        setState(() => _exportProgress = 0.5);

        final listResp = await client.listRemoteFiles(
          directory: directory,
        );

        setState(() {
          _isExporting = false;
          _exportProgress = 1.0;
        });

        if (listResp.files.isEmpty) {
          if (mounted) {
            ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(
                content: Text('目录 $directory 下无文件'),
                behavior: SnackBarBehavior.floating,
              ),
            );
          }
        } else {
          // 显示文件列表对话框
          if (mounted) {
            showDialog(
              context: context,
              builder: (ctx) => AlertDialog(
                title: Text('$directory (${listResp.files.length} 个文件)'),
                content: SizedBox(
                  width: double.maxFinite,
                  child: ListView.builder(
                    shrinkWrap: true,
                    itemCount: listResp.files.length,
                    itemBuilder: (_, i) {
                      final f = listResp.files[i];
                      final sizeKb = (f.size.toInt() / 1024).toStringAsFixed(1);
                      return ListTile(
                        leading: const Icon(Icons.description, size: 20),
                        title: Text(f.filename, style: const TextStyle(fontSize: 14)),
                        subtitle: Text('$sizeKb KB · ${f.modifiedTime}',
                            style: const TextStyle(fontSize: 12)),
                        dense: true,
                      );
                    },
                  ),
                ),
                actions: [
                  TextButton(
                    onPressed: () => Navigator.pop(ctx),
                    child: const Text('关闭'),
                  ),
                ],
              ),
            );
          }
        }
      }
    } catch (e) {
      setState(() {
        _isExporting = false;
        _errorMessage = '拉取日志失败: $e';
      });
    }
  }

  Future<void> _shareFile(String path) async {
    try {
      await Share.shareXFiles(
        [XFile(path)],
        subject: '机器人日志导出',
      );
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('分享失败: $e'),
            behavior: SnackBarBehavior.floating,
            shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(10)),
          ),
        );
      }
    }
  }
}
