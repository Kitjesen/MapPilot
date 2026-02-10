import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/gateway/cloud_ota_client.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/services/ui_error_mapper.dart';

class FirmwareOtaPage extends StatelessWidget {
  const FirmwareOtaPage({super.key});

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final ota = context.watch<OtaGateway>();
    final taskGw = context.watch<TaskGateway>();
    final isConnected = provider.isConnected;
    final slowState = provider.latestSlowState;
    final currentVersion = slowState?.currentMode ?? '未知';
    final battery = slowState?.resources.batteryPercent ?? 0.0;

    return Scaffold(
      appBar: AppBar(
        title: const Text('固件升级'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 18),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
        children: [
          // ===== 设备状态 =====
          _DeviceStatusSection(
            isDark: isDark,
            isConnected: isConnected,
            currentVersion: currentVersion,
            battery: battery,
          ),
          const SizedBox(height: 24),

          // ===== 状态提示 =====
          if (!isConnected) const _Notice(text: '请先连接机器人', type: NoticeType.warning),
          if (isConnected && battery < 30)
            const _Notice(text: '电量低于 30%，建议充电后再升级', type: NoticeType.error),
          if (taskGw.isRunning)
            const _Notice(text: '当前有任务执行中，暂不可执行固件部署', type: NoticeType.warning),
          if (ota.statusMessage != null && ota.activeDeployment == null)
            _Notice(
              text: ota.statusMessage!.contains('失败')
                  ? UiErrorMapper.fromMessage(ota.statusMessage)
                  : ota.statusMessage!,
              type: ota.statusMessage!.contains('成功')
                  ? NoticeType.success
                  : ota.statusMessage!.contains('失败')
                      ? NoticeType.error
                      : NoticeType.info,
            ),

          // ===== 部署进度（事务式）=====
          if (ota.activeDeployment != null) ...[
            _DeploymentProgressSection(
              deployment: ota.activeDeployment!,
              isDark: isDark,
              ota: ota,
            ),
            const SizedBox(height: 24),
          ],

          // Upload progress now handled by deployment transaction above

          // ===== 本地固件 =====
          _SectionTitle(title: '本地固件'),
          const SizedBox(height: 8),
          _LocalFirmwareSection(
            isDark: isDark,
            isConnected: isConnected,
            battery: battery,
            ota: ota,
            taskGw: taskGw,
          ),
          const SizedBox(height: 28),

          // ===== 云端更新 =====
          _SectionTitle(title: '云端更新'),
          const SizedBox(height: 8),
          _CloudSection(
            isDark: isDark,
            isConnected: isConnected,
            battery: battery,
            ota: ota,
            taskGw: taskGw,
          ),
          const SizedBox(height: 28),

          // ===== 已安装版本 =====
          _SectionTitle(title: '已安装版本'),
          const SizedBox(height: 8),
          _InstalledVersionsSection(isDark: isDark, isConnected: isConnected, ota: ota),
          const SizedBox(height: 28),

          // ===== 升级历史 =====
          _SectionTitle(title: '升级历史'),
          const SizedBox(height: 8),
          _HistorySection(isDark: isDark, isConnected: isConnected, ota: ota),
          const SizedBox(height: 28),

          // ===== 升级须知 =====
          _SectionTitle(title: '升级须知'),
          const SizedBox(height: 8),
          _InfoSection(isDark: isDark),

          const SizedBox(height: 40),
        ],
      ),
    );
  }
}

// ================================================================
// 设备状态
// ================================================================

class _DeviceStatusSection extends StatelessWidget {
  final bool isDark;
  final bool isConnected;
  final String currentVersion;
  final double battery;

  const _DeviceStatusSection({
    required this.isDark,
    required this.isConnected,
    required this.currentVersion,
    required this.battery,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('当前固件',
                    style: TextStyle(fontSize: 12, color: context.subtitleColor)),
                const SizedBox(height: 4),
                Text(currentVersion,
                    style: TextStyle(
                        fontSize: 17,
                        fontWeight: FontWeight.w600,
                        color: context.titleColor,
                        letterSpacing: -0.3)),
              ],
            ),
          ),
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              _StatusDot(
                  label: isConnected ? '已连接' : '未连接',
                  color: isConnected ? AppColors.success : const Color(0xFF86868B)),
              const SizedBox(width: 16),
              _StatusDot(
                  label: '${battery.toStringAsFixed(0)}%',
                  color: battery > 30 ? AppColors.success : AppColors.error),
            ],
          ),
        ],
      ),
    );
  }
}

// ================================================================
// 本地固件
// ================================================================

class _LocalFirmwareSection extends StatelessWidget {
  final bool isDark;
  final bool isConnected;
  final double battery;
  final OtaGateway ota;
  final TaskGateway taskGw;

  const _LocalFirmwareSection({
    required this.isDark,
    required this.isConnected,
    required this.battery,
    required this.ota,
    required this.taskGw,
  });

  Future<void> _selectAndUpload(BuildContext context) async {
    HapticFeedback.mediumImpact();

    final result = await FilePicker.platform.pickFiles(
      type: FileType.any,
      withData: true,
    );

    if (result == null || result.files.isEmpty) return;

    final file = result.files.first;
    if (file.bytes == null && file.path == null) {
      _showError(context, '无法读取文件');
      return;
    }

    final bytes = file.bytes ?? await File(file.path!).readAsBytes();
    await ota.deployFromLocal(bytes, file.name);
  }

  void _showError(BuildContext context, String msg) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(UiErrorMapper.fromMessage(msg)),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        children: [
          InkWell(
            borderRadius: BorderRadius.circular(AppRadius.card),
            onTap: isConnected &&
                    ota.activeDeployment?.isBusy != true &&
                    battery >= 30 &&
                    !taskGw.blocksColdOta
                ? () => _selectAndUpload(context)
                : null,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
              child: Row(
                children: [
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text('选择固件文件并上传',
                            style: TextStyle(
                                fontSize: 14,
                                fontWeight: FontWeight.w500,
                                color: isConnected &&
                                        battery >= 30 &&
                                        !taskGw.blocksColdOta
                                    ? context.titleColor
                                    : context.subtitleColor)),
                        const SizedBox(height: 2),
                        Text('支持 .bin / .fw / .zip / .deb 格式',
                            style: TextStyle(fontSize: 12, color: context.subtitleColor)),
                      ],
                    ),
                  ),
                  Icon(Icons.upload_outlined,
                      size: 18,
                      color: isConnected && battery >= 30
                          ? AppColors.primary
                          : context.subtitleColor),
                ],
              ),
            ),
          ),
          if (ota.activeDeployment?.phase == DeployPhase.failed) ...[
            Divider(height: 1, color: isDark ? AppColors.borderDark : AppColors.borderLight),
            InkWell(
              borderRadius: const BorderRadius.only(
                  bottomLeft: Radius.circular(12), bottomRight: Radius.circular(12)),
              onTap: isConnected && ota.activeDeployment?.phase == DeployPhase.failed
                  ? () {
                      HapticFeedback.mediumImpact();
                      ota.retryDeploy();
                    }
                  : null,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                child: Row(
                  children: [
                    Expanded(
                      child: Text(
                          ota.activeDeployment?.phase == DeployPhase.failed ? '重试部署' : '应用固件到机器人',
                          style: TextStyle(
                              fontSize: 14,
                              fontWeight: FontWeight.w500,
                              color: AppColors.warning)),
                    ),
                    Icon(Icons.refresh_outlined, size: 18, color: AppColors.warning),
                  ],
                ),
              ),
            ),
          ],
        ],
      ),
    );
  }
}

// ================================================================
// 云端更新
// ================================================================

class _CloudSection extends StatelessWidget {
  final bool isDark;
  final bool isConnected;
  final double battery;
  final OtaGateway ota;
  final TaskGateway taskGw;

  const _CloudSection({
    required this.isDark,
    required this.isConnected,
    required this.battery,
    required this.ota,
    required this.taskGw,
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Container(
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(AppRadius.card),
            border:
                Border.all(color: isDark ? AppColors.borderDark : AppColors.borderLight),
          ),
          child: Column(
            children: [
              Padding(
                padding: const EdgeInsets.fromLTRB(16, 14, 16, 0),
                child: Row(
                  children: [
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text('更新源',
                              style: TextStyle(fontSize: 12, color: context.subtitleColor)),
                          const SizedBox(height: 2),
                          Text(ota.cloud.repoDisplay,
                              style: TextStyle(
                                  fontSize: 14,
                                  fontWeight: FontWeight.w500,
                                  color: context.titleColor)),
                        ],
                      ),
                    ),
                    Text('GitHub Releases',
                        style: TextStyle(fontSize: 11, color: context.subtitleColor)),
                  ],
                ),
              ),
              Padding(
                padding: const EdgeInsets.fromLTRB(16, 12, 16, 14),
                child: SizedBox(
                  width: double.infinity,
                  height: 40,
                  child: TextButton(
                    onPressed: ota.isCheckingCloud ? null : () => ota.checkCloudUpdate(),
                    style: TextButton.styleFrom(
                      backgroundColor: isDark
                          ? Colors.white.withValues(alpha: 0.05)
                          : Colors.black.withValues(alpha: 0.03),
                      foregroundColor: context.titleColor,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                    ),
                    child: ota.isCheckingCloud
                        ? SizedBox(
                            width: 16,
                            height: 16,
                            child: CircularProgressIndicator(
                                strokeWidth: 2, color: context.subtitleColor))
                        : const Text('检查最新版本',
                            style: TextStyle(fontSize: 13, fontWeight: FontWeight.w500)),
                  ),
                ),
              ),
            ],
          ),
        ),
        if (ota.cloudError != null) ...[
          const SizedBox(height: 12),
          _Notice(text: ota.cloudError!, type: NoticeType.error),
        ],
        if (ota.latestRelease != null) ...[
          const SizedBox(height: 16),
          _ReleaseCard(
              isDark: isDark,
              isConnected: isConnected,
              battery: battery,
              ota: ota,
              taskGw: taskGw),
        ],
      ],
    );
  }
}

class _ReleaseCard extends StatelessWidget {
  final bool isDark;
  final bool isConnected;
  final double battery;
  final OtaGateway ota;
  final TaskGateway taskGw;

  const _ReleaseCard({
    required this.isDark,
    required this.isConnected,
    required this.battery,
    required this.ota,
    required this.taskGw,
  });

  @override
  Widget build(BuildContext context) {
    final release = ota.latestRelease!;
    final firmwareAssets =
        release.assets.where((a) => a.category == 'firmware').toList();

    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Header
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 14, 16, 0),
            child: Row(
              children: [
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                  decoration: BoxDecoration(
                    color: AppColors.primary.withValues(alpha: isDark ? 0.15 : 0.08),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: Text(release.tagName,
                      style: const TextStyle(
                          fontSize: 12,
                          fontWeight: FontWeight.w600,
                          color: AppColors.primary)),
                ),
                if (release.prerelease) ...[
                  const SizedBox(width: 6),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                    decoration: BoxDecoration(
                      color: AppColors.warning.withValues(alpha: 0.1),
                      borderRadius: BorderRadius.circular(6),
                    ),
                    child: Text('Pre',
                        style: TextStyle(
                            fontSize: 10,
                            fontWeight: FontWeight.w600,
                            color: AppColors.warning)),
                  ),
                ],
                const Spacer(),
                Text(release.publishedDate,
                    style: TextStyle(fontSize: 11, color: context.subtitleColor)),
              ],
            ),
          ),
          if (release.name.isNotEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 0),
              child: Text(release.name,
                  style: TextStyle(
                      fontSize: 14,
                      fontWeight: FontWeight.w600,
                      color: context.titleColor)),
            ),
          if (release.body.isNotEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 6, 16, 0),
              child: Text(
                  release.body.length > 200
                      ? '${release.body.substring(0, 200)}...'
                      : release.body,
                  style: TextStyle(
                      fontSize: 12, color: context.subtitleColor, height: 1.5)),
            ),
          const SizedBox(height: 12),
          if (firmwareAssets.isEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 0, 16, 14),
              child: Text('此版本无固件文件',
                  style: TextStyle(fontSize: 12, color: context.subtitleColor)),
            )
          else ...[
            Divider(
                height: 1,
                color: isDark ? AppColors.borderDark : AppColors.borderLight),
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 4),
              child: Text('固件文件（${firmwareAssets.length}）',
                  style: TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w600,
                      color: context.subtitleColor,
                      letterSpacing: 0.3)),
            ),
            ...firmwareAssets.map((asset) => _AssetRow(
                asset: asset,
                isDark: isDark,
                canUpload: isConnected && battery >= 30,
                ota: ota,
                taskGw: taskGw)),
            const SizedBox(height: 8),
          ],
          if (release.deployableAssets.length > firmwareAssets.length) ...[
            Divider(
                height: 1,
                color: isDark ? AppColors.borderDark : AppColors.borderLight),
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 4),
              child: Text('其他资源',
                  style: TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w600,
                      color: context.subtitleColor,
                      letterSpacing: 0.3)),
            ),
            ...release.deployableAssets
                .where((a) => a.category != 'firmware')
                .map((asset) => _AssetRow(
                    asset: asset,
                    isDark: isDark,
                    canUpload: isConnected,
                    ota: ota,
                    taskGw: taskGw)),
            const SizedBox(height: 8),
          ],
        ],
      ),
    );
  }
}

class _AssetRow extends StatelessWidget {
  final CloudAsset asset;
  final bool isDark;
  final bool canUpload;
  final OtaGateway ota;
  final TaskGateway taskGw;

  const _AssetRow({
    required this.asset,
    required this.isDark,
    required this.canUpload,
    required this.ota,
    required this.taskGw,
  });

  @override
  Widget build(BuildContext context) {
    final isThisDeploying = ota.activeDeployment?.isBusy == true &&
        ota.activeDeployment?.cloudAsset?.name == asset.name;
    final anyBusy = ota.activeDeployment?.isBusy == true;
    final firmwareBlockedByTask =
        asset.category == 'firmware' && taskGw.blocksColdOta;
    final canDeploy = canUpload && !anyBusy && !firmwareBlockedByTask;

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      child: Row(
        children: [
          Container(
            width: 6,
            height: 6,
            decoration: BoxDecoration(
                color: _categoryColor(asset.category), shape: BoxShape.circle),
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(asset.name,
                    style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w500,
                        color: context.titleColor),
                    overflow: TextOverflow.ellipsis),
                Text(asset.formattedSize,
                    style: TextStyle(fontSize: 11, color: context.subtitleColor)),
              ],
            ),
          ),
          const SizedBox(width: 8),
          SizedBox(
            height: 28,
            child: TextButton(
              onPressed: canDeploy
                  ? () {
                      HapticFeedback.mediumImpact();
                      _showDeployMethodDialog(context, ota, asset);
                    }
                  : null,
              style: TextButton.styleFrom(
                padding: const EdgeInsets.symmetric(horizontal: 12),
                backgroundColor: isDark
                    ? Colors.white.withValues(alpha: 0.06)
                    : Colors.black.withValues(alpha: 0.04),
                foregroundColor: AppColors.primary,
                disabledForegroundColor: context.subtitleColor,
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
                textStyle:
                    const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
              ),
              child: isThisDeploying
                  ? SizedBox(
                      width: 12,
                      height: 12,
                      child: CircularProgressIndicator(
                          strokeWidth: 1.5, color: AppColors.primary))
                  : Text(firmwareBlockedByTask ? '需空闲' : '部署'),
            ),
          ),
        ],
      ),
    );
  }

  void _showDeployMethodDialog(BuildContext context, OtaGateway ota, CloudAsset asset) {
    showModalBottomSheet(
      context: context,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (ctx) {
        return SafeArea(
          child: Padding(
            padding: const EdgeInsets.symmetric(vertical: 16),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 8),
                  child: Text(
                    '选择部署方式',
                    style: TextStyle(
                      fontSize: 15,
                      fontWeight: FontWeight.w600,
                      color: context.titleColor,
                    ),
                  ),
                ),
                const SizedBox(height: 8),
                ListTile(
                  leading: const Icon(Icons.phone_android, color: AppColors.primary),
                  title: const Text('通过手机中转'),
                  subtitle: const Text('手机下载 → 上传到机器人（适合小文件）'),
                  onTap: () {
                    Navigator.pop(ctx);
                    ota.deployFromCloud(asset);
                  },
                ),
                ListTile(
                  leading: const Icon(Icons.cloud_download, color: AppColors.success),
                  title: const Text('机器人直接下载'),
                  subtitle: const Text('机器人从 URL 直接拉取（适合大文件，省手机内存）'),
                  onTap: () {
                    Navigator.pop(ctx);
                    ota.deployDirectFromUrl(asset);
                  },
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}

// ================================================================
// 已安装版本
// ================================================================

class _InstalledVersionsSection extends StatelessWidget {
  final bool isDark;
  final bool isConnected;
  final OtaGateway ota;

  const _InstalledVersionsSection({
    required this.isDark,
    required this.isConnected,
    required this.ota,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Column(
        children: [
          InkWell(
            borderRadius: BorderRadius.circular(AppRadius.card),
            onTap: isConnected && !ota.isLoadingVersions
                ? () => ota.fetchInstalledVersions()
                : null,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
              child: Row(
                children: [
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text('查看已安装组件',
                            style: TextStyle(
                                fontSize: 14,
                                fontWeight: FontWeight.w500,
                                color: isConnected
                                    ? context.titleColor
                                    : context.subtitleColor)),
                        if (ota.robotSystemVersion != null)
                          Text('系统版本: ${ota.robotSystemVersion}',
                              style:
                                  TextStyle(fontSize: 12, color: context.subtitleColor)),
                      ],
                    ),
                  ),
                  if (ota.isLoadingVersions)
                    const SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2))
                  else
                    Icon(Icons.refresh, size: 18, color: context.subtitleColor),
                ],
              ),
            ),
          ),
          if (ota.installedVersions.isNotEmpty) ...[
            Divider(
                height: 1,
                color: isDark ? AppColors.borderDark : AppColors.borderLight),
            ...ota.installedVersions.map((item) => Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                  child: Row(
                    children: [
                      Container(
                        width: 6,
                        height: 6,
                        decoration: BoxDecoration(
                            color: _categoryColor(item.category),
                            shape: BoxShape.circle),
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(item.name,
                                style: TextStyle(
                                    fontSize: 13,
                                    fontWeight: FontWeight.w500,
                                    color: context.titleColor)),
                            Text('v${item.version}',
                                style: TextStyle(
                                    fontSize: 11, color: context.subtitleColor)),
                          ],
                        ),
                      ),
                      if (ota.rollbackEntries.any((r) => r.name == item.name))
                        SizedBox(
                          height: 26,
                          child: TextButton(
                            onPressed: () => _confirmRollback(context, item.name),
                            style: TextButton.styleFrom(
                              padding: const EdgeInsets.symmetric(horizontal: 10),
                              foregroundColor: AppColors.warning,
                              textStyle: const TextStyle(
                                  fontSize: 11, fontWeight: FontWeight.w500),
                            ),
                            child: const Text('回滚'),
                          ),
                        ),
                    ],
                  ),
                )),
            const SizedBox(height: 4),
          ],
        ],
      ),
    );
  }

  Future<void> _confirmRollback(BuildContext context, String artifactName) async {
    HapticFeedback.mediumImpact();
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('确认回滚'),
        content: Text('确定要回滚 $artifactName 到上一版本吗？'),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx, false), child: const Text('取消')),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            style: TextButton.styleFrom(foregroundColor: AppColors.warning),
            child: const Text('回滚'),
          ),
        ],
      ),
    );

    if (confirmed == true) {
      ota.performRollback(artifactName);
    }
  }
}

// ================================================================
// 升级历史
// ================================================================

class _HistorySection extends StatelessWidget {
  final bool isDark;
  final bool isConnected;
  final OtaGateway ota;

  const _HistorySection({
    required this.isDark,
    required this.isConnected,
    required this.ota,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Column(
        children: [
          InkWell(
            borderRadius: BorderRadius.circular(AppRadius.card),
            onTap: isConnected && !ota.isLoadingHistory
                ? () => ota.fetchUpgradeHistory()
                : null,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
              child: Row(
                children: [
                  Expanded(
                    child: Text('查看升级历史',
                        style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w500,
                            color: isConnected
                                ? context.titleColor
                                : context.subtitleColor)),
                  ),
                  if (ota.isLoadingHistory)
                    const SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(strokeWidth: 2))
                  else
                    Icon(Icons.history, size: 18, color: context.subtitleColor),
                ],
              ),
            ),
          ),
          if (ota.historyEntries.isNotEmpty) ...[
            Divider(
                height: 1,
                color: isDark ? AppColors.borderDark : AppColors.borderLight),
            ...ota.historyEntries.take(10).map((item) => Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
                  child: Row(
                    children: [
                      Container(
                        width: 6,
                        height: 6,
                        decoration: BoxDecoration(
                          color: item.status == 'success'
                              ? AppColors.success
                              : item.status.contains('rolled_back')
                                  ? AppColors.warning
                                  : AppColors.error,
                          shape: BoxShape.circle,
                        ),
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text('${item.action} ${item.artifactName}',
                                style: TextStyle(
                                    fontSize: 13,
                                    fontWeight: FontWeight.w500,
                                    color: context.titleColor)),
                            Text(
                                '${item.fromVersion} -> ${item.toVersion} (${item.status})',
                                style: TextStyle(
                                    fontSize: 11, color: context.subtitleColor)),
                          ],
                        ),
                      ),
                      Text(
                          item.timestamp.length > 10
                              ? item.timestamp.substring(0, 10)
                              : item.timestamp,
                          style: TextStyle(fontSize: 10, color: context.subtitleColor)),
                    ],
                  ),
                )),
            const SizedBox(height: 4),
          ],
        ],
      ),
    );
  }
}

// ================================================================
// 通用组件
// ================================================================

class _SectionTitle extends StatelessWidget {
  final String title;
  const _SectionTitle({required this.title});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(left: 2),
      child: Text(title,
          style: TextStyle(
              fontSize: 13, fontWeight: FontWeight.w600, color: context.subtitleColor)),
    );
  }
}

enum NoticeType { error, warning, success, info }

class _Notice extends StatelessWidget {
  final String text;
  final NoticeType type;

  const _Notice({required this.text, required this.type});

  @override
  Widget build(BuildContext context) {
    final Color accentColor;
    switch (type) {
      case NoticeType.error:
        accentColor = AppColors.error;
      case NoticeType.warning:
        accentColor = AppColors.warning;
      case NoticeType.success:
        accentColor = AppColors.success;
      case NoticeType.info:
        accentColor = AppColors.primary;
    }

    final isDark = context.isDark;
    return Padding(
      padding: const EdgeInsets.only(bottom: 12),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
          border:
              Border.all(color: isDark ? AppColors.borderDark : AppColors.borderLight),
        ),
        child: Row(
          children: [
            Container(
              width: 3,
              height: 28,
              decoration: BoxDecoration(
                  color: accentColor, borderRadius: BorderRadius.circular(2)),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Text(text,
                  style: TextStyle(
                      fontSize: 13,
                      fontWeight: FontWeight.w500,
                      color: context.titleColor)),
            ),
          ],
        ),
      ),
    );
  }
}

// ================================================================
// 部署进度（事务式状态机驱动 — 可视化阶段步进器）
// ================================================================

/// The visible deployment phases for the stepper (excludes idle).
const _stepperPhases = [
  DeployPhase.downloading,
  DeployPhase.uploading,
  DeployPhase.checking,
  DeployPhase.applying,
  DeployPhase.validating,
];

const _stepperIcons = <DeployPhase, IconData>{
  DeployPhase.downloading: Icons.cloud_download_outlined,
  DeployPhase.uploading: Icons.upload_outlined,
  DeployPhase.checking: Icons.verified_user_outlined,
  DeployPhase.applying: Icons.system_update_outlined,
  DeployPhase.validating: Icons.fact_check_outlined,
};

const _stepperLabels = <DeployPhase, String>{
  DeployPhase.downloading: '下载',
  DeployPhase.uploading: '上传',
  DeployPhase.checking: '预检',
  DeployPhase.applying: '安装',
  DeployPhase.validating: '验证',
};

class _DeploymentProgressSection extends StatelessWidget {
  final DeploymentSession deployment;
  final bool isDark;
  final OtaGateway ota;

  const _DeploymentProgressSection({
    required this.deployment,
    required this.isDark,
    required this.ota,
  });

  int get _activeStepIndex {
    final idx = _stepperPhases.indexOf(deployment.phase);
    if (deployment.phase == DeployPhase.completed) return _stepperPhases.length;
    if (deployment.phase == DeployPhase.failed) {
      final failIdx = _stepperPhases.indexOf(deployment.failedAtPhase ?? deployment.phase);
      return failIdx >= 0 ? failIdx : 0;
    }
    return idx >= 0 ? idx : 0;
  }

  @override
  Widget build(BuildContext context) {
    final isFailed = deployment.phase == DeployPhase.failed;
    final isCompleted = deployment.phase == DeployPhase.completed;
    final Color accentColor = isFailed
        ? AppColors.error
        : isCompleted
            ? AppColors.success
            : AppColors.primary;
    final activeIdx = _activeStepIndex;

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
          color: isFailed
              ? AppColors.error.withValues(alpha: 0.3)
              : isCompleted
                  ? AppColors.success.withValues(alpha: 0.3)
                  : accentColor.withValues(alpha: 0.2),
        ),
        boxShadow: [
          if (!isDark)
            BoxShadow(
              color: accentColor.withValues(alpha: 0.06),
              blurRadius: 12,
              offset: const Offset(0, 2),
            ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // ─── Header: phase label + percentage ───
          Row(
            children: [
              // Animated icon
              Container(
                width: 32,
                height: 32,
                decoration: BoxDecoration(
                  color: accentColor.withValues(alpha: 0.12),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: deployment.isBusy
                    ? Padding(
                        padding: const EdgeInsets.all(7),
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          color: accentColor,
                        ),
                      )
                    : Icon(
                        isCompleted
                            ? Icons.check_circle_outline
                            : isFailed
                                ? Icons.error_outline
                                : Icons.cloud_download_outlined,
                        size: 18,
                        color: accentColor,
                      ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      deployment.phaseLabel,
                      style: TextStyle(
                        fontSize: 15,
                        fontWeight: FontWeight.w600,
                        color: accentColor,
                      ),
                    ),
                    const SizedBox(height: 2),
                    Text(
                      deployment.artifactName,
                      style: TextStyle(fontSize: 12, color: context.subtitleColor),
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ],
                ),
              ),
              // Percentage badge
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                decoration: BoxDecoration(
                  color: accentColor.withValues(alpha: 0.1),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Text(
                  '${(deployment.progress * 100).toStringAsFixed(0)}%',
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w700,
                    color: accentColor,
                  ),
                ),
              ),
            ],
          ),

          // ─── Phase Stepper ───
          const SizedBox(height: 16),
          SizedBox(
            height: 56,
            child: Row(
              children: [
                for (int i = 0; i < _stepperPhases.length; i++) ...[
                  if (i > 0)
                    Expanded(
                      child: Container(
                        height: 2,
                        margin: const EdgeInsets.symmetric(horizontal: 2),
                        decoration: BoxDecoration(
                          color: i <= activeIdx
                              ? (isFailed && i == activeIdx
                                  ? AppColors.error.withValues(alpha: 0.5)
                                  : accentColor.withValues(alpha: 0.6))
                              : isDark
                                  ? Colors.white.withValues(alpha: 0.08)
                                  : Colors.black.withValues(alpha: 0.06),
                          borderRadius: BorderRadius.circular(1),
                        ),
                      ),
                    ),
                  _StepDot(
                    phase: _stepperPhases[i],
                    index: i,
                    activeIndex: activeIdx,
                    isFailed: isFailed,
                    isCompleted: isCompleted,
                    isCurrent: _stepperPhases[i] == deployment.phase,
                    isDark: isDark,
                    accentColor: accentColor,
                  ),
                ],
              ],
            ),
          ),

          // ─── Overall Progress Bar ───
          const SizedBox(height: 4),
          ClipRRect(
            borderRadius: BorderRadius.circular(3),
            child: LinearProgressIndicator(
              value: deployment.progress,
              minHeight: 3,
              color: accentColor,
              backgroundColor: isDark
                  ? Colors.white.withValues(alpha: 0.06)
                  : Colors.black.withValues(alpha: 0.04),
            ),
          ),

          // ─── Readiness check results ───
          if (deployment.readinessResults.isNotEmpty) ...[
            const SizedBox(height: 14),
            Container(
              padding: const EdgeInsets.all(10),
              decoration: BoxDecoration(
                color: isDark
                    ? Colors.white.withValues(alpha: 0.03)
                    : Colors.black.withValues(alpha: 0.02),
                borderRadius: BorderRadius.circular(8),
              ),
              child: Column(
                children: deployment.readinessResults
                    .map((check) => Padding(
                          padding: const EdgeInsets.only(bottom: 3),
                          child: Row(
                            children: [
                              Icon(
                                check.passed
                                    ? Icons.check_circle
                                    : Icons.cancel,
                                size: 14,
                                color: check.passed
                                    ? AppColors.success
                                    : AppColors.error,
                              ),
                              const SizedBox(width: 6),
                              Expanded(
                                child: Text(
                                  '${check.checkName}: ${check.message}',
                                  style: TextStyle(
                                    fontSize: 12,
                                    color: check.passed
                                        ? context.subtitleColor
                                        : AppColors.error,
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ))
                    .toList(),
              ),
            ),
          ],

          // ─── Error message ───
          if (isFailed && deployment.errorMessage != null) ...[
            const SizedBox(height: 12),
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: AppColors.error.withValues(alpha: isDark ? 0.1 : 0.05),
                borderRadius: BorderRadius.circular(8),
                border: Border.all(
                    color: AppColors.error.withValues(alpha: 0.15)),
              ),
              child: Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Icon(Icons.error_outline, size: 16, color: AppColors.error),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      deployment.errorMessage!,
                      style: const TextStyle(fontSize: 12, color: AppColors.error, height: 1.4),
                    ),
                  ),
                ],
              ),
            ),
          ],

          // ─── Action buttons ───
          if (isFailed || isCompleted) ...[
            const SizedBox(height: 14),
            Row(
              children: [
                if (deployment.canRetry) ...[
                  Expanded(
                    child: _ActionButton(
                      label: '重试部署',
                      icon: Icons.refresh,
                      color: AppColors.primary,
                      isDark: isDark,
                      onTap: () {
                        HapticFeedback.mediumImpact();
                        ota.retryDeploy();
                      },
                    ),
                  ),
                  const SizedBox(width: 8),
                ],
                if (deployment.canRollback) ...[
                  Expanded(
                    child: _ActionButton(
                      label: '回滚',
                      icon: Icons.undo,
                      color: AppColors.warning,
                      isDark: isDark,
                      onTap: () {
                        HapticFeedback.mediumImpact();
                        ota.performRollback(deployment.artifact.name);
                      },
                    ),
                  ),
                  const SizedBox(width: 8),
                ],
                Expanded(
                  child: _ActionButton(
                    label: isCompleted ? '完成' : '关闭',
                    icon: isCompleted ? Icons.done : Icons.close,
                    color: context.subtitleColor,
                    isDark: isDark,
                    onTap: () => ota.cancelDeploy(),
                  ),
                ),
              ],
            ),
          ],

          // ─── Cancel during progress ───
          if (deployment.isBusy) ...[
            const SizedBox(height: 10),
            Align(
              alignment: Alignment.centerRight,
              child: TextButton.icon(
                onPressed: () => ota.cancelDeploy(),
                icon: Icon(Icons.close, size: 14, color: context.subtitleColor),
                label: Text('取消',
                    style: TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.w500,
                        color: context.subtitleColor)),
              ),
            ),
          ],
        ],
      ),
    );
  }
}

/// A single dot in the phase stepper.
class _StepDot extends StatelessWidget {
  final DeployPhase phase;
  final int index;
  final int activeIndex;
  final bool isFailed;
  final bool isCompleted;
  final bool isCurrent;
  final bool isDark;
  final Color accentColor;

  const _StepDot({
    required this.phase,
    required this.index,
    required this.activeIndex,
    required this.isFailed,
    required this.isCompleted,
    required this.isCurrent,
    required this.isDark,
    required this.accentColor,
  });

  @override
  Widget build(BuildContext context) {
    final bool isDone = isCompleted || index < activeIndex;
    final bool isActive = isCurrent && !isCompleted && !isFailed;
    final bool isBroken = isFailed && index == activeIndex;

    final Color dotColor = isBroken
        ? AppColors.error
        : isDone
            ? AppColors.success
            : isActive
                ? accentColor
                : isDark
                    ? Colors.white.withValues(alpha: 0.15)
                    : Colors.black.withValues(alpha: 0.1);

    final Color iconColor = isBroken || isDone || isActive
        ? Colors.white
        : isDark
            ? Colors.white.withValues(alpha: 0.25)
            : Colors.black.withValues(alpha: 0.2);

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        // Dot with icon
        AnimatedContainer(
          duration: const Duration(milliseconds: 250),
          width: isActive ? 30 : 26,
          height: isActive ? 30 : 26,
          decoration: BoxDecoration(
            color: dotColor,
            shape: BoxShape.circle,
            boxShadow: isActive
                ? [BoxShadow(color: accentColor.withValues(alpha: 0.3), blurRadius: 8)]
                : null,
          ),
          child: Icon(
            isBroken
                ? Icons.close
                : isDone
                    ? Icons.check
                    : _stepperIcons[phase] ?? Icons.circle,
            size: isActive ? 15 : 13,
            color: iconColor,
          ),
        ),
        const SizedBox(height: 4),
        // Label
        Text(
          _stepperLabels[phase] ?? '',
          style: TextStyle(
            fontSize: 10,
            fontWeight: isActive || isDone ? FontWeight.w600 : FontWeight.w400,
            color: isBroken
                ? AppColors.error
                : isDone || isActive
                    ? (isDark ? Colors.white70 : Colors.black87)
                    : (isDark ? Colors.white24 : Colors.black26),
          ),
        ),
      ],
    );
  }
}

/// Pill-shaped action button for deployment actions.
class _ActionButton extends StatelessWidget {
  final String label;
  final IconData icon;
  final Color color;
  final bool isDark;
  final VoidCallback onTap;

  const _ActionButton({
    required this.label,
    required this.icon,
    required this.color,
    required this.isDark,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: onTap,
        borderRadius: BorderRadius.circular(8),
        child: Container(
          padding: const EdgeInsets.symmetric(vertical: 10),
          decoration: BoxDecoration(
            color: color.withValues(alpha: isDark ? 0.12 : 0.08),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: color.withValues(alpha: 0.2)),
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(icon, size: 15, color: color),
              const SizedBox(width: 6),
              Text(
                label,
                style: TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w600,
                  color: color,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _InfoSection extends StatelessWidget {
  final bool isDark;
  const _InfoSection({required this.isDark});

  @override
  Widget build(BuildContext context) {
    final items = [
      '确保机器人电量 >= 30%',
      '升级过程中请勿断开连接',
      '确保机器人处于静止状态',
      '支持 .bin / .fw / .zip / .deb 格式',
      '上传完成后需点击"应用固件"触发刷写',
    ];

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      child: Column(
        children: items.asMap().entries.map((entry) {
          final isLast = entry.key == items.length - 1;
          return Padding(
            padding: EdgeInsets.only(bottom: isLast ? 0 : 10),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Padding(
                  padding: const EdgeInsets.only(top: 6),
                  child: Container(
                    width: 4,
                    height: 4,
                    decoration: BoxDecoration(
                        color: context.subtitleColor, shape: BoxShape.circle),
                  ),
                ),
                const SizedBox(width: 10),
                Expanded(
                  child: Text(entry.value,
                      style: TextStyle(
                          fontSize: 13, color: context.subtitleColor, height: 1.3)),
                ),
              ],
            ),
          );
        }).toList(),
      ),
    );
  }
}

class _StatusDot extends StatelessWidget {
  final String label;
  final Color color;

  const _StatusDot({required this.label, required this.color});

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 6,
          height: 6,
          decoration: BoxDecoration(color: color, shape: BoxShape.circle),
        ),
        const SizedBox(width: 5),
        Text(label,
            style: TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w500,
                color: context.subtitleColor)),
      ],
    );
  }
}

// ================================================================
// Helpers
// ================================================================

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
