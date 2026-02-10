import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';

/// Settings page for configuring the Cloud OTA source (GitHub repo or custom URL).
class CloudConfigPage extends StatefulWidget {
  const CloudConfigPage({super.key});

  @override
  State<CloudConfigPage> createState() => _CloudConfigPageState();
}

class _CloudConfigPageState extends State<CloudConfigPage> {
  late TextEditingController _ownerController;
  late TextEditingController _repoController;
  late TextEditingController _customUrlController;
  late bool _useCustomUrl;
  bool _saving = false;

  @override
  void initState() {
    super.initState();
    final cloud = context.read<OtaGateway>().cloud;
    _ownerController = TextEditingController(text: cloud.owner);
    _repoController = TextEditingController(text: cloud.repo);
    _customUrlController = TextEditingController(text: cloud.customUrl);
    _useCustomUrl = cloud.useCustomUrl;
  }

  @override
  void dispose() {
    _ownerController.dispose();
    _repoController.dispose();
    _customUrlController.dispose();
    super.dispose();
  }

  Future<void> _save() async {
    setState(() => _saving = true);
    final cloud = context.read<OtaGateway>().cloud;
    await cloud.saveConfig(
      owner: _ownerController.text.trim(),
      repo: _repoController.text.trim(),
      customUrl: _customUrlController.text.trim(),
      useCustomUrl: _useCustomUrl,
    );
    if (mounted) {
      setState(() => _saving = false);
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('云端更新源已保存'),
          behavior: SnackBarBehavior.floating,
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final isDark = Theme.of(context).brightness == Brightness.dark;
    final cardColor = isDark ? AppColors.darkCard : Colors.white;
    final borderColor = isDark ? AppColors.borderDark : AppColors.borderLight;

    return Scaffold(
      appBar: AppBar(
        title: const Text('云端更新源'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          TextButton(
            onPressed: _saving ? null : _save,
            child: _saving
                ? const SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  )
                : const Text('保存'),
          ),
        ],
      ),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          // Mode toggle
          Container(
            decoration: BoxDecoration(
              color: cardColor,
              borderRadius: BorderRadius.circular(AppRadius.card),
              boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
            ),
            child: SwitchListTile(
              title: const Text('使用自定义 URL'),
              subtitle: const Text('替代 GitHub API 的自定义更新源'),
              value: _useCustomUrl,
              onChanged: (v) => setState(() => _useCustomUrl = v),
            ),
          ),
          const SizedBox(height: 16),

          if (_useCustomUrl) ...[
            // Custom URL
            _buildField(
              label: '自定义 API URL',
              hint: 'https://your-server.com/api/releases',
              controller: _customUrlController,
              icon: Icons.link,
              cardColor: cardColor,
              borderColor: borderColor,
            ),
          ] else ...[
            // GitHub owner
            _buildField(
              label: 'GitHub 用户/组织',
              hint: 'DaSuanRobot',
              controller: _ownerController,
              icon: Icons.person_outline,
              cardColor: cardColor,
              borderColor: borderColor,
            ),
            const SizedBox(height: 12),
            // GitHub repo
            _buildField(
              label: 'GitHub 仓库名',
              hint: 'MapPilot',
              controller: _repoController,
              icon: Icons.inventory_2_outlined,
              cardColor: cardColor,
              borderColor: borderColor,
            ),
          ],

          const SizedBox(height: 24),

          // Preview
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: isDark
                  ? Colors.blueGrey.withValues(alpha:0.15)
                  : Colors.blue.withValues(alpha:0.05),
              borderRadius: BorderRadius.circular(12),
              border: Border.all(
                color: isDark
                    ? Colors.blueGrey.withValues(alpha:0.3)
                    : Colors.blue.withValues(alpha:0.15),
              ),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  '当前更新源',
                  style: TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w600,
                    color: Colors.blueGrey,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  _useCustomUrl
                      ? (_customUrlController.text.isEmpty
                          ? '(未设置)'
                          : _customUrlController.text)
                      : '${_ownerController.text}/${_repoController.text}',
                  style: TextStyle(
                    fontSize: 14,
                    color: isDark ? Colors.white70 : Colors.black87,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildField({
    required String label,
    required String hint,
    required TextEditingController controller,
    required IconData icon,
    required Color cardColor,
    required Color borderColor,
  }) {
    return Container(
      decoration: BoxDecoration(
        color: cardColor,
        borderRadius: BorderRadius.circular(AppRadius.card),
        boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
      ),
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: TextField(
        controller: controller,
        decoration: InputDecoration(
          labelText: label,
          hintText: hint,
          icon: Icon(icon, size: 20),
          border: InputBorder.none,
        ),
        onChanged: (_) => setState(() {}),
      ),
    );
  }
}
