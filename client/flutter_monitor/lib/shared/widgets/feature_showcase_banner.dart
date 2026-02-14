import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/responsive.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';

// ═══════════════════════════════════════════════════════════════
//  FeatureShowcaseBanner — Ultralytics-style dark hero banner
//  with brand section + horizontal white feature cards
// ═══════════════════════════════════════════════════════════════

class FeatureShowcaseBanner extends StatefulWidget {
  final VoidCallback? onGetStarted;
  final LocaleProvider? locale;

  const FeatureShowcaseBanner({super.key, this.onGetStarted, this.locale});

  @override
  State<FeatureShowcaseBanner> createState() => _FeatureShowcaseBannerState();
}

class _FeatureShowcaseBannerState extends State<FeatureShowcaseBanner>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1200),
    )..forward();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final isMobile = context.isMobile;

    return ClipRRect(
      borderRadius: BorderRadius.circular(24),
      child: Container(
        constraints: const BoxConstraints(minHeight: 200),
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              Color(0xFF0B1026),  // deep navy
              Color(0xFF141B3D),  // mid navy
              Color(0xFF1A2550),  // lighter navy
              Color(0xFF0E1A3A),  // back to dark
            ],
            stops: [0.0, 0.35, 0.65, 1.0],
          ),
        ),
        child: Stack(
          children: [
            // ── Decorative light streaks ──
            Positioned(
              top: -60,
              right: -40,
              child: Container(
                width: 300,
                height: 300,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  gradient: RadialGradient(
                    colors: [
                      const Color(0xFF3B82F6).withValues(alpha: 0.08),
                      Colors.transparent,
                    ],
                  ),
                ),
              ),
            ),
            Positioned(
              bottom: -80,
              left: -60,
              child: Container(
                width: 250,
                height: 250,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  gradient: RadialGradient(
                    colors: [
                      const Color(0xFF8B5CF6).withValues(alpha: 0.06),
                      Colors.transparent,
                    ],
                  ),
                ),
              ),
            ),

            // ── Subtle horizontal light beam ──
            Positioned(
              top: 0,
              left: 0,
              right: 0,
              bottom: 0,
              child: Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.centerLeft,
                    end: Alignment.centerRight,
                    colors: [
                      Colors.transparent,
                      const Color(0xFF60A5FA).withValues(alpha: 0.03),
                      const Color(0xFF818CF8).withValues(alpha: 0.04),
                      Colors.transparent,
                    ],
                    stops: const [0.0, 0.3, 0.7, 1.0],
                  ),
                ),
              ),
            ),

            // ── Content ──
            Padding(
              padding: EdgeInsets.all(isMobile ? 20 : 32),
              child: isMobile
                  ? _buildMobileLayout(context)
                  : _buildDesktopLayout(context),
            ),
          ],
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  DESKTOP: Brand left + Cards right
  // ═══════════════════════════════════════════════════════════
  Widget _buildDesktopLayout(BuildContext context) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        // ── Brand section ──
        _staggerIn(0, SizedBox(
          width: 220,
          child: _buildBrandSection(context),
        )),
        const SizedBox(width: 32),

        // ── Feature cards ──
        Expanded(
          child: SizedBox(
            height: 160,
            child: _buildCardRow(context),
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  MOBILE: Brand top + Cards bottom (horizontal scroll)
  // ═══════════════════════════════════════════════════════════
  Widget _buildMobileLayout(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: [
        _staggerIn(0, _buildBrandSection(context)),
        const SizedBox(height: 20),
        SizedBox(
          height: 140,
          child: _buildCardRow(context),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  BRAND SECTION — Logo, tagline, CTA button
  // ═══════════════════════════════════════════════════════════
  Widget _buildBrandSection(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: [
        // Logo row
        Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            // DS monogram
            Container(
              width: 44,
              height: 44,
              decoration: BoxDecoration(
                gradient: const LinearGradient(
                  begin: Alignment.topLeft,
                  end: Alignment.bottomRight,
                  colors: [Color(0xFF6C63FF), Color(0xFF8B5CF6)],
                ),
                borderRadius: BorderRadius.circular(14),
                boxShadow: [
                  BoxShadow(
                    color: const Color(0xFF6C63FF).withValues(alpha: 0.4),
                    blurRadius: 16,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: const Center(
                child: Text(
                  'DS',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.w900,
                    color: Colors.white,
                    letterSpacing: 0.5,
                    height: 1,
                  ),
                ),
              ),
            ),
            const SizedBox(width: 12),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  '3D NAV',
                  style: TextStyle(
                    fontSize: 22,
                    fontWeight: FontWeight.w800,
                    color: Colors.white,
                    letterSpacing: -0.5,
                    height: 1.1,
                  ),
                ),
                Text(
                  'v1.3.2',
                  style: TextStyle(
                    fontSize: 11,
                    fontWeight: FontWeight.w600,
                    color: Colors.white.withValues(alpha: 0.4),
                    letterSpacing: 1,
                  ),
                ),
              ],
            ),
          ],
        ),
        const SizedBox(height: 14),

        // Tagline
        Text(
          widget.locale?.tr('为自主而生。\n为边缘而生。', 'Built for Autonomy.\nBuilt for the Edge.') ?? 'Built for Autonomy.\nBuilt for the Edge.',
          style: TextStyle(
            fontSize: 14,
            fontWeight: FontWeight.w500,
            color: Colors.white.withValues(alpha: 0.55),
            height: 1.5,
          ),
        ),
        const SizedBox(height: 16),

        // CTA button
        GestureDetector(
          onTap: () {
            HapticFeedback.mediumImpact();
            widget.onGetStarted?.call();
          },
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
            decoration: BoxDecoration(
              color: const Color(0xFFD4FF00),
              borderRadius: BorderRadius.circular(20),
              boxShadow: [
                BoxShadow(
                  color: const Color(0xFFD4FF00).withValues(alpha: 0.3),
                  blurRadius: 16,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: Text(
              widget.locale?.tr('开始使用', 'Get started') ?? 'Get started',
              style: const TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.w700,
                color: Color(0xFF0B1026),
                letterSpacing: 0.2,
              ),
            ),
          ),
        ),
      ],
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  CARD ROW — Horizontal scrollable feature cards
  // ═══════════════════════════════════════════════════════════
  Widget _buildCardRow(BuildContext context) {
    final l = widget.locale;
    final features = [
      _FeatureItem(Icons.view_in_ar_rounded, l?.tr('3D SLAM\n建图', '3D SLAM\nMapping') ?? '3D SLAM\nMapping'),
      _FeatureItem(Icons.route_rounded, l?.tr('路径\n规划', 'Path\nPlanning') ?? 'Path\nPlanning'),
      _FeatureItem(Icons.speed_rounded, l?.tr('实时\n控制', 'Real-time\nControl') ?? 'Real-time\nControl'),
      _FeatureItem(Icons.security_rounded, l?.tr('安全\n围栏', 'Safety\nGeofence') ?? 'Safety\nGeofence'),
      _FeatureItem(Icons.cloud_sync_rounded, l?.tr('OTA\n部署', 'OTA\nDeploy') ?? 'OTA\nDeploy'),
    ];

    return ListView.separated(
      scrollDirection: Axis.horizontal,
      physics: const BouncingScrollPhysics(),
      padding: const EdgeInsets.symmetric(horizontal: 4),
      itemCount: features.length,
      separatorBuilder: (_, __) => const SizedBox(width: 14),
      itemBuilder: (context, index) {
        return _staggerIn(
          index + 1,
          _ShowcaseCard(
            icon: features[index].icon,
            label: features[index].label,
          ),
        );
      },
    );
  }

  // ═══════════════════════════════════════════════════════════
  //  STAGGER ANIMATION
  // ═══════════════════════════════════════════════════════════
  Widget _staggerIn(int index, Widget child) {
    final delay = index * 0.08;
    final animation = CurvedAnimation(
      parent: _controller,
      curve: Interval(
        delay.clamp(0.0, 0.6),
        (delay + 0.4).clamp(0.0, 1.0),
        curve: Curves.easeOutCubic,
      ),
    );
    return FadeTransition(
      opacity: animation,
      child: SlideTransition(
        position: Tween(
          begin: const Offset(0, 0.08),
          end: Offset.zero,
        ).animate(animation),
        child: child,
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Feature data
// ═══════════════════════════════════════════════════════════════

class _FeatureItem {
  final IconData icon;
  final String label;
  const _FeatureItem(this.icon, this.label);
}

final List<_FeatureItem> _featureItems = [
  const _FeatureItem(
    Icons.view_in_ar_rounded,
    '3D SLAM\nMapping',
  ),
  const _FeatureItem(
    Icons.route_rounded,
    'Path\nPlanning',
  ),
  const _FeatureItem(
    Icons.speed_rounded,
    'Real-time\nControl',
  ),
  const _FeatureItem(
    Icons.security_rounded,
    'Safety\nGeofence',
  ),
  const _FeatureItem(
    Icons.cloud_sync_rounded,
    'OTA\nDeploy',
  ),
];

// ═══════════════════════════════════════════════════════════════
//  Showcase Card — white card with icon + label
// ═══════════════════════════════════════════════════════════════

class _ShowcaseCard extends StatefulWidget {
  final IconData icon;
  final String label;

  const _ShowcaseCard({
    required this.icon,
    required this.label,
  });

  @override
  State<_ShowcaseCard> createState() => _ShowcaseCardState();
}

class _ShowcaseCardState extends State<_ShowcaseCard> {
  bool _hovering = false;

  @override
  Widget build(BuildContext context) {
    return MouseRegion(
      onEnter: (_) => setState(() => _hovering = true),
      onExit: (_) => setState(() => _hovering = false),
      child: GestureDetector(
        onTap: () => HapticFeedback.lightImpact(),
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          curve: Curves.easeOut,
          width: 120,
          padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 18),
          transform: Matrix4.identity()
            ..translate(0.0, _hovering ? -4.0 : 0.0),
          decoration: BoxDecoration(
            color: Colors.white,
            borderRadius: BorderRadius.circular(16),
            boxShadow: [
              BoxShadow(
                color: _hovering
                    ? const Color(0xFF6C63FF).withValues(alpha: 0.15)
                    : Colors.black.withValues(alpha: 0.06),
                blurRadius: _hovering ? 20 : 12,
                offset: Offset(0, _hovering ? 8 : 4),
              ),
            ],
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              // Icon container with subtle blue tint
              Container(
                width: 52,
                height: 52,
                decoration: BoxDecoration(
                  color: const Color(0xFFEEF2FF),
                  borderRadius: BorderRadius.circular(14),
                ),
                child: Icon(
                  widget.icon,
                  size: 26,
                  color: const Color(0xFF4F5DCC),
                ),
              ),
              const SizedBox(height: 12),
              // Label
              Text(
                widget.label,
                textAlign: TextAlign.center,
                style: const TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                  color: Color(0xFF1E1B4B),
                  height: 1.3,
                  letterSpacing: -0.1,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// ═══════════════════════════════════════════════════════════════
//  Compact variant: CapabilityChip — small horizontal badge
//  for use in tighter spaces (e.g. status bars)
// ═══════════════════════════════════════════════════════════════

class CapabilityChip extends StatelessWidget {
  final IconData icon;
  final String label;
  final bool active;

  const CapabilityChip({
    super.key,
    required this.icon,
    required this.label,
    this.active = true,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: active
            ? const Color(0xFF6C63FF).withValues(alpha: 0.1)
            : Colors.grey.withValues(alpha: 0.1),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: active
              ? const Color(0xFF6C63FF).withValues(alpha: 0.2)
              : Colors.grey.withValues(alpha: 0.15),
        ),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            icon,
            size: 14,
            color: active ? const Color(0xFF6C63FF) : Colors.grey,
          ),
          const SizedBox(width: 6),
          Text(
            label,
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.w600,
              color: active ? const Color(0xFF4338CA) : Colors.grey,
            ),
          ),
        ],
      ),
    );
  }
}
