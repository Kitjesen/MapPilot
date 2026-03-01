import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';
import 'package:flutter_monitor/shared/widgets/skeleton_loader.dart';

class EventsScreen extends StatefulWidget {
  const EventsScreen({super.key});

  @override
  State<EventsScreen> createState() => _EventsScreenState();
}

class _EventsScreenState extends State<EventsScreen>
    with AutomaticKeepAliveClientMixin {
  final List<Event> _events = [];
  StreamSubscription? _subscription;
  bool _isStreaming = false;
  bool _initialLoading = true;

  static const int _maxEvents = 200;

  int _retryCount = 0;
  Timer? _retryTimer;

  // ─── Filter + search state ───
  EventSeverity? _filterSeverity; // null = ALL
  String _searchQuery = '';
  final _searchCtrl = TextEditingController();

  List<Event> get _filteredEvents {
    var list = _events;
    if (_filterSeverity != null) {
      list = list.where((e) => e.severity == _filterSeverity).toList();
    }
    if (_searchQuery.isNotEmpty) {
      final q = _searchQuery.toLowerCase();
      list = list.where((e) =>
        e.title.toLowerCase().contains(q) ||
        e.description.toLowerCase().contains(q)).toList();
    }
    return list;
  }

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startListening();
    });
    // Clear initial loading skeleton after 3s even if no events arrive
    Future.delayed(const Duration(seconds: 3), () {
      if (mounted && _initialLoading) setState(() => _initialLoading = false);
    });
  }

  @override
  void dispose() {
    _subscription?.cancel();
    _retryTimer?.cancel();
    _searchCtrl.dispose();
    super.dispose();
  }

  void _startListening() {
    _retryTimer?.cancel();
    final client = context.read<RobotConnectionProvider>().client;
    if (client == null) return;

    try {
      final lastId = _events.isNotEmpty ? _events.first.eventId : '';

      _subscription = client.streamEvents(lastEventId: lastId).listen(
        (event) {
          if (!mounted) return;
          setState(() {
            if (!_events.any((e) => e.eventId == event.eventId)) {
              _events.insert(0, event);
              if (_events.length > _maxEvents) {
                _events.removeRange(_maxEvents, _events.length);
              }
            }
            _isStreaming = true;
            _initialLoading = false;
            _retryCount = 0;
          });
        },
        onError: (e) {
          debugPrint('[Events] Stream error: $e');
          if (mounted) {
            setState(() => _isStreaming = false);
            _scheduleRetry();
          }
        },
      );
    } catch (e) {
      debugPrint('[Events] Failed to start stream: $e');
      _scheduleRetry();
    }
  }

  void _scheduleRetry() {
    _retryTimer?.cancel();
    _retryCount++;
    final delay = Duration(
      seconds: (3 * (1 << (_retryCount - 1).clamp(0, 3))).clamp(3, 30),
    );
    _retryTimer = Timer(delay, () {
      if (mounted) _startListening();
    });
  }

  Color _getSeverityColor(EventSeverity severity) {
    switch (severity) {
      case EventSeverity.EVENT_SEVERITY_DEBUG:
        return Colors.grey;
      case EventSeverity.EVENT_SEVERITY_INFO:
        return AppColors.primary;
      case EventSeverity.EVENT_SEVERITY_WARNING:
        return AppColors.warning;
      case EventSeverity.EVENT_SEVERITY_ERROR:
        return AppColors.error;
      case EventSeverity.EVENT_SEVERITY_CRITICAL:
        return AppColors.accent;
      default:
        return Colors.grey;
    }
  }

  String _getSeverityLabel(EventSeverity severity) {
    switch (severity) {
      case EventSeverity.EVENT_SEVERITY_DEBUG:
        return 'DEBUG';
      case EventSeverity.EVENT_SEVERITY_INFO:
        return 'INFO';
      case EventSeverity.EVENT_SEVERITY_WARNING:
        return 'WARN';
      case EventSeverity.EVENT_SEVERITY_ERROR:
        return 'ERROR';
      case EventSeverity.EVENT_SEVERITY_CRITICAL:
        return 'CRITICAL';
      default:
        return '';
    }
  }

  @override
  Widget build(BuildContext context) {
    super.build(context);
    final locale = context.watch<LocaleProvider>();

    return Scaffold(
      appBar: AppBar(
        title: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(locale.tr('事件日志', 'Event Log')),
            if (_events.isNotEmpty) ...[
              const SizedBox(width: 8),
              Text(
                '${_events.length}',
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w500,
                  color: context.subtitleColor,
                ),
              ),
            ],
          ],
        ),
        leading: Navigator.canPop(context)
            ? IconButton(
                icon: const Icon(Icons.arrow_back_ios_new, size: 20),
                onPressed: () => Navigator.pop(context),
              )
            : null,
        actions: [
          if (_filteredEvents.isNotEmpty)
            IconButton(
              icon: const Icon(Icons.share_outlined, size: 20),
              tooltip: locale.tr('导出事件', 'Export events'),
              onPressed: () => _exportEvents(context, locale),
            ),
          if (_events.isNotEmpty)
            IconButton(
              icon: const Icon(Icons.delete_sweep_outlined, size: 20),
              tooltip: locale.tr('清除全部', 'Clear all'),
              onPressed: () {
                HapticFeedback.lightImpact();
                setState(() {
                  _events.clear();
                  _filterSeverity = null;
                  _searchQuery = '';
                  _searchCtrl.clear();
                });
              },
            ),
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () {
              HapticFeedback.lightImpact();
              _subscription?.cancel();
              setState(() { _events.clear(); _retryCount = 0; });
              _startListening();
            },
          ),
        ],
      ),
      body: RefreshIndicator(
        onRefresh: () async {
          _subscription?.cancel();
          setState(() { _events.clear(); _retryCount = 0; });
          _startListening();
          // Give stream a moment to deliver initial events
          await Future.delayed(const Duration(milliseconds: 500));
        },
        child: Column(
          children: [
            // ── Filter chips ──
            if (_events.isNotEmpty) _buildFilterBar(context, locale),
            // ── Events list ──
            Expanded(child: _buildEventsList(context, locale)),
          ],
        ),
      ),
    );
  }

  void _exportEvents(BuildContext context, LocaleProvider locale) {
    HapticFeedback.lightImpact();
    final events = _filteredEvents;
    if (events.isEmpty) return;

    final buffer = StringBuffer();
    buffer.writeln(locale.tr('事件日志导出', 'Event Log Export'));
    buffer.writeln('${'=' * 50}');
    for (final event in events) {
      final time = event.timestamp.toDateTime().toLocal();
      final timeStr = DateFormat('yyyy-MM-dd HH:mm:ss').format(time);
      final level = _getSeverityLabel(event.severity);
      final title = event.title.isNotEmpty ? event.title : event.type.toString();
      final desc = event.description;
      buffer.writeln('$timeStr | $level | $title${desc.isNotEmpty ? ' | $desc' : ''}');
    }

    Clipboard.setData(ClipboardData(text: buffer.toString()));
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(locale.tr(
          '${events.length} 条事件已复制到剪贴板',
          '${events.length} events copied to clipboard',
        )),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        duration: const Duration(seconds: 2),
      ),
    );
  }

  Widget _buildFilterBar(BuildContext context, LocaleProvider locale) {
    const chips = [
      (null, 'ALL'),
      (EventSeverity.EVENT_SEVERITY_INFO, 'INFO'),
      (EventSeverity.EVENT_SEVERITY_WARNING, 'WARN'),
      (EventSeverity.EVENT_SEVERITY_ERROR, 'ERROR'),
      (EventSeverity.EVENT_SEVERITY_CRITICAL, 'CRIT'),
    ];
    return Container(
      padding: const EdgeInsets.fromLTRB(12, 6, 12, 0),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SingleChildScrollView(
            scrollDirection: Axis.horizontal,
            child: Row(
              children: chips.map((c) {
                final selected = _filterSeverity == c.$1;
                final chipColor = c.$1 == null ? AppColors.primary : _getSeverityColor(c.$1!);
                return Padding(
                  padding: const EdgeInsets.only(right: 6),
                  child: FilterChip(
                    label: Text(c.$2, style: TextStyle(
                      fontSize: 11, fontWeight: FontWeight.w600,
                      color: selected ? Colors.white : chipColor,
                    )),
                    selected: selected,
                    onSelected: (_) => setState(() => _filterSeverity = c.$1),
                    selectedColor: chipColor.withValues(alpha: 0.85),
                    backgroundColor: chipColor.withValues(alpha: 0.1),
                    checkmarkColor: Colors.white,
                    side: BorderSide(color: chipColor.withValues(alpha: 0.3)),
                    padding: const EdgeInsets.symmetric(horizontal: 2),
                    materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
                    visualDensity: VisualDensity.compact,
                  ),
                );
              }).toList(),
            ),
          ),
          const SizedBox(height: 6),
          TextField(
            controller: _searchCtrl,
            onChanged: (v) => setState(() => _searchQuery = v),
            style: const TextStyle(fontSize: 13),
            decoration: InputDecoration(
              hintText: locale.tr('搜索消息/来源...', 'Search message/source...'),
              hintStyle: TextStyle(fontSize: 13, color: context.hintColor),
              prefixIcon: const Icon(Icons.search, size: 18),
              suffixIcon: _searchQuery.isNotEmpty
                  ? IconButton(
                      icon: const Icon(Icons.clear, size: 16),
                      onPressed: () => setState(() { _searchQuery = ''; _searchCtrl.clear(); }),
                    )
                  : null,
              isDense: true,
              contentPadding: const EdgeInsets.symmetric(vertical: 8),
              border: OutlineInputBorder(borderRadius: BorderRadius.circular(10), borderSide: BorderSide.none),
              filled: true,
              fillColor: context.isDark
                  ? Colors.white.withValues(alpha: 0.06)
                  : Colors.black.withValues(alpha: 0.05),
            ),
          ),
          const SizedBox(height: 4),
        ],
      ),
    );
  }

  Widget _buildEventsList(BuildContext context, LocaleProvider locale) {
    final filtered = _filteredEvents;
    if (_events.isEmpty && _initialLoading) {
      // Skeleton shimmer while waiting for first data
      return ListView(
        physics: const AlwaysScrollableScrollPhysics(),
        padding: const EdgeInsets.fromLTRB(16, 8, 16, 100),
        children: const [
          SkeletonListTile(),
          SkeletonListTile(),
          SkeletonListTile(),
          SkeletonListTile(),
          SkeletonListTile(),
        ],
      );
    }
    if (_events.isEmpty) {
      return ListView(
        physics: const AlwaysScrollableScrollPhysics(),
        children: [
          SizedBox(height: MediaQuery.of(context).size.height * 0.25),
          Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.history, size: 48, color: context.subtitleColor),
                const SizedBox(height: 16),
                Text(locale.tr('暂无事件记录', 'No events recorded'),
                  style: TextStyle(fontSize: 15, color: context.subtitleColor)),
                if (!_isStreaming)
                  Padding(
                    padding: const EdgeInsets.only(top: 12.0),
                    child: Row(mainAxisSize: MainAxisSize.min, children: [
                      SizedBox(width: 14, height: 14,
                        child: CircularProgressIndicator(strokeWidth: 1.5, color: context.subtitleColor)),
                      const SizedBox(width: 8),
                      Text(
                        _retryCount > 0
                            ? locale.tr('重连中 (第 $_retryCount 次)...', 'Reconnecting (attempt $_retryCount)...')
                            : locale.tr('连接中...', 'Connecting...'),
                        style: TextStyle(color: context.subtitleColor, fontSize: 13),
                      ),
                    ]),
                  ),
              ],
            ),
          ),
        ],
      );
    }
    if (filtered.isEmpty) {
      return ListView(
        physics: const AlwaysScrollableScrollPhysics(),
        children: [
          SizedBox(height: MediaQuery.of(context).size.height * 0.3),
          Center(child: Text(locale.tr('无匹配事件', 'No matching events'),
            style: TextStyle(fontSize: 14, color: context.subtitleColor))),
        ],
      );
    }
    return ListView.builder(
      padding: const EdgeInsets.fromLTRB(16, 8, 16, 100),
      itemCount: filtered.length,
      cacheExtent: 300,
      itemBuilder: (context, index) {
        return _EventListItem(
          event: filtered[index],
          color: _getSeverityColor(filtered[index].severity),
          severityLabel: _getSeverityLabel(filtered[index].severity),
          onAcknowledge: () async {
            HapticFeedback.lightImpact();
            final client = context.read<RobotConnectionProvider>().client;
            if (client == null) return;
            await client.ackEvent(filtered[index].eventId);
            if (mounted) {
              ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                content: Text(locale.tr('事件已确认', 'Event acknowledged')),
                duration: const Duration(milliseconds: 500),
                behavior: SnackBarBehavior.floating,
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
              ));
            }
          },
        );
      },
    );
  }

}

/// Clean card-style event item with left accent strip.
class _EventListItem extends StatelessWidget {
  final Event event;
  final Color color;
  final String severityLabel;
  final VoidCallback onAcknowledge;

  const _EventListItem({
    required this.event,
    required this.color,
    required this.severityLabel,
    required this.onAcknowledge,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final time = event.timestamp.toDateTime().toLocal();
    final timeStr = DateFormat('HH:mm:ss').format(time);

    return Padding(
      padding: const EdgeInsets.only(bottom: 8.0),
      child: Container(
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
          boxShadow: [context.isDark ? AppShadows.dark() : AppShadows.light()],
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(AppRadius.card),
          child: IntrinsicHeight(
            child: Row(
              children: [
                // Left accent strip
                Container(
                  width: 3,
                  decoration: BoxDecoration(
                    color: color,
                    borderRadius: const BorderRadius.only(
                      topLeft: Radius.circular(12),
                      bottomLeft: Radius.circular(12),
                    ),
                  ),
                ),
                // Content
                Expanded(
                  child: Padding(
                    padding: const EdgeInsets.fromLTRB(14, 12, 14, 12),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        // Top row: severity + time
                        Row(
                          children: [
                            Text(
                              severityLabel,
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.w700,
                                color: color,
                                letterSpacing: 0.5,
                              ),
                            ),
                            const Spacer(),
                            Text(
                              timeStr,
                              style: TextStyle(
                                fontSize: 11,
                                color: context.subtitleColor,
                                fontFeatures: const [
                                  FontFeature.tabularFigures()
                                ],
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 6),
                        // Title
                        Text(
                          event.title.isNotEmpty
                              ? event.title
                              : event.type.toString(),
                          style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w600,
                            color: context.titleColor,
                          ),
                          overflow: TextOverflow.ellipsis,
                          maxLines: 2,
                        ),
                        if (event.description.isNotEmpty) ...[
                          const SizedBox(height: 4),
                          Text(
                            event.description,
                            style: TextStyle(
                              fontSize: 12,
                              color: context.subtitleColor,
                              height: 1.4,
                            ),
                            maxLines: 2,
                            overflow: TextOverflow.ellipsis,
                          ),
                        ],
                      ],
                    ),
                  ),
                ),
                // ACK button — minimal
                InkWell(
                  onTap: onAcknowledge,
                  child: Padding(
                    padding: const EdgeInsets.symmetric(
                        horizontal: 14, vertical: 12),
                    child: Text(
                      'ACK',
                      style: TextStyle(
                        fontSize: 10,
                        fontWeight: FontWeight.w700,
                        color: context.subtitleColor,
                        letterSpacing: 0.5,
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
