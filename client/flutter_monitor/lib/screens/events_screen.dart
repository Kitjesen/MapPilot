import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';
import '../services/robot_connection_provider.dart';
import 'package:robot_proto/robot_proto.dart';
import '../theme/app_theme.dart';

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

  static const int _maxEvents = 200;

  int _retryCount = 0;
  Timer? _retryTimer;

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _startListening());
  }

  @override
  void dispose() {
    _subscription?.cancel();
    _retryTimer?.cancel();
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
        return AppColors.textTertiary;
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
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Text('事件日志'),
            if (_events.isNotEmpty) ...[
              const SizedBox(width: 8),
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                decoration: BoxDecoration(
                  color: AppColors.primary.withOpacity(isDark ? 0.15 : 0.1),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Text(
                  '${_events.length}',
                  style: const TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w700,
                    color: AppColors.primary,
                  ),
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
          IconButton(
            icon: const Icon(Icons.refresh, color: AppColors.textSecondary),
            onPressed: () {
              HapticFeedback.lightImpact();
              _subscription?.cancel();
              setState(() {
                _events.clear();
                _retryCount = 0;
              });
              _startListening();
            },
          )
        ],
      ),
      body: _events.isEmpty
          ? Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(Icons.history,
                      size: 48, color: context.subtitleColor),
                  const SizedBox(height: 16),
                  Text(
                    '暂无事件记录',
                    style: TextStyle(
                      fontSize: 15,
                      color: context.subtitleColor,
                    ),
                  ),
                  if (!_isStreaming)
                    Padding(
                      padding: const EdgeInsets.only(top: 12.0),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          SizedBox(
                            width: 14,
                            height: 14,
                            child: CircularProgressIndicator(
                              strokeWidth: 1.5,
                              color: context.subtitleColor,
                            ),
                          ),
                          const SizedBox(width: 8),
                          Text(
                            _retryCount > 0
                                ? '重连中 (第 $_retryCount 次)...'
                                : '连接中...',
                            style: TextStyle(
                              color: context.subtitleColor,
                              fontSize: 13,
                            ),
                          ),
                        ],
                      ),
                    ),
                ],
              ),
            )
          : Stack(
              children: [
                Positioned(
                  left: 24,
                  top: 0,
                  bottom: 0,
                  child: Container(
                    width: 2,
                    color: context.dividerColor,
                  ),
                ),
                ListView.builder(
                  padding: EdgeInsets.fromLTRB(
                      16, MediaQuery.of(context).padding.top + 16, 16, 100),
                  itemCount: _events.length,
                  cacheExtent: 300,
                  itemBuilder: (context, index) {
                    return _EventListItem(
                      event: _events[index],
                      color: _getSeverityColor(_events[index].severity),
                      severityLabel: _getSeverityLabel(_events[index].severity),
                      onAcknowledge: () async {
                        HapticFeedback.lightImpact();
                        final client =
                            context.read<RobotConnectionProvider>().client;
                        if (client == null) return;
                        await client.ackEvent(_events[index].eventId);
                        if (mounted) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            SnackBar(
                              content: const Text('事件已确认'),
                              duration: const Duration(milliseconds: 500),
                              behavior: SnackBarBehavior.floating,
                              shape: RoundedRectangleBorder(
                                  borderRadius: BorderRadius.circular(10)),
                            ),
                          );
                        }
                      },
                    );
                  },
                ),
              ],
            ),
    );
  }
}

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
      padding: const EdgeInsets.only(bottom: 18.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Container(
            margin: const EdgeInsets.only(top: 8),
            width: 14,
            height: 14,
            decoration: BoxDecoration(
              color: color,
              shape: BoxShape.circle,
              border: Border.all(
                color: isDark ? AppColors.darkBackground : Colors.white,
                width: 2.5,
              ),
              boxShadow: [
                BoxShadow(
                  color: color.withOpacity(0.4),
                  blurRadius: 8,
                  offset: const Offset(0, 2),
                ),
              ],
            ),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Container(
              padding: const EdgeInsets.all(18),
              decoration: BoxDecoration(
                color: isDark ? AppColors.darkCard : Colors.white,
                borderRadius: BorderRadius.circular(20),
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
                        padding: const EdgeInsets.symmetric(
                            horizontal: 9, vertical: 3),
                        decoration: BoxDecoration(
                          color: color.withOpacity(isDark ? 0.15 : 0.1),
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: Text(
                          severityLabel,
                          style: TextStyle(
                            fontSize: 9,
                            fontWeight: FontWeight.w700,
                            color: color,
                            letterSpacing: 0.5,
                          ),
                        ),
                      ),
                      const Spacer(),
                      Text(
                        timeStr,
                        style: const TextStyle(
                          fontSize: 12,
                          color: context.subtitleColor,
                          fontFeatures: const [FontFeature.tabularFigures()],
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 10),
                  Text(
                    event.title.isNotEmpty
                        ? event.title
                        : event.type.toString(),
                    style: TextStyle(
                      fontSize: 15,
                      fontWeight: FontWeight.bold,
                      color: isDark ? Colors.white : Colors.black87,
                    ),
                    overflow: TextOverflow.ellipsis,
                    maxLines: 2,
                  ),
                  if (event.description.isNotEmpty) ...[
                    const SizedBox(height: 6),
                    Text(
                      event.description,
                      style: const TextStyle(
                        fontSize: 14,
                        color: context.subtitleColor,
                        height: 1.4,
                      ),
                      maxLines: 3,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ],
                  const SizedBox(height: 12),
                  Align(
                    alignment: Alignment.centerRight,
                    child: InkWell(
                      onTap: onAcknowledge,
                      borderRadius: BorderRadius.circular(20),
                      child: Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 14, vertical: 7),
                        decoration: BoxDecoration(
                          color: isDark
                              ? Colors.white.withOpacity(0.06)
                              : Colors.black.withOpacity(0.04),
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(Icons.check,
                                size: 14, color: context.subtitleColor),
                            const SizedBox(width: 4),
                            const Text(
                              'ACK',
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.w600,
                                color: context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}
