import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';
import '../services/robot_connection_provider.dart';
import '../generated/common.pb.dart';
import '../widgets/glass_widgets.dart';

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

  // 事件数量上限
  static const int _maxEvents = 200;

  // 重连退避
  int _retryCount = 0;
  Timer? _retryTimer;

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _startListening();
    });
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
            // 去重
            if (!_events.any((e) => e.eventId == event.eventId)) {
              _events.insert(0, event);
              // 限制最大数量，避免内存泄漏
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
    // 指数退避: 3s, 6s, 12s, max 30s
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
        return const Color(0xFF007AFF);
      case EventSeverity.EVENT_SEVERITY_WARNING:
        return const Color(0xFFFF9500);
      case EventSeverity.EVENT_SEVERITY_ERROR:
        return const Color(0xFFFF3B30);
      case EventSeverity.EVENT_SEVERITY_CRITICAL:
        return const Color(0xFFAF52DE);
      default:
        return Colors.black;
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
    super.build(context); // Required by AutomaticKeepAliveClientMixin

    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        title: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Text('Timeline'),
            if (_events.isNotEmpty) ...[
              const SizedBox(width: 8),
              Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                decoration: BoxDecoration(
                  color: const Color(0xFF007AFF).withOpacity(0.1),
                  borderRadius: BorderRadius.circular(10),
                ),
                child: Text(
                  '${_events.length}',
                  style: const TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w700,
                    color: Color(0xFF007AFF),
                  ),
                ),
              ),
            ],
          ],
        ),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
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
                      size: 48, color: Colors.grey.withOpacity(0.3)),
                  const SizedBox(height: 16),
                  Text('No events recorded',
                      style: TextStyle(color: Colors.grey.withOpacity(0.5))),
                  if (!_isStreaming)
                    Padding(
                      padding: const EdgeInsets.only(top: 8.0),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          SizedBox(
                            width: 12,
                            height: 12,
                            child: CircularProgressIndicator(
                              strokeWidth: 1.5,
                              color: Colors.grey.withOpacity(0.4),
                            ),
                          ),
                          const SizedBox(width: 8),
                          Text(
                            _retryCount > 0
                                ? 'Reconnecting (attempt $_retryCount)...'
                                : 'Connecting...',
                            style: const TextStyle(
                                color: Colors.grey, fontSize: 12),
                          ),
                        ],
                      ),
                    ),
                ],
              ),
            )
          : Stack(
              children: [
                // Vertical Timeline Line
                Positioned(
                  left: 24,
                  top: 0,
                  bottom: 0,
                  child: Container(
                    width: 2,
                    color: Colors.grey.withOpacity(0.2),
                  ),
                ),
                // Event List — 使用 ListView.builder 的虚拟滚动
                ListView.builder(
                  padding: EdgeInsets.fromLTRB(
                      16, MediaQuery.of(context).padding.top + 60, 16, 100),
                  itemCount: _events.length,
                  // 缓存区域限制，提升滚动性能
                  cacheExtent: 300,
                  itemBuilder: (context, index) {
                    return _EventListItem(
                      event: _events[index],
                      color: _getSeverityColor(_events[index].severity),
                      severityLabel:
                          _getSeverityLabel(_events[index].severity),
                      onAcknowledge: () async {
                        HapticFeedback.lightImpact();
                        final client =
                            context.read<RobotConnectionProvider>().client;
                        if (client == null) return;
                        await client.ackEvent(_events[index].eventId);
                        if (mounted) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            const SnackBar(
                              content: Text('Event acknowledged'),
                              duration: Duration(milliseconds: 500),
                              behavior: SnackBarBehavior.floating,
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

/// 独立的事件列表项 Widget，减少不必要的重建
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
    final time = event.timestamp.toDateTime().toLocal();
    final timeStr = DateFormat('HH:mm:ss').format(time);

    return Padding(
      padding: const EdgeInsets.only(bottom: 24.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Timeline Dot
          Container(
            margin: const EdgeInsets.only(top: 6),
            width: 14,
            height: 14,
            decoration: BoxDecoration(
              color: color,
              shape: BoxShape.circle,
              border: Border.all(color: Colors.white, width: 2.5),
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
          // Event Card
          Expanded(
            child: GlassCard(
              padding: const EdgeInsets.all(16),
              borderRadius: 16,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      // Severity badge
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 8, vertical: 2),
                        decoration: BoxDecoration(
                          color: color.withOpacity(0.12),
                          borderRadius: BorderRadius.circular(6),
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
                        style: TextStyle(
                          fontSize: 12,
                          color: Colors.black.withOpacity(0.4),
                          fontFeatures: const [FontFeature.tabularFigures()],
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Text(
                    event.title.isNotEmpty
                        ? event.title
                        : event.type.toString(),
                    style: const TextStyle(
                      fontSize: 15,
                      fontWeight: FontWeight.bold,
                      color: Colors.black87,
                    ),
                    overflow: TextOverflow.ellipsis,
                    maxLines: 2,
                  ),
                  if (event.description.isNotEmpty) ...[
                    const SizedBox(height: 6),
                    Text(
                      event.description,
                      style: TextStyle(
                        fontSize: 14,
                        color: Colors.black.withOpacity(0.6),
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
                            horizontal: 12, vertical: 6),
                        decoration: BoxDecoration(
                          color: Colors.black.withOpacity(0.05),
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(Icons.check,
                                size: 14,
                                color: Colors.black.withOpacity(0.6)),
                            const SizedBox(width: 4),
                            Text(
                              'ACK',
                              style: TextStyle(
                                fontSize: 10,
                                fontWeight: FontWeight.w600,
                                color: Colors.black.withOpacity(0.6),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  )
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}
