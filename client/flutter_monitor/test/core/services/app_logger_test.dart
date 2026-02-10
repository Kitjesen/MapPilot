import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';

void main() {
  setUp(() {
    AppLogger.clear();
  });

  group('AppLogger', () {
    test('logs are stored in buffer', () {
      AppLogger.ota.info('test message');
      final entries = AppLogger.recentEntries();
      expect(entries, hasLength(1));
      expect(entries.first.tag, 'OTA');
      expect(entries.first.message, 'test message');
      expect(entries.first.level, LogLevel.info);
    });

    test('different loggers use different tags', () {
      AppLogger.ota.info('ota msg');
      AppLogger.grpc.error('grpc err');
      AppLogger.control.debug('ctrl dbg');

      final entries = AppLogger.recentEntries();
      expect(entries, hasLength(3));
      expect(entries[0].tag, 'OTA');
      expect(entries[1].tag, 'gRPC');
      expect(entries[2].tag, 'CTRL');
    });

    test('ring buffer caps at maxBufferSize', () {
      for (var i = 0; i < AppLogger.maxBufferSize + 100; i++) {
        AppLogger.system.debug('msg $i');
      }
      final entries = AppLogger.recentEntries(AppLogger.maxBufferSize + 100);
      expect(entries.length, AppLogger.maxBufferSize);
    });

    test('exportLogs returns formatted string', () {
      AppLogger.ota.info('hello');
      final export = AppLogger.exportLogs();
      expect(export, contains('[OTA]'));
      expect(export, contains('hello'));
    });

    test('onError callback fires for error level', () {
      LogEntry? captured;
      AppLogger.onError = (entry) => captured = entry;

      AppLogger.ota.info('info msg');
      expect(captured, isNull);

      AppLogger.ota.error('error msg');
      expect(captured, isNotNull);
      expect(captured!.level, LogLevel.error);

      AppLogger.onError = null;
    });

    test('clear empties the buffer', () {
      AppLogger.ota.info('msg');
      expect(AppLogger.recentEntries(), isNotEmpty);
      AppLogger.clear();
      expect(AppLogger.recentEntries(), isEmpty);
    });
  });

  group('GrpcErrorFormatter', () {
    // GrpcErrorFormatter tests are separate since they need grpc import
    // which may not be available in all test environments
  });
}
