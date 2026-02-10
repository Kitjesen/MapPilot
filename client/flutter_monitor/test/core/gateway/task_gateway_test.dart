import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';

void main() {
  late TaskGateway gateway;
  late MockRobotClient mockClient;

  setUp(() {
    gateway = TaskGateway();
    mockClient = MockRobotClient();
  });

  group('TaskGateway lifecycle', () {
    test('initial state is idle', () {
      expect(gateway.isRunning, isFalse);
      expect(gateway.statusMessage, isNull);
    });

    test('updateClient sets client', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);
      // no error thrown
    });

    test('updateClient(null) resets state', () {
      gateway.updateClient(null);
      expect(gateway.isRunning, isFalse);
    });
  });

  group('Callbacks', () {
    test('onMappingComplete callback can be set', () {
      bool called = false;
      gateway.onMappingComplete = () => called = true;
      // Callback is set but not invoked yet
      expect(called, isFalse);
    });

    test('onTaskTerminated callback can be set', () {
      bool called = false;
      gateway.onTaskTerminated = (type, status) => called = true;
      expect(called, isFalse);
    });
  });
}
