import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/core/gateway/control_gateway.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';

void main() {
  late ControlGateway gateway;
  late MockRobotClient mockClient;

  setUp(() {
    gateway = ControlGateway();
    mockClient = MockRobotClient();
  });

  group('ControlGateway lifecycle', () {
    test('initial state has no lease', () {
      expect(gateway.hasLease, isFalse);
    });

    test('updateClients sets client', () async {
      await mockClient.connect();
      gateway.updateClients(client: mockClient);
      // no error
    });

    test('updateClients with null disconnects', () {
      gateway.updateClients(client: null);
      expect(gateway.hasLease, isFalse);
    });
  });

  group('Lease management', () {
    test('toggleLease acquires lease when not held', () async {
      await mockClient.connect();
      gateway.updateClients(client: mockClient);

      await gateway.toggleLease();
      expect(gateway.hasLease, isTrue);
    });

    test('toggleLease releases lease when held', () async {
      await mockClient.connect();
      gateway.updateClients(client: mockClient);

      await gateway.toggleLease(); // acquire
      await gateway.toggleLease(); // release
      expect(gateway.hasLease, isFalse);
    });
  });

  group('Emergency stop', () {
    test('emergencyStop works when connected', () async {
      await mockClient.connect();
      gateway.updateClients(client: mockClient);

      // Should not throw
      await gateway.emergencyStop();
    });
  });
}
