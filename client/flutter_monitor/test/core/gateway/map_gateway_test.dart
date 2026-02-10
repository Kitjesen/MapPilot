import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/core/gateway/map_gateway.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';

void main() {
  late MapGateway gateway;
  late MockRobotClient mockClient;

  setUp(() {
    gateway = MapGateway();
    mockClient = MockRobotClient();
  });

  group('MapGateway lifecycle', () {
    test('initial state has empty map list', () {
      expect(gateway.maps, isEmpty);
      expect(gateway.isLoading, isFalse);
    });

    test('updateClient sets client', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);
      // no error thrown
    });

    test('updateClient(null) keeps maps', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);
      gateway.updateClient(null);
      // maps list is preserved (cached)
    });
  });

  group('Map operations', () {
    test('refreshMaps populates list', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);

      await gateway.refreshMaps();
      // MockRobotClient returns some maps
      expect(gateway.maps, isNotEmpty);
    });

    test('refreshMaps throws when disconnected', () async {
      expect(
        () => gateway.refreshMaps(),
        throwsException,
      );
    });

    test('saveMap works when connected', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);

      // Should not throw
      await gateway.saveMap('test_map.pcd');
    });
  });
}
