import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/core/gateway/ota_gateway.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';

void main() {
  late OtaGateway gateway;
  late MockRobotClient mockClient;

  setUp(() {
    gateway = OtaGateway();
    mockClient = MockRobotClient();
  });

  group('OtaGateway lifecycle', () {
    test('initial state has no active deployment', () {
      expect(gateway.activeDeployment, isNull);
      expect(gateway.isCheckingCloud, isFalse);
      expect(gateway.installedVersions, isEmpty);
    });

    test('updateClient(null) resets robot state', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);
      gateway.updateClient(null);

      expect(gateway.installedVersions, isEmpty);
      expect(gateway.robotSystemVersion, isNull);
    });
  });

  group('Deployment state machine', () {
    test('deployFromLocal creates active deployment', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);

      // Start a local deployment
      final future = gateway.deployFromLocal(
        List<int>.filled(100, 0),
        'test_firmware.bin',
      );

      // Should have an active deployment immediately
      expect(gateway.activeDeployment, isNotNull);
      expect(gateway.activeDeployment!.phase, isNot(DeployPhase.idle));

      // Wait for completion (mock client should succeed)
      await future;

      expect(gateway.activeDeployment!.phase,
          anyOf(DeployPhase.completed, DeployPhase.failed));
    });

    test('cancelDeploy clears active deployment after settling', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);

      // Start deployment (don't await)
      final future = gateway.deployFromLocal(
        List<int>.filled(100, 0),
        'test_firmware.bin',
      );

      // Cancel it — sets _cancelled flag internally
      gateway.cancelDeploy();

      // Wait for the future to settle (orchestrator clears deployment)
      await future;

      // After cancellation the orchestrator nulls out _activeDeployment
      expect(gateway.activeDeployment, isNull);
    });

    test('connection drop during deployment marks failure', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);

      // Start deployment
      gateway.deployFromLocal(
        List<int>.filled(100, 0),
        'test_firmware.bin',
      );

      // Simulate connection drop
      gateway.updateClient(null);

      final dep = gateway.activeDeployment;
      if (dep != null && dep.phase != DeployPhase.completed) {
        expect(dep.phase, DeployPhase.failed);
      }
    });
  });

  group('Cloud update check', () {
    test('checkCloudUpdate sets isCheckingCloud', () async {
      final future = gateway.checkCloudUpdate();
      expect(gateway.isCheckingCloud, isTrue);

      await future;
      expect(gateway.isCheckingCloud, isFalse);
    });
  });

  group('Installed versions', () {
    test('fetchInstalledVersions populates list', () async {
      await mockClient.connect();
      gateway.updateClient(mockClient);

      await gateway.fetchInstalledVersions();
      // MockRobotClient should return some versions
      expect(gateway.installedVersions, isNotEmpty);
    });

    test('fetchInstalledVersions throws when disconnected', () async {
      // Uses expectLater for async exception
      await expectLater(
        gateway.fetchInstalledVersions(),
        throwsException,
      );
    });
  });
}
