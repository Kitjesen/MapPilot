import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/core/gateway/file_gateway.dart';
import 'package:flutter_monitor/core/grpc/mock_robot_client.dart';

void main() {
  late FileGateway gateway;
  late MockRobotClient mockClient;

  setUp(() async {
    gateway = FileGateway();
    mockClient = MockRobotClient();
    await mockClient.connect();
  });

  group('FileGateway lifecycle', () {
    test('initial state is empty with default directory', () {
      expect(gateway.files, isEmpty);
      expect(gateway.currentDirectory, FileGateway.defaultDirectory);
      expect(gateway.totalSize, 0);
      expect(gateway.freeSpace, 0);
      expect(gateway.isLoading, isFalse);
      expect(gateway.errorMessage, isNull);
      expect(gateway.isUploading, isFalse);
      expect(gateway.isDownloading, isFalse);
    });

    test('updateClient(null) resets state to default directory', () {
      gateway.updateClient(mockClient);
      gateway.updateClient(null);

      expect(gateway.files, isEmpty);
      expect(gateway.currentDirectory, FileGateway.defaultDirectory);
      expect(gateway.totalSize, 0);
      expect(gateway.freeSpace, 0);
    });
  });

  group('FileGateway navigation', () {
    test('listFiles fetches mock file list', () async {
      gateway.updateClient(mockClient);
      await gateway.listFiles();

      expect(gateway.isLoading, isFalse);
      expect(gateway.errorMessage, isNull);
      // Mock client returns some files
      expect(gateway.files, isNotNull);
    });

    test('openDirectory changes currentDirectory and lists', () async {
      gateway.updateClient(mockClient);
      // Navigate to root first, then into a subdirectory
      await gateway.listFiles('/');
      await gateway.openDirectory('models');

      expect(gateway.currentDirectory, '/models');
    });

    test('goUp navigates to parent directory', () async {
      gateway.updateClient(mockClient);
      await gateway.listFiles('/');
      await gateway.openDirectory('models');
      expect(gateway.currentDirectory, '/models');

      await gateway.goUp();
      expect(gateway.currentDirectory, '/');
    });

    test('goUp from root stays at root', () async {
      gateway.updateClient(mockClient);
      await gateway.listFiles('/');
      expect(gateway.currentDirectory, '/');

      await gateway.goUp();
      expect(gateway.currentDirectory, '/');
    });

    test('breadcrumbs are computed correctly', () async {
      gateway.updateClient(mockClient);
      await gateway.listFiles('/');
      await gateway.openDirectory('models');
      await gateway.openDirectory('v2');

      final crumbs = gateway.breadcrumbs;
      expect(crumbs, isNotEmpty);
      expect(crumbs.first, '/');
    });

    test('navigateToBreadcrumb truncates path', () async {
      gateway.updateClient(mockClient);
      await gateway.listFiles('/');
      await gateway.openDirectory('a');
      await gateway.openDirectory('b');
      await gateway.openDirectory('c');

      expect(gateway.currentDirectory, '/a/b/c');

      await gateway.navigateToBreadcrumb(1); // navigate to 'a'
      expect(gateway.currentDirectory, '/a');
    });
  });

  group('FileGateway upload', () {
    test('uploadFile succeeds with mock client', () async {
      gateway.updateClient(mockClient);

      final result = await gateway.uploadFile(
        bytes: [0x01, 0x02, 0x03],
        filename: 'test.bin',
      );

      expect(result, isTrue);
      expect(gateway.isUploading, isFalse);
      expect(gateway.errorMessage, isNull);
    });
  });

  group('FileGateway delete', () {
    test('deleteFile succeeds with mock client', () async {
      gateway.updateClient(mockClient);

      final result = await gateway.deleteFile('/models/old_model.bin');
      expect(result, isTrue);
      expect(gateway.errorMessage, isNull);
    });
  });

  group('FileGateway download', () {
    test('downloadFile returns bytes with mock client', () async {
      gateway.updateClient(mockClient);

      final bytes = await gateway.downloadFile('/some/file.txt');
      expect(bytes, isNotNull);
      expect(bytes!.isNotEmpty, isTrue);
      expect(gateway.isDownloading, isFalse);
    });
  });

  group('FileGateway error handling', () {
    test('operations without client return gracefully', () async {
      // No client set
      await gateway.listFiles();
      expect(gateway.isLoading, isFalse);

      final uploadResult = await gateway.uploadFile(
        bytes: [0x01],
        filename: 'test.bin',
      );
      expect(uploadResult, isFalse);

      final deleteResult = await gateway.deleteFile('/foo');
      expect(deleteResult, isFalse);

      final downloadResult = await gateway.downloadFile('/foo');
      expect(downloadResult, isNull);
    });

    test('clearError resets error message', () {
      gateway.updateClient(mockClient);
      // Manually trigger an error state for testing
      gateway.clearError();
      expect(gateway.errorMessage, isNull);
    });
  });
}
